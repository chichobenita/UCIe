// tb_axi_bridge_ip_tx.sv
`timescale 1ns/1ps

import axi_bridge_ip_tx_tb_pkg::*;

module tb_axi_bridge_ip_tx;

  // אותם פרמטרים כמו ב-DUT/Package
  localparam int DATA_W   = DATA_W;
  localparam int IF_W     = IF_W;
  localparam int TUSER_W  = TUSER_W;

  // שעון וריסט
  logic clk;
  logic rst_n;

  initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 100MHz
  end

  // יצירת אינטרפייסים
  axi_stream_if #(DATA_W, TUSER_W) axi_if(.clk(clk), .rst_n(rst_n));
  client_if     #(IF_W,   TUSER_W) cl_if (.clk(clk), .rst_n(rst_n));

  // CSR + סטטיסטיקות מה-DUT
  logic                   bridge_enable;
  logic                   strict_tkeep_en;
  logic [7:0]             tx_fifo_afull_thr;
  logic                   drop_on_midreset;

  logic [31:0]            stat_tx_frames;
  logic [31:0]            stat_tx_bytes;
  logic [15:0]            stat_tx_fifo_level;
  logic [31:0]            stat_tx_stall_cycles;
  logic                   ev_err_tkeep_illegal;
  logic                   ev_err_midreset_drop;
  logic                   ev_err_overflow_tx;
  
  logic [IF_W-1:0]        last_data;
  logic [IF_W/8-1:0]      last_keep;
  logic [TUSER_W-1:0]     last_user;
  logic                   last_sop, last_eop;
  logic                   prev_valid, prev_ready;
  bit                     first_sample;
  
  logic bp_long_stall;  // 0 = pattern הרגיל (טסט 5), 1 = stall ארוך (טסט 6)
  int unsigned  max_fifo_level;
  logic         saw_tready_low_flag;
  logic         saw_tready_high_flag;
  
  int unsigned  midreset_err_cnt;
  int unsigned midreset_before;
  int unsigned frames_before;
  int unsigned bytes_before;

  // אינסטנציה של ה-DUT
  axi_bridge_ip_tx #(
    .DATA_W    (DATA_W),
    .IF_W      (IF_W),
    .TUSER_W   (TUSER_W),
    .FIFO_DEPTH(16)
  ) dut (
    .clk_i              (clk),
    .rst_ni             (rst_n),

    // AXI-Stream
    .s_axis_tdata       (axi_if.tdata),
    .s_axis_tkeep       (axi_if.tkeep),
    .s_axis_tuser       (axi_if.tuser),
    .s_axis_tvalid      (axi_if.tvalid),
    .s_axis_tlast       (axi_if.tlast),
    .s_axis_tready      (axi_if.tready),

    // Client-IF
    .cl_tx_data         (cl_if.data),
    .cl_tx_keep         (cl_if.keep),
    .cl_tx_user         (cl_if.user),
    .cl_tx_valid        (cl_if.valid),
    .cl_tx_sop          (cl_if.sop),
    .cl_tx_eop          (cl_if.eop),
    .cl_tx_ready        (cl_if.ready),

    // CSRs
    .bridge_enable      (bridge_enable),
    .strict_tkeep_en    (strict_tkeep_en),
    .tx_fifo_afull_thr  (tx_fifo_afull_thr),
    .drop_on_midreset   (drop_on_midreset),

    // Status
    .stat_tx_frames     (stat_tx_frames),
    .stat_tx_bytes      (stat_tx_bytes),
    .stat_tx_fifo_level (stat_tx_fifo_level),
    .stat_tx_stall_cycles(stat_tx_stall_cycles),
    .ev_err_tkeep_illegal(ev_err_tkeep_illegal),
    .ev_err_midreset_drop(ev_err_midreset_drop),
    .ev_err_overflow_tx (ev_err_overflow_tx)
  );

  // אובייקטים של Driver/Monitor/Scoreboard
  axi_driver      drv;
  client_monitor  mon;
  scoreboard      sb;

  
    int unsigned tkeep_err_cnt;

  // ספירת פעמי האירוע ev_err_tkeep_illegal
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tkeep_err_cnt <= 0;
    end else if (ev_err_tkeep_illegal) begin
      tkeep_err_cnt <= tkeep_err_cnt + 1;
    end
  end

  
  //==============counter for test 5===============
    // enable ל-generator של backpressure
  logic backpressure_en;

  // מונה רפרנס ל-valid && !ready
  int unsigned stall_ref_cnt;

  // גנרטור cl_tx_ready + מונה stall ref
  int unsigned bp_cycle;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cl_if.ready   <= 1'b1;
      bp_cycle      <= 0;
      stall_ref_cnt <= 0;
    end else begin
      // מונה stall cycles reference (כבר היה לך)
      if (bridge_enable && cl_if.valid && !cl_if.ready)
        stall_ref_cnt <= stall_ref_cnt + 1;

      if (!backpressure_en) begin
        cl_if.ready <= 1'b1;
        bp_cycle    <= 0;
      end else begin
        bp_cycle <= bp_cycle + 1;

        if (bp_long_stall) begin
          // מצב "סטול כבד" – למשך ~80 מחזורים cl_tx_ready=0
          if (bp_cycle < 80)
            cl_if.ready <= 1'b0;
          else
            cl_if.ready <= 1'b1;
        end else begin
          // המוד הישן (לטסט 5) – pattern+random
          if ((bp_cycle >= 10 && bp_cycle < 18) ||
              (bp_cycle >= 35 && bp_cycle < 45)) begin
            cl_if.ready <= 1'b0;
          end else begin
            cl_if.ready <= $urandom_range(0, 1);
          end
        end
      end
    end
  end

  
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    prev_valid <= 1'b0;
    last_data  <= '0;
    last_keep  <= '0;
    last_user  <= '0;
    last_sop   <= 1'b0;
    last_eop   <= 1'b0;
  end else begin
    // בודקים יציבות רק אם גם במחזור הקודם valid היה 1
    if ((cl_if.valid &&!cl_if.ready) && (prev_valid && !prev_ready)) begin
      if (cl_if.data !== last_data ||
          cl_if.keep !== last_keep ||
          cl_if.user !== last_user ||
          cl_if.sop  !== last_sop  ||
          cl_if.eop  !== last_eop) begin
        $error("Backpressure violation: cl_tx_* changed while valid && !ready");
      end
    end

    // בכל מחזור שבו valid=1 – נזכור את הערכים
    if (cl_if.valid) begin
      last_data <= cl_if.data;
      last_keep <= cl_if.keep;
      last_user <= cl_if.user;
      last_sop  <= cl_if.sop;
      last_eop  <= cl_if.eop;
    end

    prev_valid <= cl_if.valid;
  end
end
  
//============================================
  
  
//======================================
//========= test 6 aids blocks =========
//======================================
  
    // מעקב אחרי fifo_level ו-tready בכל הסימולציה
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      max_fifo_level       <= 0;
      saw_tready_low_flag  <= 1'b0;
      saw_tready_high_flag <= 1'b0;
    end else begin
      // שומרים את המקסימום שראינו
      if (stat_tx_fifo_level > max_fifo_level)
        max_fifo_level <= stat_tx_fifo_level;

      // סימון: ראינו מצב שבו FIFO>=thr ו-tready=0
      if (!axi_if.tready && (stat_tx_fifo_level >= tx_fifo_afull_thr))
        saw_tready_low_flag <= 1'b1;

      // אחרי שראינו low, סימון: ראינו גם חזרה ל-high
      if (saw_tready_low_flag &&
          axi_if.tready &&
          (stat_tx_fifo_level < tx_fifo_afull_thr))
        saw_tready_high_flag <= 1'b1;
    end
  end
//======================================
  
  
// ======== counter for test 7==========
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      midreset_err_cnt <= 0;
    else if (ev_err_midreset_drop)
      midreset_err_cnt <= midreset_err_cnt + 1;
  end
//======================================
  
  
  
  // enable tests
  localparam bit ENABLE_TEST1 = 0;
  localparam bit ENABLE_TEST2 = 0;
  localparam bit ENABLE_TEST3 = 0;
  localparam bit ENABLE_TEST4 = 0;
  localparam bit ENABLE_TEST5 = 0; 
  localparam bit ENABLE_TEST6 = 0;
  localparam bit ENABLE_TEST7a = 0;
  localparam bit ENABLE_TEST7b = 0;
  localparam bit ENABLE_TEST7cd = 1;

  
  
initial begin : main_test
  // === DECLARATIONS קודם ===
  axi_frame  frame, frame2, frame3, frame4_legal, frame_small, frame5, frame6;
  axi_frame   frame7a, frame7b, frame7c, frame7d, frame7e;
  axi_frame   frame4_illegal_nl;   // non-last beat illegal
  axi_frame   frame4_illegal_last; // last-beat illegal
  axi_beat_t beat, beat2, beat4, beat_full, beat_last, beat_small, beat5, beat6;
  axi_beat_t  beat7;
  int unsigned N_BEATS;
  int unsigned exp_frames, exp_bytes;
  int unsigned num_bytes;
  int unsigned err_before, err_after;
  int unsigned N_BEATS5;
  int unsigned N_BEATS6;
  int unsigned N_BEATS7;
  
  // שומרים ערך קודם של מונה stall ושל stat_tx_stall_cycles
  int unsigned stall_ref_before;
  int unsigned stall_stat_before;

  // חישוב דלתא של stall לפי TB ושל ה-DUT
  int unsigned stall_ref_after;
  int unsigned stall_stat_after;

  int unsigned ref_delta;
  int unsigned dut_delta;

  // אתחול
  rst_n            = 0;
  bridge_enable    = 0;
  strict_tkeep_en  = 0;
  drop_on_midreset = 0;
  tx_fifo_afull_thr= 8'd16; // מעל עומק ה-FIFO, שלא יהיה almost-full
  cl_if.ready      = 1'b1;  // בלי backpressure

  // reset
  repeat (4) @(posedge clk);
  rst_n = 1;

  // יצירת virtual interfaces ואובייקטי ה-SV
  drv = new(axi_if);
  mon = new(cl_if);
  sb  = new(mon);

  // הפעלת המוניטור ברקע
  fork
    mon.run();
  join_none

  // מחכים קצת אחרי reset
  repeat (2) @(posedge clk);
  bridge_enable = 1'b1;

  // -------------------------
  // בניית frame: beat אחד מלא
  // -------------------------
  if (ENABLE_TEST1) begin
  frame = new();  // ה-new אחרי ההכרזה, לא בתוך ה-declaration

  beat.data = 256'h1122334455667788_99AABBCCDDEEFF00_F0E0D0C0B0A09080_7766554433221100;
  beat.keep = {BYTES_PER_BEAT{1'b1}}; // כל ה-32 בייטים חוקיים
  beat.last = 1'b1;
  beat.user = 16'h00AA;

  frame.add_beat(beat);

  // שליחת ה-frame
  $display("[TEST] Sending single-beat frame with full tkeep");
  drv.drive_frame(frame);

  // ביקורת באמצעות ה-scoreboard
  sb.check_frame(frame);

  // עוד קצת זמן לסיום
  repeat (10) @(posedge clk);
  end
  
  
    // -----------------------------
    // TEST 2: multi-beat frame, full tkeep, TLAST on last beat
    // -----------------------------
  if (ENABLE_TEST2) begin  
  N_BEATS = 3;
    frame2  = new();

    for (int i = 0; i < N_BEATS; i++) begin
      // סתם תבנית קלה לזיהוי – אתה יכול לשנות
      beat2.data = {
        64'h1000_0000_0000_0000 + i,
        64'h2000_0000_0000_0000 + i,
        64'h3000_0000_0000_0000 + i,
        64'h4000_0000_0000_0000 + i
      };
      beat2.keep = {BYTES_PER_BEAT{1'b1}}; // כל הבייטים תקפים
      beat2.last = (i == N_BEATS-1);       // TLAST רק על ה-beat האחרון
      beat2.user = 16'h00BB;               // user שונה מטסט1

      frame2.add_beat(beat2);
    end

    $display("[TEST] Sending multi-beat frame with %0d beats (full tkeep)", N_BEATS);
    drv.drive_frame(frame2);
    sb.check_frame(frame2);

    // שוב קצת זמן שדברים יתיישבו
    repeat (10) @(posedge clk);

    // -----------------------------
    // בדיקות סטטיסטיות (מצטברות לשני ה-frames)
    // -----------------------------
    // Test 1: 1 beat מלא → BYTES_PER_BEAT bytes
    // Test 2: N_BEATS beats מלאים → N_BEATS * BYTES_PER_BEAT bytes
    exp_frames = 2;                           // שני frames בסך הכל
    exp_bytes  = BYTES_PER_BEAT * (1 + N_BEATS);  // סה"כ bytes שנשלחו

    if (stat_tx_frames !== exp_frames) begin
      $error("stat_tx_frames expected %0d, got %0d",
             exp_frames, stat_tx_frames);
    end

    if (stat_tx_bytes !== exp_bytes) begin
      $error("stat_tx_bytes expected %0d, got %0d",
             exp_bytes, stat_tx_bytes);
    end
  end
  
      // -----------------------------
    // TEST 3a: multi-beat frame, last beat with partial tkeep
    // -----------------------------
  if (ENABLE_TEST3) begin
    for (num_bytes = 1; num_bytes < BYTES_PER_SEG; num_bytes++) begin
      frame3 = new();

      // beat ראשון מלא (כל הבייטים תקפים)
      beat_full.data = '0;
      for (int b = 0; b < BYTES_PER_BEAT; b++) begin
        beat_full.data[b*8 +: 8] = 8'h10 + b; // תבנית נוחה לזיהוי
      end
      beat_full.keep = {BYTES_PER_BEAT{1'b1}};
      beat_full.last = 1'b0;          // לא TLAST
      beat_full.user = 16'h00CC;
      frame3.add_beat(beat_full);

      // beat אחרון: רק num_bytes בייטים תקפים
      beat_last.data = '0;
      for (int b = 0; b < num_bytes; b++) begin
        beat_last.data[b*8 +: 8] = 8'h80 + b; // תבנית שונה מה-beat הראשון
      end

      beat_last.keep = '0;
      for (int b = 0; b < num_bytes; b++) begin
        beat_last.keep[b] = 1'b1;     // tkeep[0..num_bytes-1] = 1, השאר 0
      end

      beat_last.last = 1'b1;          // TLAST על ה-beat האחרון
      beat_last.user = 16'h00CC;

      frame3.add_beat(beat_last);

      $display("[TEST 3a] Frame with full beat + last beat with %0d valid bytes", num_bytes);

      drv.drive_frame(frame3);
      sb.check_frame(frame3);

      // זמן לסיום כל הסגמנטים לפני הטסט הבא
      repeat (10) @(posedge clk);

      // לוודא שאין סגמנטים מיותרים מעבר למה שציפינו
      sb.check_no_extra_segments($sformatf("TEST 3a (num_bytes=%0d)", num_bytes));
    end

  
      // -----------------------------
    // TEST 3b: single-beat frame smaller than one segment
    // -----------------------------

    for (num_bytes = 1; num_bytes < BYTES_PER_SEG; num_bytes++) begin
      frame_small = new();

      beat_small.data = '0;
      for (int b = 0; b < num_bytes; b++) begin
        beat_small.data[b*8 +: 8] = 8'hA0 + b;
      end

      beat_small.keep = '0;
      for (int b = 0; b < num_bytes; b++) begin
        beat_small.keep[b] = 1'b1;
      end

      beat_small.last = 1'b1;         // TLAST על beat בודד
      beat_small.user = 16'h00DD;

      frame_small.add_beat(beat_small);

      $display("[TEST 3b] Tiny single-beat frame with %0d bytes (< BYTES_PER_SEG)", num_bytes);

      drv.drive_frame(frame_small);
      sb.check_frame(frame_small);

      repeat (10) @(posedge clk);
      sb.check_no_extra_segments($sformatf("TEST 3b (num_bytes=%0d)", num_bytes));
    end
  end
  
  
  
      // -----------------------------
    // TEST 4: strict_tkeep_en checks
    // -----------------------------
  if (ENABLE_TEST4) begin  
  $display("=== TEST 4: strict_tkeep_en ===");

    // נוודא שאין שאריות סגמנטים קודמים בתור המוניטור
    mon.seg_q.delete();

    // מפעילים strict mode
    strict_tkeep_en = 1'b1;

      // 4a) Frame חוקי (full tkeep, TLAST תקין) -> לא אמור להיות ev_err_tkeep_illegal
    frame4_legal = new();

    beat4.data = 256'hCAFEBABE_DEADBEEF_0011223344556677_8899AABBCCDDEEFF;
    beat4.keep = {BYTES_PER_BEAT{1'b1}};   // beat מלא
    beat4.last = 1'b1;                     // TLAST
    beat4.user = 16'h0E4A;                 // user שונה לזיהוי

    frame4_legal.add_beat(beat4);

    err_before = tkeep_err_cnt;
    $display("[TEST 4a] Legal frame with strict_tkeep_en=1 (full keep, TLAST only on last beat)");

    drv.drive_frame(frame4_legal);
    sb.check_frame(frame4_legal); // פה אנחנו רוצים שה-data יהיה 100%

    repeat (10) @(posedge clk);
    err_after = tkeep_err_cnt;

    if (err_after != err_before) begin
      $error("TEST 4a: ev_err_tkeep_illegal fired on a legal frame (before=%0d, after=%0d)",
             err_before, err_after);
    end

    // ליתר ביטחון – אין סגמנטים מיותרים
    sb.check_no_extra_segments("TEST 4a");

      // 4b) Non-last beat עם tkeep חלקי -> מצפים ל-ev_err_tkeep_illegal, אבל data ממשיך
    frame4_illegal_nl = new();
    mon.seg_q.delete();

    // beat 0: לא אחרון, keep חלקי (לא all-ones) -> ILLEGAL ב-strict
    beat4.data = '0;
    for (int b = 0; b < BYTES_PER_BEAT; b++) begin
      beat4.data[b*8 +: 8] = 8'h20 + b;
    end
    beat4.keep = '0;
    beat4.keep[0] = 1'b1;
    beat4.keep[1] = 1'b1;   // שני בייטים בלבד, last=0 -> אסור ב-strict
    beat4.last = 1'b0;
    beat4.user = 16'h0BAD;
    frame4_illegal_nl.add_beat(beat4);

    // beat 1: אחרון, full keep, חוקי
    beat4.data = '0;
    for (int b = 0; b < BYTES_PER_BEAT; b++) begin
      beat4.data[b*8 +: 8] = 8'h40 + b;
    end
    beat4.keep = {BYTES_PER_BEAT{1'b1}};
    beat4.last = 1'b1;
    beat4.user = 16'h0BAD;
    frame4_illegal_nl.add_beat(beat4);

    err_before = tkeep_err_cnt;
    $display("[TEST 4b] Non-last beat with partial tkeep (strict_tkeep_en=1) -> expect error event");

    drv.drive_frame(frame4_illegal_nl);

    // נותנים זמן שהברידג' יסיים לשדר את כל הסגמנטים
    repeat (40) @(posedge clk);
    err_after = tkeep_err_cnt;

    if (err_after <= err_before) begin
      $error("TEST 4b: Expected ev_err_tkeep_illegal for non-last beat with partial keep, but counter did not increase (before=%0d, after=%0d)",
             err_before, err_after);
    end

    // לוודא שה-data *באמת* זרם – לפחות סגמנט אחד יצא
    if (mon.seg_q.size() == 0) begin
      $error("TEST 4b: Illegal tkeep on non-last beat blocked data – expected some segments on cl_tx");
    end

    // מנקים את התור לקראת הטסט הבא
    mon.seg_q.delete();

      // 4c) Beat אחרון עם tkeep לא מונוטוני -> שוב מצפים ל-ev_err_tkeep_illegal
    frame4_illegal_last = new();
    mon.seg_q.delete();

    beat4.data = '0;
    for (int b = 0; b < BYTES_PER_BEAT; b++) begin
      beat4.data[b*8 +: 8] = 8'h80 + b;
    end

    // tkeep לא מונוטוני: 1,1,1,0,1,... (חור באמצע)
    beat4.keep = '0;
    beat4.keep[0] = 1'b1;
    beat4.keep[1] = 1'b1;
    beat4.keep[2] = 1'b1;
    beat4.keep[4] = 1'b1;   // חור ב-index 3 => pattern לא מונוטוני

    beat4.last = 1'b1;      // TLAST
    beat4.user = 16'h0E11;
    frame4_illegal_last.add_beat(beat4);

    err_before = tkeep_err_cnt;
    $display("[TEST 4c] Last beat with non-monotonic tkeep (strict_tkeep_en=1) -> expect error event");

    drv.drive_frame(frame4_illegal_last);

    repeat (40) @(posedge clk);
    err_after = tkeep_err_cnt;

    if (err_after <= err_before) begin
      $error("TEST 4c: Expected ev_err_tkeep_illegal for last beat with non-monotonic keep, but counter did not increase (before=%0d, after=%0d)",
             err_before, err_after);
    end

    if (mon.seg_q.size() == 0) begin
      $error("TEST 4c: Illegal non-monotonic tkeep on last beat blocked data – expected some segments");
    end

    // אפשר להחזיר strict_tkeep_en ל-0 אם תרצה
    strict_tkeep_en = 1'b0;
  end
  
  
    //==================
  //===== test 5 =====
  //==================
  
    if (ENABLE_TEST5) begin
      $display("=== TEST 5: Client backpressure ===");

      // טוענים frame גדול מספיק כדי לקבל stall באמצע
      frame5 = new();
      
      N_BEATS5 = 4;     // 4 beats מלאים
      for (int i = 0; i < N_BEATS5; i++) begin
        beat5.data = {
          64'hA000_0000_0000_0000 + i,
          64'hB000_0000_0000_0000 + i,
          64'hC000_0000_0000_0000 + i,
          64'hD000_0000_0000_0000 + i
        };
        beat5.keep = {BYTES_PER_BEAT{1'b1}};
        beat5.last = (i == N_BEATS5-1);
        beat5.user = 16'h0B5A;   // סתם tag ייחודי
        frame5.add_beat(beat5);
      end

      // מנקים תור מוניטור
      mon.seg_q.delete();

      // מפעילים backpressure
      backpressure_en = 1'b1;

      // שומרים ערך קודם של מונה stall ושל stat_tx_stall_cycles
      stall_ref_before   = stall_ref_cnt;
      stall_stat_before  = stat_tx_stall_cycles;

      $display("[TEST 5] Driving multi-beat frame under random backpressure");
      drv.drive_frame(frame5);

      // scoreboard: מוודא שאין כפל / דילוג / סדר שגוי בסגמנטים
      sb.check_frame(frame5);

      // נותנים עוד קצת זמן לסגמנטים האחרונים
      repeat (50) @(posedge clk);

      // מכבים backpressure כדי לא להשפיע על טסטים אחרים (אם יופעלו בעתיד)
      backpressure_en = 1'b0;

      // חישוב דלתא של stall לפי TB ושל ה-DUT
      stall_ref_after  = stall_ref_cnt;
      stall_stat_after = stat_tx_stall_cycles;

      ref_delta  = stall_ref_after  - stall_ref_before;
      dut_delta  = stall_stat_after - stall_stat_before;

      $display("[TEST 5] stall_ref_delta=%0d, dut_stall_delta=%0d",
               ref_delta, dut_delta);

      if (dut_delta !== ref_delta) begin
        $error("stat_tx_stall_cycles mismatch: expected %0d, got %0d",
               ref_delta, dut_delta);
      end

      // ליתר ביטחון – אין סגמנטים מיותרים
      sb.check_no_extra_segments("TEST 5");

      $display("=== TEST 5 DONE ===");
    end
     
  
  //==================
  //===== test 6 =====
  //==================
  
      if (ENABLE_TEST6) begin
      $display("=== TEST 6: IP-side backpressure / FIFO almost-full ===");

      // קונפיגורציה ל-test הזה
      strict_tkeep_en   = 1'b0;
      drop_on_midreset  = 1'b0;

      // סף almost-full קטן כדי שנפגע בו מהר
      tx_fifo_afull_thr = 8'd4;

      // backpressure כבד בצד ה-Client
      backpressure_en   = 1'b1;
      bp_long_stall     = 1'b1;

      // איפוס מוניטור
      mon.seg_q.delete();

      // בונים frame גדול – הרבה beats מלאים
      frame6   = new();
      N_BEATS6 = 32;    // לדוגמה: 32 beats

      for (int i = 0; i < N_BEATS6; i++) begin
        beat6.data = {
          64'h9000_0000_0000_0000 + i,
          64'h8000_0000_0000_0000 + i,
          64'h7000_0000_0000_0000 + i,
          64'h6000_0000_0000_0000 + i
        };
        beat6.keep = {BYTES_PER_BEAT{1'b1}};   // full keep
        beat6.last = (i == N_BEATS6-1);
        beat6.user = 16'h06F6;
        frame6.add_beat(beat6);
      end

      $display("[TEST 6] Driving big frame under heavy client backpressure...");

      // שליחה + בדיקה פונקציונלית רגילה (אין דילוגים/כפילות)
      drv.drive_frame(frame6);
      sb.check_frame(frame6);
      sb.check_no_extra_segments("TEST 6");

      // נותנים לקאונטרים עוד קצת זמן להתעדכן
      repeat (20) @(posedge clk);

      // ==== בדיקות ספציפיות ל-test ====

      // 1) האם אי פעם tready ירד כש-fifo_level >= thr?
      if (!saw_tready_low_flag) begin
        $error("TEST 6: s_axis_tready never deasserted when FIFO level >= tx_fifo_afull_thr");
      end

      // 2) האם אחרי זה tready חזר ל-1 כשה-FIFO ירד מתחת לסף?
      if (!saw_tready_high_flag) begin
        $error("TEST 6: s_axis_tready never reasserted after FIFO drained below threshold");
      end

      // 3) לא אמור להיות overflow בכלל
      if (ev_err_overflow_tx) begin
        $error("TEST 6: ev_err_overflow_tx asserted unexpectedly (no overflow expected)");
      end

      $display("[TEST 6] max FIFO level observed = %0d (thr=%0d)",
               max_fifo_level, tx_fifo_afull_thr);

      $display("=== TEST 6 DONE ===");
    end

  //==================
  //===== test 7 =====
  //==================
  if (ENABLE_TEST7a) begin
    $display("=== TEST 7ab: bridge_enable & drop_on_midreset ===");

      //---------------------------------------------
      // 7a) bridge_enable=0 -> אין תנועה
      //---------------------------------------------
      strict_tkeep_en   = 1'b0;
      drop_on_midreset  = 1'b0;
      tx_fifo_afull_thr = 8'd16;   // גבוה, שלא יפריע
      backpressure_en   = 1'b0;    // cl_if.ready תמיד 1

      bridge_enable     = 1'b0;

      mon.seg_q.delete();
      frames_before = stat_tx_frames;
      bytes_before  = stat_tx_bytes;

      $display("[TEST 7a] Driving manual TVALID pulses while bridge_enable=0 - expect NO output");

      // דחיפה ידנית של beat אחד, בלי לחכות ל-tready
      axi_if.tdata  <= {
        64'hAAAA_AAAA_AAAA_AAAA,
        64'hBBBB_BBBB_BBBB_BBBB,
        64'hCCCC_CCCC_CCCC_CCCC,
        64'hDDDD_DDDD_DDDD_DDDD
      };
      axi_if.tkeep  <= {BYTES_PER_BEAT{1'b1}};
      axi_if.tlast  <= 1'b1;
      axi_if.tuser  <= 16'h7A0A;
      axi_if.tvalid <= 1'b1;

      // "IP" מנסה לשדר כמה מחזורים, אבל ה-bridge כבוי ולכן tready=0
      repeat (5) @(posedge clk);

      // מפסיקים לשדר
      axi_if.tvalid <= 1'b0;
      axi_if.tlast  <= 1'b0;
      axi_if.tdata  <= '0;
      axi_if.tkeep  <= '0;
      axi_if.tuser  <= '0;

      // נותנים עוד כמה מחזורים כדי לוודא שאין שום תגובה
      repeat (10) @(posedge clk);


      if (mon.seg_q.size() != 0)
        $error("TEST 7a: got segments on cl_tx_* while bridge_enable=0");

      if (stat_tx_frames != frames_before)
        $error("TEST 7a: stat_tx_frames changed while bridge_enable=0");

      if (stat_tx_bytes != bytes_before)
        $error("TEST 7a: stat_tx_bytes changed while bridge_enable=0");

      if (axi_if.tready !== 1'b0)
        $error("TEST 7a: s_axis_tready not 0 while bridge_enable=0");
  end

      //---------------------------------------------
      // 7b) bridge_enable עולה ל-1 -> frame זורם רגיל
      //---------------------------------------------
      if (ENABLE_TEST7b) begin
  	  bridge_enable = 1'b1;
      repeat (5) @(posedge clk);

      mon.seg_q.delete();

      frame7b = new();
      beat7.data = {
        64'h1111_1111_1111_1111,
        64'h2222_2222_2222_2222,
        64'h3333_3333_3333_3333,
        64'h4444_4444_4444_4444
      };
      beat7.keep = {BYTES_PER_BEAT{1'b1}};
      beat7.last = 1'b1;
      beat7.user = 16'h7B0B;
      frame7b.add_beat(beat7);

      $display("[TEST 7b] bridge_enable=1, regular frame should pass");
      drv.drive_frame(frame7b);
      sb.check_frame(frame7b);
      sb.check_no_extra_segments("TEST 7b");
    
  end

      //---------------------------------------------
      // 7c) drop_on_midreset=1, כיבוי באמצע frame פעיל
      //---------------------------------------------
  	  if (ENABLE_TEST7cd) begin
      $display("=== TEST 7cd: bridge_enable & drop_on_midreset ===");
  	  drop_on_midreset = 1'b1;    // מפעילים drop-on-midreset
      backpressure_en  = 1'b0;    // cl_if.ready תמיד 1, אין stalls

      frame7c  = new();
      N_BEATS7 = 8;               // frame ארוך מספיק

      for (int i = 0; i < N_BEATS7; i++) begin
        beat7.data = {
          64'hA000_0000_0000_0000 + i,
          64'hB000_0000_0000_0000 + i,
          64'hC000_0000_0000_0000 + i,
          64'hD000_0000_0000_0000 + i
        };
        beat7.keep = {BYTES_PER_BEAT{1'b1}};
        beat7.last = (i == N_BEATS7-1);
        beat7.user = 16'h7C0C;
        frame7c.add_beat(beat7);
      end

      mon.seg_q.delete();
      midreset_before = midreset_err_cnt;

      $display("[TEST 7c] drop_on_midreset=1, dropping bridge_enable mid-frame");

      fork
        // ענף 1 – שידור ה-frame
        begin
          drv.drive_frame(frame7c);
        end
        // ענף 2 – מחכים לתחילת frame ואז מורידים bridge_enable באמצע
        begin
          // מחכים ל-handshake של סגמנט ראשון (SOP) בצד ה-Client
          @(posedge clk iff (cl_if.valid && cl_if.ready && cl_if.sop));
          // נותנים לעוד כמה סגמנטים לצאת
          repeat (3) @(posedge clk);
          bridge_enable = 1'b0; // כיבוי באמצע frame
          repeat (5) @(posedge clk);
          bridge_enable = 1'b1; // הדלקה מחדש
        end
      join

      // זמן לתת ל-flush לקרות
      repeat (30) @(posedge clk);

      if (midreset_err_cnt != midreset_before + 1)
        $error("TEST 7c: expected 1 ev_err_midreset_drop event, got %0d (before=%0d)",
               midreset_err_cnt, midreset_before);

      // אחרי ה-midreset+re-enable – שולחים frame חדש ונבדוק שהוא נקי
      frame7d = new();
      beat7.data = {
        64'h5555_5555_5555_5555,
        64'h6666_6666_6666_6666,
        64'h7777_7777_7777_7777,
        64'h8888_8888_8888_8888
      };
      beat7.keep = {BYTES_PER_BEAT{1'b1}};
      beat7.last = 1'b1;
      beat7.user = 16'h7D0D;
      frame7d.add_beat(beat7);

      mon.seg_q.delete();
      $display("[TEST 7c] Sending clean frame after midreset");
      drv.drive_frame(frame7d);
      sb.check_frame(frame7d);
      sb.check_no_extra_segments("TEST 7c after midreset");

      //---------------------------------------------
      // 7d) drop_on_midreset=1, FIFO מלא אבל עוד לא התחיל frame החוצה
      //---------------------------------------------
      $display("[TEST 7d] drop_on_midreset=1, FIFO has data but no frame started");

      // נכפה מצב שבו Client לא מוכן -> beats נערמים ב-FIFO
      backpressure_en  = 1'b1;
      bp_long_stall    = 1'b1;  // cl_if.ready=0 לתקופה ארוכה
      bridge_enable    = 1'b1;  // פעיל
      drop_on_midreset = 1'b1;

      frame7e  = new();
      N_BEATS7 = 8;

      for (int j = 0; j < N_BEATS7; j++) begin
        beat7.data = {
          64'h9000_0000_0000_0000 + j,
          64'hA000_0000_0000_0000 + j,
          64'hB000_0000_0000_0000 + j,
          64'hC000_0000_0000_0000 + j
        };
        beat7.keep = {BYTES_PER_BEAT{1'b1}};
        beat7.last = (j == N_BEATS7-1);
        beat7.user = 16'h7E0E;
        frame7e.add_beat(beat7);
      end

      mon.seg_q.delete();
      midreset_before = midreset_err_cnt;

      fork
        // ענף 1 – ה-IP מנסה לשלוח frame בעוד ready=0, FIFO מתמלא
        begin
          drv.drive_frame(frame7e);
        end
        // ענף 2 – אחרי כמה מחזורים, מורידים bridge_enable בלי שיצא אפילו seg אחד
        begin
          repeat (20) @(posedge clk);
          bridge_enable = 1'b0;
          repeat (5) @(posedge clk);
          bridge_enable = 1'b1;
        end
      join

      repeat (30) @(posedge clk);

      if (midreset_err_cnt != midreset_before + 1)
        $error("TEST 7d: expected another ev_err_midreset_drop event, got %0d (before=%0d)",
               midreset_err_cnt, midreset_before);

      // לא אמורים להיות סגמנטים מה-frame הזה (ה-client היה תמיד not-ready)
      if (mon.seg_q.size() != 0)
        $error("TEST 7d: got unexpected segments although cl_tx_ready was low and midreset dropped FIFO contents");

      // שוב – אחרי הכל, שולחים frame נקי לוודא שאין mixing
      backpressure_en = 1'b0;  // client מוכן
      frame7d         = new();
      beat7.data = {
        64'hDEAD_BEEF_DEAD_BEEF,
        64'hFEED_FACE_FEED_FACE,
        64'h1234_5678_9ABC_DEF0,
        64'h0BAD_CAFE_0BAD_CAFE
      };
      beat7.keep = {BYTES_PER_BEAT{1'b1}};
      beat7.last = 1'b1;
      beat7.user = 16'h7F0F;
      frame7d.add_beat(beat7);

      mon.seg_q.delete();
      $display("[TEST 7d] Final clean frame after FIFO midreset");
      drv.drive_frame(frame7d);
      sb.check_frame(frame7d);
      sb.check_no_extra_segments("TEST 7d final frame");

      $display("=== TEST 7 DONE ===");
    end

  
  $display("=== ALL TESTS (1,2,3,4,5,6,7) DONE ===");
    $finish;

  end


  
  initial begin
    $dumpfile("waves.vcd");              // שם הקובץ (לא משנה כל כך ב-EDA PG)
    $dumpvars(0, tb_axi_bridge_ip_tx);   // כל ההיררכיה מתחת לטופ
  end
  
endmodule



