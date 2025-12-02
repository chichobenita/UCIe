// Ingress path: AXI-Stream from IP -> Client-IF segments
module axi_bridge_ip_tx #(
  parameter int DATA_W   = 256,  // רוחב AXI-Stream בצד ה-IP
  parameter int IF_W     = 64,   // רוחב הסגמנט ל-Client-IF
  parameter int TUSER_W  = 16,   // רוחב מטא-דאטה (VC, DST, וכו')
  parameter int FIFO_DEPTH = 16  // עומק ה-Ingress FIFO ב-beats
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  // ================= AXI-Stream from IP =================
  input  logic [DATA_W-1:0]      s_axis_tdata,
  input  logic [DATA_W/8-1:0]    s_axis_tkeep,
  input  logic [TUSER_W-1:0]     s_axis_tuser,
  input  logic                   s_axis_tvalid,
  input  logic                   s_axis_tlast,
  output logic                   s_axis_tready,

  // ================= Client-IF toward Protocol (segments) =================
  output logic [IF_W-1:0]        cl_tx_data,
  output logic [IF_W/8-1:0]      cl_tx_keep,
  output logic [TUSER_W-1:0]     cl_tx_user,
  output logic                   cl_tx_valid,
  output logic                   cl_tx_sop,
  output logic                   cl_tx_eop,
  input  logic                   cl_tx_ready,

  // ================= CSR Control =================
  input  logic                   bridge_enable,
  input  logic                   strict_tkeep_en,
  input  logic [7:0]             tx_fifo_afull_thr,
  input  logic                   drop_on_midreset,

  // ================= CSR Status / Telemetry =================
  output logic [31:0]            stat_tx_frames,
  output logic [31:0]            stat_tx_bytes,
  output logic [15:0]            stat_tx_fifo_level,
  output logic [31:0]            stat_tx_stall_cycles,
  output logic                   ev_err_tkeep_illegal,
  output logic                   ev_err_midreset_drop,
  output logic                   ev_err_overflow_tx
);

  //--------------------------------------------------------------------------
  // פרמטרים פנימיים
  //--------------------------------------------------------------------------

  localparam int BYTES_PER_BEAT = DATA_W / 8;
  localparam int BYTES_PER_SEG  = IF_W   / 8;
  localparam int MAX_SEGS_PER_BEAT = (BYTES_PER_BEAT + BYTES_PER_SEG - 1) / BYTES_PER_SEG;

  localparam int BYTE_CNT_W = $clog2(BYTES_PER_BEAT+1);
  localparam int SEG_IDX_W  = $clog2(MAX_SEGS_PER_BEAT+1);

  initial begin
    // שמירה על הנחות סינתזה בסיסיות
    if (DATA_W % 8 != 0 || IF_W % 8 != 0) begin
      $error("DATA_W and IF_W must be multiples of 8");
    end
    if (IF_W > DATA_W) begin
      $error("Current axi_bridge_ip_tx assumes IF_W <= DATA_W");
    end
  end

  //--------------------------------------------------------------------------
  // פונקציות עזר: popcount + בדיקת חוקיות tkeep
  //--------------------------------------------------------------------------

  function automatic [BYTE_CNT_W-1:0] popcount_keep(
    input logic [BYTES_PER_BEAT-1:0] keep
  );
    int i;
    popcount_keep = '0;
    for (i = 0; i < BYTES_PER_BEAT; i++) begin
      if (keep[i]) popcount_keep++;
    end
  endfunction

  // tkeep_illegal: אם strict_tkeep_en=1, נבדוק:
  // - beat שאינו אחרון חייב להיות all-ones
  // - beat אחרון חייב להיות מונוטוני: 1..10..0, בלי "חור" 0 ואז שוב 1
  function automatic logic tkeep_illegal_f(
    input logic [BYTES_PER_BEAT-1:0] keep,
    input logic                       is_last
  );
    int i;
    logic seen_zero;
    tkeep_illegal_f = 1'b0;
    if (!is_last) begin
      if (keep != {BYTES_PER_BEAT{1'b1}})
        tkeep_illegal_f = 1'b1;
    end else begin
      seen_zero = 1'b0;
      for (i = 0; i < BYTES_PER_BEAT; i++) begin
        if (!keep[i]) begin
          seen_zero = 1'b1;
        end else if (seen_zero) begin
          // מצאנו 1 אחרי שכבר היו 0-ים -> לא מונוטוני
          tkeep_illegal_f = 1'b1;
        end
      end
    end
  endfunction

  //--------------------------------------------------------------------------
  // Ingress FIFO: שומר beats של AXI (tdata+tkeep+tlast+tuser)
  //--------------------------------------------------------------------------

  localparam int ENTRY_W = DATA_W + BYTES_PER_BEAT + TUSER_W + 1;

  logic                 fifo_push, fifo_pop;
  logic [ENTRY_W-1:0]   fifo_wdata, fifo_rdata;
  logic                 fifo_full, fifo_empty;
  logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level;
  logic                 fifo_clear;

  // פירוק ה-entry היוצא
  logic [DATA_W-1:0]            fifo_data;
  logic [BYTES_PER_BEAT-1:0]    fifo_keep;
  logic                         fifo_last;
  logic [TUSER_W-1:0]           fifo_user;

  assign fifo_data = fifo_rdata[DATA_W-1:0];
  assign fifo_keep = fifo_rdata[DATA_W +: BYTES_PER_BEAT];
  assign fifo_user = fifo_rdata[DATA_W + BYTES_PER_BEAT +: TUSER_W];
  assign fifo_last = fifo_rdata[ENTRY_W-1];

  // כתיבה ל-FIFO
  assign fifo_wdata = {
    s_axis_tlast,        // [ENTRY_W-1]
    s_axis_tuser,        // [..]
    s_axis_tkeep,        // [..]
    s_axis_tdata         // [DATA_W-1:0]
  };

  fifo_sync #(
    .WIDTH (ENTRY_W),
    .DEPTH (FIFO_DEPTH)
  ) u_ingress_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (fifo_clear),
    .push_i   (fifo_push),
    .wdata_i  (fifo_wdata),
    .pop_i    (fifo_pop),
    .rdata_o  (fifo_rdata),
    .full_o   (fifo_full),
    .empty_o  (fifo_empty),
    .level_o  (fifo_level)
  );

  //--------------------------------------------------------------------------
  // AXI-Stream handshaking מול ה-IP + backpressure
  //--------------------------------------------------------------------------

  logic s_axis_handshake;
  logic fifo_almost_full;

  // נטרול מודול אם bridge_enable=0
  // fifo_almost_full: נשען על סף tx_fifo_afull_thr (אק"ט לפי CSR)
  always_comb begin
    // השוואה ברוחב מתאים
    logic [$bits(fifo_level)-1:0] thr_trunc;
    thr_trunc = tx_fifo_afull_thr[$bits(fifo_level)-1:0];
    fifo_almost_full = (fifo_level >= thr_trunc);
  end

  assign s_axis_handshake = s_axis_tvalid && s_axis_tready;

  // s_axis_tready תלוי ב:
  // - bridge_enable
  // - FIFO לא מלא
  // - לא עברנו את סף ה-almost-full
  assign s_axis_tready =
    bridge_enable &&
    !fifo_full    &&
    !fifo_almost_full;

  // דחיפת beat ל-FIFO מתבצעת רק עם handshake אמיתי
  assign fifo_push = s_axis_handshake;

  //--------------------------------------------------------------------------
  // FSM לסיריאליזציה: DATA_W => IF_W segments
  //--------------------------------------------------------------------------

  // state של ה-beat הנוכחי
  logic [DATA_W-1:0]         cur_data;
  logic [BYTES_PER_BEAT-1:0] cur_keep;
  logic                      cur_last;
  logic [BYTE_CNT_W-1:0]     cur_valid_bytes;
  logic [SEG_IDX_W-1:0]      cur_num_segs;
  logic [BYTE_CNT_W-1:0]     cur_last_seg_bytes;
  logic [SEG_IDX_W-1:0]      cur_seg_idx;
  logic                      have_beat;

  // tracking של מסגרת (frame/packet)
  logic                      in_packet;
  logic [TUSER_W-1:0]        frame_user;

  // יציאה רשומה (skid קטן)
  logic [IF_W-1:0]           cl_tx_data_q;
  logic [IF_W/8-1:0]         cl_tx_keep_q;
  logic [TUSER_W-1:0]        cl_tx_user_q;
  logic                      cl_tx_valid_q;
  logic                      cl_tx_sop_q;
  logic                      cl_tx_eop_q;

  assign cl_tx_data  = cl_tx_data_q;
  assign cl_tx_keep  = cl_tx_keep_q;
  assign cl_tx_user  = cl_tx_user_q;
  assign cl_tx_valid = cl_tx_valid_q;
  assign cl_tx_sop   = cl_tx_sop_q;
  assign cl_tx_eop   = cl_tx_eop_q;


  // נעדיף register ל-pop כדי שלא ניצור קריאות כפולות
  logic fifo_pop_q;
  
  // pop מה-FIFO כאשר אנחנו צריכים beat חדש
  // (נשלוט עליו בפנים ב-always_ff)
  assign fifo_pop = fifo_pop_q; // יוגדר בלוגיקה הסיקוונציאלית בעזרת reg

  //--------------------------------------------------------------------------
  // אירועים למעלה ל-CSR: כ-pulse (1-cycle)
  //--------------------------------------------------------------------------

  logic ev_err_tkeep_illegal_int;
  logic ev_err_midreset_drop_int;
  logic ev_err_overflow_tx_int;

  assign ev_err_tkeep_illegal = ev_err_tkeep_illegal_int;
  assign ev_err_midreset_drop = ev_err_midreset_drop_int;
  assign ev_err_overflow_tx   = ev_err_overflow_tx_int;

  //--------------------------------------------------------------------------
  // Main sequential logic
  //--------------------------------------------------------------------------

  logic bridge_enable_q;
  logic tx_fire;
  logic [SEG_IDX_W-1:0] seg_idx_cur;
  int                   seg_byte_base;
  logic [IF_W/8-1:0]    seg_keep_mask;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // reset state
      bridge_enable_q        <= 1'b0;
      have_beat              <= 1'b0;
      in_packet              <= 1'b0;
      cur_data               <= '0;
      cur_keep               <= '0;
      cur_last               <= 1'b0;
      cur_valid_bytes        <= '0;
      cur_num_segs           <= '0;
      cur_last_seg_bytes     <= '0;
      cur_seg_idx            <= '0;

      frame_user             <= '0;

      cl_tx_data_q           <= '0;
      cl_tx_keep_q           <= '0;
      cl_tx_user_q           <= '0;
      cl_tx_valid_q          <= 1'b0;
      cl_tx_sop_q            <= 1'b0;
      cl_tx_eop_q            <= 1'b0;

      stat_tx_frames         <= '0;
      stat_tx_bytes          <= '0;
      stat_tx_fifo_level     <= '0;
      stat_tx_stall_cycles   <= '0;

      ev_err_tkeep_illegal_int <= 1'b0;
      ev_err_midreset_drop_int <= 1'b0;
      ev_err_overflow_tx_int   <= 1'b0;

      fifo_pop_q             <= 1'b0;
      fifo_clear             <= 1'b0;
    end else begin
      bridge_enable_q <= bridge_enable;

      // ברירת מחדל: אין pop / אין clear / אין אירוע
      fifo_pop_q              <= 1'b0;
      fifo_clear              <= 1'b0;
      ev_err_tkeep_illegal_int <= 1'b0;
      ev_err_midreset_drop_int <= 1'b0;
      ev_err_overflow_tx_int   <= 1'b0;

      // עדכון עומק FIFO לסטטוס
      stat_tx_fifo_level <= {{(16-$bits(fifo_level)){1'b0}}, fifo_level};

      // overflow detection: אם מישהו דוחף ל-FIFO כשהוא full (לא אמור לקרות אם tready מכובד)
      if (fifo_push && fifo_full) begin
        ev_err_overflow_tx_int <= 1'b1;
      end

      // count bytes שנקלטו מה-IP
      if (s_axis_handshake) begin
        stat_tx_bytes <= stat_tx_bytes + popcount_keep(s_axis_tkeep);
        // strict_tkeep: בדיקת חוקיות
        if (strict_tkeep_en && tkeep_illegal_f(s_axis_tkeep, s_axis_tlast)) begin
          ev_err_tkeep_illegal_int <= 1'b1;
        end
      end

      // stall counter: כאשר יש לנו נתון ליציאה אבל הצד השני לא מוכן
      if (bridge_enable && cl_tx_valid_q && !cl_tx_ready) begin
        stat_tx_stall_cycles <= stat_tx_stall_cycles + 1;
      end

      // midreset/drop_on_midreset: אם bridge_enable יורד באמצע frame או שיש דאטה ב-FIFO
      if (drop_on_midreset &&
          bridge_enable_q && !bridge_enable &&
          (in_packet || have_beat || !fifo_empty)) begin
        ev_err_midreset_drop_int <= 1'b1;
        // פינוי כל ה-state
        in_packet     <= 1'b0;
        have_beat     <= 1'b0;
        cl_tx_valid_q <= 1'b0;
        fifo_clear    <= 1'b1;
      end

      // אם bridge_enable=0 – נעצור שידור החוצה (לא נקבל beats חדשים גם בגלל s_axis_tready=0)
      if (!bridge_enable) begin
        cl_tx_valid_q <= 1'b0;
      end else begin
        //========================
        // שלב 1: טעינת beat חדש מה-FIFO (אם אין לנו beat פעיל)
        //========================
        if (!have_beat && !fifo_empty && !cl_tx_valid_q) begin
          // נקרא את ה-entry הנוכחי
          cur_data        <= fifo_data;
          cur_keep        <= fifo_keep;
          cur_last        <= fifo_last;
          cur_valid_bytes <= popcount_keep(fifo_keep);

          // חישוב מספר הסגמנטים והבייטים בסגמנט האחרון
          if (popcount_keep(fifo_keep) == 0) begin
            cur_num_segs       <= '0;
            cur_last_seg_bytes <= '0;
          end else begin
            cur_num_segs       <= (popcount_keep(fifo_keep) + BYTES_PER_SEG - 1) / BYTES_PER_SEG;
            cur_last_seg_bytes <= popcount_keep(fifo_keep)
                                  - (((popcount_keep(fifo_keep) + BYTES_PER_SEG - 1) / BYTES_PER_SEG) - 1)
                                    * BYTES_PER_SEG;
          end
          cur_seg_idx     <= '0;
          have_beat       <= (popcount_keep(fifo_keep) != 0);

          // נעדכן frame_user אם זו תחילת מסגרת חדשה
          if (!in_packet) begin
            frame_user <= fifo_user;
          end

          fifo_pop_q <= 1'b1; // pop beat אחד
        end

        //========================
        // שלב 2: FSM לסגמנטים + יציאה רשומה
        //========================
        tx_fire = cl_tx_valid_q && cl_tx_ready;

        // ספירת frames: בכל פעם ש-EOP יוצא בהנדשייק
        if (tx_fire && cl_tx_eop_q) begin
          stat_tx_frames <= stat_tx_frames + 1;
        end

        // עדכון מצב in_packet לפי הסגמנט שנשלח במחזור הקודם
        if (tx_fire) begin
          if (!in_packet && (cur_seg_idx == '0)) begin
            in_packet <= 1'b1;
          end
          if (cur_last && (cur_seg_idx == cur_num_segs - 1)) begin
            in_packet <= 1'b0;
          end
        end

        // לוגיקה של יציאת סגמנטים:
        if (cl_tx_valid_q && !cl_tx_ready) begin
          // backpressure – מחכים, משאירים הכל
        end else begin
          if (have_beat && (cur_num_segs != 0)) begin
            // יש לנו beat טעון וצריך להוציא סגמנט במחזור הזה

            // אינדקס הסגמנט שיוצא במחזור הנוכחי
            seg_idx_cur = cur_seg_idx;

            // אם במחזור הקודם היה handshake – עוברים לסגמנט הבא
            if (tx_fire) begin
              if (cur_seg_idx < cur_num_segs - 1) begin
                seg_idx_cur = cur_seg_idx + 1;
              end else begin
                seg_idx_cur = cur_num_segs - 1; // נשאר על האחרון
              end
            end

            // חישוב בסיס הבייטים וה-keep לפי seg_idx_cur
            seg_byte_base = seg_idx_cur * BYTES_PER_SEG;
            cl_tx_data_q  <= cur_data[(seg_byte_base*8) +: IF_W];

            if (seg_idx_cur < cur_num_segs - 1) begin
              seg_keep_mask = {BYTES_PER_SEG{1'b1}};
            end else begin
              if (cur_last_seg_bytes == 0 || cur_last_seg_bytes == BYTES_PER_SEG) begin
                seg_keep_mask = {BYTES_PER_SEG{1'b1}};
              end else begin
                seg_keep_mask = '0;
                for (int b = 0; b < BYTES_PER_SEG; b++) begin
                  if (b < cur_last_seg_bytes) seg_keep_mask[b] = 1'b1;
                end
              end
            end
            cl_tx_keep_q <= seg_keep_mask;

            // מטא-דאטה
            cl_tx_user_q <= frame_user;

            // SOP/EOP לפי seg_idx_cur
            cl_tx_sop_q  <= (!in_packet && (seg_idx_cur == '0));
            cl_tx_eop_q  <= (cur_last && (seg_idx_cur == cur_num_segs - 1));

            // יש סגמנט תקף על הקו
            cl_tx_valid_q <= 1'b1;

            // עדכון state
            cur_seg_idx <= seg_idx_cur;

            // אם במחזור הקודם היה handshake על הסגמנט האחרון – הבייט הזה הסתיים
            if (tx_fire && (cur_seg_idx == cur_num_segs - 1)) begin
              have_beat     <= 1'b0;
              cl_tx_valid_q <= 1'b0;
            end

          end else begin
            cl_tx_valid_q <= 1'b0;
            cl_tx_sop_q   <= 1'b0;
            cl_tx_eop_q   <= 1'b0;
          end
        end // if (!stall)
      end // if bridge_enable
    end // !rst
  end // always_ff

endmodule
