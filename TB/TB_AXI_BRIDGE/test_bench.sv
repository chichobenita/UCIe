// tb_axi_bridge_ip_tx.sv
`timescale 1ns/1ps

import axi_bridge_ip_tx_tb_pkg::*;

module tb_axi_bridge_ip_tx;

  // Same parameter values as in the DUT / package (mirrors axi_bridge_ip_tx_tb_pkg)
  localparam int DATA_W   = DATA_W;
  localparam int IF_W     = IF_W;
  localparam int TUSER_W  = TUSER_W;

  // Clock and reset
  logic clk;
  logic rst_n;

  initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 100 MHz clock (10 ns period)
  end

  // Interface instances (connects to DUT)
  axi_stream_if #(DATA_W, TUSER_W) axi_if(.clk(clk), .rst_n(rst_n));
  client_if     #(IF_W,   TUSER_W) cl_if (.clk(clk), .rst_n(rst_n));

  // CSR and status signals from DUT
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
  
  // Tracking last observed Client-IF values for backpressure checks
  logic [IF_W-1:0]        last_data;
  logic [IF_W/8-1:0]      last_keep;
  logic [TUSER_W-1:0]     last_user;
  logic                   last_sop, last_eop;
  logic                   prev_valid, prev_ready;
  bit                     first_sample;
  
  // Backpressure behavior configuration
  // 0 = regular short-stall pattern (test 5), 1 = long stall (test 6)
  logic bp_long_stall;
  int unsigned  max_fifo_level;
  logic         saw_tready_low_flag;
  logic         saw_tready_high_flag;
  
  // Mid-reset error counter (for test 7)
  int unsigned  midreset_err_cnt;
  int unsigned midreset_before;
  int unsigned frames_before;
  int unsigned bytes_before;

  // DUT instance
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

    // Status / events
    .stat_tx_frames     (stat_tx_frames),
    .stat_tx_bytes      (stat_tx_bytes),
    .stat_tx_fifo_level (stat_tx_fifo_level),
    .stat_tx_stall_cycles(stat_tx_stall_cycles),
    .ev_err_tkeep_illegal(ev_err_tkeep_illegal),
    .ev_err_midreset_drop(ev_err_midreset_drop),
    .ev_err_overflow_tx (ev_err_overflow_tx)
  );

  // UVM-like components (simple SV classes): Driver / Monitor / Scoreboard
  axi_driver      drv;
  client_monitor  mon;
  scoreboard      sb;

  // Counter for tkeep-illegal error event (test 4 / coverage)
  int unsigned tkeep_err_cnt;

  // Count the number of cycles where ev_err_tkeep_illegal is asserted
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tkeep_err_cnt <= 0;
    end else if (ev_err_tkeep_illegal) begin
      tkeep_err_cnt <= tkeep_err_cnt + 1;
    end
  end

  //============== Backpressure generator (for test 5 / 6) ==============

  // Enable for backpressure pattern generator
  logic backpressure_en;

  // Reference counter for cycles where valid && !ready on Client-IF
  int unsigned stall_ref_cnt;

  // Backpressure cycle counter (drives cl_tx_ready pattern)
  int unsigned bp_cycle;

  // cl_tx_ready generator + reference stall counter
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cl_if.ready   <= 1'b1;
      bp_cycle      <= 0;
      stall_ref_cnt <= 0;
    end else begin
      // Reference count of stall cycles: valid && !ready while bridge is enabled
      if (bridge_enable && cl_if.valid && !cl_if.ready)
        stall_ref_cnt <= stall_ref_cnt + 1;

      if (!backpressure_en) begin
        // No backpressure: always ready
        cl_if.ready <= 1'b1;
        bp_cycle    <= 0;
      end else begin
        // Backpressure enabled: advance local pattern counter
        bp_cycle <= bp_cycle + 1;

        if (bp_long_stall) begin
          // "Heavy stall" mode – cl_tx_ready=0 for ~80 cycles, then deassert
          if (bp_cycle < 80)
            cl_if.ready <= 1'b0;
          else
            cl_if.ready <= 1'b1;
        end else begin
          // Original mode (for test 5) – deterministic pattern + random stalls
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

  //===========================================================
  // Backpressure stability check on Client-IF
  // Ensures cl_tx_* signals are stable while valid && !ready
  //===========================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_valid <= 1'b0;
      last_data  <= '0;
      last_keep  <= '0;
      last_user  <= '0;
      last_sop   <= 1'b0;
      last_eop   <= 1'b0;
    end else begin
      // Check stability only if previous and current cycles both have valid && !ready
      if ((cl_if.valid && !cl_if.ready) && (prev_valid && !prev_ready)) begin
        if (cl_if.data !== last_data ||
            cl_if.keep !== last_keep ||
            cl_if.user !== last_user ||
            cl_if.sop  !== last_sop  ||
            cl_if.eop  !== last_eop) begin
          $error("Backpressure violation: cl_tx_* changed while valid && !ready");
        end
      end

      // On any cycle with valid=1, capture the current values
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
  
  //=========================================
  //========= Test 6 helper logic ===========
  //=========================================

  // Track maximum FIFO level and tready behavior over entire simulation
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      max_fifo_level       <= 0;
      saw_tready_low_flag  <= 1'b0;
      saw_tready_high_flag <= 1'b0;
    end else begin
      // Track maximum observed FIFO level
      if (stat_tx_fifo_level > max_fifo_level)
        max_fifo_level <= stat_tx_fifo_level;

      // Flag: we saw condition where FIFO >= threshold and tready=0
      if (!axi_if.tready && (stat_tx_fifo_level >= tx_fifo_afull_thr))
        saw_tready_low_flag <= 1'b1;

      // After we saw low, flag that we also saw tready return high
      if (saw_tready_low_flag &&
          axi_if.tready &&
          (stat_tx_fifo_level < tx_fifo_afull_thr))
        saw_tready_high_flag <= 1'b1;
    end
  end
  //=========================================

  // ======== Counter for test 7 (mid-reset drops) ==========
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      midreset_err_cnt <= 0;
    else if (ev_err_midreset_drop)
      midreset_err_cnt <= midreset_err_cnt + 1;
  end
  //=========================================================
  
  // Enable flags for each test scenario
  localparam bit ENABLE_TEST1  = 0;
  localparam bit ENABLE_TEST2  = 0;
  localparam bit ENABLE_TEST3  = 0;
  localparam bit ENABLE_TEST4  = 0;
  localparam bit ENABLE_TEST5  = 0; 
  localparam bit ENABLE_TEST6  = 0;
  localparam bit ENABLE_TEST7a = 1;
  localparam bit ENABLE_TEST7b = 1;
  localparam bit ENABLE_TEST7cd = 1;

  //=========================================================
  // Main test sequence
  //=========================================================
  initial begin : main_test
    // === Declarations (kept here for clarity) ===
    axi_frame  frame, frame2, frame3, frame4_legal, frame_small, frame5, frame6;
    axi_frame  frame7a, frame7b, frame7c, frame7d, frame7e;
    axi_frame  frame4_illegal_nl;   // Non-last-beat illegal tkeep
    axi_frame  frame4_illegal_last; // Last-beat illegal tkeep
    axi_beat_t beat, beat2, beat4, beat_full, beat_last, beat_small, beat5, beat6;
    axi_beat_t beat7;
    int unsigned N_BEATS;
    int unsigned exp_frames, exp_bytes;
    int unsigned num_bytes;
    int unsigned err_before, err_after;
    int unsigned N_BEATS5;
    int unsigned N_BEATS6;
    int unsigned N_BEATS7;
  
    // Keep previous values of stall counters and DUT stall statistic
    int unsigned stall_ref_before;
    int unsigned stall_stat_before;

    // Delta computation for stall cycles (TB vs DUT)
    int unsigned stall_ref_after;
    int unsigned stall_stat_after;

    int unsigned ref_delta;
    int unsigned dut_delta;

    //========================
    // Global initialization
    //========================
    rst_n            = 0;
    bridge_enable    = 0;
    strict_tkeep_en  = 0;
    drop_on_midreset = 0;
    tx_fifo_afull_thr= 8'd16; // Above FIFO depth, so we don't hit "almost-full" by default
    cl_if.ready      = 1'b1;  // No backpressure at start

    // Apply reset for a few cycles
    repeat (4) @(posedge clk);
    rst_n = 1;

    // Create virtual interfaces and SV objects (driver / monitor / scoreboard)
    drv = new(axi_if);
    mon = new(cl_if);
    sb  = new(mon);

    // Run monitor in background (collects Client-IF activity)
    fork
      mon.run();
    join_none

    // Wait a few cycles after reset, then enable the bridge
    repeat (2) @(posedge clk);
    bridge_enable = 1'b1;

    // ------------------------------------------------
    // TEST 1: single-beat frame with full tkeep
    // ------------------------------------------------
    if (ENABLE_TEST1) begin
      frame = new();  // new after declaration, not in the declaration line

      beat.data = 256'h1122334455667788_99AABBCCDDEEFF00_F0E0D0C0B0A09080_7766554433221100;
      beat.keep = {BYTES_PER_BEAT{1'b1}}; // All 32 bytes are valid
      beat.last = 1'b1;
      beat.user = 16'h00AA;

      frame.add_beat(beat);

      // Send frame into the DUT
      $display("[TEST] Sending single-beat frame with full tkeep");
      drv.drive_frame(frame);

      // Functional check via scoreboard
      sb.check_frame(frame);

      // Some extra time for DUT to finish all segments
      repeat (10) @(posedge clk);
    end
  
    // ------------------------------------------------
    // TEST 2: multi-beat frame, full tkeep, TLAST on last beat
    // ------------------------------------------------
    if (ENABLE_TEST2) begin  
      N_BEATS = 3;
      frame2  = new();

      for (int i = 0; i < N_BEATS; i++) begin
        // Simple recognizable pattern (can be changed if needed)
        beat2.data = {
          64'h1000_0000_0000_0000 + i,
          64'h2000_0000_0000_0000 + i,
          64'h3000_0000_0000_0000 + i,
          64'h4000_0000_0000_0000 + i
        };
        beat2.keep = {BYTES_PER_BEAT{1'b1}}; // All bytes are valid
        beat2.last = (i == N_BEATS-1);       // TLAST only on the last beat
        beat2.user = 16'h00BB;               // Different user from test 1

        frame2.add_beat(beat2);
      end

      $display("[TEST] Sending multi-beat frame with %0d beats (full tkeep)", N_BEATS);
      drv.drive_frame(frame2);
      sb.check_frame(frame2);

      // Extra time for all segments to be drained
      repeat (10) @(posedge clk);

      // ------------------------------------------------
      // Cumulative statistics checks (tests 1 + 2)
      // ------------------------------------------------
      // Test 1: 1 full beat  → BYTES_PER_BEAT bytes
      // Test 2: N_BEATS full beats → N_BEATS * BYTES_PER_BEAT bytes
      exp_frames = 2;                               // Two frames total
      exp_bytes  = BYTES_PER_BEAT * (1 + N_BEATS);  // Total bytes transmitted

      if (stat_tx_frames !== exp_frames) begin
        $error("stat_tx_frames expected %0d, got %0d",
               exp_frames, stat_tx_frames);
      end

      if (stat_tx_bytes !== exp_bytes) begin
        $error("stat_tx_bytes expected %0d, got %0d",
               exp_bytes, stat_tx_bytes);
      end
    end
  
    // ------------------------------------------------
    // TEST 3a: multi-beat frame, last beat with partial tkeep
    // ------------------------------------------------
    if (ENABLE_TEST3) begin
      // Sweep over 1..(BYTES_PER_SEG-1) valid bytes in last beat
      for (num_bytes = 1; num_bytes < BYTES_PER_SEG; num_bytes++) begin
        frame3 = new();

        // First beat: full (all bytes valid)
        beat_full.data = '0;
        for (int b = 0; b < BYTES_PER_BEAT; b++) begin
          beat_full.data[b*8 +: 8] = 8'h10 + b; // Easy-to-recognize pattern
        end
        beat_full.keep = {BYTES_PER_BEAT{1'b1}};
        beat_full.last = 1'b0;          // Not TLAST
        beat_full.user = 16'h00CC;
        frame3.add_beat(beat_full);

        // Last beat: only num_bytes valid bytes
        beat_last.data = '0;
        for (int b = 0; b < num_bytes; b++) begin
          beat_last.data[b*8 +: 8] = 8'h80 + b; // Different pattern from first beat
        end

        beat_last.keep = '0;
        for (int b = 0; b < num_bytes; b++) begin
          beat_last.keep[b] = 1'b1;     // tkeep[0..num_bytes-1] = 1, rest 0
        end

        beat_last.last = 1'b1;          // TLAST on the last beat
        beat_last.user = 16'h00CC;

        frame3.add_beat(beat_last);

        $display("[TEST 3a] Frame with full beat + last beat with %0d valid bytes", num_bytes);

        drv.drive_frame(frame3);
        sb.check_frame(frame3);

        // Time for all segments to be transmitted before next iteration
        repeat (10) @(posedge clk);

        // Ensure no extra segments beyond the expected ones
        sb.check_no_extra_segments($sformatf("TEST 3a (num_bytes=%0d)", num_bytes));
      end

      // ------------------------------------------------
      // TEST 3b: single-beat frame smaller than one segment
      // ------------------------------------------------
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

        beat_small.last = 1'b1;         // TLAST on a single beat
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

    // Make sure there are no leftover segments from previous tests in the monitor queue
    mon.seg_q.delete();

    // Enable strict tkeep mode
    strict_tkeep_en = 1'b1;

      // 4a) Legal frame (full tkeep, TLAST on last beat) -> ev_err_tkeep_illegal must NOT fire
    frame4_legal = new();

    beat4.data = 256'hCAFEBABE_DEADBEEF_0011223344556677_8899AABBCCDDEEFF;
    beat4.keep = {BYTES_PER_BEAT{1'b1}};   // Full beat (all bytes valid)
    beat4.last = 1'b1;                     // TLAST asserted
    beat4.user = 16'h0E4A;                 // Different user tag for identification

    frame4_legal.add_beat(beat4);

    err_before = tkeep_err_cnt;
    $display("[TEST 4a] Legal frame with strict_tkeep_en=1 (full keep, TLAST only on last beat)");

    drv.drive_frame(frame4_legal);
    sb.check_frame(frame4_legal); // Here we require 100% correct data

    repeat (10) @(posedge clk);
    err_after = tkeep_err_cnt;

    if (err_after != err_before) begin
      $error("TEST 4a: ev_err_tkeep_illegal fired on a legal frame (before=%0d, after=%0d)",
             err_before, err_after);
    end

    // Extra safety – make sure there are no unexpected extra segments
    sb.check_no_extra_segments("TEST 4a");

      // 4b) Non-last beat with partial tkeep -> expect ev_err_tkeep_illegal, but data must still flow
    frame4_illegal_nl = new();
    mon.seg_q.delete();

    // beat 0: non-last, partial keep (not all-ones) -> ILLEGAL in strict mode
    beat4.data = '0;
    for (int b = 0; b < BYTES_PER_BEAT; b++) begin
      beat4.data[b*8 +: 8] = 8'h20 + b;
    end
    beat4.keep = '0;
    beat4.keep[0] = 1'b1;
    beat4.keep[1] = 1'b1;   // Only two bytes valid, last=0 -> forbidden in strict mode
    beat4.last = 1'b0;
    beat4.user = 16'h0BAD;
    frame4_illegal_nl.add_beat(beat4);

    // beat 1: last, full keep, legal
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

    // Allow enough time for the bridge to transmit all segments
    repeat (40) @(posedge clk);
    err_after = tkeep_err_cnt;

    if (err_after <= err_before) begin
      $error("TEST 4b: Expected ev_err_tkeep_illegal for non-last beat with partial keep, but counter did not increase (before=%0d, after=%0d)",
             err_before, err_after);
    end

    // Make sure data actually flowed – at least one segment must have been transmitted
    if (mon.seg_q.size() == 0) begin
      $error("TEST 4b: Illegal tkeep on non-last beat blocked data – expected some segments on cl_tx");
    end

    // Clear monitor queue before the next test
    mon.seg_q.delete();

      // 4c) Last beat with non-monotonic tkeep -> again expect ev_err_tkeep_illegal
    frame4_illegal_last = new();
    mon.seg_q.delete();

    beat4.data = '0;
    for (int b = 0; b < BYTES_PER_BEAT; b++) begin
      beat4.data[b*8 +: 8] = 8'h80 + b;
    end

    // Non-monotonic tkeep: 1,1,1,0,1,... (a “hole” in the middle)
    beat4.keep = '0;
    beat4.keep[0] = 1'b1;
    beat4.keep[1] = 1'b1;
    beat4.keep[2] = 1'b1;
    beat4.keep[4] = 1'b1;   // Hole at index 3 => non-monotonic pattern

    beat4.last = 1'b1;      // TLAST asserted
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

    // Optionally disable strict_tkeep_en for subsequent tests
    strict_tkeep_en = 1'b0;
  end
  
  
    //==================
  //===== test 5 =====
  //==================
  
    if (ENABLE_TEST5) begin
      $display("=== TEST 5: Client backpressure ===");

      // Build a large enough frame so that we get stalls in the middle
      frame5 = new();
      
      N_BEATS5 = 4;     // 4 full beats
      for (int i = 0; i < N_BEATS5; i++) begin
        beat5.data = {
          64'hA000_0000_0000_0000 + i,
          64'hB000_0000_0000_0000 + i,
          64'hC000_0000_0000_0000 + i,
          64'hD000_0000_0000_0000 + i
        };
        beat5.keep = {BYTES_PER_BEAT{1'b1}};
        beat5.last = (i == N_BEATS5-1);
        beat5.user = 16'h0B5A;   // Just a unique tag
        frame5.add_beat(beat5);
      end

      // Clear monitor queue
      mon.seg_q.delete();

      // Enable backpressure generation
      backpressure_en = 1'b1;

      // Save previous values of stall_ref_cnt and stat_tx_stall_cycles
      stall_ref_before   = stall_ref_cnt;
      stall_stat_before  = stat_tx_stall_cycles;

      $display("[TEST 5] Driving multi-beat frame under random backpressure");
      drv.drive_frame(frame5);

      // Scoreboard: checks that there are no duplicates / drops / reordering in segments
      sb.check_frame(frame5);

      // Give some extra cycles for the last segments to drain
      repeat (50) @(posedge clk);

      // Disable backpressure to avoid affecting other tests
      backpressure_en = 1'b0;

      // Compute stall deltas (TB reference vs DUT counters)
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

      // Extra safety – check no unexpected extra segments
      sb.check_no_extra_segments("TEST 5");

      $display("=== TEST 5 DONE ===");
    end
     
  
  //==================
  //===== test 6 =====
  //==================
  
      if (ENABLE_TEST6) begin
      $display("=== TEST 6: IP-side backpressure / FIFO almost-full ===");

      // Configuration specific for this test
      strict_tkeep_en   = 1'b0;
      drop_on_midreset  = 1'b0;

      // Low almost-full threshold so we hit it quickly
      tx_fifo_afull_thr = 8'd4;

      // Heavy backpressure on the Client side
      backpressure_en   = 1'b1;
      bp_long_stall     = 1'b1;

      // Reset monitor queue
      mon.seg_q.delete();

      // Build a large frame – many full beats
      frame6   = new();
      N_BEATS6 = 32;    // Example: 32 beats

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

      // Send + normal functional checking (no drops/duplicates)
      drv.drive_frame(frame6);
      sb.check_frame(frame6);
      sb.check_no_extra_segments("TEST 6");

      // Give counters some extra cycles to settle
      repeat (20) @(posedge clk);

      // ==== Test-specific checks ====

      // 1) Did tready ever deassert while fifo_level >= threshold?
      if (!saw_tready_low_flag) begin
        $error("TEST 6: s_axis_tready never deasserted when FIFO level >= tx_fifo_afull_thr");
      end

      // 2) After that, did tready reassert when FIFO dropped below the threshold?
      if (!saw_tready_high_flag) begin
        $error("TEST 6: s_axis_tready never reasserted after FIFO drained below threshold");
      end

      // 3) No overflow is expected at all
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
      // 7a) bridge_enable=0 -> no traffic should pass
      //---------------------------------------------
      strict_tkeep_en   = 1'b0;
      drop_on_midreset  = 1'b0;
      tx_fifo_afull_thr = 8'd16;   // High threshold, should not interfere
      backpressure_en   = 1'b0;    // cl_if.ready always 1

      bridge_enable     = 1'b0;

      mon.seg_q.delete();
      frames_before = stat_tx_frames;
      bytes_before  = stat_tx_bytes;

      $display("[TEST 7a] Driving manual TVALID pulses while bridge_enable=0 - expect NO output");

      // Manually drive a single beat, without waiting for tready
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

      // "IP" tries to send for several cycles, but bridge is disabled so tready=0
      repeat (5) @(posedge clk);

      // Stop transmission
      axi_if.tvalid <= 1'b0;
      axi_if.tlast  <= 1'b0;
      axi_if.tdata  <= '0;
      axi_if.tkeep  <= '0;
      axi_if.tuser  <= '0;

      // Wait a few more cycles to ensure no reaction
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
      // 7b) bridge_enable goes to 1 -> frame should pass normally
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
      // 7c) drop_on_midreset=1, disable bridge in the middle of an active frame
      //---------------------------------------------
  	  if (ENABLE_TEST7cd) begin
      $display("=== TEST 7cd: bridge_enable & drop_on_midreset ===");
  	  drop_on_midreset = 1'b1;    // Enable drop-on-midreset mode
      backpressure_en  = 1'b0;    // cl_if.ready always 1, no stalls

      frame7c  = new();
      N_BEATS7 = 8;               // Long enough frame

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
        // Branch 1 – drive the frame
        begin
          drv.drive_frame(frame7c);
        end
        // Branch 2 – wait for frame start on client side, then drop bridge_enable mid-frame
        begin
          // Wait for handshake of the first segment (SOP) on Client side
          @(posedge clk iff (cl_if.valid && cl_if.ready && cl_if.sop));
          // Let a few segments pass
          repeat (3) @(posedge clk);
          bridge_enable = 1'b0; // Disable in the middle of the frame
          repeat (5) @(posedge clk);
          bridge_enable = 1'b1; // Re-enable
        end
      join

      // Allow time for the flush to occur
      repeat (30) @(posedge clk);

      if (midreset_err_cnt != midreset_before + 1)
        $error("TEST 7c: expected 1 ev_err_midreset_drop event, got %0d (before=%0d)",
               midreset_err_cnt, midreset_before);

      // After midreset + re-enable – send a new clean frame and verify it is correct
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
      // 7d) drop_on_midreset=1, FIFO has data but no frame has started on client side yet
      //---------------------------------------------
      $display("[TEST 7d] drop_on_midreset=1, FIFO has data but no frame started");

      // Force a situation where Client is not ready -> beats accumulate in FIFO
      backpressure_en  = 1'b1;
      bp_long_stall    = 1'b1;  // cl_if.ready=0 for a long period
      bridge_enable    = 1'b1;  // Active
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
        // Branch 1 – IP tries to send frame while ready=0, FIFO fills up
        begin
          drv.drive_frame(frame7e);
        end
        // Branch 2 – after some cycles, drop bridge_enable before any segment is sent out
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

      // No segments from this frame should be observed (client was always not-ready)
      if (mon.seg_q.size() != 0)
        $error("TEST 7d: got unexpected segments although cl_tx_ready was low and midreset dropped FIFO contents");

      // Again – after everything, send a clean frame to ensure no mixing/corruption
      backpressure_en = 1'b0;  // client ready
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
    $dumpfile("waves.vcd");              // Waveform dump file name
    $dumpvars(0, tb_axi_bridge_ip_tx);   // Dump entire hierarchy under the top-level TB
  end
  
endmodule




