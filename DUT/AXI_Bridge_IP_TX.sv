// Ingress path: AXI-Stream from IP -> Client-IF segments
// This module receives wide AXI-Stream beats from the IP (DATA_W bits),
// buffers them in a FIFO, and serializes them into narrower Client-IF
// segments (IF_W bits) with SOP/EOP and per-byte keep. It also exposes
// basic counters and error events for CSR/diagnostics.
module axi_bridge_ip_tx #(
  parameter int DATA_W   = 256,  // AXI-Stream data width on IP side
  parameter int IF_W     = 64,   // Client-IF segment width toward Protocol
  parameter int TUSER_W  = 16,   // Metadata width (VC, DST, etc.)
  parameter int FIFO_DEPTH = 16  // Ingress FIFO depth in AXI beats
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  // ================= AXI-Stream from IP =================
  // Wide AXI-Stream payload coming from the IP (DATA_W bits + TKEEP/TUSER)
  input  logic [DATA_W-1:0]      s_axis_tdata,
  input  logic [DATA_W/8-1:0]    s_axis_tkeep,  // Per-byte valid mask
  input  logic [TUSER_W-1:0]     s_axis_tuser,  // Frame-level metadata (e.g. VC, DST)
  input  logic                   s_axis_tvalid, // Beat valid from IP
  input  logic                   s_axis_tlast,  // Last beat of AXI frame
  output logic                   s_axis_tready, // Backpressure toward IP

  // ================= Client-IF toward Protocol (segments) =================
  // Serialized segments toward Protocol core (narrower IF_W width)
  output logic [IF_W-1:0]        cl_tx_data,    // Segment payload
  output logic [IF_W/8-1:0]      cl_tx_keep,    // Per-byte valid mask in segment
  output logic [TUSER_W-1:0]     cl_tx_user,    // Metadata replicated per frame
  output logic                   cl_tx_valid,   // Segment valid
  output logic                   cl_tx_sop,     // Start-of-packet indicator
  output logic                   cl_tx_eop,     // End-of-packet indicator
  input  logic                   cl_tx_ready,   // Backpressure from Protocol

  // ================= CSR Control =================
  // Control signals from CSR (already synchronized to clk_i domain)
  input  logic                   bridge_enable,     // Global enable for TX path
  input  logic                   strict_tkeep_en,   // Enforce strict tkeep legality
  input  logic [7:0]             tx_fifo_afull_thr, // FIFO almost-full threshold (beats)
  input  logic                   drop_on_midreset,  // Drop packet on mid-reset event

  // ================= CSR Status / Telemetry =================
  // Status counters and error events toward CSR (1-cycle pulses for events)
  output logic [31:0]            stat_tx_frames,        // Number of frames transmitted
  output logic [31:0]            stat_tx_bytes,         // Number of payload bytes accepted from IP
  output logic [15:0]            stat_tx_fifo_level,    // Ingress FIFO fill level
  output logic [31:0]            stat_tx_stall_cycles,  // Cycles with cl_tx_valid && !cl_tx_ready
  output logic                   ev_err_tkeep_illegal,  // Illegal tkeep pattern detected
  output logic                   ev_err_midreset_drop,  // Packet dropped due to mid-reset
  output logic                   ev_err_overflow_tx     // FIFO overflow due to protocol violation
);

  //--------------------------------------------------------------------------
  // Internal parameters
  //--------------------------------------------------------------------------

  localparam int BYTES_PER_BEAT = DATA_W / 8;
  localparam int BYTES_PER_SEG  = IF_W   / 8;
  localparam int MAX_SEGS_PER_BEAT = (BYTES_PER_BEAT + BYTES_PER_SEG - 1) / BYTES_PER_SEG;

  localparam int BYTE_CNT_W = $clog2(BYTES_PER_BEAT+1);
  localparam int SEG_IDX_W  = $clog2(MAX_SEGS_PER_BEAT+1);

  initial begin
    // Basic synthesis-time sanity checks on interface widths
    if (DATA_W % 8 != 0 || IF_W % 8 != 0) begin
      $error("DATA_W and IF_W must be multiples of 8");
    end
    if (IF_W > DATA_W) begin
      $error("Current axi_bridge_ip_tx assumes IF_W <= DATA_W");
    end
  end

  //--------------------------------------------------------------------------
  // Helper functions: popcount for TKEEP + tkeep legality check
  //--------------------------------------------------------------------------

  // Count how many bytes are valid in this beat according to TKEEP
  function automatic [BYTE_CNT_W-1:0] popcount_keep(
    input logic [BYTES_PER_BEAT-1:0] keep
  );
    int i;
    popcount_keep = '0;
    for (i = 0; i < BYTES_PER_BEAT; i++) begin
      if (keep[i]) popcount_keep++;
    end
  endfunction

  // tkeep_illegal_f:
  // If strict_tkeep_en=1, we enforce:
  // - Non-last beats must have all-ones TKEEP (all bytes valid).
  // - Last beat must be monotonic: 1...10...0 (no 0 then 1 "holes" inside).
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
          // Found a '1' after '0's -> non-monotonic TKEEP pattern
          tkeep_illegal_f = 1'b1;
        end
      end
    end
  endfunction

  //--------------------------------------------------------------------------
  // Ingress FIFO: stores AXI beats (tdata + tkeep + tlast + tuser)
  //--------------------------------------------------------------------------
  // Each FIFO entry holds a complete AXI beat + its metadata.
  // This decouples the IP's AXI-Stream timing from downstream serialization
  // into IF_W segments.

  localparam int ENTRY_W = DATA_W + BYTES_PER_BEAT + TUSER_W + 1;

  logic                 fifo_push, fifo_pop;
  logic [ENTRY_W-1:0]   fifo_wdata, fifo_rdata;
  logic                 fifo_full, fifo_empty;
  logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level;
  logic                 fifo_clear;

  // Decompose FIFO read data into fields
  logic [DATA_W-1:0]            fifo_data;
  logic [BYTES_PER_BEAT-1:0]    fifo_keep;
  logic                         fifo_last;
  logic [TUSER_W-1:0]           fifo_user;

  assign fifo_data = fifo_rdata[DATA_W-1:0];
  assign fifo_keep = fifo_rdata[DATA_W +: BYTES_PER_BEAT];
  assign fifo_user = fifo_rdata[DATA_W + BYTES_PER_BEAT +: TUSER_W];
  assign fifo_last = fifo_rdata[ENTRY_W-1];

  // FIFO write packing order
  assign fifo_wdata = {
    s_axis_tlast,        // [ENTRY_W-1]  : last-beat flag
    s_axis_tuser,        // [..]        : metadata
    s_axis_tkeep,        // [..]        : byte-valid mask
    s_axis_tdata         // [DATA_W-1:0]: payload
  };

  // Simple synchronous FIFO used as ingress buffer
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
  // AXI-Stream handshaking with IP + backpressure control
  //--------------------------------------------------------------------------

  logic s_axis_handshake;
  logic fifo_almost_full;

  // Use CSR-configured threshold to derive FIFO almost-full condition.
  // When FIFO approaches this level we deassert s_axis_tready to slow the IP.
  always_comb begin
    // Truncate CSR threshold to FIFO level width
    logic [$bits(fifo_level)-1:0] thr_trunc;
    thr_trunc = tx_fifo_afull_thr[$bits(fifo_level)-1:0];
    fifo_almost_full = (fifo_level >= thr_trunc);
  end

  assign s_axis_handshake = s_axis_tvalid && s_axis_tready;

  // AXI-Stream backpressure:
  // - Only accept beats when bridge_enable=1
  // - FIFO must not be full
  // - FIFO must not have hit almost-full threshold
  assign s_axis_tready =
    bridge_enable &&
    !fifo_full    &&
    !fifo_almost_full;

  // Push beat into FIFO only on real AXI-Stream handshake
  assign fifo_push = s_axis_handshake;

  //--------------------------------------------------------------------------
  // FSM for serialization: DATA_W => IF_W segments
  //--------------------------------------------------------------------------

  // State of the currently active beat being serialized
  logic [DATA_W-1:0]         cur_data;           // Buffered AXI beat data
  logic [BYTES_PER_BEAT-1:0] cur_keep;           // Corresponding TKEEP
  logic                      cur_last;           // Is this the last beat of the frame?
  logic [BYTE_CNT_W-1:0]     cur_valid_bytes;    // Number of valid bytes in this beat
  logic [SEG_IDX_W-1:0]      cur_num_segs;       // How many segments we will emit from this beat
  logic [BYTE_CNT_W-1:0]     cur_last_seg_bytes; // Valid bytes in the final segment of this beat
  logic [SEG_IDX_W-1:0]      cur_seg_idx;        // Index of current segment (0..cur_num_segs-1)
  logic                      have_beat;          // Indicates we have a beat loaded for serialization

  // Frame/packet tracking across beats (for SOP/EOP and user metadata)
  logic                      in_packet;          // We are currently inside an active frame
  logic [TUSER_W-1:0]        frame_user;         // Latched user for the current frame

  // Registered output (small skid buffer for cl_tx_*)
  logic [IF_W-1:0]           cl_tx_data_q;
  logic [IF_W/8-1:0]         cl_tx_keep_q;
  logic [TUSER_W-1:0]        cl_tx_user_q;
  logic                      cl_tx_valid_q;
  logic                      cl_tx_sop_q;
  logic                      cl_tx_eop_q;

  // Drive the actual outputs from the registered signals
  assign cl_tx_data  = cl_tx_data_q;
  assign cl_tx_keep  = cl_tx_keep_q;
  assign cl_tx_user  = cl_tx_user_q;
  assign cl_tx_valid = cl_tx_valid_q;
  assign cl_tx_sop   = cl_tx_sop_q;
  assign cl_tx_eop   = cl_tx_eop_q;

  // Use a registered pop control to avoid multiple reads in the same cycle
  logic fifo_pop_q;
  
  // Pop from FIFO when the sequential logic decides we need a new beat
  assign fifo_pop = fifo_pop_q; // Controlled inside the always_ff block

  //--------------------------------------------------------------------------
  // Event pulses toward CSR: one-cycle pulses
  //--------------------------------------------------------------------------

  logic ev_err_tkeep_illegal_int;
  logic ev_err_midreset_drop_int;
  logic ev_err_overflow_tx_int;

  assign ev_err_tkeep_illegal = ev_err_tkeep_illegal_int;
  assign ev_err_midreset_drop = ev_err_midreset_drop_int;
  assign ev_err_overflow_tx   = ev_err_overflow_tx_int;

  //--------------------------------------------------------------------------
  // Main sequential logic (state, counters, segment generation)
  //--------------------------------------------------------------------------

  logic bridge_enable_q;
  logic tx_fire;                    // cl_tx_valid && cl_tx_ready handshake
  logic [SEG_IDX_W-1:0] seg_idx_cur;
  int                   seg_byte_base;
  logic [IF_W/8-1:0]    seg_keep_mask;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Reset all state
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
      // Registered copy of enable (used for midreset detection)
      bridge_enable_q <= bridge_enable;

      // Default values each cycle (unless overridden below)
      fifo_pop_q              <= 1'b0;
      fifo_clear              <= 1'b0;
      ev_err_tkeep_illegal_int <= 1'b0;
      ev_err_midreset_drop_int <= 1'b0;
      ev_err_overflow_tx_int   <= 1'b0;

      // Update FIFO level status (zero-extended into 16 bits)
      stat_tx_fifo_level <= {{(16-$bits(fifo_level)){1'b0}}, fifo_level};

      // Detect overflow: push when FIFO is already full (IP violated tready)
      // This should not happen if IP respects AXI-Stream protocol.
      if (fifo_push && fifo_full) begin
        ev_err_overflow_tx_int <= 1'b1;
      end

      // Count bytes accepted from IP on AXI handshake
      if (s_axis_handshake) begin
        stat_tx_bytes <= stat_tx_bytes + popcount_keep(s_axis_tkeep);
        // Strict TKEEP legality check when enabled
        if (strict_tkeep_en && tkeep_illegal_f(s_axis_tkeep, s_axis_tlast)) begin
          ev_err_tkeep_illegal_int <= 1'b1;
        end
      end

      // Stall counter: cycles where we have valid segment but downstream is not ready
      if (bridge_enable && cl_tx_valid_q && !cl_tx_ready) begin
        stat_tx_stall_cycles <= stat_tx_stall_cycles + 1;
      end

      // Mid-reset / drop_on_midreset:
      // If bridge_enable is de-asserted while we are in the middle of a frame,
      // or while data is still pending in FIFO, we drop everything and flag an event.
      if (drop_on_midreset &&
          bridge_enable_q && !bridge_enable &&
          (in_packet || have_beat || !fifo_empty)) begin
        ev_err_midreset_drop_int <= 1'b1;
        // Flush all state and FIFO content
        in_packet     <= 1'b0;
        have_beat     <= 1'b0;
        cl_tx_valid_q <= 1'b0;
        fifo_clear    <= 1'b1;
      end

      // If bridge is disabled, stop issuing cl_tx_valid
      if (!bridge_enable) begin
        cl_tx_valid_q <= 1'b0;
      end else begin
        //========================
        // Stage 1: Load a new beat from FIFO (if needed)
        //========================
        if (!have_beat && !fifo_empty && !cl_tx_valid_q) begin
          // Latch one FIFO entry as the current beat
          cur_data        <= fifo_data;
          cur_keep        <= fifo_keep;
          cur_last        <= fifo_last;
          cur_valid_bytes <= popcount_keep(fifo_keep);

          // Calculate how many segments we need for this beat
          // and how many valid bytes are in the last segment
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

          // For a new frame, latch the TUSER from this beat as frame_user
          if (!in_packet) begin
            frame_user <= fifo_user;
          end

          // Pop this beat from FIFO
          fifo_pop_q <= 1'b1;
        end

        //========================
        // Stage 2: Segment FSM + registered output
        //========================
        tx_fire = cl_tx_valid_q && cl_tx_ready; // Handshake for current segment

        // Frame counter: increment when an EOP segment is successfully transferred
        if (tx_fire && cl_tx_eop_q) begin
          stat_tx_frames <= stat_tx_frames + 1;
        end

        // Update in_packet based on the segment that just fired
        if (tx_fire) begin
          // Enter packet on first segment of a frame
          if (!in_packet && (cur_seg_idx == '0)) begin
            in_packet <= 1'b1;
          end
          // Leave packet on last segment of the last beat
          if (cur_last && (cur_seg_idx == cur_num_segs - 1)) begin
            in_packet <= 1'b0;
          end
        end

        // Segment generation logic:
        if (cl_tx_valid_q && !cl_tx_ready) begin
          // Backpressure: downstream not ready, hold current segment stable
        end else begin
          if (have_beat && (cur_num_segs != 0)) begin
            // We have a beat to serialize and at least one segment to send

            // Select the segment index we are going to output this cycle
            seg_idx_cur = cur_seg_idx;

            // If previous segment handshaked in this cycle, advance to next segment
            if (tx_fire) begin
              if (cur_seg_idx < cur_num_segs - 1) begin
                seg_idx_cur = cur_seg_idx + 1;
              end else begin
                seg_idx_cur = cur_num_segs - 1; // Stay on last segment index
              end
            end

            // Compute byte base offset for this segment
            seg_byte_base = seg_idx_cur * BYTES_PER_SEG;

            // Slice the appropriate IF_W bits from the stored DATA_W beat
            cl_tx_data_q  <= cur_data[(seg_byte_base*8) +: IF_W];

            // Compute KEEP mask for this segment
            if (seg_idx_cur < cur_num_segs - 1) begin
              // All segments except the last are full (all bytes valid)
              seg_keep_mask = {BYTES_PER_SEG{1'b1}};
            end else begin
              // Last segment: may be partial depending on cur_last_seg_bytes
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

            // Metadata: frame_user is held constant across the whole frame
            cl_tx_user_q <= frame_user;

            // SOP: asserted on the very first segment of a frame
            cl_tx_sop_q  <= (!in_packet && (seg_idx_cur == '0));
            // EOP: asserted on the last segment of the last beat of the frame
            cl_tx_eop_q  <= (cur_last && (seg_idx_cur == cur_num_segs - 1));

            // We have a valid segment on the output
            cl_tx_valid_q <= 1'b1;

            // Update current segment index
            cur_seg_idx <= seg_idx_cur;

            // If previous segment handshake was on the last segment, this beat is done
            if (tx_fire && (cur_seg_idx == cur_num_segs - 1)) begin
              have_beat     <= 1'b0;
              cl_tx_valid_q <= 1'b0; // Will be reasserted when next segment/beat is loaded
            end

          end else begin
            // No beat currently loaded or no valid bytes - nothing to send
            cl_tx_valid_q <= 1'b0;
            cl_tx_sop_q   <= 1'b0;
            cl_tx_eop_q   <= 1'b0;
          end
        end // if (!stall)
      end // if bridge_enable
    end // !rst
  end // always_ff

endmodule
