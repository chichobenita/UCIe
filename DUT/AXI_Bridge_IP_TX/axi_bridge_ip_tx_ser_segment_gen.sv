module axi_bridge_ip_tx_ser_segment_gen #(
  parameter int DATA_W  = 256,
  parameter int IF_W    = 64,
  parameter int TUSER_W = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  input  logic                   enable_i,
  input  logic                   flush_i,

  // Beat input from beat_fetch (stable while beat_valid_i=1)
  input  logic                   beat_valid_i,
  input  logic [DATA_W-1:0]      beat_data_i,
  input  logic [DATA_W/8-1:0]    beat_keep_i,              // (לא חובה במימוש keep הנוכחי)
  input  logic [TUSER_W-1:0]     beat_user_i,
  input  logic                   beat_last_i,

  input  logic [$clog2(((DATA_W/8)+(IF_W/8)-1)/(IF_W/8) + 1)-1:0] beat_num_segs_i,
  input  logic [$clog2((DATA_W/8)+1)-1:0]                        beat_last_seg_bytes_i,

  // Downstream
  input  logic                   cl_tx_ready_i,

  output logic [IF_W-1:0]        cl_tx_data_o,
  output logic [IF_W/8-1:0]      cl_tx_keep_o,
  output logic [TUSER_W-1:0]     cl_tx_user_o,
  output logic                   cl_tx_valid_o,
  output logic                   cl_tx_sop_o,
  output logic                   cl_tx_eop_o,

  // Handshakes to others
  output logic                   beat_done_pulse_o,
  output logic                   frame_done_pulse_o,
  output logic                   stall_cycle_en_o,

  output logic                   beat_ready_o,
  output logic                   in_packet_o
);

  // --------------------------------------------------------------------------
  // Derived constants / types
  // --------------------------------------------------------------------------
  localparam int BYTES_PER_SEG     = IF_W/8;
  localparam int BYTES_PER_BEAT    = DATA_W/8;
  localparam int MAX_SEGS_PER_BEAT = (BYTES_PER_BEAT + BYTES_PER_SEG - 1) / BYTES_PER_SEG;
  localparam int SEG_CNT_W         = $clog2(MAX_SEGS_PER_BEAT + 1);

  typedef logic [SEG_CNT_W-1:0] seg_cnt_t;

  // --------------------------------------------------------------------------
  // Internal state
  // --------------------------------------------------------------------------
  logic [DATA_W-1:0]   data_shreg_q;        // Shift register holding remaining beat data
  seg_cnt_t            seg_idx_q;           // Current segment index (0..num_segs-1)
  seg_cnt_t            num_segs_q;          // Latched beat_num_segs
  logic                beat_last_q;         // Latched beat_last flag
  logic [TUSER_W-1:0]  beat_user_q;         // Latched beat_user
  logic [$bits(beat_last_seg_bytes_i)-1:0] last_seg_bytes_q; // Latched last seg bytes

  logic                active_q;            // We have a beat loaded in shreg
  logic                in_packet_q;         // Frame tracking
  logic [TUSER_W-1:0]  frame_user_q;        // Latched user for current frame

  // Output regs
  logic [IF_W-1:0]         data_q;
  logic [BYTES_PER_SEG-1:0] keep_q;
  logic [TUSER_W-1:0]      user_q;
  logic                    valid_q, sop_q, eop_q;

  // --------------------------------------------------------------------------
  // Helper: last index + keep mask for last segment
  // --------------------------------------------------------------------------
  function automatic seg_cnt_t last_seg_idx(input seg_cnt_t nsegs);
    if (nsegs == '0) last_seg_idx = '0;
    else            last_seg_idx = nsegs - 1;
  endfunction

  function automatic logic [BYTES_PER_SEG-1:0] make_keep_mask(input int unsigned nbytes);
    logic [BYTES_PER_SEG-1:0] m;
    m = '0;
    for (int b = 0; b < BYTES_PER_SEG; b++) begin
      if (b < nbytes) m[b] = 1'b1;
    end
    return m;
  endfunction

  // --------------------------------------------------------------------------
  // Combinational handshakes / pulses
  // --------------------------------------------------------------------------
  logic     fire_w;
  seg_cnt_t last_idx_w;

  assign fire_w     = enable_i && valid_q && cl_tx_ready_i;
  assign last_idx_w = last_seg_idx(num_segs_q);

  // Pulses that beat_fetch needs *in the same cycle*
  assign beat_done_pulse_o  = fire_w && (seg_idx_q == last_idx_w);
  assign frame_done_pulse_o = fire_w && eop_q;

  // Stall indicator (for stats)
  assign stall_cycle_en_o = enable_i && valid_q && !cl_tx_ready_i;

  // Ready to accept a new beat (simple policy)
  assign beat_ready_o = enable_i && !flush_i && !active_q;

  // --------------------------------------------------------------------------
  // Drive outputs
  // --------------------------------------------------------------------------
  assign cl_tx_data_o  = data_q;
  assign cl_tx_keep_o  = keep_q;
  assign cl_tx_user_o  = user_q;
  assign cl_tx_valid_o = valid_q;
  assign cl_tx_sop_o   = sop_q;
  assign cl_tx_eop_o   = eop_q;

  assign in_packet_o   = in_packet_q;



	function automatic logic [IF_W-1:0] lsb_after_shift(input logic [DATA_W-1:0] x);
	  logic [DATA_W-1:0] y;
	  y = (x >> IF_W);
	  return y[IF_W-1:0];
	endfunction



  // --------------------------------------------------------------------------
  // Main sequential logic
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      data_shreg_q      <= '0;
      seg_idx_q         <= '0;
      num_segs_q        <= '0;
      beat_last_q       <= 1'b0;
      beat_user_q       <= '0;
      last_seg_bytes_q  <= '0;

      active_q          <= 1'b0;
      in_packet_q       <= 1'b0;
      frame_user_q      <= '0;

      data_q            <= '0;
      keep_q            <= '0;
      user_q            <= '0;
      valid_q           <= 1'b0;
      sop_q             <= 1'b0;
      eop_q             <= 1'b0;

    end else begin
      // Flush drops all local state
      if (flush_i) begin
        active_q     <= 1'b0;
        valid_q      <= 1'b0;
        seg_idx_q    <= '0;
        sop_q        <= 1'b0;
        eop_q        <= 1'b0;
        in_packet_q  <= 1'b0;
      end else if (!enable_i) begin
        // Keep state, but stop driving valid (like a "pause")
        valid_q <= 1'b0;
        sop_q   <= 1'b0;
        eop_q   <= 1'b0;

      end else begin
        // ------------------------------------------------------------
        // 1) If a segment is accepted (fire), advance state
        // ------------------------------------------------------------
        if (fire_w) begin
          // Update in_packet based on SOP/EOP of the segment that just fired
          if (sop_q) in_packet_q <= 1'b1;
          if (eop_q) in_packet_q <= 1'b0; // EOP wins if both

          if (seg_idx_q == last_idx_w) begin
            // Beat completed
            active_q  <= 1'b0;
            valid_q   <= 1'b0;
            seg_idx_q <= '0;
            sop_q     <= 1'b0;
            eop_q     <= 1'b0;
          end else begin
            // Advance to next segment: shift the beat data
            data_shreg_q <= (data_shreg_q >> IF_W);
            seg_idx_q    <= seg_idx_q + 1;

            // Drive next segment outputs (from shifted data)
            data_q       <= lsb_after_shift(data_shreg_q);

            // KEEP: full for non-last, partial for last
            if ((seg_idx_q + 1) < last_idx_w) begin
              keep_q <= {BYTES_PER_SEG{1'b1}};
            end else begin
              if (last_seg_bytes_q == 0 || last_seg_bytes_q == BYTES_PER_SEG) begin
                keep_q <= {BYTES_PER_SEG{1'b1}};
              end else begin
                keep_q <= make_keep_mask(int'(last_seg_bytes_q));
              end
            end

            // SOP is only on first segment of a frame (cannot happen here because seg_idx>0)
            sop_q  <= 1'b0;

            // EOP when last beat and this is the last segment
            eop_q  <= beat_last_q && ((seg_idx_q + 1) == last_idx_w);

            // USER is constant for the whole frame
            user_q  <= frame_user_q;
            valid_q <= 1'b1;
          end
        end

        // ------------------------------------------------------------
        // 2) If not firing: hold if valid (stall), else (re)drive/load
        // ------------------------------------------------------------
        else begin
          if (valid_q) begin
            // Stall: hold outputs stable (do nothing)
          end else begin
            // valid_q==0: either resume current beat, or load a new one

            if (active_q) begin
              // Resume driving current segment from current shreg (LSBs)
              data_q <= data_shreg_q[IF_W-1:0];

              // KEEP for current seg_idx
              if (seg_idx_q < last_idx_w) begin
                keep_q <= {BYTES_PER_SEG{1'b1}};
              end else begin
                if (last_seg_bytes_q == 0 || last_seg_bytes_q == BYTES_PER_SEG) begin
                  keep_q <= {BYTES_PER_SEG{1'b1}};
                end else begin
                  keep_q <= make_keep_mask(int'(last_seg_bytes_q));
                end
              end

              sop_q   <= (!in_packet_q && (seg_idx_q == '0));
              eop_q   <= beat_last_q && (seg_idx_q == last_idx_w);
              user_q  <= frame_user_q;
              valid_q <= 1'b1;

            end else if (beat_valid_i && (beat_num_segs_i != '0)) begin
              // Load a new beat from beat_fetch and drive its first segment
              active_q         <= 1'b1;
              seg_idx_q        <= '0;

              data_shreg_q     <= beat_data_i;
              num_segs_q       <= seg_cnt_t'(beat_num_segs_i);
              beat_last_q      <= beat_last_i;
              beat_user_q      <= beat_user_i;
              last_seg_bytes_q <= beat_last_seg_bytes_i;

              // Latch frame_user at start of a new frame
              if (!in_packet_q) begin
                frame_user_q <= beat_user_i;
                user_q       <= beat_user_i;
              end else begin
                user_q       <= frame_user_q;
              end

              data_q  <= beat_data_i[IF_W-1:0];

              // KEEP for first segment
              if (seg_cnt_t'(beat_num_segs_i) == '0) begin
                keep_q <= '0;
              end else if (last_seg_idx(seg_cnt_t'(beat_num_segs_i)) != '0) begin
                // More than one segment => first is full
                keep_q <= {BYTES_PER_SEG{1'b1}};
              end else begin
                // Only one segment => it's also last segment
                if (beat_last_seg_bytes_i == 0 || beat_last_seg_bytes_i == BYTES_PER_SEG) begin
                  keep_q <= {BYTES_PER_SEG{1'b1}};
                end else begin
                  keep_q <= make_keep_mask(int'(beat_last_seg_bytes_i));
                end
              end

              sop_q   <= (!in_packet_q); // idx is 0 here
              eop_q   <= beat_last_i && (last_seg_idx(seg_cnt_t'(beat_num_segs_i)) == '0);
              valid_q <= 1'b1;

            end else begin
              // Nothing to send
              valid_q <= 1'b0;
              sop_q   <= 1'b0;
              eop_q   <= 1'b0;
            end
          end
        end
      end
    end
  end

endmodule
