//------------------------------------------------------------------------------
// axi_bridge_ip_tx_ser_beat_fetch
//------------------------------------------------------------------------------
// FIFO -> Beat buffer loader (show-ahead FIFO assumed):
// - When enabled and no active beat, pop one FIFO entry and latch it.
// - Precompute: valid_bytes, num_segs, last_seg_bytes for segment_gen.
// - Exposes beat_* signals stable while beat_valid_o=1.
// - Accepts beat_done_pulse_i OR beat_ready_i (edge-detected) to release current beat.
// - Accepts flush_i to drop state (used for midreset drop).
//------------------------------------------------------------------------------
module axi_bridge_ip_tx_ser_beat_fetch #(
  parameter int DATA_W      = 256,
  parameter int IF_W        = 64,
  parameter int TUSER_W     = 16,
  parameter int FIFO_DEPTH  = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  input  logic                   enable_i,
  input  logic                   flush_i,

  // From segment_gen: "safe to advance / release beat" (can be level or pulse)
  input  logic                   beat_ready_i,

  // FIFO peek + status (show-ahead: fifo_rdata_i is valid when !fifo_empty_i)
  input  logic                   fifo_empty_i,
  input  logic [DATA_W + (DATA_W/8) + TUSER_W + 1 - 1:0] fifo_rdata_i,
  output logic                   fifo_pop_o,

  // From segment_gen: beat completed (1-cycle pulse) - optional/legacy
  input  logic                   beat_done_pulse_i,

  // Beat outputs (stable while beat_valid_o=1)
  output logic                   beat_valid_o,
  output logic [DATA_W-1:0]      beat_data_o,
  output logic [DATA_W/8-1:0]    beat_keep_o,
  output logic [TUSER_W-1:0]     beat_user_o,
  output logic                   beat_last_o,

  // Precomputed fields for segmentation
  output logic [$clog2(((DATA_W/8) + (IF_W/8) - 1) / (IF_W/8) + 1)-1:0] beat_num_segs_o,
  output logic [$clog2((DATA_W/8) + 1)-1:0]                            beat_last_seg_bytes_o
);

  import axi_bridge_ip_tx_pkg::*;

  //--------------------------------------------------------------------------
  // Derived constants
  //--------------------------------------------------------------------------
  localparam int BYTES_PER_BEAT    = DATA_W/8;
  localparam int BYTES_PER_SEG     = IF_W/8;
  localparam int MAX_SEGS_PER_BEAT = (BYTES_PER_BEAT + BYTES_PER_SEG - 1) / BYTES_PER_SEG;

  localparam int BYTE_CNT_W = $clog2(BYTES_PER_BEAT + 1);
  localparam int SEG_CNT_W  = $clog2(MAX_SEGS_PER_BEAT + 1);

  localparam int ENTRY_W    = DATA_W + BYTES_PER_BEAT + TUSER_W + 1;

  // Keep padding for fixed-width package functions
  localparam int KEEP_PAD_W = MAX_KEEP_W - BYTES_PER_BEAT;

  initial begin
    if (DATA_W % 8 != 0) begin
      $error("axi_bridge_ip_tx_ser_beat_fetch: DATA_W must be multiple of 8");
    end
    if (IF_W % 8 != 0) begin
      $error("axi_bridge_ip_tx_ser_beat_fetch: IF_W must be multiple of 8");
    end
    if (IF_W > DATA_W) begin
      $error("axi_bridge_ip_tx_ser_beat_fetch: assumes IF_W <= DATA_W");
    end
    if (BYTES_PER_BEAT > MAX_KEEP_W) begin
      $error("axi_bridge_ip_tx_ser_beat_fetch: BYTES_PER_BEAT (%0d) exceeds pkg MAX_KEEP_W (%0d)",
             BYTES_PER_BEAT, MAX_KEEP_W);
    end
  end

  typedef logic [BYTE_CNT_W-1:0] byte_cnt_t;
  typedef logic [SEG_CNT_W-1:0]  seg_cnt_t;

  //--------------------------------------------------------------------------
  // Unpack FIFO peek data
  //--------------------------------------------------------------------------
  logic [DATA_W-1:0]          fifo_data;
  logic [BYTES_PER_BEAT-1:0]  fifo_keep;
  logic [TUSER_W-1:0]         fifo_user;
  logic                       fifo_last;

  assign fifo_data = fifo_rdata_i[DATA_W-1:0];
  assign fifo_keep = fifo_rdata_i[DATA_W +: BYTES_PER_BEAT];
  assign fifo_user = fifo_rdata_i[DATA_W + BYTES_PER_BEAT +: TUSER_W];
  assign fifo_last = fifo_rdata_i[ENTRY_W-1];

  //--------------------------------------------------------------------------
  // FIFO pop pulse
  //--------------------------------------------------------------------------
  logic fifo_pop_q;
  assign fifo_pop_o = fifo_pop_q;

  //--------------------------------------------------------------------------
  // Edge detect for beat_ready_i (in case it's held high for >1 cycle)
  //--------------------------------------------------------------------------
  logic beat_ready_q;          // previous sampled value
  logic beat_ready_rise;       // 1-cycle pulse on rising edge
  assign beat_ready_rise = beat_ready_i && !beat_ready_q;

  // "release" condition: either legacy pulse or ready rising edge
  logic beat_release_pulse;
  assign beat_release_pulse = beat_done_pulse_i || beat_ready_rise;

  // Can load new beat if:
  // - enabled
  // - FIFO not empty
  // - buffer is empty OR current beat is being released now
  logic can_load;
  assign can_load = enable_i && !fifo_empty_i && (!beat_valid_o || beat_release_pulse);

  //--------------------------------------------------------------------------
  // Main sequential logic
  //--------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fifo_pop_q            <= 1'b0;

      beat_valid_o          <= 1'b0;
      beat_data_o           <= '0;
      beat_keep_o           <= '0;
      beat_user_o           <= '0;
      beat_last_o           <= 1'b0;

      beat_num_segs_o       <= '0;
      beat_last_seg_bytes_o <= '0;

      beat_ready_q          <= 1'b0;

    end else begin
      fifo_pop_q   <= 1'b0;
      beat_ready_q <= beat_ready_i;

      // Flush drops any buffered beat
      if (flush_i) begin
        beat_valid_o <= 1'b0;
      end else begin
        // If beat released and we are not reloading immediately, clear valid
        if (beat_release_pulse && !can_load) begin
          beat_valid_o <= 1'b0;
        end

        if (can_load) begin
          // Prepare padded keep for fixed popcount function
          logic [MAX_KEEP_W-1:0] keep_pad;
          int unsigned           vb_int;
          int unsigned           nsegs_int;
          int unsigned           last_bytes_int;

          keep_pad = { {KEEP_PAD_W{1'b0}}, fifo_keep };

          vb_int = popcount_keep_fixed(keep_pad, BYTES_PER_BEAT);
		  

          if (vb_int == 0) begin
            nsegs_int      = 0;
            last_bytes_int = 0;
          end else begin
            nsegs_int      = (vb_int + BYTES_PER_SEG - 1) / BYTES_PER_SEG;
            last_bytes_int = vb_int - (nsegs_int - 1) * BYTES_PER_SEG;
          end

          // Consume FIFO entry (show-ahead FIFO assumed)
          fifo_pop_q <= 1'b1;

          // Latch beat fields
          beat_data_o <= fifo_data;
          beat_keep_o <= fifo_keep;
          beat_user_o <= fifo_user;
          beat_last_o <= fifo_last;

          beat_num_segs_o       <= seg_cnt_t'(nsegs_int);
          beat_last_seg_bytes_o <= byte_cnt_t'(last_bytes_int);

          // Valid only if there is payload
          beat_valid_o <= (vb_int != 0);
        end
      end
    end
  end

endmodule
