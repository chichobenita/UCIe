//------------------------------------------------------------------------------
// axi_bridge_ip_tx_ingress
//------------------------------------------------------------------------------
// AXI-Stream slave-side ingress:
// - Generates s_axis_tready based on bridge_enable + FIFO full + almost-full threshold
// - Pushes accepted beats into FIFO as {tlast, tuser, tkeep, tdata}
// - Performs optional strict TKEEP legality check
// - Generates event pulses: tkeep_illegal, overflow_tx
// - Provides byte increment info for stats
//------------------------------------------------------------------------------
module axi_bridge_ip_tx_ingress #(
  parameter int DATA_W      = 256,
  parameter int TUSER_W     = 16,
  parameter int FIFO_DEPTH  = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  //================= AXI-Stream from IP =================
  input  logic [DATA_W-1:0]      s_axis_tdata,
  input  logic [DATA_W/8-1:0]    s_axis_tkeep,
  input  logic [TUSER_W-1:0]     s_axis_tuser,
  input  logic                   s_axis_tvalid,
  input  logic                   s_axis_tlast,
  output logic                   s_axis_tready,

  //================= CSR =================
  input  logic                   bridge_enable,
  input  logic                   strict_tkeep_en,
  input  logic [7:0]             tx_fifo_afull_thr,

  //================= FIFO status =================
  input  logic                   fifo_full,
  input  logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level,

  //================= FIFO push interface =================
  output logic                   fifo_push,
  output logic [DATA_W + (DATA_W/8) + TUSER_W + 1 - 1:0] fifo_wdata,

  //================= To stats =================
  output logic [31:0]            bytes_inc,
  output logic                   bytes_inc_valid,

  //================= Events (1-cycle pulses) =================
  output logic                   ev_err_tkeep_illegal_pulse,
  output logic                   ev_err_overflow_tx_pulse
);

  import axi_bridge_ip_tx_pkg::*;

  //--------------------------------------------------------------------------
  // Local parameters
  //--------------------------------------------------------------------------
  localparam int BYTES_PER_BEAT = DATA_W/8;
  localparam int ENTRY_W        = DATA_W + BYTES_PER_BEAT + TUSER_W + 1;

  // Padding width for fixed-size keep functions in the package
  localparam int KEEP_PAD_W = MAX_KEEP_W - BYTES_PER_BEAT;

  // Synthesis/sim-time sanity
  initial begin
    if (DATA_W % 8 != 0) begin
      $error("axi_bridge_ip_tx_ingress: DATA_W must be multiple of 8");
    end
    if (BYTES_PER_BEAT > MAX_KEEP_W) begin
      $error("axi_bridge_ip_tx_ingress: BYTES_PER_BEAT (%0d) exceeds pkg MAX_KEEP_W (%0d)",
             BYTES_PER_BEAT, MAX_KEEP_W);
    end
  end

  //--------------------------------------------------------------------------
  // Backpressure (almost-full) logic
  //--------------------------------------------------------------------------
  logic fifo_almost_full;
  logic s_axis_handshake;

  always_comb begin
    logic [$bits(fifo_level)-1:0] thr_trunc;
    thr_trunc        = tx_fifo_afull_thr[$bits(fifo_level)-1:0];
    fifo_almost_full = (fifo_level >= thr_trunc);
  end

  // Ready only when enabled and FIFO has headroom
  assign s_axis_tready =
    bridge_enable &&
    !fifo_full    &&
    !fifo_almost_full;

  assign s_axis_handshake = s_axis_tvalid && s_axis_tready;

  // Push beat into FIFO only on handshake
  assign fifo_push = s_axis_handshake;

  // Pack FIFO entry: {tlast, tuser, tkeep, tdata}
  assign fifo_wdata = {
    s_axis_tlast,
    s_axis_tuser,
    s_axis_tkeep,
    s_axis_tdata
  };

  //--------------------------------------------------------------------------
  // Byte counting (to stats) using fixed-width keep helper
  //--------------------------------------------------------------------------
  logic [MAX_KEEP_W-1:0] keep_pad;

  // Pad keep into MAX_KEEP_W (LSBs are the real keep bits)
  assign keep_pad = { {KEEP_PAD_W{1'b0}}, s_axis_tkeep };

  // Provide per-beat byte increment; stats module will accumulate on valid
  assign bytes_inc       = popcount_keep_fixed(keep_pad, BYTES_PER_BEAT);
  
  
  assign bytes_inc_valid = s_axis_handshake;

  //--------------------------------------------------------------------------
  // Event pulses
  //--------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ev_err_tkeep_illegal_pulse <= 1'b0;
      ev_err_overflow_tx_pulse   <= 1'b0;
    end else begin
      // default: pulses are 0 unless fired this cycle
      ev_err_tkeep_illegal_pulse <= 1'b0;
      ev_err_overflow_tx_pulse   <= 1'b0;

      // Strict TKEEP legality on accepted beats
      if (s_axis_handshake && strict_tkeep_en &&
          tkeep_illegal_f_fixed(keep_pad, BYTES_PER_BEAT, s_axis_tlast)) begin
        ev_err_tkeep_illegal_pulse <= 1'b1;
      end

      // Overflow detection (typically unreachable because fifo_push implies tready,
      // but kept for compatibility with original intent)
      if (fifo_push && fifo_full) begin
        ev_err_overflow_tx_pulse <= 1'b1;
      end
    end
  end

endmodule
