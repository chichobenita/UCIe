// AXI Bridge top-level (sits in core_clk domain)
module axi_bridge #(
  parameter int DATA_W  = 256,   // AXI-Stream data width from/to IP
  parameter int IF_W    = 64,    // Client-IF segment width toward Protocol
  parameter int TUSER_W = 16     // Metadata width (VC/DST/etc.)
)(
  // Core clock / reset
  input  logic               core_clk,
  input  logic               core_rst_n,

  // ================= AXI-Stream from IP (TX direction: IP -> Bridge) =================
  input  logic [DATA_W-1:0]  s_axis_tdata,
  input  logic [DATA_W/8-1:0] s_axis_tkeep,   // NEW: byte-valid mask
  input  logic [TUSER_W-1:0] s_axis_tuser,
  input  logic               s_axis_tvalid,
  input  logic               s_axis_tlast,
  output logic               s_axis_tready,

  // ================= AXI-Stream to IP (RX direction: Bridge -> IP) ==================
  output logic [DATA_W-1:0]  m_axis_tdata,
  output logic [DATA_W/8-1:0] m_axis_tkeep,   // for full support later
  output logic [TUSER_W-1:0] m_axis_tuser,
  output logic               m_axis_tvalid,
  output logic               m_axis_tlast,
  input  logic               m_axis_tready,

  // ================= Client-IF toward Protocol (TX: Bridge -> Protocol) ==============
  output logic [IF_W-1:0]    cl_tx_data,
  output logic [IF_W/8-1:0]  cl_tx_keep,      // NEW: per-byte valid mask for segment
  output logic               cl_tx_valid,
  input  logic               cl_tx_ready,
  output logic               cl_tx_sop,
  output logic               cl_tx_eop,
  output logic [TUSER_W-1:0] cl_tx_user,

  // ================= Client-IF from Protocol (RX: Protocol -> Bridge) ===============
  input  logic [IF_W-1:0]    cl_rx_data,
  input  logic [IF_W/8-1:0]  cl_rx_keep,      // for later RX path
  input  logic               cl_rx_valid,
  output logic               cl_rx_ready,
  input  logic               cl_rx_sop,
  input  logic               cl_rx_eop,
  input  logic [TUSER_W-1:0] cl_rx_user,

  // ===================== CSR Control (core_clk domain) ==============================
  input  logic               bridge_enable,       // global enable for data path
  input  logic               strict_tkeep_en,     // enforce legal tkeep patterns
  input  logic [7:0]         tx_fifo_afull_thr,   // almost-full threshold (TX ingress)
  input  logic [7:0]         rx_fifo_afull_thr,   // reserved for RX side (later)
  input  logic               drop_on_midreset,    // policy when reset in middle of pkt

  // ===================== CSR Status / Telemetry (core_clk domain) ===================
  output logic [31:0]        stat_tx_frames,
  output logic [31:0]        stat_tx_bytes,
  output logic [15:0]        stat_tx_fifo_level,
  output logic [31:0]        stat_tx_stall_cycles,
  output logic               ev_err_tkeep_illegal,
  output logic               ev_err_midreset_drop,
  output logic               ev_err_overflow_tx

);

  // ===========================================
  // Ingress path: IP -> Bridge -> Client-IF
  // ===========================================

  axi_bridge_ip_rx #(
    .DATA_W   (DATA_W),
    .IF_W     (IF_W),
    .TUSER_W  (TUSER_W)
  ) u_axi_bridge_ip_rx (
    .clk_i            (core_clk),
    .rst_ni           (core_rst_n),

    // AXI-Stream from IP
    .s_axis_tdata     (s_axis_tdata),
    .s_axis_tkeep     (s_axis_tkeep),
    .s_axis_tuser     (s_axis_tuser),
    .s_axis_tvalid    (s_axis_tvalid),
    .s_axis_tlast     (s_axis_tlast),
    .s_axis_tready    (s_axis_tready),

    // Client-IF toward Protocol (TX)
    .cl_tx_data       (cl_tx_data),
    .cl_tx_keep       (cl_tx_keep),
    .cl_tx_user       (cl_tx_user),
    .cl_tx_valid      (cl_tx_valid),
    .cl_tx_sop        (cl_tx_sop),
    .cl_tx_eop        (cl_tx_eop),
    .cl_tx_ready      (cl_tx_ready),

    // CSR control
    .bridge_enable    (bridge_enable),
    .strict_tkeep_en  (strict_tkeep_en),
    .tx_fifo_afull_thr(tx_fifo_afull_thr),
    .drop_on_midreset (drop_on_midreset),

    // CSR status / telemetry
    .stat_tx_frames       (stat_tx_frames),
    .stat_tx_bytes        (stat_tx_bytes),
    .stat_tx_fifo_level   (stat_tx_fifo_level),
    .stat_tx_stall_cycles (stat_tx_stall_cycles),
    .ev_err_tkeep_illegal (ev_err_tkeep_illegal),
    .ev_err_midreset_drop (ev_err_midreset_drop),
    .ev_err_overflow_tx   (ev_err_overflow_tx)
  );

  // ===========================================
  // Egress path: Client-IF -> Bridge -> IP
  // (TODO: ימומש בשלב מאוחר יותר)
  // ===========================================
  // כרגע אפשר להשאיר כ-stub / אפסים כדי שהקוד יתקמפל,
  // עד שנבנה את מודול ה-RX מה-Protocol ל-AXI-Stream.

  assign m_axis_tdata  = '0;
  assign m_axis_tkeep  = '0;
  assign m_axis_tuser  = '0;
  assign m_axis_tvalid = 1'b0;
  assign m_axis_tlast  = 1'b0;
  assign cl_rx_ready   = 1'b0;

endmodule
