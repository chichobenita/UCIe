// ============================================================================
// UCIe D2D Endpoint — TOP (SystemVerilog skeleton)
// MVP with forward-looking hooks (sideband, CSR, LTSSM, CDC options)
// ----------------------------------------------------------------------------
// Notes:
// - This file wires submodules only; internal logic is inside sub-blocks.
// - Keep interfaces stable; extend via parameters/generate blocks.
// - All names are consistent with our previous spec.
// ============================================================================

`timescale 1ns/1ps

module ucie_d2d_endpoint_top #(
  // ------------------ Public Parameters ------------------
  parameter int DATA_W       = 256,   // AXI-Stream data width (to/from IP)
  parameter int VC_W         = 2,
  parameter int DST_W        = 4,
  parameter int FLG_W        = 4,
  parameter int TUSER_W      = VC_W + DST_W + FLG_W, // IP-facing tuser

  parameter int IF_W         = DATA_W, // internal segment width (Bridge↔Protocol)
  parameter int HEADER_W     = 64,
  parameter int PAYLOAD_W    = 256,    // payload per flit (can be <= DATA_W)
  parameter int CRC_W        = 32,
  parameter int FLIT_W       = HEADER_W + PAYLOAD_W + CRC_W,

  parameter int NUM_VC       = (1 << VC_W),

  // Link/PHY mapping
  parameter int PHY_W        = FLIT_W, // for MVP choose PHY_W == FLIT_W
  parameter int LANES        = 1,      // future: multi-lane
  parameter int LANE_W       = (LANES==0) ? PHY_W : (PHY_W / (LANES>0?LANES:1)),

  // Feature toggles (forward-looking hooks)
  parameter bit USE_SIDEBAND = 1'b0,   // add sideband bus
  parameter bit USE_PHY_IF   = 1'b1,   // use a PHY-IF wrapper (vs direct to Framing)
  parameter bit ASYNC_CDC    = 1'b0    // CDC inside PHY-IF
) (
  // ------------------ Clocks & Resets ------------------
  input  logic                core_clk,
  input  logic                core_rst_n,

  input  logic                link_clk,
  input  logic                link_rst_n,

  input  logic                axil_aclk,
  input  logic                axil_aresetn,

  // ------------------ AXI-Stream (to/from IP) ------------------
  // TX from IP → into endpoint
  input  logic [DATA_W-1:0]   s_axis_tdata,
  input  logic [TUSER_W-1:0]  s_axis_tuser,
  input  logic                s_axis_tvalid,
  input  logic                s_axis_tlast,
  output logic                s_axis_tready,

  // RX to IP ← out of endpoint
  output logic [DATA_W-1:0]   m_axis_tdata,
  output logic [TUSER_W-1:0]  m_axis_tuser,
  output logic                m_axis_tvalid,
  output logic                m_axis_tlast,
  input  logic                m_axis_tready,

  // ------------------ AXI-Lite (CSR slave) ------------------
  input  logic [15:0]         s_axil_awaddr,
  input  logic                s_axil_awvalid,
  output logic                s_axil_awready,
  input  logic [31:0]         s_axil_wdata,
  input  logic [3:0]          s_axil_wstrb,
  input  logic                s_axil_wvalid,
  output logic                s_axil_wready,
  output logic [1:0]          s_axil_bresp,
  output logic                s_axil_bvalid,
  input  logic                s_axil_bready,
  input  logic [15:0]         s_axil_araddr,
  input  logic                s_axil_arvalid,
  output logic                s_axil_arready,
  output logic [31:0]         s_axil_rdata,
  output logic [1:0]          s_axil_rresp,
  output logic                s_axil_rvalid,
  input  logic                s_axil_rready,
  output logic                irq,

  // ------------------ Raw D2D PHY (pins outward) ------------------
  output logic [PHY_W-1:0]    phy_tx_data,
  output logic                phy_tx_valid,
  input  logic                phy_tx_ready,
  input  logic [PHY_W-1:0]    phy_rx_data,
  input  logic                phy_rx_valid,
  output logic                phy_rx_ready,

  // Optional sideband (narrow mgmt path)
  output logic [31:0]         sb_tx_data,
  output logic                sb_tx_valid,
  input  logic                sb_tx_ready,
  input  logic [31:0]         sb_rx_data,
  input  logic                sb_rx_valid,
  output logic                sb_rx_ready,

  // Status/debug outward
  output logic                link_up,
  output logic [LANES-1:0]    lane_up
);

  // ------------------ Localparams / Types ------------------
  localparam int TUSER_CL_W = TUSER_W; // internal user width (can extend later)

  // ------------------ Internal Wires ------------------
  // AXI Bridge ↔ Protocol (Client-IF segments)
  logic [IF_W-1:0]            cl_tx_data;
  logic                       cl_tx_valid, cl_tx_ready;
  logic                       cl_tx_sop,   cl_tx_eop;
  logic [TUSER_CL_W-1:0]      cl_tx_user;

  logic [IF_W-1:0]            cl_rx_data;
  logic                       cl_rx_valid, cl_rx_ready;
  logic                       cl_rx_sop,   cl_rx_eop;
  logic [TUSER_CL_W-1:0]      cl_rx_user;

  // Protocol ↔ Scheduler (per-VC head-of-line)
  logic [NUM_VC-1:0]                          vcq_valid;
  logic [NUM_VC-1:0][FLIT_W-1:0]              vcq_flit;
  logic [NUM_VC-1:0]                          vcq_is_ctrl;
  logic [NUM_VC-1:0]                          vc_can_send;   // credit/eligibility
  logic [NUM_VC-1:0]                          vcq_deq;
  logic [NUM_VC-1:0]                          dec_credit_vc; // DATA only
  logic [VC_W-1:0]                            grant_vc_id;
  logic                                       grant_is_ctrl;

  // Scheduler → Proto-IF TX (single flit stream)
  logic [FLIT_W-1:0]                          sched_tx_flit;
  logic                                       sched_tx_valid;
  logic                                       sched_tx_ready;

  // Proto-IF ↔ Framing (flit-wide)
  logic [FLIT_W-1:0]                          pl_tx_flit;
  logic                                       pl_tx_valid, pl_tx_ready;
  logic [FLIT_W-1:0]                          pl_rx_flit;
  logic                                       pl_rx_valid, pl_rx_ready;

  // Framing ↔ PHY-IF (beat-wide)
  logic [PHY_W-1:0]                           dl_tx_data;
  logic                                       dl_tx_valid, dl_tx_ready;
  logic [PHY_W-1:0]                           dl_rx_data;
  logic                                       dl_rx_valid, dl_rx_ready;

  // Bring-up / alignment hooks
  logic [3:0]                                  link_state;   // LTSSM state
  logic                                        align_done;
  logic                                        force_retrain;

  // CSR observable metrics (subset)
  logic [31:0]                                 stat_tx_flits, stat_rx_flits;
  logic [31:0]                                 stat_crc_errs, stat_naks, stat_replays;
  logic [31:0]                                 stat_credit_stall;

  // ------------------ Submodule Instances ------------------

  // AXI Bridge: AXI-Stream ↔ Client-IF (segments)
  axi_bridge #(
    .DATA_W     (DATA_W),
    .IF_W       (IF_W),
    .TUSER_W    (TUSER_CL_W)
  ) u_axi_bridge (
    .clk_i      (core_clk),
    .rst_ni     (core_rst_n),

    // AXI-Stream IP side
    .s_axis_tdata   (s_axis_tdata),
    .s_axis_tuser   (s_axis_tuser),
    .s_axis_tvalid  (s_axis_tvalid),
    .s_axis_tlast   (s_axis_tlast),
    .s_axis_tready  (s_axis_tready),

    .m_axis_tdata   (m_axis_tdata),
    .m_axis_tuser   (m_axis_tuser),
    .m_axis_tvalid  (m_axis_tvalid),
    .m_axis_tlast   (m_axis_tlast),
    .m_axis_tready  (m_axis_tready),

    // Client-IF toward Protocol
    .cl_tx_data     (cl_tx_data),
    .cl_tx_valid    (cl_tx_valid),
    .cl_tx_ready    (cl_tx_ready),
    .cl_tx_sop      (cl_tx_sop),
    .cl_tx_eop      (cl_tx_eop),
    .cl_tx_user     (cl_tx_user),

    .cl_rx_data     (cl_rx_data),
    .cl_rx_valid    (cl_rx_valid),
    .cl_rx_ready    (cl_rx_ready),
    .cl_rx_sop      (cl_rx_sop),
    .cl_rx_eop      (cl_rx_eop),
    .cl_rx_user     (cl_rx_user)
  );

  // Protocol Core: segments ↔ per-VC queues, deframe/frame, CRC, ACK/NAK, credits
  d2d_controller_protocol #(
    .IF_W       (IF_W),
    .FLIT_W     (FLIT_W),
    .HEADER_W   (HEADER_W),
    .PAYLOAD_W  (PAYLOAD_W),
    .CRC_W      (CRC_W),
    .VC_W       (VC_W),
    .NUM_VC     (NUM_VC)
  ) u_protocol (
    .clk_i          (link_clk),
    .rst_ni         (link_rst_n),

    // Client-IF northbound
    .cl_tx_data     (cl_tx_data),
    .cl_tx_valid    (cl_tx_valid),
    .cl_tx_ready    (cl_tx_ready),
    .cl_tx_sop      (cl_tx_sop),
    .cl_tx_eop      (cl_tx_eop),
    .cl_tx_user     (cl_tx_user),

    .cl_rx_data     (cl_rx_data),
    .cl_rx_valid    (cl_rx_valid),
    .cl_rx_ready    (cl_rx_ready),
    .cl_rx_sop      (cl_rx_sop),
    .cl_rx_eop      (cl_rx_eop),
    .cl_rx_user     (cl_rx_user),

    // To/From VC Scheduler (TX path)
    .vcq_valid      (vcq_valid),
    .vcq_flit       (vcq_flit),
    .vcq_is_ctrl    (vcq_is_ctrl),
    .vc_can_send    (vc_can_send),
    .vcq_deq        (vcq_deq),
    .dec_credit_vc  (dec_credit_vc),

    // RX path from Framing (flits)
    .pl_rx_flit     (pl_rx_flit),
    .pl_rx_valid    (pl_rx_valid),
    .pl_rx_ready    (pl_rx_ready),

    // Bring-up / status
    .align_done_i   (align_done),
    .link_state_o   (link_state),
    .link_up_o      (link_up),

    // Sideband (optional pass-through into protocol for mgmt)
    .sb_tx_data_i   (USE_SIDEBAND ? sb_rx_data  : '0),
    .sb_tx_valid_i  (USE_SIDEBAND ? sb_rx_valid : 1'b0),
    .sb_tx_ready_o  (/* unused */),
    .sb_rx_data_o   (/* unused */),
    .sb_rx_valid_o  (),
    .sb_rx_ready_i  (1'b1),

    // Stats (to CSR)
    .stat_tx_flits  (stat_tx_flits),
    .stat_rx_flits  (stat_rx_flits),
    .stat_crc_errs  (stat_crc_errs),
    .stat_naks      (stat_naks),
    .stat_replays   (stat_replays),
    .stat_credit_stall (stat_credit_stall)
  );

  // VC Scheduler: select next flit among VCs
  vc_scheduler_tx #(
    .NUM_VC       (NUM_VC),
    .VC_W         (VC_W),
    .FLIT_W       (FLIT_W),
    .CTRL_FIRST   (1),     // MVP: control flits first
    .POLICY       (0)      // 0=RR, 1=WRR, 2=DRR (example encoding)
  ) u_vc_sched (
    .clk_i          (link_clk),
    .rst_ni         (link_rst_n),

    .vcq_valid_i    (vcq_valid),
    .vcq_flit_i     (vcq_flit),
    .vcq_is_ctrl_i  (vcq_is_ctrl),
    .vc_can_send_i  (vc_can_send),

    .sched_tx_flit_o(sched_tx_flit),
    .sched_tx_valid_o(sched_tx_valid),
    .sched_tx_ready_i(sched_tx_ready),

    .vcq_deq_o      (vcq_deq),
    .dec_credit_vc_o(dec_credit_vc),

    .grant_vc_id_o  (grant_vc_id),
    .grant_is_ctrl_o(grant_is_ctrl)
  );

  // Proto-IF: thin flit-level handshake bridge to Framing
  proto_if #(
    .FLIT_W     (FLIT_W)
  ) u_proto_if (
    .clk_i          (link_clk),
    .rst_ni         (link_rst_n),

    // From scheduler (TX)
    .tx_flit_i      (sched_tx_flit),
    .tx_valid_i     (sched_tx_valid),
    .tx_ready_o     (sched_tx_ready),

    // To Framing (TX)
    .pl_tx_flit_o   (pl_tx_flit),
    .pl_tx_valid_o  (pl_tx_valid),
    .pl_tx_ready_i  (pl_tx_ready),

    // From Framing (RX)
    .pl_rx_flit_i   (pl_rx_flit),
    .pl_rx_valid_i  (pl_rx_valid),
    .pl_rx_ready_o  (pl_rx_ready),

    // To Protocol (RX)
    .rx_flit_o      (/* alias to protocol */), // unused: protocol consumes pl_rx_* directly
    .rx_valid_o     (),
    .rx_ready_i     (1'b1)
  );

  // Framing Layer: flits ↔ PHY beats (+ alignment/deskew hooks)
  d2d_framing_layer #(
    .FLIT_W     (FLIT_W),
    .PHY_W      (PHY_W),
    .LANES      (LANES),
    .LANE_W     (LANE_W),
    .ALIGN_EN   (1'b0) // MVP: off; enable later
  ) u_framing (
    .clk_i          (link_clk),
    .rst_ni         (link_rst_n),

    // From Proto-IF (TX)
    .pl_tx_flit_i   (pl_tx_flit),
    .pl_tx_valid_i  (pl_tx_valid),
    .pl_tx_ready_o  (pl_tx_ready),

    // To Proto-IF (RX)
    .pl_rx_flit_o   (pl_rx_flit),
    .pl_rx_valid_o  (pl_rx_valid),
    .pl_rx_ready_i  (pl_rx_ready),

    // Downlink beats (to/from PHY-IF)
    .dl_tx_data_o   (dl_tx_data),
    .dl_tx_valid_o  (dl_tx_valid),
    .dl_tx_ready_i  (dl_tx_ready),
    .dl_rx_data_i   (dl_rx_data),
    .dl_rx_valid_i  (dl_rx_valid),
    .dl_rx_ready_o  (dl_rx_ready),

    // Alignment/deskew status
    .align_done_o   (align_done),
    .lane_up_o      (lane_up)
  );

  // PHY-IF wrapper (optional). Handles CDC/hold and connects to pins
  generate if (USE_PHY_IF) begin : g_phy_if
    phy_if_txrx #(
      .PHY_W        (PHY_W),
      .ASYNC_TX     (ASYNC_CDC),
      .ASYNC_RX     (ASYNC_CDC)
    ) u_phy_if (
      .link_clk_i     (link_clk),
      .link_rst_ni    (link_rst_n),

      // Upward (to Framing)
      .dl_tx_data_i   (dl_tx_data),
      .dl_tx_valid_i  (dl_tx_valid),
      .dl_tx_ready_o  (dl_tx_ready),
      .dl_rx_data_o   (dl_rx_data),
      .dl_rx_valid_o  (dl_rx_valid),
      .dl_rx_ready_i  (dl_rx_ready),

      // Pins outward (PHY)
      .phy_tx_data_o  (phy_tx_data),
      .phy_tx_valid_o (phy_tx_valid),
      .phy_tx_ready_i (phy_tx_ready),
      .phy_rx_data_i  (phy_rx_data),
      .phy_rx_valid_i (phy_rx_valid),
      .phy_rx_ready_o (phy_rx_ready)
    );
  end else begin : g_phy_direct
    // Direct wiring (no extra IF)
    assign phy_tx_data = dl_tx_data;
    assign phy_tx_valid = dl_tx_valid;
    assign dl_tx_ready = phy_tx_ready;

    assign dl_rx_data = phy_rx_data;
    assign dl_rx_valid = phy_rx_valid;
    assign phy_rx_ready = dl_rx_ready;
  end endgenerate

  // ------------------ CSR Block ------------------
  // Exposes status, accepts control (force_retrain, init_credit cfg, masks, etc.)
  csr_block u_csr (
    .axil_aclk       (axil_aclk),
    .axil_aresetn    (axil_aresetn),

    .s_axil_awaddr   (s_axil_awaddr),
    .s_axil_awvalid  (s_axil_awvalid),
    .s_axil_awready  (s_axil_awready),
    .s_axil_wdata    (s_axil_wdata),
    .s_axil_wstrb    (s_axil_wstrb),
    .s_axil_wvalid   (s_axil_wvalid),
    .s_axil_wready   (s_axil_wready),
    .s_axil_bresp    (s_axil_bresp),
    .s_axil_bvalid   (s_axil_bvalid),
    .s_axil_bready   (s_axil_bready),
    .s_axil_araddr   (s_axil_araddr),
    .s_axil_arvalid  (s_axil_arvalid),
    .s_axil_arready  (s_axil_arready),
    .s_axil_rdata    (s_axil_rdata),
    .s_axil_rresp    (s_axil_rresp),
    .s_axil_rvalid   (s_axil_rvalid),
    .s_axil_rready   (s_axil_rready),
    .irq_o           (irq),

    // Status inputs
    .link_state_i    (link_state),
    .link_up_i       (link_up),
    .lane_up_i       (lane_up),
    .align_done_i    (align_done),
    .stat_tx_flits_i (stat_tx_flits),
    .stat_rx_flits_i (stat_rx_flits),
    .stat_crc_errs_i (stat_crc_errs),
    .stat_naks_i     (stat_naks),
    .stat_replays_i  (stat_replays),
    .stat_credit_stall_i (stat_credit_stall),

    // Control outputs
    .force_retrain_o (force_retrain)
  );

  // Sideband ready defaults (if not used)
  generate if (!USE_SIDEBAND) begin
    assign sb_tx_data  = '0;
    assign sb_tx_valid = 1'b0;
    assign sb_rx_ready = 1'b1;
  end else begin
    // For now: simple pass-through readiness; actual SB engine to be added later
    assign sb_rx_ready = 1'b1; // always ready; protocol may sample sb_rx_* as needed
    assign sb_tx_data  = '0;   // drive when SB engine exists
    assign sb_tx_valid = 1'b0;
  end endgenerate

endmodule

// ============================================================================
// End of File
// ============================================================================
