module axi_bridge_ip_tx #(
  parameter int DATA_W       = 256,
  parameter int IF_W         = 64,
  parameter int TUSER_W      = 16,
  parameter int FIFO_DEPTH   = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  // AXI-Stream from IP
  input  logic [DATA_W-1:0]      s_axis_tdata,
  input  logic [DATA_W/8-1:0]    s_axis_tkeep,
  input  logic [TUSER_W-1:0]     s_axis_tuser,
  input  logic                   s_axis_tvalid,
  input  logic                   s_axis_tlast,
  output logic                   s_axis_tready,

  // Client-IF segments
  output logic [IF_W-1:0]        cl_tx_data,
  output logic [IF_W/8-1:0]      cl_tx_keep,
  output logic [TUSER_W-1:0]     cl_tx_user,
  output logic                   cl_tx_valid,
  output logic                   cl_tx_sop,
  output logic                   cl_tx_eop,
  input  logic                   cl_tx_ready,

  // CSR Control
  input  logic                   bridge_enable,
  input  logic                   strict_tkeep_en,
  input  logic [7:0]             tx_fifo_afull_thr,
  input  logic                   drop_on_midreset,

  // CSR Status / Telemetry
  output logic [31:0]            stat_tx_frames,
  output logic [31:0]            stat_tx_bytes,
  output logic [15:0]            stat_tx_fifo_level,
  output logic [31:0]            stat_tx_stall_cycles,

  output logic                   ev_err_tkeep_illegal,
  output logic                   ev_err_midreset_drop,
  output logic                   ev_err_overflow_tx
);

  localparam int BYTES_PER_BEAT = DATA_W/8;
  localparam int ENTRY_W        = DATA_W + BYTES_PER_BEAT + TUSER_W + 1;

  initial begin
    if (DATA_W % 8 != 0 || IF_W % 8 != 0) begin
      $error("DATA_W and IF_W must be multiples of 8");
    end
    if (IF_W > DATA_W) begin
      $error("axi_bridge_ip_tx assumes IF_W <= DATA_W");
    end
  end

  // FIFO signals
  logic                 fifo_push, fifo_pop;
  logic [ENTRY_W-1:0]   fifo_wdata, fifo_rdata;
  logic                 fifo_full, fifo_empty;
  logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level;
  logic                 fifo_clear;

  // Ingress -> Stats
  logic [31:0] bytes_inc;
  logic        bytes_inc_valid;

  // Serializer -> Stats
  logic frame_done_pulse;
  logic stall_cycle_en;

  // Ingress
  axi_bridge_ip_tx_ingress #(
    .DATA_W     (DATA_W),
    .TUSER_W    (TUSER_W),
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_ingress (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),

    .s_axis_tdata           (s_axis_tdata),
    .s_axis_tkeep           (s_axis_tkeep),
    .s_axis_tuser           (s_axis_tuser),
    .s_axis_tvalid          (s_axis_tvalid),
    .s_axis_tlast           (s_axis_tlast),
    .s_axis_tready          (s_axis_tready),

    .bridge_enable          (bridge_enable),
    .strict_tkeep_en        (strict_tkeep_en),
    .tx_fifo_afull_thr      (tx_fifo_afull_thr),

    .fifo_full              (fifo_full),
    .fifo_level             (fifo_level),

    .fifo_push              (fifo_push),
    .fifo_wdata             (fifo_wdata),

    .bytes_inc              (bytes_inc),
    .bytes_inc_valid        (bytes_inc_valid),

    .ev_err_tkeep_illegal_pulse (ev_err_tkeep_illegal),
    .ev_err_overflow_tx_pulse   (ev_err_overflow_tx)
  );

  // FIFO
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

  // Serializer
  axi_bridge_ip_tx_serializer #(
    .DATA_W     (DATA_W),
    .IF_W       (IF_W),
    .TUSER_W    (TUSER_W),
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_serializer (
    .clk_i                     (clk_i),
    .rst_ni                    (rst_ni),

    .bridge_enable             (bridge_enable),
    .drop_on_midreset          (drop_on_midreset),

    .fifo_empty_i              (fifo_empty),
    .fifo_rdata_i              (fifo_rdata),
    .fifo_pop_o                (fifo_pop),
    .fifo_clear_o              (fifo_clear),

    .cl_tx_ready_i             (cl_tx_ready),
    .cl_tx_data_o              (cl_tx_data),
    .cl_tx_keep_o              (cl_tx_keep),
    .cl_tx_user_o              (cl_tx_user),
    .cl_tx_valid_o             (cl_tx_valid),
    .cl_tx_sop_o               (cl_tx_sop),
    .cl_tx_eop_o               (cl_tx_eop),

    .frame_done_pulse_o        (frame_done_pulse),
    .stall_cycle_en_o          (stall_cycle_en),

    .ev_err_midreset_drop_pulse_o (ev_err_midreset_drop)
  );

  // Stats
  axi_bridge_ip_tx_stats #(
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_stats (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),

    .bytes_inc_i          (bytes_inc),
    .bytes_inc_valid_i    (bytes_inc_valid),

    .frame_done_pulse_i   (frame_done_pulse),
    .stall_cycle_en_i     (stall_cycle_en),

    .fifo_level_i         (fifo_level),

    .stat_tx_frames       (stat_tx_frames),
    .stat_tx_bytes        (stat_tx_bytes),
    .stat_tx_fifo_level   (stat_tx_fifo_level),
    .stat_tx_stall_cycles (stat_tx_stall_cycles)
  );

endmodule
