module axi_bridge_ip_tx_serializer #(
  parameter int DATA_W      = 256,
  parameter int IF_W        = 64,
  parameter int TUSER_W     = 16,
  parameter int FIFO_DEPTH  = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  // CSR
  input  logic                   bridge_enable,
  input  logic                   drop_on_midreset,

  // FIFO interface
  input  logic                   fifo_empty_i,
  input  logic [DATA_W + (DATA_W/8) + TUSER_W + 1 - 1:0] fifo_rdata_i,
  output logic                   fifo_pop_o,
  output logic                   fifo_clear_o,

  // Downstream
  input  logic                   cl_tx_ready_i,
  output logic [IF_W-1:0]        cl_tx_data_o,
  output logic [IF_W/8-1:0]      cl_tx_keep_o,
  output logic [TUSER_W-1:0]     cl_tx_user_o,
  output logic                   cl_tx_valid_o,
  output logic                   cl_tx_sop_o,
  output logic                   cl_tx_eop_o,

  // To stats
  output logic                   frame_done_pulse_o,
  output logic                   stall_cycle_en_o,

  // Event
  output logic                   ev_err_midreset_drop_pulse_o
);

  // Internal wires between submodules
  logic flush_pulse;

  logic beat_done_pulse;
  logic beat_ready; // from segment_gen (optional/for debug/future use)

  logic beat_valid;
  logic [DATA_W-1:0]   beat_data;
  logic [DATA_W/8-1:0] beat_keep;
  logic [TUSER_W-1:0]  beat_user;
  logic               beat_last;

  logic [$clog2(((DATA_W/8)+(IF_W/8)-1)/(IF_W/8) + 1)-1:0] beat_num_segs;
  logic [$clog2((DATA_W/8)+1)-1:0]                        beat_last_seg_bytes;

  logic in_packet;

  // Midreset detection state
  logic bridge_enable_q;

  // Pending activity (for midreset drop)
  logic pending_activity;
  assign pending_activity = in_packet || beat_valid || cl_tx_valid_o || !fifo_empty_i;

  // Flush + fifo_clear generation
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      bridge_enable_q              <= 1'b0;
      flush_pulse                  <= 1'b0;
      fifo_clear_o                 <= 1'b0;
      ev_err_midreset_drop_pulse_o <= 1'b0;
    end else begin
      bridge_enable_q              <= bridge_enable;

      flush_pulse                  <= 1'b0;
      fifo_clear_o                 <= 1'b0;
      ev_err_midreset_drop_pulse_o <= 1'b0;

      if (drop_on_midreset &&
          bridge_enable_q && !bridge_enable &&
          pending_activity) begin
        flush_pulse                  <= 1'b1;
        fifo_clear_o                 <= 1'b1;
        ev_err_midreset_drop_pulse_o <= 1'b1;
      end
    end
  end

  axi_bridge_ip_tx_ser_beat_fetch #(
    .DATA_W     (DATA_W),
    .IF_W       (IF_W),
    .TUSER_W    (TUSER_W),
    .FIFO_DEPTH (FIFO_DEPTH)
  ) u_beat_fetch (
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    .enable_i            (bridge_enable),
    .flush_i             (flush_pulse),

    // IMPORTANT:
    // We drive beat_ready_i with beat_done_pulse to keep semantics consistent
    // and avoid relying on beat_ready_o definition.
    .beat_ready_i        (beat_done_pulse),

    .fifo_empty_i        (fifo_empty_i),
    .fifo_rdata_i        (fifo_rdata_i),
    .fifo_pop_o          (fifo_pop_o),

    .beat_done_pulse_i   (beat_done_pulse),

    .beat_valid_o        (beat_valid),
    .beat_data_o         (beat_data),
    .beat_keep_o         (beat_keep),
    .beat_user_o         (beat_user),
    .beat_last_o         (beat_last),

    .beat_num_segs_o       (beat_num_segs),
    .beat_last_seg_bytes_o (beat_last_seg_bytes)
  );

  axi_bridge_ip_tx_ser_segment_gen #(
    .DATA_W  (DATA_W),
    .IF_W    (IF_W),
    .TUSER_W (TUSER_W)
  ) u_segment_gen (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .enable_i              (bridge_enable),
    .flush_i               (flush_pulse),

    .beat_valid_i          (beat_valid),
    .beat_data_i           (beat_data),
    .beat_keep_i           (beat_keep),
    .beat_user_i           (beat_user),
    .beat_last_i           (beat_last),
    .beat_num_segs_i       (beat_num_segs),
    .beat_last_seg_bytes_i (beat_last_seg_bytes),

    .cl_tx_ready_i         (cl_tx_ready_i),

    .cl_tx_data_o          (cl_tx_data_o),
    .cl_tx_keep_o          (cl_tx_keep_o),
    .cl_tx_user_o          (cl_tx_user_o),
    .cl_tx_valid_o         (cl_tx_valid_o),
    .cl_tx_sop_o           (cl_tx_sop_o),
    .cl_tx_eop_o           (cl_tx_eop_o),

    .beat_done_pulse_o     (beat_done_pulse),
    .frame_done_pulse_o    (frame_done_pulse_o),
    .stall_cycle_en_o      (stall_cycle_en_o),

    .beat_ready_o          (beat_ready),   // not used by beat_fetch currently
    .in_packet_o           (in_packet)
  );

endmodule
