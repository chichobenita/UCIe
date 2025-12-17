// Top-level TX side of the Protocol Core.
// From Client-IF segments (cl_tx_*) to logical flits + CRC (pc_tx_*)
// towards the Framing layer.

module protocol_core_tx #(
  // -------- Client-IF / AXI-Bridge side --------
  parameter int IF_W             	= 64,    // segment data width
  parameter int TUSER_W         	= 16,    // user meta width

  // -------- Protocol parameters --------
  parameter int NUM_VC           	= 8,
  parameter int SEQ_W            	= 12,    // sequence number width
  parameter int LEN_W            	= 9,     // enough for payload bytes count
  parameter int FLAGS_W          	= 4,     // SOP/EOP + future flags
  parameter int FLIT_PAYLOAD_W   	= 256,   // payload bits per flit
  parameter int CRC_W            	= 16     // CRC width
)(
  input  logic                   	clk_i,
  input  logic                   	rst_ni,

  // ================================================================
  // Client-IF TX side (from AXI-Bridge)
  // ================================================================
  input  logic [IF_W-1:0]        	cl_tx_data,
  input  logic [IF_W/8-1:0]     	cl_tx_keep,
  input  logic [TUSER_W-1:0]     	cl_tx_user,
  input  logic                   	cl_tx_sop,
  input  logic                   	cl_tx_eop,
  input  logic                   	cl_tx_valid,
  output logic                   	cl_tx_ready,

  // ================================================================
  // Flit TX interface towards Framing layer (logical flit + CRC)
  // ================================================================
  output logic                   	pc_tx_valid,
  input  logic                   	pc_tx_ready,
  output logic [1:0]             	pc_tx_type,       // DATA / ACK / NAK / CTRL
  output logic [$clog2(NUM_VC)-1:0] pc_tx_vc_id,
  output logic [SEQ_W-1:0]       	pc_tx_seq,
  output logic [LEN_W-1:0]       	pc_tx_len,
  output logic [FLAGS_W-1:0]     	pc_tx_flags,
  output logic [FLIT_PAYLOAD_W-1:0] pc_tx_payload,
  output logic [CRC_W-1:0]       	pc_tx_crc,

  // ================================================================
  // CSR control (config)
  // ================================================================
  input  logic                   	tx_enable,
  input  logic [NUM_VC-1:0]      	vc_enable_mask,
  input  logic                   	replay_enable,
  input  logic [NUM_VC-1:0][15:0] 	tx_init_credits,

  // ================================================================
  // CSR status / events (placeholders)
  // ================================================================
  output logic [31:0]            	stat_tx_flits_total,
  output logic [31:0]            	stat_tx_bytes_total,
  output logic [NUM_VC-1:0][15:0] 	stat_tx_vc_fifo_level,
  output logic [31:0]            	stat_tx_stall_cycles,
  output logic                   	ev_tx_no_credits,
  output logic                   	ev_tx_vc_fifo_overflow,
  output logic                   	ev_tx_replay_exhausted,

  // ================================================================
  // Control flits to inject (from RX side / txrx_ctrl) – e.g. ACK/NAK
  // ================================================================
  input  logic                   	ctrl_flit_valid,
  output logic                   	ctrl_flit_ready,
  input  logic [1:0]             	ctrl_flit_type,   // ACK / NAK / CTRL
  input  logic [$clog2(NUM_VC)-1:0] ctrl_flit_vc_id,
  input  logic [SEQ_W-1:0]       	ctrl_flit_seq,

  // ================================================================
  // ACK / NAK updates into Replay / Credits (from RX / txrx_ctrl)
  // ================================================================
  input  logic                   	ack_update_valid,
  input  logic [$clog2(NUM_VC)-1:0] ack_update_vc,
  input  logic [SEQ_W-1:0]       	ack_update_seq,
  input  logic                   	nak_update_valid,
  input  logic [$clog2(NUM_VC)-1:0] nak_update_vc,
  input  logic [SEQ_W-1:0]       	nak_update_seq
);

  // ----------------------------------------------------------------
  // Local parameters
  // ----------------------------------------------------------------
  localparam int VC_W = $clog2(NUM_VC);

  // ----------------------------------------------------------------
  // Wires between submodules
  // ----------------------------------------------------------------

  // ---- ingress_seg -> vc_tx ----
  logic                   seg_in_valid;
  logic                   seg_in_ready;
  logic [IF_W-1:0]        seg_in_data;
  logic [IF_W/8-1:0]      seg_in_keep;
  logic [TUSER_W-1:0]     seg_in_user;
  logic                   seg_in_sop;
  logic                   seg_in_eop;
  logic [VC_W-1:0]        seg_in_vc_id;

  // ---- vc_tx -> vc_scheduler ----
  logic [NUM_VC-1:0]      vc_can_send;
  logic [NUM_VC-1:0]      vc_has_data;

  // ---- vc_scheduler -> vc_tx / flit_builder ----
  logic                   vc_grant_valid;
  logic [VC_W-1:0]        vc_grant_id;

  // ---- vc_tx -> flit_builder_tx ----
  logic                   seg_out_valid;
  logic                   seg_out_ready;
  logic [IF_W-1:0]        seg_out_data;
  logic [IF_W/8-1:0]      seg_out_keep;
  logic [TUSER_W-1:0]     seg_out_user;
  logic                   seg_out_sop;
  logic                   seg_out_eop;
  logic [VC_W-1:0]        seg_out_vc_id;

  // ---- flit_builder_tx -> crc_gen_tx / replay_tx (DATA flits) ----
  logic                   data_flit_valid;
  logic                   data_flit_ready;
  logic [1:0]             data_flit_type;    // will be DATA here
  logic [VC_W-1:0]        data_flit_vc_id;
  logic [SEQ_W-1:0]       data_flit_seq;
  logic [LEN_W-1:0]       data_flit_len;
  logic [FLAGS_W-1:0]     data_flit_flags;
  logic [FLIT_PAYLOAD_W-1:0] data_flit_payload;
  logic [CRC_W-1:0]       data_flit_crc;

  // ---- replay_tx -> tx_flit_arbiter (Replay stream) ----
  logic                   replay_valid;
  logic                   replay_ready;
  logic [1:0]             replay_type;
  logic [VC_W-1:0]        replay_vc_id;
  logic [SEQ_W-1:0]       replay_seq;
  logic [LEN_W-1:0]       replay_len;
  logic [FLAGS_W-1:0]     replay_flags;
  logic [FLIT_PAYLOAD_W-1:0] replay_payload;
  logic [CRC_W-1:0]       replay_crc;

  // ---- tx_flit_arbiter -> pc_tx_* (final flit to Framing) ----
  logic                   arb_flit_valid;
  logic                   arb_flit_ready;
  logic [1:0]             arb_flit_type;
  logic [VC_W-1:0]        arb_flit_vc_id;
  logic [SEQ_W-1:0]       arb_flit_seq;
  logic [LEN_W-1:0]       arb_flit_len;
  logic [FLAGS_W-1:0]     arb_flit_flags;
  logic [FLIT_PAYLOAD_W-1:0] arb_flit_payload;
  logic [CRC_W-1:0]       arb_flit_crc;

  // ----------------------------------------------------------------
  // Submodule instantiations (skeletons – logic to be implemented inside)
  // ----------------------------------------------------------------

  // 1) ingress_seg: AXI-Bridge Client-IF -> single segment stream + vc_id
  ingress_seg_to_vc_tx #(
    .IF_W   (IF_W),
    .TUSER_W(TUSER_W),
    .NUM_VC (NUM_VC)
  ) u_ingress_seg (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // From AXI-Bridge
    .cl_tx_data (cl_tx_data),
    .cl_tx_keep (cl_tx_keep),
    .cl_tx_user (cl_tx_user),
    .cl_tx_sop  (cl_tx_sop),
    .cl_tx_eop  (cl_tx_eop),
    .cl_tx_valid(cl_tx_valid),
    .cl_tx_ready(cl_tx_ready),

    // Out to vc_tx
    .seg_valid_o(seg_in_valid),
    .seg_ready_i(seg_in_ready),
    .seg_data_o (seg_in_data),
    .seg_keep_o (seg_in_keep),
    .seg_user_o (seg_in_user),
    .seg_sop_o  (seg_in_sop),
    .seg_eop_o  (seg_in_eop),
    .seg_vc_id_o(seg_in_vc_id)
  );

  // 2) vc_tx: per-VC FIFOs + credits
  vc_tx #(
    .IF_W   (IF_W),
    .TUSER_W(TUSER_W),
    .NUM_VC (NUM_VC),
    .SEQ_W  (SEQ_W)
  ) u_vc_tx (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Ingress from ingress_seg
    .in_valid_i (seg_in_valid),
    .in_ready_o (seg_in_ready),
    .in_data_i  (seg_in_data),
    .in_keep_i  (seg_in_keep),
    .in_user_i  (seg_in_user),
    .in_sop_i   (seg_in_sop),
    .in_eop_i   (seg_in_eop),
    .in_vc_id_i (seg_in_vc_id),

    // Status to scheduler
    .vc_has_data_o(vc_has_data),
    .vc_can_send_o(vc_can_send),

    // Grant from scheduler
    .grant_valid_i(vc_grant_valid),
    .grant_vc_id_i(vc_grant_id),

    // Egress segment stream to flit_builder
    .out_valid_o (seg_out_valid),
    .out_ready_i (seg_out_ready),
    .out_data_o  (seg_out_data),
    .out_keep_o  (seg_out_keep),
    .out_user_o  (seg_out_user),
    .out_sop_o   (seg_out_sop),
    .out_eop_o   (seg_out_eop),
    .out_vc_id_o (seg_out_vc_id),

    // Credits / ACK/NAK updates
    .ack_update_valid_i(ack_update_valid),
    .ack_update_seq_i  (ack_update_seq),
    .nak_update_valid_i(nak_update_valid),
    .nak_update_seq_i  (nak_update_seq),

    // CSR config
    .tx_enable_i      (tx_enable),
    .vc_enable_mask_i (vc_enable_mask),
    .tx_init_credits_i(tx_init_credits),

    // Optional status
    .stat_vc_fifo_level_o(stat_tx_vc_fifo_level)
  );

  // 3) vc_scheduler: chooses which VC to serve next
  vc_scheduler #(
    .NUM_VC(NUM_VC)
  ) u_vc_scheduler (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .vc_can_send_i(vc_can_send),
    .grant_valid_o(vc_grant_valid),
    .grant_vc_id_o(vc_grant_id)
  );

  // 4) flit_builder_tx: segments -> DATA flits (header + payload, no CRC)
  flit_builder_tx #(
    .IF_W           (IF_W),
    .TUSER_W        (TUSER_W),
    .NUM_VC         (NUM_VC),
    .SEQ_W          (SEQ_W),
    .LEN_W          (LEN_W),
    .FLAGS_W        (FLAGS_W),
    .FLIT_PAYLOAD_W (FLIT_PAYLOAD_W)
  ) u_flit_builder_tx (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    // Segments selected by vc_tx/scheduler
    .seg_valid_i    (seg_out_valid),
    .seg_ready_o    (seg_out_ready),
    .seg_data_i     (seg_out_data),
    .seg_keep_i     (seg_out_keep),
    .seg_user_i     (seg_out_user),
    .seg_sop_i      (seg_out_sop),
    .seg_eop_i      (seg_out_eop),
    .seg_vc_id_i    (seg_out_vc_id),

    // Logical DATA flit out (no CRC yet)
    .flit_valid_o   (data_flit_valid),
    .flit_ready_i   (data_flit_ready),
    .flit_type_o    (data_flit_type),
    .flit_vc_id_o   (data_flit_vc_id),
    .flit_seq_o     (data_flit_seq),
    .flit_len_o     (data_flit_len),
    .flit_flags_o   (data_flit_flags),
    .flit_payload_o (data_flit_payload)
  );

  // 5) crc_gen_tx: combinational CRC over (Header || Payload)
  crc_gen_tx #(
    .NUM_VC         (NUM_VC),
    .SEQ_W          (SEQ_W),
    .LEN_W          (LEN_W),
    .FLAGS_W        (FLAGS_W),
    .FLIT_PAYLOAD_W (FLIT_PAYLOAD_W),
    .CRC_W          (CRC_W)
  ) u_crc_gen_tx (
    .type_i     (data_flit_type),
    .vc_id_i    (data_flit_vc_id),
    .seq_i      (data_flit_seq),
    .len_i      (data_flit_len),
    .flags_i    (data_flit_flags),
    .payload_i  (data_flit_payload),
    .crc_o      (data_flit_crc)
  );

  // 6) replay_tx: store DATA flits (with CRC) until ACKed, output replay stream
  replay_tx #(
    .NUM_VC         (NUM_VC),
    .SEQ_W          (SEQ_W),
    .LEN_W          (LEN_W),
    .FLAGS_W        (FLAGS_W),
    .FLIT_PAYLOAD_W (FLIT_PAYLOAD_W),
    .CRC_W          (CRC_W)
  ) u_replay_tx (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    // New DATA flits from flit_builder + CRC
    .flit_valid_i   (data_flit_valid),
    .flit_ready_o   (data_flit_ready),
    .flit_type_i    (data_flit_type),
    .flit_vc_id_i   (data_flit_vc_id),
    .flit_seq_i     (data_flit_seq),
    .flit_len_i     (data_flit_len),
    .flit_flags_i   (data_flit_flags),
    .flit_payload_i (data_flit_payload),
    .flit_crc_i     (data_flit_crc),

    // ACK / NAK window updates
    .ack_update_valid_i(ack_update_valid),
    .ack_update_seq_i  (ack_update_seq),
    .nak_update_valid_i(nak_update_valid),
    .nak_update_seq_i  (nak_update_seq),

    // Replay flit stream out
    .replay_valid_o (replay_valid),
    .replay_ready_i (replay_ready),
    .replay_type_o  (replay_type),
    .replay_vc_id_o (replay_vc_id),
    .replay_seq_o   (replay_seq),
    .replay_len_o   (replay_len),
    .replay_flags_o (replay_flags),
    .replay_payload_o(replay_payload),
    .replay_crc_o   (replay_crc),

    .replay_enable_i(replay_enable),
    .ev_replay_exhausted_o(ev_tx_replay_exhausted)
  );

  // 7) tx_flit_arbiter: choose between Replay, control (ACK/NAK), and new DATA
  tx_flit_arbiter #(
    .NUM_VC         (NUM_VC),
    .SEQ_W          (SEQ_W),
    .LEN_W          (LEN_W),
    .FLAGS_W        (FLAGS_W),
    .FLIT_PAYLOAD_W (FLIT_PAYLOAD_W),
    .CRC_W          (CRC_W)
  ) u_tx_flit_arbiter (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    // Replay stream (highest priority)
    .replay_valid_i (replay_valid),
    .replay_ready_o (replay_ready),
    .replay_type_i  (replay_type),
    .replay_vc_id_i (replay_vc_id),
    .replay_seq_i   (replay_seq),
    .replay_len_i   (replay_len),
    .replay_flags_i (replay_flags),
    .replay_payload_i(replay_payload),
    .replay_crc_i   (replay_crc),

    // Control flits from RX side (ACK / NAK)
    .ctrl_valid_i   (ctrl_flit_valid),
    .ctrl_ready_o   (ctrl_flit_ready),
    .ctrl_type_i    (ctrl_flit_type),
    .ctrl_vc_id_i   (ctrl_flit_vc_id),
    .ctrl_seq_i     (ctrl_flit_seq),

    // Optional: direct DATA flits bypassing replay when replay disabled
    .new_valid_i    (data_flit_valid),
    .new_ready_o    (), // can be left unconnected or wired as needed
    .new_type_i     (data_flit_type),
    .new_vc_id_i    (data_flit_vc_id),
    .new_seq_i      (data_flit_seq),
    .new_len_i      (data_flit_len),
    .new_flags_i    (data_flit_flags),
    .new_payload_i  (data_flit_payload),
    .new_crc_i      (data_flit_crc),

    // Output to Framing
    .out_valid_o    (arb_flit_valid),
    .out_ready_i    (arb_flit_ready),
    .out_type_o     (arb_flit_type),
    .out_vc_id_o    (arb_flit_vc_id),
    .out_seq_o      (arb_flit_seq),
    .out_len_o      (arb_flit_len),
    .out_flags_o    (arb_flit_flags),
    .out_payload_o  (arb_flit_payload),
    .out_crc_o      (arb_flit_crc)
  );

  // ----------------------------------------------------------------
  // Final mapping: arbiter -> pc_tx_* (towards Framing layer)
  // ----------------------------------------------------------------
  assign pc_tx_valid   = arb_flit_valid;
  assign arb_flit_ready = pc_tx_ready;

  assign pc_tx_type    = arb_flit_type;
  assign pc_tx_vc_id   = arb_flit_vc_id;
  assign pc_tx_seq     = arb_flit_seq;
  assign pc_tx_len     = arb_flit_len;
  assign pc_tx_flags   = arb_flit_flags;
  assign pc_tx_payload = arb_flit_payload;
  assign pc_tx_crc     = arb_flit_crc;

  // ----------------------------------------------------------------
  // TODO: implement counters/statistics and stall counting
  // ----------------------------------------------------------------
  // stat_tx_flits_total, stat_tx_bytes_total, stat_tx_stall_cycles, etc.
  // can be driven here once the internal handshakes are finalized.

endmodule
