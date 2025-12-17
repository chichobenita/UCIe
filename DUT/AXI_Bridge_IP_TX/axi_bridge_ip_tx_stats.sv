module axi_bridge_ip_tx_stats #(
  parameter int FIFO_DEPTH = 16
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,

  input  logic [31:0]            bytes_inc_i,
  input  logic                   bytes_inc_valid_i,

  input  logic                   frame_done_pulse_i,
  input  logic                   stall_cycle_en_i,

  input  logic [$clog2(FIFO_DEPTH+1)-1:0] fifo_level_i,

  output logic [31:0]            stat_tx_frames,
  output logic [31:0]            stat_tx_bytes,
  output logic [15:0]            stat_tx_fifo_level,
  output logic [31:0]            stat_tx_stall_cycles
);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      stat_tx_frames       <= '0;
      stat_tx_bytes        <= '0;
      stat_tx_fifo_level   <= '0;
      stat_tx_stall_cycles <= '0;
    end else begin
      stat_tx_fifo_level <= {{(16-$bits(fifo_level_i)){1'b0}}, fifo_level_i};

      if (bytes_inc_valid_i) begin
        stat_tx_bytes <= stat_tx_bytes + bytes_inc_i;
      end

      if (frame_done_pulse_i) begin
        stat_tx_frames <= stat_tx_frames + 1;
      end

      if (stall_cycle_en_i) begin
        stat_tx_stall_cycles <= stat_tx_stall_cycles + 1;
      end
    end
  end

endmodule
