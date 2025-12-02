// Simple synchronous FIFO, parameterized by width and depth.
// - Single clock, active-low reset.
// - push_i / pop_i handshake.
// - clear_i מאפשר לרוקן את ה-FIFO (למשל ב-drop_on_midreset).
module fifo_sync #(
  parameter int WIDTH = 32,
  parameter int DEPTH = 16
)(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 clear_i,

  input  logic                 push_i,
  input  logic [WIDTH-1:0]     wdata_i,

  input  logic                 pop_i,
  output logic [WIDTH-1:0]     rdata_o,

  output logic                 full_o,
  output logic                 empty_o,
  output logic [$clog2(DEPTH+1)-1:0] level_o
);

  localparam int ADDR_W = $clog2(DEPTH);

  logic [WIDTH-1:0] mem [DEPTH-1:0];
  logic [ADDR_W-1:0] wr_ptr, rd_ptr;
  logic [$clog2(DEPTH+1)-1:0] count;

  // read data is "show-ahead" (data at rd_ptr)
  assign rdata_o = mem[rd_ptr];

  assign full_o  = (count == DEPTH);
  assign empty_o = (count == 0);
  assign level_o = count;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
      count  <= '0;
    end else if (clear_i) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
      count  <= '0;
    end else begin
      // default: no change
      logic do_push;
      logic do_pop;
      
      do_push = push_i && !full_o;   // אנחנו לא אמורים לדחוף כשהוא מלא, אבל מגנים
      do_pop  = pop_i  && !empty_o;

      unique case ({do_push, do_pop})
        2'b10: begin // push only
          mem[wr_ptr] <= wdata_i;
          wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1;
          count       <= count + 1;
        end
        2'b01: begin // pop only
          rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1;
          count       <= count - 1;
        end
        2'b11: begin // push + pop (throughput)
          mem[wr_ptr] <= wdata_i;
          wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1;
          rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1;
          // count stays the same
        end
        default: ;   // 2'b00: no-op
      endcase
    end
  end

endmodule
