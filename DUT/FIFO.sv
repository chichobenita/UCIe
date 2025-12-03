// Simple synchronous FIFO, parameterized by width and depth.
// - Single clock, active-low reset.
// - push_i / pop_i handshake.
// - clear_i can be used to flush the FIFO contents (for example in drop_on_midreset scenarios).
module fifo_sync #(
  parameter int WIDTH = 32,
  parameter int DEPTH = 16
)(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 clear_i,

  // Write side (enqueue)
  input  logic                 push_i,
  input  logic [WIDTH-1:0]     wdata_i,

  // Read side (dequeue)
  input  logic                 pop_i,
  output logic [WIDTH-1:0]     rdata_o,

  // Status flags and occupancy level
  output logic                 full_o,
  output logic                 empty_o,
  output logic [$clog2(DEPTH+1)-1:0] level_o
);

  // Address width for memory indexing (DEPTH entries)
  localparam int ADDR_W = $clog2(DEPTH);

  // Storage array for FIFO data
  logic [WIDTH-1:0] mem [DEPTH-1:0];

  // Read / write pointers
  logic [ADDR_W-1:0] wr_ptr, rd_ptr;

  // Occupancy counter: number of valid entries in FIFO (0..DEPTH)
  logic [$clog2(DEPTH+1)-1:0] count;

  // Read data is "show-ahead": rdata_o always reflects mem[rd_ptr]
  assign rdata_o = mem[rd_ptr];

  // Status signals derived from occupancy
  assign full_o  = (count == DEPTH);
  assign empty_o = (count == 0);
  assign level_o = count;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Asynchronous active-low reset: clear pointers and occupancy
      wr_ptr <= '0;
      rd_ptr <= '0;
      count  <= '0;
    end else if (clear_i) begin
      // Synchronous clear: flush FIFO contents and reset pointers
      wr_ptr <= '0;
      rd_ptr <= '0;
      count  <= '0;
    end else begin
      // Default: no change, decide action based on push/pop handshake
      logic do_push;
      logic do_pop;
      
      // We should not push when the FIFO is full, but guard against it anyway
      do_push = push_i && !full_o;
      // Similarly, avoid popping when FIFO is empty
      do_pop  = pop_i  && !empty_o;

      // Encoded {do_push, do_pop} behavior:
      unique case ({do_push, do_pop})
        2'b10: begin // push only (enqueue one element)
          mem[wr_ptr] <= wdata_i;
          // Circular increment of write pointer
          wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1;
          count       <= count + 1;
        end
        2'b01: begin // pop only (dequeue one element)
          // Circular increment of read pointer
          rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1;
          count       <= count - 1;
        end
        2'b11: begin // push + pop in the same cycle (throughput mode)
          // Overwrite at write pointer while advancing both pointers:
          // logical "shift" without changing occupancy
          mem[wr_ptr] <= wdata_i;
          wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1;
          rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1;
          // count stays the same
        end
        default: ;   // 2'b00: no-op (no push, no pop)
      endcase
    end
  end

endmodule
