// axi_stream_if.sv
interface axi_stream_if #(
  parameter int DATA_W  = 256,
  parameter int TUSER_W = 16
) (
  input  logic clk,
  input  logic rst_n
);
  // AXI-Stream payload and sideband signals
  logic [DATA_W-1:0]         tdata;
  logic [DATA_W/8-1:0]       tkeep;
  logic [TUSER_W-1:0]        tuser;
  logic                      tvalid;
  logic                      tready;
  logic                      tlast;

  // Modport towards the DUT (AXI-Stream slave)
  modport dut (
    input  tdata, tkeep, tuser, tvalid, tlast,
    output tready
  );

  // Modport towards the Driver (AXI-Stream master in TB)
  modport drv (
    input  clk, rst_n, tready,
    output tdata, tkeep, tuser, tvalid, tlast
  );

  // Modport for monitor / debug (passive observation)
  modport mon (
    input clk, rst_n, tdata, tkeep, tuser, tvalid, tready, tlast
  );
endinterface

// client_if.sv
interface client_if #(
  parameter int IF_W    = 64,
  parameter int TUSER_W = 16
) (
  input  logic clk,
  input  logic rst_n
);
  // Client-side data-path: segmented output of the bridge
  logic [IF_W-1:0]          data;
  logic [IF_W/8-1:0]        keep;
  logic [TUSER_W-1:0]       user;
  logic                     valid;
  logic                     ready;
  logic                     sop;   // Start-of-packet
  logic                     eop;   // End-of-packet

  // Clocking block: sampled after DUT has updated its outputs / FIFOs
  clocking cb @(posedge clk);
    default input #1step output #0;
    input data, keep, user, valid, ready, sop, eop;
  endclocking

  // Modport towards the DUT (bridge drives Client-IF)
  modport dut (
    output data, keep, user, valid, sop, eop,
    input  ready
  );

  // Modport towards a driver / stimulus that controls ready
  modport drv (
    input  clk, rst_n, data, keep, user, valid, sop, eop,
    output ready
  );

  // Modport towards monitor (uses clocking block, plus rst_n)
  modport mon (
    clocking cb,
    input    rst_n
  );
endinterface
