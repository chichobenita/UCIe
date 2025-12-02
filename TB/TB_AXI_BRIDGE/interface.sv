// axi_stream_if.sv
interface axi_stream_if #(
  parameter int DATA_W  = 256,
  parameter int TUSER_W = 16
) (
  input  logic clk,
  input  logic rst_n
);
  logic [DATA_W-1:0]         tdata;
  logic [DATA_W/8-1:0]       tkeep;
  logic [TUSER_W-1:0]        tuser;
  logic                      tvalid;
  logic                      tready;
  logic                      tlast;

  // modport ל-DUT
  modport dut (
    input  tdata, tkeep, tuser, tvalid, tlast,
    output tready
  );

  // modport ל-Driver
  modport drv (
    input  clk, rst_n, tready,
    output tdata, tkeep, tuser, tvalid, tlast
  );

  // modport למוניטור/דיבאג
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
  logic [IF_W-1:0]          data;
  logic [IF_W/8-1:0]        keep;
  logic [TUSER_W-1:0]       user;
  logic                     valid;
  logic                     ready;
  logic                     sop;
  logic                     eop;

  // דגימה אחרי שה-DUT עדכן את ה-Qים
  clocking cb @(posedge clk);
    default input #1step output #0;
    input data, keep, user, valid, ready, sop, eop;
  endclocking

  modport dut (output data, keep, user, valid, sop, eop,
               input  ready);

  modport drv (input  clk, rst_n, data, keep, user, valid, sop, eop,
               output ready);

  modport mon (clocking cb, input rst_n);
endinterface



