package axi_bridge_ip_tx_pkg;

  // Max supported bytes-per-beat keep width.
  // Choose a value comfortably above your maximum DATA_W/8.
  localparam int MAX_KEEP_W = 256; // 512 bytes -> supports DATA_W up to 4096 bits

  // Popcount for a fixed-width keep vector, using only bits [0 .. nbytes-1]
  function automatic int unsigned popcount_keep_fixed(
    input logic [MAX_KEEP_W-1:0] keep,
    input int unsigned           nbytes
  );
    int unsigned cnt;
    cnt = 0;
    for (int i = 0; i < nbytes; i++) begin
      if (keep[i]) cnt++;
    end
    return cnt;
  endfunction

  // tkeep legality for a fixed-width keep vector, using only bits [0 .. nbytes-1]
  // Rules:
  // - Non-last: all valid bytes must be 1 (no gaps, i.e., must be "all ones" for nbytes)
  // - Last: monotonic 1...10...0 (no holes)
  function automatic logic tkeep_illegal_f_fixed(
    input logic [MAX_KEEP_W-1:0] keep,
    input int unsigned           nbytes,
    input logic                  is_last
  );
    logic seen_zero;

    tkeep_illegal_f_fixed = 1'b0;

    if (!is_last) begin
      for (int i = 0; i < nbytes; i++) begin
        if (!keep[i]) tkeep_illegal_f_fixed = 1'b1;
      end
    end else begin
      seen_zero = 1'b0;
      for (int i = 0; i < nbytes; i++) begin
        if (!keep[i]) begin
          seen_zero = 1'b1;
        end else if (seen_zero) begin
          tkeep_illegal_f_fixed = 1'b1;
        end
      end
    end
  endfunction

endpackage
