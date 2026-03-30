// =============================================================================
// mesi_main_mem.sv
//
// Main memory for the MESI cache coherence protocol.
//
// Behaviour
//   - Single-port, synchronous read, synchronous write.
//   - One request is serviced per cycle (no pipelining).
//   - Responds to BUS_RD and BUS_RDX by putting data on data_o after
//     mem_latency_p cycles (ack_o pulses for exactly one cycle).
//   - Responds to BUS_FLUSH by writing the supplied data into the array
//     (ack_o pulses for one cycle when the write is committed).
//   - All other bus transactions are ignored.
//
// Interface signals
//   clk_i        – system clock
//   reset_i      – synchronous active-high reset
//   trans_i      – bus transaction type (from mesi_pkg::bus_trans_e)
//   addr_i       – byte address of the request
//   data_i       – write data (valid only when trans_i == BUS_FLUSH)
//   data_o       – read data  (valid only when ack_o == 1)
//   ack_o        – single-cycle pulse: request has been serviced
//
// Coding standard: BSG
// =============================================================================

`include "mesi_pkg.sv"   // include guard handled by the package itself

import mesi_pkg::*;

module mesi_main_mem
  #(parameter int MEM_DEPTH_P   = 1024        // number of words in memory
   ,parameter int DATA_WIDTH_P  = mesi_pkg::DATA_WIDTH_P
   ,parameter int ADDR_WIDTH_P  = mesi_pkg::ADDR_WIDTH_P
   ,parameter int MEM_LATENCY_P = 4           // cycles before ack_o fires
  )
  (input  logic                    clk_i
  ,input  logic                    reset_i

  // Bus-side interface
  ,input  bus_trans_e              trans_i     // transaction type
  ,input  logic [ADDR_WIDTH_P-1:0] addr_i      // request address
  ,input  logic [DATA_WIDTH_P-1:0] data_i      // write data (BUS_FLUSH only)
  ,output logic [DATA_WIDTH_P-1:0] data_o      // read data
  ,output logic                    ack_o        // 1-cycle pulse when done
  );

  // ---------------------------------------------------------------------------
  // Localparams
  // ---------------------------------------------------------------------------
  localparam int MEM_ADDR_WIDTH_LP = $clog2(MEM_DEPTH_P);
  localparam int WORD_OFFSET_LP    = $clog2(DATA_WIDTH_P / 8); // byte→word shift

  // ---------------------------------------------------------------------------
  // Memory array
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH_P-1:0] mem_r [0:MEM_DEPTH_P-1];

  // Word-aligned index into the array (drop byte-offset bits)
  logic [MEM_ADDR_WIDTH_LP-1:0] word_addr;
  assign word_addr = addr_i[MEM_ADDR_WIDTH_LP + WORD_OFFSET_LP - 1 : WORD_OFFSET_LP];

  // ---------------------------------------------------------------------------
  // Request qualification
  // ---------------------------------------------------------------------------
  logic is_read;
  logic is_write;

  always_comb begin
    is_read  = (trans_i == BUS_RD) | (trans_i == BUS_RDX);
    is_write = (trans_i == BUS_FLUSH);
  end

  // ---------------------------------------------------------------------------
  // Latency counter
  // Counts down from MEM_LATENCY_P to 0 once a request arrives.
  // When it reaches 1 the ack fires on the next posedge.
  // ---------------------------------------------------------------------------
  localparam int CTR_WIDTH_LP = $clog2(MEM_LATENCY_P + 1);

  logic [CTR_WIDTH_LP-1:0] ctr_r, ctr_n;
  logic                     busy_r;          // high while request is in-flight
  logic                     busy_n;

  // Latch the incoming request while the counter runs
  logic [ADDR_WIDTH_P-1:0] req_addr_r;
  logic [DATA_WIDTH_P-1:0] req_data_r;
  logic                     req_write_r;

  always_comb begin
    // Default: hold state
    ctr_n   = ctr_r;
    busy_n  = busy_r;

    if (!busy_r) begin
      // Accept a new request only when idle
      if (is_read | is_write) begin
        ctr_n  = CTR_WIDTH_LP'(MEM_LATENCY_P);
        busy_n = 1'b1;
      end
    end else begin
      // Count down
      ctr_n = ctr_r - 1'b1;
      if (ctr_r == '0)
        busy_n = 1'b0;
    end
  end

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      ctr_r  <= '0;
      busy_r <= 1'b0;
    end else begin
      ctr_r  <= ctr_n;
      busy_r <= busy_n;
    end
  end

  // Capture request on acceptance
  always_ff @(posedge clk_i) begin
    if (!busy_r && (is_read | is_write)) begin
      req_addr_r  <= addr_i;
      req_data_r  <= data_i;
      req_write_r <= is_write;
    end
  end

  // ---------------------------------------------------------------------------
  // Memory read / write
  // Fires exactly when the counter expires (ctr_r == 1 → ctr_n == 0)
  // ---------------------------------------------------------------------------
  logic [MEM_ADDR_WIDTH_LP-1:0] req_word_addr;
  assign req_word_addr = req_addr_r[MEM_ADDR_WIDTH_LP + WORD_OFFSET_LP - 1 : WORD_OFFSET_LP];

  logic do_service;
  assign do_service = busy_r && (ctr_r == '0);

  // Synchronous write
  always_ff @(posedge clk_i) begin
    if (do_service && req_write_r)
      mem_r[req_word_addr] <= req_data_r;
  end

  // Registered read-data and ack
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      data_o <= '0;
      ack_o  <= 1'b0;
    end else begin
      ack_o  <= 1'b0;         // default: de-assert every cycle
      data_o <= '0;

      if (do_service) begin
        ack_o <= 1'b1;
        if (!req_write_r)
          data_o <= mem_r[req_word_addr];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Simulation-only: initialise memory to a known pattern so uninitialised
  // reads show up as 0xDEADBEEF rather than X (easier to spot in waveforms).
  // The `ifdef guard keeps this out of synthesis.
  // ---------------------------------------------------------------------------
  `ifdef SIMULATION
  initial begin
    for (int i = 0; i < MEM_DEPTH_P; i++)
      mem_r[i] = 32'hDEAD_BEEF;
  end
  `endif

endmodule
