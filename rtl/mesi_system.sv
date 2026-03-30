// =============================================================================
// mesi_system.sv
//
// Top-level: four MESI cache controllers + one bus arbiter + main memory.
//
//   ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
//   │ Cache 0  │  │ Cache 1  │  │ Cache 2  │  │ Cache 3  │
//   └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘
//        │              │              │              │
//        └──────────────┴──────────────┴──────────────┘
//                              │  shared bus
//                       ┌──────┴──────┐
//                       │ Bus Arbiter │
//                       └──────┬──────┘
//                              │
//                       ┌──────┴──────┐
//                       │ Main Memory │
//                       └─────────────┘
//
// Processor ports are exposed as flat arrays indexed [NUM_CACHES_P-1:0]
// so the testbench can drive each processor independently.
// =============================================================================

import mesi_pkg::*;

module mesi_system
  #(parameter int NUM_CACHES_P   = 4
   ,parameter int ADDR_WIDTH_P   = mesi_pkg::ADDR_WIDTH_P
   ,parameter int DATA_WIDTH_P   = mesi_pkg::DATA_WIDTH_P
   ,parameter int MEM_DEPTH_P    = 1024
   ,parameter int MEM_LATENCY_P  = 4
   ,parameter int SNOOP_WAIT_P   = 2
  )
  (input  logic                    clk_i
  ,input  logic                    reset_i

  // ------------------------------------------------------------------
  // Processor interfaces (one per cache, flat arrays)
  // ------------------------------------------------------------------
  ,input  proc_req_e               proc_req_i   [NUM_CACHES_P-1:0]
  ,input  logic [ADDR_WIDTH_P-1:0] proc_addr_i  [NUM_CACHES_P-1:0]
  ,input  logic [DATA_WIDTH_P-1:0] proc_wdata_i [NUM_CACHES_P-1:0]
  ,output logic [DATA_WIDTH_P-1:0] proc_rdata_o [NUM_CACHES_P-1:0]
  ,output logic                    proc_stall_o [NUM_CACHES_P-1:0]
  ,output logic                    proc_ack_o   [NUM_CACHES_P-1:0]
  );

  // ---------------------------------------------------------------------------
  // Internal wires: Cache → Arbiter
  // ---------------------------------------------------------------------------
  logic       req        [NUM_CACHES_P-1:0];
  logic       gnt        [NUM_CACHES_P-1:0];
  bus_trans_e bus_trans  [NUM_CACHES_P-1:0];
  logic [ADDR_WIDTH_P-1:0] bus_addr [NUM_CACHES_P-1:0];
  logic [DATA_WIDTH_P-1:0] bus_data [NUM_CACHES_P-1:0];

  // Flatten req/gnt to logic vectors for the arbiter port
  logic [NUM_CACHES_P-1:0] req_vec, gnt_vec;
  always_comb begin
    for (int k = 0; k < NUM_CACHES_P; k++) begin
      req_vec[k] = req[k];
      gnt[k]     = gnt_vec[k];
    end
  end

  // ---------------------------------------------------------------------------
  // Internal wires: Arbiter → Caches (broadcast)
  // ---------------------------------------------------------------------------
  logic                    bus_valid;
  bus_trans_e              snoop_trans;
  logic [ADDR_WIDTH_P-1:0] snoop_addr;
  logic [DATA_WIDTH_P-1:0] snoop_data;
  logic [NUM_CACHES_P-1:0] trans_done_vec;
  logic                    trans_done [NUM_CACHES_P-1:0];

  always_comb begin
    for (int k = 0; k < NUM_CACHES_P; k++)
      trans_done[k] = trans_done_vec[k];
  end

  // ---------------------------------------------------------------------------
  // Internal wires: Caches → Arbiter (snoop responses)
  // ---------------------------------------------------------------------------
  logic                    snoop_shared [NUM_CACHES_P-1:0];
  logic                    snoop_flush  [NUM_CACHES_P-1:0];
  logic [DATA_WIDTH_P-1:0] flush_data   [NUM_CACHES_P-1:0];

  logic [NUM_CACHES_P-1:0] snoop_shared_vec, snoop_flush_vec;
  always_comb begin
    for (int k = 0; k < NUM_CACHES_P; k++) begin
      snoop_shared_vec[k] = snoop_shared[k];
      snoop_flush_vec [k] = snoop_flush [k];
    end
  end

  // ---------------------------------------------------------------------------
  // Internal wires: Arbiter → Memory
  // ---------------------------------------------------------------------------
  bus_trans_e              mem_trans;
  logic [ADDR_WIDTH_P-1:0] mem_addr;
  logic [DATA_WIDTH_P-1:0] mem_wdata;
  logic [DATA_WIDTH_P-1:0] mem_rdata;
  logic                    mem_ack;

  // ---------------------------------------------------------------------------
  // Instantiate cache controllers
  // ---------------------------------------------------------------------------
  genvar gi;
  generate
    for (gi = 0; gi < NUM_CACHES_P; gi++) begin : gen_caches

      mesi_cache_ctrl #(
         .NUM_CACHES_P  (NUM_CACHES_P)
        ,.CACHE_ID_P    (gi)
        ,.ADDR_WIDTH_P  (ADDR_WIDTH_P)
        ,.DATA_WIDTH_P  (DATA_WIDTH_P)
      ) cache_ctrl (
         .clk_i         (clk_i)
        ,.reset_i        (reset_i)
        // Processor
        ,.proc_req_i     (proc_req_i  [gi])
        ,.proc_addr_i    (proc_addr_i [gi])
        ,.proc_wdata_i   (proc_wdata_i[gi])
        ,.proc_rdata_o   (proc_rdata_o[gi])
        ,.proc_stall_o   (proc_stall_o[gi])
        ,.proc_ack_o     (proc_ack_o  [gi])
        // Bus request/grant
        ,.req_o          (req      [gi])
        ,.gnt_i          (gnt      [gi])
        ,.bus_trans_o    (bus_trans[gi])
        ,.bus_addr_o     (bus_addr [gi])
        ,.bus_data_o     (bus_data [gi])
        // Bus broadcast (snoop inputs)
        ,.bus_valid_i    (bus_valid)
        ,.snoop_trans_i  (snoop_trans)
        ,.snoop_addr_i   (snoop_addr)
        ,.snoop_data_i   (snoop_data)
        ,.trans_done_i   (trans_done[gi])
        // Snoop responses
        ,.snoop_shared_o (snoop_shared[gi])
        ,.snoop_flush_o  (snoop_flush [gi])
        ,.flush_data_o   (flush_data  [gi])
      );

    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Instantiate bus arbiter
  // ---------------------------------------------------------------------------
  mesi_bus_arbiter #(
     .NUM_CACHES_P (NUM_CACHES_P)
    ,.ADDR_WIDTH_P (ADDR_WIDTH_P)
    ,.DATA_WIDTH_P (DATA_WIDTH_P)
    ,.SNOOP_WAIT_P (SNOOP_WAIT_P)
  ) arbiter (
     .clk_i         (clk_i)
    ,.reset_i        (reset_i)
    // Cache requests
    ,.req_i          (req_vec)
    ,.bus_trans_i    (bus_trans)
    ,.bus_addr_i     (bus_addr)
    ,.bus_data_i     (bus_data)
    ,.gnt_o          (gnt_vec)
    // Broadcast
    ,.bus_valid_o    (bus_valid)
    ,.snoop_trans_o  (snoop_trans)
    ,.snoop_addr_o   (snoop_addr)
    ,.snoop_data_o   (snoop_data)
    ,.trans_done_o   (trans_done_vec)
    // Snoop responses
    ,.snoop_shared_i (snoop_shared_vec)
    ,.snoop_flush_i  (snoop_flush_vec)
    ,.flush_data_i   (flush_data)
    // Memory
    ,.mem_trans_o    (mem_trans)
    ,.mem_addr_o     (mem_addr)
    ,.mem_data_o     (mem_wdata)
    ,.mem_data_i     (mem_rdata)
    ,.mem_ack_i      (mem_ack)
  );

  // ---------------------------------------------------------------------------
  // Instantiate main memory
  // ---------------------------------------------------------------------------
  mesi_main_mem #(
     .MEM_DEPTH_P  (MEM_DEPTH_P)
    ,.DATA_WIDTH_P (DATA_WIDTH_P)
    ,.ADDR_WIDTH_P (ADDR_WIDTH_P)
    ,.MEM_LATENCY_P(MEM_LATENCY_P)
  ) main_mem (
     .clk_i    (clk_i)
    ,.reset_i   (reset_i)
    ,.trans_i   (mem_trans)
    ,.addr_i    (mem_addr)
    ,.data_i    (mem_wdata)
    ,.data_o    (mem_rdata)
    ,.ack_o     (mem_ack)
  );

endmodule
