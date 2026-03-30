// =============================================================================
// mesi_bus_arbiter.sv
//
// Central bus arbiter for the MESI cache coherence protocol.
//
// Responsibilities
//   1. ARBITRATION  – picks one requester per transaction (round-robin).
//   2. SNOOP PHASE  – broadcasts the winning transaction + address to every
//                     cache so each one can react (invalidate, flush, supply).
//   3. INTERVENTION – if a snooping cache signals it owns the line in M-state
//                     it flushes the data; the arbiter steers that data to the
//                     requester AND to main memory.
//   4. MEMORY PHASE – if no cache intervenes the arbiter forwards the request
//                     to main memory and waits for ack.
//   5. COMPLETION   – pulses trans_done_o for one cycle so the granted cache
//                     knows it may capture the returned data and release the bus.
//
// FSM states
//   IDLE      – bus is free; watching for requests
//   SNOOP     – transaction broadcast to all caches; waiting for snoop_done_i
//   MEM_WAIT  – forwarded to memory; waiting for mem_ack_i
//   COMPLETE  – one-cycle pulse to signal end of transaction
//
// Port naming (BSG standard)
//   _i  input   _o  output
//   _r  register output   _n  next-state wire
//   _p  parameter         _lp localparam
//   _e  enum type
//
// =============================================================================

import mesi_pkg::*;

module mesi_bus_arbiter
  #(parameter int NUM_CACHES_P  = 4
   ,parameter int ADDR_WIDTH_P  = mesi_pkg::ADDR_WIDTH_P
   ,parameter int DATA_WIDTH_P  = mesi_pkg::DATA_WIDTH_P
   ,parameter int SNOOP_WAIT_P  = 2    // cycles to collect snoop responses
  )
  (input  logic                     clk_i
  ,input  logic                     reset_i

  // ------------------------------------------------------------------
  // Cache-side request interface  (one signal per cache, indexed [N-1:0])
  // ------------------------------------------------------------------
  ,input  logic  [NUM_CACHES_P-1:0]  req_i          // cache asserts to request bus
  ,input  bus_trans_e                bus_trans_i [NUM_CACHES_P-1:0] // what each cache wants
  ,input  logic  [ADDR_WIDTH_P-1:0]  bus_addr_i  [NUM_CACHES_P-1:0] // address per cache
  ,input  logic  [DATA_WIDTH_P-1:0]  bus_data_i  [NUM_CACHES_P-1:0] // data (for BUS_FLUSH)

  ,output logic  [NUM_CACHES_P-1:0]  gnt_o          // one-hot grant to winner
  ,output logic                      bus_valid_o    // broadcast is live this cycle
  ,output bus_trans_e                snoop_trans_o  // transaction broadcast to all
  ,output logic  [ADDR_WIDTH_P-1:0]  snoop_addr_o   // address broadcast to all
  ,output logic  [DATA_WIDTH_P-1:0]  snoop_data_o   // data broadcast (read response)
  ,output logic  [NUM_CACHES_P-1:0]  trans_done_o   // one-cycle pulse per cache

  // ------------------------------------------------------------------
  // Snoop response interface  (caches drive these during snoop phase)
  // ------------------------------------------------------------------
  ,input  logic  [NUM_CACHES_P-1:0]  snoop_shared_i  // cache has line in S or E
  ,input  logic  [NUM_CACHES_P-1:0]  snoop_flush_i   // cache has line in M → flushing
  ,input  logic  [DATA_WIDTH_P-1:0]  flush_data_i [NUM_CACHES_P-1:0] // flushed data

  // ------------------------------------------------------------------
  // Memory interface
  // ------------------------------------------------------------------
  ,output bus_trans_e                mem_trans_o    // forwarded transaction
  ,output logic  [ADDR_WIDTH_P-1:0]  mem_addr_o     // forwarded address
  ,output logic  [DATA_WIDTH_P-1:0]  mem_data_o     // data to write (BUS_FLUSH)
  ,input  logic  [DATA_WIDTH_P-1:0]  mem_data_i     // data returned by memory
  ,input  logic                      mem_ack_i      // memory finished
  );

  // ---------------------------------------------------------------------------
  // FSM state encoding
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    IDLE     = 2'b00,
    SNOOP    = 2'b01,
    MEM_WAIT = 2'b10,
    COMPLETE = 2'b11
  } arb_state_e;

  arb_state_e state_r, state_n;

  // ---------------------------------------------------------------------------
  // Round-robin pointer
  // Points to the cache that gets priority this round.
  // ---------------------------------------------------------------------------
  localparam int PTR_WIDTH_LP = $clog2(NUM_CACHES_P);

  logic [PTR_WIDTH_LP-1:0] rr_ptr_r, rr_ptr_n;

  // ---------------------------------------------------------------------------
  // Registered snapshot of the winning transaction
  // (held stable for the entire duration of the transaction)
  // ---------------------------------------------------------------------------
  logic [PTR_WIDTH_LP-1:0] winner_r;
  bus_trans_e               win_trans_r;
  logic [ADDR_WIDTH_P-1:0]  win_addr_r;
  logic [DATA_WIDTH_P-1:0]  win_data_r;

  // ---------------------------------------------------------------------------
  // Snoop phase counter
  // Gives every cache SNOOP_WAIT_P cycles to assert snoop_flush_i / snoop_shared_i
  // ---------------------------------------------------------------------------
  localparam int SNOOP_CTR_WIDTH_LP = $clog2(SNOOP_WAIT_P + 1);
  logic [SNOOP_CTR_WIDTH_LP-1:0] snoop_ctr_r, snoop_ctr_n;

  // ---------------------------------------------------------------------------
  // Intervention detection
  // If any non-requester cache asserts snoop_flush_i, it owns the line in M.
  // ---------------------------------------------------------------------------
  logic                    intervention;      // combinational
  logic [PTR_WIDTH_LP-1:0] flush_src;         // which cache is flushing
  logic [DATA_WIDTH_P-1:0] flush_data_mux;    // data from the flushing cache

  always_comb begin
    intervention   = 1'b0;
    flush_src      = '0;
    flush_data_mux = '0;
    for (int k = 0; k < NUM_CACHES_P; k++) begin
      // Only non-requester caches can intervene
      if (snoop_flush_i[k] && (PTR_WIDTH_LP'(k) != winner_r)) begin
        intervention   = 1'b1;
        flush_src      = PTR_WIDTH_LP'(k);
        flush_data_mux = flush_data_i[k];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Arbitration: round-robin winner selection (combinational)
  // ---------------------------------------------------------------------------
  logic [NUM_CACHES_P-1:0]  gnt_n;
  logic [PTR_WIDTH_LP-1:0]  winner_n;
  logic                      any_req;

  always_comb begin
    gnt_n    = '0;
    winner_n = rr_ptr_r;
    any_req  = |req_i;

    if (any_req) begin
      // Scan from rr_ptr_r upward, wrap around
      for (int k = 0; k < NUM_CACHES_P; k++) begin
        automatic int idx = (rr_ptr_r + PTR_WIDTH_LP'(k)) % NUM_CACHES_P;
        if (req_i[idx] && (gnt_n == '0)) begin
          gnt_n[idx] = 1'b1;
          winner_n   = PTR_WIDTH_LP'(idx);
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // FSM next-state logic
  // ---------------------------------------------------------------------------
  always_comb begin
    // Defaults – hold everything
    state_n     = state_r;
    snoop_ctr_n = snoop_ctr_r;
    rr_ptr_n    = rr_ptr_r;

    unique case (state_r)

      IDLE: begin
        if (any_req) begin
          state_n     = SNOOP;
          snoop_ctr_n = SNOOP_CTR_WIDTH_LP'(SNOOP_WAIT_P);
          // Advance RR pointer past the winner for next time
          rr_ptr_n = PTR_WIDTH_LP'((winner_n + 1) % NUM_CACHES_P);
        end
      end

      SNOOP: begin
        if (snoop_ctr_r == '0) begin
          // Snoop window closed
          if (intervention) begin
            // Intervening cache supplies data; still need memory to absorb the
            // flush (write-back), but we can complete from the cache's data.
            // Memory write is issued below; go straight to COMPLETE.
            state_n = COMPLETE;
          end else if (win_trans_r == BUS_RD || win_trans_r == BUS_RDX) begin
            // No intervention → fetch from memory
            state_n = MEM_WAIT;
          end else begin
            // BUS_FLUSH or BUS_UPGR with no intervention → just complete
            state_n = COMPLETE;
          end
        end else begin
          snoop_ctr_n = snoop_ctr_r - 1'b1;
        end
      end

      MEM_WAIT: begin
        if (mem_ack_i)
          state_n = COMPLETE;
      end

      COMPLETE: begin
        state_n = IDLE;
      end

      default: state_n = IDLE;

    endcase
  end

  // ---------------------------------------------------------------------------
  // FSM state register + snapshot capture
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state_r    <= IDLE;
      rr_ptr_r   <= '0;
      snoop_ctr_r <= '0;
      winner_r   <= '0;
      win_trans_r <= BUS_NONE;
      win_addr_r  <= '0;
      win_data_r  <= '0;
    end else begin
      state_r     <= state_n;
      rr_ptr_r    <= rr_ptr_n;
      snoop_ctr_r <= snoop_ctr_n;

      // Capture winner snapshot when leaving IDLE
      if (state_r == IDLE && any_req) begin
        winner_r    <= winner_n;
        win_trans_r <= bus_trans_i[winner_n];
        win_addr_r  <= bus_addr_i [winner_n];
        win_data_r  <= bus_data_i [winner_n];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Grant register  (hold grant stable for entire transaction)
  // ---------------------------------------------------------------------------
  logic [NUM_CACHES_P-1:0] gnt_r;

  always_ff @(posedge clk_i) begin
    if (reset_i)
      gnt_r <= '0;
    else if (state_r == IDLE && any_req)
      gnt_r <= gnt_n;
    else if (state_r == COMPLETE)
      gnt_r <= '0;
  end

  assign gnt_o = gnt_r;

  // ---------------------------------------------------------------------------
  // Snoop broadcast outputs
  // Active throughout SNOOP and MEM_WAIT so caches see the transaction.
  // ---------------------------------------------------------------------------
  always_comb begin
    bus_valid_o   = (state_r == SNOOP) | (state_r == MEM_WAIT) | (state_r == COMPLETE);
    snoop_trans_o = win_trans_r;
    snoop_addr_o  = win_addr_r;
    // Data mux: intervention → cache flush data; mem done → memory data; else 0
    if (intervention && state_r == COMPLETE)
      snoop_data_o = flush_data_mux;
    else if (state_r == COMPLETE && !intervention)
      snoop_data_o = mem_data_i;
    else
      snoop_data_o = '0;
  end

  // ---------------------------------------------------------------------------
  // trans_done_o – one-cycle pulse to every cache when COMPLETE is reached
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (reset_i)
      trans_done_o <= '0;
    else if (state_r == COMPLETE)
      trans_done_o <= {NUM_CACHES_P{1'b1}};  // tell every cache we're done
    else
      trans_done_o <= '0;
  end

  // ---------------------------------------------------------------------------
  // Memory interface
  // Issue a memory request during SNOOP (last cycle) or MEM_WAIT entry.
  // ---------------------------------------------------------------------------
  always_comb begin
    mem_addr_o  = win_addr_r;
    mem_data_o  = '0;
    mem_trans_o = BUS_NONE;

    unique case (state_r)
      SNOOP: begin
        if (snoop_ctr_r == '0) begin
          if (intervention) begin
            // Write flushed data to memory
            mem_trans_o = BUS_FLUSH;
            mem_data_o  = flush_data_mux;
          end else if (win_trans_r == BUS_RD || win_trans_r == BUS_RDX) begin
            mem_trans_o = win_trans_r;
          end else if (win_trans_r == BUS_FLUSH) begin
            mem_trans_o = BUS_FLUSH;
            mem_data_o  = win_data_r;
          end
        end
      end
      MEM_WAIT: begin
        // Keep driving memory request until ack
        if (!mem_ack_i) begin
          mem_trans_o = win_trans_r;
          mem_data_o  = win_data_r;
        end
      end
      default: begin
        mem_trans_o = BUS_NONE;
      end
    endcase
  end

  // ---------------------------------------------------------------------------
  // Simulation assertions – catch protocol violations early
  // ---------------------------------------------------------------------------
  `ifdef SIMULATION

  // Only one cache should ever flush at a time
  always_ff @(posedge clk_i) begin
    if (!reset_i && state_r == SNOOP) begin
      assert ($onehot0(snoop_flush_i))
        else $error("[arbiter] More than one cache asserted snoop_flush_i simultaneously!");
    end
  end

  // Grant should always be one-hot or zero
  always_ff @(posedge clk_i) begin
    if (!reset_i)
      assert ($onehot0(gnt_o))
        else $error("[arbiter] Grant is not one-hot!");
  end

  `endif

endmodule
