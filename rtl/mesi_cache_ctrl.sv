// =============================================================================
// mesi_cache_ctrl.sv
//
// Single-cache MESI cache controller.
//
// Instantiate four of these (one per processor) and connect them to the same
// mesi_bus_arbiter and mesi_main_mem.
//
// Internal structure
//   Direct-mapped cache:
//     tag_r   [LINES_LP]          – stored tag per line
//     data_r  [LINES_LP]          – cached data word
//     state_r [LINES_LP]          – MESI state per line (reset → I)
//
// FSM states
//   CC_IDLE       – ready; hit → serve processor; miss → request bus
//   CC_WAIT_GRANT – waiting for arbiter to assert gnt_i
//   CC_WAIT_DONE  – grant received; waiting for trans_done_i pulse + data
//   CC_WRITEBACK  – evicting a dirty (M) line before serving a new miss
//
// MESI transitions (Table 7.2 notation)
//   Processor side:
//     I + PrRd  → BUS_RD  → S (shared) or E (exclusive, no other copy)
//     I + PrWr  → BUS_RDX → M
//     S + PrRd  → hit, stay S
//     S + PrWr  → BUS_UPGR → M
//     E + PrRd  → hit, stay E
//     E + PrWr  → silent upgrade → M  (no bus traffic)
//     M + PrRd  → hit, stay M
//     M + PrWr  → hit, stay M
//
//   Snoop side (other cache's transaction seen on bus):
//     M + BUS_RD  → flush → S
//     M + BUS_RDX → flush → I
//     E + BUS_RD  → S
//     E + BUS_RDX → I
//     S + BUS_RD  → S
//     S + BUS_RDX → I
//     S + BUS_UPGR→ I
//     I + any     → stay I
//
// Port naming: BSG standard (_i/_o/_r/_n/_p/_lp/_e)
// =============================================================================

import mesi_pkg::*;

module mesi_cache_ctrl
  #(parameter int NUM_CACHES_P   = 4
   ,parameter int CACHE_ID_P     = 0                      // which cache this is
   ,parameter int ADDR_WIDTH_P   = mesi_pkg::ADDR_WIDTH_P
   ,parameter int DATA_WIDTH_P   = mesi_pkg::DATA_WIDTH_P
   ,parameter int INDEX_WIDTH_P  = mesi_pkg::INDEX_WIDTH_P
   ,parameter int TAG_WIDTH_P    = mesi_pkg::TAG_WIDTH_P
   ,parameter int OFFSET_WIDTH_P = mesi_pkg::OFFSET_WIDTH_P
  )
  (input  logic                     clk_i
  ,input  logic                     reset_i

  // ------------------------------------------------------------------
  // Processor interface
  // ------------------------------------------------------------------
  ,input  proc_req_e                proc_req_i    // PrRd / PrWr / PR_NONE
  ,input  logic [ADDR_WIDTH_P-1:0]  proc_addr_i   // byte address
  ,input  logic [DATA_WIDTH_P-1:0]  proc_wdata_i  // write data (PrWr only)
  ,output logic [DATA_WIDTH_P-1:0]  proc_rdata_o  // read data (PrRd hit/fill)
  ,output logic                     proc_stall_o  // high → processor must wait
  ,output logic                     proc_ack_o    // one-cycle pulse: request done

  // ------------------------------------------------------------------
  // Bus arbiter – request / grant
  // ------------------------------------------------------------------
  ,output logic                     req_o         // this cache wants the bus
  ,input  logic                     gnt_i         // arbiter granted the bus
  ,output bus_trans_e               bus_trans_o   // transaction we're issuing
  ,output logic [ADDR_WIDTH_P-1:0]  bus_addr_o    // address we're issuing
  ,output logic [DATA_WIDTH_P-1:0]  bus_data_o    // data for BUS_FLUSH

  // ------------------------------------------------------------------
  // Bus broadcast (snoop inputs – driven by arbiter)
  // ------------------------------------------------------------------
  ,input  logic                     bus_valid_i   // a transaction is live
  ,input  bus_trans_e               snoop_trans_i // what's happening on bus
  ,input  logic [ADDR_WIDTH_P-1:0]  snoop_addr_i  // address on bus
  ,input  logic [DATA_WIDTH_P-1:0]  snoop_data_i  // data returned (fill/flush)
  ,input  logic                     trans_done_i  // arbiter: transaction complete

  // ------------------------------------------------------------------
  // Snoop response outputs (back to arbiter)
  // ------------------------------------------------------------------
  ,output logic                     snoop_shared_o // we have line in S or E
  ,output logic                     snoop_flush_o  // we have line in M → flushing
  ,output logic [DATA_WIDTH_P-1:0]  flush_data_o   // data we're flushing
  );

  // ---------------------------------------------------------------------------
  // Localparams
  // ---------------------------------------------------------------------------
  localparam int LINES_LP      = 2 ** INDEX_WIDTH_P;
  localparam int BYTE_BITS_LP  = $clog2(DATA_WIDTH_P / 8);

  // Address field extraction helpers
  // [ ADDR_WIDTH-1 : TAG+IDX+OFF ]  (upper bits ignored / not used in this demo)
  // [ TAG_WIDTH+INDEX_WIDTH+OFFSET_WIDTH-1 : INDEX_WIDTH+OFFSET_WIDTH ] → tag
  // [ INDEX_WIDTH+OFFSET_WIDTH-1            : OFFSET_WIDTH ]             → index
  // [ OFFSET_WIDTH-1                        : 0 ]                        → offset (ignored)

  localparam int OFF_HI_LP = OFFSET_WIDTH_P - 1;
  localparam int IDX_LO_LP = OFFSET_WIDTH_P;
  localparam int IDX_HI_LP = OFFSET_WIDTH_P + INDEX_WIDTH_P - 1;
  localparam int TAG_LO_LP = OFFSET_WIDTH_P + INDEX_WIDTH_P;
  localparam int TAG_HI_LP = OFFSET_WIDTH_P + INDEX_WIDTH_P + TAG_WIDTH_P - 1;

  // ---------------------------------------------------------------------------
  // Cache arrays  (registered – actual flip-flop arrays)
  // ---------------------------------------------------------------------------
  mesi_state_e              state_r [0:LINES_LP-1];
  logic [TAG_WIDTH_P-1:0]   tag_r   [0:LINES_LP-1];
  logic [DATA_WIDTH_P-1:0]  data_r  [0:LINES_LP-1];

  // ---------------------------------------------------------------------------
  // Address decode – processor request
  // ---------------------------------------------------------------------------
  logic [TAG_WIDTH_P-1:0]   proc_tag;
  logic [INDEX_WIDTH_P-1:0] proc_idx;

  assign proc_tag = proc_addr_i[TAG_HI_LP : TAG_LO_LP];
  assign proc_idx = proc_addr_i[IDX_HI_LP : IDX_LO_LP];

  // Address decode – snooped address
  logic [TAG_WIDTH_P-1:0]   snoop_tag;
  logic [INDEX_WIDTH_P-1:0] snoop_idx;

  assign snoop_tag = snoop_addr_i[TAG_HI_LP : TAG_LO_LP];
  assign snoop_idx = snoop_addr_i[IDX_HI_LP : IDX_LO_LP];

  // ---------------------------------------------------------------------------
  // Hit detection – processor side
  // ---------------------------------------------------------------------------
  logic proc_hit;
  assign proc_hit = (state_r[proc_idx] != I) &&
                    (tag_r[proc_idx]   == proc_tag);

  // ---------------------------------------------------------------------------
  // Hit detection – snoop side
  // (is the snooped address resident and valid in our cache?)
  // ---------------------------------------------------------------------------
  logic snoop_hit;
  assign snoop_hit = (state_r[snoop_idx] != I) &&
                     (tag_r[snoop_idx]   == snoop_tag);

  // ---------------------------------------------------------------------------
  // FSM state encoding
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    CC_IDLE       = 2'b00,
    CC_WAIT_GRANT = 2'b01,
    CC_WAIT_DONE  = 2'b10,
    CC_WRITEBACK  = 2'b11
  } cc_state_e;

  cc_state_e cc_state_r, cc_state_n;

  // ---------------------------------------------------------------------------
  // Pending request registers
  // (held stable while we wait for bus grant and data)
  // ---------------------------------------------------------------------------
  proc_req_e               pend_req_r;
  logic [ADDR_WIDTH_P-1:0] pend_addr_r;
  logic [DATA_WIDTH_P-1:0] pend_wdata_r;
  logic [TAG_WIDTH_P-1:0]  pend_tag_r;
  logic [INDEX_WIDTH_P-1:0]pend_idx_r;

  // Eviction registers (when we must writeback an M-line before filling)
  logic [ADDR_WIDTH_P-1:0] evict_addr_r;
  logic [DATA_WIDTH_P-1:0] evict_data_r;

  // Bus transaction we are driving
  bus_trans_e               bus_trans_r;

  // ---------------------------------------------------------------------------
  // Snoop response logic (combinational)
  // Responds within the same cycle the bus broadcast is seen.
  // ---------------------------------------------------------------------------
  always_comb begin
    snoop_shared_o = 1'b0;
    snoop_flush_o  = 1'b0;
    flush_data_o   = '0;

    if (bus_valid_i && snoop_hit) begin
      unique case (state_r[snoop_idx])

        M: begin
          // We own a dirty copy – must flush to satisfy the requester
          snoop_flush_o = 1'b1;
          flush_data_o  = data_r[snoop_idx];
        end

        E, S: begin
          snoop_shared_o = 1'b1;  // signal we have a clean copy
        end

        I: begin
          // Nothing to report
        end

        default: begin end

      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // FSM next-state logic  (combinational)
  // ---------------------------------------------------------------------------
  always_comb begin
    cc_state_n = cc_state_r;   // default: hold

    unique case (cc_state_r)

      CC_IDLE: begin
        if (proc_req_i != PR_NONE) begin
          if (proc_hit) begin
            // Hit – serve immediately; special case: E/S + PrWr needs silent upgrade
            // (E→M needs no bus, S→M needs BUS_UPGR)
            if (proc_req_i == PR_WR &&
                state_r[proc_idx] == S) begin
              cc_state_n = CC_WAIT_GRANT;  // need bus for UPGR
            end
            // E→M is silent; M/E + PrRd is a plain hit → stay IDLE
          end else begin
            // Miss – must go to bus
            // If the line being replaced is dirty → writeback first
            if (state_r[proc_idx] == M)
              cc_state_n = CC_WRITEBACK;
            else
              cc_state_n = CC_WAIT_GRANT;
          end
        end
      end

      CC_WRITEBACK: begin
        // Wait for bus to complete the eviction flush
        if (trans_done_i)
          cc_state_n = CC_WAIT_GRANT;
      end

      CC_WAIT_GRANT: begin
        if (gnt_i)
          cc_state_n = CC_WAIT_DONE;
      end

      CC_WAIT_DONE: begin
        if (trans_done_i)
          cc_state_n = CC_IDLE;
      end

      default: cc_state_n = CC_IDLE;

    endcase
  end

  // ---------------------------------------------------------------------------
  // FSM state register + pending request capture
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      cc_state_r   <= CC_IDLE;
      pend_req_r   <= PR_NONE;
      pend_addr_r  <= '0;
      pend_wdata_r <= '0;
      pend_tag_r   <= '0;
      pend_idx_r   <= '0;
      bus_trans_r  <= BUS_NONE;
      evict_addr_r <= '0;
      evict_data_r <= '0;
    end else begin
      cc_state_r <= cc_state_n;

      // Capture processor request when we first leave IDLE
      if (cc_state_r == CC_IDLE && proc_req_i != PR_NONE) begin
        pend_req_r   <= proc_req_i;
        pend_addr_r  <= proc_addr_i;
        pend_wdata_r <= proc_wdata_i;
        pend_tag_r   <= proc_tag;
        pend_idx_r   <= proc_idx;

        // Decide what bus transaction we'll issue
        if (!proc_hit) begin
          // Miss
          bus_trans_r <= (proc_req_i == PR_WR) ? BUS_RDX : BUS_RD;
          // If evicting an M-line, save eviction info
          if (state_r[proc_idx] == M) begin
            // Reconstruct eviction address from stored tag + current index
            evict_addr_r <= {tag_r[proc_idx]
                            ,proc_idx
                            ,{OFFSET_WIDTH_P{1'b0}}};
            evict_data_r <= data_r[proc_idx];
          end
        end else begin
          // Hit cases that need the bus
          bus_trans_r <= BUS_UPGR;  // only S+PrWr hits reach here
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Cache array updates
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      for (int i = 0; i < LINES_LP; i++) begin
        state_r[i] <= I;
        tag_r[i]   <= '0;
        data_r[i]  <= '0;
      end
    end else begin

      // ---- Processor hit updates (CC_IDLE, this cycle) --------------------
      if (cc_state_r == CC_IDLE && proc_req_i != PR_NONE && proc_hit) begin

        unique case ({state_r[proc_idx], proc_req_i})

          // E + PrWr → silent upgrade M
          {E, PR_WR}: begin
            state_r[proc_idx] <= M;
            data_r [proc_idx] <= proc_wdata_i;
          end

          // M + PrWr → stay M, update data
          {M, PR_WR}: begin
            data_r[proc_idx] <= proc_wdata_i;
          end

          // S/E/M + PrRd → no state change (reads don't dirty)
          default: begin end

        endcase
      end

      // ---- Fill on BUS_RD / BUS_RDX completion ---------------------------
      if (cc_state_r == CC_WAIT_DONE && trans_done_i) begin

        tag_r  [pend_idx_r] <= pend_tag_r;

        if (pend_req_r == PR_WR) begin
          // We got exclusive ownership → write our data and mark M
          data_r [pend_idx_r] <= pend_wdata_r;
          state_r[pend_idx_r] <= M;
        end else begin
          // Read fill → store returned data
          data_r[pend_idx_r] <= snoop_data_i;
          // E if no other cache signalled shared; otherwise S
          // (arbiter broadcasts snoop_shared → we use that info via snoop_data_i
          //  In this version we conservatively go to S when other caches exist;
          //  a real implementation checks the shared line from the arbiter.)
          state_r[pend_idx_r] <= S;
        end
      end

      // ---- BUS_UPGR completion (S→M) -------------------------------------
      if (cc_state_r == CC_WAIT_DONE && trans_done_i &&
          bus_trans_r == BUS_UPGR) begin
        data_r [pend_idx_r] <= pend_wdata_r;
        state_r[pend_idx_r] <= M;
      end

      // ---- Snoop-triggered state transitions (can happen any time) --------
      if (bus_valid_i && snoop_hit &&
          // Don't update our own transaction (we're the requester)
          !(cc_state_r == CC_WAIT_DONE)) begin

        unique case (snoop_trans_i)

          BUS_RD: begin
            unique case (state_r[snoop_idx])
              M: begin
                // We flushed; requester and memory get the data; we go S
                state_r[snoop_idx] <= S;
              end
              E: state_r[snoop_idx] <= S;
              S: begin end   // stay S
              I: begin end
              default: begin end
            endcase
          end

          BUS_RDX: begin
            unique case (state_r[snoop_idx])
              M: state_r[snoop_idx] <= I;  // flushed + invalidated
              E: state_r[snoop_idx] <= I;
              S: state_r[snoop_idx] <= I;
              I: begin end
              default: begin end
            endcase
          end

          BUS_UPGR: begin
            // Another cache upgrading S→M; we must invalidate our S copy
            if (state_r[snoop_idx] == S)
              state_r[snoop_idx] <= I;
          end

          BUS_FLUSH, BUS_NONE: begin end

          default: begin end

        endcase
      end

    end // !reset
  end // always_ff

  // ---------------------------------------------------------------------------
  // Bus request and transaction outputs
  // ---------------------------------------------------------------------------
  always_comb begin
    req_o        = 1'b0;
    bus_trans_o  = BUS_NONE;
    bus_addr_o   = '0;
    bus_data_o   = '0;

    unique case (cc_state_r)

      CC_WRITEBACK: begin
        req_o       = 1'b1;
        bus_trans_o = BUS_FLUSH;
        bus_addr_o  = evict_addr_r;
        bus_data_o  = evict_data_r;
      end

      CC_WAIT_GRANT, CC_WAIT_DONE: begin
        req_o       = (cc_state_r == CC_WAIT_GRANT);
        bus_trans_o = bus_trans_r;
        bus_addr_o  = pend_addr_r;
        bus_data_o  = pend_wdata_r;   // only relevant for BUS_FLUSH (not used here)
      end

      default: begin end

    endcase
  end

  // ---------------------------------------------------------------------------
  // Processor response outputs
  // ---------------------------------------------------------------------------
  always_comb begin
    proc_stall_o = 1'b0;
    proc_ack_o   = 1'b0;
    proc_rdata_o = '0;

    unique case (cc_state_r)

      CC_IDLE: begin
        if (proc_req_i != PR_NONE) begin
          if (proc_hit) begin
            // Hit: respond this cycle
            proc_ack_o   = 1'b1;
            proc_rdata_o = data_r[proc_idx];
            proc_stall_o = 1'b0;
          end else begin
            // Miss: stall processor
            proc_stall_o = 1'b1;
          end
        end
      end

      CC_WRITEBACK,
      CC_WAIT_GRANT: begin
        proc_stall_o = 1'b1;
      end

      CC_WAIT_DONE: begin
        if (trans_done_i) begin
          // Fill arrived
          proc_ack_o   = 1'b1;
          proc_stall_o = 1'b0;
          proc_rdata_o = (pend_req_r == PR_RD) ? snoop_data_i : pend_wdata_r;
        end else begin
          proc_stall_o = 1'b1;
        end
      end

      default: begin end

    endcase
  end

  // ---------------------------------------------------------------------------
  // Simulation assertions
  // ---------------------------------------------------------------------------
  `ifdef SIMULATION

  // A line must never be in M in two caches simultaneously.
  // (This is checked at the system level in the testbench, not here,
  //  but we guard against our own FSM going wrong.)
  always_ff @(posedge clk_i) begin
    if (!reset_i) begin
      assert (!(snoop_flush_o && !snoop_hit))
        else $error("[cache %0d] snoop_flush asserted but snoop_hit is 0!", CACHE_ID_P);

      assert (!(cc_state_r == CC_WAIT_DONE && bus_trans_r == BUS_NONE))
        else $error("[cache %0d] in CC_WAIT_DONE with BUS_NONE transaction!", CACHE_ID_P);
    end
  end

  `endif

endmodule
