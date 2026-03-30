// =============================================================================
// mesi_tb.sv
//
// Directed SystemVerilog testbench for the MESI cache coherence system.
//
// Walks through the classic Table 7.2 reference sequence:
//
//   Step | Proc | Op    | Expected bus   | Expected final states
//   -----|------|-------|----------------|-------------------------------
//    1   |  P0  | Read  | BUS_RD         | P0:E,  P1:I, P2:I, P3:I
//    2   |  P0  | Write | (silent E→M)   | P0:M,  P1:I, P2:I, P3:I
//    3   |  P2  | Read  | BUS_RD+FLUSH   | P0:S,  P1:I, P2:S, P3:I
//    4   |  P2  | Write | BUS_UPGR       | P0:I,  P1:I, P2:M, P3:I
//    5   |  P0  | Read  | BUS_RD+FLUSH   | P0:S,  P1:I, P2:S, P3:I
//    6   |  P2  | Read  | (hit)          | P0:S,  P1:I, P2:S, P3:I
//    7   |  P1  | Read  | BUS_RD         | P0:S,  P1:S, P2:S, P3:I
//
// (Processors are 0-indexed here; textbook uses 1-indexed P1..P4.)
//
// How to run (Questa / VCS / Xcelium):
//   vlog -sv mesi_pkg.sv mesi_main_mem.sv mesi_bus_arbiter.sv \
//             mesi_cache_ctrl.sv mesi_system.sv mesi_tb.sv
//   vsim -c mesi_tb -do "run -all; quit"
// =============================================================================

`timescale 1ns/1ps
`define CLK_PERIOD 10

import mesi_pkg::*;

module mesi_tb;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  localparam int NUM_CACHES_P  = 4;
  localparam int ADDR_WIDTH_P  = mesi_pkg::ADDR_WIDTH_P;
  localparam int DATA_WIDTH_P  = mesi_pkg::DATA_WIDTH_P;
  localparam int MEM_LATENCY_P = 4;
  localparam int SNOOP_WAIT_P  = 2;

  // The single address all steps operate on (word-aligned)
  localparam logic [ADDR_WIDTH_P-1:0] ADDR_X = 32'h0000_0040;

  // ---------------------------------------------------------------------------
  // Clock and reset
  // ---------------------------------------------------------------------------
  logic clk_i   = 0;
  logic reset_i = 1;

  always #(`CLK_PERIOD/2) clk_i = ~clk_i;

  // ---------------------------------------------------------------------------
  // DUT signals
  // ---------------------------------------------------------------------------
  proc_req_e               proc_req_i   [NUM_CACHES_P-1:0];
  logic [ADDR_WIDTH_P-1:0] proc_addr_i  [NUM_CACHES_P-1:0];
  logic [DATA_WIDTH_P-1:0] proc_wdata_i [NUM_CACHES_P-1:0];
  logic [DATA_WIDTH_P-1:0] proc_rdata_o [NUM_CACHES_P-1:0];
  logic                    proc_stall_o [NUM_CACHES_P-1:0];
  logic                    proc_ack_o   [NUM_CACHES_P-1:0];

  // ---------------------------------------------------------------------------
  // DUT instantiation
  // ---------------------------------------------------------------------------
  mesi_system #(
     .NUM_CACHES_P  (NUM_CACHES_P)
    ,.ADDR_WIDTH_P  (ADDR_WIDTH_P)
    ,.DATA_WIDTH_P  (DATA_WIDTH_P)
    ,.MEM_LATENCY_P (MEM_LATENCY_P)
    ,.SNOOP_WAIT_P  (SNOOP_WAIT_P)
  ) dut (
     .clk_i         (clk_i)
    ,.reset_i        (reset_i)
    ,.proc_req_i     (proc_req_i)
    ,.proc_addr_i    (proc_addr_i)
    ,.proc_wdata_i   (proc_wdata_i)
    ,.proc_rdata_o   (proc_rdata_o)
    ,.proc_stall_o   (proc_stall_o)
    ,.proc_ack_o     (proc_ack_o)
  );

  // ---------------------------------------------------------------------------
  // Shorthand references into the DUT hierarchy
  // (used in checker tasks – adjust path if your hierarchy differs)
  // ---------------------------------------------------------------------------
  // Cache state arrays: dut.gen_caches[k].cache_ctrl.state_r[line]
  // We only check line 1 (the index bits of ADDR_X with INDEX_WIDTH_P=4)
  localparam int IDX_X = (ADDR_X >> mesi_pkg::OFFSET_WIDTH_P) & ((1 << mesi_pkg::INDEX_WIDTH_P)-1);

  // ---------------------------------------------------------------------------
  // Task: idle_all
  // Drive all processor ports to no-request.
  // ---------------------------------------------------------------------------
  task automatic idle_all();
    for (int k = 0; k < NUM_CACHES_P; k++) begin
      proc_req_i  [k] = PR_NONE;
      proc_addr_i [k] = '0;
      proc_wdata_i[k] = '0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Task: do_read(proc_id, addr)
  // Drives a PrRd on the given processor and waits for ack, respecting stall.
  // Returns the read data.
  // ---------------------------------------------------------------------------
  task automatic do_read(
    input  int                      proc_id
   ,input  logic [ADDR_WIDTH_P-1:0] addr
   ,output logic [DATA_WIDTH_P-1:0] rdata
  );
    idle_all();
    proc_req_i [proc_id] = PR_RD;
    proc_addr_i[proc_id] = addr;

    // Wait until ack fires (processor may be stalled for many cycles)
    do @(posedge clk_i); while (!proc_ack_o[proc_id]);
    rdata = proc_rdata_o[proc_id];

    idle_all();
    @(posedge clk_i);   // one idle cycle between transactions
  endtask

  // ---------------------------------------------------------------------------
  // Task: do_write(proc_id, addr, wdata)
  // ---------------------------------------------------------------------------
  task automatic do_write(
    input int                      proc_id
   ,input logic [ADDR_WIDTH_P-1:0] addr
   ,input logic [DATA_WIDTH_P-1:0] wdata
  );
    idle_all();
    proc_req_i  [proc_id] = PR_WR;
    proc_addr_i [proc_id] = addr;
    proc_wdata_i[proc_id] = wdata;

    do @(posedge clk_i); while (!proc_ack_o[proc_id]);

    idle_all();
    @(posedge clk_i);
  endtask

  // ---------------------------------------------------------------------------
  // Task: check_states(expected[4])
  // Reads the internal state_r for ADDR_X's cache line in each controller
  // and asserts it matches the expected value.
  // ---------------------------------------------------------------------------
  task automatic check_states(
    input mesi_state_e exp [NUM_CACHES_P-1:0]
   ,input string       step_name
  );
    mesi_state_e actual;
    logic pass = 1'b1;

    for (int k = 0; k < NUM_CACHES_P; k++) begin
      // Force-read internal state array through hierarchical reference
      case (k)
        0: actual = dut.gen_caches[0].cache_ctrl.state_r[IDX_X];
        1: actual = dut.gen_caches[1].cache_ctrl.state_r[IDX_X];
        2: actual = dut.gen_caches[2].cache_ctrl.state_r[IDX_X];
        3: actual = dut.gen_caches[3].cache_ctrl.state_r[IDX_X];
        default: actual = I;
      endcase

      if (actual !== exp[k]) begin
        $error("[%s] Cache %0d: expected %s, got %s",
               step_name, k,
               exp[k].name(), actual.name());
        pass = 1'b0;
      end
    end

    if (pass)
      $display("[PASS] %s — all cache states correct", step_name);
  endtask

  // ---------------------------------------------------------------------------
  // Write data values used in the test
  // ---------------------------------------------------------------------------
  localparam logic [DATA_WIDTH_P-1:0] WDATA_P0 = 32'hAAAA_1111;
  localparam logic [DATA_WIDTH_P-1:0] WDATA_P2 = 32'hBBBB_2222;

  // ---------------------------------------------------------------------------
  // Main stimulus
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH_P-1:0] rd_data;
  mesi_state_e exp_states [NUM_CACHES_P-1:0];

  initial begin
    // ---- Reset ---------------------------------------------------------------
    idle_all();
    reset_i = 1;
    repeat (4) @(posedge clk_i);
    @(negedge clk_i);
    reset_i = 0;
    @(posedge clk_i);
    $display("=== Reset released ===");

    // =========================================================================
    // Step 1: P0 reads ADDR_X
    //   P0: I + PrRd → BUS_RD → no other cache has it → memory supplies
    //   Expected: P0=E, P1=I, P2=I, P3=I
    // =========================================================================
    $display("\n--- Step 1: P0 reads ADDR_X ---");
    do_read(.proc_id(0), .addr(ADDR_X), .rdata(rd_data));
    $display("  P0 read data = 0x%08h", rd_data);

    exp_states = '{E, I, I, I};
    check_states(exp_states, "Step1: P0 Rd (cold miss → E)");

    // =========================================================================
    // Step 2: P0 writes ADDR_X
    //   P0: E + PrWr → SILENT upgrade (no bus traffic)
    //   Expected: P0=M, P1=I, P2=I, P3=I
    // =========================================================================
    $display("\n--- Step 2: P0 writes ADDR_X ---");
    do_write(.proc_id(0), .addr(ADDR_X), .wdata(WDATA_P0));

    exp_states = '{M, I, I, I};
    check_states(exp_states, "Step2: P0 Wr (silent E→M)");

    // =========================================================================
    // Step 3: P2 reads ADDR_X
    //   P2: I + PrRd → BUS_RD
    //   P0 snoops BUS_RD: M → flush → S
    //   P2: I → S (receives flushed data)
    //   Expected: P0=S, P1=I, P2=S, P3=I
    // =========================================================================
    $display("\n--- Step 3: P2 reads ADDR_X ---");
    do_read(.proc_id(2), .addr(ADDR_X), .rdata(rd_data));
    $display("  P2 read data = 0x%08h (should be 0x%08h)", rd_data, WDATA_P0);

    assert (rd_data === WDATA_P0)
      else $error("Step3: P2 got wrong data! expected=0x%08h got=0x%08h",
                  WDATA_P0, rd_data);

    exp_states = '{S, I, S, I};
    check_states(exp_states, "Step3: P2 Rd miss → intervention flush (P0:M→S, P2:I→S)");

    // =========================================================================
    // Step 4: P2 writes ADDR_X
    //   P2: S + PrWr → BUS_UPGR
    //   P0 snoops BUS_UPGR: S → I
    //   P2: S → M
    //   Expected: P0=I, P1=I, P2=M, P3=I
    // =========================================================================
    $display("\n--- Step 4: P2 writes ADDR_X ---");
    do_write(.proc_id(2), .addr(ADDR_X), .wdata(WDATA_P2));

    exp_states = '{I, I, M, I};
    check_states(exp_states, "Step4: P2 Wr (BUS_UPGR, P0:S→I, P2:S→M)");

    // =========================================================================
    // Step 5: P0 reads ADDR_X
    //   P0: I + PrRd → BUS_RD
    //   P2 snoops BUS_RD: M → flush → S
    //   P0: I → S (receives flushed data)
    //   Expected: P0=S, P1=I, P2=S, P3=I
    // =========================================================================
    $display("\n--- Step 5: P0 reads ADDR_X ---");
    do_read(.proc_id(0), .addr(ADDR_X), .rdata(rd_data));
    $display("  P0 read data = 0x%08h (should be 0x%08h)", rd_data, WDATA_P2);

    assert (rd_data === WDATA_P2)
      else $error("Step5: P0 got wrong data! expected=0x%08h got=0x%08h",
                  WDATA_P2, rd_data);

    exp_states = '{S, I, S, I};
    check_states(exp_states, "Step5: P0 Rd miss → intervention flush (P2:M→S, P0:I→S)");

    // =========================================================================
    // Step 6: P2 reads ADDR_X  (already in S — should be a HIT)
    //   P2: S + PrRd → hit, no bus traffic
    //   Expected: P0=S, P1=I, P2=S, P3=I  (unchanged)
    // =========================================================================
    $display("\n--- Step 6: P2 reads ADDR_X (expect HIT) ---");
    do_read(.proc_id(2), .addr(ADDR_X), .rdata(rd_data));
    $display("  P2 read data = 0x%08h", rd_data);

    exp_states = '{S, I, S, I};
    check_states(exp_states, "Step6: P2 Rd hit (stays S, no bus traffic)");

    // =========================================================================
    // Step 7: P1 reads ADDR_X  (cold miss, P0 and P2 hold S copies)
    //   P1: I + PrRd → BUS_RD
    //   P0 and P2 snoop: both S → signal shared, no flush needed
    //   Memory supplies the data (shared line, no intervention)
    //   P1: I → S
    //   Expected: P0=S, P1=S, P2=S, P3=I
    // =========================================================================
    $display("\n--- Step 7: P1 reads ADDR_X ---");
    do_read(.proc_id(1), .addr(ADDR_X), .rdata(rd_data));
    $display("  P1 read data = 0x%08h", rd_data);

    exp_states = '{S, S, S, I};
    check_states(exp_states, "Step7: P1 Rd miss (P0,P2 signal shared, P1:I→S)");

    // =========================================================================
    // Done
    // =========================================================================
    $display("\n=== Table 7.2 sequence complete ===");
    repeat (5) @(posedge clk_i);
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Timeout watchdog – kill simulation if it hangs
  // ---------------------------------------------------------------------------
  initial begin
  #100000;
    $error("TIMEOUT: simulation exceeded 100,000 ns");
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Waveform dump (works with VCS + Verdi, Questa, Xcelium)
  // ---------------------------------------------------------------------------
  initial begin
    $dumpfile("mesi_tb.vcd");
    $dumpvars(0, mesi_tb);
  end

endmodule
