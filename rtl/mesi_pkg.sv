// =============================================================================
// mesi_pkg.sv
//
// Shared type definitions for the MESI cache coherence protocol.
// Import this package in every module:  import mesi_pkg::*;
//
// Coding standard: BSG (bespoke-silicon-group)
//   - logic everywhere (no reg/wire)
//   - _i / _o port suffixes
//   - _r register output, _n next-state wire
//   - _p parameter suffix, _lp localparam suffix
//   - _e enum type suffix
//   - leading comma in port/parameter lists
// =============================================================================

package mesi_pkg;

  // ---------------------------------------------------------------------------
  // MESI cache line states  (2 bits)
  //
  //  M – Modified  : line is dirty; this cache is the sole owner.
  //                  Must write back before another cache can read.
  //  E – Exclusive : line is clean; this cache is the sole owner.
  //                  Can silently upgrade to M on a write (no bus traffic).
  //  S – Shared    : line is clean; one or more caches hold a copy.
  //                  Must issue BusRdX / BusUpgr before writing.
  //  I – Invalid   : no valid copy.  Default / reset state.
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    M = 2'b00,
    E = 2'b01,
    S = 2'b10,
    I = 2'b11   // reset-friendly: 2'b11 = all-ones, set with '1 fill
  } mesi_state_e;

  // ---------------------------------------------------------------------------
  // Bus transaction types  (3 bits)
  //
  //  BUS_NONE  – no transaction this cycle (bus is idle)
  //  BUS_RD    – read request; requester does not intend to write
  //  BUS_RDX   – read-exclusive; requester will write (cold miss + ownership)
  //  BUS_UPGR  – upgrade; requester already has data in S, wants ownership only
  //              (avoids a redundant data transfer on the bus)
  //  BUS_FLUSH – requester (or snoop responder) is writing dirty data to memory
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    BUS_NONE  = 3'b000,
    BUS_RD    = 3'b001,
    BUS_RDX   = 3'b010,
    BUS_UPGR  = 3'b011,
    BUS_FLUSH = 3'b100
  } bus_trans_e;

  // ---------------------------------------------------------------------------
  // Processor request types  (2 bits)
  //
  //  PR_NONE – no request from the processor this cycle
  //  PR_RD   – processor load  (PrRd in textbook notation)
  //  PR_WR   – processor store (PrWr in textbook notation)
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PR_NONE = 2'b00,
    PR_RD   = 2'b01,
    PR_WR   = 2'b10
  } proc_req_e;

  // ---------------------------------------------------------------------------
  // Convenience: address and data widths as package-level parameters.
  // Change here and everything recompiles consistently.
  // ---------------------------------------------------------------------------
  parameter int ADDR_WIDTH_P = 32;
  parameter int DATA_WIDTH_P = 32;
  parameter int TAG_WIDTH_P  = 26;   // ADDR_WIDTH_P - index - offset bits
  parameter int INDEX_WIDTH_P = 4;   // 2^4 = 16 cache lines per cache
  parameter int OFFSET_WIDTH_P = 2;  // 2^2 = 4-byte (word) blocks

endpackage
