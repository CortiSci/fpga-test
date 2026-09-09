// Tail-unit BIST entry point.
//
// Scope: one tail_fpga_small plus its built-in self-test source and SPI master
// stimulus.  It does not instantiate the Consolidator or USB boundary, so it
// is unit evidence only.  The existing BIST checks remain in tb_bist; this
// wrapper provides the explicit architecture-plan target while ACED is
// refactored away from its current Consolidator receive-side dependency.
`timescale 1ns/100ps

module tb_tail_unit;
    tb_bist u_bist ();
endmodule
