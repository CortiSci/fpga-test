// Named one-leg integration root.  The implementation stays shared with the
// V2 full-stack bench; RUN_SINGLE_LEG removes the other three Tail/ASIC legs.
`define tb_top tb_single_leg
`include "tb_top.sv"
`undef tb_top
