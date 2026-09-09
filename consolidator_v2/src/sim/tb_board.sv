// Named board-integration entry point.  The shared functional assembly is
// preprocessed under tb_board so all task hierarchy remains local while the
// extracted V2 board interconnect is part of the elaborated DUT composition.
`define tb_top tb_board
`include "tb_top.sv"
`undef tb_top
