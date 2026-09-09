// Simulation stub for Lattice GSR (Global Set/Reset) primitive.
// In simulation the DEVRST_N → rst_n chain in top-level already handles reset.
`timescale 1ns/1ps
module GSR (input wire GSR);
endmodule

// PUR stub — Lattice Power-Up Reset primitive, unused in simulation.
module PUR (input wire PUR);
endmodule
