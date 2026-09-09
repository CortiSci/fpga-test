// Simulation stub for Lattice ODDRXE DDR output register (MachXO2 synthesis primitive).
// Port-compatible with ODDRX1F — renamed to match synthesis RTL.
//
// Correct hardware behaviour:
//   posedge SCLK : Q  <- D0   (output first half-period immediately)
//                  d1r <- D1  (latch D1 into internal register at same posedge)
//   negedge SCLK : Q  <- d1r  (output the latched D1, NOT live D1)
//
// The previous stub sampled D1 live at negedge which reads post-NBA combinatorial
// values — incorrect for hardware and produces different SPI SCLK timing.
`timescale 1ns/1ps
/* verilator lint_off DECLFILENAME */
module ODDRXE (
    input  wire D0,
    input  wire D1,
    input  wire SCLK,
    input  wire RST,
    output reg  Q
);
    reg d1r;  // D1 latched at posedge — mirrors the internal FF in the real primitive

    // One process models both clock edges so simulators see one driver for Q.
    // This is a simulation primitive, not synthesizable FPGA RTL.
    always @(posedge SCLK or negedge SCLK or posedge RST) begin
        if (RST) begin
            Q   <= 1'b0;
            d1r <= 1'b0;
        end else if (SCLK) begin
            Q   <= D0;
            d1r <= D1;
        end else begin
            Q   <= d1r;
        end
    end
endmodule
/* verilator lint_on DECLFILENAME */
