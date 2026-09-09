// Simulation stub for pll_48m (EHXPLLJ-based PLL).
// Asserts locked=1 after 100 ns; passes clki through as clkop at same frequency.
// In real hardware the PLL multiplies 20.48 MHz → 51.2 MHz (CLKI_DIV=2, CLKFB_DIV=5,
// CLKOP_DIV=9); simulation uses the MCLK source directly and relies on clk generation
// in testbenches producing 51.2 MHz (half-period #9.766).
`timescale 1ns/1ps
/* verilator lint_off DECLFILENAME */
`ifdef USE_PLL_STUB

module pll_48m (
    input  wire clki,
    output wire clkop,
    output reg  locked
);
    assign clkop = clki;

    initial begin
        locked = 1'b0;
        #100 locked = 1'b1;
    end
endmodule
/* verilator lint_on DECLFILENAME */

// Also stub EHXPLLJ directly in case it is instantiated by library modules.
module EHXPLLJ #(
    parameter CLKI_DIV  = 1,
    parameter CLKFB_DIV = 1,
    parameter CLKOP_DIV = 1,
    parameter CLKOS_DIV = 1,
    parameter CLKOS2_DIV = 1,
    parameter CLKOS3_DIV = 1,
    parameter FEEDBK_PATH = "CLKOP",
    parameter CLKOP_TRIM_DELAY = 0,
    parameter CLKOS_TRIM_DELAY = 0,
    parameter DELAY_CNTL = "STATIC",
    parameter FREQUENCY_PIN_CLKI  = 100.0,
    parameter FREQUENCY_PIN_CLKOP = 100.0,
    parameter CLKOP_CPHASE = 0,
    parameter CLKOS_CPHASE = 0,
    parameter STDBY_ENABLE = "DISABLED",
    parameter DPHASE_SOURCE = "STATIC",
    parameter CLKOS_TRIM_POL = "RISING",
    parameter CLKOP_TRIM_POL = "RISING",
    parameter OUTDIVIDER_MUXA = "DIVA",
    parameter OUTDIVIDER_MUXB = "DIVB",
    parameter OUTDIVIDER_MUXC = "DIVC",
    parameter OUTDIVIDER_MUXD = "DIVD",
    parameter PLL_LOCK_MODE = 0
) (
    input  wire CLKI,
    input  wire CLKFB,
    input  wire PHASESEL0,
    input  wire PHASESEL1,
    input  wire PHASEDIR,
    input  wire PHASESTEP,
    input  wire PHASELOADREG,
    input  wire STDBY,
    input  wire PLLWAKESYNC,
    input  wire RST,
    input  wire ENCLKOP,
    input  wire ENCLKOS,
    input  wire ENCLKOS2,
    input  wire ENCLKOS3,
    output wire CLKOP,
    output wire CLKOS,
    output wire CLKOS2,
    output wire CLKOS3,
    output wire LOCK,
    output wire INTLOCK,
    output wire REFCLK,
    output wire CLKINTFB
);
    reg lock_r = 1'b0;
    initial #100 lock_r = 1'b1;
    assign CLKOP    = CLKI;
    assign CLKOS    = CLKI;
    assign CLKOS2   = CLKI;
    assign CLKOS3   = CLKI;
    assign LOCK     = lock_r;
    assign INTLOCK  = lock_r;
    assign REFCLK   = CLKI;
    assign CLKINTFB = CLKI;
endmodule

`endif // USE_PLL_STUB
