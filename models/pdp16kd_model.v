// Simulation model for Lattice PDP16KD pseudo-dual-port EBR block.
// Port A: synchronous write (clka).
// Port B: synchronous read (clkb), REGMODE_B=NOREG → one-cycle read latency
//         (data appears the cycle after ada is presented, same as registered mode
//          except OUTREG is bypassed).
// Width: 16-bit data, 10-bit address (1K × 16 = 16 Kbits).
// Only the configuration used by ebr_circ_buf.v is modelled.
`timescale 1ns/1ps
module PDP16KD #(
    parameter DATA_WIDTH_A = 18,
    parameter DATA_WIDTH_B = 18,
    parameter REGMODE_B    = "NOREG",
    parameter RESETMODE    = "ASYNC",
    parameter ASYNC_RESET_RELEASE = "SYNC",
    parameter INITVAL_00   = 320'h0,
    parameter INITVAL_01   = 320'h0,
    parameter INITVAL_02   = 320'h0,
    parameter INITVAL_03   = 320'h0,
    parameter INITVAL_04   = 320'h0,
    parameter INITVAL_05   = 320'h0,
    parameter INITVAL_06   = 320'h0,
    parameter INITVAL_07   = 320'h0,
    parameter INITVAL_08   = 320'h0,
    parameter INITVAL_09   = 320'h0,
    parameter INITVAL_0A   = 320'h0,
    parameter INITVAL_0B   = 320'h0,
    parameter INITVAL_0C   = 320'h0,
    parameter INITVAL_0D   = 320'h0,
    parameter INITVAL_0E   = 320'h0,
    parameter INITVAL_0F   = 320'h0,
    parameter INIT_DATA    = "STATIC"
) (
    // Port A (write)
    input  wire [13:0] ADA,
    input  wire [17:0] DIA,
    input  wire        CEA,
    input  wire        CLKA,
    input  wire        WEA,
    input  wire        RSTA,
    input  wire        OCEA,

    // Port B (read)
    input  wire [13:0] ADB,
    output reg  [17:0] DOB,
    input  wire        CEB,
    input  wire        CLKB,
    input  wire        RSTB,
    input  wire        OCEB
);

    // 1 K × 18-bit storage (16 data + 2 parity; parity unused in this design)
    reg [17:0] mem [0:1023];

    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            mem[i] = 18'h0;
    end

    // Port A write — address is ADA[13:4] for 16-bit width (lower bits select byte lane)
    always @(posedge CLKA) begin
        if (CEA && WEA)
            mem[ADA[13:4]] <= DIA;
    end

    // Port B read — NOREG mode: registered output (one-cycle latency)
    always @(posedge CLKB) begin
        if (RSTB)
            DOB <= 18'h0;
        else if (CEB)
            DOB <= mem[ADB[13:4]];
    end

endmodule
