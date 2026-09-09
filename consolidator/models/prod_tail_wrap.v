// Simulation-only wrapper — exposes record_tail_fpga under the name tail_fpga_small.
//
// tb_top.sv instantiates module tail_fpga_small.  In the production-Tail regression
// sim (test_spi_ping_prod_tail.do) we want the real record_tail_fpga RTL to sit in
// that socket instead of the simplified tail_fpga_small.
//
// Port lists of tail_fpga_small and record_tail_fpga are identical (same names, same
// directions, same widths), so `(.*)` wildcard port connections are correct.
// This file must be compiled AFTER record_tail_fpga.v and BEFORE tb_top.sv.
`timescale 1ns/1ps

module tail_fpga_small (
    input  wire [15:0] RO1_SD,
    input  wire        RO1_FRAME,
    input  wire        RO1_CLK,
    output wire        RO_RSTn,
    output wire        SPI_RO_MOSI,
    input  wire        SPI_RO_MISO,
    output wire        SPI_RO_SCLK,
    output wire        SPI_RO_SS0_B,
    output wire        SPI_RO_SS1_B,
    input  wire        FPGA_SPI_MOSI,
    output wire        FPGA_SPI_MISO,
    input  wire        FPGA_SPI_SCLK,
    input  wire        FPGA_SPI_SS,
    output wire        MCLK_EN,
    output wire        TEST_SIG,
    output wire        REC_TEST_AMP_SHDN,
    input  wire        MCLK_20_48M,
    input  wire        DEVRST_N,
    inout  wire        SDA_CurrentSense_ADC,
    inout  wire        SCL_CurrentSense_ADC,
    input  wire        DRDY_CurrentSense_ADC,
    output wire        LEDn_0
);
    record_tail_fpga u0 (.*);
endmodule
