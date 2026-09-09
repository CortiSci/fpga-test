// Observable digital connectivity extracted from the Consolidator/Tail EDIF.
// R13/R14 and the SPI isolators are represented as zero-delay digital paths;
// the Tail 20.48 MHz fanout and I2C pull-ups are modeled here as board nets.
// FAULTN is open-drain with the Consolidator-board R58 pull-up.
`timescale 1ns/1ps
module ionm_board_interconnect_v2 (
    input  wire [3:0] con_sclk,
    input  wire [3:0] con_mosi,
    input  wire [3:0] con_ss_n,
    output wire [3:0] tail_sclk,
    output wire [3:0] tail_mosi,
    output wire [3:0] tail_ss_n,
    input  wire [3:0] tail_miso,
    output wire [3:0] con_miso,
    input  wire       mclk_20m48_in,
    output wire [3:0] tail_mclk_20m48,
    inout  wire [3:0] tail_i2c_sda,
    inout  wire [3:0] tail_i2c_scl,
    input  wire       faultn_ext,
    output wire       faultn_con
);
    assign tail_sclk = con_sclk;
    assign tail_mosi = con_mosi;
    assign tail_ss_n = con_ss_n;
    assign con_miso  = tail_miso;
    assign tail_mclk_20m48 = {4{mclk_20m48_in}};

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_tail_pullups
            pullup (tail_i2c_sda[i]);
            pullup (tail_i2c_scl[i]);
        end
    endgenerate

    wand faultn_w;
    pullup (faultn_w);
    assign faultn_w  = faultn_ext;
    assign faultn_con = faultn_w;
endmodule
