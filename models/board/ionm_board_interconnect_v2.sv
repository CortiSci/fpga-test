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
    // Link propagation.  On the board every SPI wire has a few ns of ODDR/pad/
    // cable/isolator delay in each direction: SCLK and MOSI reach the tail
    // ~3-5 ns after the consolidator's fabric edge, and a MISO bit the tail
    // launches on the SCLK edge it receives is back at the consolidator pad
    // ~5-8 ns later still.  The receiver is self-timed for the DATA, but the
    // RUN=1 arm handshake is cycle-counted: the receiver pulses MOSI, the tail
    // aborts its in-flight word and drops MISO ~4 SCLKs later, and the
    // receiver must not begin hunting for START before that drop is visible
    // to it — with a registered MISO input the drop is seen one cycle later,
    // and on the bench (2026-09-10) a 4-cycle arm pulse let the receiver take
    // the tail's leftover '1' bits as a START: no anchor after most RUN=1
    // (leg FIFO empty, no overflow) or a false anchor whose garbage frames
    // carry PAR and that never re-anchors after the first overflow.
    // +define+SCLK_DELAY_NS / MOSI_DELAY_NS / MISO_DELAY_NS (+MISO_JITTER_NS)
    // model it; without the defines every path is zero-delay as before.
`ifdef SCLK_DELAY_NS
    assign #(`SCLK_DELAY_NS) tail_sclk = con_sclk;
`else
    assign tail_sclk = con_sclk;
`endif
`ifdef MOSI_DELAY_NS
    assign #(`MOSI_DELAY_NS) tail_mosi = con_mosi;
    assign #(`MOSI_DELAY_NS) tail_ss_n = con_ss_n;
`else
    assign tail_mosi = con_mosi;
    assign tail_ss_n = con_ss_n;
`endif
`ifdef MISO_DELAY_NS
    reg [3:0] con_miso_r = 4'b0000;
    genvar mi;
    generate
        for (mi = 0; mi < 4; mi = mi + 1) begin : g_miso_delay
            always @(tail_miso[mi]) begin
                real d;
`ifdef MISO_JITTER_NS
                d = `MISO_DELAY_NS - `MISO_JITTER_NS + ($urandom % 1000) * (2.0 * `MISO_JITTER_NS) / 1000.0;
`else
                d = `MISO_DELAY_NS;
`endif
                con_miso_r[mi] <= #(d) tail_miso[mi];   // transport delay per edge
            end
        end
    endgenerate
    assign con_miso = con_miso_r;
`else
    assign con_miso  = tail_miso;
`endif
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
