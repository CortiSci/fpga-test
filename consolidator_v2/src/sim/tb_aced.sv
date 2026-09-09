// ============================================================
// tb_aced.sv  —  UNIT TEST: ACED raw-bit-plane forwarding
//
// This bench does not instantiate consolidator_v2_top and does not exercise
// USB. It may use a local SPI mux for Tail setup; it must never be presented
// as system-integration evidence.
//
// Tests: tail_fpga_small → SPI MISO → spi_ch_stream (consolidator)
//
// Test data: all 1024 sensors per frame output value 0xACED.
// This requires ro1_sd to follow the bit-serial encoding of 0xACED:
//   At RO1_CLK edge k (mod 16): ro1_sd = 0xFFFF if bit(15-k) of 0xACED = 1
//                                         0x0000 if bit(15-k) of 0xACED = 0
// With this input, spi_ch_stream must forward the corresponding 16 raw
// bit-planes in MSB-first order. Host software reconstructs 0xACED from each
// consecutive group of 16 bit-planes.
//
// Pass criterion: all TOTAL_WORDS forwarded bit-planes match the source order.
//
// TC-ACED: covers the path:
//   ro1_sd (bit-serial 0xACED) → asic_stream_tx (ping-pong + serial TX)
//   → FPGA_SPI_MISO → spi_ch_stream (raw bit-plane forwarding)
//   → word_data
//
// Setup via SPI commands (no force on any DUT-internal signals):
//   CTRL   0x01 data=0x11: RO_RSTn=1 (ASIC running), MCLK_EN=1
//   TELEM_EN 0x02 data=0x01: telem_mode=2'b01 (normal streaming)
// Testbench mux (bfm_sel) switches SPI bus from bit-bang to spi_ch_stream.
// ============================================================
`timescale 1ns/1ps

module tb_aced;

// ---- Parameters --------------------------------------------------------
localparam [15:0] DATA          = 16'hACED;
localparam        ASIC_WORDS_PF = 1024;
localparam        GROUPS_PF     = 64;
localparam        LANES         = 16;
localparam        N_FRAMES      = 2;
localparam        TOTAL_WORDS   = N_FRAMES * GROUPS_PF * LANES;

// ---- Clocks and Reset --------------------------------------------------
// sys_clk: 100 MHz (proportional to 51.2 MHz FPGA fabric)
// ro1_clk: 2 MHz (proportional to 2.56 MHz ASIC readout clock)
reg sys_clk = 1'b0;
always #5 sys_clk = ~sys_clk;

reg ro1_clk = 1'b0;
always #250 ro1_clk = ~ro1_clk;

reg mclk_20m = 1'b0;
always #24 mclk_20m = ~mclk_20m;

reg rst_n;    // uninitialized (x) so the first assignment 0 fires negedge → async reset
initial begin
    rst_n = 1'b0;               // x→0 negedge: fires DUT async reset
    repeat(4) @(posedge sys_clk);
    rst_n = 1'b1;               // release reset
end

// ---- Bit-serial 0xACED generator (RO1_CLK domain) ---------------------
// Edge k (mod 16): ro1_sd = 0xFFFF if DATA[15-k]=1, else 0x0000.
// Updated on negedge ro1_clk so it is stable for the next posedge.
reg [3:0]  bit_ph    = 4'd0;
reg [9:0]  asic_wc   = 10'd0;
reg [15:0] ro1_sd    = 16'hFFFF;
reg        ro1_frame = 1'b0;

always @(negedge ro1_clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_ph    <= 4'd0;
        asic_wc   <= 10'd0;
        ro1_sd    <= 16'hFFFF;
        ro1_frame <= 1'b0;
    end else begin
        ro1_sd    <= DATA[15 - bit_ph] ? 16'hFFFF : 16'h0000;
        ro1_frame <= (asic_wc == 10'd0 && bit_ph == 4'd0) ? 1'b1 : 1'b0;
        if (bit_ph == 4'd15) begin
            bit_ph  <= 4'd0;
            asic_wc <= (asic_wc == 10'd1023) ? 10'd0 : asic_wc + 10'd1;
        end else begin
            bit_ph  <= bit_ph + 4'd1;
        end
    end
end

// ---- SPI bus mux -------------------------------------------------------
// bfm_sel=1: testbench bit-bang drives the tail (CTRL + TELEM_EN setup).
// bfm_sel=0: spi_ch_stream drives the tail (streaming acquisition).
// Mirrors the testbench mux approach specified in docs/sim_guidelines.md.
reg  bfm_sel = 1'b1;
reg  tb_sclk = 1'b0;
reg  tb_mosi = 1'b0;
reg  tb_ss_n = 1'b1;

wire spi_sclk_oddr;  // ODDRXE output (25 MHz when sclk_d1=1)
wire spi_mosi_strm;  // spi_ch_stream ARM output
wire spi_ss_n_strm;  // spi_ch_stream SS_N

// Muxed SPI lines to the tail
wire tail_sclk = bfm_sel ? tb_sclk     : spi_sclk_oddr;
wire tail_mosi = bfm_sel ? tb_mosi     : spi_mosi_strm;
wire tail_ss_n = bfm_sel ? tb_ss_n     : spi_ss_n_strm;
wire tail_miso;  // tail → spi_ch_stream.miso_in (always connected)

// ---- ODDRXE: generates SPI SCLK from sys_clk --------------------------
wire sclk_d1;

ODDRXE u_oddr (
    .D0   (1'b0),
    .D1   (sclk_d1),
    .SCLK (sys_clk),
    .RST  (~rst_n),
    .Q    (spi_sclk_oddr)
);

// ---- spi_ch_stream: consolidator receive side --------------------------
reg  run = 1'b0;
wire [15:0] word_data;
wire        word_valid;

spi_ch_stream u_strm (
    .sclk        (sys_clk),
    .rst_n       (rst_n),
    .miso_in     (tail_miso),
    .run         (run),
    .cfg_hold    (1'b0),
    .sclk_d1     (sclk_d1),
    .mosi_out    (spi_mosi_strm),
    .ss_n_out    (spi_ss_n_strm),
    .word_data   (word_data),
    .word_valid  (word_valid),
    .par_err_flag(),
    .resync_arm  (1'b0),
    .fifo_full   (1'b0),
    .quad_flush  (),
    .w0_dist     (),
    .phase_known ()
);

// ---- tail_fpga_small: channel 0 ----------------------------------------
wire sda_stub, scl_stub;

tail_fpga_small u_tail (
    .RO1_SD               (ro1_sd),
    .RO1_FRAME            (ro1_frame),
    .RO1_CLK              (ro1_clk),
    .RO_RSTn              (),
    .SPI_RO_MOSI          (),
    .SPI_RO_MISO          (1'b0),
    .SPI_RO_SCLK          (),
    .SPI_RO_SS0_B         (),
    .SPI_RO_SS1_B         (),
    .FPGA_SPI_MOSI        (tail_mosi),
    .FPGA_SPI_MISO        (tail_miso),
    .FPGA_SPI_SCLK        (tail_sclk),
    .FPGA_SPI_SS          (tail_ss_n),
    .MCLK_EN              (),
    .TEST_SIG             (),
    .REC_TEST_AMP_SHDN    (),
    .MCLK_20_48M          (mclk_20m),
    .DEVRST_N             (rst_n),
    .SDA_CurrentSense_ADC (sda_stub),
    .SCL_CurrentSense_ADC (scl_stub),
    .DRDY_CurrentSense_ADC(1'b0),
    .LEDn_0               ()
);

// ---- Bit-bang SPI tasks (Mode 0: CPOL=0 CPHA=0, MSB-first) ------------
// Half-period = 200 ns → 2.5 MHz.  Runs only during bfm_sel=1 phase.
// spi_slave_small samples MOSI on posedge of FPGA_SPI_SCLK = tail_sclk = tb_sclk.

task spi_send_byte;
    input [7:0] data;
    integer i;
    begin
        for (i = 7; i >= 0; i = i - 1) begin
            tb_mosi = data[i];   // present bit before rising edge
            #200;
            tb_sclk = 1'b1;      // rising edge: slave samples MOSI
            #200;
            tb_sclk = 1'b0;      // falling edge
        end
    end
endtask

task spi_write_cmd;
    input [7:0] opcode;
    input [7:0] data;
    begin
        tb_ss_n = 1'b0;
        #200;                    // SS_N setup before first clock
        spi_send_byte(opcode);
        spi_send_byte(data);
        #200;                    // inter-byte hold before SS_N rises
        tb_ss_n = 1'b1;
        // One SCLK posedge with SS_N=1 so slave resets byte_cnt/ss_n_d for next cmd
        #200; tb_sclk = 1'b1;
        #200; tb_sclk = 1'b0;
        #400;                    // remaining idle gap
    end
endtask

// ---- Setup: SPI commands → hand off to spi_ch_stream ------------------
initial begin
    @(posedge rst_n);
    @(posedge sys_clk);   // one clock settling

    // CTRL 0x01 data=0x11: RO_RSTn=1, MCLK_EN=1
    spi_write_cmd(8'h01, 8'h11);
    // TELEM_EN 0x02 data=0x01: normal streaming
    spi_write_cmd(8'h02, 8'h01);

    // Hand off: release bus to spi_ch_stream and start streaming
    bfm_sel = 1'b0;
    run     = 1'b1;
end

// ---- Result capture ----------------------------------------------------
integer n_total = 0;
integer n_fail  = 0;
integer n_pass  = 0;
reg [15:0] expected_plane;

always @(posedge sys_clk) begin
    if (word_valid) begin
        n_total = n_total + 1;
        expected_plane = DATA[15 - ((n_total - 1) % 16)] ? 16'hFFFF : 16'h0000;
        if (word_data === expected_plane) begin
            n_pass = n_pass + 1;
        end else begin
            n_fail = n_fail + 1;
            if (n_fail <= 8)
                $display("[FAIL] TC-ACED: plane #%0d = 0x%04X (expected 0x%04X)",
                         n_total, word_data, expected_plane);
        end
    end
end

// ---- Simulation driver -------------------------------------------------
initial begin
    wait (n_total >= TOTAL_WORDS);
    #20;

    if (n_fail == 0) begin
        $display("[PASS] TC-ACED: all %0d raw bit-planes match 0xACED source order", n_pass);
        $display("[PASS] TC-ACED: host-side transpose input is correct");
        $display("RESULTS: 1 passed, 0 failed");
        $display("STATUS : PASS");
    end else begin
        $display("[FAIL] TC-ACED: %0d/%0d raw bit-planes wrong", n_fail, n_total);
        $display("RESULTS: 0 passed, 1 failed");
        $display("STATUS : FAIL");
    end
    $finish;
end

// Timeout guard (50 ms at 100 MHz = 5 000 000 cycles)
initial begin
    #50_000_000;
    $display("[FAIL] TC-ACED: TIMEOUT — only %0d/%0d words received after 50 ms",
             n_total, TOTAL_WORDS);
    $display("RESULTS: 0 passed, 1 failed");
    $display("STATUS : FAIL");
    $finish;
end

endmodule
