// SP-01 through SP-04: End-to-end SPI ping tests via the USB→SPI_CFG path.
//
// Each test sends CMD_PING (0xAA) to one tail_fpga_small channel through the
// complete RTL stack: FT600Q BFM → cmd_decoder → spi_cfg_ctrl → spi_master →
// physical SPI bus → user_spi_slave / simple_ping_decoder → MISO response
// captured by spi_cfg_ctrl → readable via REG_SPI_CFG_RD23.
//
// SPI clock: REG_SPI_CLK_DIV is set to 31 (800 kHz) at the start of each
// channel test, matching the intended hardware bringup rate.
//
// Expected response:
//   rx[0] = wd_sts (0x00 — simple_ping_decoder has no watchdog reset flag)
//   rx[1] = wd_sts (0x00)
//   rx[2] = 0x55  (PING_ACK; simple_ping_decoder responds from byte 2 onward)
//
// The spi_cfg_xact helper is defined in test_con_v2.sv which is included
// before this file in tb_top.sv.
// Do NOT add `timescale or import directives here.

task automatic run_spi_ping_ch(input int ch);
`ifdef ICARUS
    // Icarus cannot pass unpacked arrays as task parameters.  Use the packed
    // spi_cfg_xact overload; byte 0 occupies [47:40], byte 2 [31:24].
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] m, f, a, d;
    string       leg;

    case (ch)
        0: leg = "LEG5";
        1: leg = "LEG6";
        2: leg = "LEG7";
        3: leg = "LEG8";
        default: leg = "???";
    endcase

    $display("[SP-%02d] ── PING Tail ch%0d (%s) at %0t ns ──────────────────",
             ch+1, ch, leg, $time);

    // Set SCLK to 800 kHz (spi_clk_pre=31) before any SPI access.
    // Power-up default is 0 (25.6 MHz); always set explicitly so tests are
    // self-contained and independent of prior test state.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(),
                                       REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;

    // Enable only this channel
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(),
                                       REG_SPI_EN_MASK, 16'h0001 << ch);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;

    // 3-byte PING: byte0=opcode(0xAA), bytes1-2=zero padding.
    // n_bytes=3 holds SS_N low through byte2, allowing simple_ping_decoder
    // to drive PING_ACK=0x55 on MISO byte2.
`ifdef ICARUS
    tx = 48'hAA00_0000_0000;
    spi_cfg_xact(2'(ch), tx, 3'd3, rx);
`else
    tx = '{8'hAA, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00};
    spi_cfg_xact(2'(ch), tx, 3'd3, rx);
`endif

`ifdef ICARUS
    $display("[SP-%02d] Raw MISO: rx[0]=0x%02h rx[1]=0x%02h rx[2]=0x%02h",
             ch+1, rx[47:40], rx[39:32], rx[31:24]);

    if (^rx[47:40] === 1'bx) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[0] is X — SPI transfer did not complete",
               ch+1, ch, leg);
    end else if (^rx[31:24] === 1'bx) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[2] is X — 3rd MISO byte not captured",
               ch+1, ch, leg);
    end else if (rx[31:24] !== 8'h55) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[2]=0x%02h expected 0x55 — simple_ping_decoder did not respond",
               ch+1, ch, leg, rx[31:24]);
    end else begin
        $display("[SP-%02d] PASS ch%0d (%s): 0x55 at rx[2]", ch+1, ch, leg);
    end
`else
    $display("[SP-%02d] Raw MISO: rx[0]=0x%02h rx[1]=0x%02h rx[2]=0x%02h",
             ch+1, rx[0], rx[1], rx[2]);

    if (^rx[0] === 1'bx) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[0] is X — SPI transfer did not complete",
               ch+1, ch, leg);
    end else if (^rx[2] === 1'bx) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[2] is X — 3rd MISO byte not captured",
               ch+1, ch, leg);
    end else if (rx[2] !== 8'h55) begin
        $error("[SP-%02d] FAIL ch%0d (%s): rx[2]=0x%02h expected 0x55 — simple_ping_decoder did not respond",
               ch+1, ch, leg, rx[2]);
    end else begin
        $display("[SP-%02d] PASS ch%0d (%s): 0x55 at rx[2]", ch+1, ch, leg);
    end
`endif

    // Disable channel before moving to next
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(),
                                       REG_SPI_EN_MASK, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #1000;
endtask

task automatic run_SP01(); run_spi_ping_ch(0); endtask  // LEG5
task automatic run_SP02(); run_spi_ping_ch(1); endtask  // LEG6
task automatic run_SP03(); run_spi_ping_ch(2); endtask  // LEG7
task automatic run_SP04(); run_spi_ping_ch(3); endtask  // LEG8
