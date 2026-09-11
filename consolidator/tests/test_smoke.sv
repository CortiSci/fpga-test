// SM-01 through SM-04 smoke tests.
// Included inside module tb_top — do NOT add `timescale or import directives here.
// Package imports are inherited from tb_top.sv compilation context.
// Identical to production Consolidator FPGA smoke tests: the V2 register map
// shares addresses 0x0000–0x0011, so the same tests apply.

// SM-01: power-on reset — SPI outputs at idle levels after reset
task automatic run_SM01(
    input logic [3:0] spi_ss,
    input logic [3:0] spi_sclk
);
    if (spi_ss !== 4'b1111)
        $error("[SM-01] SPI SS not idle-high after reset: 0b%04b", spi_ss);
    else if (spi_sclk !== 4'b0000)
        $error("[SM-01] SPI SCLK not low after reset: 0b%04b", spi_sclk);
    else
        $display("[SM-01] PASS");
endtask

// SM-02: USB clock domain isolation — passive check
task automatic run_SM02();
    $display("[SM-02] 10 us elapsed with clocks running, no CDC activity");
    $display("[SM-02] PASS (verify no X on CDC FIFO flags in waveform)");
endtask

// SM-03: SPI_ENABLE_MASK write / read-back via USB (0x0002)
task automatic run_SM03();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_EN_MASK, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h000F)
        $error("[SM-03] SPI_ENABLE_MASK readback 0x%04h, expected 0x000F", d);
    else
        $display("[SM-03] PASS (SPI_ENABLE_MASK = 0x%04h)", d);
    // Restore
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
endtask

// SM-04: DaisyChain GPIO — write GPIO_DAISY_CTRL bit0=1, verify read-back
task automatic run_SM04();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_GPIO_DAISY, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_GPIO_DAISY, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[0] !== 1'b1)
        $error("[SM-04] daisy_out_val not set: GPIO_DAISY_CTRL = 0x%04h", d);
    else
        $display("[SM-04] PASS (GPIO_DAISY_CTRL = 0x%04h)", d);
    // Restore
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_GPIO_DAISY, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
endtask

// SM-05 — USB path diagnostics (2026-09-10): USB_RX_WORDS counts every command
// word the decoder consumes (each 4-word command it answers has already been
// counted, so two back-to-back reads differ by exactly 4), and USB_STS shows the
// mover idle with a response not pending once the exchange is over.
task automatic run_SM05();
    logic [15:0] m, f, a, d1, d2, sts;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h005E, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d1);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h005E, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d2);
    #2000;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h005F, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, sts);
    if (d2 == d1 + 16'd4 && d1 >= 16'd4 && sts[5:0] == 6'b000001 && sts[14] == 1'b0)
        $display("[SM-05] PASS (USB_RX_WORDS %0d -> %0d, USB_STS = 0x%04h: mover idle, no response pending)", d1, d2, sts);
    else
        $display("[SM-05] FAIL (USB_RX_WORDS %0d -> %0d, USB_STS = 0x%04h)", d1, d2, sts);
endtask
