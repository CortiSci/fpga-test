// SA-BIST: full-system Tail self-test through USB -> SPI_CFG -> telemetry.
// Included inside tb_top.sv.  The Tail BIST source is enabled through the same
// USB/SPI configuration path used on hardware; no DUT or link signal is forced.

task automatic run_SA_BIST();
    logic [15:0] m, f, a, d;
    int ch;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0] tx[0:5], rx[0:5];
`endif

`ifdef RUN_BIST_SKEW
    $display("[SA-BIST-SKEW] Four-Tail BIST with deterministic USB configuration-phase skew starting at %0t ns", $time);
`else
    $display("[SA-BIST] Full-system four-Tail BIST starting at %0t ns", $time);
`endif

    // Enable the physical SPI paths and use the documented low configuration rate.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;

    // SELF_TEST opcode 0x06 selects each Tail's internal ASIC-pattern source.
    for (ch = 0; ch < 4; ch++) begin
`ifdef ICARUS
        tx = 48'h06_01_00_00_00_00;
        spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h06, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00};
        spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
        $display("[SA-BIST] ch%0d SELF_TEST=1 sent through USB/SPI_CFG", ch);
    end

    // run_SA_ACED supplies the common CTRL/TELEM_EN setup, starts all four
    // channels, and verifies frame structure/CRC.  RUN_BIST adds counter
    // reconstruction checks to its received V3 frames; self-test overrides the
    // ASIC model's constant source inside each Tail.
    run_SA_ACED();
endtask
