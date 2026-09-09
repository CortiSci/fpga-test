`ifdef RUN_SINGLE_LEG
// One-leg USB integration test.  The assembly contains exactly one ASIC and
// one Tail (LEG5); the other Consolidator MISO pins use their board pull-down.
// All setup and observation use the FT600Q USB boundary.

task automatic single_leg_tail_write(input logic [7:0] opcode, input logic [7:0] value);
    logic [15:0] m, f, a, d;
    begin
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA,
                                           {opcode, value});
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_CTRL,
                                           spi_cfg_ctrl_word(2'd0, 1'b0, 1'b1, 3'd2));
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        #35_000; // bounded real SPI transaction time at the configured divider
    end
endtask

task automatic run_SA_SINGLE_LEG_ACED();
    localparam logic [15:0] EXPECTED = 16'hACED;
    logic [15:0] m, f, a, d, hdr;
    logic [15:0] data [0:4095];
    logic [15:0] phase [0:3];
    logic [15:0] crcw [0:1];
    logic [15:0] expected_plane;
    integer i, n_fail, n_pass, n_flush;

    $display("[SA-SINGLE-LEG] USB -> LEG5 Tail -> ASIC -> V3 telemetry");
    tb_top.tail_ch[0].active_tail.asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[0].active_tail.asic_model.data_mode    = MODE_CONSTANT;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    single_leg_tail_write(8'h01, 8'h11); // RO_RSTn + MCLK_EN
    single_leg_tail_write(8'h02, 8'h01); // Tail normal telemetry
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_ACQ_ALL_RUN, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.flush_tx_capture(n_flush);

    // Discard the arm/frame-sync startup frame, then verify one complete V3 frame.
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
    for (i = 0; i < 4096; i++) data[i] = tb_top.u_ft600q.v3_data[i];
    for (i = 0; i < 4; i++) phase[i] = tb_top.u_ft600q.v3_phase[i];
    for (i = 0; i < 2; i++) crcw[i] = tb_top.u_ft600q.v3_crc[i];

    n_fail = 0; n_pass = 0;
    if (hdr !== 16'h0001) begin
        $error("[SA-SINGLE-LEG] V3 header 0x%04h, expected 0x0001", hdr);
        n_fail = n_fail + 1;
    end
    for (i = 0; i < 1024; i++) begin
        expected_plane = EXPECTED[15 - (i % 16)] ? 16'hFFFF : 16'h0000;
        if (data[4*i] !== expected_plane) begin
            if (n_fail < 8) $error("[SA-SINGLE-LEG] ch0 tick %0d = 0x%04h, expected 0x%04h",
                                   i, data[4*i], expected_plane);
            n_fail = n_fail + 1;
        end else n_pass = n_pass + 1;
        if (data[4*i+1] !== 16'h0000 || data[4*i+2] !== 16'h0000 || data[4*i+3] !== 16'h0000) begin
            if (n_fail < 8) $error("[SA-SINGLE-LEG] inactive slots nonzero at tick %0d", i);
            n_fail = n_fail + 1;
        end else n_pass = n_pass + 3;
    end
    if ((phase[0] & 16'h03FF) !== 10'd0) begin
        $error("[SA-SINGLE-LEG] active-leg phase %0d, expected 0", phase[0] & 16'h03FF);
        n_fail = n_fail + 1;
    end
    if (n_fail != 0) $fatal(1, "[SA-SINGLE-LEG] FAIL: %0d checks failed", n_fail);
    $display("[SA-SINGLE-LEG] PASS: %0d V3 lane-slot checks through one real Tail", n_pass);
endtask
`endif  // RUN_SINGLE_LEG
