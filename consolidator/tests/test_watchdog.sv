// ============================================================================
// test_watchdog.sv — the Consolidator watchdog BITES: arm, hold-off, bite,
//                    reset, fault packet, re-arm  (WD-01..WD-05)
//
// Compiled only for its target (+define+RUN_WD_BITE +define+SIM_SHORT_WD): the
// watchdog's half-period is 4,194,304 MCLK cycles (204.8 ms) in the bitstream,
// far beyond any simulation; SIM_SHORT_WD makes it 512 cycles (~25 us at
// 20.48 MHz), so a bite is 4 half-periods = ~100 us after the last pet.
//
// Contract under test (watchdog_con.v, consolidator_v2_top.v, reg_map_v2.v,
// telemetry_v3_software_contract.md §5):
//   * arming is the FIRST WD_PET (0x0052) write with wdata[0]=1; there is no
//     WD_EN register (0x0054 is reserved).  WD_STS (0x0053): bit2 armed,
//     bit1 counting, bit0 expired;
//   * pets more often than 3 half-periods apart hold the bite off indefinitely;
//   * a bite soft-resets every fabric register (tokens, enable mask, run bits,
//     the watchdog's own enable) without touching the PLL, then the consolidator
//     sends ONE global fault packet {0x55AA, 0xFFFF, flags, 0x0000} with flag
//     bit 3 (watchdog) set;
//   * the watchdog can be armed again after the bite.
// Nothing in the suite had ever simulated a bite (CV-03 only checks arming);
// the SW emulator's model was found to diverge on 2026-09-10 (explicit enable
// register, expiry without reset).
// ============================================================================
`ifdef RUN_WD_BITE

task automatic run_WD_BITE();
    // SIM_SHORT_WD: HALF_PERIOD = 512 cycles of the pad clock.  This bench feeds
    // the consolidator's MCLK pad with 51.2 MHz (mclk_con, PLL stub), so a
    // half-period is 10 us here (25 us on the real 20.48 MHz oscillator) and a
    // bite follows 4 half-periods = 40 us without a pet.
    localparam int HALF_US   = 10;
    localparam int PET_NS    = 15_000;                   // pet interval: < 3 half-periods
    localparam int BITE_WAIT = 4 * HALF_US * 1000 + 60_000;   // 4 half-periods + reset hold + margin
    logic [15:0] m, f, a, d;
    int n_pass = 0, n_fail = 0, i;

    $display("");
    $display("[WD] consolidator watchdog: arm by first pet, pets hold off, bite resets and reports, re-arm");

    // ── WD-01 not armed at reset; first pet with bit0=1 arms it ─────────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0053, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2:0] !== 3'b000) begin n_fail++; $display("[WD-01] FAIL WD_STS at reset = 0x%04h (expected 0)", d); end
    else n_pass++;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052, 16'h0001);   // arm + pet
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0053, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2:0] !== 3'b110) begin n_fail++; $display("[WD-01] FAIL WD_STS after first pet = 0x%04h (expected armed+counting = 0x0006)", d); end
    else begin n_pass++; $display("[WD-01] PASS armed by first WD_PET(1): WD_STS = 0x%04h", d); end

    // ── WD-02 regular pets hold the bite off across > 2 bite windows ────────
    for (i = 0; i < 6; i++) begin
        #PET_NS;
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052, 16'h0001);
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0053, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2:0] !== 3'b110) begin n_fail++; $display("[WD-02] FAIL WD_STS after %0d pets %0d us apart = 0x%04h (expected 0x0006, no bite)", i, PET_NS/1000, d); end
    else begin n_pass++; $display("[WD-02] PASS %0d pets %0d us apart: still armed, no bite (window is ~%0d us)", i, PET_NS/1000, 4*HALF_US); end

    // ── WD-03 put state in the registers the bite must clear ─────────────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052, 16'h0001);   // pet
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_TOKEN_HI, 16'h1234);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    // (No ACQ_ALL_RUN here: with legs enabled and run but no tail streaming, an
    // idle-high MISO anchors a receiver on 0xFFFF words and the engine emits a
    // garbage frame that swallows the command responses — a separate finding.)
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052, 16'h0001);   // last pet
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_EN_MASK, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[3:0] !== 4'hF) begin n_fail++; $display("[WD-03] FAIL SPI_EN_MASK before bite = 0x%04h (expected 0x000F)", d); end
    else n_pass++;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h1234) begin n_fail++; $display("[WD-03] FAIL TOKEN_HI before bite = 0x%04h (expected 0x1234)", d); end
    else begin n_pass++; $display("[WD-03] PASS state set: TOKEN_HI=0x1234, SPI_EN_MASK=0xF; pets now stop  t=%0t", $time); end

    // ── WD-04 no pets: the bite must reset the fabric and report itself ──────
    #BITE_WAIT;
    // The ONLY packet the consolidator may send unasked is the global fault packet.
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (m !== 16'h55AA || f !== 16'hFFFF || a[3] !== 1'b1 || d !== 16'h0000) begin
        n_fail++;
        $display("[WD-04] FAIL no watchdog fault packet after the bite window: got {%04h %04h %04h %04h} (expected {55AA FFFF xxx8 0000})", m, f, a, d);
    end else begin
        n_pass++;
        $display("[WD-04] PASS fault packet {55AA FFFF %04h 0000}: flag bit 3 = watchdog  t=%0t", a, $time);
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'hCEFA) begin n_fail++; $display("[WD-04] FAIL TOKEN_HI after bite = 0x%04h (expected reset value 0xCEFA)", d); end
    else begin n_pass++; $display("[WD-04] PASS TOKEN_HI back at its reset value 0xCEFA"); end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_EN_MASK, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[3:0] !== 4'h0) begin n_fail++; $display("[WD-04] FAIL SPI_EN_MASK after bite = 0x%04h (expected 0: legs disabled, acquisition stopped)", d); end
    else begin n_pass++; $display("[WD-04] PASS SPI_EN_MASK = 0: acquisition disabled by the bite"); end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0053, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2:0] !== 3'b000) begin n_fail++; $display("[WD-04] FAIL WD_STS after bite = 0x%04h (expected 0: disarmed and cleared by the reset)", d); end
    else begin n_pass++; $display("[WD-04] PASS WD_STS = 0 after the bite (reset clears armed/expired)"); end
    // Exactly one fault packet: nothing else may be waiting.
    if (tb_top.u_ft600q.ctrl_wr_ptr != tb_top.u_ft600q.ctrl_rd_ptr) begin
        n_fail++;
        $display("[WD-04] FAIL %0d unexpected ctrl word(s) queued after the bite sequence", tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr);
    end else n_pass++;

    // ── WD-05 the watchdog can be armed again ────────────────────────────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), 16'h0053, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2:0] !== 3'b110) begin n_fail++; $display("[WD-05] FAIL re-arm after bite: WD_STS = 0x%04h (expected 0x0006)", d); end
    else begin n_pass++; $display("[WD-05] PASS re-armed after the bite: WD_STS = 0x%04h", d); end

    $display("");
    if (n_fail == 0) $display("[WD] PASS — %0d checks: arm on first pet, pets hold off, bite resets registers + one fault packet (flag 3), re-arm", n_pass);
    else             $display("[WD] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask

`endif  // RUN_WD_BITE
