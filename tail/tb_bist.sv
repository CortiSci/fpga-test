// ============================================================
// tb_bist.sv — Standalone BIST verification for tail_fpga_small
//
// Tests the asic_self_test circuit end-to-end through the tail's
// serial transmit path.
//
// BIST design (new):
//   1. SPI opcode 0x06 (data=1) sets test_en.  asic_self_test immediately
//      begins generating:
//        st_clk   = 2.56 MHz square wave (MCLK / 8)
//        st_frame = 2.5 kHz pulse (every 1024 st_clk cycles)
//        st_sd    = free-running 16-bit counter (increments each st_clk posedge)
//      No startup delay.  BIST runs freely regardless of telem_mode.
//
//   2. SPI opcode 0x02 (data=0x01) sets telem_mode to normal streaming (2'b01).
//
//   3. The Consolidator arms the tail by driving MOSI=1 while SS_N=1.
//      asic_stream_tx detects arm=1.
//
//   4. asic_stream_tx waits for the first st_frame rising edge (frame_armed=1),
//      then starts streaming from that frame boundary.
//
//   5. Received words are a monotonically increasing 16-bit counter
//      (consecutive words differ by exactly 1).  Even parity = XOR of the
//      16 data bits (^rx_word) must match the received parity bit.
//
//   6. TC-BIST-07: disabling test_en resets asic_self_test (st_clk=0,
//      st_sd=0, st_frame=0).  Re-enabling restarts the BIST.
//
// Two-speed SCLK model:
//   SPI command mode : SCLK = 3.906 MHz  (proportional to 800 kHz = 51.2/64)
//   Telemetry mode   : SCLK = 250 MHz    (proportional to 51.2 MHz, ratio 2.5:1 vs MCLK)
// MCLK stays at 100 MHz throughout.  The 2.5× SCLK/MCLK ratio required by
// asic_stream_tx and asic_self_test is active in telemetry mode.
//
// Frame format (20 SCLK cycles per word, sampled on posedge SCLK):
//   [1 start=1][16 data MSB-first][1 even parity]
//   Note: no guard band (TX_GUARD removed).  2 idle cycles between words.
//
// recv_word stable_lo=2: at 20:1 ratio, only 2 TX_IDLE cycles separate words.
//   Two consecutive MISO=0 reliably identifies the inter-word gap; the very
//   next 0→1 is guaranteed to be the START bit.  Works for the first call
//   (long pre-stream idle) and all subsequent calls (2-cycle TX_IDLE gap).
//
// Run from QuestaSim GUI:
//   do "…/tail_fpga_small/scripts/sim_bist.do"   then: run -all
// ============================================================
`timescale 1ns/100ps

module tb_bist;

    // =========================================================================
    // Parameters and clocks
    // =========================================================================

    localparam MCLK_HALF      = 5;   // 10 ns period → 100 MHz (proportional to 20.48 MHz)
    localparam SCLK_HALF      = 2;   // 4 ns period  → 250 MHz (proportional to 51.2 MHz, telemetry mode)
    localparam SLOW_SCLK_HALF = 128; // 256 ns period → 3.906 MHz (proportional to 800 kHz, SPI cmd mode)
    // Max fast-SCLK cycles to wait for a START bit in recv_word phase 2.
    // Worst case from frame_armed: ~40 cycles (next ro_edge + 2 FFs + TX_START pipeline).
    // 500 is 10× margin; far below the 5 ms outer guard (1,250,000 cycles).
    localparam RECV_TIMEOUT   = 500;

    reg mclk;
    reg sclk_fast;   // 250 MHz — telemetry rate (proportional to 51.2 MHz)
    reg sclk_slow;   // 3.906 MHz — SPI command rate (proportional to 800 kHz)
    reg sclk_sel;    // 0=SPI-rate, 1=telemetry-rate
    wire sclk = sclk_sel ? sclk_fast : sclk_slow;  // mux driven to DUT and all tasks

    initial begin mclk      = 1'b0; forever #MCLK_HALF      mclk      = ~mclk;      end
    initial begin sclk_fast = 1'b0; forever #SCLK_HALF      sclk_fast = ~sclk_fast; end
    initial begin sclk_slow = 1'b0; forever #SLOW_SCLK_HALF sclk_slow = ~sclk_slow; end

    // =========================================================================
    // DUT signals
    // =========================================================================

    reg        devrst_n;
    reg        mosi;
    wire       miso;
    reg        ss_n;

    // Real ASIC readout signals — driven idle (unused in BIST mode)
    reg [15:0] ro1_sd;
    reg        ro1_frame;
    reg        ro1_clk;

    // =========================================================================
    // DUT instantiation
    // =========================================================================

    tail_fpga_small dut (
        .RO1_SD               (ro1_sd),
        .RO1_FRAME            (ro1_frame),
        .RO1_CLK              (ro1_clk),
        .RO_RSTn              (              ),
        .SPI_RO_MOSI          (              ),
        .SPI_RO_MISO          (1'b0          ),
        .SPI_RO_SCLK          (              ),
        .SPI_RO_SS0_B         (              ),
        .SPI_RO_SS1_B         (              ),
        .FPGA_SPI_MOSI        (mosi          ),
        .FPGA_SPI_MISO        (miso          ),
        .FPGA_SPI_SCLK        (sclk          ),
        .FPGA_SPI_SS          (ss_n          ),
        .MCLK_EN              (              ),
        .TEST_SIG             (              ),
        .REC_TEST_AMP_SHDN    (              ),
        .MCLK_20_48M          (mclk          ),
        .DEVRST_N             (devrst_n      ),
        .SDA_CurrentSense_ADC (              ),
        .SCL_CurrentSense_ADC (              ),
        .DRDY_CurrentSense_ADC(1'b0          ),
        .LEDn_0               (              )
    );

    // =========================================================================
    // Pass/fail counters
    // =========================================================================

    integer pass_cnt;
    integer fail_cnt;

    // =========================================================================
    // Task: SPI 2-byte command  [opcode][data]
    //
    // SPI Mode 0: CPOL=0, CPHA=0, MSB-first, free-running SCLK.
    // =========================================================================

    task automatic spi_cmd(input [7:0] opcode, input [7:0] data);
        integer i;
        @(negedge sclk); #0.2;
        ss_n = 1'b0;
        for (i = 7; i >= 0; i--) begin
            mosi = opcode[i];
            @(posedge sclk);
            @(negedge sclk); #0.2;
        end
        for (i = 7; i >= 0; i--) begin
            mosi = data[i];
            @(posedge sclk);
            @(negedge sclk); #0.2;
        end
        ss_n = 1'b1;
        mosi = 1'b0;
        @(posedge sclk);
    endtask

    // =========================================================================
    // Task: receive one streaming word from FPGA_SPI_MISO
    //
    // Phase 1 — guard (stable_lo=2):
    //   Wait for 2 consecutive MISO=0 posedges.  At the 20:1 SCLK/st_clk ratio,
    //   only 2 TX_IDLE cycles separate consecutive words.  This is sufficient to
    //   distinguish the inter-word gap from data bits for any counter value,
    //   because the 2-cycle idle is always followed by a guaranteed START bit —
    //   not a data-bit transition.  Works for both the first call (long pre-stream
    //   idle) and all subsequent calls (2-cycle gap).
    //
    // Phase 2 — START detection: scan for 0→1 on MISO.
    // Phase 3 — data: capture 16 bits MSB-first.
    // Phase 4 — parity: capture 1 bit.
    //
    // stream_miso_ff is pipeline-registered by 1 SCLK, so each bit appears on
    // MISO one SCLK cycle after the state machine outputs it.
    // =========================================================================

    // recv_word: receive one 18-bit serial word from FPGA_SPI_MISO.
    // Sets timed_out=1 and returns early (word/par=0) if no START bit arrives
    // within RECV_TIMEOUT fast-SCLK cycles.  This prevents an infinite hang
    // when streaming is not actually running on the bus.
    task automatic recv_word(output logic [15:0] word, output logic par,
                             output logic timed_out);
        logic prev_miso, cur_miso;
        integer i, stable_lo, phase2_cnt;
        timed_out  = 1'b0;
        word       = 16'h0;
        par        = 1'b0;
        // Phase 1: guard — wait for 2 consecutive MISO=0 (inter-word gap)
        stable_lo = 0;
        while (stable_lo < 2) begin
            @(posedge sclk);
            if (miso === 1'b0) stable_lo = stable_lo + 1;
            else               stable_lo = 0;
        end
        // Phase 2: scan for 0→1 START bit, with timeout
        prev_miso  = 1'b0;
        phase2_cnt = 0;
        forever begin
            @(posedge sclk); cur_miso = miso;
            if (cur_miso & ~prev_miso) break;
            prev_miso = cur_miso;
            if (phase2_cnt >= RECV_TIMEOUT) begin
                timed_out = 1'b1;
                return;
            end
            phase2_cnt = phase2_cnt + 1;
        end
        // Phase 3: capture 16 data bits MSB-first
        for (i = 15; i >= 0; i--) begin
            @(posedge sclk);
            word[i] = miso;
        end
        // Phase 4: parity bit
        @(posedge sclk);
        par = miso;
    endtask

    // =========================================================================
    // Checker helpers
    // =========================================================================

    task automatic chk1(input string name, input logic got, input logic exp);
        if (got === exp) begin
            $display("    [PASS] %s", name);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s  got=%0b  exp=%0b", name, got, exp);
            fail_cnt++;
        end
    endtask

    task automatic chk16(input string name, input logic [15:0] got, input logic [15:0] exp);
        if (got === exp) begin
            $display("    [PASS] %s  =  %0d (0x%04h)", name, got, got);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s  got=%0d (0x%04h)  exp=%0d (0x%04h)",
                     name, got, got, exp, exp);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    // Task: check one received counter word
    //
    // Verifies that rx_word equals exp_word (the previous word + 1) and that
    // the received parity bit equals the even parity of the data (^rx_word).
    // =========================================================================

    task automatic chk_counter_word(
        input string        label,
        input logic [15:0]  rx_word,
        input logic [15:0]  exp_word,
        input logic         rx_par
    );
        logic exp_par;
        exp_par = ^rx_word;   // even parity: XOR of all 16 data bits
        if (rx_word !== exp_word) begin
            $display("    [FAIL] %s  word=0x%04h (%0d)  exp=0x%04h (%0d)  [not monotonic+1]",
                     label, rx_word, rx_word, exp_word, exp_word);
            fail_cnt++;
        end else begin
            $display("    [PASS] %s  word=0x%04h (%0d)  [+1 monotonic]", label, rx_word, rx_word);
            pass_cnt++;
        end
        if (rx_par !== exp_par) begin
            $display("    [FAIL] %s parity: got=%0b  exp=%0b  (^0x%04h=%0b)",
                     label, rx_par, exp_par, rx_word, exp_par);
            fail_cnt++;
        end else begin
            $display("    [PASS] %s parity OK (%0b)", label, rx_par);
            pass_cnt++;
        end
    endtask

    // =========================================================================
    // Local variables for main thread
    // =========================================================================

    logic [15:0] rx_word;
    logic        rx_par;
    logic        rx_timeout;     // recv_word timed out (no start bit found)
    logic [15:0] prev_word;
    logic        bist05_passed;  // TC-BIST-05 result; guards TC-BIST-06
    logic        stream_ok;      // word[00] received OK; enables words 1-31
    integer      w;

    // =========================================================================
    // Main test sequence
    // =========================================================================

    initial begin : tb_main
        // ---- Initialise driven signals ------------------------------------
        pass_cnt  = 0;
        fail_cnt  = 0;
        devrst_n  = 1'b0;
        ss_n      = 1'b1;
        mosi      = 1'b0;
        sclk_sel  = 1'b0;    // start in SPI-rate mode (3.906 MHz ≡ 800 kHz)
        ro1_sd    = 16'h0000;
        ro1_frame = 1'b0;
        ro1_clk   = 1'b0;

        // ---- Reset: hold for 20 SCLK cycles, then release ----------------
        repeat (20) @(posedge sclk);
        devrst_n = 1'b1;
        repeat (4) @(posedge sclk);

        $display("=============================================================");
        $display(" TB-BIST  tail_fpga_small BIST Self-Test Verification");
        $display("=============================================================");

        // ==================================================================
        // TC-BIST-01  Post-reset state
        // ==================================================================
        $display("\n[TC-BIST-01] Initial state after reset");
        @(posedge sclk);
        chk1("test_en  = 0 at reset",   dut.test_en,               1'b0);
        chk1("telem_mode = off at reset",   dut.telem_mode == 2'b00, 1'b1);
        chk1("st_frame = 0 at reset",   dut.u_self_test.st_frame,  1'b0);
        chk1("st_clk   = 0 at reset",   dut.u_self_test.st_clk,    1'b0);
        chk16("st_sd   = 0 at reset",   dut.u_self_test.st_sd,    16'd0);

        // ==================================================================
        // TC-BIST-02  SPI write opcode 0x06 enables test_en
        //   BIST starts running immediately (no startup delay, no telem_en
        //   dependency).  By the time the slow-SCLK command completes,
        //   st_clk has already been toggling for hundreds of MCLK cycles.
        // ==================================================================
        $display("\n[TC-BIST-02] SPI write SELF_TEST_EN (0x06, data=0x01)");
        spi_cmd(8'h06, 8'h01);
        @(posedge sclk);
        chk1("test_en = 1 after SELF_TEST write", dut.test_en,  1'b1);
        chk1("telem_mode = off (not yet set)",   dut.telem_mode == 2'b00, 1'b1);

        // ==================================================================
        // TC-BIST-03  SPI write opcode 0x02 enables telem_en
        //   Switch to telemetry-rate SCLK after the command completes.
        // ==================================================================
        $display("\n[TC-BIST-03] SPI write TELEM_EN (0x02, data=0x01)");
        spi_cmd(8'h02, 8'h01);
        sclk_sel = 1'b1;   // fast SCLK for telemetry reception
        @(posedge sclk);
        chk1("telem_mode = normal after TELEM_EN write", dut.telem_mode == 2'b01, 1'b1);

        // ==================================================================
        // TC-BIST-04  Send arm pulse — MOSI=1 while SS_N=1
        //   asic_stream_tx detects arm when mosi_r=1, ss_n_r=1, telem_en=1,
        //   tx_state=TX_IDLE.  arm=1 takes effect 2 fast-SCLK cycles after
        //   MOSI is asserted.  Hold MOSI high for 4 cycles then deassert.
        // ==================================================================
        $display("\n[TC-BIST-04] Send arm pulse (MOSI=1 while SS_N=1)");
        mosi = 1'b1;
        repeat(4) @(posedge sclk);
        mosi = 1'b0;
        @(posedge sclk);
        chk1("arm=1 after MOSI pulse", dut.u_stream_tx.arm, 1'b1);

        // ==================================================================
        // TC-BIST-05  Wait for frame_armed
        //   asic_stream_tx 3-FF sync detects the first st_frame rising edge
        //   after arm fires and sets frame_armed.  Worst-case wait: one full
        //   BIST frame period = 1024 st_clk cycles = ~20480 fast-SCLK.
        // ==================================================================
        $display("\n[TC-BIST-05] Wait for frame_armed (first st_frame after arm)");
        bist05_passed = 1'b0;
        begin
            integer timeout;
            timeout = 0;
            // Worst case: arm just after a frame boundary → wait one full frame
            // period = 1024 st_clk cycles = 20480 fast-SCLK.  30000 is 1.5× margin.
            while (!dut.u_stream_tx.frame_armed && timeout < 30000) begin
                @(posedge sclk);
                timeout++;
            end
            if (timeout >= 30000) begin
                $display("    [FAIL] frame_armed never asserted — timeout at 30000 SCLK");
                fail_cnt++;
            end else begin
                $display("    [PASS] frame_armed asserted after %0d SCLK cycles", timeout);
                pass_cnt++;
                bist05_passed = 1'b1;
                // In new asic_stream_tx, frame_armed is set simultaneously with
                // TX_IDLE→TX_START (word 0 already started).  Wait for word 0 to
                // complete (tx_state returns to TX_IDLE = 2'd0) so recv_word
                // (TC-BIST-06) starts scanning during the inter-word gap, not
                // inside an active word.  TX_IDLE = 2'd0 = asic_stream_tx localparam.
                begin : wait_w0_bist06
                    integer to_w0;
                    to_w0 = 0;
                    while (dut.u_stream_tx.tx_state !== 2'd0 && to_w0 < 30) begin
                        @(posedge sclk);
                        to_w0 = to_w0 + 1;
                    end
                end
            end
        end

        // ==================================================================
        // TC-BIST-06  Receive 32 streaming words — verify counter + parity
        //
        //   st_sd increments by 1 each st_clk posedge.  After crossing the
        //   CDC and being transmitted, consecutive received words must differ
        //   by exactly 1.  Even parity = XOR of the 16 data bits.
        //
        //   First word establishes the baseline (counter value is non-deterministic
        //   from this testbench's perspective); words 2-32 are each checked for
        //   prev+1 monotonicity and correct parity.
        //
        //   Guarded by bist05_passed: if frame_armed never fired, no data can
        //   arrive on MISO and recv_word would hang.  Skip the entire TC and
        //   mark all 32 words FAIL so the root cause is visible.
        // ==================================================================
        $display("\n[TC-BIST-06] Receive 32 streaming words — verify counter+1 and parity");
        if (!bist05_passed) begin
            $display("    [SKIP] TC-BIST-05 failed (frame_armed timeout) — no streaming data");
            $display("           Marking all 32 word checks as FAIL");
            for (w = 0; w < 32; w++) fail_cnt++;
        end else begin
            stream_ok = 1'b1;
            recv_word(rx_word, rx_par, rx_timeout);
            if (rx_timeout) begin
                $display("    [FAIL] Word[00] recv_word timed out after %0d SCLK cycles —", RECV_TIMEOUT);
                $display("           frame_armed fired but no START bit on MISO (streaming not running)");
                fail_cnt++;
                stream_ok = 1'b0;
            end else begin
                prev_word = rx_word;
                $display("    [INFO] Baseline word[00] = 0x%04h (%0d)", rx_word, rx_word);
                if (rx_par !== ^rx_word) begin
                    $display("    [FAIL] Word[00] parity: got=%0b  exp=%0b", rx_par, ^rx_word);
                    fail_cnt++;
                end else begin
                    $display("    [PASS] Word[00] parity OK");
                    pass_cnt++;
                end
            end

            if (stream_ok) begin
                for (w = 1; w < 32; w++) begin
                    recv_word(rx_word, rx_par, rx_timeout);
                    if (rx_timeout) begin
                        $display("    [FAIL] Word[%02d] recv_word timed out — streaming stopped mid-frame", w);
                        fail_cnt++;
                    end else begin
                        chk_counter_word($sformatf("Word[%02d]", w), rx_word, prev_word + 16'd1, rx_par);
                        prev_word = rx_word;
                    end
                end
            end else begin
                $display("    [SKIP] Words 1-31 skipped — baseline word[00] not received");
                for (w = 1; w < 32; w++) fail_cnt++;
            end
        end

        // ==================================================================
        // TC-BIST-07  Disable test_en → verify BIST resets; re-enable
        //             and verify BIST restarts (st_clk toggles again).
        //   Disabling test_en resets asic_self_test (st_clk=0, st_sd=0,
        //   st_frame=0).  Re-enabling restarts the free-running BIST.
        //   Note: arm/frame_armed in asic_stream_tx are not reset here;
        //   those clear only via RST_N or at natural frame end (word_cnt=1023).
        // ==================================================================
        $display("\n[TC-BIST-07] Disable test_en → verify BIST resets; re-enable");

        sclk_sel = 1'b0;   // slow SCLK for SPI commands
        spi_cmd(8'h06, 8'h00);   // SELF_TEST_EN = 0
        @(posedge sclk);
        chk1("test_en  = 0 after disable", dut.test_en,               1'b0);
        chk1("st_clk   = 0 after disable", dut.u_self_test.st_clk,    1'b0);
        chk16("st_sd   = 0 after disable", dut.u_self_test.st_sd,    16'd0);
        chk1("st_frame = 0 after disable", dut.u_self_test.st_frame,  1'b0);

        // Re-enable: BIST should restart immediately
        spi_cmd(8'h06, 8'h01);   // SELF_TEST_EN = 1
        @(posedge sclk);
        chk1("test_en = 1 after re-enable", dut.test_en, 1'b1);

        // Verify st_clk begins toggling (BIST active; no startup delay)
        begin
            integer tmo;
            tmo = 0;
            while (!dut.u_self_test.st_clk && tmo < 200) begin
                @(posedge sclk);
                tmo++;
            end
            if (tmo >= 200) begin
                $display("    [FAIL] st_clk did not toggle after test_en re-enable — timeout");
                fail_cnt++;
            end else begin
                $display("    [PASS] st_clk toggled after %0d SCLK cycles", tmo);
                pass_cnt++;
            end
        end

        // ==================================================================
        // TC-BIST-08  Static 0xACED through real RO1_SD path
        //
        //   test_en=0 → tail mux routes RO1_SD (not st_sd) to asic_stream_tx.
        //   Drives ro1_sd=0xACED with free-running ro1_clk + ro1_frame.
        //   Arms for each of 10 consecutive ASIC frames; captures word[0]
        //   per frame and checks it equals 0xACED with correct even parity.
        //   Isolates tail-side data-path corruption from consolidator or SPI.
        // ==================================================================
        $display("\n[TC-BIST-08] Static ASIC data: ro1_sd=0xACED, 10 frame spot-check");

        // Fresh reset — clears arm, frame_armed, telem_mode, and ping-pong state
        sclk_sel  = 1'b0;
        devrst_n  = 1'b0;
        repeat(20) @(posedge sclk);
        devrst_n  = 1'b1;
        repeat(4)  @(posedge sclk);

        // Static ASIC data; test_en stays 0 so mux routes RO1_SD into asic_stream_tx
        ro1_sd    = 16'hACED;
        ro1_frame = 1'b0;
        ro1_clk   = 1'b0;

        // Enable normal telemetry (opcode 0x02, data=0x01 → telem_mode=2'b01)
        spi_cmd(8'h02, 8'h01);
        sclk_sel = 1'b1;
        @(posedge sclk);
        chk1("TC-BIST-08 setup: telem_mode=01",        dut.telem_mode == 2'b01, 1'b1);
        chk1("TC-BIST-08 setup: test_en=0 (real path)", dut.test_en,             1'b0);

        // ro1_clk (proportional to 2.56 MHz, half-period=40 ns) + ro1_frame (1-cycle
        // pulse every 1024 ro1_clk cycles) run in the background for 22 frame periods.
        // 22 frames × 82 µs = 1.8 ms covers 10 test frames (each ~164 µs round-trip).
        fork
            begin : bist08_clkgen
                integer ci;
                ro1_frame = 1'b0;
                for (ci = 0; ci < 22 * 1024; ci = ci + 1) begin
                    #40; ro1_clk = 1'b1;
                    #40; ro1_clk = 1'b0;
                    if ((ci % 1024) == 0) ro1_frame = 1'b1;
                    if ((ci % 1024) == 1) ro1_frame = 1'b0;
                end
                ro1_clk   = 1'b0;
                ro1_frame = 1'b0;
            end
        join_none   // clock generator runs in background; killed by $finish

        begin : bist08_main
            integer frame08;
            integer to08a;
            integer to08b;
            integer to_w0_08;
            for (frame08 = 0; frame08 < 10; frame08 = frame08 + 1) begin
                // Arm: MOSI=1 while SS_N=1 for 4 fast-SCLK cycles
                mosi = 1'b1;
                repeat(4) @(posedge sclk);
                mosi = 1'b0;
                @(posedge sclk);

                // Wait for frame_armed (first ro1_frame edge after arm)
                // Worst case: arm fires just after a pulse → wait one full period (~20480 sclk)
                to08a = 0;
                while (!dut.u_stream_tx.frame_armed && to08a < 50000) begin
                    @(posedge sclk); to08a = to08a + 1;
                end

                if (to08a >= 50000) begin
                    $display("    [FAIL] Frame[%0d]: frame_armed timeout", frame08);
                    fail_cnt = fail_cnt + 1;
                end else begin
                    // Same fix as TC-BIST-06: frame_armed fires simultaneously with
                    // TX_IDLE→TX_START in new asic_stream_tx.  Wait for word 0 to
                    // complete (tx_state returns to TX_IDLE=2'd0) so recv_word starts
                    // scanning during the inter-word gap, not inside an active word.
                    to_w0_08 = 0;
                    while (dut.u_stream_tx.tx_state !== 2'd0 && to_w0_08 < 30) begin
                        @(posedge sclk);
                        to_w0_08 = to_w0_08 + 1;
                    end
                    // Discard one recv_word: for some re-arm timings, word 1 starts
                    // in the same cycle the TX_IDLE loop exits, leaving only 1 idle
                    // MISO cycle before the START bit.  Phase 1 (stable_lo=2) then
                    // scans into word 1's data and the capture is misaligned.
                    // The second recv_word call always starts from the inter-word gap
                    // after the (mis-)aligned first word completes, and correctly
                    // captures the next word.  All words carry 0xACED so discarding
                    // one word does not affect the test assertion.
                    recv_word(rx_word, rx_par, rx_timeout);   // discard (may misalign)
                    recv_word(rx_word, rx_par, rx_timeout);
                    if (rx_timeout) begin
                        $display("    [FAIL] Frame[%0d] Word[0]: recv_word timeout", frame08);
                        fail_cnt = fail_cnt + 1;
                    end else begin
                        chk16($sformatf("Frame[%0d] Word[0]", frame08), rx_word, 16'hACED);
                        if (rx_par !== ^16'hACED) begin
                            $display("    [FAIL] Frame[%0d] Word[0] parity: got=%0b exp=%0b",
                                     frame08, rx_par, ^16'hACED);
                            fail_cnt = fail_cnt + 1;
                        end else begin
                            $display("    [PASS] Frame[%0d] Word[0] parity OK", frame08);
                            pass_cnt = pass_cnt + 1;
                        end
                    end
                    // Drain: wait for arm to deassert after word[1023] TX_PAR completes.
                    // Max: 1023 words × 20 sclk cycles = 20,460; 25,000 adds margin.
                    // Continuous-stream contract: the initial arm remains
                    // asserted across ASIC frame boundaries. TELEM_EN=0 is
                    // the only normal stop mechanism.
                    if (!dut.u_stream_tx.arm) begin
                        $display("    [FAIL] Frame[%0d]: continuous stream unexpectedly disarmed",
                                 frame08);
                        fail_cnt = fail_cnt + 1;
                    end else begin
                        $display("    [PASS] Frame[%0d]: continuous stream remains armed", frame08);
                        pass_cnt = pass_cnt + 1;
                    end
                end
            end
        end

        // The existing SPI stop is the explicit close/re-arm boundary.
        spi_cmd(8'h02, 8'h00);
        repeat(4) @(posedge sclk);
        chk1("TC-BIST-08 stop: arm cleared by TELEM_EN=0", dut.u_stream_tx.arm, 1'b0);

        // ==================================================================
        // Final report
        // ==================================================================
        $display("\n=============================================================");
        $display(" RESULTS: %0d passed,  %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display(" STATUS : PASS");
        else
            $display(" STATUS : FAIL");
        $display("=============================================================");
        $finish;
    end

    // =========================================================================
    // Simulation timeout guard (prevents infinite hang in recv_word)
    // =========================================================================

    initial begin
        #10_000_000;  // 10 ms — TC-BIST-08 adds ~1.8 ms (10 frames × 82 µs each × 2)
        $display("[TIMEOUT] Simulation exceeded 10 ms — possible deadlock in recv_word");
        $display("STATUS : FAIL");
        $finish;
    end

endmodule
