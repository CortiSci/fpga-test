// Consolidator V2 Top-Level Testbench
// Instantiates: consolidator_v2_top DUT, 4× tail_fpga_small DUT,
//               4× UCSD ASIC model, 4× ADS122C14 I2C stub,
//               FT600Q TLM, 4× SPI master BFM (retained for debug).
// Models, task packages, and primitive stubs are shared from the
// production Consolidator FPGA sim directory — no duplication.
`timescale 1ns/1ps

import checker_pkg::*;
import usb_cmd_pkg::*;
import spi_tasks_pkg::*;

module tb_top;

    // =========================================================================
    // Clock & reset
    // =========================================================================
    logic mclk_con;      // 51.2 MHz → Consolidator PLL input
    logic tail_mclk;     // 20.48 MHz → Tail FPGA MCLK_20_48M
    logic usb_fifo_clk;  // 66 MHz source-synchronous from FT600Q
    logic devrst_n;
    logic faultn_tb;

    initial mclk_con = 1'b0;
    always #9.766 mclk_con = ~mclk_con;

    initial tail_mclk = 1'b0;
    always #24.414 tail_mclk = ~tail_mclk;

    initial usb_fifo_clk = 1'b0;
    always #7.576 usb_fifo_clk = ~usb_fifo_clk;

    initial begin
        devrst_n  = 1'b0;
        faultn_tb = 1'b1;
        #200 devrst_n = 1'b1;
    end

    // =========================================================================
    // FT600Q ↔ Consolidator V2 boundary wires
    // =========================================================================
    wire [15:0] usb_fifo_d;
    wire [1:0]  usb_fifo_be;
    wire        usb_fifo_rxf_n;
    wire        usb_fifo_txe_n;
    wire        usb_fifo_rd_n;
    wire        usb_fifo_wr_n;
    wire        usb_fifo_oe_n;
    wire        usb_fifo_siwu_n;
    wire        usb_reset_n;
    wire [1:0]  usb_gpio;

    // =========================================================================
    // Consolidator ↔ Tail FPGA SPI buses (4 channels)
    // =========================================================================
    wire [3:0] spi_sclk_c2t;
    wire [3:0] spi_mosi_c2t;
    // A physically absent Tail leaves its MISO input at the board pull-down.
    // RUN_SINGLE_LEG instantiates only ch0; this passive net model is not a
    // DUT override and preserves the V2 input boundary for ch1..ch3.
    tri0 [3:0] spi_miso_t2c;
    wire [3:0] spi_miso_b2c;
    wire [3:0] spi_ss_c2t;
    wire [3:0] spi_sclk_t2b;
    wire [3:0] spi_mosi_t2b;
    wire [3:0] spi_ss_t2b;
    wire       faultn_board;
    wire [3:0] tail_mclk_board;
    tri  [3:0] tail_i2c_sda_board;
    tri  [3:0] tail_i2c_scl_board;

    ionm_board_interconnect_v2 board_wiring (
        .con_sclk  (spi_sclk_c2t), .con_mosi (spi_mosi_c2t), .con_ss_n (spi_ss_c2t),
        .tail_sclk (spi_sclk_t2b), .tail_mosi(spi_mosi_t2b), .tail_ss_n(spi_ss_t2b),
        .tail_miso (spi_miso_t2c), .con_miso (spi_miso_b2c),
        .mclk_20m48_in(tail_mclk), .tail_mclk_20m48(tail_mclk_board),
        .tail_i2c_sda(tail_i2c_sda_board), .tail_i2c_scl(tail_i2c_scl_board),
        .faultn_ext(faultn_tb), .faultn_con(faultn_board)
    );

    // =========================================================================
    // Tail FPGA ↔ UCSD ASIC buses (4 channels)
    // =========================================================================
    wire [15:0] ro1_sd    [0:3];
    wire        ro1_frame [0:3];
    wire        ro1_clk   [0:3];
    wire        ro_rst_n  [0:3];
    wire        spi_ro_mosi [0:3];
    wire        spi_ro_miso [0:3];
    wire        spi_ro_sclk [0:3];
    wire        spi_ro_ss0  [0:3];
    wire        spi_ro_ss1  [0:3];

    // I2C buses (4 channels; open-drain with pull-ups)
    // I2C nets live INSIDE the per-channel generate block: Verilator cannot
    // resolve a tristate onto an array element (LHS ARRAYSEL), and nothing
    // outside the block references them.  tri1 semantics preserved per index.
    wire        drdy_n  [0:3];

    // =========================================================================
    // Consolidator V2 DUT
    // =========================================================================
    consolidator_v2_top dut_con (
        .MCLK_20_48M_FPGA  (mclk_con),
        .DEVRST_N          (devrst_n),
        .PROGRAMN          (),

        .LEDn_0            (),
        .FAULTN            (faultn_board),

        .DaisyChain_input0  (1'b0),
        .DaisyChain_output0 (),

        .USB_FIFO_CLK      (usb_fifo_clk),
        .USB_FIFO_D        (usb_fifo_d),
        .USB_FIFO_BE       (usb_fifo_be),
        .USB_FIFO_RXF_N    (usb_fifo_rxf_n),
        .USB_FIFO_TXE_N    (usb_fifo_txe_n),
        .USB_FIFO_OE_N     (usb_fifo_oe_n),
        .USB_FIFO_RD_N     (usb_fifo_rd_n),
        .USB_FIFO_WR_N     (usb_fifo_wr_n),
        .USB_FIFO_SIWU_N   (usb_fifo_siwu_n),
        .USB_RESET_N       (usb_reset_n),
        .USB_GPIO          (usb_gpio),

        // Active SPI channels: LEG5=ch0, LEG6=ch1, LEG7=ch2, LEG8=ch3
        .SPI_MISO_LEG5     (spi_miso_b2c[0]),
        .SPI_MOSI_LEG5     (spi_mosi_c2t[0]),
        .SPI_SCLK_LEG5     (spi_sclk_c2t[0]),
        .SPI_SS_LEG5       (spi_ss_c2t[0]),

        .SPI_MISO_LEG6     (spi_miso_b2c[1]),
        .SPI_MOSI_LEG6     (spi_mosi_c2t[1]),
        .SPI_SCLK_LEG6     (spi_sclk_c2t[1]),
        .SPI_SS_LEG6       (spi_ss_c2t[1]),

        .SPI_MISO_LEG7     (spi_miso_b2c[2]),
        .SPI_MOSI_LEG7     (spi_mosi_c2t[2]),
        .SPI_SCLK_LEG7     (spi_sclk_c2t[2]),
        .SPI_SS_LEG7       (spi_ss_c2t[2]),

        .SPI_MISO_LEG8     (spi_miso_b2c[3]),
        .SPI_MOSI_LEG8     (spi_mosi_c2t[3]),
        .SPI_SCLK_LEG8     (spi_sclk_c2t[3]),
        .SPI_SS_LEG8       (spi_ss_c2t[3]),

        // Reserve SPI channels — outputs left open, MISO inputs tied 0
        .SPI_MISO_LEG1     (),           // output in V2
        .SPI_MOSI_LEG1     (),
        .SPI_SCLK_LEG1     (),
        .SPI_SS_LEG1       (),

        .SPI_MISO_LEG2     (1'b0),       // input
        .SPI_MOSI_LEG2     (),
        .SPI_SCLK_LEG2     (),
        .SPI_SS_LEG2       (),

        .SPI_MISO_LEG3     (1'b0),       // input
        .SPI_MOSI_LEG3     (),
        .SPI_SCLK_LEG3     (),
        .SPI_SS_LEG3       (),

        .SPI_MOSI_LEG4     (),
        .SPI_SS_LEG4       (),

        .SPI_MISO_LEG9     (),           // output in V2
        .SPI_MOSI_LEG9     (),
        .SPI_SCLK_LEG9     (),
        .SPI_SS_LEG9       ()
    );

    // =========================================================================
    // Small Tail FPGA DUT instances + UCSD ASIC models + I2C ADC stubs (×4)
    // tail_fpga_small has the same external port interface as record_tail_fpga.
    // =========================================================================
    genvar ch;
    generate
        for (ch = 0; ch < 4; ch = ch + 1) begin : tail_ch
`ifdef RUN_SINGLE_LEG
            if (ch == 0) begin : active_tail
`endif

            wire mclk_en_w;

            tail_fpga_small dut_tail (
                .RO1_SD                 (ro1_sd[ch]),
                .RO1_FRAME              (ro1_frame[ch]),
                .RO1_CLK                (ro1_clk[ch]),
                .RO_RSTn                (ro_rst_n[ch]),

                .SPI_RO_MOSI            (spi_ro_mosi[ch]),
                .SPI_RO_MISO            (spi_ro_miso[ch]),
                .SPI_RO_SCLK            (spi_ro_sclk[ch]),
                .SPI_RO_SS0_B           (spi_ro_ss0[ch]),
                .SPI_RO_SS1_B           (spi_ro_ss1[ch]),

                .FPGA_SPI_MOSI          (spi_mosi_t2b[ch]),
                .FPGA_SPI_MISO          (spi_miso_t2c[ch]),
                .FPGA_SPI_SCLK          (spi_sclk_t2b[ch]),
                .FPGA_SPI_SS            (spi_ss_t2b[ch]),

                .MCLK_EN                (mclk_en_w),
                .TEST_SIG               (),
                .REC_TEST_AMP_SHDN      (),

                .MCLK_20_48M            (tail_mclk_board[ch]),
                .DEVRST_N               (devrst_n),

                .SDA_CurrentSense_ADC   (tail_i2c_sda_board[ch]),
                .SCL_CurrentSense_ADC   (tail_i2c_scl_board[ch]),
                .DRDY_CurrentSense_ADC  (drdy_n[ch]),

                .LEDn_0                 ()
            );

`ifdef RUN_ASIC_START_SKEW
            // Deterministic behavioral-ASIC start phases.  The four models
            // retain their real shared MCLK and differ only in the documented
            // startup delay after MCLK_EN; FPGA and inter-FPGA signals remain
            // entirely driven by the DUT.
            ucsd_asic_model #(.SAMPLES_PER_FRAME(64), .STARTUP_DELAY_MCLK((ch + 1) * 17), .LEG_ID(ch[1:0])) asic_model (
`else
            ucsd_asic_model #(.SAMPLES_PER_FRAME(64), .LEG_ID(ch[1:0])) asic_model (
`endif
                .mclk      (tail_mclk_board[ch]),
                .mclk_en   (mclk_en_w),
                .ro_rst_n  (ro_rst_n[ch]),
                .ro1_clk   (ro1_clk[ch]),
                .ro1_sd    (ro1_sd[ch]),
                .ro1_frame (ro1_frame[ch]),
                .spi_sclk  (spi_ro_sclk[ch]),
                .spi_mosi  (spi_ro_mosi[ch]),
                .spi_miso  (spi_ro_miso[ch]),
                .spi_ss0_n (spi_ro_ss0[ch]),
                .spi_ss1_n (spi_ro_ss1[ch])
            );

            ads122c14_i2c_model i2c_adc (
                .scl    (tail_i2c_scl_board[ch]),
                .sda    (tail_i2c_sda_board[ch]),
                .drdy_n (drdy_n[ch])
            );

`ifdef RUN_SINGLE_LEG
            end
`endif
        end
    endgenerate

    // =========================================================================
    // FT600Q Transaction-Level Model
    // =========================================================================
    ft600q_tlm #(.TELEM_FRAME_LEN(4105)) u_ft600q (
        .clk_66m     (usb_fifo_clk),
        .rst_n       (devrst_n),
        .usb_fifo_d  (usb_fifo_d),
        .usb_fifo_be (usb_fifo_be),
        .rxf_n       (usb_fifo_rxf_n),
        .txe_n       (usb_fifo_txe_n),
        .rd_n        (usb_fifo_rd_n),
        .wr_n        (usb_fifo_wr_n),
        .oe_n        (usb_fifo_oe_n)
    );

    // =========================================================================
    // SPI Master BFMs — retained for waveform visibility and future debug.
    // BFM outputs would need to be forced onto spi_{sclk,mosi,ss}_c2t[ch]
    // for direct-BFM tail tests; not used in the V2 test suite.
    // =========================================================================
    wire [3:0] bfm_sclk_w, bfm_mosi_w, bfm_ss_n_w;

    genvar bfm_ch;
    generate
        for (bfm_ch = 0; bfm_ch < 4; bfm_ch = bfm_ch + 1) begin : spi_bfm
            spi_master_bfm #(.CLK_PERIOD_NS(19.531)) u_bfm (
                .sclk (bfm_sclk_w[bfm_ch]),
                .mosi (bfm_mosi_w[bfm_ch]),
                .miso (spi_miso_t2c[bfm_ch]),
                .ss_n (bfm_ss_n_w[bfm_ch])
            );
        end
    endgenerate

    // =========================================================================
    // Test task libraries
    // =========================================================================
    `include "tests/test_smoke.sv"
    `include "tests/test_con_v2.sv"
    `include "tests/test_spi_ping.sv"
    `include "tests/test_stream_aced.sv"
    `include "tests/test_stream_bist.sv"
    `include "tests/test_deferred_v2.sv"
    `include "tests/test_silent_control.sv"
    `include "tests/test_single_leg.sv"
    `include "tests/test_host_xact.sv"
    `include "tests/test_watchdog.sv"

    // =========================================================================
    // Main test sequence (default — runs when no other define selects a suite)
    // =========================================================================
`ifndef RUN_SPI_PING
`ifndef RUN_ACED
`ifndef RUN_BIST
`ifndef RUN_DEFERRED
`ifndef RUN_START_STOP
`ifndef RUN_REVERSE_SETUP
`ifndef RUN_SILENT_CONTROL
`ifndef RUN_SINGLE_LEG
`ifndef RUN_UNIQUE
`ifndef RUN_HOST_XACT
`ifndef RUN_WD_BITE
    initial begin
        @(posedge devrst_n);
        #500;
        $display("[TB] Reset released, starting V2 tests at %0t ns", $time);

        // Smoke tests
        run_SM01(spi_ss_c2t, spi_sclk_c2t);
        #10_000;
        run_SM02();
        run_SM03();
        run_SM04();

        // Consolidator common unit tests (fault, ESN)
        // C-05: run_CU02 uses 'ref' parameter — QuestaSim only; Icarus skips.
`ifndef ICARUS
        run_CU02(faultn_tb);
`endif
        run_CU03();
        run_CU04();

        // V2-specific register tests
        run_CV01();
        run_CV02();
        run_CV03();
        run_CV04();

        // SPI ping: USB → cmd_decoder → spi_cfg_ctrl → spi_master → tail_fpga_small
        run_SP01();
        run_SP02();
        run_SP03();
        run_SP04();

        #1000;
        $display("[TB] All V2 tests complete at %0t ns", $time);
        $finish;
    end

    initial begin
        #10_000_000;  // 10 ms — SM + CU + CV + SP well within budget
        $fatal(1, "[TB] Simulation timeout: V2 tests did not finish within 10 ms");
    end
`endif  // !RUN_WD_BITE
`endif  // !RUN_HOST_XACT
`endif  // !RUN_SILENT_CONTROL
`endif  // !RUN_SINGLE_LEG
`endif  // !RUN_UNIQUE
`endif  // !RUN_REVERSE_SETUP
`endif  // !RUN_START_STOP
`endif  // !RUN_DEFERRED
`endif  // !RUN_BIST
`endif  // !RUN_ACED
`endif  // !RUN_SPI_PING

    // =========================================================================
    // SA-ACED streaming test sequence (compiled with +define+RUN_ACED)
    // Runs the complete USB → SPI → tail → ASIC model → telemetry path.
    // ASIC model set to CONSTANT 0xACED; no force on any DUT/inter-FPGA signal.
    // =========================================================================
`ifdef RUN_ACED
`ifndef RUN_START_STOP
`ifndef RUN_REVERSE_SETUP
    initial begin
        @(posedge devrst_n);
        #500;
        run_SA_ACED();
        #1000;
        $finish;
    end
    initial begin
        #80_000_000;  // 80 ms — 4-ch × 1024 reads × ~10.9 µs/read × 2 frames + setup
        $fatal(1, "[TB_ACED] Timeout: SA-ACED test did not complete within 80 ms");
    end

`endif  // !RUN_REVERSE_SETUP
`endif  // !RUN_START_STOP
`endif  // RUN_ACED

`ifdef RUN_UNIQUE
    initial begin
        @(posedge devrst_n);
        #500;
        run_SA_ACED();
        #1000;
        $finish;
    end
    initial begin
        #80_000_000;
        $fatal(1, "[TB_UNIQUE] Timeout: unique-data board test did not complete within 80 ms");
    end
`endif
    // =========================================================================
    // Host SPI_CFG readback traffic during streaming — strict alignment
    // (compiled with +define+RUN_HOST_XACT; see tests/test_host_xact.sv)
    // =========================================================================
`ifdef RUN_HOST_XACT
    initial begin
        @(posedge devrst_n);
        #500;
`ifdef RUN_USB_JITTER
        run_SA_USB_JITTER();     // many short host read gaps: phase stability (tests/test_host_xact.sv)
`elsif RUN_USB_STALL
        run_SA_USB_STALL();      // host USB read gaps (tests/test_host_xact.sv)
`elsif RUN_USB_STALL_CMDS
        run_SA_USB_STALL_CMDS(); // commands issued while the host is not reading (tests/test_host_xact.sv)
`elsif RUN_FAULT_MID_CMD
        run_FAULT_MID_CMD();     // fault frame while a command arrives / is half-received (tests/test_host_xact.sv)
`elsif RUN_STREAM_DIES
        run_STREAM_DIES();       // tails go silent with RUN set: host still answered (tests/test_host_xact.sv)
`elsif RUN_LEG_PAUSE
        run_LEG_PAUSE();         // one leg pauses ~1 ms: re-admission at plane offset 0 (tests/test_host_xact.sv)
`else
        run_SA_HOST_XACT();
`endif
        #1000;
        $finish;
    end
    initial begin
        #200_000_000;   // 200 ms: 2 + 6x3 frames at 400 us plus six 800 kHz readbacks
        $fatal(1, "[TB_HOST_XACT] Timeout: host-transaction alignment test did not complete within 200 ms");
    end
`endif

`ifdef RUN_REVERSE_SETUP
    initial begin
        @(posedge devrst_n);
        #500;
        $display("[TB_REVERSE_SETUP] cold-start reversed USB/SPI Tail setup order");
        run_SA_ACED();
        #1000;
        $finish;
    end
    initial begin
        #80_000_000;
        $fatal(1, "[TB_REVERSE_SETUP] Timeout: reverse setup test did not complete within 80 ms");
    end
`endif

    // =========================================================================
    // One-leg integration: one ASIC + one Tail + V2 + FT600Q USB boundary.
    // =========================================================================
`ifdef RUN_SINGLE_LEG
    initial begin
        @(posedge devrst_n);
        #500;
        run_SA_SINGLE_LEG_ACED();
        #1000;
        $finish;
    end
    initial begin
        #80_000_000;
        $fatal(1, "[TB_SINGLE_LEG] Timeout: one-leg ACED did not complete within 80 ms");
    end
`endif

`ifdef RUN_START_STOP
    initial begin
        @(posedge devrst_n);
        #500;
        $display("[TB_STARTSTOP] two USB-controlled acquisition cycles starting");
        run_SA_ACED();
        sa_start_stop_cycle = 1;
        run_SA_ACED();
        #1000;
        $finish;
    end
    initial begin
        #100_000_000;
        $fatal(1, "[TB_STARTSTOP] Timeout: start/stop test did not complete within 100 ms");
    end
`endif

    // =========================================================================
    // SA-BIST: four-Tail self-test over the complete USB-to-telemetry path.
    // =========================================================================
`ifdef RUN_BIST
    initial begin
        @(posedge devrst_n);
        #500;
        run_SA_BIST();
        #1000;
        $finish;
    end
    initial begin
        #100_000_000;
        $fatal(1, "[TB_BIST] Timeout: full-system BIST did not complete within 100 ms");
    end
`endif  // RUN_BIST

    // =========================================================================
    // Deferred test sequence (sim_deferred.do — compiled with RUN_DEFERRED)
    // SA-04, UC-03, UC-04, FR-03 — robustness and recovery tests.
    // Timeout: 50 ms (SA-04 ~100 µs + UC-03 ~4 ms/frame + UC-04 ~4 ms/frame +
    // FR-03 < 1 ms, each with setup overhead).
    // =========================================================================
`ifdef RUN_DEFERRED
`ifndef RUN_SPI_PING
`ifndef RUN_ACED
`ifndef RUN_BIST
    initial begin
        @(posedge devrst_n);
        // Use the same reset-to-command timing as the working USB integration
        // suites.  The FT600Q BFM and DUT have completed their reset contract at
        // this point; the previous 15 us legacy delay allowed unrelated model
        // activity before SA-04 began.
        #500;
        $display("[TB_DEF] Deferred V2 tests starting at %0t ns", $time);

        run_SA04_v2();
        // FR-03 is an idle-path recovery test.  Run it before UC-03 opens the
        // continuous V2 telemetry session used by both boundary-preemption tests.
        run_FR03_v2();
        run_UC03_v2();
        run_UC04_v2();

        #1000;
        $display("[TB_DEF] All deferred V2 tests complete at %0t ns", $time);
        $finish;
    end
    initial begin
        #50_000_000;  // 50 ms
        $fatal(1, "[TB_DEF] Timeout: deferred tests did not complete within 50 ms");
    end
`endif  // !RUN_ACED
`endif  // !RUN_BIST
`endif  // !RUN_SPI_PING
`endif  // RUN_DEFERRED

    // =========================================================================
    // SA-SILENT-CTRL: all Tail links silent after legal USB acquisition arm.
    // The test must receive control responses and stop without emitting V3.
    // =========================================================================
`ifdef RUN_SILENT_CONTROL
    initial begin
        @(posedge devrst_n);
        #500;
        run_SA_SILENT_CONTROL();
        #1000;
        $finish;
    end
`endif

    // =========================================================================
    // Consolidator watchdog bite (compiled with +define+RUN_WD_BITE +define+SIM_SHORT_WD;
    // see tests/test_watchdog.sv)
    // =========================================================================
`ifdef RUN_WD_BITE
    initial begin
        @(posedge devrst_n);
        #500;
        run_WD_BITE();
        #1000;
        $finish;
    end
    initial begin
        #5_000_000;   // 5 ms: pets, bite window (~100 us short mode) and the register checks
        $fatal(1, "[TB_WD] Timeout: watchdog bite test did not complete within 5 ms");
    end
    initial begin
        #30_000_000;
        $fatal(1, "[TB_SILENT_CTRL] Timeout: silent control arbitration did not complete within 30 ms");
    end
`endif

    // =========================================================================
    // SPI Ping-only sequence (sim_spi_ping.do — compiled with RUN_SPI_PING)
    // =========================================================================
`ifdef RUN_SPI_PING
    initial begin
        @(posedge devrst_n);
        #500;
        $display("[TB_PING] SPI ping sequence starting at %0t ns", $time);
        run_SP01();
        run_SP02();
        run_SP03();
        run_SP04();
        #1000;
        $display("[TB_PING] All SPI ping tests complete at %0t ns", $time);
        $finish;
    end
    initial begin
        #5_000_000;  // 5 ms — each ping < 100 µs; 4 channels well within budget
        $fatal(1, "[TB_PING] Timeout: SPI ping tests did not complete within 5 ms");
    end

    // =========================================================================
    // Debug monitoring — traces USB handshake and cmd_decoder state.
    // Remove once the 5ms timeout root cause is identified.
    // =========================================================================
    initial u_ft600q.dbg_tx_trace = 1'b1;

    // Edge-detect on key USB FIFO signals (top-level wires, always visible)
    always @(usb_fifo_rxf_n)
        $display("[DBG_RXF] t=%0t rxf_n=%b (0=BFM has data for FPGA)", $time, usb_fifo_rxf_n);
    always @(negedge usb_fifo_wr_n)
        $display("[DBG_WR]  t=%0t wr_n ASSERTED — FPGA writing D=0x%04h to BFM", $time, usb_fifo_d);
    always @(negedge usb_fifo_oe_n)
        $display("[DBG_OE]  t=%0t oe_n ASSERTED — USB FSM started READ cycle", $time);

    // Periodic snapshot every 10 µs for first 2 ms — reveals where the stall is
    initial begin
        @(posedge devrst_n);
        #1000;
        repeat(200) begin
            #10000;
            $display("[POLL] t=%0t cmd_st=%0d fb=%b rx_v=%b rx_r=%b tx_v=%b tx_r=%b rx_mt=%b tx_mt=%b ctrl_wr=%0d",
                $time,
                dut_con.cmd_dec.state, dut_con.framer_busy,
                dut_con.cmd_rx_valid,  dut_con.cmd_rx_ready,
                dut_con.ctrl_tx_valid, dut_con.ctrl_tx_ready,
                dut_con.cdc_rx_empty,  dut_con.cdc_tx_empty,
                u_ft600q.ctrl_wr_ptr);
        end
    end
`endif

    // =========================================================================
    // Waveform dump
    // =========================================================================
    initial begin
        `ifdef DUMP_VCD
            $dumpfile("tb_top.vcd");
            $dumpvars(0, tb_top);
        `endif
    end

endmodule
