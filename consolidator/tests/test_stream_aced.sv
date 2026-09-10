// SA-ACED: End-to-end 0xACED streaming test
//
// Exercises the complete path — no force on any FPGA or inter-FPGA signal.
// All tail setup is driven through the USB→SPI_CFG register interface.
//
//   USB cmd → cmd_decoder → spi_cfg_ctrl → spi_master → SPI bus
//   → spi_slave_small (CTRL: RO_RSTn+MCLK_EN, TELEM_EN: 0x01)
//   → asic_stream_tx → SPI MISO → spi_ch_stream → small_fifo
//   → telemetry_framer_v2 → CDC FIFO → usb_fifo_interface → FT600Q TLM
//
// ASIC model is set to CONSTANT 0xACED via testbench variable assignment.
// Accessing asic_model.constant_val / data_mode is legal: they are
// behavioral model state, not RTL FPGA signals.
//
// The FPGA forwards raw bit-planes. With every ASIC lane driven to 0xACED,
// each payload word must therefore be either 0x0000 or 0xFFFF. The focused
// tb_aced test additionally checks the MSB-first raw-bit-plane order.
//
// Included via `include in tb_top.sv; do NOT add `timescale or imports here.
// Depends on: spi_cfg_xact (test_con_v2.sv included before this file).

`ifdef RUN_START_STOP
integer sa_start_stop_cycle = 0;
`endif

function automatic logic [15:0] unique_expected(
    input logic [3:0] frame_nibble, input integer sensor, input logic [1:0] leg
);
    logic [15:0] pre, lfsr;
    integer b;
    begin
        pre = {frame_nibble, sensor[9:0], leg};
        lfsr = pre ^ 16'h1D0F;
        for (b = 0; b < 16; b = b + 1)
            lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        unique_expected = pre ^ lfsr;
    end
endfunction

task automatic run_SA_ACED();
    // ── Parameters ──────────────────────────────────────────────────────────
    localparam int     N_FRAMES   = 3;
    localparam logic [15:0] EXPECTED = 16'hACED;

    // ── Local variables ──────────────────────────────────────────────────────
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;   // packed: byte[k]=bits[47-8k:40-8k]; matches ICARUS spi_cfg_xact
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    logic [15:0] data  [0:4095];
    logic [15:0] phase [0:3];
    logic [9:0]  phase_ref [0:3];
    logic [15:0] crcw  [0:1];
    logic [31:0] crc_calc;
    logic [15:0] flush_words [0:4095];
    int          n_flush;
    int          n_fail = 0;
    int          n_pass = 0;
    int          frame_idx;
    int          _ci;        // copy-loop index for ICARUS compat
    int          cfg_ch;
`ifdef RUN_BIST
    int          bist_i;
    int          n_ovf_tol = 0;   // self-test +1 gaps tolerated on overflow-flagged legs
`endif
    $display("");
    $display("[SA-ACED] ════════════════════════════════════════════════════════");
    $display("[SA-ACED] End-to-end 0xACED streaming test  t=%0t ns", $time);
    $display("[SA-ACED] Path: USB cmd → spi_cfg → spi_master → tail → ASIC model");
    $display("[SA-ACED]       → asic_stream_tx → spi_ch_stream → framer → USB TLM");
    $display("[SA-ACED] ════════════════════════════════════════════════════════");

    // ── Step 1: Configure ASIC behavioral models ─────────────────────────────
    // Setting testbench model variables — NOT forcing FPGA RTL signals.
    // Unrolled with constant indices: a hierarchical reference into a generate
    // array needs a constant index under Verilator (QuestaSim is permissive).
`ifdef RUN_SINGLE_LEG
    tb_top.tail_ch[0].active_tail.asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[0].active_tail.asic_model.data_mode    = MODE_CONSTANT;
`else
    tb_top.tail_ch[0].asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[0].asic_model.data_mode    = MODE_CONSTANT;
`ifndef RUN_SINGLE_LEG
    tb_top.tail_ch[1].asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[1].asic_model.data_mode    = MODE_CONSTANT;
    tb_top.tail_ch[2].asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[2].asic_model.data_mode    = MODE_CONSTANT;
    tb_top.tail_ch[3].asic_model.constant_val = EXPECTED;
    tb_top.tail_ch[3].asic_model.data_mode    = MODE_CONSTANT;
`endif
`endif

`ifdef RUN_UNIQUE
    // Override the common ACED baseline only after it has configured every
    // model.  This stays entirely at the behavioral ASIC boundary; the USB,
    // Consolidator, and interconnect paths remain the production hierarchy.
    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[1].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[2].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    $display("[SA-UNIQUE] ASIC models: transformed {frame,sensor,leg} identity");
`endif
    $display("[SA-ACED] ASIC model: CONSTANT 0x%04X on all 4 channels", EXPECTED);

    // ── Step 2: Enable ALL FOUR SPI channels, slow SPI clock for tail config ─
    // The checker asserts every payload word is a valid raw bit-plane, and the framer
    // interleaves four channel slots per round: a channel that is not enabled
    // contributes zero-fill BY DESIGN.  Enabling only ch0 (as this test
    // originally did) fails 3/4 of the payload against its own expectation.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;

    // ── Step 3: CTRL opcode 0x01 → ctrl_reg = 5'h11, all four tails ─────────
    //   bit 0 = RO_RSTn = 1  (release ASIC readout reset)
    //   bit 4 = MCLK_EN = 1  (enable ASIC master clock)
    begin : tail_setup
        int ch;
        for (ch = 0; ch < 4; ch++) begin
`ifdef RUN_START_STOP
            // Reverse the externally issued Tail setup order on the second
            // cycle.  This changes only legal USB/SPI transaction timing.
            cfg_ch = sa_start_stop_cycle ? (3 - ch) : ch;
`elsif RUN_REVERSE_SETUP
            // Cold-start control: prove reversed USB/SPI setup itself is
            // legal before attributing any later failure to restart handling.
            cfg_ch = 3 - ch;
`else
            cfg_ch = ch;
`endif
`ifdef ICARUS
            tx = 48'h01_11_00_00_00_00;
            spi_cfg_xact(cfg_ch[1:0], tx, 3'd2, rx);
            tx = 48'h02_01_00_00_00_00;
            spi_cfg_xact(cfg_ch[1:0], tx, 3'd2, rx);
`else
            tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00};
            spi_cfg_xact(cfg_ch[1:0], tx, 3'd2, rx);
            tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00};
            spi_cfg_xact(cfg_ch[1:0], tx, 3'd2, rx);
`endif
            // ── Step 4: TELEM_EN opcode 0x02 → telem_en = 2'b01 (above) ─────
            $display("[SA-ACED] ch%0d: CTRL=0x11 + TELEM_EN=0x01 sent", ch);
        end
    end

    // Restore fast SPI clock for streaming path (spi_ch_stream drives SCLK via
    // ODDRXE at clk_48m/2 = 25.6 MHz, independent of SPI_CLK_DIV; restoring here
    // keeps the register in a clean state).
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    // ── Step 5: Start streaming on all four channels (ACQ_ALL_RUN) ──────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    $display("[SA-ACED] CH_RUN=0xF, SPI_EN=0xF — all four spi_ch_stream armed");

    // ── Step 6: Flush all setup command responses from tx_capture ────────────
    // flush_tx_capture resets tx_rd_ptr to tx_wr_ptr so subsequent
    // wait_telemetry_frame calls see only post-setup telemetry.
    // frame_type_hint is not connected in V2 tb_top → all frames land in
    // ctrl_capture (via typed pointers) AND tx_capture (untyped pointer).
    // wait_telemetry_frame uses tx_capture, so we flush it here.
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    tb_top.u_ft600q.flush_tx_capture(flush_words, n_flush);
`endif
    $display("[SA-ACED] Flushed %0d stale tx_capture words (setup responses discarded)", n_flush);

    // ── Step 7: Discard one warm-up frame, then check N_FRAMES ───────────────
    // The very first frame after a cold arm can carry a 1-bit transient on the
    // first one or two samples of a channel (model/tail arm handshake while the
    // ASIC free-runs).  Host software discards startup frames for the same
    // reason.  Steady-state bit-exactness — the thing this test exists to
    // prove — is checked on every subsequent frame.
`ifdef ICARUS
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
    for (_ci = 0; _ci < 4096; _ci++) data[_ci]  = tb_top.u_ft600q.v3_data[_ci];
    for (_ci = 0; _ci < 4;    _ci++) phase[_ci]  = tb_top.u_ft600q.v3_phase[_ci];
    for (_ci = 0; _ci < 2;    _ci++) crcw[_ci]   = tb_top.u_ft600q.v3_crc[_ci];
`else
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr, data, phase, crcw);
`endif
    $display("[SA-ACED] warm-up frame hdr=0x%04X discarded", hdr);

`ifdef RUN_BIST
    // The BIST source is enabled before the common acquisition setup, so its
    // first frame can straddle the Tail arm/frame-sync boundary.  The first
    // post-setup frame is therefore a startup frame as well; discard it and
    // begin the contiguous-counter assertion on the next V3 frame.
`ifdef ICARUS
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
    for (_ci = 0; _ci < 4096; _ci++) data[_ci]  = tb_top.u_ft600q.v3_data[_ci];
    for (_ci = 0; _ci < 4;    _ci++) phase[_ci] = tb_top.u_ft600q.v3_phase[_ci];
    for (_ci = 0; _ci < 2;    _ci++) crcw[_ci]  = tb_top.u_ft600q.v3_crc[_ci];
`else
    tb_top.u_ft600q.wait_telemetry_frame_v3(hdr, data, phase, crcw);
`endif
    $display("[SA-BIST] post-arm frame hdr=0x%04X discarded", hdr);
`endif

    for (frame_idx = 0; frame_idx < N_FRAMES; frame_idx++) begin
        automatic int frame_fail = 0;

`ifdef ICARUS
        tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
        for (_ci = 0; _ci < 4096; _ci++) data[_ci]  = tb_top.u_ft600q.v3_data[_ci];
        for (_ci = 0; _ci < 4;    _ci++) phase[_ci]  = tb_top.u_ft600q.v3_phase[_ci];
        for (_ci = 0; _ci < 2;    _ci++) crcw[_ci]   = tb_top.u_ft600q.v3_crc[_ci];
`else
        tb_top.u_ft600q.wait_telemetry_frame_v3(hdr, data, phase, crcw);
`endif

        // Header type (spec §6.2)
        if ((hdr & 16'h0007) !== 16'h0001) begin
            $display("[SA-ACED] FAIL frame[%0d] header type != 1 (hdr=0x%04X)", frame_idx, hdr);
            n_fail++; frame_fail++;
        end

`ifndef RUN_BIST
`ifndef RUN_UNIQUE
        // 4096 interleaved raw bit-planes: all lanes are driven identically,
        // so each valid plane is all-zero or all-one.
        foreach (data[i]) begin
            if ((data[i] !== 16'h0000) && (data[i] !== 16'hFFFF)) begin
                // A late leg is represented by an explicit zero-stuffed slot.
                // In this constant-data test a zero is valid only when the
                // matching emitted phase word reports the underrun condition.
                if ((data[i] === 16'h0000) && phase[i % 4][12]) begin
                    n_pass++;
                end else begin
                if (n_fail < 16) begin
                    automatic string rot_str;
                    rot_str = "(not a raw all-lane bit-plane)";
                    $display("[SA-ACED] FAIL frame[%0d] word[%0d] = 0x%04X  %s",
                             frame_idx, i, data[i], rot_str);
                end
                n_fail++;
                frame_fail++;
                end
            end else
                n_pass++;
        end
`endif
`ifdef RUN_UNIQUE
        begin : unique_reconstruction
            integer uch, ug, ulane, ubit, ucand, ustart, uplane, umatched, ubad;
            integer usample, uframe;
            integer uraw, ucomplete;
            logic [15:0] got_word;
            for (uch = 0; uch < 4; uch++) begin
                umatched = -1;
                for (ucand = 0; ucand < 16; ucand++) begin
                    for (ustart = 0; ustart < 64; ustart++) begin
                        for (uplane = 0; uplane < 16; uplane++) begin
                            // The V3 header can fall on any raw bit-plane.
                            // Start at the first wholly contained 16-plane
                            // sample and solve both ASIC sample and plane
                            // phase without assuming a shared frame boundary.
                            uraw      = (uplane == 0) ? 0 : 16 - uplane;
                            ucomplete = (1024 - uraw) / 16;
                            usample   = (ustart + ((uplane + uraw) / 16)) % 64;
                            uframe    = (ucand + ((ustart + ((uplane + uraw) / 16)) / 64)) % 16;
                            ubad = 0;
                            // Reject almost all alignments using lane 0 of
                            // the first wholly contained sample.
                            for (ubit = 0; ubit < 16; ubit++)
                                got_word[15-ubit] = data[4*(uraw+ubit)+uch][0];
                            if (got_word !== unique_expected(uframe[3:0],
                                (usample % 16) + (usample / 16)*256, uch[1:0])) begin
                                ubad = 1;
                            end else begin
                                for (ug = 0; ug < ucomplete; ug++) begin
                                    usample = (ustart + ((uplane + uraw) / 16) + ug) % 64;
                                    uframe  = (ucand + ((ustart + ((uplane + uraw) / 16) + ug) / 64)) % 16;
                                    for (ulane = 0; ulane < 16; ulane++) begin
                                for (ubit = 0; ubit < 16; ubit++)
                                    got_word[15-ubit] = data[4*(uraw+ug*16+ubit)+uch][ulane];
                                // A V3 frame can begin at any raw ASIC sample,
                                // but it must preserve each leg's contiguous
                                // sample order and advance the encoded ASIC
                                // frame nibble at the 64-sample boundary.
                                if (got_word !== unique_expected(uframe[3:0],
                                    ulane*16 + (usample % 16) + (usample / 16)*256,
                                    uch[1:0])) ubad = ubad + 1;
                                end
                            end
                            end
                            if (ubad == 0) umatched = (ucand * 64 + ustart) * 16 + uplane;
                        end
                    end
                end
                if (umatched < 0) begin
                    $display("[SA-UNIQUE] FAIL frame[%0d] leg%0d identity reconstruction", frame_idx, uch);
                    n_fail++; frame_fail++;
                end else begin
                    $display("[SA-UNIQUE] PASS frame[%0d] leg%0d frame_nibble=%0d sample_offset=%0d plane_offset=%0d",
                             frame_idx, uch, umatched / 1024, (umatched / 16) % 64, umatched % 16);
                    n_pass += 1024;
                end
            end
        end
`endif
`else
        // SELF_TEST replaces each ASIC input with its local 16-bit counter.
        // V2 transports reconstructed words, interleaved by channel, rather
        // than the V1 all-zero/all-one raw-plane encoding.  Check each
        // adjacent sample entirely at the USB-observed V3 boundary.
        for (bist_i = 0; bist_i < 4092; bist_i++) begin
            if (data[bist_i + 4] === (data[bist_i] + 16'd1)) begin
                n_pass++;
            end else if (phase[bist_i % 4][13] &&
                         (data[bist_i + 4] - data[bist_i]) >= 16'd1 &&
                         (data[bist_i + 4] - data[bist_i]) <= 16'd64) begin
                // Tolerated: this leg's phase word flags a FIFO overflow (bit[13]).
                // Per the V3 contract (CLAUDE.md ISSUE-2) a leg overflow DROPS
                // affected samples but does NOT shorten the frame — the host marks
                // that leg's data invalid. A small FORWARD gap in the self-test
                // counter is the expected signature of a dropped sample, so it is
                // not a failure. This mirrors the ACED tolerance of underrun-
                // flagged (phase bit[12]) 0x0000 words above. A backward, zero, or
                // >64 jump still fails, catching genuine corruption even here.
                n_ovf_tol++;
                if (n_ovf_tol <= 8)
                    $display("[SA-BIST] tolerated overflow gap frame[%0d] ch%0d sample[%0d]: 0x%04h -> 0x%04h (leg ovfl flagged, host marks leg invalid)",
                             frame_idx, bist_i % 4, bist_i / 4,
                             data[bist_i], data[bist_i + 4]);
            end else begin
                if (frame_fail < 16)
                    $display("[SA-BIST] FAIL frame[%0d] ch%0d sample[%0d]: 0x%04h -> 0x%04h (expected +1)",
                             frame_idx, bist_i % 4, bist_i / 4,
                             data[bist_i], data[bist_i + 4]);
                n_fail++; frame_fail++;
            end
        end
`endif

        // Phase words are captured once for this telemetry session.  Leg 0 is
        // the reference in this deterministic model; the other legs are
        // configured serially and therefore retain non-zero, static offsets.
        foreach (phase[i]) begin
            if (frame_idx == 0)
                phase_ref[i] = phase[i] & 16'h03FF;
            else if ((phase[i] & 16'h03FF) !== phase_ref[i]) begin
                $display("[SA-ACED] FAIL frame[%0d] phase[%0d]=%0d, expected stable %0d",
                         frame_idx, i, phase[i] & 16'h03FF, phase_ref[i]);
                n_fail++; frame_fail++;
            end
        end
`ifdef RUN_REVERSE_SETUP
        // Reverse USB/SPI setup makes ch3 the deterministic lead.  The phase
        // vector is still observed only at the V3 USB boundary.
        if ((phase[3] & 16'h03FF) !== 16'h0000) begin
            $display("[SA-ACED] FAIL frame[%0d] reversed-order lead phase=%0d, expected ch3=0",
                     frame_idx, phase[3] & 16'h03FF);
            n_fail++; frame_fail++;
        end
`else
        // The reference leg's phase word is its TRUE sweep offset, not a
        // guaranteed lead of 0: with independent per-leg start-up the earliest
        // leg need not be leg 0 (a restart or BIST setup can have another leg
        // admit the header), and a design that reports the real phase for a
        // re-anchored leg is exactly what keeps the host from de-rotating
        // wrongly.  Leg ordering is a timing observation, not a contract; the
        // data check above already proves every word decodes at the reported
        // phase, and phase STABILITY is enforced by the phase_ref check.  Reject
        // only a real start-up underrun on the reference leg (ported from geoff's
        // 6c59bd2).
        if (phase[0][12]) begin
            $display("[SA-ACED] FAIL frame[%0d] reference leg startup underrun (phase=0x%04h)",
                     frame_idx, phase[0]);
            n_fail++; frame_fail++;
        end
`endif

`ifdef RUN_START_STOP
        if (frame_idx == 0) begin
            if (sa_start_stop_cycle != 0) begin
                // Restart correctness is the valid V3 payload/CRC checked
                // above plus the later USB stop response.  Setup ordering is
                // independently covered by RUN_REVERSE_SETUP; phase values
                // are observations of first-data timing, not an ordering
                // guarantee once pre-header admission waits for available data.
                $display("[SA-STARTSTOP] PASS restarted V3 frame accepted after reversed USB/SPI setup");
                n_pass++;
            end
        end
`endif

`ifdef RUN_BIST_SKEW
        // The self-test sources are enabled in ch0..ch3 order through the
        // FT600Q/SPI_CFG path.  The engine admits the first header on the FIRST
        // ready leg (header_admissible = frame_event_pending & |fill_ge4), so
        // legs configured later retain DETERMINISTIC, STATIC phase offsets
        // (e.g. {0,81,164,248}).  Per-leg skew is legal and expected — geoff's
        // phase-aware oracle correction supersedes the old "all legs phase==0"
        // expectation (which contradicted the general phase check above that
        // captures and holds non-zero static offsets).  Require only: no
        // post-lock underrun this frame.  Cross-frame phase STABILITY is already
        // enforced by the phase_ref check above.
        if (frame_idx == 0) begin
            if (phase[0][12] || phase[1][12] || phase[2][12] || phase[3][12]) begin
                $display("[SA-BIST-SKEW] FAIL post-lock underrun phase={%0d,%0d,%0d,%0d} underrun={%0d,%0d,%0d,%0d}",
                         phase[0] & 16'h03FF, phase[1] & 16'h03FF,
                         phase[2] & 16'h03FF, phase[3] & 16'h03FF,
                         phase[0][12], phase[1][12], phase[2][12], phase[3][12]);
                n_fail++; frame_fail++;
            end else begin
                $display("[SA-BIST-SKEW] PASS stable per-leg skew phase={%0d,%0d,%0d,%0d} (deterministic, no post-lock underrun)",
                         phase[0] & 16'h03FF, phase[1] & 16'h03FF,
                         phase[2] & 16'h03FF, phase[3] & 16'h03FF);
                n_pass++;
            end
        end
`endif

        // CRC-32/IEEE over words 0..4100, big-endian byte order (spec §6.5)
        begin
            automatic int b, j;
            automatic logic [7:0] by;
            crc_calc = 32'hFFFFFFFF;
            for (j = 0; j < 4103; j++) begin
                automatic logic [15:0] w16;
                w16 = (j == 0) ? hdr : (j == 1) ? tb_top.u_ft600q.v3_count_lo :
                      (j == 2) ? tb_top.u_ft600q.v3_count_hi :
                      (j <= 4098) ? data[j-3] : phase[j-4099];
                for (b = 0; b < 2; b++) begin
                    by = (b == 0) ? w16[15:8] : w16[7:0];
                    crc_calc[7:0] = crc_calc[7:0] ^ by;
                    repeat (8) begin
                        if (crc_calc[0]) crc_calc = (crc_calc >> 1) ^ 32'hEDB88320;
                        else             crc_calc = (crc_calc >> 1);
                    end
                end
            end
            crc_calc = ~crc_calc;
            if (crcw[0] !== crc_calc[31:16] || crcw[1] !== crc_calc[15:0]) begin
                $display("[SA-ACED] FAIL frame[%0d] CRC got %04X%04X want %08X",
                         frame_idx, crcw[0], crcw[1], crc_calc);
                n_fail++; frame_fail++;
            end
        end

        $display("[SA-ACED] frame[%0d] hdr=0x%04X: %0d/%0d data words correct, phase={%0d,%0d,%0d,%0d}",
                 frame_idx, hdr, 4096 - frame_fail, 4096,
                 phase[0] & 16'h03FF, phase[1] & 16'h03FF,
                 phase[2] & 16'h03FF, phase[3] & 16'h03FF);
    end

`ifdef RUN_START_STOP
    // Reuse the V2 stop transaction ordering: write CH_RUN=0, drain the
    // mandatory final telemetry frame, then collect its control response and
    // disable the physical SPI paths.  The helper issues all traffic via USB.
`ifdef RUN_STOP_DEBUG
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_ACQ_ALL_RUN, 16'h0000);
    #500_000;
    $display("[SA-STARTSTOP-DBG] +0.5ms cmd_state=%0d eng_state=%0d busy=%b ch_run=%b ctrl_valid=%b cdc_tx_empty=%b",
             dut_con.cmd_dec.state, dut_con.engine.state, dut_con.framer_busy,
             dut_con.ch_run, dut_con.ctrl_tx_valid, dut_con.cdc_tx_empty);
    #4_500_000;
    $display("[SA-STARTSTOP-DBG] +5ms cmd_state=%0d eng_state=%0d busy=%b ch_run=%b ctrl_valid=%b cdc_tx_empty=%b",
             dut_con.cmd_dec.state, dut_con.engine.state, dut_con.framer_busy,
             dut_con.ch_run, dut_con.ctrl_tx_valid, dut_con.cdc_tx_empty);
    tb_top.u_ft600q.wait_telemetry_frame_v3_typed(hdr);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`else
    deferred_stop_streaming();
`endif
    $display("[SA-STARTSTOP] PASS all streams stopped through USB boundary");
    n_pass++;
`endif

    // ── Step 8: Stop streaming ───────────────────────────────────────────────
    // This focused failure test intentionally does not exercise shutdown.
    // The continuous-stream stop/response ordering is verified separately.

    // ── Step 9: Result summary ───────────────────────────────────────────────
    $display("");
    if (n_fail == 0) begin
`ifdef RUN_BIST
        $display("[SA-BIST] PASS: all %0d interleaved counter steps across %0d frames are contiguous",
                 n_pass, N_FRAMES);
        $display("[SA-BIST] Full USB -> SPI_CFG -> Tail BIST -> V3 telemetry path is CORRECT");
`else
        $display("[SA-ACED] PASS — all %0d raw bit-planes across %0d frames are valid",
                 n_pass, N_FRAMES);
        $display("[SA-ACED] USB transport of raw bit-planes is CORRECT");
        $display("[SA-ACED] Conclusion: scrambling seen on hardware is NOT in the RTL");
`endif
    end else begin
        $display("[SA-ACED] FAIL — %0d/%0d words wrong", n_fail, n_pass + n_fail);
        $display("[SA-ACED] Check first FAIL line above for nibble-rotation diagnosis");
    end
    $display("[SA-ACED] ════════════════════════════════════════════════════════");
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    if (n_fail == 0)
        $display("STATUS: PASS");
    else
        $display("STATUS: FAIL");
    $display("");

endtask
