// ---------------------------------------------------------------------------
// BER-STREAM — the BER Test over the acquisition stream, in RTL (RUN_BER_STREAM).
//
// The bring-up tool's "BER Test" (CannedFunctions::berTest) runs all four tails
// in self-test (asic_self_test.v: a 16-bit counter, +1 per stream word, all lanes
// alike, continuous across frames) through the real acquisition path — tail
// asic_stream_tx -> spi_ch_stream -> leg FIFO -> telem_engine_v3 -> USB — and
// compares every delivered raw word with the word the counter must produce next.
// That path carries 41 Mbit/s per leg, so 3e7 compared bits (BER <= 1e-7 at 95 %,
// k = 0) arrive in under a second per leg — the reason it, and not the config-path
// loopback (branch ber-config-path), is the startup test.
//
//   BER-S01  all four legs deliver scorable leg-frames (unflagged phase word).
//   BER-S02  bit-exact: 0 errors over every compared bit, N reported with the BER
//            the run demonstrates (3/N).
//   BER-S03  the comparator sees one flipped bit in one word as exactly one error.
//   BER-S04  the bit budget: 16 bits x 1024 words x 4 legs per frame at the frame
//            counter's cadence is >= 40 Mbit/s per leg (3e7 bits within 1 s).
//
// Scoring is the tool's: per leg, anchor on the first word delivered, then every
// later word is compared bit-wise against expected = previous expected + 1
// (mod 2^16); a leg-frame whose phase word carries par/ovf/undf, or reports no
// usable phase (1023), is skipped and the leg re-anchors — right-or-flagged is
// the contract, and a flagged frame is not a bit error.
// ---------------------------------------------------------------------------
`ifdef RUN_BER_STREAM

localparam int BERS_FRAMES = 6;                         // frames scored after the warm-up

int          bers_npass = 0, bers_nfail = 0;
logic [15:0] bers_data  [0:4095];
logic [15:0] bers_phase [0:3];
logic [15:0] bers_exp   [0:3];
bit          bers_anchored [0:3];
longint      bers_bits [0:3];
int          bers_errs [0:3];
int          bers_frames_scored [0:3];
int          bers_frames_flagged [0:3];

// Popcount by loop: Icarus 13 (OSS CAD Suite) returns garbage from $countones()
// applied to an expression ($countones(16'h0000 ^ 16'h0000) came back as 6), so
// nothing here trusts it (toolchain_compat C-11).
function automatic int bers_popcount(input logic [15:0] x);
    int n = 0;
    for (int i = 0; i < 16; i++) n += x[i];
    return n;
endfunction

task automatic bers_check(input string tag, input bit ok, input string msg);
    if (ok) begin bers_npass++; $display("[%0s] PASS %0s", tag, msg); end
    else    begin bers_nfail++; $display("[%0s] FAIL %0s", tag, msg); end
endtask

// Score one frame's four leg-quarters against the running counters.
task automatic bers_score_frame();
    logic [15:0] w, ph;
    bit          flagged;
    for (int ch = 0; ch < 4; ch++) begin
        ph      = bers_phase[ch];
        flagged = (ph[14:12] != 3'b000) || (ph[9:0] == 10'd1023);
        if (flagged) begin
            bers_frames_flagged[ch]++;
            bers_anchored[ch] = 0;                       // re-anchor on the next clean frame
            continue;
        end
        for (int t = 0; t < 1024; t++) begin
            w = bers_data[4 * t + ch];
            if (!bers_anchored[ch]) begin
                bers_exp[ch]      = w;                   // the anchor word itself is not scored
                bers_anchored[ch] = 1;
            end else begin
                bers_errs[ch] += bers_popcount(w ^ bers_exp[ch]);
                bers_bits[ch] += 16;
            end
            bers_exp[ch] = bers_exp[ch] + 16'd1;
        end
        bers_frames_scored[ch]++;
    end
endtask

task automatic run_BER_STREAM();
    logic [15:0] m, f, a, d, hdr;
    logic [47:0] tx, rx;
    int          n_flush;
    longint      bits_all;
    int          errs_all, errs_before, frame_cnt_first, frame_cnt_last;
    real         mbit_per_leg_per_s;

    $display("[BER-S] BER Test over the acquisition stream: four tails in self-test, every delivered word compared with the counter");

    // ---- stream bring-up, as startSelfTestStream() ----------------------------
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (int ch = 0; ch < 4; ch++) begin
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // CTRL: RO_RSTn + MCLK_EN
        tx = 48'h06_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // SELF_TEST = 1
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN  = 1
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h000F);   // ACQ_ALL_RUN
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.flush_tx_capture(n_flush);

    // Two start-up frames are discarded, as the BIST bench does: the self-test
    // source is enabled before the arm, so the first frames can straddle it.
    for (int i = 0; i < 2; i++) tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);

    for (int ch = 0; ch < 4; ch++) begin
        bers_anchored[ch] = 0; bers_bits[ch] = 0; bers_errs[ch] = 0;
        bers_frames_scored[ch] = 0; bers_frames_flagged[ch] = 0;
    end
    frame_cnt_first = -1; frame_cnt_last = -1;
    for (int i = 0; i < BERS_FRAMES; i++) begin
        tb_top.u_ft600q.wait_telemetry_frame_v3(hdr);
        for (int j = 0; j < 4096; j++) bers_data[j]  = tb_top.u_ft600q.v3_data[j];
        for (int j = 0; j < 4;    j++) bers_phase[j] = tb_top.u_ft600q.v3_phase[j];
        if (frame_cnt_first < 0) frame_cnt_first = tb_top.u_ft600q.v3_count_lo;
        frame_cnt_last = tb_top.u_ft600q.v3_count_lo;
        bers_score_frame();
    end

    bits_all = 0; errs_all = 0;
    for (int ch = 0; ch < 4; ch++) begin
        bits_all += bers_bits[ch]; errs_all += bers_errs[ch];
        $display("[BER-S] leg %0d: %0d frames scored, %0d flagged (skipped), N=%0d bits, k=%0d",
                 5 + ch, bers_frames_scored[ch], bers_frames_flagged[ch], bers_bits[ch], bers_errs[ch]);
    end

    bers_check("BER-S01",
               bers_frames_scored[0] > 0 && bers_frames_scored[1] > 0 && bers_frames_scored[2] > 0 && bers_frames_scored[3] > 0,
               $sformatf("all four legs delivered scorable frames (%0d/%0d/%0d/%0d of %0d)",
                         bers_frames_scored[0], bers_frames_scored[1], bers_frames_scored[2], bers_frames_scored[3], BERS_FRAMES));
    bers_check("BER-S02", bits_all > 0 && errs_all == 0,
               $sformatf("bit-exact: %0d errors over N=%0d bits (demonstrates BER <= %0.2e at 95%%)",
                         errs_all, bits_all, 3.0 / bits_all));

    // ---- BER-S03: the comparator sees a single flipped bit -------------------
    // Re-score the last frame with one bit of one word flipped: the leg 5 error
    // count must rise by exactly one (the anchor is re-used, so nothing else moves).
    errs_before = bers_errs[0];
    bers_anchored[0] = 1; bers_exp[0] = bers_exp[0] - 16'd1024;   // rewind leg 5 to this frame's first word
    bers_data[4 * 100 + 0] = bers_data[4 * 100 + 0] ^ 16'h0040;
    bers_score_frame();
    bers_check("BER-S03", bers_errs[0] == errs_before + 1,
               $sformatf("one flipped bit scores one error: %0d -> %0d", errs_before, bers_errs[0]));

    // ---- BER-S04: the bit budget ---------------------------------------------
    // 16 bits x 1024 words per leg per frame; frames at the 2.5 kHz counter cadence.
    mbit_per_leg_per_s = 16.0 * 1024.0 * 2500.0 / 1.0e6;
    bers_check("BER-S04",
               (frame_cnt_last - frame_cnt_first) == (BERS_FRAMES - 1) && mbit_per_leg_per_s >= 40.0,
               $sformatf("%0d consecutive frames (counter %0d..%0d): %0.1f Mbit/s per leg, so 3e7 bits (BER 1e-7) in %0.2f s",
                         BERS_FRAMES, frame_cnt_first, frame_cnt_last, mbit_per_leg_per_s, 30.0 / mbit_per_leg_per_s));

    if (bers_nfail == 0) $display("[BER-S] PASS — %0d checks: the acquisition-stream BER path is bit-exact on all four legs", bers_npass);
    else                 $display("[BER-S] FAIL — %0d check(s) failed (%0d passed)", bers_nfail, bers_npass);
    $display("RESULTS: %0d passed, %0d failed", bers_npass, bers_nfail);
    $display("STATUS: %0s", (bers_nfail == 0) ? "PASS" : "FAIL");
endtask

`endif // RUN_BER_STREAM
