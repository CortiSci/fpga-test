// ---------------------------------------------------------------------------
// SA-DEAD-LEG-CEILING — the "drops" of asic_grid_asic_130108 (2026-09-18 13:01),
// reproduced. Fails on the pre-fix RTL; protects intact delivered frames under
// a sustained bandwidth deficit on the fixed RTL.
//
// The recording (main @ 7948148 flashed, all four legs enabled, 10.4 s = 25,924
// frames).  Jeremy saw "drops and spikes maybe once every 3 seconds" on the
// healthy legs.  What the data says:
//   * They are CONSTANT, not occasional: 2,075 events where a healthy leg (5, 6
//     or 8) is flagged undf -- one every ~6 frames, ~19 % of frames, flat across
//     the run (11-31 % per 250 ms bin) with NO periodicity (undf autocorrelation
//     r < 0.04 anywhere in 0.5-5 s).  The tool conceals flagged frames, so only
//     the odd one that coincides with a display refresh is seen -- hence "every
//     3 s".  Every frame the tool DOES show is right: no leg-wide or per-sensor
//     jump anywhere in the displayed (de-rotated, concealed) series, so
//     right-or-flagged holds and this is not silent corruption.
//   * 74 % hit legs 5, 6 and 8 in the SAME frame with their phases moving by the
//     same amount; 60 % re-anchor (1023 -> a new phase: leg-FIFO overflow), 40 %
//     pure underrun (undf, phase kept: the escape released a healthy leg).
//     Inside a flagged frame ~30 % of the leg's words are the stuffed 0x0000.
//   * The HOST did not stall.  The tool's own capture stats for the recording:
//     "reap gap max 674-847 us, >5ms 0, >20ms 0" -- smooth reading throughout.
//     But "wire rx 19.6-19.7 KB/s x1024" = ~20.1 MB/s, ~2,450 frames/s, against
//     the FPGA's fixed 4105 words x 2.5 kHz = 20.5 MB/s.  The High-Speed link is
//     ~2 % SHORT of the stream, continuously.  A steady deficit fills every
//     buffer and spills it on a steady cadence: 569 one-frame counter skips
//     (2.2 % of frames -- the engine missing ticks with the CDC full) and the
//     16-word leg FIFOs overflowing every ~6 frames.  No buffer depth fixes a
//     steady deficit; it only sets the spill period.
//   * LEG7 is a KNOWN-BROKEN tail (Jeremy: "leg 7 is busted -- ignore it").  It
//     was enabled and never anchored (ph=1023 + undf on all 25,924 frames).  The
//     bench keeps it dead only because that is the recorded condition, and uses
//     the dead-vs-alive comparison as a CONTROL: does a dead enabled leg make the
//     healthy ones worse?  (It does not -- DLC-02.)
//
// Two windows under the SAME link model -- the FT600 drained at the recorded
// host ceiling (set_txe_drain_limit 152 = ~20.1 MB/s) against the engine's
// 155.5 (20.5 MB/s) fill, a smooth host, no hiccups:
//   D  LEG7 enabled but CLOCKLESS from bring-up (CTRL RO_RSTn=1, MCLK_EN=0 ->
//      no RO1_CLK -> no words -> never anchors), as recorded.  Healthy = 5, 6, 8.
//   A  LEG7's clock turned on so it anchors and streams; the same three legs
//      measured again.  D vs A isolates whether a dead enabled leg AGGRAVATES the
//      others (the escape/tick logic) or whether this is bandwidth alone.
//
// Contract (what must hold) and measurement (what a fix must move):
//   DLC-01 with LEG7 dead the healthy legs must not drop at all. Failed on the
//          original RTL -- the recording's defect. Reports the rate and split.
//   DLC-02 a dead enabled leg must not aggravate the healthy legs: drops with
//          LEG7 dead <= 1.5x drops with LEG7 alive (+3 for small samples).
//   DLC-03 the dead leg is flagged (undf) and reports 1023 on EVERY frame -- an
//          enabled leg that never anchors is never presented as data.
//   DLC-04 frame counter never repeats; every alignment change is flagged (a
//          change flagged by the preceding 1023+undf frame counts as signalled:
//          a re-anchor on a frame boundary has no stuffed slot).
//   DLC-05 every judged leg-frame right-or-flagged (undf/par excuse; ovf sticky).
//   report: healthy-leg drop rate (% of leg-frames) dead vs alive; re-anchor vs
//           pure-underrun split; multi-leg frames; counter skips.
// ---------------------------------------------------------------------------
`ifdef RUN_DEAD_LEG_CEILING

int dlc_prev_ph [0:3];   // per-leg last anchored phase (module scope: Icarus C-05)
int dlc_prev_1023 [0:3]; // 1 if that leg's PREVIOUS frame was un-anchored (1023) and flagged

// One window of frames under the link deficit.  Judges right-or-flagged on the
// legs in `healthy` (hx_judge_leg), counts their flagged leg-frames ("drops")
// split into re-anchors (1023 or a changed phase) and pure underruns (flagged,
// phase kept), frames with >= 2 healthy legs flagged, counter skips/dups and
// unflagged phase changes.  If dead_ch >= 0 that leg must be flagged AND 1023 on
// every frame (dead_ok / dead_bad).
task automatic dlc_window(input int nframes, input logic [3:0] healthy, input int dead_ch,
                          inout int frame_no, inout int n_pass, inout int n_fail, inout int last_cnt,
                          output int legframes, output int drops, output int reanch, output int pure_undf,
                          output int multi, output int skips, output int dups, output int unfl,
                          output int dead_ok, output int dead_bad);
    logic [15:0] hdr;
    int i, ch, v, cnt_now, delta, nflag, ph;
    legframes = 0; drops = 0; reanch = 0; pure_undf = 0; multi = 0; skips = 0; dups = 0; unfl = 0;
    dead_ok = 0; dead_bad = 0;
    for (i = 0; i < nframes; i++) begin
        hx_grab_frame(hdr); frame_no++;
        cnt_now = tb_top.u_ft600q.v3_count_lo;
        delta   = (cnt_now - last_cnt) & 16'hFFFF;
        last_cnt = cnt_now;
        if (delta == 0) dups++;
        else if (delta > 1) skips += delta - 1;
        nflag = 0;
        for (ch = 0; ch < 4; ch++) begin
            ph = hx_phase[ch] & 16'h03FF;
            if (ch == dead_ch) begin
                if (hx_phase[ch][12] && ph == 1023) dead_ok++; else dead_bad++;
                continue;
            end
            if (!healthy[ch]) continue;
            legframes++;
            hx_judge_leg(frame_no, ch, v);            // 0 clean, 1 flagged, 2 WRONG AND UNFLAGGED
            if (v == 2) n_fail++; else n_pass++;
            if (v != 0) begin
                drops++; nflag++;
                if (ph == 1023 || ph != dlc_prev_ph[ch]) reanch++; else pure_undf++;
            end
            if (ph < 64 && ph != dlc_prev_ph[ch]) begin
                if (!hx_phase[ch][12] && !dlc_prev_1023[ch]) unfl++;
                dlc_prev_ph[ch] = ph;
            end
            dlc_prev_1023[ch] = (ph == 1023 && hx_phase[ch][12]) ? 1 : 0;
        end
        if (nflag >= 2) multi++;
    end
endtask

task automatic run_DEAD_LEG_CEILING();
    localparam logic [3:0] MASK           = 4'hF;   // all four enabled, as recorded
    localparam int         DEAD_CH        = 2;      // LEG7: enabled, TELEM_EN on, ASIC clock never runs
    localparam logic [3:0] HEALTHY        = 4'hB;   // legs 5, 6, 8
    localparam int         DRAIN_X1000    = 152;    // the host's High-Speed ceiling as recorded, ~20.1 MB/s
                                                    //   (words per 66 MHz cycle x1000; the engine fills at 155.5
                                                    //   = 20.5 MB/s) -> a steady ~2 % deficit, smooth host, no hiccups
    localparam int         DEFICIT_LEADIN = 25;     // unjudged frames for the FT600's 2048-word credit to run out
                                                    //   (2048 / 3.5e-3 words/cycle ~ 9 ms) and the spill cadence to settle
    // Keep the full FT600 credit and lead-in, but avoid the old 60-frame
    // statistical soak in CI. DLC-06 requires actual deficit-induced omissions
    // in BOTH windows, so shortening this cannot silently remove the stress.
    // Define DEAD_LEG_EXTENDED for the original 60-frame measurement windows.
`ifdef DEAD_LEG_EXTENDED
    localparam int         WINDOW_FRAMES  = 60;
`else
    localparam int         WINDOW_FRAMES  = 24;
`endif
    localparam int         RECOVER_FRAMES = 10;
    logic [15:0] m, f, a, d, hdr;
    logic [47:0] tx, rx;
    int  n_pass = 0, n_fail = 0, n_flush, frame_no = 0, i, ch, all_clean, last_cnt;
    int  lf_d, dr_d, re_d, pu_d, mu_d, sk_d, du_d, un_d, dok_d, dbad_d;   // window D: LEG7 dead
    int  lf_a, dr_a, re_a, pu_a, mu_a, sk_a, du_a, un_a, dok_a, dbad_a;   // window A: LEG7 alive
    int  bp_ev, bp_cy, bp_wd;
    real rate_d, rate_a;

    $display("");
    $display("[SA-DEADLEG] the 2026-09-18 13:01 drops: four legs enabled, LEG7 enabled but clockless, the link ~2%% short of the stream (drain %0d vs fill 155.5 words/kcycle), smooth host", DRAIN_X1000);

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[1].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[2].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask  = HEALTHY;                     // hx_judge_frame judges the healthy legs; LEG7 is judged separately
    hx_excuse_mask = 3'b101;                      // undf or par excuse wrong data; sticky ovf does not
    for (ch = 0; ch < 4; ch++) begin dlc_prev_ph[ch] = -1; dlc_prev_1023[ch] = 0; end

    // ---- bring-up as the tool does it, LEG7 without its ASIC clock ---------
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, {12'h0, MASK});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch++) begin
        if (ch == DEAD_CH) tx = 48'h01_01_00_00_00_00;   // CTRL: RO_RSTn=1, MCLK_EN=0 -> no RO1_CLK: the dead leg
        else               tx = 48'h01_11_00_00_00_00;   // CTRL: RO_RSTn + MCLK_EN
        spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal (LEG7 too, as recorded)
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, {12'h0, MASK});   // ACQ_ALL_RUN
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.flush_tx_capture(n_flush);

    // ---- settle: the healthy legs all-clean, LEG7 un-anchored ---------------
    hx_grab_frame(hdr);
    all_clean = 0;
    for (i = 0; i < RECOVER_FRAMES && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if (!all_clean) begin n_fail++; $display("[SA-DEADLEG] FAIL start-up: healthy legs never all-clean within %0d frames", RECOVER_FRAMES); end
    $display("[SA-DEADLEG] start-up: healthy legs clean after %0d frame(s); LEG7 phase word %04h (%s)",
             i, hx_phase[DEAD_CH], ((hx_phase[DEAD_CH] & 16'h03FF) == 1023 && hx_phase[DEAD_CH][12]) ? "un-anchored + undf, as recorded" : "UNEXPECTED");

    // ---- window D: LEG7 dead, the link deficit on --------------------------------
    tb_top.u_ft600q.set_txe_drain_limit(DRAIN_X1000, 2048);
    for (i = 0; i < DEFICIT_LEADIN; i++) begin hx_grab_frame(hdr); frame_no++; end   // credit runs out, spill cadence settles
    for (ch = 0; ch < 4; ch++) if (HEALTHY[ch]) dlc_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
    last_cnt = tb_top.u_ft600q.v3_count_lo;
    dlc_window(WINDOW_FRAMES, HEALTHY, DEAD_CH, frame_no, n_pass, n_fail, last_cnt,
               lf_d, dr_d, re_d, pu_d, mu_d, sk_d, du_d, un_d, dok_d, dbad_d);
    tb_top.u_ft600q.set_txe_drain_limit(0, 2048);
    tb_top.u_ft600q.get_bp_stats(bp_ev, bp_cy, bp_wd);
    rate_d = (lf_d > 0) ? 100.0 * dr_d / lf_d : 0.0;
    $display("[SA-DEADLEG] window D (LEG7 DEAD): %0d forced-stall event(s) (the rate limiter is not counted), %0d us; healthy leg-frames %0d, DROPPED %0d (%.1f%%): %0d re-anchor, %0d pure underrun; %0d frame(s) with >=2 legs; counter skipped %0d, repeated %0d; LEG7 flagged+1023 on %0d/%0d frames",
             bp_ev, bp_cy / 66, lf_d, dr_d, rate_d, re_d, pu_d, mu_d, sk_d, du_d, dok_d, WINDOW_FRAMES);

    // ---- settle (link unlimited), then bring LEG7 alive -----------------------------
    all_clean = 0;
    for (i = 0; i < RECOVER_FRAMES && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    // CTRL 0x11 (RO_RSTn + MCLK_EN) to LEG7 while its RUN is set.  spi_cfg_xact
    // is the read-mode form (rw=0, C-07) and did not take effect on a running
    // leg; use the write-mode form the pause/dies benches use mid-stream
    // (WDATA, then a control word with rw=1 go=1 n=2), which does.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0030, {8'h01, 8'h11});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0031, spi_cfg_ctrl_word(2'd2, 1'b1, 1'b1, 3'd2));
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #100_000;                                              // the transaction completes; LEG7's ASIC clock starts
    // A leg whose ASIC clock was OFF when RUN was set starts on its own once the
    // clock appears (no RUN re-arm needed -- measured).  Two unjudged frames let
    // the first words arrive; the settle below then judges LEG7 with the rest.
    hx_grab_frame(hdr); frame_no++;
    hx_grab_frame(hdr); frame_no++;
    hx_armed_mask = 4'hF;
    all_clean = 0;
    for (i = 0; i < RECOVER_FRAMES && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if (!all_clean) begin n_fail++; $display("[SA-DEADLEG] FAIL LEG7 never anchored within %0d frames after its clock was enabled", RECOVER_FRAMES); end
    $display("[SA-DEADLEG] LEG7 clock on: all four legs clean after %0d frame(s), LEG7 phase %0d", i, hx_phase[DEAD_CH] & 16'h03FF);

    // ---- window A: all four alive, the SAME three legs measured under the same deficit ----
    tb_top.u_ft600q.set_txe_drain_limit(DRAIN_X1000, 2048);
    for (i = 0; i < DEFICIT_LEADIN; i++) begin hx_grab_frame(hdr); frame_no++; end
    for (ch = 0; ch < 4; ch++) if (HEALTHY[ch]) dlc_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
    last_cnt = tb_top.u_ft600q.v3_count_lo;
    dlc_window(WINDOW_FRAMES, HEALTHY, -1, frame_no, n_pass, n_fail, last_cnt,
               lf_a, dr_a, re_a, pu_a, mu_a, sk_a, du_a, un_a, dok_a, dbad_a);
    tb_top.u_ft600q.set_txe_drain_limit(0, 2048);
    tb_top.u_ft600q.get_bp_stats(bp_ev, bp_cy, bp_wd);
    rate_a = (lf_a > 0) ? 100.0 * dr_a / lf_a : 0.0;
    $display("[SA-DEADLEG] window A (LEG7 ALIVE): %0d forced-stall event(s) total (the rate limiter is not counted), %0d us; healthy leg-frames %0d, DROPPED %0d (%.1f%%): %0d re-anchor, %0d pure underrun; %0d frame(s) with >=2 legs; counter skipped %0d, repeated %0d",
             bp_ev, bp_cy / 66, lf_a, dr_a, rate_a, re_a, pu_a, mu_a, sk_a, du_a);

    // ---- verdicts -----------------------------------------------------------------
    if (dr_d == 0) begin
        n_pass++; $display("[DLC-01] PASS with LEG7 dead the healthy legs did not drop in %0d leg-frames", lf_d);
    end else begin
        n_fail++; $display("[DLC-01] FAIL with LEG7 dead the healthy legs DROPPED %0d of %0d leg-frames (%.1f%%): %0d re-anchor + %0d pure underrun, %0d frame(s) hit >=2 legs together, frame counter skipped %0d -- the recording's drops (it lost ~19%% of healthy leg-frames and 2.2%% of frames to a ~2%% link deficit)",
                           dr_d, lf_d, rate_d, re_d, pu_d, mu_d, sk_d);
    end
    if (dr_d * 2 <= dr_a * 3 + 6) begin
        n_pass++; $display("[DLC-02] PASS a dead enabled leg does not aggravate the others: %.1f%% dropped with LEG7 dead vs %.1f%% alive -- the drops are the link deficit, not the dead leg", rate_d, rate_a);
    end else begin
        n_fail++; $display("[DLC-02] FAIL a dead enabled leg AGGRAVATES the healthy legs: %.1f%% dropped with LEG7 dead vs %.1f%% alive (%0d vs %0d of %0d leg-frames) -- the escape/tick logic, not just bandwidth", rate_d, rate_a, dr_d, dr_a, lf_d);
    end
    if (dbad_d == 0 && dok_d == WINDOW_FRAMES) begin
        n_pass++; $display("[DLC-03] PASS the dead leg is flagged (undf) and un-anchored (1023) on all %0d frames -- never presented as data", dok_d);
    end else begin
        n_fail++; $display("[DLC-03] FAIL the dead leg was presented without undf/1023 on %0d of %0d frames", dbad_d, WINDOW_FRAMES);
    end
    if (du_d + du_a == 0 && un_d + un_a == 0) begin
        n_pass++; $display("[DLC-04] PASS frame counter never repeated, every alignment change flagged");
    end else begin
        n_fail++; $display("[DLC-04] FAIL %0d repeated frame counter(s), %0d unflagged alignment change(s)", du_d + du_a, un_d + un_a);
    end
    // DLC-05 is the per-leg right-or-flagged verdicts folded into n_pass/n_fail above.
    $display("[DLC-05] %s right-or-flagged on every judged leg-frame", (n_fail == 0) ? "PASS" : "see FAIL lines above:");
    if (sk_d > 0 && sk_a > 0) begin
        n_pass++; $display("[DLC-06] PASS bandwidth deficit exercised in both windows: %0d dead / %0d alive counter skips", sk_d, sk_a);
    end else begin
        n_fail++; $display("[DLC-06] FAIL insufficient deficit exposure: %0d dead / %0d alive counter skips", sk_d, sk_a);
    end

    // ---- recover: link unlimited again, all four strict-correct -----------------------
    all_clean = 0;
    for (i = 0; i < RECOVER_FRAMES && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if (all_clean) begin n_pass++; $display("[SA-DEADLEG] link unlimited again: strict-correct within %0d frame(s)", i); end
    else begin n_fail++; $display("[SA-DEADLEG] FAIL no all-correct frame within %0d frames after the deficit", RECOVER_FRAMES); end

    $display("");
    $display("[SA-DEADLEG] MEASURED: healthy-leg drop rate %.1f%% with LEG7 dead (%0d re-anchor / %0d pure underrun / %0d multi-leg frames, %0d counter skips) vs %.1f%% with LEG7 alive; recording: ~19%%, 60/40 split, 74%% multi-leg, 2.2%% frames skipped",
             rate_d, re_d, pu_d, mu_d, sk_d, rate_a);
    if (n_fail == 0) $display("[SA-DEADLEG] PASS — %0d checks", n_pass);
    else             $display("[SA-DEADLEG] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif // RUN_DEAD_LEG_CEILING
