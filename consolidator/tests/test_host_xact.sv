// ============================================================================
// test_host_xact.sv — host SPI_CFG traffic DURING streaming must not silently
//                     rotate a leg (the host contract: right, or flagged)
//
// Regression for the 2026-09-09 16:19 ACED recording on the 72606e4-based
// consolidator (6ac218f): both enabled legs arrived bit-ROTATED with phase 0
// and a valid CRC — data that is wrong and says nothing about it.  frame_cnt
// advanced by exactly 1 per frame, so the engine never stalled (not USB
// back-pressure).  Two mechanisms, both reproduced here:
//   * legs armed one after another (the bring-up tool programs tails serially)
//     enter the engine's frame at whatever slot the tick counter has reached,
//     and stay rotated for good;
//   * the bring-up log shows ~19 SPI_CFG READBACK transactions per second while
//     streaming (GO on 0x0031, then RD01/RD23 reads: PINGs, config walks, health
//     polls).  A readback asserts cfg_hold on that leg's receiver; the tail
//     restarts at its next sweep boundary and the leg re-enters mid-frame.
//
// The host contract this bench enforces (all at the FT600Q USB boundary,
// MODE_UNIQUE data so every word names its {frame, sensor, leg}):
//   (1) NO SILENT CORRUPTION: for every frame and leg, the leg's data decodes
//       strictly (plane_offset 0, sample_offset == its phase word) OR its phase
//       word carries a fault flag (bits 14..12: par/ovfl/undf).  Flagged data
//       may be anything — the host discards it.  Unflagged wrong data is the
//       defect.  The RUN_UNIQUE search, which accepts any offset as long as the
//       stream is contiguous, is what let the rotation through the suite unseen.
//   (2) LIVENESS: after start-up, and after each host transaction, at least one
//       of the last RECOVER_FRAMES frames is clean on all four legs — the
//       design may flag while it re-aligns, but it must re-align.
//   (3) NO PUNCTURE: no ctrl response lands inside a telemetry frame (ISSUE 3).
//
// Measured 2026-09-09: con_phase (907dcfd, the previously flashed lineage)
// PASSES — every unreconstructible leg-frame it emitted was flagged;
// 72606e4 + anchoring (6ac218f) FAILS on (1) — legs 1..3 rotated, phase 0, in
// every frame; plain 72606e4 FAILS on (2) — sticky flags, never a clean frame.
//
// Bench mechanics worth knowing: a sequential bench cannot drain telemetry
// while it waits for a response (the DUT holds each to an inter-frame gap), so
// frames pile up during a transaction.  We judge the first two frames of that
// pile (where a disturbance would show) and then the newest RECOVER_FRAMES
// (keep_newest_telemetry_frames), which is what a live host would see.
// ============================================================================

// Compiled only for its own target: the task names tb_top.tail_ch[0..3].asic_model,
// which does not elaborate under RUN_SINGLE_LEG (that build has only
// tail_ch[0].active_tail), and nothing else needs the 16x64x16 search below.
`ifdef RUN_HOST_XACT

logic [15:0] hx_data  [0:4095];  // frame under test (module scope: functions read it, and
logic [15:0] hx_phase [0:3];     // C-05: Icarus takes no unpacked-array task ports)
int          hx_armed_mask = 4'hF;  // legs judged by hx_judge_frame (bit per leg); an
                                    // unarmed leg carries the 1023 sentinel and is skipped
// Which phase-word flag bits EXCUSE wrong data: {par[14], ovfl[13], undf[12]}.
// Default = all three (the original contract).  The USB-jitter scenario clears
// bit 13: the overflow flag is STICKY for the session, so after the first host
// gap it is set on every frame and excuses nothing a host could act on — the
// bring-up tool conceals a leg only on undf (bit 12), exactly what the
// 2026-09-10 11:07 recordings show (flagged == every frame, ovf=0x9 sticky).
int          hx_excuse_mask = 3'b111;

// Strict per-leg reconstruction of hx_data.  Returns the matched
// (frame_nibble*64 + sample_offset)*16 + plane_offset, or -1.
function automatic integer hx_leg_match(input integer uch);
    integer ug, ulane, ubit, ucand, ustart, uplane, ubad, usample, uframe, uraw, ucomplete, found;
    logic [15:0] got_word;
    begin
        found = -1;
        for (ucand = 0; ucand < 16; ucand++) begin
            for (ustart = 0; ustart < 64; ustart++) begin
                for (uplane = 0; uplane < 16; uplane++) begin
                    uraw      = (uplane == 0) ? 0 : 16 - uplane;
                    ucomplete = (1024 - uraw) / 16;
                    usample   = (ustart + ((uplane + uraw) / 16)) % 64;
                    uframe    = (ucand + ((ustart + ((uplane + uraw) / 16)) / 64)) % 16;
                    ubad = 0;
                    for (ubit = 0; ubit < 16; ubit++)
                        got_word[15-ubit] = hx_data[4*(uraw+ubit)+uch][0];
                    if (got_word !== unique_expected(uframe[3:0],
                        (usample % 16) + (usample / 16)*256, uch[1:0])) begin
                        ubad = 1;
                    end else begin
                        for (ug = 0; ug < ucomplete && ubad == 0; ug++) begin
                            usample = (ustart + ((uplane + uraw) / 16) + ug) % 64;
                            uframe  = (ucand + ((ustart + ((uplane + uraw) / 16) + ug) / 64)) % 16;
                            for (ulane = 0; ulane < 16; ulane++) begin
                                for (ubit = 0; ubit < 16; ubit++)
                                    got_word[15-ubit] = hx_data[4*(uraw+ug*16+ubit)+uch][ulane];
                                if (got_word !== unique_expected(uframe[3:0],
                                    ulane*16 + (usample % 16) + (usample / 16)*256,
                                    uch[1:0])) ubad = ubad + 1;
                            end
                        end
                    end
                    if (ubad == 0 && found < 0) found = (ucand * 64 + ustart) * 16 + uplane;
                end
            end
        end
        hx_leg_match = found;
    end
endfunction

// Capture one V3 frame into hx_data + hx_phase (module scope, see C-05 note).
task automatic hx_grab_frame(output logic [15:0] hdr_o);
    int i;
`ifdef ICARUS
    // TYPED reader: this bench issues USB commands while streaming, so the raw
    // capture interleaves 4-word responses; the untyped reader desyncs on them.
    tb_top.u_ft600q.wait_telemetry_frame_v3_typed(hdr_o);
    for (i = 0; i < 4096; i++) hx_data[i]  = tb_top.u_ft600q.v3_data[i];
    for (i = 0; i < 4;    i++) hx_phase[i] = tb_top.u_ft600q.v3_phase[i];
`else
    begin
        logic [15:0] cw [0:1];
        tb_top.u_ft600q.wait_telemetry_frame_v3(hdr_o, hx_data, hx_phase, cw);
    end
`endif
endtask

// Judge one leg of the frame in hx_data against the host contract.
//   verdict: 0 = clean: strict-correct (flags, e.g. the sticky ovfl bit, may ride along)
//            1 = flagged AND wrong (phase word bits 14..12) — excused, the host discards it
//            2 = SILENT CORRUPTION: unflagged and not strict-correct
task automatic hx_judge_leg(input int frame_no, input int uch, output int verdict);
    int matched, plane_off, samp_off, ph, flagged, w, n_ff, n_zero;
    logic [15:0] phw;
    begin
        phw     = hx_phase[uch];
        flagged = (|(phw[14:12] & hx_excuse_mask[2:0])) ? 1 : 0;
        ph      = phw & 16'h03FF;
        matched = hx_leg_match(uch);
        plane_off = (matched < 0) ? -1 : matched % 16;
        samp_off  = (matched < 0) ? -1 : (matched / 16) % 64;
        // Strict-correct data is clean whether or not a flag rides along: the
        // overflow bit is STICKY for the session on both lineages (leg_quad_fifo
        // clears it only on channel reset), so one start-up overflow would
        // otherwise make "clean" unattainable while the data is perfectly good.
        // Flags only EXCUSE wrong data.
        if (matched >= 0 && plane_off == 0 && samp_off == ph) begin
            verdict = 0;
        end else if (flagged) begin
            verdict = 1;
            $display("[SA-HOSTXACT] frame[%0d] leg%0d: flagged (phase word 0x%04h) — data excused (plane_offset=%0d sample_offset=%0d)",
                     frame_no, uch, phw, plane_off, samp_off);
        end else begin
            verdict = 2;
            n_ff = 0; n_zero = 0;
            for (w = 0; w < 1024; w++) begin
                if (hx_data[4*w+uch] === 16'hFFFF) n_ff++;
                if (hx_data[4*w+uch] === 16'h0000) n_zero++;
            end
            if (matched >= 0)
                $display("[SA-HOSTXACT] FAIL frame[%0d] leg%0d: SILENT ROTATION plane_offset=%0d sample_offset=%0d, phase word says %0d (0x%04h) — wrong data, no flag",
                         frame_no, uch, plane_off, samp_off, ph, phw);
            else
                $display("[SA-HOSTXACT] FAIL frame[%0d] leg%0d: SILENT CORRUPTION not reconstructible, unflagged (phase word 0x%04h; %0d words 0xFFFF, %0d words 0x0000 of 1024)",
                         frame_no, uch, phw, n_ff, n_zero);
        end
    end
endtask

// Judge all four legs of the current frame.  all_clean_o = every leg verdict 0.
task automatic hx_judge_frame(input int frame_no, inout int n_pass, inout int n_fail, output int all_clean_o);
    int uch, v;
    begin
        all_clean_o = 1;
        for (uch = 0; uch < 4; uch++) begin
            if (!hx_armed_mask[uch]) continue;
            hx_judge_leg(frame_no, uch, v);
            if (v == 2) n_fail++; else n_pass++;
            if (v != 0) all_clean_o = 0;
        end
    end
endtask

task automatic run_SA_HOST_XACT();
    localparam int N_XACT         = 6;   // host readback transactions while streaming
    localparam int RECOVER_FRAMES = 8;   // frames allowed after each transaction for an all-correct frame to reappear (3.2 ms)
    localparam int SETTLE_FRAMES  = 8;   // start-up window in which an all-clean frame must appear
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, dropped;
    int          frame_no = 0, k, ch, xact_leg, i, all_clean, window_clean, pending;

    $display("");
    $display("[SA-HOSTXACT] host SPI_CFG readback traffic during streaming — right or flagged, never silently rotated");

    // ── ASIC models: unique {frame,sensor,leg} identity on all four legs ──────
    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[1].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[2].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;

    // ── Bring-up, as ionm_test / the bring-up tool do it (tails programmed serially) ─
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h000F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch++) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // CTRL: RO_RSTn + MCLK_EN
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    // The config SPI stays at 800 kHz (the tail cannot take faster), as the tool
    // leaves it: its in-stream transactions hold a leg for tens of us.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h000F);   // ACQ_ALL_RUN
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);   // one TLM line per frame boundary

    // ── Phase A: start-up.  Every frame: right or flagged.  Within SETTLE_FRAMES
    //    at least one frame must be all-clean (liveness at start-up).
    hx_grab_frame(hdr);                          // arm / frame-sync warm-up frame
    $display("[SA-HOSTXACT] warm-up frame hdr=0x%04X discarded", hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        $display("[SA-HOSTXACT] frame[%0d] hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                 frame_no, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin
        n_pass++;
        $display("[SA-HOSTXACT] start-up: all-clean frame within %0d frame(s)", i + 1);
    end else begin
        n_fail++;
        $display("[SA-HOSTXACT] FAIL start-up: no all-clean frame within %0d frames (legs never strict-aligned)", SETTLE_FRAMES);
    end

    // ── Phase B: host readback transactions while streaming ───────────────────
    for (k = 0; k < N_XACT; k++) begin
        xact_leg = k % 4;
`ifdef ICARUS
        tx = 48'hAA_00_00_00_00_00; spi_cfg_xact(xact_leg[1:0], tx, 3'd3, rx);   // PING (readback)
        $display("[SA-HOSTXACT] xact %0d: readback PING to leg%0d (answered 0x%02h; a streaming tail need not answer 0x55) t=%0t",
                 k, xact_leg, rx[31:24], $time);
`else
        tx = '{8'hAA, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(xact_leg[1:0], tx, 3'd3, rx);
        $display("[SA-HOSTXACT] xact %0d: readback PING to leg%0d t=%0t", k, xact_leg, $time);
`endif
        // Frames that arrived during the transaction: judge the first two (where a
        // disturbance shows), then skip to the newest RECOVER_FRAMES.
        pending = tb_top.u_ft600q.telem_frames_pending();
        for (i = 0; i < 2 && i < pending - 2; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-HOSTXACT] frame[%0d] (during xact %0d) hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        if (dropped > 0)
            $display("[SA-HOSTXACT]   %0d frame(s) that arrived during xact %0d skipped (a live host would have read them)", dropped, k);
        // Recovery: keep reading (every frame still judged right-or-flagged) until
        // a frame is strict-correct on all four legs, or RECOVER_FRAMES have gone
        // by.  con_phase flags a 3-frame underrun/re-sync stretch every ~7 frames
        // (its known periodic dip), so the window must span more than one cycle.
        window_clean = 0;
        for (i = 0; i < RECOVER_FRAMES && !window_clean; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-HOSTXACT] frame[%0d] (after xact %0d, +%0d) hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, i, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (all_clean) window_clean = 1;
        end
        if (window_clean) begin
            n_pass++;
            $display("[SA-HOSTXACT] after xact %0d: all legs strict-correct again within %0d frame(s)", k, i);
        end else begin
            n_fail++;
            $display("[SA-HOSTXACT] FAIL after xact %0d: no all-correct frame within %0d frames (no recovery)", k, RECOVER_FRAMES);
        end
    end

    // ── (3) No puncture (ISSUE 3) ─────────────────────────────────────────────
    if (tb_top.u_ft600q.puncture_count != 0) begin
        $display("[SA-HOSTXACT] FAIL %0d telemetry frame(s) PUNCTURED by a ctrl response (first at frame position %0d)",
                 tb_top.u_ft600q.puncture_count, tb_top.u_ft600q.puncture_first_pos);
        n_fail += tb_top.u_ft600q.puncture_count;
    end else begin
        $display("[SA-HOSTXACT] no-puncture: %0d host readbacks, 0 responses inside a telemetry frame", N_XACT);
        n_pass++;
    end

    // ── Verdict ───────────────────────────────────────────────────────────────
    $display("");
    if (n_fail == 0)
        $display("[SA-HOSTXACT] PASS — %0d checks: every leg-frame right or flagged, re-aligned after each of %0d host readbacks, no puncture",
                 n_pass, N_XACT);
    else
        $display("[SA-HOSTXACT] FAIL — %0d check(s) failed (%0d passed): unflagged wrong data, or no re-alignment after host traffic",
                 n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask

`ifdef RUN_LEG_PAUSE
// ---------------------------------------------------------------------------
// LEG-PAUSE — one streaming leg stops delivering for ~1 ms while the other
// keeps going, then resumes.  2026-09-10 16:28 recording (legs 5+8, raw): LEG5
// went silent, the engine waited two frame ticks, the dead-leg escape released
// and un-aligned LEG5 (LEG8 overflowed and re-anchored meanwhile); LEG5's
// receiver had never lost its anchor, so when its words resumed the engine
// re-admitted it at a 16-slot boundary with the FIFO head at an arbitrary word:
// 151 and 313 frames of UNFLAGGED data at plane offsets 12 and 3, until an
// unrelated re-anchor.  Contract: every leg-frame strictly correct (plane
// offset 0, sample offset == phase word) OR flagged; both legs strict-correct
// again within RECOVER_FRAMES of the resume.
// ---------------------------------------------------------------------------
task automatic run_LEG_PAUSE();
    localparam int N_PAUSE        = 3;
    localparam int RECOVER_FRAMES = 10;
    localparam int SETTLE_FRAMES  = 8;
    localparam int JUDGE_AFTER    = 12;      // frames judged after each resume
    int pause_ns [0:2];
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, ch, i, k, all_clean, window_clean, frame_no = 0, dropped, pending;

    pause_ns[0] = 1_200_000; pause_ns[1] = 900_000; pause_ns[2] = 1_500_000;
    $display("");
    $display("[LEG-PAUSE] one leg pauses ~1 ms while the other streams: after it resumes every frame must be right-or-flagged, then strict-correct");

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);

    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) n_pass++;
    else begin n_fail++; $display("[LEG-PAUSE] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    for (k = 0; k < N_PAUSE; k++) begin
        // steady state
        for (i = 0; i < 3; i++) begin hx_grab_frame(hdr); frame_no++; hx_judge_frame(frame_no, n_pass, n_fail, all_clean); end
        $display("[LEG-PAUSE] pause %0d: LEG5's ASIC clock off for %0d us (LEG8 keeps streaming)  t=%0t", k, pause_ns[k] / 1000, $time);
        cfg_write2(2'd0, 8'h01, 8'h01);              // tail 0 CTRL: RO_RSTn=1, MCLK_EN=0 -> no RO1_CLK -> no words
        #(pause_ns[k]);
        cfg_write2(2'd0, 8'h01, 8'h11);              // clock back
        $display("[LEG-PAUSE] pause %0d: clock back  t=%0t", k, $time);
        // Judge the frames from the pause onward: the engine stalled and then
        // released, so several piled up; judge the two oldest, skip the pile,
        // then judge JUDGE_AFTER live frames — every one right-or-flagged, and an
        // all-clean one within RECOVER_FRAMES.
        pending = tb_top.u_ft600q.telem_frames_pending();
        for (i = 0; i < 2 && i < pending - 2; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        window_clean = 0;
        for (i = 0; i < JUDGE_AFTER; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[LEG-PAUSE] frame[%0d] (after pause %0d, +%0d) cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, i, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (all_clean && !window_clean) begin
                window_clean = 1;
                if (i < RECOVER_FRAMES) $display("[LEG-PAUSE] pause %0d: both legs strict-correct again after %0d frame(s)", k, i + 1);
            end
        end
        if (window_clean) n_pass++;
        else begin n_fail++; $display("[LEG-PAUSE] FAIL pause %0d: no all-correct frame within %0d frames of the resume", k, JUDGE_AFTER); end
    end

    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[LEG-PAUSE] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end
    $display("");
    if (n_fail == 0) $display("[LEG-PAUSE] PASS — %0d checks: a paused leg comes back right-or-flagged and re-aligns at plane offset 0", n_pass);
    else             $display("[LEG-PAUSE] FAIL — %0d check(s) failed (%0d passed): a paused leg was re-admitted at an arbitrary word (unflagged plane offset) — the 16:28 recording", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_LEG_PAUSE

`ifdef RUN_STREAM_DIES
`define HX_NEED_CFG_WRITE2
`endif
`ifdef RUN_LEG_PAUSE
`define HX_NEED_CFG_WRITE2
`endif
`ifdef RUN_IMP_CYCLE
`define HX_NEED_CFG_WRITE2
`endif
`ifdef HX_NEED_CFG_WRITE2
// A WRITE-mode cfg transaction (rw=1): the only kind a streaming leg accepts
// (reads and passthroughs to a running leg are rejected — fault bit 4).
// 2 bytes: opcode + data.  ~25 us at the divided clock; wait 100 us.
task automatic cfg_write2(input logic [1:0] c, input logic [7:0] b0, input logic [7:0] b1);
    logic [15:0] mm, ff, aa, dd;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0030, {b0, b1});
    tb_top.u_ft600q.wait_response_frame_typed(mm, ff, aa, dd);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0031, spi_cfg_ctrl_word(c, 1'b1, 1'b1, 3'd2));
    tb_top.u_ft600q.wait_response_frame_typed(mm, ff, aa, dd);
    #100_000;
endtask
`endif

`ifdef RUN_STREAM_DIES
// ---------------------------------------------------------------------------
// STREAM-DIES — the tails go silent while RUN is set; the host must still be
// answered.  2026-09-10 15:04:37 (sequential impedance sweep, bring-up log):
// after a RUN whose capture delivered 0 frames, the CH_CTRL read got no
// response, nor did anything after it, and 15 s later FT_WritePipe timed out —
// the consolidator had stopped answering AND stopped reading commands; only a
// reconnect recovered it.  telem_engine_v3 was mid-frame with every anchored
// leg empty: its dead-leg escape needed `progress`, so the frame never ended,
// framer_busy stayed high and every response sat in ST_TX_WAIT.
//   0. RUN with the tails never streaming (TELEM_EN off): a register read is
//      answered, RUN=0 is answered.
//   1. Two legs streaming and aligned; both tails' MCLK is switched off by a
//      CTRL write (the ASIC clock stops, MISO goes idle): within 2 ms a register
//      read must be answered, RUN=0 must be answered; MCLK back on + RUN → the
//      stream is strict-correct again within SETTLE_FRAMES.
// ---------------------------------------------------------------------------

task automatic run_STREAM_DIES();
    localparam int SETTLE_FRAMES = 8;
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, ch, i, all_clean, window_clean, frame_no = 0, dropped, words;


    $display("");
    $display("[STREAM-DIES] tails silent while RUN is set: the host must still be answered, RUN=0 must take, the stream must come back");

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    // Tails clocked and out of reset, but NOT streaming (no TELEM_EN yet).
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end

    // ── Phase 0: RUN with tails that never stream ────────────────────────────
    $display("[STREAM-DIES] phase 0: RUN on legs 0,3 with the tails not streaming  t=%0t", $time);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);   // ACQ_ALL_RUN legs 0,3
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #1_000_000;                                                                        // 1 ms
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
    #2_000_000;
    words = tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr;
    if (words < 4) begin n_fail++; $display("[STREAM-DIES] FAIL phase 0: TOKEN_HI read not answered within 2 ms while RUN is set on silent tails (%0d ctrl words)", words); end
    else begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (a !== REG_TOKEN_HI || d !== 16'hCEFA) begin n_fail++; $display("[STREAM-DIES] FAIL phase 0: response {%04h %04h %04h %04h}", m, f, a, d); end
        else begin n_pass++; $display("[STREAM-DIES] phase 0: read answered while RUN is set on silent tails"); end
    end
    while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4) begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        $display("[STREAM-DIES]   phase 0 extra ctrl packet {%04h %04h %04h %04h}", m, f, a, d);
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0000);   // RUN off
    #1_000_000;
    if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4) begin n_fail++; $display("[STREAM-DIES] FAIL phase 0: RUN=0 not answered within 1 ms"); end
    else begin tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d); n_pass++; end
    $display("[STREAM-DIES] phase 0: %0d telemetry frame(s) were emitted with no tail streaming", tb_top.u_ft600q.telem_frames_pending());
    tb_top.u_ft600q.keep_newest_telemetry_frames(0, dropped);

    // ── Phase 1: streaming, then the tails go silent mid-run ─────────────────
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal
`else
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) n_pass++;
    else begin n_fail++; $display("[STREAM-DIES] FAIL phase 1 start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    $display("[STREAM-DIES] phase 1: MCLK off on both streaming tails (ASIC clock stops, MISO idle)  t=%0t", $time);
    cfg_write2(2'd0, 8'h01, 8'h01);   // CTRL: RO_RSTn=1, MCLK_EN=0
    cfg_write2(2'd3, 8'h01, 8'h01);
    #2_000_000;                       // 2 ms = 5 frame times of silence
    tb_top.u_ft600q.keep_newest_telemetry_frames(0, dropped);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
    #2_000_000;
    words = tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr;
    if (words < 4) begin
        n_fail++;
        $display("[STREAM-DIES] FAIL phase 1: TOKEN_HI read not answered within 2 ms after the tails went silent — framer_busy stuck on an unfinishable frame (the 15:04:37 sweep hang)");
    end else begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (a !== REG_TOKEN_HI || d !== 16'hCEFA) begin n_fail++; $display("[STREAM-DIES] FAIL phase 1: response {%04h %04h %04h %04h}", m, f, a, d); end
        else begin n_pass++; $display("[STREAM-DIES] phase 1: read answered with the tails silent and RUN set"); end
    end
    while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4) begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        $display("[STREAM-DIES]   phase 1 extra ctrl packet {%04h %04h %04h %04h}", m, f, a, d);
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0000);   // RUN off
    #1_000_000;
    if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4) begin n_fail++; $display("[STREAM-DIES] FAIL phase 1: RUN=0 not answered within 1 ms"); end
    else begin tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d); n_pass++; $display("[STREAM-DIES] phase 1: RUN=0 answered"); end
    $display("[STREAM-DIES] phase 1: %0d telemetry frame(s) emitted after the tails went silent", tb_top.u_ft600q.telem_frames_pending());
    tb_top.u_ft600q.keep_newest_telemetry_frames(0, dropped);

    // ── Phase 2: clocks back, RUN again — the stream must come back clean ────
    cfg_write2(2'd0, 8'h01, 8'h11);
    cfg_write2(2'd3, 8'h01, 8'h11);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin n_pass++; $display("[STREAM-DIES] phase 2: stream strict-correct again within %0d frame(s)", i + 1); end
    else begin n_fail++; $display("[STREAM-DIES] FAIL phase 2: no all-clean frame within %0d frames after the clocks returned", SETTLE_FRAMES); end

    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[STREAM-DIES] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    if (n_fail == 0) $display("[STREAM-DIES] PASS — %0d checks: silent tails never wedge the command path; the stream returns", n_pass);
    else             $display("[STREAM-DIES] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_STREAM_DIES

`ifdef RUN_FAULT_MID_CMD
// ---------------------------------------------------------------------------
// FAULT-MID-CMD — a fault frame must never cost the host a command.
// 2026-09-10 14:51:01 and 14:51:04 (bring-up log): the tool's passthrough
// request to a streaming leg was rejected by the consolidator (fault bit 4,
// cfg_spi_collision) and the very next register read got no response for
// 3 s.  cmd_decoder dispatched the fault frame with rx_ready still high, so
// the read's four words were popped from the command FIFO while the frame
// waited for the telemetry boundary — silently gone.  Two shapes, while two
// legs stream (so the fault frame really does wait up to a frame):
//   A. fault, then a command sent immediately (arrives during the fault frame);
//   B. a command delivered in two halves with the fault inside the gap
//      (the decoder is mid-command when the fault becomes pending).
// Contract: exactly one fault frame {55AA FFFF 0001 0000} AND the correct
// response (A: fault first; B: the half-received command completes first);
// TOKEN_HI untouched; the stream stays right-or-flagged.
// ---------------------------------------------------------------------------
task automatic run_FAULT_MID_CMD();
    localparam int N_ITER        = 2;
    localparam int SETTLE_FRAMES = 8;
    localparam int WAIT_FRAMES   = 6;      // both packets must be in by then
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, ch, i, k, shape, all_clean, window_clean, frame_no = 0, words, dropped;

    $display("");
    $display("[FAULT-MID-CMD] a fault frame dispatched while a command arrives / is half-received must not lose the command");

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);   // ACQ_ALL_RUN legs 0,3
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);

    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) n_pass++;
    else begin n_fail++; $display("[FAULT-MID-CMD] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    // Pre-check for shape B: a command delivered in two halves with NO fault
    // must simply be answered (pins the RXF-gap handling on its own).
    tb_top.u_ft600q.set_rxf_packet_gap(2, 20_000);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
    for (i = 0; i < WAIT_FRAMES && (tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    tb_top.u_ft600q.set_rxf_packet_gap(0, 12);
    words = tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr;
    if (words != 4) begin
        n_fail++;
        $display("[FAULT-MID-CMD] FAIL pre-check: split (2+2 words, no fault) TOKEN_HI read -> %0d ctrl word(s) within %0d frames (expected exactly the response)", words, WAIT_FRAMES);
        while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4) begin
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            $display("[FAULT-MID-CMD]   got {%04h %04h %04h %04h}", m, f, a, d);
        end
    end else begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (m !== 16'h55AA || a !== REG_TOKEN_HI || d !== 16'hCEFA) begin n_fail++; $display("[FAULT-MID-CMD] FAIL pre-check: split read answered {%04h %04h %04h %04h}", m, f, a, d); end
        else begin n_pass++; $display("[FAULT-MID-CMD] pre-check: a command split 2+2 by an RXF gap is answered normally"); end
    end
    // and a plain read right after it, to prove the decoder is still aligned
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
    for (i = 0; i < WAIT_FRAMES && (tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4) begin
        n_fail++; $display("[FAULT-MID-CMD] FAIL pre-check: the read AFTER the split command got no response — decoder left misaligned by the split (words consumed: rx_rd=%0d)", tb_top.u_ft600q.rx_rd_ptr);
        $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail); $display("STATUS: FAIL"); return;
    end else begin
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        if (d !== 16'hCEFA) begin n_fail++; $display("[FAULT-MID-CMD] FAIL pre-check: follow-up read {%04h %04h %04h %04h}", m, f, a, d); end
        else n_pass++;
    end
    tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);

    for (shape = 0; shape < 2; shape++) begin
        for (k = 0; k < N_ITER; k++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (shape == 0) begin
                // A: fault first, command right behind it — the command lands
                //    while the fault frame waits for the telemetry boundary.
                $display("[FAULT-MID-CMD] A%0d: FAULTN low, TOKEN_HI read 2 us later  t=%0t", k, $time);
                faultn_tb = 1'b0;
                #2_000;
                tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
            end else begin
                // B: command in two halves (RXF gap after 2 words), fault inside the gap.
                $display("[FAULT-MID-CMD] B%0d: TOKEN_HI read split 2+2 words, FAULTN low in the gap  t=%0t", k, $time);
                tb_top.u_ft600q.set_rxf_packet_gap(2, 20_000);        // ~300 us gap at 66 MHz
                tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
                #30_000;                                               // first half consumed by now
                if (tb_top.u_ft600q.rx_count() != 2) begin
                    n_fail++;
                    $display("[FAULT-MID-CMD] B%0d: SETUP FAIL — %0d word(s) still queued, expected 2 (gap not in effect)", k, tb_top.u_ft600q.rx_count());
                end else n_pass++;
                faultn_tb = 1'b0;
                #2_000;
            end
            // Both packets must appear within WAIT_FRAMES of stream; keep judging the stream.
            for (i = 0; i < WAIT_FRAMES && (tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 8; i++) begin
                hx_grab_frame(hdr); frame_no++;
                hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            end
            faultn_tb = 1'b1;
            if (shape == 1) tb_top.u_ft600q.set_rxf_packet_gap(0, 12);
            words = tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr;
            if (words < 8) begin
                n_fail++;
                $display("[FAULT-MID-CMD] FAIL %s%0d: %0d ctrl word(s) within %0d frames — expected the fault frame AND the response (8 words); the command was swallowed by the fault dispatch",
                         (shape == 0) ? "A" : "B", k, words, WAIT_FRAMES);
                while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4) begin
                    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
                    $display("[FAULT-MID-CMD]   got {%04h %04h %04h %04h}", m, f, a, d);
                end
            end else begin
                // Order: A = the fault was pending before the command arrived, so
                // the fault frame goes first; B = the fault became pending while the
                // command was half-received, so the command completes first and the
                // fault frame follows.  Either way BOTH must be present and correct.
                begin : two_packets
                    logic got_fault, got_resp;
                    int p;
                    got_fault = 0; got_resp = 0;
                    for (p = 0; p < 2; p++) begin
                        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
                        if (m === 16'h55AA && f === 16'hFFFF) begin
                            if (a[0] !== 1'b1 || d !== 16'h0000) begin n_fail++; $display("[FAULT-MID-CMD] FAIL %s%0d: fault frame {%04h %04h %04h %04h} lacks the FAULT_N bit", (shape == 0) ? "A" : "B", k, m, f, a, d); end
                            else begin
                                got_fault = 1;
                                if ((shape == 0 && p != 0) || (shape == 1 && p != 1)) begin n_fail++; $display("[FAULT-MID-CMD] FAIL %s%0d: fault frame arrived as packet %0d (expected %0d)", (shape == 0) ? "A" : "B", k, p, (shape == 0) ? 0 : 1); end
                            end
                        end else if (m === 16'h55AA && f === 16'h0000 && a === REG_TOKEN_HI && d === 16'hCEFA) begin
                            got_resp = 1;
                        end else begin
                            n_fail++; $display("[FAULT-MID-CMD] FAIL %s%0d: packet %0d {%04h %04h %04h %04h} is neither the fault frame nor {55AA 0000 %04h CEFA} (a read that decodes as a WRITE shows flags=FFFF / wrong data)", (shape == 0) ? "A" : "B", k, p, m, f, a, d, REG_TOKEN_HI);
                        end
                    end
                    if (got_fault && got_resp) begin
                        n_pass++;
                        $display("[FAULT-MID-CMD] %s%0d: fault frame and correct response both delivered (%s first), %0d frame(s)", (shape == 0) ? "A" : "B", k, (shape == 0) ? "fault" : "response", i);
                    end else begin
                        n_fail++; $display("[FAULT-MID-CMD] FAIL %s%0d: fault frame %0d, response %0d", (shape == 0) ? "A" : "B", k, got_fault, got_resp);
                    end
                end
                if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) != 0) begin
                    n_fail++;
                    $display("[FAULT-MID-CMD] FAIL %s%0d: %0d unexpected extra ctrl word(s)", (shape == 0) ? "A" : "B", k, tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr);
                    while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4)
                        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
                end
            end
            // TOKEN_HI must still be at its reset value (a swallowed read that
            // decoded as a write would have changed it).
            tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI, 16'h0000);
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            if (d !== 16'hCEFA) begin n_fail++; $display("[FAULT-MID-CMD] FAIL %s%0d: TOKEN_HI = 0x%04h after the episode (expected 0xCEFA untouched)", (shape == 0) ? "A" : "B", k, d); end
            else n_pass++;
            tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        end
    end

    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[FAULT-MID-CMD] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    if (n_fail == 0) $display("[FAULT-MID-CMD] PASS — %0d checks: a fault frame never costs a command, whether the command arrives during the fault frame or is half-received when the fault fires", n_pass);
    else             $display("[FAULT-MID-CMD] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_FAULT_MID_CMD

`ifdef RUN_USB_STALL_CMDS
// ---------------------------------------------------------------------------
// SA-USBSTALL-CMDS — the host stops reading (the FT600 fills, TXE_N high) and,
// while it is not reading, keeps issuing register commands.  2026-09-10 14:23:53
// (asic_grid_asic_20260910_142349): 2.4 ms after RUN the bring-up tool's capture
// thread blocked itself for 6 s (see DeviceLayer.cpp tl_in_capture_loop) while
// sending four SPI_CFG writes; none was answered, the fifth USB write failed and
// the link had to be reconnected.  At 14:24:18 a write failed the same way with
// one command in flight.  This bench pins the FPGA's side of that contract:
//   1. commands written while the FPGA cannot send are still READ out of the
//      FT600 (the OUT FIFO never backs up onto the host: no 500 ms write timeout);
//   2. every one of them is answered, in order, once the host reads again
//      (one per frame boundary: the no-puncture hold, so N_CMD + 2 frames);
//   3. streaming resumes right-or-flagged and strict-correct within RECOVER_FRAMES;
//   4. no write into a full FT600, no punctured frame.
// ---------------------------------------------------------------------------
task automatic run_SA_USB_STALL_CMDS();
    localparam int N_STALL        = 2;
    localparam int STALL_NS       = 2_000_000;   // ~2 ms host read gap
    localparam int N_CMD          = 4;           // commands issued inside each gap
    localparam int DRAIN_NS       = 400_000;     // FPGA must have read them within this
    localparam int RECOVER_FRAMES = 8;
    localparam int SETTLE_FRAMES  = 8;
    localparam int BETWEEN_FRAMES = 3;
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, dropped, queued;
    int          frame_no = 0, k, ch, i, all_clean, window_clean, pending;

    $display("");
    $display("[SA-USBSTALL-CMDS] host read gaps (~%0d us) with %0d commands issued inside each: FPGA must keep reading and answer them all", STALL_NS / 1000, N_CMD);

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);   // ACQ_ALL_RUN legs 0,3
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);

    // ── start-up: an all-clean frame within SETTLE_FRAMES ─────────────────────
    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin n_pass++; $display("[SA-USBSTALL-CMDS] start-up: all-clean frame within %0d frame(s)", i + 1); end
    else begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    for (k = 0; k < N_STALL; k++) begin
        for (i = 0; i < BETWEEN_FRAMES; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        $display("[SA-USBSTALL-CMDS] gap %0d: host stops reading  t=%0t", k, $time);
        tb_top.u_ft600q.set_txe_backpressure(1'b1);
        #(STALL_NS / 4);
        // The tool's 14:23:53 traffic: SPI_CFG arm writes + a register read, while
        // the FT600 is full.  Here: token write, mask read, token read, WD pet.
        $display("[SA-USBSTALL-CMDS] gap %0d: %0d commands issued while not reading  t=%0t", k, N_CMD, $time);
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_TOKEN_HI,    16'h1230 + k);
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_EN_MASK, 16'h0000);
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_TOKEN_HI,    16'h0000);
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0052,        16'h0000);   // WD_PET, bit0 clear
        #DRAIN_NS;
        // 1. the FPGA read them out of the FT600 although it cannot send anything
        queued = tb_top.u_ft600q.rx_count();
        if (queued != 0) begin
            n_fail++;
            $display("[SA-USBSTALL-CMDS] FAIL gap %0d: %0d command word(s) still unread in the FT600 %0d us after being written — the FPGA stopped taking commands while it could not send (the host's FT_WritePipe would back up and time out)", k, queued, DRAIN_NS / 1000);
        end else begin
            n_pass++;
            $display("[SA-USBSTALL-CMDS] gap %0d: all %0d command words read by the FPGA within %0d us while TXE_N was high", k, 4 * N_CMD, DRAIN_NS / 1000);
        end
        #(STALL_NS - STALL_NS / 4 - DRAIN_NS);
        tb_top.u_ft600q.set_txe_backpressure(1'b0);
        $display("[SA-USBSTALL-CMDS] gap %0d: host reading again  t=%0t", k, $time);

        // 3. streaming: judge the first frames out, skip the pile, strict-correct within RECOVER_FRAMES
        pending = tb_top.u_ft600q.telem_frames_pending();
        for (i = 0; i < 2 && i < pending - 2; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        window_clean = 0;
        for (i = 0; i < RECOVER_FRAMES && !window_clean; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-USBSTALL-CMDS] frame[%0d] (after gap %0d, +%0d) hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, i, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (all_clean) window_clean = 1;
        end
        if (window_clean) begin n_pass++; $display("[SA-USBSTALL-CMDS] after gap %0d: strict-correct again within %0d frame(s)", k, i); end
        else begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL after gap %0d: no all-correct frame within %0d frames", k, RECOVER_FRAMES); end

        // 2. every command answered, in order.  The decoder holds each response
        //    until a frame boundary (no-puncture contract), so with the stream
        //    running they arrive one per 400 us frame: allow N_CMD + 2 frames
        //    from the moment the host reads again, counting the frames above.
        for (; i < N_CMD + 2 && (tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4 * N_CMD; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        $display("[SA-USBSTALL-CMDS] gap %0d: %0d response word(s) after %0d frame(s) of reading again", k, tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr, i);
        if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) < 4 * N_CMD) begin
            n_fail++;
            $display("[SA-USBSTALL-CMDS] FAIL gap %0d: only %0d of %0d response words arrived within %0d frames of the host reading again — command(s) issued during the gap were lost or the decoder is stuck",
                     k, tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr, 4 * N_CMD, N_CMD + 2);
            while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4)
                tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        end else begin
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            if (m !== 16'h55AA || a !== REG_TOKEN_HI || d !== (16'h1230 + k)) begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL gap %0d resp 1: {%04h %04h %04h %04h} (expected token write echo 0x%04h)", k, m, f, a, d, 16'h1230 + k); end
            else n_pass++;
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            if (m !== 16'h55AA || a !== REG_SPI_EN_MASK || d[3:0] !== 4'h9) begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL gap %0d resp 2: {%04h %04h %04h %04h} (expected SPI_EN_MASK 0x0009)", k, m, f, a, d); end
            else n_pass++;
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            if (m !== 16'h55AA || a !== REG_TOKEN_HI || d !== (16'h1230 + k)) begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL gap %0d resp 3: {%04h %04h %04h %04h} (expected TOKEN_HI 0x%04h)", k, m, f, a, d, 16'h1230 + k); end
            else n_pass++;
            tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
            if (m !== 16'h55AA || a !== 16'h0052) begin n_fail++; $display("[SA-USBSTALL-CMDS] FAIL gap %0d resp 4: {%04h %04h %04h %04h} (expected WD_PET echo)", k, m, f, a, d); end
            else begin n_pass++; $display("[SA-USBSTALL-CMDS] gap %0d: all %0d commands answered in order after the host resumed reading", k, N_CMD); end
        end
        if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) != 0) begin
            n_fail++;
            $display("[SA-USBSTALL-CMDS] FAIL gap %0d: %0d unexpected ctrl word(s) after the %0d responses", k, tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr, N_CMD);
            while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4)
                tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        end
    end

    if (tb_top.u_ft600q.overflow_drop_count != 0) begin
        n_fail++;
        $display("[SA-USBSTALL-CMDS] FAIL %0d word(s) written while TXE_N was high (dropped by the FT600)", tb_top.u_ft600q.overflow_drop_count);
    end else n_pass++;
    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[SA-USBSTALL-CMDS] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    if (n_fail == 0)
        $display("[SA-USBSTALL-CMDS] PASS — %0d checks: commands issued during %0d host read gaps were all read, all answered in order, stream right-or-flagged and re-aligned", n_pass, N_STALL);
    else
        $display("[SA-USBSTALL-CMDS] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_USB_STALL_CMDS

`ifdef RUN_IMP_CYCLE
// ---------------------------------------------------------------------------
// SA-IMP-CYCLE — the sequential impedance sweep's per-pixel command cycle, as
// the bring-up tool issues it (CannedFunctions::asicImpedanceSweepSequential,
// 2026-09-10 19:24): with two legs streaming, the host's capture reader stops
// (the FT600 fills for ~1.5 ms), then, reading again, the tool sends
//   RUN=0 (CH_CTRL leg 0)  ->  CS2_PASS  ->  3-byte Pixel write  ->  TELEM_EN
//   ->  SPI_EN_MASK  ->  RUN=1
// waiting for each response.  After 97 pixels the FT_WritePipe of the next
// command timed out (status 19) with the previous command answered 1 ms
// earlier.  This bench pins the FPGA's side of that cycle, N_CYCLE times:
//   1. every command is answered within RESP_NS (the decoder never wedges after
//      RUN=0 lands mid-frame with the FT600 full);
//   2. the FPGA keeps draining RXF between responses (no unread command words);
//   3. after RUN=1 the stream is right-or-flagged and strict-correct again
//      within RECOVER_FRAMES;
//   4. no write into a full FT600, no punctured frame.
// If it passes, the wedge is on the FT600 / driver / host side of the OUT pipe.
// ---------------------------------------------------------------------------
task automatic cfg_write3(input logic [1:0] c, input logic [7:0] b0, input logic [7:0] b1, input logic [7:0] b2);
    logic [15:0] mm, ff, aa, dd;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0030, {b0, b1});
    tb_top.u_ft600q.wait_response_frame_typed(mm, ff, aa, dd);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0032, {b2, 8'h00});
    tb_top.u_ft600q.wait_response_frame_typed(mm, ff, aa, dd);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0031, spi_cfg_ctrl_word(c, 1'b1, 1'b1, 3'd3));
    tb_top.u_ft600q.wait_response_frame_typed(mm, ff, aa, dd);
    #100_000;
endtask

// One register command with a bounded wait for its response.  Icarus has no
// `ref` task ports, so the verdicts accumulate in these two and are folded into
// the task's counters at the end.
int ic_npass = 0, ic_nfail = 0;
task automatic ic_cmd(input logic [15:0] flags, input logic [15:0] addr, input logic [15:0] data,
                      input string what);
    logic [15:0] m, f, a, d;
    time t0;
    t0 = $time;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags, addr, data);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (($time - t0) > 64'd3_000_000) begin
        ic_nfail++;
        $display("[SA-IMP-CYCLE] FAIL %s: response took %0d us (tool times out at 1.5 s; the decoder is holding it)", what, ($time - t0) / 1000);
    end else if (m !== 16'h55AA || a !== addr) begin
        ic_nfail++;
        $display("[SA-IMP-CYCLE] FAIL %s: response {%04h %04h %04h %04h}", what, m, f, a, d);
    end else ic_npass++;
endtask

task automatic run_IMP_CYCLE();
    localparam int N_CYCLE        = 3;
    localparam int GAP_NS         = 1_500_000;   // the reader stopped: ~1.5 ms before RUN=0 lands
    localparam int RECOVER_FRAMES = 8;
    localparam int SETTLE_FRAMES  = 8;
    localparam int DWELL_FRAMES   = 4;           // the tool's dwell is 50 ms; 4 frames here
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, dropped, queued;
    int          frame_no = 0, k, ch, i, all_clean, window_clean, pending;

    $display("");
    $display("[SA-IMP-CYCLE] the impedance sweep's per-pixel cycle x%0d: reader gap, RUN=0 mid-frame, CS2_PASS, Pixel write, TELEM_EN, mask, RUN=1", N_CYCLE);

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);   // ACQ_ALL_RUN legs 0,3
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);

    hx_grab_frame(hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin n_pass++; $display("[SA-IMP-CYCLE] start-up: all-clean frame within %0d frame(s)", i + 1); end
    else begin n_fail++; $display("[SA-IMP-CYCLE] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    for (k = 0; k < N_CYCLE; k++) begin
        // the dwell: the tool reads frames for 50 ms
        for (i = 0; i < DWELL_FRAMES; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        // "Capture: stopped" — the reader is gone until the RUN=0 do_cmd reads again
        $display("[SA-IMP-CYCLE] cycle %0d: reader stops  t=%0t", k, $time);
        tb_top.u_ft600q.set_txe_backpressure(1'b1);
        #GAP_NS;
        tb_top.u_ft600q.set_txe_backpressure(1'b0);
        $display("[SA-IMP-CYCLE] cycle %0d: RUN=0 leg 0 (mid-frame, FT600 was full)  t=%0t", k, $time);
        ic_cmd(flags_wr(), 16'h0100, 16'h0000, "RUN=0");
        // the tool's asicPassthroughWrite: CS2_PASS then the 3-byte pixel word,
        // then TELEM_EN, mask, RUN — each a register write pair answered in turn
        cfg_write2(2'd0, 8'h04, 8'h00);                 // CS2_PASS
        cfg_write3(2'd0, 8'hC5, 8'h11, 8'h01);          // Pixel {C5 11 01}
        cfg_write2(2'd0, 8'h02, 8'h01);                 // TELEM_EN normal
        ic_cmd(flags_wr(), REG_SPI_EN_MASK, 16'h0009, "SPI_EN_MASK");
        ic_cmd(flags_wr(), 16'h0100, 16'h0001, "RUN=1");
        // 2. nothing left unread in the FT600's OUT FIFO
        #200_000;
        queued = tb_top.u_ft600q.rx_count();
        if (queued != 0) begin n_fail++; $display("[SA-IMP-CYCLE] FAIL cycle %0d: %0d command word(s) unread in the FT600 200 us after the last command", k, queued); end
        else n_pass++;
        // 3. the stream comes back right-or-flagged and strict-correct
        pending = tb_top.u_ft600q.telem_frames_pending();
        tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        window_clean = 0;
        for (i = 0; i < RECOVER_FRAMES && !window_clean; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-IMP-CYCLE] frame[%0d] (cycle %0d, +%0d) cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, i, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (all_clean) window_clean = 1;
        end
        if (window_clean) begin n_pass++; $display("[SA-IMP-CYCLE] cycle %0d: strict-correct again within %0d frame(s) of RUN=1", k, i); end
        else begin n_fail++; $display("[SA-IMP-CYCLE] FAIL cycle %0d: no all-correct frame within %0d frames of RUN=1", k, RECOVER_FRAMES); end
        if ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) != 0) begin
            n_fail++;
            $display("[SA-IMP-CYCLE] FAIL cycle %0d: %0d unexpected ctrl word(s)", k, tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr);
            while ((tb_top.u_ft600q.ctrl_wr_ptr - tb_top.u_ft600q.ctrl_rd_ptr) >= 4)
                tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        end
    end

    n_pass += ic_npass; n_fail += ic_nfail;
    if (tb_top.u_ft600q.overflow_drop_count != 0) begin
        n_fail++;
        $display("[SA-IMP-CYCLE] FAIL %0d word(s) written while TXE_N was high (dropped by the FT600)", tb_top.u_ft600q.overflow_drop_count);
    end else n_pass++;
    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[SA-IMP-CYCLE] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    if (n_fail == 0)
        $display("[SA-IMP-CYCLE] PASS — %0d checks: %0d sweep cycles, every command answered, FT600 OUT drained, stream re-aligned", n_pass, N_CYCLE);
    else
        $display("[SA-IMP-CYCLE] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_IMP_CYCLE

`ifdef RUN_USB_STALL
// ============================================================================
// USB back-pressure — host read gaps must not leave a leg silently rotated
// (compiled with +define+RUN_HOST_XACT +define+RUN_USB_STALL)
//
// Hardware, 2026-09-10 10:10 (asic_grid_asic_20260910_101002, legs 5+8, 0xACED):
// every frame flagged, ovf=0x9 sticky on both legs, skip=9, ACED bit-rotated by
// every offset 1..15 — and the bring-up tool's own capture stat said
// "reap gap max 2098 us": its USB read loop paused ~2 ms.  A 2 ms gap fills the
// FT600 (txe_n high), then the 1024-word CDC (~100 us at 10 Mword/s), stalls
// the engine, and overflows the 16-word leg FIFOs (~6 us at 2.56 Mword/s).  A
// FIFO that drops ARBITRARY words on overflow shifts that leg's 16-plane
// lattice by the drop count, and the leg decodes bit-rotated until something
// re-anchors it.  The bench's FT600 model is always ready, so no other target
// sees this.
//
// Contract at the USB boundary, same as run_SA_HOST_XACT:
//   * every delivered leg-frame is strictly correct (plane_offset 0,
//     sample_offset == its phase word) OR carries a fault flag (bits 14..12);
//   * within RECOVER_FRAMES after each gap an all-correct frame reappears —
//     data lost DURING the gap is inevitable (the host was not reading), but
//     alignment must recover;
//   * the FPGA never writes into a full FT600 (the TLM drops and counts those).
// Leg mask 0x9 mirrors the recording; disabled legs are skipped by the judge.
// ============================================================================
task automatic run_SA_USB_STALL();
    localparam int N_STALL        = 3;         // host read gaps
    localparam int STALL_NS       = 2_000_000; // ~2 ms each ("reap gap max 2098 us")
    localparam int RECOVER_FRAMES = 8;         // all-correct frame must reappear within this
    localparam int SETTLE_FRAMES  = 8;
    localparam int BETWEEN_FRAMES = 3;         // clean frames between gaps (steady-state check)
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, dropped;
    int          frame_no = 0, k, ch, i, all_clean, window_clean, pending;

    $display("");
    $display("[SA-USBSTALL] host USB read gaps (~%0d us) during streaming — right or flagged, and re-aligned after each gap", STALL_NS / 1000);

    // ── ASIC models: unique identity on the two armed legs (5 -> ch0, 8 -> ch3) ─
    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask = 4'h9;

    // ── Bring-up as the tool did it: mask 0x9, tails 0 and 3 programmed ────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, 16'h0009);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch += 3) begin
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // CTRL: RO_RSTn + MCLK_EN
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, 16'h0009);   // ACQ_ALL_RUN legs 0,3
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif
    tb_top.u_ft600q.enable_frame_trace(1'b1);

    // ── Phase A: start-up — an all-clean frame within SETTLE_FRAMES ────────────
    hx_grab_frame(hdr);
    $display("[SA-USBSTALL] warm-up frame hdr=0x%04X discarded", hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        $display("[SA-USBSTALL] frame[%0d] hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                 frame_no, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin n_pass++; $display("[SA-USBSTALL] start-up: all-clean frame within %0d frame(s)", i + 1); end
    else begin n_fail++; $display("[SA-USBSTALL] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end

    // ── Phase S: host read gaps ───────────────────────────────────────────────
    for (k = 0; k < N_STALL; k++) begin
        // steady state between gaps: every frame right-or-flagged
        for (i = 0; i < BETWEEN_FRAMES; i++) begin
            hx_grab_frame(hdr); frame_no++;
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        $display("[SA-USBSTALL] gap %0d: host stops reading for %0d us  t=%0t", k, STALL_NS / 1000, $time);
        tb_top.u_ft600q.set_txe_backpressure(1'b1);
        #STALL_NS;
        tb_top.u_ft600q.set_txe_backpressure(1'b0);
        $display("[SA-USBSTALL] gap %0d: host reading again  t=%0t", k, $time);
        // Judge the first two frames delivered after the gap (where the damage
        // shows), skip the pile a live host would have read, then require an
        // all-correct frame within RECOVER_FRAMES.
        pending = tb_top.u_ft600q.telem_frames_pending();
        for (i = 0; i < 2 && i < pending - 2; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-USBSTALL] frame[%0d] (after gap %0d, first) hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        end
        tb_top.u_ft600q.keep_newest_telemetry_frames(2, dropped);
        if (dropped > 0)
            $display("[SA-USBSTALL]   %0d frame(s) skipped (a live host would have read them)", dropped);
        window_clean = 0;
        for (i = 0; i < RECOVER_FRAMES && !window_clean; i++) begin
            hx_grab_frame(hdr); frame_no++;
            $display("[SA-USBSTALL] frame[%0d] (after gap %0d, +%0d) hdr=0x%04h cnt_lo=%0d phase={%04h %04h %04h %04h}",
                     frame_no, k, i, hdr, tb_top.u_ft600q.v3_count_lo, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
            hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
            if (all_clean) window_clean = 1;
        end
        if (window_clean) begin
            n_pass++;
            $display("[SA-USBSTALL] after gap %0d: armed legs strict-correct again within %0d frame(s)", k, i);
        end else begin
            n_fail++;
            $display("[SA-USBSTALL] FAIL after gap %0d: no all-correct frame within %0d frames — leg(s) left rotated by the overflow (the 2026-09-10 recording)", k, RECOVER_FRAMES);
        end
    end

    // ── The FPGA must never write into a full FT600 ───────────────────────────
    if (tb_top.u_ft600q.overflow_drop_count != 0) begin
        n_fail++;
        $display("[SA-USBSTALL] FAIL %0d word(s) written while TXE_N was high (dropped by the FT600)", tb_top.u_ft600q.overflow_drop_count);
    end else begin
        n_pass++;
        $display("[SA-USBSTALL] no write into a full FT600 across %0d gaps", N_STALL);
    end
    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[SA-USBSTALL] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    if (n_fail == 0)
        $display("[SA-USBSTALL] PASS — %0d checks: every leg-frame right or flagged, re-aligned after each of %0d host read gaps", n_pass, N_STALL);
    else
        $display("[SA-USBSTALL] FAIL — %0d check(s) failed (%0d passed): unflagged wrong data, or no re-alignment after a host read gap", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_USB_STALL

`ifdef RUN_USB_JITTER
// ============================================================================
// SA-USBJITTER — many SHORT host USB read gaps (the real host's read-loop
// jitter) must not make the image unusable.
//
// The 2026-09-10 11:07 recordings (from-scratch-2 iteration 2 flashed; one leg
// mask 0x1, then legs 5+8 mask 0x9; NO command traffic during capture; the
// tool's own stat "reap gap max 240..589 us"): the reported phase word changed
// every ~17 frames with one leg and every ~6 frames with two, always with the
// undf bit set on the re-aligning leg(s), the ovf bit sticky, and the host
// concealing 6% / 13% of all frames.  Decoding the recorded data showed the
// phase words RIGHT for 99.8% of aligned frames — but ~1.5% of the re-align
// events produced an UNFLAGGED frame holding the old alignment in its first
// slots and the new one in the rest (the re-anchor happened to land on a
// 16-slot boundary, so no slot was stuffed and undf stayed clear), the two-leg
// run repeated the frame counter 116 times (a frame emitted faster than a
// sweep), and both legs' phases jumped by the SAME amount at every joint
// event.  With two legs every such event disturbs both halves of the image.
//
// Contract judged here (host-realistic — see hx_excuse_mask):
//   (1) every leg-frame is strict-correct OR carries undf/par (not the sticky
//       ovf bit): what the host conceals is what may be wrong;
//   (2) a frame whose leg phase word differs from the previous frame's is
//       flagged undf — a silent alignment change is the "jump";
//   (3) the frame counter never REPEATS (a frame cannot be shorter than one
//       sweep); it may skip (a host gap delays frames, that is honest);
//   (4) every gap costs at most one re-alignment per leg, and the design is
//       strict-correct again on all armed legs once the gaps stop;
//   (5) the FPGA never writes into a full FT600, no frame is punctured.
// Also reported (not judged): clean vs flagged leg-frames, phase changes, and
// how the phase moved per gap, so a stability regression is visible.
// ============================================================================

`ifdef JITTER_DEBUG
// Timeline of one leg's alignment machinery (debug only).
reg [9:0] dbg_tick_q; reg dbg_pl0_q, dbg_pl3_q, dbg_anch0_q, dbg_anch3_q, dbg_txe_q; reg [2:0] dbg_st_q;
always @(posedge tb_top.dut_con.engine.clk) begin
    if (tb_top.dut_con.quad.ovf_pulse != 0)
        $display("[DBG] t=%0t OVF legs=%b tick_cnt=%0d state=%0d cnt0=%0d cdc_full=%b",
                 $time, tb_top.dut_con.quad.ovf_pulse, tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state,
                 tb_top.dut_con.quad.cnt0, !tb_top.dut_con.engine.telem_tx_ready);
    if (tb_top.dut_con.ch_gen[0].strm.anchor_flush)
        $display("[DBG] t=%0t FLUSH leg0 (re-anchor) tick_cnt=%0d state=%0d cnt0=%0d", $time,
                 tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state, tb_top.dut_con.quad.cnt0);
    if (tb_top.dut_con.ch_gen[3].strm.anchor_flush)
        $display("[DBG] t=%0t FLUSH leg3 (re-anchor) tick_cnt=%0d state=%0d cnt3=%0d", $time,
                 tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state, tb_top.dut_con.quad.cnt3);
    if (tb_top.dut_con.engine.phase_locked_0 != dbg_pl0_q)
        $display("[DBG] t=%0t leg0 aligned=%b tick_cnt=%0d phase_save=%0d anchored=%b wait_out=%b cnt0=%0d", $time,
                 tb_top.dut_con.engine.phase_locked_0, tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.phase_save_0,
                 tb_top.dut_con.engine.leg_anchored[0], tb_top.dut_con.engine.wait_out, tb_top.dut_con.quad.cnt0);
    if (tb_top.dut_con.engine.phase_locked_3 != dbg_pl3_q)
        $display("[DBG] t=%0t leg3 aligned=%b tick_cnt=%0d phase_save=%0d anchored=%b wait_out=%b cnt3=%0d", $time,
                 tb_top.dut_con.engine.phase_locked_3, tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.phase_save_3,
                 tb_top.dut_con.engine.leg_anchored[3], tb_top.dut_con.engine.wait_out, tb_top.dut_con.quad.cnt3);
    if (tb_top.dut_con.engine.leg_anchored[0] != dbg_anch0_q)
        $display("[DBG] t=%0t leg0 anchored=%b tick_cnt=%0d state=%0d", $time, tb_top.dut_con.engine.leg_anchored[0],
                 tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state);
    if (tb_top.dut_con.engine.leg_anchored[3] != dbg_anch3_q)
        $display("[DBG] t=%0t leg3 anchored=%b tick_cnt=%0d state=%0d", $time, tb_top.dut_con.engine.leg_anchored[3],
                 tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state);
    if (tb_top.dut_con.engine.state == 3'd1 && dbg_st_q != 3'd1)
        $display("[DBG] t=%0t S_HDR frame_cnt=%0d tick_2k5 phase: frame_event_pending=%b", $time,
                 tb_top.dut_con.engine.ext_frame_cnt[15:0], tb_top.dut_con.engine.frame_event_pending);
    if (tb_top.dut_con.engine.state == 3'd5 && dbg_st_q != 3'd5)
        $display("[DBG] t=%0t S_PHASE_WR (frame end) undf=%b aligned=%b%b", $time, tb_top.dut_con.engine.undf_cnt,
                 tb_top.dut_con.engine.phase_locked_3, tb_top.dut_con.engine.phase_locked_0);
    if (tb_top.u_ft600q.txe_n != dbg_txe_q)
        $display("[DBG] t=%0t TXE_N=%b tick_cnt=%0d state=%0d cnt0=%0d cnt3=%0d", $time, tb_top.u_ft600q.txe_n,
                 tb_top.dut_con.engine.tick_cnt, tb_top.dut_con.engine.state, tb_top.dut_con.quad.cnt0, tb_top.dut_con.quad.cnt3);
    if (tb_top.dut_con.engine.wait_out && tb_top.dut_con.engine.state == 3'd2 && tb_top.dut_con.engine.tick_req)
        $display("[DBG] t=%0t WAIT_OUT tick tick_cnt=%0d must_have=%b leg_empty=%b", $time, tb_top.dut_con.engine.tick_cnt,
                 tb_top.dut_con.engine.must_have, tb_top.dut_con.engine.leg_empty);
    dbg_pl0_q <= tb_top.dut_con.engine.phase_locked_0; dbg_pl3_q <= tb_top.dut_con.engine.phase_locked_3;
    dbg_anch0_q <= tb_top.dut_con.engine.leg_anchored[0]; dbg_anch3_q <= tb_top.dut_con.engine.leg_anchored[3];
    dbg_st_q <= tb_top.dut_con.engine.state; dbg_txe_q <= tb_top.u_ft600q.txe_n;
end
`endif
`ifndef JITTER_MASK
`define JITTER_MASK 4'h9
`endif
task automatic run_SA_USB_JITTER();
    localparam int N_FRAMES       = 40;         // frames judged under jitter (16 ms)
    localparam int GAP_MIN_US     = 60;         // the CDC (512 words) + leg FIFO absorb ~56 us
    localparam int GAP_MAX_US     = 400;        // the tool's worst per-second reap gap
    localparam int GAP_PROB_PPM   = 40;         // per write cycle: ~1 gap per 2.5 ms of stream
    localparam int RECOVER_FRAMES = 8;
    localparam int SETTLE_FRAMES  = 8;
    localparam logic [3:0] MASK   = `JITTER_MASK;
    logic [15:0] m, f, a, d;
`ifdef ICARUS
    logic [47:0] tx, rx;
`else
    logic [7:0]  tx[0:5], rx[0:5];
`endif
    logic [15:0] hdr;
    int          n_fail = 0, n_pass = 0, n_flush, dropped;
    int          frame_no = 0, i, ch, all_clean, window_clean, v;
    int          prev_ph [0:3];
    int          ph_changes [0:3];
    int          n_clean_leg = 0, n_flagged_leg = 0, n_dup = 0, n_skip = 0;
    int          last_cnt, cnt_now, delta, gaps, gap_cycles, gap_words, armed_legs;

    $display("");
    $display("[SA-USBJITTER] short host USB read gaps (%0d..%0d us, ~1 per 2.5 ms) during streaming, legs mask=0x%01h — phase must stay right, changes flagged, no repeated frame counter",
             GAP_MIN_US, GAP_MAX_US, MASK);

    // ── ASIC models: unique identity on every leg (only armed legs are judged) ──
    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[1].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[2].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask  = MASK;
    hx_excuse_mask = 3'b101;        // undf or par excuse; the sticky ovf bit does not
    armed_legs = 0;
    for (ch = 0; ch < 4; ch++) if (MASK[ch]) armed_legs++;

    // ── Bring-up as the tool did it ───────────────────────────────────────────
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, {12'h0, MASK});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch++) begin
        if (!MASK[ch]) continue;
`ifdef ICARUS
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // CTRL: RO_RSTn + MCLK_EN
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal
`else
        tx = '{8'h01, 8'h11, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
        tx = '{8'h02, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00}; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);
`endif
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, {12'h0, MASK});   // ACQ_ALL_RUN
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
`ifdef ICARUS
    tb_top.u_ft600q.flush_tx_capture(n_flush);
`else
    begin logic [15:0] fw [0:4095]; tb_top.u_ft600q.flush_tx_capture(fw, n_flush); end
`endif

    // ── Phase A: start-up — an all-clean frame within SETTLE_FRAMES ────────────
    hx_grab_frame(hdr);
    $display("[SA-USBJITTER] warm-up frame hdr=0x%04X discarded", hdr);
    window_clean = 0;
    for (i = 0; i < SETTLE_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) begin window_clean = 1; break; end
    end
    if (window_clean) begin n_pass++; $display("[SA-USBJITTER] start-up: all-clean frame within %0d frame(s)", i + 1); end
    else begin n_fail++; $display("[SA-USBJITTER] FAIL start-up: no all-clean frame within %0d frames", SETTLE_FRAMES); end
    for (ch = 0; ch < 4; ch++) begin prev_ph[ch] = hx_phase[ch] & 16'h03FF; ph_changes[ch] = 0; end
    last_cnt = tb_top.u_ft600q.v3_count_lo;

    // ── Phase J: random short read gaps while N_FRAMES stream ─────────────────
    // 66 MHz FT600 clock: 15.15 ns per cycle.
    tb_top.u_ft600q.set_txe_random_backpressure(1'b1, GAP_PROB_PPM, GAP_MIN_US * 66, GAP_MAX_US * 66);
    for (i = 0; i < N_FRAMES; i++) begin
        hx_grab_frame(hdr); frame_no++;
        cnt_now = tb_top.u_ft600q.v3_count_lo;
        delta   = (cnt_now - last_cnt) & 16'hFFFF;
        last_cnt = cnt_now;
        $display("[SA-USBJITTER] frame[%0d] cnt_lo=%0d (+%0d) phase={%04h %04h %04h %04h}",
                 frame_no, cnt_now, delta, hx_phase[0], hx_phase[1], hx_phase[2], hx_phase[3]);
        // (3) the frame counter never repeats
        if (delta == 0) begin
            n_dup++; n_fail++;
            $display("[SA-USBJITTER] FAIL frame[%0d]: frame counter REPEATED (%0d) — a frame was emitted faster than one sweep (the 11:07:28 recording: dup=116)", frame_no, cnt_now);
        end else begin
            n_pass++;
            if (delta > 1) n_skip += delta - 1;
        end
        // (1) right or flagged (undf/par), per armed leg; (2) a phase change is flagged
        for (ch = 0; ch < 4; ch++) begin
            if (!MASK[ch]) continue;
            hx_judge_leg(frame_no, ch, v);
            if (v == 2) n_fail++; else n_pass++;
            if (v == 0) n_clean_leg++; else if (v == 1) n_flagged_leg++;
            // An alignment CHANGE is a new phase value below 64.  1023 (no aligned
            // data this frame) is not a change: a leg that comes back at the same
            // phase kept its alignment.
            if ((hx_phase[ch] & 16'h03FF) < 64 && (hx_phase[ch] & 16'h03FF) != prev_ph[ch]) begin
                ph_changes[ch]++;
                $display("[SA-USBJITTER] frame[%0d] leg%0d: phase %0d -> %0d (delta %0d groups)%s",
                         frame_no, ch, prev_ph[ch], hx_phase[ch] & 16'h03FF,
                         (prev_ph[ch] < 64) ? (((hx_phase[ch] & 16'h03FF) - prev_ph[ch]) & 63) : -1,
                         hx_phase[ch][12] ? "" : "  <-- UNFLAGGED");
                if (!hx_phase[ch][12]) begin
                    n_fail++;
                    $display("[SA-USBJITTER] FAIL frame[%0d] leg%0d: alignment changed without the undf flag — the host displays a frame that is half old, half new alignment", frame_no, ch);
                end else n_pass++;
                prev_ph[ch] = hx_phase[ch] & 16'h03FF;
            end
        end
    end
    tb_top.u_ft600q.set_txe_random_backpressure(1'b0, 0, 0, 0);
    tb_top.u_ft600q.get_bp_stats(gaps, gap_cycles, gap_words);
    $display("[SA-USBJITTER] %0d gap(s), %0d us stalled in total, %0d write(s) held off", gaps, gap_cycles / 66, gap_words);

    // ── Phase R: gaps stop -> strict-correct again on every armed leg ──────────
    window_clean = 0;
    for (i = 0; i < RECOVER_FRAMES && !window_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
        if (all_clean) window_clean = 1;
    end
    if (window_clean) begin n_pass++; $display("[SA-USBJITTER] after the gaps: all armed legs strict-correct within %0d frame(s)", i); end
    else begin n_fail++; $display("[SA-USBJITTER] FAIL after the gaps: no all-correct frame within %0d frames", RECOVER_FRAMES); end

    // ── (4) at most one re-alignment per gap per leg ──────────────────────────
    for (ch = 0; ch < 4; ch++) begin
        if (!MASK[ch]) continue;
        if (ph_changes[ch] > gaps) begin
            n_fail++;
            $display("[SA-USBJITTER] FAIL leg%0d: %0d phase changes for %0d gaps — the leg re-aligned more often than the host stalled", ch, ph_changes[ch], gaps);
        end else begin
            n_pass++;
            $display("[SA-USBJITTER] leg%0d: %0d phase change(s) over %0d gap(s)", ch, ph_changes[ch], gaps);
        end
    end

    // ── (5) the FPGA must never write into a full FT600; no puncture ──────────
    if (tb_top.u_ft600q.overflow_drop_count != 0) begin
        n_fail++;
        $display("[SA-USBJITTER] FAIL %0d word(s) written while TXE_N was high (dropped by the FT600)", tb_top.u_ft600q.overflow_drop_count);
    end else n_pass++;
    if (tb_top.u_ft600q.puncture_count != 0) begin
        n_fail += tb_top.u_ft600q.puncture_count;
        $display("[SA-USBJITTER] FAIL %0d telemetry frame(s) punctured", tb_top.u_ft600q.puncture_count);
    end

    $display("");
    $display("[SA-USBJITTER] stability: %0d clean / %0d flagged leg-frames of %0d under jitter; frame counter skipped %0d, repeated %0d",
             n_clean_leg, n_flagged_leg, N_FRAMES * armed_legs, n_skip, n_dup);
    if (n_fail == 0)
        $display("[SA-USBJITTER] PASS — %0d checks: right or flagged, every alignment change flagged, no repeated frame counter, <=1 re-align per gap, re-aligned after the gaps", n_pass);
    else
        $display("[SA-USBJITTER] FAIL — %0d check(s) failed (%0d passed): unflagged wrong data, silent alignment change, repeated frame counter, or no re-alignment", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif  // RUN_USB_JITTER

`endif  // RUN_HOST_XACT
