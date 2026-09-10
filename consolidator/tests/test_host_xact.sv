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
        flagged = (phw[14] | phw[13] | phw[12]) ? 1 : 0;
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

`endif  // RUN_HOST_XACT
