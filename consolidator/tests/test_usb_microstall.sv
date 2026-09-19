// USB stall regression: real board acquisition path and FT600 backpressure.
// The original 512-word CDC dipped at 64--96 us. The 2026-09-19 buffer fix
// increases it to 2048 words while retaining early whole-frame admission.
// Require the previously failing 96 us idle and 64 us near-ceiling pauses to
// be absorbed. A 256 us idle stall and 300 us ceiling prefill still exceed
// finite buffering and must exercise flagged loss, then clean recovery.
// Ordered data/right-or-flagged, phase changes and counter monotonicity remain
// checked throughout. This is the existing extended board test, not the new
// short recorded-deficit reproducer.
`ifdef RUN_USB_MICROSTALL
`ifndef MICROSTALL_MASK
`define MICROSTALL_MASK 4'h9
`endif

int ms_prev_ph [0:3];   // per-leg last phase (module scope: Icarus takes no unpacked-array task ports, C-05)

function automatic int ms_stall_us(input int i);
    case (i)
        0: return 4;  1: return 16;  2: return 64;  3: return 96;
        default: return 256;
    endcase
endfunction

// Grab frames after a stall until an all-clean one or `max` frames; count the
// flagged leg-frames, the frames where BOTH armed legs are flagged, frame-counter
// skips/dups, and unflagged phase changes.  Judges right-or-flagged on the way.
task automatic ms_observe(input int max, input logic [3:0] mask, inout int frame_no,
                          inout int n_pass, inout int n_fail, inout int last_cnt,
                          output int flagged, output int pairs, output int skips, output int dups,
                          output int unflagged_changes, output int frames_seen);
    logic [15:0] hdr;
    int i, ch, v, cnt_now, delta, nflag_this, all_clean;
    flagged = 0; pairs = 0; skips = 0; dups = 0; unflagged_changes = 0; frames_seen = 0;
    for (i = 0; i < max; i++) begin
        hx_grab_frame(hdr); frame_no++; frames_seen++;
        cnt_now = tb_top.u_ft600q.v3_count_lo;
        delta   = (cnt_now - last_cnt) & 16'hFFFF;
        last_cnt = cnt_now;
        if (delta == 0) dups++;
        else if (delta > 1) skips += delta - 1;
        nflag_this = 0; all_clean = 1;
        for (ch = 0; ch < 4; ch++) begin
            if (!mask[ch]) continue;
            hx_judge_leg(frame_no, ch, v);          // 0 clean, 1 flagged, 2 WRONG AND UNFLAGGED
            if (v == 2) n_fail++; else n_pass++;
            if (v != 0) begin all_clean = 0; nflag_this++; flagged++; end
            if ((hx_phase[ch] & 16'h03FF) < 64 && (hx_phase[ch] & 16'h03FF) != ms_prev_ph[ch]) begin
                if (!hx_phase[ch][12]) unflagged_changes++;
                ms_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
            end
        end
        if (nflag_this >= 2) pairs++;
        if (all_clean && i >= 1) break;             // a clean frame after the event: settled
    end
endtask

task automatic run_SA_USB_MICROSTALL();
    localparam logic [3:0] MASK      = `MICROSTALL_MASK;
    localparam int ABSORB_MIN_US     = 96;    // regress the formerly failing 96 us stall
    localparam int CEIL_FRAMES       = 16;    // 8 hiccups at the ceiling: 4 short, 4 long
    localparam int HICCUP_US         = 8;     // absorbed everywhere: below the leg FIFO + CDC
    localparam int LONG_HICCUP_US    = 64;    // previously dipped with a 512-word buffer; must now be absorbed
    logic [15:0] m, f, a, d, hdr;
    logic [47:0] tx, rx;
    int  n_pass = 0, n_fail = 0, n_flush;
    int  frame_no = 0, i, ch, all_clean, last_cnt, armed_legs = 0;
    int  flagged, pairs, skips, dups, unfl, seen;
    int  tol_us = 0, first_dip_us = 0, dip_overflow_pairs = 0, dip_overflow_flagged = 0;
    int  c_flagged = 0, c_pairs = 0, c_skips = 0, c_dups = 0, c_unfl = 0, c_frames = 0;
    int  hic_short = 0, hic_short_dips = 0, hic_long = 0, hic_long_dips = 0, hic_long_pairs = 0, hic_long_noskip = 0;
    int  bp_events, bp_cycles, bp_words;

    $display("");
    $display("[SA-MICROSTALL] the 2026-09-15 dips: host USB micro-stalls vs the 16-word leg FIFOs, legs mask=0x%01h", MASK);

    tb_top.tail_ch[0].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[1].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[2].asic_model.data_mode = MODE_UNIQUE;
    tb_top.tail_ch[3].asic_model.data_mode = MODE_UNIQUE;
    hx_armed_mask  = MASK;
    hx_excuse_mask = 3'b101;                    // undf or par excuse wrong data; sticky ovf does not
    for (ch = 0; ch < 4; ch++) begin ms_prev_ph[ch] = -1; if (MASK[ch]) armed_legs++; end

    // ---- bring-up as the tool does it ----------------------------------------
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_EN_MASK, {12'h0, MASK});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    for (ch = 0; ch < 4; ch++) begin
        if (!MASK[ch]) continue;
        tx = 48'h01_11_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // CTRL: RO_RSTn + MCLK_EN
        tx = 48'h02_01_00_00_00_00; spi_cfg_xact(ch[1:0], tx, 3'd2, rx);   // TELEM_EN normal
    end
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), 16'h0140, {12'h0, MASK});   // ACQ_ALL_RUN
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.flush_tx_capture(n_flush);

    // ---- settle: an all-clean frame ------------------------------------------
    hx_grab_frame(hdr);
    all_clean = 0;
    for (i = 0; i < 8 && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if (!all_clean) begin n_fail++; $display("[SA-MICROSTALL] FAIL start-up: no all-clean frame within 8 frames"); end
    for (ch = 0; ch < 4; ch++) ms_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
    last_cnt = tb_top.u_ft600q.v3_count_lo;

    // ---- phase S: single stalls from idle, increasing length -----------------
    for (i = 0; i < 5; i++) begin
        // two quiet frames so the CDC is drained and the last event has settled
        hx_grab_frame(hdr); frame_no++; hx_grab_frame(hdr); frame_no++;
        last_cnt = tb_top.u_ft600q.v3_count_lo;
        for (ch = 0; ch < 4; ch++) if (MASK[ch]) ms_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
        tb_top.u_ft600q.set_txe_backpressure(1'b1);
        #(ms_stall_us(i) * 1000);
        tb_top.u_ft600q.set_txe_backpressure(1'b0);
        ms_observe(6, MASK, frame_no, n_pass, n_fail, last_cnt, flagged, pairs, skips, dups, unfl, seen);
        c_dups += dups; c_unfl += unfl;
        $display("[SA-MICROSTALL] idle, stall %3d us: %0d flagged leg-frame(s) in %0d frames, %0d frame(s) with both legs flagged, counter skipped %0d%s",
                 ms_stall_us(i), flagged, seen, pairs, skips, flagged ? "   <-- DIP" : "");
        if (flagged == 0) tol_us = ms_stall_us(i);
        else if (first_dip_us == 0) first_dip_us = ms_stall_us(i);
        if (ms_stall_us(i) == 256) begin dip_overflow_flagged = flagged; dip_overflow_pairs = pairs; end
    end
    if (tol_us >= ABSORB_MIN_US && (first_dip_us == 0 || first_dip_us > ABSORB_MIN_US)) begin
        n_pass++; $display("[MS-01] PASS from idle the design absorbs stalls up to %0d us (>= %0d us: CDC + leg FIFO)", tol_us, ABSORB_MIN_US);
    end else begin
        n_fail++; $display("[MS-01] FAIL from idle a %0d us stall already dips (tolerance %0d us, expected >= %0d us)", first_dip_us, tol_us, ABSORB_MIN_US);
    end
    if (dip_overflow_flagged > 0 && dip_overflow_pairs > 0) begin
        n_pass++; $display("[MS-02] PASS a 256 us stall dips, and the legs dip TOGETHER (%0d flagged leg-frames, %0d paired frame(s)) -- the recording's signature", dip_overflow_flagged, dip_overflow_pairs);
    end else begin
        n_fail++; $display("[MS-02] FAIL a 256 us stall: %0d flagged leg-frames, %0d paired (expected a paired dip)", dip_overflow_flagged, dip_overflow_pairs);
    end
    $display("[SA-MICROSTALL] idle tolerance %0d us, first dip at %0d us", tol_us, first_dip_us);

    // ---- phase C: at the ceiling -------------------------------------------
    // Begin with an over-capacity stall, then verify short pauses are absorbed
    // even with the same near-ceiling FT600 drain used by the original test.
    hx_grab_frame(hdr); frame_no++; hx_grab_frame(hdr); frame_no++;
    last_cnt = tb_top.u_ft600q.v3_count_lo;
    for (ch = 0; ch < 4; ch++) if (MASK[ch]) ms_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
    tb_top.u_ft600q.set_txe_backpressure(1'b1);
    #300_000;
    tb_top.u_ft600q.set_txe_backpressure(1'b0);
    tb_top.u_ft600q.set_txe_drain_limit(158, 2048);
    // the gap's own dip settles first
    ms_observe(4, MASK, frame_no, n_pass, n_fail, last_cnt, flagged, pairs, skips, dups, unfl, seen);
    if (flagged > 0 && pairs > 0) begin
        n_pass++; $display("[MS-07] PASS 300 us stall still exercises paired flagged loss");
    end else begin
        n_fail++; $display("[MS-07] FAIL over-capacity stall did not exercise flagged loss");
    end
    c_dups += dups; c_unfl += unfl;
    $display("[SA-MICROSTALL] ceiling: the 300 us gap itself: %0d flagged leg-frame(s), %0d paired, counter skipped %0d", flagged, pairs, skips);
    for (i = 0; i < CEIL_FRAMES / 2; i++) begin
        int len_us;
        len_us = (i % 2 == 0) ? HICCUP_US : LONG_HICCUP_US;
        hx_grab_frame(hdr); frame_no++; c_frames++;           // one quiet frame between hiccups
        last_cnt = tb_top.u_ft600q.v3_count_lo;
        for (ch = 0; ch < 4; ch++) if (MASK[ch]) ms_prev_ph[ch] = hx_phase[ch] & 16'h03FF;
        tb_top.u_ft600q.set_txe_backpressure(1'b1);
        #(len_us * 1000);
        tb_top.u_ft600q.set_txe_backpressure(1'b0);
        ms_observe(3, MASK, frame_no, n_pass, n_fail, last_cnt, flagged, pairs, skips, dups, unfl, seen);
        c_frames += seen; c_flagged += flagged; c_pairs += pairs; c_skips += skips; c_dups += dups; c_unfl += unfl;
        if (len_us == HICCUP_US) begin
            hic_short++; if (flagged) hic_short_dips++;
        end else begin
            hic_long++;
            if (flagged) begin hic_long_dips++; if (pairs) hic_long_pairs++; if (skips == 0) hic_long_noskip++; end
        end
        $display("[SA-MICROSTALL] ceiling, hiccup %2d us: %0d flagged leg-frame(s), %0d paired, counter skipped %0d%s",
                 len_us, flagged, pairs, skips, flagged ? "   <-- DIP" : "");
    end
    tb_top.u_ft600q.set_txe_drain_limit(0, 2048);
    tb_top.u_ft600q.get_bp_stats(bp_events, bp_cycles, bp_words);
    $display("[SA-MICROSTALL] ceiling: %0d frames; %0d us hiccups: %0d of %0d dipped; %0d us hiccups: %0d of %0d dipped (%0d paired, %0d without a frame-counter skip); counter skipped %0d, repeated %0d",
             c_frames, HICCUP_US, hic_short_dips, hic_short, LONG_HICCUP_US, hic_long_dips, hic_long, hic_long_pairs, hic_long_noskip, c_skips, c_dups);
    if (hic_long_dips == 0) begin
        n_pass++; $display("[MS-04] PASS near-ceiling %0d us pauses absorbed (%0d checked)", LONG_HICCUP_US, hic_long);
    end else begin
        n_fail++; $display("[MS-04] FAIL near-ceiling %0d us pauses dipped %0d/%0d", LONG_HICCUP_US, hic_long_dips, hic_long);
    end
    if (hic_short_dips == 0) begin
        n_pass++; $display("[MS-06] PASS at the ceiling the %0d us hiccups are absorbed (0 of %0d dipped)", HICCUP_US, hic_short);
    end else begin
        n_fail++; $display("[MS-06] FAIL at the ceiling %0d of %0d %0d us hiccups dipped", hic_short_dips, hic_short, HICCUP_US);
    end
    if (c_dups == 0 && c_unfl == 0) begin
        n_pass++; $display("[MS-05] PASS frame counter never repeated, every alignment change flagged");
    end else begin
        n_fail++; $display("[MS-05] FAIL %0d repeated frame counter(s), %0d unflagged alignment change(s)", c_dups, c_unfl);
    end
    // MS-03 is the per-leg right-or-flagged verdicts folded into n_pass/n_fail above.
    $display("[MS-03] %s right-or-flagged on every judged leg-frame", (n_fail == 0) ? "PASS" : "see FAIL lines above:");

    // ---- recover: gaps stop, strict-correct again -------------------------------
    all_clean = 0;
    for (i = 0; i < 8 && !all_clean; i++) begin
        hx_grab_frame(hdr); frame_no++;
        hx_judge_frame(frame_no, n_pass, n_fail, all_clean);
    end
    if (all_clean) begin n_pass++; $display("[SA-MICROSTALL] after the stalls: strict-correct again within %0d frame(s)", i); end
    else begin n_fail++; $display("[SA-MICROSTALL] FAIL no all-correct frame within 8 frames after the stalls"); end

    $display("");
    $display("[SA-MICROSTALL] MEASURED: idle tolerance %0d us (first dip %0d us); at the ceiling %0d us hiccups dip %0d/%0d, %0d us hiccups dip %0d/%0d",
             tol_us, first_dip_us, HICCUP_US, hic_short_dips, hic_short, LONG_HICCUP_US, hic_long_dips, hic_long);
    if (n_fail == 0) $display("[SA-MICROSTALL] PASS — %0d checks: short stalls absorbed, over-capacity loss flagged, and clean recovery", n_pass);
    else             $display("[SA-MICROSTALL] FAIL — %0d check(s) failed (%0d passed)", n_fail, n_pass);
    $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
    $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
endtask
`endif // RUN_USB_MICROSTALL
