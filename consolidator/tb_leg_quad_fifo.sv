`timescale 1ns / 1ps
// ============================================================================
// tb_leg_quad_fifo.sv — leg_quad_fifo unit test: the read/flush race at re-anchor
//
// leg_quad_fifo reads the four legs in a fixed sequence per tick_req: it samples
// a leg's emptiness (R_IDLE / R_ISSUE1) two cycles before it captures the word
// (R_CAP0..3) and decrements the leg's count.  A re-anchor flush (spi_ch_stream's
// anchor_flush at the sweep marker) that lands between the sample and the
// capture zeroes the count first; the stale-sampled read then stepped it from 0
// to 31 — a leg that never reads empty again and serves ring content at an
// arbitrary plane offset until its next re-anchor.  Hardware: the 2026-09-10
// 11:07:28 two-leg recording had two runs of bit-rotated leg-frames that no
// phase word explained.  Board-level benches hit this only when a leg re-anchors
// while another leg keeps the engine ticking and the re-anchoring leg still
// holds stale words — rare (<1% of gaps), so it is pinned here deterministically.
//
//   TC-QF-01  control: a tick reads the head word, count decrements by one
//   TC-QF-02  flush ONE cycle after the emptiness sample (before the capture):
//             count must be 0 afterwards, the slot stuffed+flagged, and the next
//             written word must be the next head
//   TC-QF-03  flush ON the capture cycle: count 0 afterwards, next word readable
//   TC-QF-04  17th word into a full leg: one ovf_pulse, sticky fifo_ovf, word dropped
//   TC-QF-05  flush keeps the sticky fifo_ovf, clears the count
//
// Compiles under Icarus (C-05/C-10 respected: no unpacked task ports, decls first).
// ============================================================================
module tb_leg_quad_fifo;

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         sync_rst = 1'b0;
    reg  [15:0] wd0 = 16'h0, wd1 = 16'h0, wd2 = 16'h0, wd3 = 16'h0;
    reg  [3:0]  wd_valid = 4'h0;
    reg         tick_req = 1'b0;
    reg  [3:0]  ch_local_rst = 4'h0;
    reg  [3:0]  rd_mask = 4'hF;
    reg  [3:0]  flush = 4'h0;

    wire [15:0] td0, td1, td2, td3;
    wire        tick_valid;
    wire [3:0]  tick_undf, fill_ge4, fifo_full, fifo_empty, fifo_ovf, ovf_pulse;
    wire [19:0] fifo_word_cnt;

    leg_quad_fifo dut (
        .clk(clk), .rst_n(rst_n), .sync_rst(sync_rst),
        .wd_data_0(wd0), .wd_data_1(wd1), .wd_data_2(wd2), .wd_data_3(wd3),
        .wd_valid(wd_valid),
        .tick_req(tick_req),
        .tick_data_0(td0), .tick_data_1(td1), .tick_data_2(td2), .tick_data_3(td3),
        .tick_valid(tick_valid), .tick_undf(tick_undf),
        .fill_ge4(fill_ge4), .fifo_full(fifo_full), .fifo_empty(fifo_empty),
        .fifo_ovf(fifo_ovf), .fifo_word_cnt(fifo_word_cnt),
        .ovf_pulse(ovf_pulse),
        .ch_local_rst(ch_local_rst), .rd_mask(rd_mask), .flush(flush)
    );

    always #9.766 clk = ~clk;   // 51.2 MHz

    integer n_pass = 0, n_fail = 0;
    integer i, n_ovf_pulses;
    reg [15:0] got;
    reg        got_undf;

    task automatic check(input integer cond, input [8*96-1:0] msg);
        begin
            if (cond) begin n_pass = n_pass + 1; end
            else begin n_fail = n_fail + 1; $display("  FAIL: %0s", msg); end
        end
    endtask

    // Push one word into leg 0 (the receiver's word_valid is a one-cycle pulse).
    task automatic push0(input [15:0] w);
        begin
            @(posedge clk); #1; wd0 = w; wd_valid[0] = 1'b1;
            @(posedge clk); #1; wd_valid[0] = 1'b0;
            @(posedge clk); #1;   // arbitration commit
        end
    endtask

    // One tick: assert tick_req for one cycle, wait for tick_valid, return leg 0.
    task automatic tick(output [15:0] d, output undf);
        begin
            @(posedge clk); #1; tick_req = 1'b1;
            @(posedge clk); #1; tick_req = 1'b0;
            while (!tick_valid) @(posedge clk);
            #1; d = td0; undf = tick_undf[0];
        end
    endtask

    wire [4:0] cnt0 = fifo_word_cnt[4:0];

    initial begin
        $display("[TB_QF] leg_quad_fifo unit test — read/flush race, overflow");
        repeat (3) @(posedge clk); #1; rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ── TC-QF-01 control ────────────────────────────────────────────────
        push0(16'h1111); push0(16'h2222); push0(16'h3333);
        check(cnt0 == 5'd3, "TC-QF-01 three words queued");
        tick(got, got_undf);
        check(got == 16'h1111 && !got_undf, "TC-QF-01 tick returns head word 0x1111 unflagged");
        @(posedge clk); #1;
        check(cnt0 == 5'd2, "TC-QF-01 count 3 -> 2 after one tick");
        $display("[TB_QF] TC-QF-01 control: head=%04h undf=%b cnt=%0d", got, got_undf, cnt0);

        // ── TC-QF-02 flush between the emptiness sample and the capture ─────
        // Edge E1: R_IDLE samples read_empty0 (cnt0=2 -> not empty), E2: R_ISSUE1,
        // E3: R_CAP0 fires.  Assert flush so the FIFO samples it at E2.
        @(posedge clk); #1; tick_req = 1'b1;            // seen at E1
        @(posedge clk); #1; tick_req = 1'b0; flush[0] = 1'b1;   // seen at E2
        @(posedge clk); #1; flush[0] = 1'b0;            // E3: capture with cnt0 == 0
        while (!tick_valid) @(posedge clk);
        #1; got = td0; got_undf = tick_undf[0];
        @(posedge clk); #1;
        $display("[TB_QF] TC-QF-02 flush-before-capture: data=%04h undf=%b cnt=%0d empty=%b", got, got_undf, cnt0, fifo_empty[0]);
        check(cnt0 == 5'd0, "TC-QF-02 count is 0 after a flush that raced the read (was 31: 0 - 1)");
        check(fifo_empty[0] == 1'b1, "TC-QF-02 leg reads empty after the flush");
        check(got_undf == 1'b1 && got == 16'h0000, "TC-QF-02 the raced slot is stuffed and flagged, not stale data");
        push0(16'h4444);
        check(cnt0 == 5'd1, "TC-QF-02 one word after the flush counts as one");
        tick(got, got_undf);
        check(got == 16'h4444 && !got_undf, "TC-QF-02 the word written after the flush is the new head");
        @(posedge clk); #1;
        check(cnt0 == 5'd0 && fifo_empty[0], "TC-QF-02 count back to 0 after reading it");

        // ── TC-QF-03 flush on the capture cycle ─────────────────────────────
        push0(16'h5555); push0(16'h6666);
        @(posedge clk); #1; tick_req = 1'b1;            // E1 sample
        @(posedge clk); #1; tick_req = 1'b0;            // E2
        @(posedge clk); #1; flush[0] = 1'b1;            // seen at E3 = capture cycle
        @(posedge clk); #1; flush[0] = 1'b0;
        while (!tick_valid) @(posedge clk);
        @(posedge clk); #1;
        $display("[TB_QF] TC-QF-03 flush-on-capture: cnt=%0d empty=%b", cnt0, fifo_empty[0]);
        check(cnt0 == 5'd0 && fifo_empty[0], "TC-QF-03 count is 0 after a flush on the capture cycle");
        push0(16'h7777);
        tick(got, got_undf);
        check(got == 16'h7777 && !got_undf, "TC-QF-03 next word is the head after the flush");
        @(posedge clk); #1;

        // ── TC-QF-04 overflow: 17th word dropped, one pulse, sticky flag ────
        n_ovf_pulses = 0;
        for (i = 0; i < 16; i = i + 1) push0(16'h0100 + i[15:0]);
        check(cnt0 == 5'd16 && fifo_full[0], "TC-QF-04 sixteen words fill the leg");
        check(fifo_ovf[0] == 1'b0, "TC-QF-04 no overflow flag while exactly full");
        fork
            begin : count_pulses
                repeat (6) begin @(posedge clk); #1; if (ovf_pulse[0]) n_ovf_pulses = n_ovf_pulses + 1; end
            end
            begin
                push0(16'h0BAD);
            end
        join
        $display("[TB_QF] TC-QF-04 overflow: pulses=%0d sticky=%b cnt=%0d", n_ovf_pulses, fifo_ovf[0], cnt0);
        check(n_ovf_pulses == 1, "TC-QF-04 exactly one ovf_pulse for the dropped word");
        check(fifo_ovf[0] == 1'b1, "TC-QF-04 sticky fifo_ovf set");
        check(cnt0 == 5'd16, "TC-QF-04 the dropped word did not change the count");
        tick(got, got_undf);
        check(got == 16'h0100 && !got_undf, "TC-QF-04 head is still the oldest queued word");

        // ── TC-QF-05 flush clears the count, keeps the sticky flag ──────────
        @(posedge clk); #1; flush[0] = 1'b1;
        @(posedge clk); #1; flush[0] = 1'b0;
        @(posedge clk); #1;
        $display("[TB_QF] TC-QF-05 flush: cnt=%0d sticky=%b", cnt0, fifo_ovf[0]);
        check(cnt0 == 5'd0 && fifo_empty[0], "TC-QF-05 flush empties the leg");
        check(fifo_ovf[0] == 1'b1, "TC-QF-05 flush keeps the sticky overflow flag");
        push0(16'h9999);
        tick(got, got_undf);
        check(got == 16'h9999 && !got_undf, "TC-QF-05 first word after the flush is the head");

        $display("");
        $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
        if (n_fail == 0) $display("STATUS: PASS");
        else             $display("STATUS: FAIL");
        $finish;
    end

    initial begin
        #200_000;
        $display("[TB_QF] Timeout");
        $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail + 1);
        $display("STATUS: FAIL");
        $finish;
    end

endmodule
