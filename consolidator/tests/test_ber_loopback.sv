// ---------------------------------------------------------------------------
// BER-LOOPBACK — the BER Test's data path, end to end in RTL (RUN_BER_LOOPBACK).
//
// The bring-up tool's "BER Test" (CannedFunctions::berTest, tools/ber_loopback.py)
// burst-writes a PRBS-23 into the ASIC's 1536-bit Pixel chain through the tail's
// CS2_PASS passthrough and burst-reads it back on the MISO echo, so one pattern
// traverses host -> USB -> cmd_decoder -> spi_cfg_ctrl/spi_master -> tail
// spi_passthrough -> ASIC pins -> back.  On hardware (2026-09-14) a live burst
// read back BIT-EXACT at a stride of 27 bits per read -- 24 data clocks plus the
// 3 trailing clocks a read-mode transaction keeps -- offset 0, MSB first.  This
// bench pins that contract on the RTL with the ASIC model's chain:
//
//   BER-01  a 64-word burst on leg 0 is LIVE: its echo tracks its own writes.
//   BER-02  ...and bit-exact: 0 errors over every aligned read (N reported, and
//           the BER the run demonstrates at 95%: 3/N).
//   BER-03  the stride is 27 bits per read (the tail's 3 trailing clocks reach
//           the ASIC), offset 0 -- what the hardware showed.
//   BER-04  the comparator sees errors: one flipped bit in one read-back word
//           scores exactly one error at the same alignment.
//   BER-05  a burst on leg 3 after leg 0's read-out drained its chain is live
//           and clean too (the write re-seeds the chain; a second leg).
//   BER-06  with the ASIC held in reset (RO_RSTn low) the echo does NOT track
//           the writes -- the case the first hardware run hit by skipping the
//           wake-up, and the reason berTest wakes the ASIC first.
//
// Scoring is the tool's: align each burst by itself over stride {27, 24} and
// offset -4..4, live if >= half the candidate reads match, then popcount the
// XOR over the candidates.  Reads whose 24-bit window would run past the burst
// (57..63 at stride 27) are not candidates.
// ---------------------------------------------------------------------------
`ifdef RUN_BER_LOOPBACK

localparam int          BER_WORDS = 64;                // one chain-full per burst
localparam logic [7:0]  BER_TAIL_CTRL    = 8'h01;
localparam logic [7:0]  BER_TAIL_CS2PASS = 8'h04;      // next transaction -> Pixel chain (SS1)

logic [23:0] ber_sent [0:BER_WORDS-1];
logic [23:0] ber_got  [0:BER_WORDS-1];
logic [22:0] ber_lfsr = 23'h7FFFF;
int          ber_npass = 0, ber_nfail = 0;

// PRBS-23, x^23 + x^18 + 1 (ITU-T O.150) -- the tool's generator, 16 bits a call.
function automatic logic [15:0] ber_next16();
    logic [15:0] w;
    logic        bit_;
    w = 16'h0;
    for (int i = 0; i < 16; i++) begin
        bit_     = ber_lfsr[22] ^ ber_lfsr[17];
        ber_lfsr = {ber_lfsr[21:0], bit_};
        w        = {w[14:0], bit_};
    end
    return w;
endfunction

// The 24-bit window of the burst's write stream starting at bit `bitoff`, or -1
// if it would run past the end.
function automatic int ber_window(input int bitoff);
    logic [23:0] v;
    int          b;
    if (bitoff < 0 || bitoff + 24 > BER_WORDS * 24) return -1;
    v = 24'h0;
    for (int t = 0; t < 24; t++) begin
        b = bitoff + t;
        v = {v[22:0], ber_sent[b / 24][23 - (b % 24)]};
    end
    return int'(v);
endfunction

task automatic ber_cmd(input logic [15:0] flags, input logic [15:0] addr, input logic [15:0] data,
                       output logic [15:0] d);
    logic [15:0] m, f, a;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags, addr, data);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
endtask

// One cfg-SPI transaction on leg `c`: n bytes, write mode (rw=1: exactly 8n
// clocks, no capture -- the ASIC-load rule) or read mode (rw=0: 3 trailing clocks
// kept, MISO captured).  Waits for GO to clear.
task automatic ber_xact(input logic [1:0] c, input logic [2:0] n, input logic [7:0] b0,
                        input logic [7:0] b1, input logic [7:0] b2, input logic rw);
    logic [15:0] d;
    int          polls;
    ber_cmd(flags_wr(), REG_SPI_CFG_DATA, {b0, b1}, d);
    if (n > 3'd2) ber_cmd(flags_wr(), REG_SPI_CFG_DATA2, {b2, 8'h00}, d);
    ber_cmd(flags_wr(), REG_SPI_CFG_CTRL, spi_cfg_ctrl_word(c, rw, 1'b1, n), d);
    polls = 0;
    do begin
        #30_000;                                      // a 3-byte read-mode xact is ~34 us at 800 kHz
        ber_cmd(flags_rd(), REG_SPI_CFG_CTRL, 16'h0, d);
        polls++;
    end while (d[3] === 1'b1 && polls < 500);
    if (d[3] !== 1'b0) $error("[BER] cfg SPI GO never cleared (leg %0d, n=%0d, b0=0x%02h)", c, n, b0);
endtask

task automatic ber_rdata(output logic [23:0] w);
    logic [15:0] d01, d23;
    ber_cmd(flags_rd(), REG_SPI_CFG_RD01, 16'h0, d01);
    ber_cmd(flags_rd(), REG_SPI_CFG_RD23, 16'h0, d23);
    w = {d01[15:8], d01[7:0], d23[15:8]};
endtask

// asicPassthroughWrite: arm in READ mode (the arming needs its trailing clocks),
// then push the 24-bit word in WRITE mode (24 clocks exactly).
task automatic ber_asic_write(input logic [1:0] c, input logic [23:0] w);
    ber_xact(c, 3'd2, BER_TAIL_CS2PASS, 8'h00, 8'h00, 1'b0);
    ber_xact(c, 3'd3, w[23:16], w[15:8], w[7:0], 1'b1);
endtask

// asicPassthroughRead: arm, then a READ-mode zero shift-in that captures the echo.
task automatic ber_asic_read(input logic [1:0] c, output logic [23:0] w);
    ber_xact(c, 3'd2, BER_TAIL_CS2PASS, 8'h00, 8'h00, 1'b0);
    ber_xact(c, 3'd3, 8'h00, 8'h00, 8'h00, 1'b0);
    ber_rdata(w);
endtask

// One burst: fill ber_sent with the next 64 PRBS words, write them all, read 64 back.
task automatic ber_burst(input logic [1:0] c);
    logic [15:0] hi, lo;
    logic [23:0] w;
    for (int i = 0; i < BER_WORDS; i++) begin
        hi = ber_next16(); lo = ber_next16();
        ber_sent[i] = {hi, lo[7:0]};
    end
    for (int i = 0; i < BER_WORDS; i++) ber_asic_write(c, ber_sent[i]);
    // (a temp, not ber_got[i] directly: Icarus does not write a task output back
    // into a variable-indexed array element -- the element stays X)
    for (int i = 0; i < BER_WORDS; i++) begin
        ber_asic_read(c, w);
        ber_got[i] = w;
    end
endtask

// The tool's per-burst aligner: best (stride, offset) by exact-match count.
task automatic ber_align(output int S, output int k, output int hits, output int cand);
    int e, h, n;
    hits = -1; S = 0; k = 0; cand = 0;
    for (int si = 0; si < 2; si++) begin
        for (int kt = -4; kt <= 4; kt++) begin
            h = 0; n = 0;
            for (int j = 0; j < BER_WORDS; j++) begin
                e = ber_window(kt + ((si == 0) ? 27 : 24) * j);
                if (e < 0) continue;
                n++;
                if (e == int'(ber_got[j])) h++;
            end
            if (h > hits) begin hits = h; S = (si == 0) ? 27 : 24; k = kt; cand = n; end
        end
    end
endtask

// Bit errors over the candidate reads at a given alignment.
task automatic ber_score(input int S, input int k, output int bits, output int errs);
    int          e;
    logic [23:0] x;
    bits = 0; errs = 0;
    for (int j = 0; j < BER_WORDS; j++) begin
        e = ber_window(k + S * j);
        if (e < 0) continue;
        x = e[23:0] ^ ber_got[j];          // (not logic'(e): that is a 1-bit cast)
        errs += $countones(x);
        bits += 24;
    end
endtask

task automatic ber_check(input string tag, input bit ok, input string msg);
    if (ok) begin ber_npass++; $display("[%0s] PASS %0s", tag, msg); end
    else    begin ber_nfail++; $display("[%0s] FAIL %0s", tag, msg); end
endtask

task automatic run_BER_LOOPBACK();
    logic [15:0] d;
    int S, k, hits, cand, bits, errs, errs2;

    $display("[BER] BER Test data path in RTL: PRBS-23 burst-written into the ASIC Pixel chain via CS2_PASS, burst-read on the MISO echo");

    // ---- wake the ASICs: what berTest does first ----------------------------
    // 800 kHz cfg clock, MCLK_EN on every tail (the board clock tree is gated by
    // tail #1 alone), RO_RSTn released on the legs under test.
    ber_cmd(flags_wr(), REG_SPI_CLK_DIV, 16'd31, d);
    ber_cmd(flags_wr(), REG_SPI_EN_MASK, 16'h000F, d);
    for (int c = 0; c < 4; c++) ber_xact(c[1:0], 3'd2, BER_TAIL_CTRL, 8'h10, 8'h00, 1'b1);
    ber_xact(2'd0, 3'd2, BER_TAIL_CTRL, 8'h11, 8'h00, 1'b1);
    ber_xact(2'd3, 3'd2, BER_TAIL_CTRL, 8'h11, 8'h00, 1'b1);

    // ---- BER-01..04: leg 0 ---------------------------------------------------
    ber_burst(2'd0);
    ber_align(S, k, hits, cand);
    if (!(cand > 0 && hits >= cand / 2)) begin
        $display("[BER] first 16 write -> read pairs of the leg 0 burst (so the relationship can be worked out from the log):");
        for (int i = 0; i < 16; i++) $display("[BER]   %2d: wrote 0x%06h -> read 0x%06h", i, ber_sent[i], ber_got[i]);
    end
    ber_check("BER-01", cand > 0 && hits >= cand / 2,
              $sformatf("leg 0 burst live: %0d of %0d candidate reads match at stride %0d, offset %0d", hits, cand, S, k));
    ber_score(S, k, bits, errs);
    ber_check("BER-02", bits > 0 && errs == 0,
              $sformatf("leg 0 bit-exact: %0d errors over N=%0d bits (demonstrates BER <= %0.2e at 95%%)", errs, bits, 3.0 / bits));
    ber_check("BER-03", S == 27 && k == 0,
              $sformatf("stride %0d offset %0d (expected 27/0: a read-mode transaction's 3 trailing clocks advance the chain, as on hardware)", S, k));
    ber_got[10] = ber_got[10] ^ 24'h000400;                  // one flipped bit in one read
    ber_score(S, k, bits, errs2);
    ber_check("BER-04", errs2 == errs + 1,
              $sformatf("comparator sees a single flipped bit: %0d -> %0d errors", errs, errs2));
    ber_got[10] = ber_got[10] ^ 24'h000400;

    // ---- BER-05: leg 3, a second burst after a read-out ----------------------
    ber_burst(2'd3);
    ber_align(S, k, hits, cand);
    ber_score(S, k, bits, errs);
    ber_check("BER-05", cand > 0 && hits >= cand / 2 && errs == 0 && S == 27,
              $sformatf("leg 3 burst live and bit-exact: %0d/%0d match, %0d errors over N=%0d, stride %0d offset %0d", hits, cand, errs, bits, S, k));

    // ---- BER-06: ASIC held in reset -> no echo -------------------------------
    ber_xact(2'd0, 3'd2, BER_TAIL_CTRL, 8'h10, 8'h00, 1'b1);   // RO_RSTn low on leg 0
    ber_burst(2'd0);
    ber_align(S, k, hits, cand);
    ber_check("BER-06", !(cand > 0 && hits >= cand / 2),
              $sformatf("ASIC in reset: echo does not track the writes (%0d/%0d match; first read 0x%06h) -- wake the ASIC first", hits, cand, ber_got[0]));
    ber_xact(2'd0, 3'd2, BER_TAIL_CTRL, 8'h11, 8'h00, 1'b1);

    if (ber_nfail == 0) $display("[BER] PASS — %0d checks: the BER Test's loopback is bit-exact through the RTL at stride 27", ber_npass);
    else                $display("[BER] FAIL — %0d check(s) failed (%0d passed)", ber_nfail, ber_npass);
    $display("RESULTS: %0d passed, %0d failed", ber_npass, ber_nfail);
    $display("STATUS: %0s", (ber_nfail == 0) ? "PASS" : "FAIL");
endtask

`endif // RUN_BER_LOOPBACK
