`timescale 1ns / 1ps
// ============================================================================
// tb_link_fault.sv — link-integrity detection for spi_ch_stream.v
//
// Regression test for the "enabled leg streaming garbage" failure observed on
// hardware (asic_grid_asic_20260908_1339*.txt): a leg whose SD line is
// floating / unclocked delivers uniform-random bits, which the host decode
// faithfully turns into full-scale uniform noise (std ~= 65536/sqrt(12) counts,
// spanning +/-32767, no coherent structure).
//
// The in-band detector for that condition is the per-word EVEN-PARITY check in
// spi_ch_stream.v (START=1, 16 data bits MSB-first, even parity). This bench
// drives the module directly.  The receiver is MARKER-ANCHORED (2026-09-09):
// it delivers nothing until the tail's sweep marker — a parity-inverted word
// on the 16-word boundary — and that word itself is sweep word 0.  So every
// scenario first anchors the DUT (anchor_dut) and then places its test word
// OFF the boundary, where a bad parity bit is a genuine error, not a marker.
// par_err_flag is a one-cycle pulse, latched here into pe_seen.  It asserts:
//   TC-LF-01  healthy stream (parity = ^data)     -> par_err_flag stays 0
//   TC-LF-02  single bad-parity word              -> par_err_flag asserts
//   TC-LF-03  floating leg (random data + parity) -> par_err trips ~50% of words
//   TC-LF-04  SD stuck HIGH                        -> par_err asserts, data=0xFFFF
//   TC-LF-05  SD stuck LOW                         -> no word_valid (dead-leg)
//
// This is the RTL-suite counterpart of the host-side statistical leg-health
// check.  It gates the release regression (canonical / release_blocking).
// ============================================================================
module tb_link_fault;

    // ---- clock / DUT I/O ----------------------------------------------------
    reg         sclk;
    reg         rst_n;
    reg         miso_in;
    reg         run;
    reg         cfg_hold;

    wire        sclk_d1;
    wire        mosi_out;
    wire        ss_n_out;
    wire [15:0] word_data;
    wire        word_valid;
    wire        raw_word_done;
    wire        par_err_flag;

    // ---- bookkeeping (hoisted per C-10) ------------------------------------
    integer     n_pass;
    integer     n_fail;
    integer     wv_count;      // word_valid pulses since last clear
    reg         pe_seen;       // par_err_flag pulse seen since last clear
    integer     i;
    integer     trip_count;
    reg  [15:0] d;
    reg         pbit;
    reg  [15:0] last_word;

    spi_ch_stream dut (
        .sclk          (sclk),
        .rst_n         (rst_n),
        .miso_in       (miso_in),
        .run           (run),
        .cfg_hold      (cfg_hold),
        .sclk_d1       (sclk_d1),
        .mosi_out      (mosi_out),
        .ss_n_out      (ss_n_out),
        .word_data     (word_data),
        .word_valid    (word_valid),
        .raw_word_done (raw_word_done),
        .par_err_flag  (par_err_flag)
    );

    // 51.2 MHz-ish; period 20 ns (rate is irrelevant to the logic under test)
    initial sclk = 1'b0;
    always #10 sclk = ~sclk;

    // Count word_valid pulses (each is one received serial word)
    always @(posedge sclk) begin
        if (rst_n && word_valid) begin
            wv_count  <= wv_count + 1;
            last_word <= word_data;
        end
    end
    always @(posedge sclk) if (rst_n && par_err_flag) pe_seen <= 1'b1;

    // ---- helpers -----------------------------------------------------------
    // Reset the DUT and leave it armed in S_WAIT, ready for a START bit.
    task arm_dut;
        begin
            @(negedge sclk); rst_n = 1'b0; miso_in = 1'b0; run = 1'b1; cfg_hold = 1'b0;
            wv_count = 0; last_word = 16'h0000; pe_seen = 1'b0;
            repeat (3) @(negedge sclk);
            rst_n = 1'b1;
            // S_IDLE -> S_ARM (3) -> S_WAIT: settle margin
            repeat (8) @(negedge sclk);
        end
    endtask

    // Arm, then deliver the sweep marker (0x0000 has even parity 0; sending 1
    // inverts it on the boundary) so the receiver anchors and delivers.  Clear
    // the counters after it so each TC counts only its own words; the next
    // word lands at wcnt==1.
    task anchor_dut;
        begin
            arm_dut;
            send_word(16'h0000, 1'b1);
            repeat (3) @(negedge sclk);
            wv_count = 0; last_word = 16'h0000; pe_seen = 1'b0;
        end
    endtask

    // Drive one serial word: START, 16 data bits MSB-first, one parity bit,
    // then one idle-low cycle.  miso is set on negedge so it is stable at the
    // posedge the DUT samples on.
    task send_word(input [15:0] data, input parity_bit);
        integer k;
        begin
            @(negedge sclk); miso_in = 1'b1;                 // START
            for (k = 15; k >= 0; k = k - 1) begin
                @(negedge sclk); miso_in = data[k];          // data MSB->LSB
            end
            @(negedge sclk); miso_in = parity_bit;           // parity
            @(negedge sclk); miso_in = 1'b0;                 // idle
            @(negedge sclk);                                 // let S_PAR/S_WAIT settle
        end
    endtask

    task check(input cond, input [511:0] tag);
        begin
            if (cond) begin n_pass = n_pass + 1; $display("[%0s] PASS: ok", tag); end
            else      begin n_fail = n_fail + 1; $display("[%0s] FAIL: assertion failed", tag); end
        end
    endtask

    // ---- test sequence -----------------------------------------------------
    initial begin
        n_pass = 0; n_fail = 0;
        miso_in = 1'b0; run = 1'b0; cfg_hold = 1'b0; rst_n = 1'b1;
        wv_count = 0; last_word = 16'h0000;

        // -- TC-LF-01: healthy stream, correct even parity -> no par_err ------
        anchor_dut;
        for (i = 0; i < 32; i = i + 1) begin
            d    = i[15:0] ^ 16'hA5C3;      // varied, deterministic data
            pbit = ^d;                      // even parity: parity = XOR of data
            send_word(d, pbit);
        end
        check(pe_seen === 1'b0,      "TC-LF-01-parity-clean");
        check(wv_count == 32,        "TC-LF-01-all-words-received");
        check(last_word == ((31 ^ 16'hA5C3)), "TC-LF-01-data-integrity");

        // -- TC-LF-02: one word with WRONG parity (off the boundary) -> asserts
        anchor_dut;
        d = 16'h1234; send_word(d, ~(^d));  // deliberately inverted parity
        check(pe_seen === 1'b1,      "TC-LF-02-bad-parity-detected");

        // -- TC-LF-03: floating leg (random data + random parity) -------------
        // Faithful model of the hardware failure. Each word is reset-isolated
        // so we can measure the trip RATE; a floating leg trips ~50% of words.
        trip_count = 0;
        for (i = 0; i < 64; i = i + 1) begin
            anchor_dut;
            d    = $random;
            pbit = $random;                 // parity independent of data
            send_word(d, pbit);
            if (pe_seen) trip_count = trip_count + 1;
        end
        $display("[TC-LF-03] floating-leg parity trips: %0d / 64", trip_count);
        // Healthy = 0; floating trips near 32. Loose bound never flakes but is
        // unmistakably separated from a clean link.
        check(trip_count >= 16, "TC-LF-03-floating-leg-detected");

        // -- TC-LF-04: SD stuck HIGH -> START seen, data=0xFFFF, par_err ------
        anchor_dut;
        @(negedge sclk); miso_in = 1'b1;    // hold high forever
        repeat (24) @(negedge sclk);        // START + 16 data(=1) + parity(=1) + latch margin
        check(word_data === 16'hFFFF, "TC-LF-04-stuck-high-data");
        check(pe_seen === 1'b1,       "TC-LF-04-stuck-high-detected");

        // -- TC-LF-05: SD stuck LOW -> no START, dead leg (no word_valid) -----
        arm_dut;
        @(negedge sclk); miso_in = 1'b0;    // hold low forever
        repeat (40) @(negedge sclk);
        check(wv_count == 0,          "TC-LF-05-stuck-low-no-data");
        check(pe_seen === 1'b0,       "TC-LF-05-stuck-low-no-false-parity");

        // ---- verdict --------------------------------------------------------
        $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
        if (n_fail == 0) $display("STATUS: PASS");
        else             $display("STATUS: FAIL");
        $finish;
    end

    // safety timeout
    initial begin
        #2000000;
        $display("FAIL: tb_link_fault timeout");
        $display("STATUS: FAIL");
        $finish;
    end

endmodule
