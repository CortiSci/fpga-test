`timescale 1ns / 1ps
// ============================================================================
// tb_rotation.sv — sweep-marker anchoring for spi_ch_stream.v
//
// Regression for the "clean ACED but garbage sensor data" failure of
// 2026-09-08/09 (see consolidator_v2/docs/rotation_regression_geoff_green.md).
//
// The failure: a leg's 1024 samples arrive as raw bit-planes and the host
// groups them in 16s to transpose.  If the leg comes up mid-sweep, or slips a
// word, every later group is assembled from the wrong 16 planes.  Length,
// token and CRC all still pass, so nothing upstream notices; only the VALUES
// are wrong.  Under 0xACED the result is a rotation of ACED's bits, i.e. a
// wrong but stable constant, which is why the fault reads as "sort of ACED"
// rather than as noise.
//
// Recovering needs an anchor to the ASIC's own sweep boundary.  The tail
// transmits sweep word 0 with DELIBERATELY INVERTED parity; spi_ch_stream
// recognises a parity mismatch landing on a 16-word group boundary as that
// marker (not as an error), delivers NOTHING until it has seen one, and
// delivers the marker word itself as sweep word 0:
//   marker_here = (state == S_PAR) && par_bad && (wcnt == 0);
//   word_valid  <= anchored | marker_here;
// and exports `phase_known` (= anchored).
//
// History.  The 72606e4 (geoff) receiver had none of this: a leg's first
// START became frame slot 0 wherever it landed, and the marker's inverted
// parity tripped its forever-sticky par_err_flag every sweep.  The con_phase
// receiver had anchoring plus a great deal more (w0_dist scan-phase export,
// resync_arm/quad_flush re-anchor after overflow, pad/re-arm on parity error,
// hunt timescales) at a cost that did not fit the LCMXO2-2000HC.  On
// 2026-09-09 the anchoring alone was ported onto the 72606e4 base
// (1029/1056 SLICEs); this bench encodes THAT contract.  The retired TCs
// (w0_dist monotonic, resync flush) are listed at the end so nobody mistakes
// their absence for an oversight.
//
//   TC-ROT-01  no marker seen yet            -> phase_known = 0 AND nothing
//                                               delivered (good words are held)
//   TC-ROT-02  bad-parity word on a boundary -> phase_known = 1, the marker
//                                               word IS delivered, and it is
//                                               NOT flagged as a parity error
//   TC-ROT-03  once anchored, every word is delivered and wcnt tracks the
//              group position mod 16 (the host's 16-plane lattice)
//   TC-ROT-04  bad parity OFF a boundary is a real parity error: one flag
//              pulse, the word still delivered, the anchor kept
//   TC-ROT-05  a stop drops the anchor; the next start delivers nothing until
//              the tail's next marker (the tail restarts at a sweep boundary)
//
//   Not tested (not in this receiver): w0_dist scan-phase export; the
//   resync_arm / quad_flush re-anchor after a FIFO overflow.
// ============================================================================
module tb_rotation;

    reg         sclk;
    reg         rst_n;
    reg         miso_in;
    reg         run;
    reg         cfg_hold;
    wire        phase_known;
    wire        sclk_d1;
    wire        mosi_out;
    wire        ss_n_out;
    wire [15:0] word_data;
    wire        word_valid;
    wire        raw_word_done;
    wire        par_err_flag;

    // Declarations hoisted to the top of the scope (toolchain constraint C-10).
    integer     n_pass;
    integer     n_fail;
    integer     wv_count;
    integer     pe_count;
    integer     wv_before;
    integer     pe_before;
    integer     i;
    reg         lattice_ok;

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
        .par_err_flag  (par_err_flag),
        .phase_known   (phase_known)
    );

    initial sclk = 1'b0;
    always #10 sclk = ~sclk;

    // Count deliveries and parity-error pulses.
    always @(posedge sclk) begin
        if (rst_n && word_valid)   wv_count <= wv_count + 1;
        if (rst_n && par_err_flag) pe_count <= pe_count + 1;
    end

    task arm_dut;
        begin
            @(negedge sclk);
            rst_n = 1'b0; miso_in = 1'b0; run = 1'b1; cfg_hold = 1'b0;
            wv_count = 0; pe_count = 0;
            repeat (3) @(negedge sclk);
            rst_n = 1'b1;
            repeat (8) @(negedge sclk);
        end
    endtask

    // One serial word: START, 16 data bits MSB-first, parity, idle.
    // good_parity=1 sends even parity (^data); 0 inverts it, which on a group
    // boundary is the sweep marker and off it is a parity error.
    task send_word(input [15:0] data, input good_parity);
        integer k;
        reg pbit;
        begin
            pbit = good_parity ? (^data) : ~(^data);
            @(negedge sclk); miso_in = 1'b1;
            for (k = 15; k >= 0; k = k - 1) begin
                @(negedge sclk); miso_in = data[k];
            end
            @(negedge sclk); miso_in = pbit;
            @(negedge sclk); miso_in = 1'b0;
            @(negedge sclk);
        end
    endtask

    task check(input cond, input [511:0] tag);
        begin
            if (cond) begin n_pass = n_pass + 1; $display("[%0s] PASS", tag); end
            else      begin n_fail = n_fail + 1; $display("[%0s] FAIL", tag); end
        end
    endtask

    initial begin
        n_pass = 0; n_fail = 0;
        miso_in = 1'b0; run = 1'b0; cfg_hold = 1'b0; rst_n = 1'b1;
        wv_count = 0; pe_count = 0; lattice_ok = 1'b1;

        // -- TC-ROT-01: a fresh stream has no anchor and delivers nothing ------
        // Good-parity words before any marker would be mid-sweep planes of
        // unknown position; delivering them is exactly how a leg comes up
        // rotated.  They must be held back.
        arm_dut;
        send_word(16'hACED, 1'b1);
        send_word(16'hACED, 1'b1);
        send_word(16'hACED, 1'b1);
        check(phase_known === 1'b0, "TC-ROT-01a");
        check(wv_count === 0,       "TC-ROT-01b");
        check(pe_count === 0,       "TC-ROT-01c");   // good parity is not an error

        // -- TC-ROT-02: bad parity ON A GROUP BOUNDARY is the sweep marker ----
        // wcnt is 0 immediately after arm, so the first word can be the marker.
        // It is delivered (it IS sweep word 0) and is not a parity error.
        arm_dut;
        send_word(16'h1234, 1'b0);          // marker at wcnt==0
        send_word(16'hACED, 1'b1);          // settle: first post-anchor word
        check(phase_known === 1'b1, "TC-ROT-02a");
        check(wv_count === 2,       "TC-ROT-02b");   // marker + one word delivered
        check(pe_count === 0,       "TC-ROT-02c");   // marker is not an error

        // -- TC-ROT-03: anchored delivery keeps the 16-word lattice ------------
        // Every word is delivered and wcnt counts the position within the
        // host's 16-plane group; a full group returns wcnt to 0.
        wv_before  = wv_count;
        lattice_ok = 1'b1;
        for (i = 0; i < 14; i = i + 1) begin      // 2 delivered so far -> 16 total
            send_word(16'hACED, 1'b1);
            if (dut.wcnt !== ((2 + i + 1) % 16)) lattice_ok = 1'b0;
        end
        check(wv_count === wv_before + 14, "TC-ROT-03a");
        check(lattice_ok && (dut.wcnt === 4'd0), "TC-ROT-03b");

        // -- TC-ROT-04: bad parity OFF the boundary is a parity error ----------
        // The word is still delivered (the engine flags the frame via the
        // phase word's bit[14]); the anchor is kept; exactly one pulse.
        send_word(16'hACED, 1'b1);          // wcnt -> 1: off the boundary now
        pe_before = pe_count; wv_before = wv_count;
        send_word(16'h5555, 1'b0);          // parity error at wcnt==1
        send_word(16'hACED, 1'b1);
        check(pe_count === pe_before + 1,   "TC-ROT-04a");
        check(wv_count === wv_before + 2,   "TC-ROT-04b");
        check(phase_known === 1'b1,         "TC-ROT-04c");

        // -- TC-ROT-05: a stop drops the anchor; restart waits for a marker ----
        // The tail restarts its stream at a sweep boundary after a stop, so the
        // receiver must not trust its old alignment.
        @(negedge sclk); run = 1'b0;
        repeat (4) @(negedge sclk);
        check(phase_known === 1'b0, "TC-ROT-05a");
        run = 1'b1;
        repeat (8) @(negedge sclk);         // S_ARM -> S_WAIT
        wv_before = wv_count;
        send_word(16'hACED, 1'b1);          // good words before the marker: held
        send_word(16'hACED, 1'b1);
        check(wv_count === wv_before,       "TC-ROT-05b");
        // The tail's w0 lands on the receiver's boundary (the receiver counted
        // the held words too, so bring wcnt back to 0 first).
        while (dut.wcnt !== 4'd0) send_word(16'hACED, 1'b1);
        send_word(16'h4321, 1'b0);          // marker
        send_word(16'hACED, 1'b1);
        check(phase_known === 1'b1 && wv_count === wv_before + 2, "TC-ROT-05c");

        $display("");
        $display("tb_rotation: %0d passed, %0d failed", n_pass, n_fail);
        if (n_fail == 0) $display("TB_ROTATION: ALL TESTS PASSED");
        else             $display("TB_ROTATION: FAILURES PRESENT");
        // Result-contract sentinel (result_contract.py): an explicit STATUS
        // line is what lets a runner score this bench without the legacy
        // "[tag] PASS evidence" fallback.
        $display("RESULTS: %0d passed, %0d failed", n_pass, n_fail);
        $display("STATUS: %0s", (n_fail == 0) ? "PASS" : "FAIL");
        $finish;
    end

    initial begin
        #2000000;
        $display("tb_rotation: TIMEOUT");
        $finish;
    end
endmodule
