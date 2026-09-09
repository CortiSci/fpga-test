`timescale 1ns / 1ps
// ============================================================================
// tb_rotation.sv — sweep-marker anchoring for spi_ch_stream.v
//
// Regression for the "clean ACED but garbage sensor data" failure of
// 2026-09-08/09 (see consolidator_v2/docs/rotation_regression_geoff_green.md).
//
// The failure: a leg's 1024 samples arrive as raw bit-planes and the host
// groups them in 16s to transpose.  If the leg slips a word — line glitch,
// FIFO overflow, a tail restarting mid-sweep — every later group is assembled
// from the wrong 16 planes.  Length, token and CRC all still pass, so nothing
// upstream notices; only the VALUES are wrong.  Under 0xACED the result is a
// rotation of ACED's bits, i.e. a wrong but stable constant, which is why the
// fault reads as "sort of ACED" rather than as noise.
//
// Recovering needs an anchor to the ASIC's own sweep boundary.  spi_ch_stream
// marks it as a word with DELIBERATELY INVERTED parity landing on a group
// boundary:
//   assign marker_here = (state == S_PAR) && (parity_acc ^ miso_r) && (wcnt == 0);
// and exports the scan phase as w0_dist / phase_known.
//
// The adopted geoff-green RTL had NONE of this — no marker_here, no w0_dist,
// no phase_known, no resync_arm/quad_flush; the ports did not exist.  Its
// engine reported a phase word, but that was the engine's own tick_cnt
// (position within the USB frame), which cannot express "this leg is rotated".
// So a rotation, once acquired, was permanent.  This bench does not elaborate
// against such a design, which is the point: it encodes marker anchoring as a
// hard interface requirement rather than a nice-to-have.
//
//   TC-ROT-01  no marker seen yet            -> phase_known = 0
//   TC-ROT-02  bad-parity word on a boundary -> phase_known = 1 (marker taken)
//   TC-ROT-03  w0_dist advances 1 per word   -> scan phase is observable
//   TC-ROT-04  a later marker re-anchors     -> w0_dist returns to 0, so a slip
//                                               self-heals at the next sweep
//                                               instead of persisting forever
//   TC-ROT-05  resync_arm: deliver nothing until the next marker, and flush the
//              leg FIFO at it so the refill starts on plane 0
// ============================================================================
module tb_rotation;

    reg         sclk;
    reg         rst_n;
    reg         miso_in;
    reg         run;
    reg         cfg_hold;
    reg         hunt_tick;
    reg         resync_arm;
    reg         fifo_full;
    reg         pad_slot;

    wire        quad_flush;
    wire [9:0]  w0_dist;
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
    integer     i;
    integer     flush_count;
    integer     wv_before;
    reg  [9:0]  dist_at_word;
    reg  [9:0]  prev_dist;
    reg  [9:0]  dist_before;
    reg         dist_ok;

    spi_ch_stream dut (
        .sclk          (sclk),
        .rst_n         (rst_n),
        .miso_in       (miso_in),
        .run           (run),
        .cfg_hold      (cfg_hold),
        .hunt_tick     (hunt_tick),
        .resync_arm    (resync_arm),
        .fifo_full     (fifo_full),
        .quad_flush    (quad_flush),
        .w0_dist       (w0_dist),
        .phase_known   (phase_known),
        .pad_slot      (pad_slot),
        .sclk_d1       (sclk_d1),
        .mosi_out      (mosi_out),
        .ss_n_out      (ss_n_out),
        .word_data     (word_data),
        .word_valid    (word_valid),
        .raw_word_done (raw_word_done),
        .par_err_flag  (par_err_flag)
    );

    initial sclk = 1'b0;
    always #10 sclk = ~sclk;

    // Sample the reported scan phase on every delivered word.
    always @(posedge sclk) begin
        if (rst_n && word_valid) begin
            wv_count     <= wv_count + 1;
            dist_at_word <= w0_dist;
        end
        if (rst_n && quad_flush) flush_count <= flush_count + 1;
    end

    task arm_dut;
        begin
            @(negedge sclk);
            rst_n = 1'b0; miso_in = 1'b0; run = 1'b1; cfg_hold = 1'b0;
            hunt_tick = 1'b0; resync_arm = 1'b0; fifo_full = 1'b0; pad_slot = 1'b0;
            wv_count = 0; flush_count = 0; dist_at_word = 10'd0;
            repeat (3) @(negedge sclk);
            rst_n = 1'b1;
            repeat (8) @(negedge sclk);
        end
    endtask

    // One serial word: START, 16 data bits MSB-first, parity, idle.
    // good_parity=1 sends even parity (^data); 0 inverts it, which on a group
    // boundary is the sweep marker.
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
        hunt_tick = 1'b0; resync_arm = 1'b0; fifo_full = 1'b0; pad_slot = 1'b0;
        wv_count = 0; flush_count = 0; dist_at_word = 10'd0; dist_ok = 1'b1;

        // -- TC-ROT-01: a fresh stream has no anchor yet ----------------------
        arm_dut;
        send_word(16'hACED, 1'b1);
        send_word(16'hACED, 1'b1);
        check(phase_known === 1'b0, "TC-ROT-01");

        // -- TC-ROT-02: bad parity ON A GROUP BOUNDARY is the sweep marker ----
        // wcnt is 0 immediately after arm, so the first word can be the marker.
        // The acceptance is visible one word later (delivery lags by one).
        arm_dut;
        send_word(16'h1234, 1'b0);          // marker at wcnt==0
        send_word(16'hACED, 1'b1);          // settle
        check(phase_known === 1'b1, "TC-ROT-02");

        // -- TC-ROT-03: scan phase advances monotonically per delivered word --
        // This is the observability geoff-green lacked entirely: without it a
        // slipped leg cannot even be identified, let alone corrected.
        dist_ok  = 1'b1;
        prev_dist = w0_dist;
        for (i = 0; i < 12; i = i + 1) begin
            send_word(16'hACED, 1'b1);
            if (w0_dist !== ((prev_dist + 10'd1) & 10'h3FF)) dist_ok = 1'b0;
            prev_dist = w0_dist;
        end
        check(dist_ok, "TC-ROT-03");

        // -- TC-ROT-04: a marker RE-ANCHORS the scan phase --------------------
        // This is the property that makes a slip self-healing.  Run the stream
        // 16 words with no anchor so the phase counter is well away from zero,
        // then present a marker on the group boundary: the reported phase must
        // collapse to the start of a sweep instead of carrying the accumulated
        // offset forward for ever, which is what made the rotation permanent
        // on hardware.
        arm_dut;
        send_word(16'h1111, 1'b0);          // anchor first: marker at wcnt==0
        send_word(16'hACED, 1'b1);
        // Now run a full group so the phase counter is well past a boundary.
        while (dut.wcnt !== 4'd0) send_word(16'hACED, 1'b1);
        for (i = 0; i < 5; i = i + 1) send_word(16'hACED, 1'b1);
        dist_before = w0_dist;
        while (dut.wcnt !== 4'd0) send_word(16'hACED, 1'b1);
        send_word(16'h4321, 1'b0);          // marker on the boundary
        send_word(16'hACED, 1'b1);          // acceptance visible one word later
        check((dist_before > 10'd8) && (w0_dist < 10'd4) && (phase_known === 1'b1),
              "TC-ROT-04");

        // -- TC-ROT-05: resync_arm discards until the marker, and flushes -----
        // The engine parks after a marred frame and asks for a marker-aligned
        // refill: nothing may be delivered until the next sweep marker, and the
        // leg FIFO must be flushed at it so the refill begins on plane 0.
        arm_dut;
        send_word(16'h1111, 1'b0);          // acquire an anchor
        send_word(16'hACED, 1'b1);
        while (dut.wcnt !== 4'd0) send_word(16'hACED, 1'b1);

        resync_arm  = 1'b1;
        @(negedge sclk);
        wv_before   = wv_count;
        flush_count = 0;
        for (i = 0; i < 8; i = i + 1) send_word(16'h2222, 1'b1);
        check(wv_count === wv_before, "TC-ROT-05a");   // nothing delivered

        while (dut.wcnt !== 4'd0) send_word(16'h2222, 1'b1);
        send_word(16'h3333, 1'b0);                     // resuming marker
        send_word(16'hACED, 1'b1);
        check(flush_count > 0,      "TC-ROT-05b");     // leg FIFO flushed at w0
        check(wv_count > wv_before, "TC-ROT-05c");     // the marker is delivered
        resync_arm = 1'b0;

        $display("");
        $display("tb_rotation: %0d passed, %0d failed", n_pass, n_fail);
        if (n_fail == 0) $display("TB_ROTATION: ALL TESTS PASSED");
        else             $display("TB_ROTATION: FAILURES PRESENT");
        $finish;
    end

    initial begin
        #2000000;
        $display("tb_rotation: TIMEOUT");
        $finish;
    end

endmodule
