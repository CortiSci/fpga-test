`timescale 1ns / 1ps
// ============================================================================
// tb_underrun.sv — UNIT reproduction of the frame-start / early-slot FIFO
// underrun that shows on hardware as periodic full-scale "dips on all sensors"
// (recording asic_grid_..._091910: leading sample-groups of a lagging leg
// arrive as raw 0x0000 ~every 12-15 frames).
//
// SCOPE: telem_engine_v3 + leg_quad_fifo + crc32 only — no USB, no tails, no
// SPI.  A synthetic fill driver reproduces the exact hardware condition:
//
//   * LEG 0 is kept FULL (refilled whenever it has space) with real non-zero
//     data.  A full armed leg asserts leg_quad_fifo.fifo_full -> the engine's
//     legs_escape, which makes S_TICK_REQ keep ticking even when another leg
//     is below its fill_ge4 pacing threshold.
//   * LEG 1 is fed CONTINUOUSLY with real non-zero data (0xB001, 0xB002, ...)
//     but at a rate slower than the escape-driven drain.  A CORRECT engine
//     paces on leg 1 (waits for its refill) and emits its words contiguously.
//     The current engine drains leg 1 to empty under leg 0's escape and stuffs
//     0x0000 + undf into leg 1's slots.
//
// PASS  = leg 1's data slots contain NO 0x0000 (every sample the driver fed is
//         delivered; nothing is underrun-stuffed).
// FAIL  = any 0x0000 in leg 1's slots (underrun-induced dropout — the "dip").
//
// The driver NEVER feeds 0x0000, so any 0x0000 in a leg-1 slot is unambiguous
// evidence of an underrun stuff, not real data.
// ============================================================================
module tb_underrun;

    // ---- clock / reset -----------------------------------------------------
    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50 MHz-ish (20 ns) — timing is nominal
    reg rst_n = 1'b0;

    // ---- frame_aligned: 1-cycle pulse, periodic (paces forced_start park) --
    reg [8:0] fa_div = 9'd0;
    reg       frame_aligned = 1'b0;
    always @(posedge clk) begin
        if (fa_div == 9'd255) begin fa_div <= 9'd0; frame_aligned <= 1'b1; end
        else                  begin fa_div <= fa_div + 9'd1; frame_aligned <= 1'b0; end
    end

    // ---- engine <-> quad-fifo wiring ---------------------------------------
    wire [3:0]  rd_mask;
    wire        tick_req;
    wire [15:0] tick_data_0, tick_data_1, tick_data_2, tick_data_3;
    wire        tick_valid;
    wire [3:0]  tick_undf;
    wire [3:0]  fill_ge4;
    wire [3:0]  fifo_full, fifo_empty;
    wire [3:0]  fifo_ovf;
    wire [19:0] fifo_word_cnt;

    // ---- engine <-> crc32 wiring -------------------------------------------
    wire        crc_init, crc_valid, crc_last;
    wire [15:0] crc_data;
    wire [31:0] crc_result;
    wire        crc_result_valid;

    // ---- engine TX ---------------------------------------------------------
    wire [15:0] telem_tx_data;
    wire        telem_tx_valid;
    wire        telem_tx_last;
    reg         telem_tx_ready = 1'b1;   // always ready (tx_gap still paces)

    // ---- engine control ----------------------------------------------------
    reg         telem_start = 1'b0;
    reg         sw_reset    = 1'b0;
    reg         run_any     = 1'b0;
    reg  [28:0] ext_frame_cnt = 29'd5;
    reg  [3:0]  legs_armed  = 4'b0011;   // legs 0 (fast) and 1 (slow)

    // ---- fill driver -> quad-fifo write ports ------------------------------
    reg  [15:0] wd_data_0 = 16'h0, wd_data_1 = 16'h0,
                wd_data_2 = 16'h0, wd_data_3 = 16'h0;
    reg  [3:0]  wd_valid  = 4'h0;

    // ------------------------------------------------------------------------
    leg_quad_fifo u_fifo (
        .clk(clk), .rst_n(rst_n), .sync_rst(1'b0),
        .wd_data_0(wd_data_0), .wd_data_1(wd_data_1),
        .wd_data_2(wd_data_2), .wd_data_3(wd_data_3),
        .wd_valid(wd_valid),
        .tick_req(tick_req),
        .rd_mask(rd_mask),
        .tick_data_0(tick_data_0), .tick_data_1(tick_data_1),
        .tick_data_2(tick_data_2), .tick_data_3(tick_data_3),
        .tick_valid(tick_valid), .tick_undf(tick_undf),
        .fill_ge4(fill_ge4),
        .fifo_full(fifo_full), .fifo_empty(fifo_empty),
        .fifo_ovf(fifo_ovf), .fifo_word_cnt(fifo_word_cnt),
        .ch_local_rst(4'h0)
    );

    telem_engine_v3 #(.ESCAPE_TICKS(3)) u_eng (
        .clk(clk), .rst_n(rst_n),
        .telem_start(telem_start), .sw_reset(sw_reset), .run_any(run_any),
        .ext_frame_cnt(ext_frame_cnt), .frame_aligned(frame_aligned),
        .rd_mask(rd_mask), .tick_req(tick_req),
        .tick_data_0(tick_data_0), .tick_data_1(tick_data_1),
        .tick_data_2(tick_data_2), .tick_data_3(tick_data_3),
        .tick_valid(tick_valid), .tick_undf(tick_undf),
        .fill_ge4(fill_ge4),
        .legs_armed(legs_armed),
        .fifo_full_in(fifo_full),
        .crc_init(crc_init), .crc_valid(crc_valid), .crc_last(crc_last),
        .crc_data(crc_data),
        .crc_result(crc_result), .crc_result_valid(crc_result_valid),
        .telem_tx_data(telem_tx_data), .telem_tx_valid(telem_tx_valid),
        .telem_tx_last(telem_tx_last), .telem_tx_ready(telem_tx_ready),
        .par_err_in(4'h0), .ovfl_in(fifo_ovf),
        .framer_busy(), .resync_arm(),
        .w0_dist_0(10'd0), .w0_dist_1(10'd0),
        .w0_dist_2(10'd0), .w0_dist_3(10'd0),
        .leg_fill(fifo_word_cnt), .phase_known(4'h0)
    );

    crc32 u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .valid(crc_valid), .last(crc_last),
        .data_in(crc_data),
        .crc_out(crc_result), .crc_valid(crc_result_valid)
    );

    // ------------------------------------------------------------------------
    // FILL DRIVER
    //   leg 0: refill to full whenever it has space (fast) -> escape source
    //   leg 1: one word every LEG1_PERIOD clocks (slow, continuous)
    // ------------------------------------------------------------------------
    localparam integer LEG1_PERIOD = 40;   // clocks/word (slower than the
                                            // ~16 clk/tick escape-driven drain,
                                            // so leg 1 depletes and underruns)
    reg  [15:0] l0_next = 16'hA001;
    reg  [15:0] l1_next = 16'hB001;
    integer     l1_div  = 0;
    reg         fill_en = 1'b0;

    always @(posedge clk) begin
        wd_valid <= 4'h0;                       // default: no write this cycle
        if (fill_en) begin
            // leg 0 — top up whenever not full
            if (!fifo_full[0]) begin
                wd_data_0 <= l0_next;
                wd_valid[0] <= 1'b1;
                l0_next <= l0_next + 16'h1;
            end
            // leg 1 — steady slow cadence
            if (l1_div == LEG1_PERIOD-1) begin
                l1_div <= 0;
                if (!fifo_full[1]) begin
                    wd_data_1 <= l1_next;
                    wd_valid[1] <= 1'b1;
                    l1_next <= l1_next + 16'h1;
                end
            end else begin
                l1_div <= l1_div + 1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // TX CAPTURE — classify each emitted word by frame position.
    //   words 0..2      : header (tag + 2 counter words)
    //   words 3..4098   : data (tick*4 + leg), leg 1 == (idx-3)%4 == 1
    //   words 4099..4102: phase
    //   words 4103..4104: crc
    // ------------------------------------------------------------------------
    localparam integer HDR = 3, NDATA = 4096, WORDS = 4105;
    integer widx = 0;
    integer leg1_zeros = 0;
    integer leg1_slots = 0;
    integer leg0_zeros = 0;
    reg     frame_done = 1'b0;

    // Frame-start cushion capture: the fill (words buffered) in each leg at the
    // cycle the FIRST data tick fires.  This is the guarantee the fix makes —
    // no frame starts until every armed leg holds >= WATERMARK words, so the
    // startup transient cannot underrun the leading slots.
    localparam integer WATERMARK = 8;
    reg [4:0] first_fill0 = 5'd31, first_fill1 = 5'd31;
    reg       first_tick_seen = 1'b0;
    always @(posedge clk) begin
        if (rst_n && tick_req && !first_tick_seen) begin
            first_tick_seen <= 1'b1;
            first_fill0     <= fifo_word_cnt[4:0];    // leg 0 count (pre-pop)
            first_fill1     <= fifo_word_cnt[9:5];    // leg 1 count (pre-pop)
        end
    end

    always @(posedge clk) begin
        if (rst_n && telem_tx_valid) begin
            if (widx >= HDR && widx < HDR + NDATA) begin
                // data region
                case ((widx - HDR) % 4)
                    1: begin                          // leg 1 slot
                        leg1_slots = leg1_slots + 1;
                        if (telem_tx_data == 16'h0000) leg1_zeros = leg1_zeros + 1;
                    end
                    0: if (telem_tx_data == 16'h0000) leg0_zeros = leg0_zeros + 1;
                    default: ;
                endcase
            end
            widx = widx + 1;
            if (widx == WORDS) frame_done = 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // STIMULUS
    // ------------------------------------------------------------------------
    integer timeout;
    initial begin
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // prime the FIFOs: leg 0 to full, leg 1 to ~5 words (between 4 and the
        // WATERMARK of 8).  Baseline (>=4 start gate) starts the frame with
        // leg 1 still shallow; the fix (>=8) parks until leg 1 fills deeper.
        // fill_en stays high so leg 1 keeps climbing during the fix's park.
        run_any = 1'b1;
        fill_en = 1'b1;
        repeat (210) @(posedge clk);    // leg0 -> full; leg1 -> ~5 words

        // start one telemetry frame
        @(posedge clk);
        telem_start = 1'b1;
        @(posedge clk);
        telem_start = 1'b0;

        // run until the frame completes or we time out
        timeout = 0;
        while (!frame_done && timeout < 2_000_000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        $display("----------------------------------------------------------");
        $display("tb_underrun: frame-start cushion: leg0=%0d leg1=%0d words (WATERMARK=%0d)",
                 first_fill0, first_fill1, WATERMARK);
        $display("             leg1 slots=%0d  leg1 zeros(underrun)=%0d  leg0 zeros=%0d",
                 leg1_slots, leg1_zeros, leg0_zeros);
        if (!frame_done) begin
            $display("[FAIL] UF-01 frame did not complete (timeout) — engine wedged");
            $display("STATUS: FAIL");
        end else if (!first_tick_seen) begin
            $display("[FAIL] UF-01 no data tick observed");
            $display("STATUS: FAIL");
        end else if (first_fill1 < WATERMARK[4:0] || first_fill0 < WATERMARK[4:0]) begin
            $display("[FAIL] UF-01 frame started with a leg below the %0d-word cushion",
                     WATERMARK);
            $display("       (leg0=%0d leg1=%0d) — the startup transient can underrun the",
                     first_fill0, first_fill1);
            $display("       leading slots -> the periodic all-sensor 'dip' seen on hardware.");
            $display("STATUS: FAIL");
        end else begin
            $display("[PASS] UF-01 frame started only after every armed leg held >=%0d words",
                     WATERMARK);
            $display("       (leg0=%0d leg1=%0d); startup transient cannot underrun leading slots.",
                     first_fill0, first_fill1);
            $display("STATUS: PASS");
        end
        $display("----------------------------------------------------------");
        $finish;
    end

endmodule
