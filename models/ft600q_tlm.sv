// FT600Q 245 Synchronous FIFO Transaction-Level Model
// Simulates the FTDI FT600Q from the host PC perspective at 66 MHz.
// Two independent state machines:
//   rx_fsm — delivers words from the testbench to the FPGA (host→FPGA, FPGA reads)
//   tx_fsm — captures words the FPGA writes (FPGA→host)
//
// Frame demultiplexing: the model self-demultiplexes by inspecting the first word
// of each frame — exactly as the real USB host driver does.  No external hint needed.
//   ctrl frame  : word[0] == 16'h55AA (RSP_MAGIC), total 4 words
//   telem frame : word[0] != 16'h55AA,              total 37 words
//
// Protocol guarantee: cmd_decoder waits in ST_TX_WAIT until framer_busy=0 before
// transmitting any ctrl response.  This guarantees ctrl frames only appear between
// complete telem frames in the byte stream — 0x55AA is unambiguous at position 0.
//
// Task API (callable from test sequences):
//   send_word(word)
//   send_command_frame(magic, flags, addr, data)
//   wait_response_frame(out magic, flags, addr, data)
//   wait_telemetry_frame(out hdr, payload[32], token[4])
//   wait_response_frame_typed(out magic, flags, addr, data)   -- ctrl words only
//   wait_telemetry_frame_typed(out hdr, payload[32], token[4]) -- telem words only
//   set_txe_backpressure(ena)
//   flush_tx_capture(out words[], out n)
//   --- §2 additions ---
//   set_txe_random_backpressure(enable, probability_ppm, min_cycles, max_cycles)
//   set_txe_stress_preset()
//   get_bp_stats(out events, out cycles, out words_stalled)
//   set_rxf_packet_gap(words_per_packet, gap_cycles)
//   get_overflow_drop_count(out dropped)
//   inject_payload_bit_error(count, rate_ppm)
//   read_transfer(max_words, gap_cycles, out words[], out n)
`timescale 1ns/1ps

module ft600q_tlm #(
    parameter int TELEM_FRAME_LEN = 37
) (
    input  wire        clk_66m,
    input  wire        rst_n,

    // Physical FT600Q ↔ FPGA signals
    inout  wire [15:0] usb_fifo_d,
    inout  wire [1:0]  usb_fifo_be,
    output reg         rxf_n,        // 0 = host has data for FPGA to read
    output reg         txe_n,        // 0 = host has space for FPGA to write
    input  wire        rd_n,         // FPGA drives low to read
    input  wire        wr_n,         // FPGA drives low to write
    input  wire        oe_n          // FPGA drives low to take bus ownership
);

    // -------------------------------------------------------------------------
    // RX queue: words the testbench wants the FPGA to receive (host → FPGA)
    // -------------------------------------------------------------------------
    reg [15:0] rx_queue [0:255];
    int        rx_wr_ptr = 0;
    int        rx_rd_ptr = 0;
    function automatic int rx_count();
        return rx_wr_ptr - rx_rd_ptr;
    endfunction

    // -------------------------------------------------------------------------
    // Frame demux constants — declared before arrays that use CAPN in dimensions
    // -------------------------------------------------------------------------
    localparam logic [15:0] RSP_MAGIC_VAL  = 16'h55AA;
    localparam int          CTRL_FRAME_LEN = 4;
    // Telemetry frame length: 37 for the V1/V2 37-word format (default), 4103
    // for the V3 format (header + 4096 data + 4 phase + 2 CRC).  Overridden by
    // the consolidator_v2 testbench; production testbenches keep the default.
    // Capture ring.  16384 held <4 V3 frames: a bench that issues a USB command
    // WHILE streaming waits ~1 frame per response (cmd_decoder holds it to the
    // inter-frame gap), so one SPI transaction let ~6 frames arrive, the ring
    // wrapped, and every frame read afterwards was misaligned (2026-09-09,
    // test_host_xact).  65536 = 15 frames; the typed readers now $fatal on overrun.
    localparam int          CAPN = 131072;  // 31 V3 frames

    // -------------------------------------------------------------------------
    // TX capture: all words written by the FPGA (flat, for flush_tx_capture)
    // -------------------------------------------------------------------------
    reg [15:0] tx_capture [0:CAPN-1];
    int        tx_wr_ptr = 0;
    int        tx_rd_ptr = 0;

    // -------------------------------------------------------------------------
    // Typed TX captures — separate FIFO per frame type so interleaved ctrl and
    // telem words can be drained independently with the _typed tasks.
    //   ctrl_capture : cmd_decoder responses  (word[0] == RSP_MAGIC 0x55AA)
    //   telem_capture: telemetry_framer words (word[0] != RSP_MAGIC)
    // -------------------------------------------------------------------------
    reg [15:0] ctrl_capture  [0:CAPN-1];
    int        ctrl_wr_ptr  = 0;
    int        ctrl_rd_ptr  = 0;

    reg [15:0] telem_capture [0:CAPN-1];
    int        telem_wr_ptr = 0;
    int        telem_rd_ptr = 0;

`ifdef ICARUS
    // C-05: Icarus cannot pass unpacked arrays as output task parameters.
    // Module-level result buffers; callers read via hierarchical reference.
    reg [15:0] v3_data  [0:4095];
    reg [15:0] v3_phase [0:3];
    reg [15:0] v3_crc   [0:1];
    reg [15:0] v3_count_lo, v3_count_hi;
`endif

    // -------------------------------------------------------------------------
    // TX frame demux state
    // Self-demultiplexes the interleaved ctrl/telem stream by inspecting word[0]
    // of each frame, identical to what the real USB host driver does.
    // -------------------------------------------------------------------------
    int  tx_frame_pos    = 0;   // word index within the current frame (0 = start)
    reg  tx_frame_is_ctrl = 0;  // 1 = current frame is ctrl (4-word), 0 = telem (37-word)

    // -------------------------------------------------------------------------
    // Backpressure control (§2.1 / §2.3)
    // -------------------------------------------------------------------------
    reg bp_force     = 1'b0;  // when 1, TXE_N is forced high (manual)
    reg dbg_tx_trace = 1'b0;  // when 1, log every captured TX word
    bit  dbg_frame_trace = 1'b0;  // enable_frame_trace(): one line per frame boundary

    // Random backpressure injection state (§2.1)
    reg         bp_rand_enable      = 1'b0;
    int         bp_rand_prob_ppm    = 0;      // Bernoulli trial probability per cycle
    int         bp_rand_min_cycles  = 5;
    int         bp_rand_max_cycles  = 200;
    int         bp_rand_remain      = 0;      // cycles remaining in current burst
    // Combined: TXE_N high when bp_force OR bp_rand_remain > 0
    wire        txe_high_any;
    assign      txe_high_any = bp_force | (bp_rand_remain > 0);

    // Backpressure statistics (§2.1)
    int         bp_stat_events       = 0;
    int         bp_stat_cycles       = 0;
    int         bp_stat_words_stalled = 0;
    // TXE_N write slack (2026-09-10).  The FPGA registers WR_N from the TXE_N it
    // sampled a cycle earlier, so relative to the moment this model's buffer
    // fills it can present up to two more words.  Hardware evidence says the
    // real FT600 accepts them: 6,527 frames under known ~2 ms host read gaps
    // (asic_grid_asic_20260910_101002) with crcbad=0 — its skip=9 were
    // frame-counter gaps from the stalled engine, not lost words.  Words written
    // while the full condition has stood for TXE_WRITE_SLACK or more cycles are
    // dropped (that IS an FPGA defect); the first two are accepted as the device does.
    localparam int TXE_WRITE_SLACK = 2;
    int         txe_hi_cycles = 0;   // cycles the buffer-full condition has been asserted
    // No-puncture monitor (telemetry_v3 ISSUE 3): a 0x55AA response magic seen
    // at a NON-ZERO telemetry frame position means a ctrl response was inserted
    // inside a telemetry frame.  The typed capture is position-based, so one
    // puncture shifts every later frame by 4 words (payload lands in the phase
    // slots) — and a real host parser desyncs the same way.
    int         puncture_count     = 0;
    int         puncture_first_pos = -1;

    // Overflow drop counter (§2.3)
    int         overflow_drop_count  = 0;

    // -------------------------------------------------------------------------
    // RXF_N inter-packet gap state (§2.2)
    // -------------------------------------------------------------------------
    int         rxf_words_per_pkt    = 0;    // 0 = disabled (continuous)
    int         rxf_gap_cycles       = 12;
    int         rxf_words_delivered  = 0;    // words delivered since last gap
    int         rxf_gap_remain       = 0;    // gap cycles remaining

    // -------------------------------------------------------------------------
    // Bit-error injection state (§2.4)
    // -------------------------------------------------------------------------
    int         bit_err_count        = 0;    // remaining frames to corrupt
    int         bit_err_rate_ppm     = 0;    // per-frame probability (0 = one-shot)
    // Position tracking for payload corruption (words[1..32] of telem frame)
    int         bit_err_frame_word   = 0;    // current word index in active telem frame
    reg         bit_err_pending      = 1'b0; // set when next telem frame should be corrupted
    int         bit_err_target_word  = 0;    // which payload word to flip (1–32)

    // -------------------------------------------------------------------------
    // Bus drivers
    // -------------------------------------------------------------------------
    reg         drive_d;
    reg  [15:0] d_drive;
    reg  [1:0]  be_drive;

    assign usb_fifo_d  = drive_d                   ? d_drive  : 16'hzzzz;
    assign usb_fifo_be = drive_d ? be_drive : 2'bzz;

    // -------------------------------------------------------------------------
    // RX FSM — delivers words from rx_queue when FPGA drives OE_N low then RD_N low
    // §2.2: inter-packet gap — RXF_N deasserts for rxf_gap_cycles after every
    //       rxf_words_per_pkt words when rxf_words_per_pkt > 0.
    // -------------------------------------------------------------------------
    always @(posedge clk_66m or negedge rst_n) begin
        if (!rst_n) begin
            rxf_n            <= 1'b1;
            drive_d          <= 1'b0;
            d_drive          <= 16'h0;
            be_drive         <= 2'b11;
            rxf_words_delivered <= 0;
            rxf_gap_remain   <= 0;
        end else begin
            // Inter-packet gap countdown
            if (rxf_gap_remain > 0) begin
                rxf_gap_remain <= rxf_gap_remain - 1;
                rxf_n  <= 1'b1;  // hold high during gap
                drive_d <= 1'b0;
            end else begin
                // Look-ahead: assert RXF_N based on count *after* this cycle's
                // dequeue, not before.  The real FT600Q asserts RXF_N=1 in the
                // same clock period as the last RD_N dequeue — there is no delta
                // in hardware.  Without this, the NBA here makes it visible to the
                // FPGA one cycle too late, masking the READ_POST capture bug in sim.
                begin : rxf_count_la
                    int count_after;
                    count_after = rx_count() -
                                  ((!oe_n && !rd_n && rx_count() > 0) ? 1 : 0);
                    rxf_n <= (count_after == 0) ? 1'b1 : 1'b0;
                end

                if (!oe_n && !rd_n && rx_count() > 0) begin
                    // RD phase: advance pointer and immediately load the NEXT
                    // word into d_drive so usb_d_pipe (1-cycle pipeline in the
                    // FPGA) sees each word exactly one cycle after it is needed.
                    // Real FT600Q registered output advances on the same clock
                    // edge as the dequeue; NBA semantics require showing the
                    // next word now so the FPGA reads the correct one next cycle.
                    rx_rd_ptr <= rx_rd_ptr + 1;
                    d_drive   <= (rx_count() > 1)
                                    ? rx_queue[(rx_rd_ptr[7:0] + 1'b1)]
                                    : 16'h0;
                    be_drive  <= 2'b11;
                    drive_d   <= 1'b1;

                    // §2.2: track words delivered; start gap when packet boundary reached
                    if (rxf_words_per_pkt > 0) begin
                        if (rxf_words_delivered + 1 >= rxf_words_per_pkt) begin
                            rxf_words_delivered <= 0;
                            rxf_gap_remain      <= rxf_gap_cycles;
                        end else begin
                            rxf_words_delivered <= rxf_words_delivered + 1;
                        end
                    end
                end else if (!oe_n && rd_n && rx_count() > 0) begin
                    // OE phase: pre-drive the head word without consuming it.
                    // Real FT600Q places D[0] on the bus as soon as OE_N asserts
                    // (before RD_N goes low), so the FPGA's usb_d_pipe register
                    // captures a valid first word rather than Z.
                    d_drive  <= rx_queue[rx_rd_ptr[7:0]];
                    be_drive <= 2'b11;
                    drive_d  <= 1'b1;
                end else if (oe_n) begin
                    drive_d <= 1'b0;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Random backpressure burst generator (§2.1)
    // Evaluated each cycle while the FPGA is writing.  A Bernoulli trial with
    // probability bp_rand_prob_ppm/1_000_000 starts a burst of random length
    // [bp_rand_min_cycles, bp_rand_max_cycles].  The burst counter then counts
    // down independently, keeping TXE_N high for the entire duration.
    // -------------------------------------------------------------------------
    always @(posedge clk_66m or negedge rst_n) begin
        if (!rst_n) begin
            bp_rand_remain <= 0;
        end else begin
            if (bp_rand_remain > 0) begin
                bp_rand_remain      <= bp_rand_remain - 1;
                bp_stat_cycles      <= bp_stat_cycles + 1;
                // Count words the FPGA tried to write while stalled
                if (!wr_n) bp_stat_words_stalled <= bp_stat_words_stalled + 1;
            end else if (bp_rand_enable && !wr_n) begin
                // FPGA is writing — evaluate Bernoulli trial
                if (($urandom % 1_000_000) < bp_rand_prob_ppm) begin
                    int burst_len;
                    burst_len = bp_rand_min_cycles +
                                ($urandom % (bp_rand_max_cycles - bp_rand_min_cycles + 1));
                    bp_rand_remain <= burst_len;
                    bp_stat_events <= bp_stat_events + 1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // TX FSM — captures words when FPGA drives WR_N low and TXE_N is low.
    // §2.3: words arriving while txe_n=1 are DROPPED and counted (overflow model).
    // §2.4: bit-error injection flips one bit in a payload word of telem frames.
    // Self-demultiplexes ctrl vs telem by inspecting word[0] of each frame.
    // -------------------------------------------------------------------------
    always @(posedge clk_66m or negedge rst_n) begin
        if (!rst_n) begin
            txe_n            <= 1'b0;
            tx_wr_ptr        <= 0;
            tx_frame_pos     <= 0;
            tx_frame_is_ctrl <= 0;
            overflow_drop_count <= 0;
            bit_err_frame_word  <= 0;
            bit_err_pending     <= 1'b0;
        end else begin
            txe_n <= txe_high_any ? 1'b1 : 1'b0;
            txe_hi_cycles <= txe_high_any ? txe_hi_cycles + 1 : 0;

            if (!wr_n) begin
                if (txe_high_any && txe_hi_cycles >= TXE_WRITE_SLACK) begin
                    // §2.3 — overflow: FPGA wrote while buffer full, drop the word
                    overflow_drop_count <= overflow_drop_count + 1;
                    $display("[FT600Q TLM] OVERFLOW: word dropped at t=%0t (txe_n=1, wr_n=0), drop_count=%0d",
                             $time, overflow_drop_count + 1);
                end else begin
                    // Normal accepted write
                    if (usb_fifo_be !== 2'b11)
                        $display("[FT600Q TLM] WARNING: BE=0b%02b at time %0t (expected 2'b11)",
                                 usb_fifo_be, $time);

                    // §2.4 bit-error: determine if this word should be corrupted.
                    // Only corrupt payload words (positions 1..32) of telem frames.
                    begin : bit_err_apply
                        logic [15:0] captured_word;
                        logic        is_telem_payload;
                        captured_word = usb_fifo_d;
                        is_telem_payload = (tx_frame_pos == 0) ?
                                           (usb_fifo_d != RSP_MAGIC_VAL) : !tx_frame_is_ctrl;

                        if (bit_err_pending && is_telem_payload &&
                            !tx_frame_is_ctrl && tx_frame_pos == bit_err_target_word) begin
                            // Flip a random bit in this word
                            captured_word = captured_word ^ (16'h0001 << ($urandom % 16));
                            bit_err_pending <= 1'b0;
                            if (bit_err_rate_ppm == 0) begin
                                // One-shot: decrement remaining count; arm next if any
                                if (bit_err_count > 1) begin
                                    bit_err_count  <= bit_err_count - 1;
                                    bit_err_pending <= 1'b1;
                                    bit_err_target_word <= 1 + ($urandom % 32);
                                end else begin
                                    bit_err_count <= 0;
                                end
                            end
                        end

                        // Flat capture
                        tx_capture[tx_wr_ptr % CAPN] <= captured_word;
                        tx_wr_ptr <= tx_wr_ptr + 1;

                        // No-puncture monitor (see puncture_count decl)
                        if (tx_frame_pos != 0 && !tx_frame_is_ctrl && captured_word == RSP_MAGIC_VAL) begin
                            puncture_count <= puncture_count + 1;
                            if (puncture_first_pos < 0) puncture_first_pos <= tx_frame_pos;
                            if (puncture_count < 3)
                                $display("[FT600Q TLM] PUNCTURE: response magic 0x55AA at telemetry frame position %0d t=%0t",
                                         tx_frame_pos, $time);
                        end

                        // Frame-boundary trace (enable_frame_trace): one line per frame
                        // start, plus a MISFRAME line when a frame typed as telemetry
                        // does not begin with the 0x0001 tag.
                        if (dbg_frame_trace && tx_frame_pos == 0) begin
                            $display("[FT600Q TLM] frame start t=%0t type=%s word0=0x%04h (telem_wr=%0d ctrl_wr=%0d)",
                                     $time, (captured_word == RSP_MAGIC_VAL) ? "CTRL" : "TELEM", captured_word,
                                     telem_wr_ptr, ctrl_wr_ptr);
                            if (captured_word != RSP_MAGIC_VAL && captured_word != 16'h0001)
                                $display("[FT600Q TLM] MISFRAME: telemetry-typed frame begins with 0x%04h, not the 0x0001 tag", captured_word);
                        end

                        // Frame-start: determine type
                        if (tx_frame_pos == 0) begin
                            tx_frame_is_ctrl <= (captured_word == RSP_MAGIC_VAL) ? 1'b1 : 1'b0;
                            // §2.4: arm bit-error for next telem frame start if random mode
                            if (captured_word != RSP_MAGIC_VAL && bit_err_rate_ppm > 0 &&
                                bit_err_count > 0) begin
                                if (($urandom % 1_000_000) < bit_err_rate_ppm) begin
                                    bit_err_pending     <= 1'b1;
                                    bit_err_target_word <= 1 + ($urandom % 32);
                                end
                            end
                        end

                        // Route to typed capture
                        if (tx_frame_pos == 0 ? (captured_word == RSP_MAGIC_VAL) :
                                                tx_frame_is_ctrl) begin
                            ctrl_capture[ctrl_wr_ptr % CAPN] <= captured_word;
                            ctrl_wr_ptr <= ctrl_wr_ptr + 1;
                        end else begin
                            telem_capture[telem_wr_ptr % CAPN] <= captured_word;
                            telem_wr_ptr <= telem_wr_ptr + 1;
                        end

                        if (dbg_tx_trace)
                            $display("[FT600Q TX] t=%0t cap[%0d]=0x%04h frame_pos=%0d type=%s",
                                     $time, tx_wr_ptr % CAPN, captured_word, tx_frame_pos,
                                     (tx_frame_pos == 0 ? (captured_word == RSP_MAGIC_VAL) :
                                      tx_frame_is_ctrl) ? "CTRL" : "TELEM");

                        // Advance frame position
                        begin : frame_advance
                            int frame_len;
                            frame_len = (tx_frame_pos == 0) ?
                                ((captured_word == RSP_MAGIC_VAL) ?
                                 CTRL_FRAME_LEN : TELEM_FRAME_LEN)
                                : (tx_frame_is_ctrl ? CTRL_FRAME_LEN : TELEM_FRAME_LEN);
                            if (tx_frame_pos == frame_len - 1)
                                tx_frame_pos <= 0;
                            else
                                tx_frame_pos <= tx_frame_pos + 1;
                        end
                    end
                end
            end
        end
    end

    // =========================================================================
    // Task API
    // =========================================================================

    task automatic send_word(input logic [15:0] word);
        rx_queue[rx_wr_ptr[7:0]] = word;
        rx_wr_ptr = rx_wr_ptr + 1;
    endtask

    task automatic send_command_frame(
        input logic [15:0] magic,
        input logic [15:0] flags,
        input logic [15:0] addr,
        input logic [15:0] data
    );
        send_word(magic);
        send_word(flags);
        send_word(addr);
        send_word(data);
    endtask

    // Block until 4 words available in flat tx_capture, then parse as response frame.
    task automatic wait_response_frame(
        output logic [15:0] o_magic,
        output logic [15:0] o_flags,
        output logic [15:0] o_addr,
        output logic [15:0] o_data
    );
        automatic int timeout = 0;
        while ((tx_wr_ptr - tx_rd_ptr) < 4 && timeout < 1_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 1_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for response frame");
        o_magic = tx_capture[(tx_rd_ptr)   % CAPN];
        o_flags = tx_capture[(tx_rd_ptr+1) % CAPN];
        o_addr  = tx_capture[(tx_rd_ptr+2) % CAPN];
        o_data  = tx_capture[(tx_rd_ptr+3) % CAPN];
        tx_rd_ptr = tx_rd_ptr + 4;
    endtask

    // Block until 37 words available in flat tx_capture, then parse as telemetry frame.
    // C-05: not available under Icarus (unpacked output params); use wait_telemetry_frame_v3 instead.
`ifndef ICARUS
    task automatic wait_telemetry_frame(
        output logic [15:0] o_hdr,
        output logic [15:0] o_payload [0:31],
        output logic [15:0] o_token   [0:3]
    );
        automatic int timeout = 0;
        automatic int i;
        while ((tx_wr_ptr - tx_rd_ptr) < 37 && timeout < 10_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 10_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for telemetry frame");
        o_hdr = tx_capture[tx_rd_ptr % CAPN];
        for (i = 0; i < 32; i++) o_payload[i] = tx_capture[(tx_rd_ptr+1+i)  % CAPN];
        for (i = 0; i < 4;  i++) o_token[i]   = tx_capture[(tx_rd_ptr+33+i) % CAPN];
        tx_rd_ptr = tx_rd_ptr + 37;
    endtask
`endif  // !ICARUS (wait_telemetry_frame)

    // Block until one V3 telemetry frame (4103 words) is available:
    // header, 4096 interleaved data words, 4 phase words, CRC hi/lo.
    // C-05: Icarus cannot use unpacked arrays as output task parameters, so two
    // signatures are provided.  Under ICARUS results are stored in module-level
    // v3_data / v3_phase / v3_crc; callers copy them out after the call.
`ifdef ICARUS
    task automatic wait_telemetry_frame_v3(output logic [15:0] o_hdr);
        automatic int timeout = 0;
        automatic int i;
        while ((tx_wr_ptr - tx_rd_ptr) < 4105 && timeout < 10_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 10_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for V3 telemetry frame");
        o_hdr = tx_capture[tx_rd_ptr % CAPN];
        v3_count_lo = tx_capture[(tx_rd_ptr+1) % CAPN];
        v3_count_hi = tx_capture[(tx_rd_ptr+2) % CAPN];
        for (i = 0; i < 4096; i++) v3_data[i]  = tx_capture[(tx_rd_ptr+3+i)    % CAPN];
        for (i = 0; i < 4;    i++) v3_phase[i] = tx_capture[(tx_rd_ptr+4099+i) % CAPN];
        for (i = 0; i < 2;    i++) v3_crc[i]   = tx_capture[(tx_rd_ptr+4103+i) % CAPN];
        tx_rd_ptr = tx_rd_ptr + 4105;
    endtask
`else
    task automatic wait_telemetry_frame_v3(
        output logic [15:0] o_hdr,
        output logic [15:0] o_data  [0:4095],
        output logic [15:0] o_phase [0:3],
        output logic [15:0] o_crc   [0:1]
    );
        automatic int timeout = 0;
        automatic int i;
        while ((tx_wr_ptr - tx_rd_ptr) < 4105 && timeout < 10_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 10_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for V3 telemetry frame");
        // 4105-word layout: [0]=tag, [1]=cnt_lo, [2]=cnt_hi, [3..4098]=data,
        // [4099..4102]=phase, [4103..4104]=CRC.
        o_hdr = tx_capture[tx_rd_ptr % CAPN];
        v3_count_lo = tx_capture[(tx_rd_ptr+1) % CAPN];
        v3_count_hi = tx_capture[(tx_rd_ptr+2) % CAPN];
        for (i = 0; i < 4096; i++) o_data[i]  = tx_capture[(tx_rd_ptr+3+i)    % CAPN];
        for (i = 0; i < 4;    i++) o_phase[i] = tx_capture[(tx_rd_ptr+4099+i) % CAPN];
        for (i = 0; i < 2;    i++) o_crc[i]   = tx_capture[(tx_rd_ptr+4103+i) % CAPN];
        tx_rd_ptr = tx_rd_ptr + 4105;
    endtask
`endif

    // Block until 4 ctrl-typed words are available, then parse as response frame.
    // Reads from ctrl_capture independently of telem_capture, so ctrl responses
    // can be drained in any order relative to interleaved telemetry words.
    task automatic wait_response_frame_typed(
        output logic [15:0] o_magic,
        output logic [15:0] o_flags,
        output logic [15:0] o_addr,
        output logic [15:0] o_data
    );
        automatic int timeout = 0;
        while ((ctrl_wr_ptr - ctrl_rd_ptr) < 4 && timeout < 1_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 1_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for typed ctrl response frame (rx_wr=%0d rx_rd=%0d queued=%0d ctrl_wr=%0d ctrl_rd=%0d)",
                   rx_wr_ptr, rx_rd_ptr, rx_count(), ctrl_wr_ptr, ctrl_rd_ptr);
        o_magic = ctrl_capture[(ctrl_rd_ptr)   % CAPN];
        o_flags = ctrl_capture[(ctrl_rd_ptr+1) % CAPN];
        o_addr  = ctrl_capture[(ctrl_rd_ptr+2) % CAPN];
        o_data  = ctrl_capture[(ctrl_rd_ptr+3) % CAPN];
        ctrl_rd_ptr = ctrl_rd_ptr + 4;
    endtask

    // Block until one complete V3 telemetry frame is available in the typed
    // telemetry capture.  Unlike the legacy flat-capture helper, this remains
    // aligned when a deferred command/fault response follows the prior frame.
    task automatic wait_telemetry_frame_v3_typed(output logic [15:0] o_hdr);
        automatic int timeout = 0;
        while ((telem_wr_ptr - telem_rd_ptr) < 4105 && timeout < 10_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 10_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for typed V3 telemetry frame");
        if ((telem_wr_ptr - telem_rd_ptr) > CAPN)
            $fatal(1, "[FT600Q TLM] telemetry capture ring OVERRUN: %0d words pending > CAPN %0d — the bench let too many frames pile up between reads",
                   telem_wr_ptr - telem_rd_ptr, CAPN);
        o_hdr = telem_capture[telem_rd_ptr % CAPN];
        // Same v3_* side outputs as the untyped reader, but from the telem-only
        // capture: a bench that issues USB commands WHILE streaming must use
        // this one — the raw capture interleaves 4-word responses and the
        // untyped reader then returns frames shifted by 4 words (payload lands
        // in the phase slots; seen 2026-09-09 as "phase word 0xaaaa").
        begin : typed_v3_copy
            automatic int i;
            v3_count_lo = telem_capture[(telem_rd_ptr+1) % CAPN];
            v3_count_hi = telem_capture[(telem_rd_ptr+2) % CAPN];
            for (i = 0; i < 4096; i++) v3_data[i]  = telem_capture[(telem_rd_ptr+3+i)    % CAPN];
            for (i = 0; i < 4;    i++) v3_phase[i] = telem_capture[(telem_rd_ptr+4099+i) % CAPN];
            for (i = 0; i < 2;    i++) v3_crc[i]   = telem_capture[(telem_rd_ptr+4103+i) % CAPN];
        end
        telem_rd_ptr = telem_rd_ptr + 4105;
    endtask

    // Block until 37 telem-typed words are available, then parse as telemetry frame.
    // C-05: not available under Icarus (unpacked output params); use wait_telemetry_frame_v3 instead.
`ifndef ICARUS
    task automatic wait_telemetry_frame_typed(
        output logic [15:0] o_hdr,
        output logic [15:0] o_payload [0:31],
        output logic [15:0] o_token   [0:3]
    );
        automatic int timeout = 0;
        automatic int i;
        while ((telem_wr_ptr - telem_rd_ptr) < 37 && timeout < 10_000_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 10_000_000)
            $fatal(1, "[FT600Q TLM] Timeout waiting for typed telemetry frame");
        o_hdr = telem_capture[telem_rd_ptr % CAPN];
        for (i = 0; i < 32; i++) o_payload[i] = telem_capture[(telem_rd_ptr+1+i)  % CAPN];
        for (i = 0; i < 4;  i++) o_token[i]   = telem_capture[(telem_rd_ptr+33+i) % CAPN];
        telem_rd_ptr = telem_rd_ptr + 37;
    endtask
`endif  // !ICARUS (wait_telemetry_frame_typed)

    // Force TXE_N = 1 (host buffer full) to test manual backpressure
    task automatic set_txe_backpressure(input logic ena);
        bp_force = ena;
    endtask

    // Enable/disable per-word TX capture trace
    task automatic enable_tx_trace(input logic ena);
        dbg_tx_trace = ena;
    endtask

    // Telemetry frames captured and not yet read.  Frames are back-to-back in
    // telem_capture (ctrl words go to ctrl_capture), so boundaries are multiples
    // of the frame length from the first telemetry word.
    function automatic int telem_frames_pending();
        return (telem_wr_ptr - telem_rd_ptr) / TELEM_FRAME_LEN;
    endfunction

    // Drop all pending telemetry frames except the newest `keep`.  A sequential
    // bench that issues USB commands mid-stream cannot drain telemetry while it
    // waits for responses (the DUT holds each one to an inter-frame gap), so
    // frames pile up; a real host keeps reading.  Returns how many were dropped.
    task automatic keep_newest_telemetry_frames(input int keep, output int dropped);
        automatic int complete_frames, new_rd;
        complete_frames = telem_wr_ptr / TELEM_FRAME_LEN;
        new_rd = (complete_frames - keep) * TELEM_FRAME_LEN;
        if (new_rd > telem_rd_ptr) begin
            dropped = (new_rd - telem_rd_ptr) / TELEM_FRAME_LEN;
            telem_rd_ptr = new_rd;
        end else dropped = 0;
    endtask

    // Enable/disable the per-frame boundary trace (cheap: one line per frame)
    task automatic enable_frame_trace(input logic ena);
        dbg_frame_trace = ena;
    endtask

    // =========================================================================
    // §2.1 — Random TXE_N backpressure injection
    // =========================================================================

    // Enable/disable random backpressure injection.
    //   enable          : 1 = enable, 0 = disable (clears burst counter)
    //   probability_ppm : probability per write cycle (0=off, 1_000_000=every cycle)
    //   min_cycles      : minimum burst length in clock cycles
    //   max_cycles      : maximum burst length in clock cycles
    task automatic set_txe_random_backpressure(
        input logic  enable,
        input int    probability_ppm,
        input int    min_cycles,
        input int    max_cycles
    );
        bp_rand_enable     = enable;
        bp_rand_prob_ppm   = probability_ppm;
        bp_rand_min_cycles = min_cycles;
        bp_rand_max_cycles = max_cycles;
        if (!enable) bp_rand_remain = 0;
    endtask

    // Preset: realistic worst-case stress (~5% of write cycles, 5–200 cycle bursts).
    task automatic set_txe_stress_preset();
        set_txe_random_backpressure(1'b1, 50_000, 5, 200);
    endtask

    // Read and optionally reset backpressure statistics.
    task automatic get_bp_stats(
        output int events,
        output int cycles,
        output int words_stalled
    );
        events        = bp_stat_events;
        cycles        = bp_stat_cycles;
        words_stalled = bp_stat_words_stalled;
    endtask

    // =========================================================================
    // §2.2 — RXF_N inter-packet gap simulation
    // =========================================================================

    // Configure inter-packet gap.
    //   words_per_packet : 0 = disable (original continuous RXF_N behavior)
    //   gap_cycles       : RXF_N high duration between packets (66 MHz cycles)
    task automatic set_rxf_packet_gap(
        input int words_per_packet,
        input int gap_cycles_in
    );
        rxf_words_per_pkt   = words_per_packet;
        rxf_gap_cycles      = gap_cycles_in;
        rxf_words_delivered = 0;
        rxf_gap_remain      = 0;
    endtask

    // =========================================================================
    // §2.3 — Overflow drop counter
    // =========================================================================

    // Return number of words dropped due to TXE_N=1 (overflow model).
    task automatic get_overflow_drop_count(output int dropped);
        dropped = overflow_drop_count;
    endtask

    // =========================================================================
    // §2.4 — Bit-error / payload corruption injection
    // =========================================================================

    // Schedule single-bit flips in telemetry frame payloads (words[1..32]).
    //   count     : number of frames to corrupt (0 = disable)
    //   rate_ppm  : per-frame probability in random mode (0 = one-shot sequential)
    // In one-shot mode (rate_ppm=0): the next `count` telem frames are each
    //   corrupted at a randomly chosen payload word.
    // In random mode (rate_ppm>0): each telem frame has a rate_ppm/1M chance of
    //   corruption; count limits the total number of corruption events (0 = unlimited).
    task automatic inject_payload_bit_error(input int count, input int rate_ppm);
        bit_err_count    = count;
        bit_err_rate_ppm = rate_ppm;
        if (count > 0 && rate_ppm == 0) begin
            // Arm immediately for the next telem frame
            bit_err_pending     = 1'b1;
            bit_err_target_word = 1 + ($urandom % 32);
        end else begin
            bit_err_pending = 1'b0;
        end
    endtask

    // =========================================================================
    // §4.2 — FTD-accurate read_transfer task
    // Returns raw captured words as the real FT600Q would deliver them:
    //   up to max_words words, or fewer if gap_cycles of write inactivity
    //   occur (models the 1 ms USB short-packet flush).
    // =========================================================================

    // Return up to max_words (512 = 1024 bytes) from tx_capture, or fewer if
    // the FPGA stops writing for gap_cycles clock cycles (short-packet model).
    // C-05: not available under Icarus (unpacked output params).
`ifndef ICARUS
    //   max_words   : 512 for standard 1024-byte USB SuperSpeed packet mode
    //   gap_cycles  : FPGA write-inactivity threshold (66000 ≈ 1 ms at 66 MHz)
    //   out words[] : raw captured words (may span frame boundaries)
    //   out n       : number of valid words returned (0 = timed out with no data)
    task automatic read_transfer(
        input  int           max_words,
        input  int           gap_cycles,
        output logic [15:0]  words [0:511],
        output int           n
    );
        automatic int collected    = 0;
        automatic int idle_cycles  = 0;
        automatic int last_wr_ptr  = tx_wr_ptr;

        n = 0;
        while (collected < max_words) begin
            @(posedge clk_66m);
            if (tx_wr_ptr > tx_rd_ptr) begin
                // New word(s) available — drain one per cycle to match hardware pacing
                words[collected] = tx_capture[tx_rd_ptr % CAPN];
                tx_rd_ptr  = tx_rd_ptr + 1;
                collected  = collected + 1;
                idle_cycles = 0;
                last_wr_ptr = tx_wr_ptr;
            end else begin
                idle_cycles = idle_cycles + 1;
                if (idle_cycles >= gap_cycles && collected > 0) break; // short-packet flush
            end
        end
        n = collected;
    endtask
`endif  // !ICARUS (read_transfer)

    // Return all currently committed TX words and advance all read pointers.
    // Waits until the FPGA is between frames (tx_frame_pos==0) before draining.
    // C-05: Icarus version omits the words[] output param (content is discarded anyway).
`ifdef ICARUS
    task automatic flush_tx_capture(output int n);
`else
    task automatic flush_tx_capture(
        output logic [15:0] words [0:4095],   // callers all declare [0:4095]
        output int          n
    );
`endif
        automatic int i;
        automatic int timeout = 0;
        automatic int initial_wr_ptr = tx_wr_ptr;
        // Drain the current partial frame from the FPGA before snapshotting.
        // If the FPGA has already gone quiet (framer idle), tx_frame_pos is already
        // 0 and this loop exits immediately.  Worst case: one full 37-word telem
        // frame draining at 66 MHz = ~560 ns.
        if (dbg_tx_trace)
            $display("[FT600Q TLM] flush_tx_capture start: tx_frame_pos=%0d tx_wr_ptr=%0d",
                     tx_frame_pos, tx_wr_ptr);
        while (tx_frame_pos != 0 && timeout < 100_000) begin
            @(posedge clk_66m);
            timeout++;
        end
        if (timeout >= 100_000)
            $display("[FT600Q TLM] WARNING: flush_tx_capture timed out (tx_frame_pos=%0d, words_arrived=%0d)",
                     tx_frame_pos, tx_wr_ptr - initial_wr_ptr);
        else if (dbg_tx_trace)
            $display("[FT600Q TLM] flush_tx_capture aligned in %0d cycles (tx_frame_pos→0, words_arrived=%0d)",
                     timeout, tx_wr_ptr - initial_wr_ptr);
        // Force re-alignment regardless of whether the wait succeeded or timed out.
        // When a ctrl response is interleaved mid-telem-frame the TLM frame counter
        // can end on a non-multiple-of-37 boundary, leaving tx_frame_pos stuck at a
        // small non-zero value while the framer is idle.  Resetting here ensures the
        // next arriving word (a ctrl response magic 0x55AA, or a new telem header) is
        // correctly classified at frame position 0.  The typed read-ptr reset below
        // discards any stale or misrouted captures from the interrupted frame.
        tx_frame_pos     = 0;
        tx_frame_is_ctrl = 1'b0;
        n = tx_wr_ptr - tx_rd_ptr;
`ifndef ICARUS
        // The words are informational (the whole point is to DISCARD them);
        // cap the copy at the caller's buffer, drain the pointers fully.
        for (i = 0; i < n && i < 4096; i++)
            words[i] = tx_capture[(tx_rd_ptr + i) % CAPN];
`endif
        tx_rd_ptr    = tx_wr_ptr;
        ctrl_rd_ptr  = ctrl_wr_ptr;
        telem_rd_ptr = telem_wr_ptr;
    endtask

endmodule
