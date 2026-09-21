`timescale 1ns/1ps
// The host-visible form of the receive loss (tb_usb_rx_backpressure_loss):
// the bring-up tool's fuzz batch check, run against the RTL.
//
// The fuzz tab sends bad-magic eight-byte packets and, after each batch, a
// valid USB_RX_WORDS probe; the batch is confirmed only if the counter has
// advanced by exactly four words per packet plus the probe's own four.  On the
// 2026-09-18 hardware runs every batch reconciled (the writes were paced and
// each arrived alone), so that workload never exercised this path.  This bench
// asks what happens when the same packets reach the FT600 as ONE contiguous
// burst -- queued host writes delivered back to back -- through the production
// mover, the 32-word receive CDC and cmd_decoder, with RUN clear as recorded:
//
//   * the probe after the burst must be answered, with the exact count
//     (4 x packets + 4).  On the current mover words are lost at every
//     ready-low exit, so either the count is short or -- because command
//     framing is a plain four-word count with no realignment -- the probe's
//     AA55 lands mid-frame and nothing answers at all;
//   * a second probe, sent with the link idle, must also be answered: the
//     path has to work again once the burst is over (it does not: the
//     misalignment persists until reset);
//   * no malformed packet may write a register or draw a response.
//
// Corpus: the tool's xorshift32-v1 generator, seed 1, five invalid-magic
// families, first 60 candidates (the recorded corpus's own opening packets).
module tb_usb_rx_probe_after_burst;
    reg usb_clk=0, core_clk=0, rst_n=0;
    always #7.5 usb_clk=~usb_clk;          // 66.7 MHz FT600 clock
    always #10 core_clk=~core_clk;   // 50 MHz core (pll_48m CLKOP)

    localparam integer N_PKTS = 60;
`ifdef FUZZ_WAITING_RESPONSE
    localparam integer PREFIX_WORDS=4, PREFIX_RESPONSES=1;
`else
    localparam integer PREFIX_WORDS=0, PREFIX_RESPONSES=0;
`endif
    reg framer_busy=0;

    wire [15:0] usb_d;
    wire [1:0] usb_be;
    wire rxf_n, txe_n, rd_n, wr_n, oe_n;
    wire [15:0] mover_data, rx_data, tx_data, reg_addr, reg_wdata;
    wire mover_valid, fifo_full, fifo_almost_full, fifo_empty;
    wire rx_ready, tx_valid, reg_we, fault_frame_sent, ctrl_early;
    wire [4:0] fault_flags_sent;
    wire mover_ready = ~fifo_almost_full & ~fifo_full;   // consolidator_v2_top fsm_rx_ready
    wire rx_valid    = ~fifo_empty;                       // consolidator_v2_top cmd_rx_valid
    reg  tx_ready = 1;                                    // TX CDC has room (nothing else is sending)
    reg  [15:0] rx_words = 0;                             // USB_RX_WORDS: words cmd_decoder consumed
    wire [15:0] reg_rdata = (reg_addr==16'h005e) ? rx_words : 16'hcafe;

    ft600q_tlm model(.clk_66m(usb_clk),.rst_n(rst_n),.usb_fifo_d(usb_d),
        .usb_fifo_be(usb_be),.rxf_n(rxf_n),.txe_n(txe_n),
        .rd_n(rd_n),.wr_n(wr_n),.oe_n(oe_n));
    wire usb_launch_clk;
    assign #2.5 usb_launch_clk = usb_clk;
    ft600_245_fifo_fsm #(.PHASED_OUTPUT(1)) mover(.usb_launch_clk(usb_launch_clk),.usb_fifo_clk(usb_clk),.devrst_n(rst_n),
        .usb_fifo_d(usb_d),.usb_fifo_rxf_n(rxf_n),.usb_fifo_txe_n(txe_n),
        .usb_fifo_rd_n(rd_n),.usb_fifo_wr_n(wr_n),.usb_fifo_oe_n(oe_n),
        .cmd_out_data(mover_data),.cmd_out_valid(mover_valid),.cmd_out_ready(mover_ready),
        .tx_in_data(16'h0),.tx_in_valid(1'b0),.tx_in_ready(),
        .tx_in_queued_words(10'h0),.telemetry_active(1'b0),.tx_source_idle(1'b1),
        .dbg_state());
    cdc_fifo #(.WIDTH(16),.DEPTH_LOG2(5)) fifo(.wr_clk(usb_clk),.wr_rst_n(rst_n),
        .wr_data(mover_data),.wr_en(mover_valid),.wr_full(fifo_full),
        .wr_almost_full(fifo_almost_full),.wr_half_full(),.rd_clk(core_clk),
        .rd_rst_n(rst_n),.rd_data(rx_data),.rd_en(rx_valid & rx_ready),
        .rd_empty(fifo_empty),.rd_level());
    cmd_decoder dut(.clk(core_clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),
        .rx_ready(rx_ready),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),
        .reg_addr(reg_addr),.reg_wdata(reg_wdata),.reg_we(reg_we),.reg_rdata(reg_rdata),
        .parity_events(4'b0),.fault_src(5'b0),.fault_frame_sent(fault_frame_sent),.fault_flags_sent(fault_flags_sent),
        .framer_busy(framer_busy),.ctrl_early(ctrl_early));

    integer failures=0, passes=0, writes=0, responses=0, resp_idx=0, burst_responses=0;
    reg [15:0] resp[0:3];
    reg in_burst=0;

    task fail(input string reason);
        begin failures=failures+1; $display("[RX-PROBE] FAIL %s",reason); end
    endtask

    // ---- register / response monitor (core clock) ----------------------------
    always @(posedge core_clk) if (rst_n) begin
        if (rx_valid && rx_ready) rx_words <= rx_words + 1'b1;
        if (reg_we) begin
            writes = writes+1;
            $display("[RX-PROBE] register WRITE addr=%04h data=%04h", reg_addr, reg_wdata);
        end
        if (tx_valid && tx_ready) begin
            resp[resp_idx] = tx_data;
            if (resp_idx==3) begin
                responses = responses+1;
`ifdef FUZZ_WAITING_RESPONSE
                if(responses==1 && (resp[0]!==16'h55aa || resp[1]!==0 || resp[2]!==16'h005e || resp[3]!==4))
                    fail("initial deferred response is damaged");
`endif
                if (in_burst) burst_responses = burst_responses+1;
                $display("[RX-PROBE] response #%0d: %04h %04h %04h %04h  (words consumed so far %0d)",
                         responses,resp[0],resp[1],resp[2],resp[3],rx_words);
            end
            resp_idx = (resp_idx==3) ? 0 : resp_idx+1;
        end
    end

    // ---- the tool's generator: xorshift32-v1, seed 1 --------------------------
    reg [31:0] rng = 32'd1;
    task automatic next_random(output reg [31:0] v);
        begin rng=rng^(rng<<13); rng=rng^(rng>>17); rng=rng^(rng<<5); v=rng; end
    endtask
    task automatic queue_bad_magic_packet(input integer idx);
        reg [31:0] value; reg [15:0] w0,w1,w2,w3;
        begin
            case (idx%5)
                0: begin next_random(value); w0=16'haa55^(32'd1<<(value%16)); end   // single bit flipped
                1: w0=16'h55aa;                                                       // byte-swapped magic
                2: begin next_random(value); w0=16'haa55+1+(value%255); end          // near-magic
                3: begin next_random(value); w0=value[15:0]; end                     // random
                4: begin next_random(value); w0=(value&255)*16'h0101; end            // repeated byte
            endcase
            if (w0==16'haa55) w0=w0^1;
            next_random(value); w1=value[15:0];
            next_random(value); w2=value[15:0];
            next_random(value); w3=value[15:0];
            model.send_word(w0); model.send_word(w1); model.send_word(w2); model.send_word(w3);
        end
    endtask

    // Send one valid USB_RX_WORDS probe and wait for its answer.
    task automatic probe(input integer expected_count, input string what);
        integer target, waits;
        begin
            target = responses+1;
            model.send_command_frame(16'haa55,16'h0000,16'h005e,16'h0000);
            waits=0;
            while (responses<target && waits<3000) begin @(negedge core_clk); waits=waits+1; end
            if (responses<target) begin
                $display("[RX-PROBE] %s: no response within %0d core cycles (decoder word_idx=%0d, state=%0d, words consumed %0d, FT600 holds %0d)",
                         what,waits,dut.word_idx,dut.state,rx_words,model.rx_count());
                fail({what,": valid probe not answered"});
            end else begin
                if (resp[0]!==16'h55aa || resp[1]!==16'h0000 || resp[2]!==16'h005e)
                    fail({what,": response header wrong"});
                if (resp[3]!==expected_count[15:0]) begin
                    $display("[RX-PROBE] %s: USB_RX_WORDS=%0d, expected %0d (%0d word(s) short)",
                             what,resp[3],expected_count,expected_count-resp[3]);
                    fail({what,": counter does not reconcile (4 words per packet + 4)"});
                end else begin
                    $display("[RX-PROBE] %s: answered, USB_RX_WORDS=%0d reconciles",what,resp[3]);
                    passes=passes+1;
                end
            end
        end
    endtask

    integer i, waits;
    initial begin
        repeat(5) @(negedge usb_clk); rst_n=1;
        repeat(4) @(negedge core_clk);

        `ifdef FUZZ_WAITING_RESPONSE
        // A real decoder response waits for a telemetry frame boundary.
        // Meanwhile a host OUT burst can fill the RX CDC without any reads.
        framer_busy=1;
        model.send_command_frame(16'haa55,16'h0000,16'h005e,16'h0000);
        waits=0;
        while(rx_words!=4 && waits<1000) begin @(negedge core_clk); waits++; end
        if(rx_words!=4) fail("initial valid command was not consumed");
        repeat(10) @(negedge core_clk);
        if(rx_ready) fail("decoder did not wait for the telemetry boundary");
`endif
        // One contiguous FT600 burst of 60 malformed packets.
        in_burst=1;
        for (i=0;i<N_PKTS;i=i+1) queue_bad_magic_packet(i);
`ifdef FUZZ_WAITING_RESPONSE
        waits=0;
        while(!fifo_almost_full && waits<1000) begin @(negedge core_clk); waits++; end
        if(!fifo_almost_full) fail("burst never reached RX backpressure");
        repeat(12) @(negedge core_clk);
        framer_busy=0;
`endif
        waits=0;
        while ((model.rx_count()!=0 || !fifo_empty) && waits<6000) begin
            @(negedge core_clk); waits=waits+1;
        end
        repeat(20) @(negedge core_clk);
        in_burst=0;
        $display("[RX-PROBE] burst done: FT600 dequeued %0d words, decoder consumed %0d, register writes %0d, responses %0d",
                 model.rx_rd_ptr,rx_words,writes,burst_responses);
        if (model.rx_rd_ptr!=PREFIX_WORDS+4*N_PKTS) fail("FT600 did not deliver the whole burst (bench/model problem)");
        if (burst_responses!=PREFIX_RESPONSES)        fail("a malformed packet drew a response");

        // The tool's batch check, then the link idle.
        probe(PREFIX_WORDS+4*N_PKTS+4,   "probe after the burst");
        repeat(200) @(negedge core_clk);
        probe(PREFIX_WORDS+4*N_PKTS+8,   "second probe, link idle");

        if (writes!=0) fail("a malformed packet performed a register write");
        else passes=passes+1;

        if (failures==0) begin
            $display("RESULTS: %0d passed, 0 failed",passes);
            $display("STATUS: PASS");
        end else begin
            $display("RESULTS: %0d passed, %0d failed",passes,failures);
            $display("STATUS: FAIL");
            $fatal(1,"command path broken by a contiguous OUT burst");
        end
        $finish;
    end
    initial begin #500000; fail("simulation watchdog"); $display("STATUS: FAIL"); $fatal(1); end
endmodule
