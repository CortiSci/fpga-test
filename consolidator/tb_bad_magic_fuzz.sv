`timescale 1ns/1ps
// Host-run 20260918-232928: a valid USB_RX_WORDS probe write timed out after
// 38,200 malformed writes. Exercise the actual decoder with the same seed-1
// corpus through 100,000 packets, checking the boundary probes and register
// isolation. This is NOT a model of FT600 silicon/driver timeout behavior.
module tb_bad_magic_fuzz;
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg [15:0] rx_data=0;
    reg rx_valid=0, tx_ready=1, framer_busy=0;
    wire rx_ready, tx_valid, reg_we, fault_frame_sent, ctrl_early;
    wire [15:0] tx_data, reg_addr, reg_wdata;
    wire [4:0] fault_flags_sent;
    reg [15:0] rx_words=0;
    wire [15:0] reg_rdata=(reg_addr==16'h005e)?rx_words:16'hcafe;
    integer cycles=0, total_words=0, replies=0, response_word=0, probes=0;
    integer packets=0, writes=0, passes=0, failures=0;
    reg probe_active=0;
    reg [15:0] expected_count;
    reg [31:0] rng=1, value;
    reg [15:0] w0,w1,w2,w3;

    cmd_decoder dut(.clk(clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),.rx_ready(rx_ready),
        .tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),
        .reg_addr(reg_addr),.reg_wdata(reg_wdata),.reg_we(reg_we),.reg_rdata(reg_rdata),
        .fault_src(5'b0),.fault_frame_sent(fault_frame_sent),.fault_flags_sent(fault_flags_sent),
        .framer_busy(framer_busy),.ctrl_early(ctrl_early));

    task automatic fail(input string reason);
        begin
            $display("FAIL packet=%0d probe=%0d word=%0d: %s",packets,probes,response_word,reason);
            $display("RESULTS: %0d passed, 1 failed",passes);
            $display("STATUS: FAIL");
            $fatal(1,"decoder fuzz failure");
        end
    endtask
    always @(posedge clk) if (rst_n) begin
        cycles<=cycles+1;
        if (rx_valid && rx_ready) begin rx_words<=rx_words+1'b1; total_words<=total_words+1; end
        if (reg_we) begin writes<=writes+1; fail("bad-magic input performed a register write"); end
        if (fault_frame_sent) fail("unexpected fault frame");
        if (tx_valid && tx_ready) begin
            if (!probe_active) fail("response emitted for bad-magic input");
            case (response_word)
                0: if (tx_data!==16'h55aa) fail("response magic");
                1: if (tx_data!==16'h0000) fail("response flags");
                2: if (tx_data!==16'h005e) fail("response address");
                3: if (tx_data!==expected_count) fail("receive count (including probe / wrap)");
            endcase
            if (response_word==3) begin response_word<=0; replies<=replies+1; end
            else response_word<=response_word+1;
        end
    end
    // Deterministic downstream backpressure, including within a response.
    always @(negedge clk) tx_ready <= (cycles%11!=3 && cycles%11!=4 && cycles%11!=5);
    task automatic next_random(output reg [31:0] v);
        begin rng=rng^(rng<<13); rng=rng^(rng>>17); rng=rng^(rng<<5); v=rng; end
    endtask
    task automatic send_word(input [15:0] w);
        integer waits;
        begin
            @(negedge clk); rx_data=w; rx_valid=1;
            waits=0;
            @(posedge clk);
            while (!rx_ready) begin
                waits=waits+1; if (waits>100) fail("RX handshake stalled");
                @(posedge clk);
            end
            @(negedge clk); rx_valid=0;
            // Host pauses between words must not lose four-word alignment.
            if (total_words%97==0) repeat(3) @(negedge clk);
        end
    endtask
    task automatic probe;
        integer target,waits;
        begin
            expected_count=rx_words+16'd4;
            target=replies+1; probe_active=1; probes=probes+1;
            framer_busy=probes[0];
            send_word(16'haa55); send_word(0); send_word(16'h005e); send_word(0);
            repeat(8) @(negedge clk);
            framer_busy=0;
            waits=0;
            while (replies<target) begin
                @(negedge clk); waits=waits+1;
                if (waits>100) fail("valid receive-counter probe did not finish");
            end
            @(negedge clk); probe_active=0;
            passes=passes+1;
        end
    endtask
    integer i;
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        probe(); probe(); // same counter-read pilot as the host harness
        for (i=0;i<100000;i=i+1) begin
            case(i%5)
                0: begin next_random(value); w0=16'haa55^(32'd1<<(value%16)); end
                1: w0=16'h55aa;
                2: begin next_random(value); w0=16'haa55+1+(value%255); end
                3: begin next_random(value); w0=value[15:0]; end
                4: begin next_random(value); w0=(value&255)*16'h0101; end
            endcase
            if(w0==16'haa55) w0=w0^1;
            next_random(value); w1=value[15:0];
            next_random(value); w2=value[15:0];
            next_random(value); w3=value[15:0];
            // Anchor the exact final candidate in the saved failing batch.
            if(i==38199 && {w3,w2,w1,w0}!==64'h54ed685df5f1e2e2) fail("seed-1 corpus differs from recorded run");
            send_word(w0); send_word(w1); send_word(w2); send_word(w3);
            packets=packets+1;
            if(packets%100==0) probe();
        end
        repeat(20) @(negedge clk);
        if (packets!=100000 || probes!=1002 || replies!=1002 || total_words!=404008 || writes!=0)
            fail("final packet/response/word/write totals");
        passes=passes+1;
        $display("100000 bad-magic packets, 1002 valid probes, 404008 consumed words, zero register writes");
        $display("RESULTS: %0d passed, 0 failed",passes);
        $display("STATUS: PASS");
        $finish;
    end
    initial begin #20_000_000; fail("simulation watchdog"); end
endmodule
