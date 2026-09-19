`timescale 1ns/1ps
// Directed command-path checks motivated by the two valid-probe timeouts.
// Production RX CDC (32 words) + decoder; no random corpus or long campaign.
// The FT600 pins, USB driver, register map and telemetry engine are not modeled.
module tb_command_recovery;
    reg usb_clk=0, clk=0, rst_n=0;
    always #7.5 usb_clk=~usb_clk;
    always #10.416667 clk=~clk; // 48 MHz core against ~66.7 MHz USB
    reg [15:0] input_data=0;
    reg input_valid=0;
    wire full, empty, rx_ready;
    wire [15:0] rx_data, tx_data, address, write_data;
    wire tx_valid, write_enable, fault_sent, early;
    wire [4:0] fault_flags;
    reg tx_ready=1, framer_busy=0;
    reg [4:0] fault_src=0;
    reg [15:0] consumed=0;
    integer received=0, expected=0, writes=0, passes=0;
    reg [15:0] expected_words[0:255];
    reg held=0;
    reg [15:0] held_data;

    cdc_fifo #(.WIDTH(16),.DEPTH_LOG2(5)) fifo(
        .wr_clk(usb_clk),.wr_rst_n(rst_n),.wr_data(input_data),.wr_en(input_valid),
        .wr_full(full),.wr_almost_full(),.wr_half_full(),
        .rd_clk(clk),.rd_rst_n(rst_n),.rd_data(rx_data),
        .rd_en(rx_ready && !empty),.rd_empty(empty),.rd_level());
    cmd_decoder dut(.clk(clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(!empty),
        .rx_ready(rx_ready),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),
        .reg_addr(address),.reg_wdata(write_data),.reg_we(write_enable),
        .reg_rdata(address==16'h005e ? consumed : 16'h1234),
        .fault_src(fault_src),.fault_frame_sent(fault_sent),.fault_flags_sent(fault_flags),
        .framer_busy(framer_busy),.ctrl_early(early));

    task fail(input string message);
        begin
            $display("[CMD] FAIL %s (received=%0d expected=%0d)",message,received,expected);
            $display("RESULTS: %0d passed, 1 failed",passes);
            $display("STATUS: FAIL"); $fatal(1,"command recovery");
        end
    endtask
    always @(posedge clk) begin
        if (!rst_n) begin consumed<=0; held<=0; end
        else begin
            if (!empty && rx_ready) consumed<=consumed+1'b1;
            if (write_enable) begin writes<=writes+1; fail("unexpected register write"); end
            if (held && (!tx_valid || tx_data!==held_data)) fail("response changed under backpressure");
            held<=tx_valid && !tx_ready; held_data<=tx_data;
            if (tx_valid && tx_ready) begin
                if (received>=expected) fail("unsolicited/duplicate response");
                if (tx_data!==expected_words[received]) fail("response word mismatch");
                received<=received+1;
            end
        end
    end
    task expect_response(input [15:0] flags, addr, data);
        begin
            expected_words[expected]=16'h55aa; expected_words[expected+1]=flags;
            expected_words[expected+2]=addr; expected_words[expected+3]=data;
            expected=expected+4;
        end
    endtask
    task send_word(input [15:0] data);
        integer n;
        begin
            @(negedge usb_clk); n=0;
            while(full) begin @(negedge usb_clk); n=n+1; if(n>500) fail("RX did not recover"); end
            input_data=data; input_valid=1;
            @(negedge usb_clk); input_valid=0;
        end
    endtask
    task packet(input [15:0] magic, flags, addr, data);
        begin send_word(magic); send_word(flags); send_word(addr); send_word(data); end
    endtask
    task settle;
        integer n;
        begin
            n=0;
            while(received!=expected || !empty || !rx_ready) begin
                @(negedge clk); n=n+1; if(n>1000) fail("probe/queue failed to drain");
            end
            repeat(20) @(negedge clk);
            if(received!=expected || writes!=0) fail("drain totals");
            passes=passes+1;
        end
    endtask
    task reset_path;
        begin
            @(negedge clk); rst_n=0; input_valid=0; tx_ready=1; framer_busy=0; fault_src=0;
            repeat(5) @(negedge usb_clk);
            received=0; expected=0; writes=0;
            @(negedge clk); rst_n=1;
        end
    endtask
    integer i;
    initial begin
        reset_path();
        // Exact final malformed packet from each hardware run, then a valid
        // counter probe: neither candidate may write a register or emit a reply.
        expect_response(0,16'h005e,12);
        packet(16'he2e2,16'hf5f1,16'h685d,16'h54ed);
        packet(16'hefef,16'h54b8,16'h2b96,16'he121);
        packet(16'haa55,0,16'h005e,0); settle();

        // Magic embedded inside an invalid four-word packet is payload, not a
        // new command start. WRITE flags must still have no side effects.
        expect_response(0,16'h005e,20);
        packet(16'h55aa,1,16'haa55,16'haa55);
        packet(16'haa55,0,16'h005e,0); settle();

        // An incomplete transfer may pause at every word boundary. No reply or
        // write is allowed until all four words have crossed the CDC.
        for(i=0;i<4;i=i+1) begin
            case(i)
                0: send_word(16'haa55);
                1: send_word(0);
                2: send_word(16'h005e);
                3: begin expect_response(0,16'h005e,24); send_word(0); end
            endcase
            repeat(25) @(negedge clk);
            if(i<3 && (received!=expected || tx_valid)) fail("premature partial-command response");
        end
        settle();

        // Full receive FIFO while a valid reply is blocked. Eight queued
        // commands must survive full->draining with exact order and counts.
        reset_path(); tx_ready=0;
        expect_response(0,16'h005e,4); packet(16'haa55,0,16'h005e,0);
        repeat(20) @(negedge clk);
        for(i=0;i<8;i=i+1) begin
            expect_response(0,16'h005e,8+i*4); packet(16'haa55,0,16'h005e,0);
        end
        repeat(20) @(negedge clk);
        if(!full || received!=0 || consumed!=4 || rx_ready) fail("blocked response did not backpressure RX");
        tx_ready=1; settle();

        // Stop at EACH response word, not just its first word.
        expect_response(0,16'h005e,40); tx_ready=0;
        packet(16'haa55,0,16'h005e,0);
        for(i=0;i<4;i=i+1) begin
            repeat(12) @(negedge clk);
            if(!tx_valid) fail("response disappeared during stall");
            tx_ready=1; @(negedge clk); tx_ready=0;
        end
        tx_ready=1; settle();

        // A current telemetry frame defers a response, then releases it.
        framer_busy=1; expect_response(0,16'h005e,44);
        packet(16'haa55,0,16'h005e,0);
        repeat(40) @(negedge clk);
        if(tx_valid || rx_ready) fail("response crossed telemetry boundary");
        framer_busy=0; settle();

        // Pending fault with a paused partial command must not overwrite it.
        expect_response(0,16'h005e,48); expect_response(16'hffff,16'h0010,0);
        send_word(16'haa55); send_word(0);
        repeat(15) @(negedge clk); fault_src=16; @(negedge clk); fault_src=0;
        repeat(15) @(negedge clk);
        send_word(16'h005e); send_word(0); settle();

        // Reset an incomplete command; next complete probe starts aligned.
        send_word(16'haa55); send_word(1); repeat(10) @(negedge clk);
        reset_path(); expect_response(0,16'h005e,4);
        packet(16'haa55,0,16'h005e,0); settle();
        $display("RESULTS: %0d passed, 0 failed",passes);
        $display("STATUS: PASS"); $finish;
    end
    initial begin #400000; fail("watchdog"); end
endmodule
