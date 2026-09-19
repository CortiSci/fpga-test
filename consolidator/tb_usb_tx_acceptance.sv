`timescale 1ns/1ps
// FT600 accepts at the reference edge only when BOTH WR_N and TXE_N are low.
// No invented grace writes after TXE_N rises (FTDI DS_FT600Q v1.04, fig 4.7).
// A queued source feeds the production CDC and phased mover. Stop the sink
// mid-burst, then at consecutive resume boundaries, and compare every word.
module tb_usb_tx_acceptance;
    reg core_clk=0, usb_clk=0, rst_n=0, txe_n=0;
    always #10 core_clk=~core_clk;
    always #7.5 usb_clk=~usb_clk;
    wire launch_clk;
    assign #2.5 launch_clk=usb_clk;
    localparam N=1024;
    integer produced=0,consumed=0,accepted=0,mismatches=0,stalls=0,passes=0,failures=0;
    wire [15:0] head,bus_data;
    wire full,almost,empty,ready,wr_n,rd_n,oe_n;
    wire [9:0] level;
    function automatic [15:0] pattern(input integer n);
        pattern=(n*73)^16'ha531;
    endfunction
    telem_tx_fifo fifo(.wr_clk(core_clk),.wr_rst_n(rst_n),
        .wr_data(pattern(produced)),.wr_en(rst_n && produced<N && !full),
        .wr_full(full),.wr_almost_full(almost),.wr_half_full(),
        .rd_clk(usb_clk),.rd_rst_n(rst_n),.rd_data(head),
        .rd_en(ready && !empty),.rd_empty(empty),.rd_level(level));
    ft600_245_fifo_fsm #(.PHASED_OUTPUT(1)) dut(
        .usb_fifo_clk(usb_clk),.usb_launch_clk(launch_clk),.devrst_n(rst_n),
        .usb_fifo_d(bus_data),.usb_fifo_rxf_n(1'b1),.usb_fifo_txe_n(txe_n),
        .usb_fifo_rd_n(rd_n),.usb_fifo_wr_n(wr_n),.usb_fifo_oe_n(oe_n),
        .cmd_out_data(),.cmd_out_valid(),.cmd_out_ready(1'b1),
        .tx_in_data(head),.tx_in_valid(!empty),.tx_in_ready(ready),
        .tx_in_queued_words(level),.telemetry_active(1'b1),
        .tx_source_idle(produced==N),.dbg_state());
    always @(posedge core_clk) if(rst_n && produced<N && !full) produced<=produced+1;
    always @(posedge usb_clk) if(rst_n) begin
        if(ready && !empty) consumed=consumed+1;
        if(!wr_n && !txe_n) begin
            if(bus_data!==pattern(accepted)) begin
                if(mismatches<4) $display("[TX-ACCEPT] mismatch index=%0d expected=%h got=%h",accepted,pattern(accepted),bus_data);
                mismatches=mismatches+1;
            end
            accepted=accepted+1;
        end
    end
    task check(input bit ok,input string label);
        if(ok) begin passes++; $display("[TX-ACCEPT] PASS %s",label); end
        else begin failures++; $display("[TX-ACCEPT] FAIL %s",label); end
    endtask
    initial begin
        repeat(5) @(negedge usb_clk); rst_n=1;
        wait(accepted>=40);
        repeat(12) begin
            @(negedge usb_clk); txe_n=1; stalls++;
            repeat(7+stalls) @(negedge usb_clk);
            txe_n=0;
            // Reassert on the first write presented after resumption too.
            wait(!wr_n); @(posedge usb_clk);
        end
        @(negedge usb_clk); txe_n=0;
        repeat(4000) @(negedge usb_clk);
        $display("[TX-ACCEPT] produced=%0d dequeued=%0d accepted=%0d mismatches=%0d stalls=%0d",produced,consumed,accepted,mismatches,stalls);
        check(produced==N && consumed==N,"source and CDC delivered all words");
        check(accepted==N,"sink accepted every word despite full transitions");
        check(mismatches==0,"accepted stream is exact and ordered");
        check(stalls==12,"all twelve full/resume boundaries exercised");
        $display("RESULTS: %0d passed, %0d failed",passes,failures);
        $display("STATUS: %s",failures==0?"PASS":"FAIL");
        $finish;
    end
    initial begin #200000; $fatal(1,"TX acceptance timeout"); end
endmodule
