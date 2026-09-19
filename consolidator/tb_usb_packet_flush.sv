`timescale 1ns/1ps
// Packet-stop regression and its required liveness cases: a 4-word residual
// behind a full packet must survive idle blips, serve RX, then flush on sustained
// idle or RUN clear. Pin-level scoreboards; no DUT forces or simulated grace writes.
module tb_usb_packet_flush;
    reg core_clk=0,usb_clk=0,rst_n=0,txe_n=0,active=1,source_idle=0;
    always #10 core_clk=~core_clk;
    always #7.5 usb_clk=~usb_clk;
    wire launch_clk; assign #2.5 launch_clk=usb_clk;
    integer limit=0,produced=0,accepted=0,errors=0,cmd_sent=0,cmd_seen=0;
    integer passes=0,failures=0;
    reg send_cmd=0;
    wire [15:0] head,bus_data,cmd_data;
    wire full,empty,pop,wr_n,rd_n,oe_n,cmd_valid;
    wire [9:0] level;
    wire rxf_n=!(send_cmd && cmd_sent<4);
    function automatic [15:0] pattern(input integer n); pattern=(n*73)^16'ha531; endfunction
    assign bus_data=(!oe_n && !rxf_n) ? (16'h1230+cmd_sent) : 16'hzzzz;
    telem_tx_fifo fifo(.wr_clk(core_clk),.wr_rst_n(rst_n),
        .wr_data(pattern(produced)),.wr_en(rst_n && produced<limit && !full),
        .wr_full(full),.wr_almost_full(),.wr_half_full(),
        .rd_clk(usb_clk),.rd_rst_n(rst_n),.rd_data(head),
        .rd_en(pop&&!empty),.rd_empty(empty),.rd_level(level));
    ft600_245_fifo_fsm #(.PHASED_OUTPUT(1)) mover(
        .usb_fifo_clk(usb_clk),.usb_launch_clk(launch_clk),.devrst_n(rst_n),
        .usb_fifo_d(bus_data),.usb_fifo_rxf_n(rxf_n),.usb_fifo_txe_n(txe_n),
        .usb_fifo_rd_n(rd_n),.usb_fifo_wr_n(wr_n),.usb_fifo_oe_n(oe_n),
        .cmd_out_data(cmd_data),.cmd_out_valid(cmd_valid),.cmd_out_ready(1'b1),
        .tx_in_data(head),.tx_in_valid(!empty),.tx_in_ready(pop),
        .tx_in_queued_words(level),.telemetry_active(active),
        .tx_source_idle(source_idle),.dbg_state());
    always @(posedge core_clk)
        if(!rst_n) produced<=0;
        else if(produced<limit && !full) produced<=produced+1;
    always @(posedge usb_clk) begin
        if(!rst_n) begin cmd_sent<=0;cmd_seen=0;accepted=0;errors=0; end
        else begin
            if(!rd_n && !oe_n && !rxf_n) cmd_sent<=cmd_sent+1;
            if(cmd_valid) begin
                if(cmd_data!==(16'h1230+cmd_seen)) errors++;
                cmd_seen++;
            end
            if(!wr_n && !txe_n) begin
                if(bus_data!==pattern(accepted)) errors++;
                accepted++;
            end
        end
    end
    task check(input bit ok,input string label);
        if(ok) begin passes++;$display("[FLUSH] PASS %s",label);end
        else begin failures++;$display("[FLUSH] FAIL %s accepted=%0d errors=%0d",label,accepted,errors);end
    endtask
    task reset_case;
        begin
            @(negedge usb_clk);rst_n=0;limit=0;active=1;source_idle=0;txe_n=0;send_cmd=0;
            repeat(5) @(negedge usb_clk);
            rst_n=1;
        end
    endtask
    initial begin
        reset_case();limit=260;
        // Reject and retry several writes within the reserved packet.
        wait(accepted>=40);
        repeat(12) begin
            @(negedge usb_clk);txe_n=1;
            repeat(9) @(negedge usb_clk);
            txe_n=0;
            repeat(5) @(negedge usb_clk);
        end
        // Reject the last word of the packet, precisely where the new stop
        // decision fires. Launching it must not consume the packet credit.
        wait(accepted==255);
        @(negedge usb_clk);txe_n=1;
        repeat(7) @(negedge usb_clk);
        check(accepted==255,"rejected final packet word does not advance acceptance");
        txe_n=0;
        repeat(700) @(negedge usb_clk);
        check(accepted==256 && errors==0,"full packet delivered exactly; residual retained after TXE stalls");
        send_cmd=1;
        repeat(30) begin
            source_idle=1;repeat(3) @(negedge usb_clk);
            source_idle=0;repeat(3) @(negedge usb_clk);
        end
        check(accepted==256,"brief inter-frame idle does not release the residual");
        check(cmd_sent==4 && cmd_seen==4 && errors==0,"host commands received while partial TX waits");
        source_idle=1;
        repeat(500) @(negedge usb_clk);
        check(accepted==256,"partial packet waits for qualified idle");
        repeat(700) @(negedge usb_clk);
        check(accepted==260 && errors==0,"sustained idle flushes all residual words within 18 us");
        // RUN remains set, source already quiet: a newly queued response must
        // also complete within the bounded flush deadline.
        limit=264;
        repeat(1100) @(negedge usb_clk);
        check(accepted==264 && errors==0,"small response on silent RUN source flushes within 18 us");
        reset_case();limit=4;
        repeat(100) @(negedge usb_clk);
        check(accepted==0,"active source retains sub-packet data");
        active=0;
        repeat(50) @(negedge usb_clk);
        check(accepted==4 && errors==0,"RUN clear flushes short tail without idle timeout");
        reset_case();limit=512;
        repeat(1300) @(negedge usb_clk);
        check(accepted==512 && errors==0,"reset clears partial-packet state for fresh stream");
        $display("RESULTS: %0d passed, %0d failed",passes,failures);
        $display("STATUS: %s",failures==0?"PASS":"FAIL");
        if(failures) $fatal(1,"packet flush regression");
        $finish;
    end
    initial begin #200000;$fatal(1,"packet flush timeout");end
endmodule
