`timescale 1ns/1ps
// Pin-level link oracle: arbitrary data, delayed pins, FIFO backpressure,
// source overrun, and an injected parity bit error. No payload DUT overrides.
module tb_pull_link;
    reg clk=0, roclk=0, rst_n=0, run=0, consume=1, inject=0, ss_glitch=0;
    always #10 clk=~clk;
    always #195.3125 roclk=~roclk;
    wire tailclk, mosi, ss, tailmosi, tailss, pre, miso;
    assign #2 tailclk=~clk;
    assign #2 tailmosi=mosi;
    assign #2 tailss=ss && !ss_glitch;
    reg miso_ff=0;
    assign #2 miso=miso_ff;
    integer srcword=0, srcframe=0;
    reg [15:0] data=0;
    reg frame=0;
    always @(negedge roclk) if (rst_n) begin
        frame=srcword==0;
        data={srcframe[5:0],srcword[9:0]};
        srcword=(srcword+1)%1024;
        if (srcword==0) srcframe++;
    end
    wire [31:0] drops;
    wire toggle;
    asic_pull_tx tail(roclk,rst_n,data,frame,tailclk,1'b1,1'b0,1'b0,
                      tailmosi,tailss,pre,drops,toggle);
    reg injected=0;
    always @(posedge tailclk) begin
        miso_ff<=pre;
        if(inject && !injected && tail.state==4 && !tail.header_word) begin
            miso_ff<=!pre; injected<=1;
        end
    end
    reg [4:0] count=0;
    reg done=0;
    wire [15:0] word;
    wire valid,raw,err,gap,anchored,flush,unused_clk,unused_miso;
    spi_ch_pull reader(clk,rst_n,miso,run,1'b0,1'b0,done,count,
        unused_clk,mosi,ss,word,valid,raw,err,gap,anchored,flush,unused_miso);
    integer received=0, completed=0, errors=0, gaps=0, passes=0;
    integer bank_id=0, next_count;
    reg gap_seen=0;
    always @(negedge clk) if(rst_n) begin
        done=0;
        if(err) errors++;
        if(gap && !gap_seen) gaps++;
        gap_seen=gap;
        if(flush) begin count=0; received=0; end
        else begin
            next_count=count;
            if(consume && count!=0) next_count--;
            if(valid) begin
                if(received==0) bank_id=word[15:10];
                if(word[9:0]!==received[9:0] || word[15:10]!==bank_id[5:0])
                    $fatal(1,"mixed/repeated/rotated bank: pos %0d data %h",received,word);
                received++; next_count++; passes++;
                if(received>1024) $fatal(1,"read beyond bank end");
            end
            if(next_count>26) $fatal(1,"backpressure reserve exhausted: %0d",next_count);
            count=next_count;
            if(received==1024 && count==0) begin
                done=1; received=0; completed++;
            end
        end
    end
    time rate_start;
    initial begin
        #100; rst_n=1; run=1;
        wait(received==50);
        @(negedge tailclk);
        while(tail.state!=1 || !tailmosi) @(negedge tailclk);
        ss_glitch=1;
        @(negedge tailclk); ss_glitch=0;
        wait(received==100); @(posedge clk); consume=0;
        #2_000_000;
        if(count<24 || drops==0) $fatal(1,"stall did not fill FIFO and overrun source");
        consume=1;
        wait(completed==6);
        if(gaps==0 || errors!=0) $fatal(1,"source loss must flag overflow, not parity");
        wait(received==100); inject=1;
        wait(injected); wait(errors!=0);
        wait(completed==9);
        rate_start=$time;
        wait(completed==19);
        if($time-rate_start < 3_999_000 || $time-rate_start > 4_001_000)
            $fatal(1,"steady-state throughput is not 2500 sweeps/s: ten sweeps took %0t",$time-rate_start);
        $display("RESULTS: %0d passed, 0 failed",passes+4);
        $display("STATUS: PASS"); $finish;
    end
    initial begin #15_000_000; $fatal(1,"pull link timeout"); end
endmodule
