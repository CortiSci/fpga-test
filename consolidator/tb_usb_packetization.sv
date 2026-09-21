`timescale 1ns/1ps
// Regression for the reproduced partial-packet pauses, registered in CI.
// Real leg FIFO, telemetry engine, CRC, TX CDC and phased USB mover. Physical
// 50/66.667 MHz clocks, normal 2.56 Mword/s legs, LEG7 inactive. No host limit.
module tb_usb_packetization;
    reg clk=0, usb_clk=0, rst_n=0, start=0;
    always #10 clk=~clk;
    always #7.5 usb_clk=~usb_clk;
    reg [3:0] live=4'hb;
    reg [3:0] wd_valid=0;
    reg [15:0] wd0=0, wd1=0, wd2=0, wd3=0;
    integer source_words=0, divider=0;
    reg epoch=0;
    // Exercise the problematic packet phase without simulating 32 sweeps.
    // Queue a 240-word prefix at the public TX FIFO input before acquisition;
    // USB packet boundaries need not coincide with telemetry frame boundaries.
    localparam PREFIX_WORDS=240;
    reg prefix_valid=0, source_enabled=0;
    integer prefix_sent=0, prefix_received=0;
    wire rd_en,wr_n,rd_n,oe_n,pop;
    wire [28:0] frame_count=source_words/1024;
    wire tick_req,tick_valid;
    wire [15:0] td0,td1,td2,td3;
    wire [3:0] undf,fill,full,empty,ovf,ovf_pulse,rd_mask,resync;
    wire [19:0] counts;
    wire [15:0] tx_data,usb_data,crc_data,fifo_data;
    wire tx_valid,tx_ready,crc_init,crc_valid,crc_last,crc_done,busy;
    wire [31:0] crc_result;
    wire cdc_full,cdc_almost,cdc_half,cdc_empty;
    wire [9:0] rd_level;
    integer passes=0,failures=0,frames=0,pos=0,errors=0,gaps=0,last_count=-1;
    integer ch,tick,base_seq=0,current_count=0;
    reg [31:0] checksum=32'hffffffff;

    leg_quad_fifo fifo(
        .clk(clk),.rst_n(rst_n),.sync_rst(1'b0),
        .wd_data_0(wd0),.wd_data_1(wd1),.wd_data_2(wd2),.wd_data_3(wd3),.wd_valid(wd_valid),
        .tick_req(tick_req),.tick_data_0(td0),.tick_data_1(td1),.tick_data_2(td2),.tick_data_3(td3),
        .tick_valid(tick_valid),.tick_undf(undf),.fill_ge4(fill),.fifo_full(full),.fifo_empty(empty),
        .fifo_ovf(ovf),.fifo_word_cnt(counts),.ovf_pulse(ovf_pulse),
        .ch_local_rst(4'b0),.rd_mask(rd_mask),.flush(4'b0));
    telem_engine_v3 engine(
        .clk(clk),.rst_n(rst_n),.telem_start(start),.sw_reset(1'b0),.run_any(1'b1),
        .ext_frame_cnt(frame_count),.frame_aligned(epoch),
        .tick_req(tick_req),.tick_data_0(td0),.tick_data_1(td1),.tick_data_2(td2),.tick_data_3(td3),
        .tick_valid(tick_valid),.tick_undf(undf),.fill_ge4(fill),
        .leg_anchored(live),.leg_empty(empty),.legs_armed(live),.rd_mask(rd_mask),.leg_resync(resync),
        .crc_init(crc_init),.crc_valid(crc_valid),.crc_last(crc_last),.crc_data(crc_data),
        .crc_result(crc_result),.crc_result_valid(crc_done),
        .telem_tx_data(tx_data),.telem_tx_valid(tx_valid),.telem_tx_ready(tx_ready),
        .drop_frame(cdc_half),
        .par_err_in(4'b0),.ovfl_in(ovf),.framer_busy(busy));
    crc32 crc(.clk(clk),.rst_n(rst_n),.init(crc_init),.valid(crc_valid),.last(crc_last),
              .data_in(crc_data),.crc_out(crc_result),.crc_valid(crc_done));
    assign tx_ready=~cdc_almost;
    telem_tx_fifo output_fifo(
        .wr_clk(clk),.wr_rst_n(rst_n),
        .wr_data(prefix_valid ? (16'h8000 + prefix_sent[15:0]) : tx_data),
        .wr_en((prefix_valid || tx_valid) && !cdc_full),
        .wr_full(cdc_full),.wr_almost_full(cdc_almost),
        .wr_half_full(cdc_half),
        .rd_clk(usb_clk),.rd_rst_n(rst_n),.rd_data(fifo_data),.rd_en(pop && !cdc_empty),
        .rd_empty(cdc_empty),.rd_level(rd_level));

    // Preserve the physical 2.56 Mword/s source at a 50 MHz fabric clock:
    // 32 words per 625 cycles. Word identity
    // encodes the sequence and leg, so dropped/repeated/interleaved words cannot
    // accidentally pass a constant-pattern comparison.
    always @(negedge clk) begin
        if (!rst_n) begin source_words=0; divider=0; wd_valid=0; epoch=0; end
        else if (source_enabled) begin
            wd_valid=0; epoch=0;
            divider=divider+32;
            if (divider>=625) begin
                divider=divider-625;
                wd0=source_words; wd1=source_words ^ 16'h4000;
                wd2=source_words ^ 16'h8000; wd3=source_words ^ 16'hc000;
                wd_valid=live;
                epoch=(source_words % 1024)==0;
                source_words=source_words+1;
            end
        end
    end
    always @(posedge clk) if (rst_n && prefix_valid && !cdc_full)
        prefix_sent <= prefix_sent+1;
    wire launch_clk; assign #2.5 launch_clk=usb_clk;
    reg [1:0] busy_sync=0;
    always @(posedge usb_clk or negedge rst_n)
        if (!rst_n) busy_sync<=0; else busy_sync<={busy_sync[0],busy};
    ft600_245_fifo_fsm #(.PHASED_OUTPUT(1)) mover(
        .usb_fifo_clk(usb_clk),.usb_launch_clk(launch_clk),.devrst_n(rst_n),
        .usb_fifo_d(usb_data),.usb_fifo_rxf_n(1'b1),.usb_fifo_txe_n(1'b0),
        .usb_fifo_rd_n(rd_n),.usb_fifo_wr_n(wr_n),.usb_fifo_oe_n(oe_n),
        .cmd_out_data(),.cmd_out_valid(),.cmd_out_ready(1'b1),
        .tx_in_data(fifo_data),.tx_in_valid(!cdc_empty),.tx_in_ready(pop),
        .tx_in_queued_words(rd_level),.telemetry_active(1'b1),
        .tx_source_idle(~busy_sync[1]),.dbg_state());
    assign rd_en=!wr_n; // TXE stays ready: no fabricated USB bandwidth limit.
    integer burst_words=0,burst_count=0,burst_sum=0,midframe_short=0,tail_short=0;
    integer wire_words=0,burst_min=1000000,burst_max=0;
    integer histogram[0:16384];
    // Count words accepted on the actual pins. A gap after a non-full USB
    // packet is a potential short transfer; this monitor does not simulate
    // host scheduling or assert that each pin burst equals a driver completion.
    always @(posedge usb_clk) if(rst_n) begin
        if(!wr_n) begin burst_words++; wire_words++; end
        else if(burst_words!=0) begin
            if(frames>=2) begin
                burst_count++; burst_sum+=burst_words;
                if(burst_words<burst_min) burst_min=burst_words;
                if(burst_words>burst_max) burst_max=burst_words;
                if(burst_words<=16384) histogram[burst_words]++;
                if(burst_words%256!=0) begin
                    if(midframe_short+tail_short<6)
                        $display("[PACKET] partial bytes=%0d end_frame_word=%0d source_idle=%b source_words=%0d time_ns=%0f",
                            2*burst_words,(wire_words-PREFIX_WORDS)%4105,~busy_sync[1],source_words,$realtime);
                    if((wire_words-PREFIX_WORDS)%4105!=0) midframe_short++;
                    else tail_short++;
                end
            end
            burst_words=0;
        end
    end
    function automatic [31:0] crc_word(input [31:0] seed,input [15:0] word);
        reg [31:0] c;
        integer octet,bitno;
        begin
            c=seed;
            for (octet=1;octet>=0;octet=octet-1)
                for (bitno=0;bitno<8;bitno=bitno+1)
                    if (c[0]^word[octet*8+bitno]) c=(c>>1)^32'hedb88320;
                    else c=c>>1;
            crc_word=c;
        end
    endfunction
    // Score only public USB words, not internal discard/state signals.
    always @(posedge usb_clk) if (rst_n && rd_en) begin
        if (prefix_received<PREFIX_WORDS) begin
            if (usb_data !== (16'h8000 + prefix_received[15:0])) errors++;
            prefix_received++;
        end else begin
        if (pos==0) begin
            checksum=32'hffffffff;
            if (usb_data!==16'h0001) errors=errors+1;
        end
        if (pos==1) begin
            current_count=usb_data;
            if (last_count>=0) begin
                if (current_count<=last_count) errors=errors+1;
                if (current_count>last_count+1) gaps=gaps+current_count-last_count-1;
            end
            last_count=current_count;
        end
        if (pos==2 && usb_data!==0) errors=errors+1;
        if (pos>=3 && pos<4099) begin
            ch=(pos-3)%4; tick=(pos-3)/4;
            if (pos==3) begin
                base_seq=usb_data;
                if (base_seq%1024!=0) errors=errors+1;
            end
            if (live[ch]) begin
                if (usb_data!==((base_seq+tick) ^ (ch*16'h4000))) errors=errors+1;
            end else if (usb_data!==0) errors=errors+1;
        end
        if (pos>=4099 && pos<=4102) begin
            ch=pos-4099;
            if (live[ch]) begin
                if (usb_data!==0) errors=errors+1; // no overflow, underrun, parity or rotation
            end else if (usb_data!==16'h03ff) errors=errors+1;
        end
        if (pos<=4102) checksum=crc_word(checksum,usb_data);
        if (pos==4103 && usb_data!==(~checksum[31:16])) errors=errors+1;
        if (pos==4104 && usb_data!==(~checksum[15:0])) errors=errors+1;
        if (pos==4104) begin frames=frames+1; pos=0; end
        else pos=pos+1;
        end
    end
    task automatic check(input bit ok,input string label);
        if (ok) begin passes++; $display("[PACKET] PASS %s",label); end
        else begin failures++; $display("[PACKET] FAIL %s (errors=%0d gaps=%0d frames=%0d)",label,errors,gaps,frames); end
    endtask
    integer k;
    initial begin
        for(k=0;k<=16384;k++) histogram[k]=0;
        repeat(5) @(negedge clk);
        rst_n=1; prefix_valid=1;
        wait(prefix_sent==PREFIX_WORDS);
        @(negedge clk); prefix_valid=0; source_enabled=1; start=1;
        @(negedge clk); start=0;
        wait(frames>=8);
        @(negedge usb_clk);
        check(errors==0,"accepted words retain payload identity, phase and CRC");
        check(gaps==0,"unlimited sink preserves all frame counters");
        check(burst_count>20,"enough bursts observed after startup");
        check(midframe_short==0,"no avoidable partial-packet pauses inside telemetry frames");
        $display("[PACKET] bursts=%0d mean_bytes=%0f min_bytes=%0d max_bytes=%0d midframe_short=%0d frame_tail_short=%0d",
            burst_count,2.0*burst_sum/burst_count,2*burst_min,2*burst_max,midframe_short,tail_short);
        for(k=1;k<=16384;k++) if(histogram[k])
            $display("[PACKET] histogram bytes=%0d count=%0d",2*k,histogram[k]);
        $display("RESULTS: %0d passed, %0d failed",passes,failures);
        $display("STATUS: %s",failures==0 ? "PASS" : "FAIL");
        if(failures) $fatal(1,"USB packetization/integrity regression");
        $finish;
    end
    initial begin #5000000; $fatal(1,"packet audit timeout"); end
endmodule
