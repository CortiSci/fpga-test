`timescale 1ns/1ps
// Real buffered tails/receivers, aggregation RAM and USB assembler. One armed
// tail is absent: it must not impose another timeout on every healthy sweep.
module tb_pull_throughput;
    reg clk=0, roclk=0, rst_n=0, start=0;
    reg [3:0] run_mask=4'hf;
    always #10 clk=~clk;
    always #195.3125 roclk=~roclk;
    reg [9:0] source_pos=0;
    reg [3:0] source_bank=0;
    reg source_frame=0;
    reg [13:0] source_data=0;
    always @(negedge roclk) if(rst_n) begin
        source_frame=source_pos==0;
        source_data={source_bank,source_pos};
        source_pos=source_pos+1'b1;
        if(source_pos==0) source_bank=source_bank+1'b1;
    end
    reg [14:0] timer=0;
    reg [28:0] timestamp=0;
    reg epoch=0;
    always @(posedge clk) if(rst_n) begin
        epoch<=timer==19999;
        if(timer==19999) begin timer<=0; timestamp<=timestamp+1'b1; end
        else timer<=timer+1'b1;
    end
    wire [3:0] done,valid,anchored,flush,errors,gaps,empty,full,ovf,overflow,resync,mask;
    wire [19:0] counts;
    wire [15:0] data[0:3];
    genvar i;
    generate for(i=0;i<4;i=i+1) begin: link
        wire ss,mosi,pre;
        reg miso=0;
        if(i<3) begin: present
            asic_pull_tx tail(.ro1_clk(roclk),.rst_n(rst_n),.ro1_sd({2'(i),source_data}),
                .ro1_frame(source_frame),.sclk(~clk),.telem_en(1'b1),.imp_en(1'b0),
                .lane_sel(1'b0),.mosi(mosi),.ss_n(ss),.stream_miso_pre(pre),
                .dropped_sweeps(),.word_toggle());
            always @(negedge clk) miso<=pre;
        end
        spi_ch_pull rx(.sclk(clk),.rst_n(rst_n),.miso_in(miso),.run(run_mask[i]),
            .cfg_hold(1'b0),.resync_req(resync[i]|overflow[i]),.frame_done(done[i]),
            .fifo_count(counts[5*i+:5]),.sclk_d1(),.mosi_out(mosi),.ss_n_out(ss),
            .word_data(data[i]),.word_valid(valid[i]),.raw_word_done(),
            .par_err_flag(errors[i]),.source_gap(gaps[i]),.phase_known(anchored[i]),
            .anchor_flush(flush[i]),.miso_reg());
    end endgenerate
    wire req,tick_valid;
    wire [15:0] td0,td1,td2,td3;
    wire [3:0] undf,fill;
    leg_quad_fifo fifo(clk,rst_n,start,data[0],data[1],data[2],data[3],valid,
        req,td0,td1,td2,td3,tick_valid,undf,fill,full,empty,ovf,counts,overflow,4'b0,mask,flush);
    wire ci,cv,cl,cdone,txvalid;
    wire [15:0] cdata,txdata;
    wire [31:0] crc;
    telem_engine_v3 #(.PULL_LINK(1),.CRC_BYTEWISE(1)) engine(
        .frame_done(done),.clk(clk),.rst_n(rst_n),.telem_start(start),.sw_reset(1'b0),
        .run_any(1'b1),.ext_frame_cnt(timestamp),.frame_aligned(epoch),
        .tick_req(req),.tick_data_0(td0),.tick_data_1(td1),.tick_data_2(td2),.tick_data_3(td3),
        .tick_valid(tick_valid),.tick_undf(undf),.fill_ge4(fill),.leg_anchored(anchored),
        .leg_empty(empty),.legs_armed(run_mask),.rd_mask(mask),.leg_resync(resync),
        .crc_init(ci),.crc_valid(cv),.crc_last(cl),.crc_data(cdata),.crc_result(crc),
        .crc_result_valid(cdone),.telem_tx_data(txdata),.telem_tx_valid(txvalid),
        .telem_tx_ready(1'b1),.drop_frame(1'b0),.par_err_in(errors),.ovfl_in(ovf|gaps),.framer_busy());
    crc32_serial checksum(clk,rst_n,ci,cv,cl,cdata,crc,cdone);
    integer pos=0,frames=0,passes=0,ch,plane;
    reg [3:0] bank[0:2], previous_bank[0:2];
    reg [31:0] expected_crc;
    integer previous_timestamp=-1;
    function automatic [31:0] crc_word(input [31:0] seed,input [15:0] value);
        reg [31:0] c;
        begin
            c=seed;
            for(integer octet=1;octet>=0;octet--)
                for(integer bitno=0;bitno<8;bitno++)
                    c=(c>>1)^((c[0]^value[octet*8+bitno]) ? 32'hedb88320 : 32'b0);
            crc_word=c;
        end
    endfunction
    time rate_start;
    always @(negedge clk) if(txvalid) begin
        if(pos==0) begin
            if(txdata!==16'h0001) $fatal(1,"bad telemetry header");
            expected_crc=32'hffffffff;
        end
        if(pos==1) begin
            if(previous_timestamp>=0 && txdata<=previous_timestamp) $fatal(1,"duplicate or backward timestamp");
            previous_timestamp=txdata;
        end
        if(pos<4103) expected_crc=crc_word(expected_crc,txdata);
        if(pos==4103 && txdata!==~expected_crc[31:16]) $fatal(1,"CRC high mismatch");
        if(pos==4104 && txdata!==~expected_crc[15:0]) $fatal(1,"CRC low mismatch");
        if(pos>=3 && pos<4099) begin
            ch=(pos-3)%4; plane=(pos-3)/4;
            if(ch==3) begin
                if(txdata!==0) $fatal(1,"absent leg supplied data");
            end else if(!(frames==13 && ch==0 && plane>=100)) begin
                if(plane==0) bank[ch]=txdata[13:10];
                if(txdata[15:14]!==2'(ch) || txdata[9:0]!==10'(plane) || txdata[13:10]!==bank[ch])
                    $fatal(1,"mixed or rotated leg %0d plane %0d: %h",ch,plane,txdata);
            end
            passes++;
        end
        if(pos>=4099 && pos<=4101) begin
            ch=pos-4099;
            if(frames==13 && ch==0) begin
                if(!txdata[12] || txdata[9:0]!==10'h3ff) $fatal(1,"stopped partial bank not flagged");
            end else if(txdata[9:0]!==0 || txdata[14]) $fatal(1,"healthy leg lost alignment");
            if(frames>0 && bank[ch]!=(previous_bank[ch]+4'd1) && !txdata[13])
                $fatal(1,"unreported source-bank gap");
            previous_bank[ch]=bank[ch];
        end
        if(frames==13 && pos==403) run_mask[0]=0;
        if(pos==4102 && txdata!==16'h13ff) $fatal(1,"absent leg not flagged: %h",txdata);
        if(pos==4104) begin
            pos=0; frames++;
            if(frames==3) rate_start=$time;
            if(frames==13) begin
                if($time-rate_start<3_999_000 || $time-rate_start>4_001_000)
                    $fatal(1,"dead leg throttles healthy capture: ten frames took %0t",$time-rate_start);
            end
            if(frames==14) begin
                $display("RESULTS: %0d passed, 0 failed",passes+2);
                $display("STATUS: PASS");$finish;
            end
        end else pos++;
    end
    initial begin #100; rst_n=1; @(negedge clk); start=1; @(negedge clk); start=0; end
    initial begin #20_000_000; $fatal(1,"throughput timeout"); end
endmodule
