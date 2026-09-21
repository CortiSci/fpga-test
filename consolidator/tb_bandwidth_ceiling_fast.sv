`timescale 1ns/1ps
// Focused bandwidth regression: real leg FIFO, assembler, CRC and asynchronous
// output FIFO; behavioral word producers and smooth USB drain. No DUT forces.
// Input 20.5 MB/s; host ceiling 20.1 MB/s, or the recorded 14 MB/s.
// Unlike the extended board bench,
// there is no 2048-word FT600 credit or SPI/ASIC setup to simulate.
module tb_bandwidth_ceiling_fast;
`ifdef RECORDED_DEFICIT
    localparam DRAIN_RATE=105, DEFICIT_END=6, RECOVERY_END=8;
`else
    // The 512-word admission watermark absorbs one more frame's deficit;
    // observe its eventual counter gap before restoring the unlimited sink.
    localparam DRAIN_RATE=151, DEFICIT_END=11, RECOVERY_END=13;
`endif
    reg clk=0, usb_clk=0, rst_n=0, start=0;
    always #10 clk=~clk;
    always #7.5 usb_clk=~usb_clk;
    reg [3:0] live=4'hb;
    reg [3:0] wd_valid=0;
    reg [15:0] wd0=0, wd1=0, wd2=0, wd3=0;
    integer source_words=0, divider=0, drain_rate=1000, credit=0;
    reg epoch=0, rd_en=0;
    wire [28:0] frame_count=source_words/1024;
    wire tick_req,tick_valid;
    wire [15:0] td0,td1,td2,td3;
    wire [3:0] undf,fill,full,empty,ovf,ovf_pulse,rd_mask,resync;
    wire [19:0] counts;
    wire [15:0] tx_data,usb_data,crc_data;
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
        .leg_anchored(live),.leg_empty(empty),.legs_armed(4'hf),.rd_mask(rd_mask),.leg_resync(resync),
        .crc_init(crc_init),.crc_valid(crc_valid),.crc_last(crc_last),.crc_data(crc_data),
        .crc_result(crc_result),.crc_result_valid(crc_done),
        .telem_tx_data(tx_data),.telem_tx_valid(tx_valid),.telem_tx_ready(tx_ready),
`ifndef CEILING_LEGACY_ENGINE
        .drop_frame(cdc_half),
`endif
        .par_err_in(4'b0),.ovfl_in(ovf),.framer_busy(busy));
    crc32 crc(.clk(clk),.rst_n(rst_n),.init(crc_init),.valid(crc_valid),.last(crc_last),
              .data_in(crc_data),.crc_out(crc_result),.crc_valid(crc_done));
    assign tx_ready=~cdc_almost;
    `ifdef CEILING_LEGACY_BUFFER
    cdc_fifo #(.WIDTH(16),.DEPTH_LOG2(9)) output_fifo(
`else
    telem_tx_fifo output_fifo(
`endif
        .wr_clk(clk),.wr_rst_n(rst_n),.wr_data(tx_data),.wr_en(tx_valid && !cdc_full),
        .wr_full(cdc_full),.wr_almost_full(cdc_almost),
`ifndef CEILING_LEGACY_ENGINE
        .wr_half_full(cdc_half),
`endif
        .rd_clk(usb_clk),.rd_rst_n(rst_n),.rd_data(usb_data),.rd_en(rd_en),
        .rd_empty(cdc_empty),.rd_level(rd_level));

    // Preserve the physical 2.56 Mword/s source at a 50 MHz fabric clock:
    // 32 words per 625 cycles. Word identity
    // encodes the sequence and leg, so dropped/repeated/interleaved words cannot
    // accidentally pass a constant-pattern comparison.
    always @(negedge clk) begin
        if (!rst_n) begin source_words=0; divider=0; wd_valid=0; epoch=0; end
        else begin
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
    // No initial host credit: exercise the same steady deficit immediately.
    always @(negedge usb_clk) begin
        if (!rst_n) begin credit=0; rd_en=0; end
        else begin
            credit=credit+drain_rate;
            if (credit>2000) credit=2000;
            rd_en=!cdc_empty && credit>=1000;
            if (rd_en) credit=credit-1000;
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
    always @(posedge usb_clk) if (rst_n && rd_en && !cdc_empty) begin
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
            end else if (usb_data!==16'h13ff) errors=errors+1;
        end
        if (pos<=4102) checksum=crc_word(checksum,usb_data);
        if (pos==4103 && usb_data!==(~checksum[31:16])) errors=errors+1;
        if (pos==4104 && usb_data!==(~checksum[15:0])) errors=errors+1;
        if (pos==4104) begin frames=frames+1; pos=0; end
        else pos=pos+1;
    end
    task automatic check(input bit ok,input string label);
        if (ok) begin passes++; $display("[CEILING] PASS %s",label); end
        else begin failures++; $display("[CEILING] FAIL %s (errors=%0d gaps=%0d frames=%0d)",label,errors,gaps,frames); end
    endtask
    task automatic scenario(input [3:0] mask);
        integer initial_gaps;
        begin
            @(negedge clk); rst_n=0; start=0; live=mask; drain_rate=1000;
            repeat(4) @(negedge clk);
            frames=0; pos=0; errors=0; gaps=0; last_count=-1;
            @(posedge clk); #1; rst_n=1; start=1;
            @(posedge clk); #1; start=0;
            wait(frames>=3);
            check(errors==0,"unlimited host: complete ordered frames, phase flags and CRC");
            initial_gaps=gaps;
            drain_rate=DRAIN_RATE;
            wait(frames>=DEFICIT_END);
            check(errors==0,"bandwidth deficit: every delivered frame intact, dead leg flagged, CRC correct");
            check(gaps>initial_gaps,"bandwidth deficit: omitted input frames exposed by counter gaps");
            drain_rate=1000;
            wait(frames>=RECOVERY_END);
            check(errors==0,"unlimited host restored: clean recovery");
            $display("[CEILING] mask=%h delivered=%0d gaps=%0d errors=%0d",mask,frames,gaps,errors);
        end
    endtask
    initial begin
        scenario(4'hb); scenario(4'hf);
        $display("RESULTS: %0d passed, %0d failed",passes,failures);
        $display("STATUS: %s",failures==0 ? "PASS" : "FAIL");
        $finish;
    end
    initial begin #30_000_000; $fatal(1,"bandwidth ceiling timeout"); end
endmodule
