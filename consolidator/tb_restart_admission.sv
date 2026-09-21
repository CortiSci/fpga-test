`timescale 1ns/1ps
// Hardware-inspired T03 reproduction, 2026-09-21: good transport counters do
// not imply valid data on every enabled leg. Production FIFO/assembler/CRC;
// receiver outputs modeled at their public word/anchor/flush interface.
// No forced DUT state, USB stalls, bad parity, or source-rate deficit.
// Stop/start uses telem_start and FIFO sync_rst, not a DUT power reset.
module tb_restart_admission;
    reg clk=0, rst_n=1, start=0, running=0;
    always #10 clk=~clk; // production 50 MHz
    wire [3:0] live=4'hb; // LEG7 deliberately excluded
    reg [3:0] wd_valid=0, anchored=0, flush=0, pending=0;
    reg [15:0] wd0=0,wd1=0,wd2=0,wd3=0,data[0:3];
    integer cycles=0, delay_cycles=0, c, seq[0:3], credit_words[0:3];
    reg epoch=0;
    wire [28:0] frame_count=cycles/20000;
    wire tick_req,tick_valid;
    wire [15:0] td0,td1,td2,td3;
    wire [3:0] undf,fill,full,empty,ovf,ovf_pulse,rd_mask,resync;
    wire [19:0] counts;
    wire [15:0] tx_data,crc_data;
    wire tx_valid,tx_ready,crc_init,crc_valid,crc_last,crc_done,busy;
    wire [31:0] crc_result;
    integer passes=0,failures=0,frames=0,pos=0,ch,tick;
    integer wire_errors=0,gaps=0,last_count=-1,current_count;
    integer phase_bad[0:3],clean_frames[0:3],payload_bad[0:3];
    reg [15:0] frame_words[0:4095],last_word[0:3];
    reg [3:0] have_last=0;
    reg [31:0] checksum;
    leg_quad_fifo fifo(
        .clk(clk),.rst_n(rst_n),.sync_rst(start),
        .wd_data_0(wd0),.wd_data_1(wd1),.wd_data_2(wd2),.wd_data_3(wd3),.wd_valid(wd_valid),
        .tick_req(tick_req),.tick_data_0(td0),.tick_data_1(td1),.tick_data_2(td2),.tick_data_3(td3),
        .tick_valid(tick_valid),.tick_undf(undf),.fill_ge4(fill),.fifo_full(full),.fifo_empty(empty),
        .fifo_ovf(ovf),.fifo_word_cnt(counts),.ovf_pulse(ovf_pulse),
        .ch_local_rst(4'b0),.rd_mask(rd_mask),.flush(flush));
    telem_engine_v3 engine(
        .clk(clk),.rst_n(rst_n),.telem_start(start),.sw_reset(1'b0),.run_any(running),
        .ext_frame_cnt(frame_count),.frame_aligned(epoch),
        .tick_req(tick_req),.tick_data_0(td0),.tick_data_1(td1),.tick_data_2(td2),.tick_data_3(td3),
        .tick_valid(tick_valid),.tick_undf(undf),.fill_ge4(fill),
        .leg_anchored(anchored),.leg_empty(empty),.legs_armed(live & {4{running}}),.rd_mask(rd_mask),.leg_resync(resync),
        .crc_init(crc_init),.crc_valid(crc_valid),.crc_last(crc_last),.crc_data(crc_data),
        .crc_result(crc_result),.crc_result_valid(crc_done),
        .telem_tx_data(tx_data),.telem_tx_valid(tx_valid),.telem_tx_ready(tx_ready),
        .drop_frame(1'b0),
        .par_err_in(4'b0),.ovfl_in(ovf),.framer_busy(busy));
    crc32 crc(.clk(clk),.rst_n(rst_n),.init(crc_init),.valid(crc_valid),.last(crc_last),
              .data_in(crc_data),.crc_out(crc_result),.crc_valid(crc_done));


    assign tx_ready=1; // always-ready host: no congestion excuse
    // Three independently started BIST-like producers: 32 words / 625 fabric
    // cycles = 2.56 Mword/s each. LEG6 starts one sweep after LEG5, LEG8 three
    // sweeps later, as with sequential host commands. Only their sub-word
    // start offset changes across scenarios: zero versus ten cycles (200 ns).
    // On resync, suppress delivery until the next 1024-word sweep marker,
    // flush before delivering its word zero, just like spi_ch_stream.
    always @(negedge clk) begin
        wd_valid=pending; pending=0; flush=0; epoch=0;
        if (running && !start) begin
            cycles=cycles+1;
            epoch=(cycles%20000)==0;
            for(c=0;c<4;c=c+1) begin
                if(ovf_pulse[c] || resync[c]) anchored[c]=0;
                if(live[c] && cycles > (c==0 ? 100 : 100+delay_cycles+c*20000)) begin
                    credit_words[c]=credit_words[c]+32;
                    if(credit_words[c]>=625) begin
                        credit_words[c]=credit_words[c]-625;
                        if(seq[c]%1024==0 && !anchored[c]) begin
                            flush[c]=1; anchored[c]=1;
                        end
                        if(anchored[c]) begin data[c]=seq[c]; pending[c]=1; end
                        seq[c]=seq[c]+1;
                    end
                end
            end
            wd0=data[0];wd1=data[1];wd2=data[2];wd3=data[3];
        end else begin
            wd_valid=0; pending=0; anchored=0;
        end
    end
    function automatic [31:0] crc_word(input [31:0] seed,input [15:0] word);
        reg [31:0] v;
        integer octet,b;
        begin
            v=seed;
            for(octet=1;octet>=0;octet=octet-1)
                for(b=0;b<8;b=b+1)
                    if(v[0]^word[octet*8+b]) v=(v>>1)^32'hedb88320;
                    else v=v>>1;
            crc_word=v;
        end
    endfunction
    // Judge serialized frame contents, never DUT state/counter internals.
    // Five settling frames include the intentionally late LEG8 start. Then
    // demand four consecutive clean complete frames on EACH expected leg.
    always @(posedge clk) if(rst_n && running && tx_valid) begin
        if(pos==0) begin
            checksum=32'hffffffff;
            if(tx_data!==16'h0001) wire_errors=wire_errors+1;
        end
        if(pos==1) begin
            current_count=tx_data;
            if(frames>=5 && current_count!=last_count+1) gaps=gaps+1;
            last_count=current_count;
        end
        if(pos==2 && tx_data!==0) wire_errors=wire_errors+1;
        if(pos>=3 && pos<4099) frame_words[pos-3]=tx_data;
        if(pos>=4099 && pos<=4102) begin
            ch=pos-4099;
            if(frames>=5) begin
                if(live[ch]) begin
                    if((tx_data & 16'h7000)!==0 || tx_data[9:0]>63) begin
                        phase_bad[ch]=phase_bad[ch]+1; have_last[ch]=0;
                    end else begin
                        clean_frames[ch]=clean_frames[ch]+1;
                        // Phase identifies the group at slot zero, not an
                        // arbitrary offset selected by the checker to fit data.
                        if((frame_words[ch] & 1023)!==(tx_data[5:0]*16)) payload_bad[ch]=payload_bad[ch]+1;
                        if(have_last[ch] && frame_words[ch]!==((last_word[ch]+1)&16'hffff)) payload_bad[ch]=payload_bad[ch]+1;
                        for(tick=1;tick<1024;tick=tick+1)
                            if(frame_words[4*tick+ch]!==((frame_words[4*(tick-1)+ch]+1)&16'hffff)) payload_bad[ch]=payload_bad[ch]+1;
                        last_word[ch]=frame_words[4*1023+ch]; have_last[ch]=1;
                    end
                end else begin
                    if(tx_data!==16'h03ff) wire_errors=wire_errors+1;
                    for(tick=0;tick<1024;tick=tick+1)
                        if(frame_words[4*tick+ch]!==0) wire_errors=wire_errors+1;
                end
            end
        end
        if(pos<=4102) checksum=crc_word(checksum,tx_data);
        if(pos==4103 && tx_data!==(~checksum[31:16])) wire_errors=wire_errors+1;
        if(pos==4104 && tx_data!==(~checksum[15:0])) wire_errors=wire_errors+1;
        if(pos==4104) begin frames=frames+1;pos=0;end else pos=pos+1;
    end
    task automatic check(input bit ok,input string label);
        if(ok) begin passes++; $display("[RESTART] PASS %s",label);end
        else begin failures++; $display("[RESTART] FAIL %s",label);end
    endtask
    task automatic scenario(input integer offset,input string label);
        integer leg;
        begin
            // Previous scenario ended at a complete frame boundary. Stop and
            // reset source configuration; then issue the production start pulse.
            @(negedge clk); #1; running=0;
            repeat(40) @(negedge clk);
            #1;
            cycles=0;delay_cycles=offset;frames=0;pos=0;wire_errors=0;
            gaps=0;last_count=-1;have_last=0;
            for(leg=0;leg<4;leg=leg+1) begin
                seq[leg]=0;credit_words[leg]=0;data[leg]=0;
                phase_bad[leg]=0;clean_frames[leg]=0;payload_bad[leg]=0;
            end
            start=1;running=1;
            @(negedge clk); #1;start=0;
            wait(frames>=9);
            $display("[RESTART] %s offset=%0d cycles frames=%0d",label,offset,frames);
            $display("[RESTART] wire_errors=%0d counter_gaps=%0d",wire_errors,gaps);
            check(wire_errors==0 && gaps==0,"transport structure, CRC, frame counters and inactive LEG7 slot");
            for(leg=0;leg<4;leg=leg+1) if(live[leg]) begin
                $display("[RESTART] LEG%0d clean=%0d flagged=%0d payload_errors=%0d",leg+5,clean_frames[leg],phase_bad[leg],payload_bad[leg]);
                check(clean_frames[leg]==4 && phase_bad[leg]==0 && payload_bad[leg]==0,
                    $sformatf("%s LEG%0d: four clean correctly phased counter sweeps",label,leg+5));
            end
        end
    endtask
    initial begin
        #1;rst_n=0;#100;rst_n=1;
        scenario(0,"cold control");
        scenario(10,"restart at shifted source phase");
        scenario(0,"restart at control phase");
        $display("RESULTS: %0d passed, %0d failed",passes,failures);
        $display("STATUS: %s",failures==0 ? "PASS" : "FAIL");
        if(failures) $fatal(1,"restart admission regression");
        $finish;
    end
    initial begin #15000000; $fatal(1,"restart admission timeout");end
endmodule
