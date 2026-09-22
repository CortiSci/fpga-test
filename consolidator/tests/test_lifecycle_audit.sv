// Manual historical-oracle repairs for T10; included inside tb_top.
// All FPGA stimulus uses USB/SPI or the external reset pin. No force/release.
// Raw USB capture is used so typed-BFM resynchronization cannot hide reset ACKs.
`ifdef RUN_LIFECYCLE_AUDIT
integer lc_cursor = 0, lc_start = 0, lc_kind = 0, lc_checks = 0;
reg [15:0] lc_words [0:4104];
integer lc_reset_pulses = 0;
reg lc_write_mode = 0;
reg lc_transition = 0;
integer lc_parity_reports = 0;
always @(posedge tail_ch[0].dut_tail.soft_reset) lc_reset_pulses = lc_reset_pulses+1;

task automatic lc_word(output reg [15:0] w);
    integer n;
    begin
        n=0;
        while (u_ft600q.tx_wr_ptr <= lc_cursor && n < 100000) begin
            @(posedge usb_fifo_clk); n=n+1;
        end
        if (u_ft600q.tx_wr_ptr <= lc_cursor) $fatal(1,"LC raw USB timeout cursor=%0d",lc_cursor);
        if (u_ft600q.tx_wr_ptr-lc_cursor >= u_ft600q.CAPN) $fatal(1,"LC capture overrun");
        w=u_ft600q.tx_capture[lc_cursor % u_ft600q.CAPN]; lc_cursor=lc_cursor+1;
    end
endtask

task automatic lc_packet;
    integer i,b,k,len;
    reg [31:0] crc;
    reg [7:0] byteval;
    reg [15:0] incoming;
    begin
        lc_start=lc_cursor; lc_word(incoming); lc_words[0]=incoming;
        if (lc_words[0]===16'h55aa) begin len=4; lc_kind=0; end
        else if (lc_words[0]===16'h0001) begin len=4105; lc_kind=1; end
        else $fatal(1,"LC unexpected packet tag %h at %0d",lc_words[0],lc_start);
        for(i=1;i<len;i=i+1) begin lc_word(incoming); lc_words[i]=incoming; end
        if(lc_kind) begin
            crc=32'hffffffff;
            for(i=0;i<4103;i=i+1) for(b=0;b<2;b=b+1) begin
                byteval=b==0 ? lc_words[i][15:8] : lc_words[i][7:0];
                crc=crc ^ byteval;
                for(k=0;k<8;k=k+1) crc=crc[0] ? (crc>>1)^32'hedb88320 : crc>>1;
            end
            crc=~crc;
            if({lc_words[4103],lc_words[4104]} !== crc) $fatal(1,"LC CRC mismatch");
            if(lc_words[2][15:13] !== 0) $fatal(1,"LC invalid counter reserved bits");
        end
    end
endtask

task automatic lc_cmd(input reg wr,input reg [15:0] addr,input reg [15:0] value);
    integer n;
    reg seen;
    begin
        u_ft600q.send_command_frame(CMD_MAGIC,wr?flags_wr():flags_rd(),addr,value);
        seen=0;
        for(n=0;n<12 && !seen;n=n+1) begin
            lc_packet();
            if(!lc_kind) begin
                if(lc_transition && lc_words[1]===16'hffff && lc_words[2]===16'h0020) begin
                    lc_parity_reports=lc_parity_reports+1;
                    $display("[LC] transition parity report count=%h",lc_words[3]);
                end else begin
                if(lc_words[1] !== (wr?flags_wr():flags_rd()) || lc_words[2] !== addr ||
                   (wr && lc_words[3] !== value))
                    $fatal(1,"LC wrong response %h %h %h",lc_words[1],lc_words[2],lc_words[3]);
                seen=1;
                end
            end
        end
        if(!seen) $fatal(1,"LC no matching command response");
    end
endtask

task automatic lc_cfg(input integer ch,input reg [7:0] opcode,input reg [7:0] value);
    integer n;
    begin
        lc_cmd(1,REG_SPI_CFG_DATA,{opcode,value});
        lc_cmd(1,REG_SPI_CFG_CTRL,spi_cfg_ctrl_word(ch[1:0],lc_write_mode,1'b1,3'd2));
        n=0;
        // Write mode is required on an actively streaming leg.
        do begin
            #30000; lc_cmd(0,REG_SPI_CFG_CTRL,0); n=n+1;
        end while(lc_words[3][3] && n<8);
        if(lc_words[3][3] !== 0) $fatal(1,"LC SPI busy did not clear");
    end
endtask

task automatic lc_setup(input reg bist);
    integer ch;
    begin
        tail_ch[0].asic_model.constant_val=16'haced; tail_ch[0].asic_model.data_mode=MODE_CONSTANT;
        tail_ch[1].asic_model.constant_val=16'haced; tail_ch[1].asic_model.data_mode=MODE_CONSTANT;
        tail_ch[2].asic_model.constant_val=16'haced; tail_ch[2].asic_model.data_mode=MODE_CONSTANT;
        tail_ch[3].asic_model.constant_val=16'haced; tail_ch[3].asic_model.data_mode=MODE_CONSTANT;
        lc_cmd(1,REG_SPI_EN_MASK,15); lc_cmd(1,REG_SPI_CLK_DIV,31);
        for(ch=0;ch<4;ch=ch+1) lc_cfg(ch,1,8'h11);
        lc_cfg(0,6,bist);
        for(ch=0;ch<4;ch=ch+1) lc_cfg(ch,2,1);
        lc_cmd(1,REG_SPI_CLK_DIV,0); lc_cmd(1,REG_ACQ_ALL_RUN,15);
        lc_write_mode=1;
    end
endtask

// Require two fresh, full, CRC-valid frames with valid phases and 1024 checked
// payload words on every leg. ACED is serialized bit-planes, NOT word 0xACED.
// Search one 16-plane phase; all subsequent words must follow that same phase.
task automatic lc_recover(input reg bist,input integer watermark,input reg allow_sticky_overflow);
    integer f,ch,i,off,good,consecutive,match;
    reg [15:0] expected,previous;
    reg [15:0] pattern;
    begin
        consecutive=0; pattern=16'haced;
        for(f=0;f<16 && consecutive<2;f=f+1) begin
            lc_packet(); good=lc_kind && lc_start>=watermark;
            if(!lc_kind) begin
                if(lc_transition && lc_words[1]===16'hffff && lc_words[2]===16'h0020)
                    lc_parity_reports=lc_parity_reports+1;
                else $fatal(1,"LC unexpected response during recovery %h %h %h",
                            lc_words[1],lc_words[2],lc_words[3]);
            end
            if(good) for(ch=0;ch<4;ch=ch+1) begin
                if((lc_words[4099+ch] & (allow_sticky_overflow ? 16'h5000 : 16'h7000)) !== 16'h0000 ||
                   (lc_words[4099+ch] & 16'h03ff)==16'h03ff) good=0;
                if(bist && ch==0) begin
                    previous=lc_words[3+ch];
                    for(i=1;i<1024;i=i+1) begin
                        expected=previous+16'd1;
                        if(lc_words[3+4*i+ch] !== expected) good=0;
                        previous=lc_words[3+4*i+ch];
                    end
                end else begin
                    match=0;
                    for(off=0;off<16;off=off+1) begin : alignment
                        integer bad;
                        bad=0;
                        for(i=0;i<1024;i=i+1) begin
                            expected=pattern[15-((i+off)%16)] ? 16'hffff : 16'h0000;
                            if(lc_words[3+4*i+ch] !== expected) bad=bad+1;
                        end
                        if(bad==0) match=match+1;
                    end
                    if(match!=1) begin
                        good=0;
                        if(f==15) $display("[LC] ASIC mismatch leg=%0d planes=%h %h %h %h match=%0d",ch,lc_words[3+ch],lc_words[7+ch],lc_words[11+ch],lc_words[15+ch],match);
                    end
                end
            end
            $display("[LC] frame=%0d start=%0d watermark=%0d good=%0d phases=%h %h %h %h",f,lc_start,watermark,good,lc_words[4099],lc_words[4100],lc_words[4101],lc_words[4102]);
            if(good) consecutive=consecutive+1; else consecutive=0;
        end
        if(consecutive!=2) $fatal(1,"LC no bounded fresh payload recovery bist=%0d",bist);
        lc_checks=lc_checks+1; $display("[LC] PASS: fresh CRC/phase/full payload recovery bist=%0d",bist);
    end
endtask

task automatic run_lifecycle_audit;
    integer mark,n,partial;
    reg [15:0] w;
    begin
`ifdef LC_SELFTEST_OFF
        lc_setup(1); lc_recover(1,0,0);
        lc_transition=1; lc_cfg(0,6,0); mark=u_ft600q.tx_wr_ptr;
        // A live source-clock switch may lose words during the transition;
        // OVF is sticky until stop/reset. Require fresh complete ASIC payloads
        // and no current PAR/UNDF, not an impossible self-clearing OVF bit.
        lc_recover(0,mark,1);
`elsif LC_TAIL_RESET
        lc_setup(0); lc_recover(0,0,0);
        lc_transition=1; lc_cfg(0,5,0);
        if(lc_reset_pulses != 1) $fatal(1,"LC missing/duplicate tail reset pulse");
        // GSR stub has no silicon reset semantics. Prove opcode delivery and
        // write-only reinitialization; physical GSR is explicitly out of scope.
        // Reset/reinit clears the tail's arm latch. Re-arm its receiver too;
        // leaving CH_RUN continuously high cannot send a new MOSI arm pulse.
        lc_cmd(1,16'h0100,0);
        lc_cfg(0,2,0); lc_cfg(0,1,8'h11); lc_cfg(0,6,0); lc_cfg(0,2,1);
        lc_cmd(1,16'h0100,1);
        mark=u_ft600q.tx_wr_ptr; lc_recover(0,mark,1);
        lc_checks=lc_checks+1; $display("[LC] PASS: reset opcode and write-only reinit");
`elsif LC_CORE_RESET
        lc_setup(0); lc_recover(0,0,0);
        // SYS_CMD_RST is an immediate telemetry abort, unlike graceful RUN=0.
        // Wait for a real partial frame, then capture its entire prefix and ACK.
        n=0;
        while(u_ft600q.tx_wr_ptr-lc_cursor<128 && n<100000) begin
            @(posedge usb_fifo_clk); n=n+1;
        end
        if(n==100000) $fatal(1,"LC no mid-frame reset opportunity");
        u_ft600q.send_command_frame(CMD_MAGIC,flags_wr(),REG_SYS_CMD_RST,1);
        partial=0; lc_word(w);
        if(w !== 16'h0001) $fatal(1,"LC expected interrupted telemetry header");
        partial=1;
        while(w !== RSP_MAGIC && partial<4105) begin lc_word(w); partial=partial+1; end
        if(w !== RSP_MAGIC || partial>=4105) $fatal(1,"LC reset ACK not bounded within aborted frame");
        lc_word(w); if(w !== flags_wr()) $fatal(1,"LC wrong reset ACK flags");
        lc_word(w); if(w !== REG_SYS_CMD_RST) $fatal(1,"LC wrong reset ACK address");
        lc_word(w); if(w !== 1) $fatal(1,"LC wrong reset ACK value");
        $display("[LC] PASS: immediate reset abort prefix=%0d words; complete ACK captured",partial-1);
        lc_checks=lc_checks+1;
        lc_cmd(1,REG_ACQ_ALL_RUN,0); lc_cmd(1,REG_ACQ_ALL_RUN,15);
        mark=u_ft600q.tx_wr_ptr; lc_recover(0,mark,0);
`endif
        lc_cmd(1,REG_ACQ_ALL_RUN,0);
        lc_cmd(0,REG_FW_VERSION,0);
        if(lc_words[3] !== 2) $fatal(1,"LC bad final version response");
        $display("ASSERTIONS: %0d passed, 0 failed",lc_checks);
        $display("TEST FINISHED"); $display("STATUS: PASS");
    end
endtask
`endif
