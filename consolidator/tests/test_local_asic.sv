// Full USB -> consolidator routing -> tail-local ASIC pins -> USB status.
`ifdef RUN_LOCAL_ASIC
integer la_pass=0;
integer la_edges[0:3],la_done[0:3];
reg [23:0] la_seen[0:3],la_reply[0:3];
reg [23:0] la_expected;
integer la_channel,la_select;
task la_check(input bit ok,input string why);
    if(!ok) $fatal(1,"FAIL: %s",why); la_pass++;
endtask
genvar la_g;
generate for(la_g=0;la_g<4;la_g++) begin: local_pin_monitor
    initial begin la_edges[la_g]=0;la_done[la_g]=0;la_seen[la_g]=0;la_reply[la_g]=0;end
    always @(posedge spi_ro_sclk[la_g]) begin
        la_check(la_channel==la_g,"ASIC clocks routed only to selected tail");
        la_check({spi_ro_ss1[la_g],spi_ro_ss0[la_g]}==(la_select ? 2'b01:2'b10),"ASIC selector delivered");
        la_seen[la_g]={la_seen[la_g][22:0],spi_ro_mosi[la_g]};la_edges[la_g]++;
        la_reply[la_g]={la_reply[la_g][22:0],spi_ro_miso[la_g]};
    end
    always @(posedge spi_ro_ss0[la_g] or posedge spi_ro_ss1[la_g]) if(la_edges[la_g]>0) begin
        la_check(la_edges[la_g]==24,"exactly 24 local ASIC clocks");
        la_check(la_seen[la_g]==la_expected,"exact ASIC payload from USB request");
        la_edges[la_g]=0;la_done[la_g]++;
    end
end endgenerate
initial begin: local_asic_sequence
    reg [15:0] m,f,a,d;
    reg [47:0] rx,tx;
    @(posedge devrst_n);#1000;
`ifdef RUN_LOCAL_ASIC_FAST
    for(integer la_div=0;la_div<3;la_div++) begin
    u_ft600q.send_command_frame(CMD_MAGIC,flags_wr(),REG_SPI_CLK_DIV,la_div[15:0]);
`else
    for(integer la_div=0;la_div<1;la_div++) begin
    u_ft600q.send_command_frame(CMD_MAGIC,flags_wr(),REG_SPI_CLK_DIV,16'd31);
`endif
    u_ft600q.wait_response_frame_typed(m,f,a,d);
    u_ft600q.send_command_frame(CMD_MAGIC,flags_wr(),REG_SPI_EN_MASK,16'hf);
    u_ft600q.wait_response_frame_typed(m,f,a,d);
    for(la_channel=0;la_channel<4;la_channel++) begin
        spi_cfg_xact(2'(la_channel),48'h070000000000,3,rx);
        la_check(rx[31:24]==8'h1f,"local ASIC capability exposed through USB");
        for(la_select=0;la_select<2;la_select++) begin
            la_expected=24'h95ac31 ^ (la_channel<<8) ^ la_select;
            tx={8'h0a,7'b0,la_select[0],la_expected,8'b0};
            spi_local_asic_write(2'(la_channel),la_select[0],la_expected);
            #40000;
            spi_cfg_xact(2'(la_channel),48'h0b0000000000,3,rx);
            la_check(rx[31:24]==2,"completion status through USB");
            la_check(la_done[la_channel]==la_div*2+la_select+1,"one execution per command");
            for(integer b=1;b<4;b++) begin
                tx={8'h0b,b[7:0],32'b0};spi_cfg_xact(2'(la_channel),tx,3,rx);
                // Independent pin capture, including the model's first-bit preload.
                la_check(rx[31:24]==((la_reply[la_channel] >> (8*(3-b))) & 255),"ASIC readback bytes routed to USB");
            end
        end
    end
    end
    $display("RESULTS: %0d passed, 0 failed",la_pass);$display("STATUS: PASS");$finish;
end
initial begin #10000000;$fatal(1,"local ASIC USB timeout");end
`endif
