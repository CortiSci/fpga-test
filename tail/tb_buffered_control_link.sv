`timescale 1ns/1ps
module tb_buffered_control_link;
    reg clk=0,rst=0,sclk=0,ss=0,mosi=0;
    wire miso,ct,pet,clear,fault,ro,led,test_sig,shdn,mclk;
    wire [4:0] controls;
    integer passes=0;
    always #24.414 clk=~clk;
`ifdef PULL_LINK
    localparam MODE=2, CAPABILITIES=31;
`else
    localparam MODE=1, CAPABILITIES=7;
`endif
    spi_slave_small #(.EXTENDED_CONTROLS(MODE)) spi (
        .sclk(sclk),.rst_n(rst),.mosi(mosi),.ss_n(ss),.spi_miso_ff(miso),
        .ctrl_reg(controls),.telem_en(),.cs1_pass_set(),.cs2_pass_set(),
        .soft_reset(),.test_en(),.watchdog_fault(fault),.control_toggle(ct),
        .command_toggle(pet),.clear_toggle(clear)
    );
    tail_control_bridge #(.WATCHDOG_CYCLES(64)) bridge (
        .clk(clk),.rst_n(rst),.spi_controls(controls),.control_toggle(ct),
        .command_toggle(pet),.word_toggle(1'b0),.clear_toggle(clear),
        .ro_rst_n(ro),.led_n(led),.test_enable(test_sig),.amp_shdn(shdn),
        .mclk_enable(mclk),.watchdog_fault(fault)
    );
    task check(input bit ok,input string name);
        if(!ok) $fatal(1,"FAIL: %s",name); passes++;
    endtask
    task byte_xfer(input [7:0] tx, output [7:0] rx);
        for(integer i=7;i>=0;i--) begin
            sclk=0;mosi=tx[i];#10;sclk=1;#1;rx[i]=miso;#9;
        end
        sclk=0;
    endtask
    reg [7:0] response, unused;
    task command(input [7:0] op,input [7:0] value);
        ss=0;#10;byte_xfer(op,unused);byte_xfer(value,unused);
        byte_xfer(0,response);#10;ss=1;
        repeat(5) @(negedge clk);
    endtask
    initial begin
        rst=1;#1;rst=0;#1;ss=1;#10; repeat(3) @(negedge clk);rst=1;
        command(7,0);check(response==CAPABILITIES,"discover buffered capture, independent controls and watchdog");
        command(1,8'h1f);check(ro && mclk && test_sig && !shdn && led,"atomic control write");
        command(1,8'h15);check(ro && mclk && test_sig && shdn && !led,"test enabled without amplifier");
        command(1,8'h19);check(ro && mclk && !test_sig && !shdn,"amplifier enabled without test");
        repeat(80) @(negedge clk);
        check(fault && !ro && !mclk && !test_sig && shdn,"SPI clock loss reaches safe state");
        command(8,0);check(response==1,"sticky watchdog status readable");
        command(1,8'h1f);check(fault && !ro,"control write cannot bypass fault");
        command(9,8'h00);check(fault,"invalid clear key rejected");
        command(9,8'ha5);check(!fault && !ro && !mclk && shdn,"clear does not restart outputs");
        command(1,8'h19);check(ro && mclk && !shdn,"separate explicit write recovers");
        $display("RESULTS: %0d passed, 0 failed",passes);
        $display("STATUS: PASS");$finish;
    end
    initial begin #100000; $fatal(1,"timeout");end
endmodule
