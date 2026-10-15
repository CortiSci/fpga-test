`timescale 1ns/1ps
module tb_impedance_sequence;
    reg clk=0,sclk=0,rst=0,ss=1,mosi=0,fault=0;
    always #24.4140625 clk=~clk;
    wire pending,response,aclk,am,capture;
    wire [1:0] cs,mode;
    integer edges=0,writes=0,passes=0;
    reg [23:0] word=0;
    time capture_start;
    integer capture_cycles=0;
    reg expect_abort=0;
    always @(posedge clk) if(capture) capture_cycles++;
    tail_asic_transaction #(.CONTROL_REGISTERS(1)) dut(
        .clk(clk),.rst_n(rst),.fault(fault),.blocked(1'b0),
        .sclk(sclk),.ss_n(ss),.mosi(mosi),.asic_miso(1'b0),
        .pending(pending),.response(response),.asic_sclk(aclk),.asic_mosi(am),.asic_ss_n(cs),
        .capture_active(capture),.capture_mode(mode));
    task check(input bit ok,input string label);
        if(!ok) $fatal(1,"FAIL: %s",label); passes++;
    endtask
    always @(negedge cs[1]) begin edges=0;word=0;end
    always @(posedge aclk) begin word={word[22:0],am};edges++;end
    always @(posedge cs[1]) if(rst && edges) begin
        check(edges==24 && cs[0],"whole pixel command on correct ASIC chip select");
        check(word==(writes==68 ? 24'hc51107 : 24'hc51101),"clear/inject/advance/retire order");
        writes++;
    end
    always @(posedge capture) begin
        capture_start=$time;
        check(writes==72 && mode==3 && !dut.self_test,"capture starts only after 68 clears, injection and row advance");
    end
    always @(negedge capture) if(rst && capture_start) begin
        if (!expect_abort) check(capture_cycles==20480,"one millisecond dwell uses ASIC clock");
        else check(capture_cycles<204800,"explicit stop ends capture before requested dwell");
    end
    task byte_xfer(input [7:0] value,output [7:0] answer);
        for(integer bitno=7;bitno>=0;bitno--) begin
            sclk=0;mosi=value[bitno];#10;sclk=1;#1;answer[bitno]=response;#9;
        end
        sclk=0;
    endtask
    reg [7:0] ignored,status;
    task poll;
        #20;ss=0;#10;byte_xfer(11,ignored);byte_xfer(0,ignored);byte_xfer(0,status);ss=1;#20;
    endtask
    initial begin
        rst=1;#1;rst=0;#100;rst=1;
        ss=0;#10;byte_xfer(6,ignored);byte_xfer(1,ignored);ss=1;#20;
        // Row 3, odd lanes, one millisecond. Host provides no clocks during execution.
        ss=0;#10;byte_xfer(12,ignored);byte_xfer(3,ignored);byte_xfer(3,ignored);
        byte_xfer(0,ignored);byte_xfer(1,ignored);ss=1;
        check(pending,"measurement accepted");
        #7000000;poll();
        check(status==2 && writes==140 && !capture,"completion only after cleanup");
        // A zero-duration request is rejected, not an unbounded dwell.
        ss=0;#10;byte_xfer(12,ignored);byte_xfer(0,ignored);byte_xfer(2,ignored);
        byte_xfer(0,ignored);byte_xfer(0,ignored);ss=1;poll();
        check(status[2] && !status[0],"invalid dwell rejected");
        writes=0;capture_cycles=0;capture_start=0;expect_abort=1;
        #20;ss=0;#10;byte_xfer(12,ignored);byte_xfer(3,ignored);byte_xfer(3,ignored);
        byte_xfer(0,ignored);byte_xfer(10,ignored);ss=1;
        wait(capture);#100000;
        ss=0;#10;byte_xfer(2,ignored);byte_xfer(0,ignored);ss=1;
        #3000000;poll();
        check(status[3] && !status[0] && writes==140 && !capture,"stop retires injection and reports invalid result");
        $display("RESULTS: %0d passed, 0 failed",passes);$display("STATUS: PASS");$finish;
    end
    initial begin #20000000;$fatal(1,"sequence timeout");end
endmodule
