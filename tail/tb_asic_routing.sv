`timescale 1ns/1ps
module tb_asic_routing;
    reg clk=0,rst=0,sclk=0,ss=0,mosi=0;
    wire miso,aclk,am,cs0,cs1;
    integer passes=0,edges=0;
    reg [23:0] captured=0;
    always #24.414 clk=~clk;
    tail_fpga_small #(.BUFFERED_CAPTURE(2)) dut (
        .RO1_SD(16'b0),.RO1_FRAME(1'b0),.RO1_CLK(clk),.RO_RSTn(),
        .SPI_RO_MOSI(am),.SPI_RO_MISO(1'b1),.SPI_RO_SCLK(aclk),
        .SPI_RO_SS0_B(cs0),.SPI_RO_SS1_B(cs1),
        .FPGA_SPI_MOSI(mosi),.FPGA_SPI_MISO(miso),.FPGA_SPI_SCLK(sclk),.FPGA_SPI_SS(ss),
        .MCLK_EN(),.TEST_SIG(),.REC_TEST_AMP_SHDN(),.MCLK_20_48M(clk),.DEVRST_N(rst),
        .SDA_CurrentSense_ADC(),.SCL_CurrentSense_ADC(),.DRDY_CurrentSense_ADC(1'b1),.LEDn_0());
    always @(posedge aclk) begin captured={captured[22:0],am};edges++;end
    task check(input bit ok,input string why);
        if(!ok) $fatal(1,"FAIL: %s",why);passes++;
    endtask
    reg [7:0] rx;
    task byte_xfer(input [7:0] data);
        for(integer i=7;i>=0;i--) begin sclk=0;mosi=data[i];#10;sclk=1;#1;rx[i]=miso;#9;end
        sclk=0;
    endtask
    task cmd(input [7:0] op,input [7:0] data);
        #20;ss=0;#10;byte_xfer(op);byte_xfer(data);byte_xfer(0);#10;ss=1;#20;
    endtask
    initial begin
        rst=1;#1;rst=0;#10;ss=1;#100;rst=1;
        #20;ss=0;byte_xfer('h0a);byte_xfer(0);byte_xfer('h95);byte_xfer('hac);byte_xfer('h31);ss=1;
        cmd(2,1);
        check(dut.local_pending && !dut.telem_active,"stream start held while ASIC configuration pending");
        #39000;
        check(edges==24 && captured==24'h95ac31 && cs0 && cs1,"local ASIC payload reaches board pins");
        cmd('h0b,0);check(rx==2 && dut.telem_active,"stream becomes eligible only after configuration completes");
        cmd(2,0);check(!dut.telem_active,"stream stop still works");
        // Removed relay opcodes must never select or clock the ASIC.
        cmd(3,0);
        #10;sclk=1;#10;sclk=0;#10; // relay arm priming edge with SS high
        edges=0;captured=0;ss=0;#10;
        byte_xfer('h0a);byte_xfer('h76);byte_xfer('h53);
        check(cs0 && cs1,"old relay command cannot select ASIC");
        ss=1;#20;sclk=1;#10;sclk=0;
        check(edges==0,"raw relay payload cannot clock ASIC");
        #40000;check(edges==0,"truncated local-looking payload has no effect");
        $display("RESULTS: %0d passed, 0 failed",passes);$display("STATUS: PASS");$finish;
    end
    initial begin #200000;$fatal(1,"timeout");end
endmodule
