`timescale 1ns/1ps
module tb_clock_bootstrap;
    reg osc=0,rst=0,sclk=0,ss=0,mosi=0;
    wire enabled; wire board_clk=osc && enabled;
    wire miso,ro,shdn;
    integer passes=0, n;
    reg [15:0] received;
    reg parity;
    always #24.414 osc=~osc;
    tail_core dut (
      .RO1_SD(16'b0),.RO1_FRAME(1'b0),.RO1_CLK(board_clk),.RO_RSTn(ro),
      .SPI_RO_MOSI(),.SPI_RO_MISO(1'b0),.SPI_RO_SCLK(),.SPI_RO_SS0_B(),.SPI_RO_SS1_B(),
      .FPGA_SPI_MOSI(mosi),.FPGA_SPI_MISO(miso),.FPGA_SPI_SCLK(sclk),.FPGA_SPI_SS(ss),
      .MCLK_EN(enabled),.TEST_SIG(),.REC_TEST_AMP_SHDN(shdn),.MCLK_20_48M(board_clk),
      .DEVRST_N(rst),.SDA_CurrentSense_ADC(),.SCL_CurrentSense_ADC(),
      .DRDY_CurrentSense_ADC(1'b1),.LEDn_0());
    defparam dut.local_safety.controls.WATCHDOG_CYCLES=32768;
    task check(input bit ok,input string why);
      if(!ok) $fatal(1,"FAIL: %s",why); passes++;
    endtask
    task tick;
      sclk=0;#100;sclk=1;#100;
    endtask
    task byte_tx(input [7:0] b);
      for(integer k=7;k>=0;k--) begin mosi=b[k];tick();end
    endtask
    task command(input [7:0] op,input [7:0] val);
      sclk=0;ss=0;#100;byte_tx(op);byte_tx(val);sclk=0;ss=1;mosi=0;#1000;
    endtask
    task read_word;
      mosi=1;n=0;
      while(miso!==1'b1 && n<10000) begin tick();n++;end
      check(n<10000,"BIST word arrives on physical MISO");
      received=0;
      for(integer k=0;k<16;k++) begin tick();received={received[14:0],miso};end
      tick();parity=miso;check(parity==^received,"wire parity");tick();
    endtask
    initial begin
      rst=1;#1;rst=0;#10;ss=1;#90;rst=1;#100;
      check(!enabled,"board clock starts gated off");
      command(1,8'h11);
      check(enabled && ro,"SPI control starts gated board clock");
      command(6,1);command(2,1);
      read_word();check(received[15:1]==15'h2d6b,"pull header");
      read_word();check(received[9:0]==0,"counter starts at complete bank boundary");
      for(integer k=1;k<32;k++) begin read_word();check(received[9:0]==k,"counter increments without gaps");end
      sclk=0;mosi=0;#1800000;
      check(!enabled && !ro && shdn && dut.watchdog_fault,"watchdog shuts down clock and outputs");
      command(9,8'ha5);check(!dut.watchdog_fault && !enabled,"clear works with board clock off");
      command(1,8'h11);check(enabled && ro,"explicit restart works after watchdog");
      $display("RESULTS: %0d passed, 0 failed",passes);$display("STATUS: PASS");$finish;
    end
    initial begin #5000000;$fatal(1,"timeout");end
endmodule

