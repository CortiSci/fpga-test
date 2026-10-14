`timescale 1ns/1ps
module tb_buffered_controls;
    reg clk=0, rst_n=0, wr=0, pet=0, clear=0;
    reg [4:0] controls=0;
    wire [4:0] active;
    wire fault, ro, led, test_en, shdn, mclk;
    integer passes=0;
    always #5 clk=~clk;
    tail_local_controls #(.WATCHDOG_CYCLES(8)) dut
      (clk,rst_n,wr,controls,pet,clear,active,fault,ro,led,test_en,shdn,mclk);
    task check(input bit ok, input string name);
      if (!ok) $fatal(1,"FAIL: %s",name);
      passes=passes+1;
    endtask
    task tick; @(posedge clk); #1; endtask
    initial begin
      #1; tick; check(active==0 && shdn && !fault,"reset safe");
      @(negedge clk); rst_n=1; wr=1; controls=5'b01000;
      tick; check(!shdn && !test_en && !mclk,"amplifier independent");
      @(negedge clk); controls=5'b10100;
      tick; check(shdn && test_en && mclk && !ro,"test and clock independent");
      @(negedge clk); wr=0; pet=1; tick;
      @(negedge clk); pet=0;
      repeat(7) tick;
      check(!fault,"not early");
      @(negedge clk); pet=1; wr=1; controls=31; clear=1;
      tick; check(fault && active==0 && shdn,"expiry wins all commands");
      @(negedge clk); clear=0;
      tick; check(fault && active==0,"pet/write cannot clear fault");
      @(negedge clk); clear=1;
      tick; check(!fault && active==0,"explicit recovery stays safe");
      @(negedge clk); clear=0;
      tick; check(active==31,"new write after recovery");
      @(negedge clk); wr=0;
      repeat(30) tick;
      check(!fault,"regular pets preserve outputs");
      @(negedge clk); pet=0;
      repeat(8) tick;
      check(fault && active==0,"watchdog without SPI activity");
      $display("RESULTS: %0d passed, 0 failed",passes);
      $display("STATUS: PASS"); $finish;
    end
    initial begin #100000; $fatal(1,"timeout"); end
endmodule
