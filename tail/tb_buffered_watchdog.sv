`timescale 1ns/1ps
module tb_buffered_watchdog;
    reg clk=0,rst_n=0,wr=0;
    wire [4:0] controls;
    wire fault;
    always #5 clk=~clk;
    tail_local_controls dut(.clk(clk),.rst_n(rst_n),.write_controls(wr),
      .controls(5'b11111),.pet(1'b0),.clear_fault(1'b0),
      .active_controls(controls),.watchdog_fault(fault));
    initial begin
      repeat(2) @(negedge clk); rst_n=1; wr=1;
      @(negedge clk); wr=0;
      repeat(1843199) @(negedge clk);
      if(fault || controls!=31) $fatal(1,"early production watchdog expiry");
      @(negedge clk);
      if(!fault || controls!=0) $fatal(1,"production watchdog not safe at 90ms cycles");
      $display("RESULTS: 2 passed, 0 failed");
      $display("STATUS: PASS"); $finish;
    end
    initial begin #20000000; $fatal(1,"timeout"); end
endmodule
