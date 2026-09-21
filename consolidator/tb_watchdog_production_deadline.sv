`timescale 1ns/1ps
// Production 100 ms deadline regression: no force or short-timeout define.
module tb_watchdog_production_deadline;
 reg clk=0, rst_n=0, en=0, pet=0;
 wire expired, active, reset;
 // Count cycles exactly; 20.48 MHz period rounded at 1 ps simulator resolution.
 always #24.414 clk=~clk;
 watchdog_con dut(clk,rst_n,en,pet,expired,active,reset);
 integer cycles=0, pets=0;
 initial begin
  repeat(3) @(negedge clk); rst_n=1; en=1; pet=1;
  @(negedge clk); pet=0;
  // Worker cadence: three legal 20 ms intervals reload the full deadline.
  for(pets=0;pets<3;pets=pets+1) begin
   repeat(409600) @(negedge clk);
   if(expired || reset || !active) $fatal(1,"20 ms pets must hold expiry off");
   pet=1; @(negedge clk); pet=0;
  end
  while(!expired && cycles<2048002) begin
   @(posedge clk); #0.001; cycles=cycles+1;
   if(cycles==2048000 && !expired)
    $fatal(1,"PLN005 deadline missed: watchdog not expired after 100 ms (2048000 oscillator cycles)");
  end
  if(cycles!=1843200) $fatal(1,"Wrong production expiry count %0d",cycles);
  @(posedge clk); #0.001;
  if(!reset || active) $fatal(1,"Expiry must produce reset pulse and clear active");
  @(posedge clk); #0.001;
  if(reset || !expired) $fatal(1,"Reset must be one cycle; expiry must remain sticky");
  $display("Production expiry: %0d cycles = 90 ms at 20.48 MHz",cycles);
  $display("RESULTS: 6 passed, 0 failed"); $display("STATUS: PASS"); $finish;
 end
 initial begin #180000000; $fatal(1,"Watchdog diagnostic timeout"); end
endmodule
