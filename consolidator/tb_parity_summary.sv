`timescale 1ns/1ps
// TC-LINK-PAR-COUNT-03: decoder boundary events, no hierarchical force.
module tb_parity_summary;
  reg clk=0; always #10 clk=~clk;
  reg rst_n=0, rx_valid=0, tx_ready=1, framer_busy=0;
  reg [15:0] rx_data=0;
  reg [3:0] parity_events=0;
  reg [4:0] fault_src=0;
  wire [15:0] tx_data;
  wire tx_valid,rx_ready;
  reg [15:0] captured[0:127];
  integer count=0, passes=0, failures=0;
  cmd_decoder dut(.clk(clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),
    .rx_ready(rx_ready),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),
    .reg_addr(),.reg_wdata(),.reg_we(),.reg_rdata(16'h1234),
`ifndef BASELINE_NO_PARITY
    .parity_events(parity_events),
`endif
    .fault_src(fault_src),.fault_frame_sent(),.fault_flags_sent(),
    .framer_busy(framer_busy),.ctrl_early());
  always @(posedge clk) if(rst_n && tx_valid && tx_ready) begin
    captured[count]=tx_data; count=count+1;
  end
  task check(input bit ok,input string label);
    if(ok) begin passes=passes+1;$display("PASS: %s",label);end
    else begin failures=failures+1;$display("FAIL: %s",label);end
  endtask
  task reset;
    @(negedge clk);rst_n=0;rx_valid=0;parity_events=0;fault_src=0;tx_ready=1;framer_busy=0;
    repeat(3) @(negedge clk);count=0;rst_n=1;
  endtask
  task word(input [15:0] v);
    @(negedge clk);while(!rx_ready) @(negedge clk);
    rx_data=v;rx_valid=1;@(negedge clk);rx_valid=0;
  endtask
  task settle;repeat(80) @(negedge clk);endtask
  task frame(input integer offset,input [15:0] mask,input [15:0] value);
    check(count>=offset+4 && captured[offset]==16'h55aa && captured[offset+1]==16'hffff
      && captured[offset+2]==mask && captured[offset+3]==value,"exact fault packet");
  endtask
  initial begin
    reset();settle();check(count==0,"no startup fault");
    @(negedge clk);parity_events=4'b1111;
    @(negedge clk);parity_events=0;settle();
    frame(0,16'h20,4);check(count==4,"four simultaneous legs counted once each");
    reset();word(16'haa55); // incomplete command blocks fault dispatch
    @(negedge clk);parity_events=15;repeat(7) @(negedge clk);parity_events=0;
    check(count==0,"no fault inserted mid-command");
    word(0);word(16'h0006);word(0);settle();
    check(captured[3]==16'h1234,"ordinary read response intact");
    frame(4,16'h20,16);check(count==8,"saturates at sixteen without wrapping");
    reset();tx_ready=0;framer_busy=1;
    @(negedge clk);parity_events=4'b0011;
    @(negedge clk);parity_events=4'b0100;fault_src=5'b10000; // snapshot edge
    @(negedge clk);parity_events=4'b1000;fault_src=0;
    @(negedge clk);parity_events=0;settle();
    check(count==0,"no output during telemetry frame/backpressure");
    framer_busy=0;repeat(10) @(negedge clk);
    check(count==0,"TX snapshot held under backpressure");
    tx_ready=1;settle();frame(0,16'h20,2);frame(4,16'h30,2);
    check(count==8,"same-cycle and queued events retained for next packet");
    reset();fault_src=1;@(negedge clk);fault_src=0;settle();
    frame(0,1,0);check(count==4,"ordinary fault keeps zero summary");
    reset();settle();check(count==0,"reset clears pending count");
    $display("RESULTS: %0d passed, %0d failed",passes,failures);
    if(failures==0)$display("STATUS: PASS");else $display("STATUS: FAIL");
    $finish;
  end
  initial begin #1000000;$display("STATUS: FAIL watchdog");$finish;end
endmodule
