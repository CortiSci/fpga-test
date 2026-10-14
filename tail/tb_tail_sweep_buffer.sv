`timescale 1ns/1ps
module tb_tail_sweep_buffer;
    reg wc=0, rc=0, rst_n=0, valid=0, start=0, take=0;
    reg [15:0] data=0;
    wire [31:0] drops, seq;
    wire available, last;
    wire [15:0] result;
    integer passes=0;
    always #5 wc=~wc;
    always #7 rc=~rc;
    tail_sweep_buffer dut(rst_n,wc,valid,start,data,drops,rc,take,available,result,seq,last);
    task check(input bit ok,input string name);
      if (!ok) $fatal(1,"FAIL: %s",name);
      passes++;
    endtask
    task send(input integer id,input integer count);
      for(integer i=0;i<count;i++) begin
        @(negedge wc); valid=1; start=(i==0); data=id*1024+i;
      end
      @(negedge wc); valid=0; start=0;
    endtask
    task receive(input integer id,input integer expected_seq);
      wait(available);
      for(integer i=0;i<1024;i++) begin
        @(negedge rc);
        check(available && seq==expected_seq,"stable bank identity");
        check(last==(i==1023),"last word only");
        take=1;
        @(posedge rc); #1;
        check(result==id*1024+i,"exact sample, no mixed bank");
        if(i%31==0) begin
          @(negedge rc); take=0;
          repeat(3) @(posedge rc);
        end
      end
      @(negedge rc); take=0;
    endtask
    initial begin
      #2; repeat(3) @(negedge wc); rst_n=1;
      send(0,1024); send(1,1024); send(2,1024);
      check(drops==1,"full buffer drops incoming sweep");
      receive(0,0); receive(1,1);
      repeat(4) @(negedge wc);
      check(!available,"no publication for discarded sweep");
      send(3,300);
      repeat(4) @(negedge rc);
      check(!available,"partial sweep never visible");
      send(4,1024); receive(4,4);
      check(drops==2,"interrupted partial sweep counted");
      repeat(4) @(negedge wc);
      fork
        begin send(5,1024); send(6,1024); end
        begin receive(5,5); receive(6,6); end
      join
      check(drops==2,"overlapped clocks preserve ownership");
      @(negedge wc); rst_n=0;
      repeat(4) @(negedge rc);
      check(!available && drops==0,"common reset empties banks");
      @(negedge wc); rst_n=1;
      send(7,1024); receive(7,0);
      $display("RESULTS: %0d passed, 0 failed",passes);
      $display("STATUS: PASS"); $finish;
    end
    initial begin #1000000; $fatal(1,"timeout"); end
endmodule
