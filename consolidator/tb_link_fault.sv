`timescale 1ns/1ps
// Public-wire fault cases for the complete-bank pull receiver.
module tb_link_fault;
    reg clk=0,rst_n=0,miso=0,run=0;
    always #10 clk=~clk;
    wire valid,error,anchored;
    wire [15:0] data;
    integer words=0,errors=0,passes=0;
    spi_ch_pull dut(.sclk(clk),.rst_n(rst_n),.miso_in(miso),.run(run),
      .cfg_hold(1'b0),.resync_req(1'b0),.frame_done(1'b0),.fifo_count(5'b0),
      .word_data(data),.word_valid(valid),.par_err_flag(error),.phase_known(anchored));
    always @(posedge clk) if(rst_n) begin
        if(valid) words<=words+1;
        if(error) errors<=errors+1;
    end
    task check(input bit ok,input string why);
        if(!ok)$fatal(1,"FAIL: %s words=%0d errors=%0d",why,words,errors);
        passes++;
    endtask
    task reset_link;
        @(negedge clk);rst_n=0;run=0;miso=0;
        repeat(3)@(negedge clk);
        words=0;errors=0;rst_n=1;run=1;
        repeat(40)@(negedge clk);
    endtask
    task send_word(input [15:0] value,input bit parity);
        @(negedge clk);miso=1;
        for(integer b=15;b>=0;b--) begin @(negedge clk);miso=value[b];end
        @(negedge clk);miso=parity;
        @(negedge clk);miso=0;
        repeat(4)@(negedge clk);
    endtask
    task anchor(input bit first=1);
        send_word(16'h5ad6,^16'h5ad6);
        check(anchored && (!first || (words==0 && errors==0)),"checked header anchors without leaking as payload");
    endtask
    initial begin
        reset_link;anchor;
        for(integer i=0;i<32;i++) send_word(16'ha5c3 ^ i[15:0],^(16'ha5c3 ^ i[15:0]));
        check(words==32 && errors==0,"healthy parity delivers all words");
        check(data== (16'ha5c3 ^ 16'd31),"healthy final payload");
        send_word(16'h1234,~^16'h1234);
        check(errors==1 && !anchored && words==32,"bad parity rejected and invalidates bank");
        repeat(40)@(negedge clk);anchor(0);
        send_word(16'hbeef,^16'hbeef);
        check(words==33 && errors==1,"fresh header recovers after parity error");
        reset_link;
        send_word(16'h0000,1'b0);
        check(errors==1 && words==0 && !anchored,"invalid header rejected even with good parity");
        reset_link;miso=1;
        repeat(200)@(negedge clk);
        check(errors>0 && words==0,"stuck-high line cannot become valid data");
        reset_link;
        repeat(200)@(negedge clk);
        check(errors==0 && words==0 && !anchored,"stuck-low line remains absent");
        $display("RESULTS: %0d passed, 0 failed",passes);$display("STATUS: PASS");$finish;
    end
    initial begin #100000;$fatal(1,"link fault timeout");end
endmodule
