`timescale 1ns/1ps
// Timestamp coherency/admission regression. Public ports only, no DUT forces.
// Model timer ticks at selected header boundaries to expose scheduling races
// without simulating hours to reach 16/29-bit timer rollover.
module tb_timestamp_snapshot;
    reg clk=0,rst_n=0,start=0,reset_frame=0,ready=1;
    always #10 clk=~clk;
    reg [28:0] timer=0;
    reg [3:0] empty=4'he;
    wire req,valid,ci,cv,cl,cdone,busy;
    wire [15:0] data,cdata;
    wire [31:0] crc;
    reg tv=0;
    reg [15:0] sample=0;
    integer next_sample=0,pos=0,frames=0,errors=0,passes=0,failures=0;
    reg [28:0] observed=0,expected=0;
    reg [31:0] check_crc=32'hffffffff;
    telem_engine_v3 dut(.clk(clk),.rst_n(rst_n),.telem_start(start),
      .sw_reset(reset_frame),.run_any(1'b1),.ext_frame_cnt(timer),.frame_aligned(1'b0),
      .tick_req(req),.tick_data_0(sample),.tick_data_1(16'b0),.tick_data_2(16'b0),.tick_data_3(16'b0),
      .tick_valid(tv),.tick_undf(4'b0),.fill_ge4(4'b1),.leg_anchored(4'b1),.leg_empty(empty),
      .legs_armed(4'b1),.rd_mask(),.leg_resync(),.crc_init(ci),.crc_valid(cv),.crc_last(cl),
      .crc_data(cdata),.crc_result(crc),.crc_result_valid(cdone),
      .telem_tx_data(data),.telem_tx_valid(valid),.telem_tx_ready(ready),.drop_frame(1'b0),
      .par_err_in(4'b0),.ovfl_in(4'b0),.framer_busy(busy));
    crc32 checksum(.clk(clk),.rst_n(rst_n),.init(ci),.valid(cv),.last(cl),.data_in(cdata),
      .crc_out(crc),.crc_valid(cdone));
    always @(posedge clk) begin
      tv<=req;
      if(start) next_sample<=0;
      else if(req) begin sample<=next_sample;next_sample<=next_sample+1;end
    end
    function automatic [31:0] crc_word(input [31:0] seed,input [15:0] word);
      reg [31:0] c;integer b,o;
      begin c=seed;for(o=1;o>=0;o=o-1)for(b=0;b<8;b=b+1)
        if(c[0]^word[o*8+b])c=(c>>1)^32'hedb88320;else c=c>>1;
        crc_word=c;end
    endfunction
    // Registered producer output: a pulse commits one word to the sink.
    always @(posedge clk) begin
      #1;
      if(rst_n && valid) begin
        if(pos==0)begin check_crc=32'hffffffff;if(data!==1)errors++;end
        if(pos==1)observed[15:0]=data;
        if(pos==2)begin observed[28:16]=data[12:0];if(data[15:13]!==0)errors++;end
        if(pos>=3 && pos<4099)begin
          if((pos-3)%4==0)begin if(data!==((frames*1024+(pos-3)/4)&16'hffff))errors++;end
          else if(data!==0)errors++;
        end
        if(pos<=4102)check_crc=crc_word(check_crc,data);
        if(pos==4103 && data!==~check_crc[31:16])errors++;
        if(pos==4104 && data!==~check_crc[15:0])errors++;
        if(pos==4104)begin frames++;pos=0;end else pos++;
      end
    end
    task automatic check(input bit ok,input string label);
      if(ok)begin passes++;$display("[TIMESTAMP] PASS %s",label);end
      else begin failures++;$display("[TIMESTAMP] FAIL %s observed=%h expected=%h errors=%0d",label,observed,expected,errors);end
    endtask
    task automatic begin_case(input [28:0] count,input bit blocked);
      @(negedge clk);reset_frame=1;ready=!blocked;empty=4'he;
      @(negedge clk);reset_frame=0;start=1;timer=count;expected=count;
      pos=0;frames=0;errors=0;observed=0;
      @(negedge clk);start=0;
    endtask
    task automatic finish_case(input string label);
      wait(frames==1);
      check(observed===expected,label);
      check(errors==0,"complete ordered payload and CRC unchanged");
    endtask
    initial begin
      repeat(3)@(negedge clk);rst_n=1;
      begin_case(29'd50,0);finish_case("unstalled timestamp");
      // No restart between these two frames: normal S_GAP must invalidate
      // the snapshot, and a real timer jump must not become a sequence count.
      @(negedge clk);timer=60;expected=60;
      wait(frames==2);
      check(observed===expected,"consecutive frame resamples live timer, including genuine elapsed ticks");
      check(errors==0,"consecutive frame payload continuity and CRC");
      begin_case(29'd51,1);
      repeat(4)@(negedge clk);timer=52;ready=1;
      finish_case("pre-header backpressure must not move available data into next timestamp tick");
      begin_case(29'd52,0);finish_case("following frame uses fresh snapshot (no artificial 2/0 step)");
      begin_case(29'h000ffff,0);
      wait(pos==2);@(negedge clk);ready=0;timer=29'h0010000;
      repeat(4)@(negedge clk);ready=1;
      finish_case("low-word rollover between header halves must not tear timestamp");
      begin_case(29'h1fffffff,0);
      wait(pos==1);@(negedge clk);ready=0;timer=0;
      repeat(4)@(negedge clk);ready=1;
      finish_case("29-bit rollover after tag retains pre-wrap snapshot");
      begin_case(29'd80,1);
      repeat(4)@(negedge clk);
      // Abort an admitted but uncommitted frame; stale snapshot must not leak.
      begin_case(29'd90,0);finish_case("restart invalidates uncommitted timestamp");
      begin_case(29'd100,1);empty=4'hf;
      repeat(4)@(negedge clk);timer=101;expected=101;empty=4'he;
      repeat(4)@(negedge clk);timer=102;ready=1;
      finish_case("silent-leg wait captures only when anchored data becomes available");
      $display("RESULTS: %0d passed, %0d failed",passes,failures);
      $display("STATUS: %s",failures ? "FAIL" : "PASS");
      if(failures)$fatal(1,"timestamp snapshot regression");$finish;
    end
    initial begin #10000000;$fatal(1,"timestamp bench timeout");end
endmodule
