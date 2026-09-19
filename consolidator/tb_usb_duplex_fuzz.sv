`timescale 1ns/1ps
// Deterministic command-path fuzzing with an independent register/response
// scoreboard at accepted USB pins. No driver, hardware timing, or ASIC model.
module tb_usb_duplex_fuzz;
 reg usb_clk=0,core_clk=0,rst_n=0;
 always #7.5 usb_clk=~usb_clk;
 always #10 core_clk=~core_clk;
 wire launch_clk; assign #2.5 launch_clk=usb_clk;
 wire [15:0] bus_data,mover_data,rx_data,tx_head,tx_data,addr,wdata;
 wire [1:0] be;
 wire rxf,txe,rd,wr,oe,mover_valid,full,almost,empty,rx_ready;
 assign be=oe ? 2'b11 : 2'bzz;
 wire tx_valid,tx_full,tx_empty,tx_pop,reg_we;
 wire [9:0] tx_level;
 wire mover_ready=!full&&!almost;
 reg [15:0] rx_words=0;
 reg [15:0] registers[0:31], reference[0:31];
 reg [4:0] fault_src=0; reg framer_busy=0;
 wire [15:0] rdata=registers[addr[4:0]];
 ft600q_tlm model(.clk_66m(usb_clk),.rst_n(rst_n),.usb_fifo_d(bus_data),.usb_fifo_be(be),.rxf_n(rxf),.txe_n(txe),.rd_n(rd),.wr_n(wr),.oe_n(oe));
 ft600_245_fifo_fsm #(.PHASED_OUTPUT(1)) mover(.usb_fifo_clk(usb_clk),.usb_launch_clk(launch_clk),.devrst_n(rst_n),.usb_fifo_d(bus_data),.usb_fifo_rxf_n(rxf),.usb_fifo_txe_n(txe),.usb_fifo_rd_n(rd),.usb_fifo_wr_n(wr),.usb_fifo_oe_n(oe),.cmd_out_data(mover_data),.cmd_out_valid(mover_valid),.cmd_out_ready(mover_ready),.tx_in_data(tx_head),.tx_in_valid(!tx_empty),.tx_in_ready(tx_pop),.tx_in_queued_words(tx_level),.telemetry_active(1'b0),.tx_source_idle(1'b1),.dbg_state());
 cdc_fifo #(.WIDTH(16),.DEPTH_LOG2(5)) rx_fifo(.wr_clk(usb_clk),.wr_rst_n(rst_n),.wr_data(mover_data),.wr_en(mover_valid),.wr_full(full),.wr_almost_full(almost),.wr_half_full(),.rd_clk(core_clk),.rd_rst_n(rst_n),.rd_data(rx_data),.rd_en(!empty&&rx_ready),.rd_empty(empty),.rd_level());
 telem_tx_fifo tx_fifo(.wr_clk(core_clk),.wr_rst_n(rst_n),.wr_data(tx_data),.wr_en(tx_valid&&!tx_full),.wr_full(tx_full),.wr_almost_full(),.wr_half_full(),.rd_clk(usb_clk),.rd_rst_n(rst_n),.rd_data(tx_head),.rd_en(tx_pop&&!tx_empty),.rd_empty(tx_empty),.rd_level(tx_level));
 cmd_decoder decoder(.clk(core_clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(!empty),.rx_ready(rx_ready),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(!tx_full),.reg_addr(addr),.reg_wdata(wdata),.reg_we(reg_we),.reg_rdata(rdata),.fault_src(fault_src),.fault_frame_sent(),.fault_flags_sent(),.framer_busy(framer_busy),.ctrl_early());
 integer passes=0,failures=0,expected_count=0,received=0,sent_words=0,consumed=0;
 integer expected_writes=0,actual_writes=0,rx_pins=0,tx_stalls=0,rx_pressure=0;
 integer midword_faults=0,response_faults=0,reset_cases=0,rejections=0;
 integer bad_family[0:4];
 reg [15:0] expected[0:16383];
 reg [31:0] rng;
 function automatic [31:0] random_word;
  begin rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);random_word=rng;end
 endfunction
 task check(input bit ok,input string what);
  if(ok) passes++; else begin
   failures++; if(failures<12) $display("[DUPLEX] FAIL %s t=%0t",what,$time);
  end
 endtask
 always @(posedge core_clk) if(rst_n) begin
  if(!empty&&rx_ready) consumed++;
  if(reg_we) begin registers[addr[4:0]]<=wdata;actual_writes++;end
  if(tx_valid&&tx_full) tx_stalls++;
 end
 always @(posedge usb_clk) if(rst_n) begin
  if(!oe&&!rd&&!rxf) rx_pins++;
  if(!mover_ready) rx_pressure++;
  if(mover_valid&&full) check(0,"RX overflow");
  if(!wr&&!txe) begin
   check(received<expected_count,"no unsolicited response word");
   if(received<expected_count) begin
    if(bus_data!==expected[received] && failures<12)
     $display("[DUPLEX] word=%0d expected=%h actual=%h",received,expected[received],bus_data);
    check(bus_data===expected[received],"exact ordered pin-accepted response");
   end
   received++;
  end
 end
 task expect_response(input [15:0] flags,input [15:0] address,input [15:0] data);
  begin
   expected[expected_count]=16'h55aa;expected[expected_count+1]=flags;
   expected[expected_count+2]=address;expected[expected_count+3]=data;
   expected_count+=4;
  end
 endtask
 task send_word(input [15:0] value);
  begin
   while(model.rx_count()>240) @(negedge usb_clk);
   @(negedge usb_clk);model.send_word(value);sent_words++;
  end
 endtask
 task command(input bit valid_magic,input bit write_cmd,input [4:0] address,input [15:0] data);
  reg [15:0] magic,flags;integer family;
  begin
   magic=16'haa55;flags={15'b0,write_cmd};
   if(!valid_magic) begin
    family=random_word()%5;bad_family[family]++;
    case(family)
     0:magic=16'haa55^(16'h1<<(random_word()%16));
     1:magic=16'h55aa;
     2:magic=16'haa55+1+random_word()%255;
     3:magic=random_word();
     4:magic=(random_word()&255)*16'h0101;
    endcase
    if(magic==16'haa55) magic=magic^1;
    flags=random_word();
   end
   if(valid_magic) begin
    if(write_cmd) begin reference[address]=data;expected_writes++;end
    expect_response({15'b0,write_cmd},{11'b0,address},write_cmd?data:reference[address]);
   end else rejections++;
   send_word(magic);send_word(flags);send_word({11'b0,address});send_word(data);
  end
 endtask
 task settle;
  integer n;
  begin
   n=0;
   do begin @(negedge core_clk);n++;end
   while((received!=expected_count || model.rx_count()!=0 || !empty || decoder.state!=0 ||
          decoder.fault_pending_flags!=0 || !tx_empty || mover.tx_pend || mover.current_state!=1) && n<30000);
   check(n<30000,"bounded completion including all expected fault/command responses");
   if(n>=30000) $display("[DUPLEX] pending rx=%0d sent=%0d pin=%0d core=%0d fifoempty=%0d decstate=%0d word=%0d txfull=%0d txempty=%0d received=%0d expected=%0d mover=%0d rxf=%b txe=%b",model.rx_count(),sent_words,rx_pins,consumed,empty,decoder.state,decoder.word_idx,tx_full,tx_empty,received,expected_count,mover.current_state,rxf,txe);
   repeat(12) @(negedge core_clk);
   check(received==expected_count,"no missing or duplicate USB words");
   check(sent_words==rx_pins&&rx_pins==consumed,"all offered RX words pin-accepted and decoded");
   check(actual_writes==expected_writes,"register writes occur exactly once");
   for(integer k=0;k<32;k++) check(registers[k]===reference[k],"independent register scoreboard");
  end
 endtask
 task pulse_fault(input [4:0] flags);
  begin @(negedge core_clk);fault_src=flags;@(negedge core_clk);fault_src=0;end
 endtask
 task reset_epoch;
  begin
   @(negedge usb_clk);rst_n=0;fault_src=0;framer_busy=0;
   repeat(6) @(negedge usb_clk);
   model.set_txe_backpressure(0);model.set_rxf_packet_gap(0,0);
   expected_count=0;received=0;sent_words=0;consumed=0;rx_pins=0;
   expected_writes=0;actual_writes=0;
   for(integer k=0;k<32;k++) begin registers[k]=16'h6000+k;reference[k]=16'h6000+k;end
   rst_n=1;repeat(6) @(negedge core_clk);
  end
 endtask
 integer i,seed,n;reg [31:0] value;
 initial begin
  rng=1;for(i=0;i<5;i++) bad_family[i]=0;reset_epoch();
  // Fill the real 2048-word TX FIFO. Decoder ready must fall: this detects
  // ignoring tx_ready, rather than merely delaying the USB sink briefly.
  model.set_txe_backpressure(1);
  fork
   begin
    for(integer j=0;j<600;j++) command(1,0,j%32,0);
   end
   begin
    wait(tx_full);repeat(40) @(negedge core_clk);model.set_txe_backpressure(0);
   end
  join
  settle();check(tx_stalls>0,"decoder response backpressure exercised");
  check(rx_pressure>0,"RX FIFO occupancy/backpressure exercised");

  // Three reproducible mixed-command seeds; plusarg supplies a fourth for
  // optional wider sweeps without changing the quick CI target.
  for(seed=0;seed<4;seed++) begin
   case(seed) 0:rng=1;1:rng=32'h12345678;2:rng=32'hdeadbeef;
    default: if(!$value$plusargs("SEED=%d",rng)) rng=32'h6d2b79f5;
   endcase
   $display("[DUPLEX] mixed seed=%0d",rng);
   model.set_rxf_packet_gap(seed==0?0:4,seed*3);
   for(i=0;i<80;i++) begin
    value=random_word();
    command(i%3!=0,value[0],value[5:1],value[31:16]);
    if(i%11==0) repeat(value[9:6]) @(negedge usb_clk);
   end
   settle();
  end
  model.set_rxf_packet_gap(0,0);

  // FT600 changes readiness during a launched response. This targeted edge
  // complements the long full-FIFO stall above and catches early TX release.
  for(i=0;i<12;i++) begin
   fork
    command(1,0,i%32,0);
    begin
     wait(!wr);@(negedge usb_clk);model.set_txe_backpressure(1);
     repeat(17) @(negedge usb_clk);model.set_txe_backpressure(0);
    end
   join
   settle();
  end

  // Fault between commands; then at each interior word boundary. A fault
  // arriving mid-command must follow that command's complete response.
  expect_response(16'hffff,16'h0001,0);pulse_fault(1);settle();
  for(i=1;i<=3;i++) begin
   expect_response(0,7,reference[7]);expect_response(16'hffff,16'h0010,0);
   send_word(16'haa55);
   if(i>1) send_word(0);
   if(i>2) send_word(7);
   wait(decoder.word_idx==i);pulse_fault(16);midword_faults++;
   if(i<2) send_word(0);
   if(i<3) send_word(7);
   send_word(0);settle();
  end
  // Response waiting for telemetry boundary; incoming malformed words must
  // survive the wait, and the pending fault must not splice either response.
  framer_busy=1;command(1,0,9,0);
  wait(decoder.state==5);expect_response(16'hffff,2,0);pulse_fault(2);response_faults++;
  for(i=0;i<24;i++) command(0,1,i%32,16'hbad0+i);
  repeat(20) @(negedge core_clk);framer_busy=0;settle();
  // Also raise a fault during ST_TX_SEND, while a response is being emitted.
  command(1,0,10,0);wait(decoder.state==4);
  expect_response(16'hffff,4,0);pulse_fault(4);response_faults++;settle();
  check(midword_faults==3&&response_faults==2,"directed fault timing covered");
  check(rejections>0&&actual_writes>0,"both rejected packets and valid writes exercised");

  // Reset aborts incomplete work, not an expectation of delivery across reset.
  // Partial write headers must never become a register write after restart.
  for(i=1;i<=3;i++) begin
   send_word(16'haa55);if(i>1) send_word(1);if(i>2) send_word(5);
   wait(decoder.word_idx==i);reset_epoch();reset_cases++;
   command(1,0,5,0);settle();
  end
  // Reset with an unsent response queued; no old reply may precede the new one.
  model.set_txe_backpressure(1);command(1,0,11,0);wait(!tx_empty);
  reset_epoch();reset_cases++;command(1,0,12,0);settle();
  // A partially emitted response is explicitly abandoned across reset. The
  // first post-reset word must be the new response header, not its old tail.
  command(1,0,13,0);wait(received==5);reset_epoch();reset_cases++;
  command(1,0,14,0);settle();
  check(reset_cases==5,"reset/restart scenarios exercised");
  for(i=0;i<5;i++) begin
   check(bad_family[i]>0,"every malformed-magic family delivered");
   $display("[DUPLEX] bad_magic_family=%0d packets=%0d",i,bad_family[i]);
  end
  $display("[DUPLEX] tx_stall_cycles=%0d rx_pressure_cycles=%0d malformed=%0d midword_faults=%0d response_faults=%0d resets=%0d",tx_stalls,rx_pressure,rejections,midword_faults,response_faults,reset_cases);
  $display("RESULTS: %0d passed, %0d failed",passes,failures);
  $display("STATUS: %s",failures?"FAIL":"PASS");$finish;
 end
 initial begin #2000000;check(0,"global bounded-progress watchdog");
  $display("RESULTS: %0d passed, %0d failed",passes,failures);$display("STATUS: FAIL");$finish;end
endmodule
