`timescale 1ns/1ps
module tb_asic_transaction;
    reg clk=0,rst=0,fault=0,blocked=0,sclk=0,ss=1,mosi=0;
    wire selected,pending,response,aclk,am,mi;
    wire [1:0] cs;
    integer passes=0,edges=0,transactions=0;
    reg [23:0] captured=0, expected=0, reply=24'h96a53c;
    reg expected_cs=0, allow_abort=0;
    real last_rise=0;
    reg clock_enabled=1;
    always #24.414 clk=clock_enabled ? ~clk : 0;
    assign mi=edges<24 ? reply[23-edges] : 0;
    tail_asic_transaction dut(.clk(clk),.rst_n(rst),.fault(fault),.blocked(blocked),
        .sclk(sclk),.ss_n(ss),.mosi(mosi),.asic_miso(mi),.selected(selected),
        .pending(pending),.response(response),.asic_sclk(aclk),.asic_mosi(am),.asic_ss_n(cs));
    task check(input bit ok,input string name);
        if(!ok) $fatal(1,"FAIL: %s",name); passes++;
    endtask
    always @(negedge cs[0] or negedge cs[1]) begin edges=0;captured=0;last_rise=0;end
    always @(posedge aclk) begin
        check(cs== (expected_cs ? 2'b01:2'b10),"one correct ASIC selected");
        if(edges>0) check(($realtime-last_rise)>1562 && ($realtime-last_rise)<1563,"640kHz clock independent of host");
        last_rise=$realtime; captured={captured[22:0],am}; edges++;
    end
    always @(cs) if(rst && cs==3 && edges>0) begin
        if(!allow_abort) begin
            check(edges==24,"exactly 24 ASIC clocks");
            check(captured==expected,"MSB-first ASIC payload");
            transactions++;
        end
        edges=0;
    end
    task byte_xfer(input [7:0] value,output [7:0] value_rx);
        for(integer i=7;i>=0;i--) begin
            sclk=0;mosi=value[i];#10;sclk=1;#1;value_rx[i]=response;#9;
        end
        sclk=0;
    endtask
    reg [7:0] ignored,stat;
    task submit(input [7:0] sel,input [23:0] payload);
        #20;ss=0;#10;byte_xfer('h0a,ignored);byte_xfer(sel,ignored);
        byte_xfer(payload[23:16],ignored);byte_xfer(payload[15:8],ignored);
        byte_xfer(payload[7:0],ignored);#10;ss=1;
    endtask
    task read(input [7:0] index,output [7:0] value);
        #20;ss=0;#10;byte_xfer('h0b,ignored);byte_xfer(index,ignored);byte_xfer(0,value);#10;ss=1;
    endtask
    initial begin
        rst=1;#1;rst=0;#100;rst=1;
        expected=24'ha56c93;submit(0,expected);
        check(pending,"request pending immediately");
        // No further host edges: execution must nevertheless finish.
        #39000;check(transactions==1 && cs==3 && !aclk,"autonomous bounded completion with host stopped");
        read(0,stat);check(stat==2,"completed result status");
        read(1,stat);check(stat==reply[23:16],"readback MSB");
        read(2,stat);check(stat==reply[15:8],"readback middle byte");
        read(3,stat);check(stat==reply[7:0],"readback LSB");
        expected_cs=1;expected=24'h123456;submit(1,expected);
        submit(0,24'hffffff);read(0,stat);check(stat[2:0]==5,"busy request rejected without overwrite");
        #39000;read(0,stat);check(stat==6 && transactions==2,"original request still completes");
        submit(2,0);read(0,stat);check(stat[2] && !stat[0],"invalid selector rejected");
        blocked=1;submit(0,0);read(0,stat);check(stat[2] && !stat[0],"stream/relay conflict rejected");blocked=0;
        #20;ss=0;#10;byte_xfer('h0a,ignored);byte_xfer(0,ignored);byte_xfer('h11,ignored);ss=1;
        #40000;check(transactions==2,"truncated command has no side effects");
        expected_cs=0;expected=24'habcdef;submit(0,expected);#4000;allow_abort=1;clock_enabled=0;fault=1;
        #200;check(cs==3 && !aclk,"watchdog abort parks ASIC pins");
        read(0,stat);check(stat[3] && !stat[0],"aborted request reported");
        read(1,stat);check(stat==0,"aborted readback not published");
        submit(0,0);read(0,stat);check(stat[2] && !stat[0],"request rejected while faulted");
        fault=0;clock_enabled=1;allow_abort=0;expected=24'h654321;submit(0,expected);
        #39000;read(0,stat);check(stat==2 && transactions==3,"explicit new request recovers");
        expected=24'h13579b;submit(0,expected);#4000;allow_abort=1;rst=0;
        #100;check(cs==3 && !aclk && !pending,"reset aborts and clears mailbox");rst=1;
        #40000;check(transactions==3,"reset does not replay abandoned transaction");
        $display("RESULTS: %0d passed, 0 failed",passes);$display("STATUS: PASS");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
