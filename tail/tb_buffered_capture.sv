`timescale 1ns/1ps
module tb_buffered_capture;
    reg wc=0, rc=0, rst_n=0, frame=0, impedance=0, lane=0, take=0;
    reg [15:0] data=0;
    wire [15:0] result;
    wire available,last;
    wire [31:0] seq,drops;
    integer passes=0;
    always #5 wc=~wc;
    always #3 rc=~rc;
    tail_buffered_capture dut(rst_n,wc,data,frame,impedance,lane,drops,
                              rc,take,available,result,seq,last);
    task check(input bit ok,input string name);
        if (!ok) $fatal(1,"FAIL: %s",name);
        passes++;
    endtask
    function automatic [15:0] sample(input integer id,input integer i);
        sample=(id*313+i*157)^16'ha531;
    endfunction
    function automatic [15:0] expected(input integer id,input integer i);
        reg [15:0] a,b;
        if (!impedance) expected=sample(id,i);
        else begin
            a=sample(id,2*i); b=sample(id,2*i+1);
            for(integer j=0;j<8;j++) begin
                expected[2*j]=a[2*j+lane];
                expected[2*j+1]=b[2*j+lane];
            end
        end
    endfunction
    task reset_mode(input bit imp,input bit selected);
        @(negedge wc); rst_n=0; frame=0; take=0;
        impedance=imp; lane=selected;
        repeat(4) @(negedge wc);
        rst_n=1;
        repeat(5) @(negedge wc);
        check(!available,"no sweep before frame start");
    endtask
    task send(input integer id,input integer words);
        for(integer i=0;i<words*(impedance?2:1);i++) begin
            @(negedge wc); frame=(i%1024<2); data=sample(id,i);
        end
        @(negedge wc); frame=0;
    endtask
    task receive(input integer id,input integer sequence_id);
        wait(available);
        for(integer i=0;i<1024;i++) begin
            @(negedge rc);
            check(available && seq==sequence_id,"capture identity retained");
            check(last==(i==1023),"complete sweep length");
            take=1;
            @(posedge rc); #1;
            check(result===expected(id,i),"exact normal/impedance lane ordering");
        end
        @(negedge rc); take=0;
    endtask
    initial begin
        reset_mode(0,0);
        send(1,1024); send(2,1024); send(3,1024);
        check(drops==1,"full buffer loses only new sweep");
        receive(1,0); receive(2,1);
        repeat(5) @(negedge wc);
        send(4,1024); receive(4,3);
        for(integer selected=0;selected<2;selected++) begin
            reset_mode(1,selected);
            // Start from an arbitrary decimation phase; a frame must re-anchor T0.
            repeat(3) @(negedge wc);
            send(5,1024); receive(5,0);
            send(6,1024); receive(6,1);
            check(drops==0,"wide frame pulse does not split impedance pairing");
        end
        reset_mode(0,0);
        send(7,300);
        check(!available,"incomplete acquisition never published");
        send(8,1024); receive(8,1);
        check(drops==1,"early frame counts discarded partial sweep");
        $display("RESULTS: %0d passed, 0 failed",passes);
        $display("STATUS: PASS"); $finish;
    end
    initial begin #1000000; $fatal(1,"timeout"); end
endmodule
