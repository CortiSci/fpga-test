`timescale 1ns/1ps
module tb_crc_bytewise;
    reg clk=0,rst=0,init=0,valid=0,last=0;
    reg [15:0] data=0;
    wire [31:0] parallel_crc,serial_crc;
    wire pv,sv;
    integer passes=0;
    reg [31:0] random_state=32'hda5c1234;
    always #5 clk=~clk;
    crc32 reference_crc(clk,rst,init,valid,last,data,parallel_crc,pv);
    crc32_serial dut(clk,rst,init,valid,last,data,serial_crc,sv);
    initial begin
        repeat(3) @(negedge clk);rst=1;
        for(integer packet=1;packet<=64;packet++) begin
            for(integer word_index=0;word_index<packet;word_index++) begin
                @(negedge clk);
                random_state=(random_state<<1) ^ (random_state[31]?32'h04c11db7:0);
                data=random_state[15:0];init=word_index==0;valid=1;last=word_index==packet-1;
                @(negedge clk);init=0;valid=0;last=0;
                @(posedge clk);#1;
                if(serial_crc!==parallel_crc || sv!==(word_index==packet-1))
                    $fatal(1,"CRC mismatch packet %0d word %0d",packet,word_index);
                passes++;
            end
        end
        $display("RESULTS: %0d passed, 0 failed",passes);
        $display("STATUS: PASS");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
