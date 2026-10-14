`timescale 1ns/1ps
// Nominal behavioral clock for the independent tail supervisor only.
module OSCH #(parameter NOM_FREQ="20.46") (
    input wire STDBY, output wire OSC, output wire SEDSTDBY
);
    reg clock=0;
    initial if(NOM_FREQ!="20.46") $fatal(1,"Unsupported OSCH model frequency");
    always #24.43792766 clock=~clock;
    assign OSC=STDBY ? 1'b0 : clock;
    assign SEDSTDBY=STDBY;
endmodule
