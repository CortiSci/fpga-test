`timescale 1ns/1ps
// Simulation stub for Lattice MachXO2 BB bidirectional I/O buffer.
// Used by record_tail_fpga.v for I2C open-drain pads (BB.I=VCC, OPENDRAIN=ON).
// Hardware behavior: T=0 → NMOS pulls pad LOW; T=1 → Hi-Z (pull-up → HIGH).
// O always reflects the current pad state (used for clock-stretch read-back).
module BB (
    inout  wire B,   // bidirectional pad
    input  wire I,   // drive data (tied 1'b1 for open-drain use; unused in stub)
    input  wire T,   // tristate: 0=drive LOW (open-drain), 1=Hi-Z
    output wire O    // pad read-back
);
    assign B = T ? 1'bz : 1'b0;
    assign O = B;
endmodule
