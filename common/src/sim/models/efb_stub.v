// Simulation stub for Lattice MachXO2 EFB (Enhanced Function Block).
// Port names match Diamond 3.14 machxo2.v EFB exactly — all Wishbone
// address/data buses are individual scalar pins, and SPI ports use
// Diamond 3.14 names (SPISCKI / SPISCSN).
//
// Compile this file BEFORE the RTL that instantiates EFB so ModelSim/
// QuestaSim resolves the EFB module here. Do NOT include in synthesis fileset.
//
// I2C1 master: The stub implements a simplified Wishbone-to-I2C bridge so
// drdy_monitor can read from the ADS122C14 I2C slave model.  I2C operations
// execute as tasks with #-delays; SCL/SDA are driven via I2C1SCLOEN/I2C1SDAOEN
// (open-drain: 1 = Hi-Z/HIGH via pull-up, 0 = drive LOW).
`timescale 1ns/1ps

// The stub exposes the complete vendor primitive interface but implements only
// the exercised UFM/Wishbone behavior. Production synthesis uses Diamond EFB.
/* verilator lint_off DECLFILENAME */
/* verilator lint_off BLKSEQ */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off CASEINCOMPLETE */
module EFB #(
    parameter EFB_UFM               = "DISABLED",
    parameter EFB_SPI               = "DISABLED",
    parameter EFB_I2C1              = "DISABLED",
    parameter EFB_I2C2              = "DISABLED",
    parameter EFB_TC                = "DISABLED",
    parameter EFB_WB_CLK_FREQ       = "48.0",
    parameter DEV_DENSITY           = "1200L",
    parameter I2C1_ADDRESSING       = "7BIT",
    parameter I2C1_BUS_PERF         = "100kHz",
    parameter I2C1_CLK_DIVIDER      = 1,
    parameter I2C1_GEN_CALL         = "DISABLED",
    parameter I2C1_WAKEUP           = "DISABLED",
    parameter UFM_INIT_FILE_FORMAT  = "NONE",
    parameter UFM_INIT_FILE_NAME    = "NONE",
    parameter TC_ICAPTURE           = "DISABLED",
    parameter GSR                   = "DISABLED",
    parameter SPI_ENABLE            = "DISABLED",
    parameter I2C1_ENABLE          = "DISABLED",
    parameter I2C2_ENABLE          = "DISABLED",
    parameter TC_ENABLE             = "DISABLED",
    parameter UFM_ENABLE            = "DISABLED",
    parameter WBCLK_EDGE            = "POSEDGE"
) (
    // Wishbone clock / reset
    input  wire        WBCLKI,
    input  wire        WBRSTI,
    // Wishbone control
    input  wire        WBCYCI,
    input  wire        WBSTBI,
    input  wire        WBWEI,
    // Address bus — individual scalar bits (Diamond 3.14 style)
    input  wire        WBADRI7, WBADRI6, WBADRI5, WBADRI4,
    input  wire        WBADRI3, WBADRI2, WBADRI1, WBADRI0,
    // Write data — individual scalar bits
    input  wire        WBDATI7, WBDATI6, WBDATI5, WBDATI4,
    input  wire        WBDATI3, WBDATI2, WBDATI1, WBDATI0,
    // Read data — individual scalar bits
    output wire        WBDATO7, WBDATO6, WBDATO5, WBDATO4,
    output wire        WBDATO3, WBDATO2, WBDATO1, WBDATO0,
    output reg         WBACKO,

    // PLL0 interface — individual scalar data bits
    input  wire        PLL0DATI7, PLL0DATI6, PLL0DATI5, PLL0DATI4,
    input  wire        PLL0DATI3, PLL0DATI2, PLL0DATI1, PLL0DATI0,
    input  wire        PLL0ACKI,
    output wire [7:0]  PLL0DATO,
    output wire        PLL0STBO,

    // PLL1 interface — individual scalar data bits
    input  wire        PLL1DATI7, PLL1DATI6, PLL1DATI5, PLL1DATI4,
    input  wire        PLL1DATI3, PLL1DATI2, PLL1DATI1, PLL1DATI0,
    input  wire        PLL1ACKI,
    output wire [7:0]  PLL1DATO,
    output wire        PLL1STBO,

    // PLL control outputs (EFB → external PLL)
    output wire        PLLCLKO,
    output wire        PLLRSTO,
    output wire        PLLWEO,
    output wire        PLLADRO4, PLLADRO3, PLLADRO2, PLLADRO1, PLLADRO0,
    output wire        PLLDATO7, PLLDATO6, PLLDATO5, PLLDATO4,
    output wire        PLLDATO3, PLLDATO2, PLLDATO1, PLLDATO0,

    // I2C1
    input  wire        I2C1SCLI,
    input  wire        I2C1SDAI,
    output wire        I2C1SCLO,
    output wire        I2C1SDAO,
    output wire        I2C1SCLOEN,
    output wire        I2C1SDAOEN,
    output wire        I2C1IRQ,

    // I2C2
    input  wire        I2C2SCLI,
    input  wire        I2C2SDAI,
    output wire        I2C2SCLO,
    output wire        I2C2SDAO,
    output wire        I2C2SCLOEN,
    output wire        I2C2SDAOEN,
    output wire        I2C2IRQ,

    // SPI — Diamond 3.14 port names
    input  wire        SPISCKI,
    input  wire        SPIMISOI,
    input  wire        SPIMOSII,
    input  wire        SPISCSN,
    output wire        SPIMISOO,
    output wire        SPIIRQ,
    output wire        SPICSNO,
    output wire        SPIMOSIO,
    output wire        SPICSO,
    output wire        SPICLKO,

    // Timer/counter
    input  wire        TCCLKI,
    input  wire        TCRSTN,
    input  wire        TCIC,
    output wire        TCINT,
    output wire        TCOC,
    output wire        TCOCOE,
    output wire        TCLIN,
    output wire        TCGSR,

    // UFM chip-select (active-low)
    input  wire        UFMSN,

    // Wishbone / configuration outputs
    output wire        WBINT,
    output wire        WBCUFMIRQ,
    output wire        CFGWAKE,
    output wire        CFGSTDBY
);

    // -------------------------------------------------------------------------
    // Reassemble bus signals from individual scalar inputs
    // -------------------------------------------------------------------------
    wire [7:0] wbadri_bus = {WBADRI7, WBADRI6, WBADRI5, WBADRI4,
                              WBADRI3, WBADRI2, WBADRI1, WBADRI0};
    wire [7:0] wbdati_bus = {WBDATI7, WBDATI6, WBDATI5, WBDATI4,
                              WBDATI3, WBDATI2, WBDATI1, WBDATI0};

    // Internal read-data register — fanned out to individual output wires
    reg [7:0] wbdato_r;
    assign {WBDATO7, WBDATO6, WBDATO5, WBDATO4,
            WBDATO3, WBDATO2, WBDATO1, WBDATO0} = wbdato_r;

    // -------------------------------------------------------------------------
    // TRACEID register file — 64-bit trace ID provided via `define.
    // Lattice SYSCONFIG addresses 0x58–0x5B (byte reads, LSB first).
    // -------------------------------------------------------------------------
    `ifndef EFB_TRACEID
    `define EFB_TRACEID 64'hDEAD_BEEF_CAFE_F00D
    `endif

    localparam [63:0] TRACE_ID = `EFB_TRACEID;

    // -------------------------------------------------------------------------
    // I2C master registers (spi_clk / WBCLKI domain)
    // -------------------------------------------------------------------------
    // WB register addresses (Lattice EFB TN1204)
    localparam WB_I2C_CR   = 8'h29;   // control: STA[7] STO[6] RD[5] WR[4] ACK[3] IACK[0]
    localparam WB_I2C_TXDR = 8'h2A;   // transmit data
    localparam WB_I2C_SR   = 8'h2B;   // status: TIP[1]
    localparam WB_I2C_RXDR = 8'h2C;   // receive data

    reg [7:0] i2c_txdr;    // byte to transmit (loaded from WB TXDR write)
    reg [7:0] i2c_rxdr;    // byte received    (returned on WB RXDR read)
    reg       i2c_tip;     // TIP: 1 = I2C transaction in progress

    // I2C bus open-drain drivers (1 = Hi-Z/HIGH via pull-up, 0 = drive LOW)
    // I2C1SCLOEN: 0 → BB T=0 → pad LOW; 1 → BB T=1 → pad Hi-Z=HIGH (tri1 pull-up)
    reg i2c_scl_oe;
    reg i2c_sda_oe;
    assign I2C1SCLOEN = i2c_scl_oe;
    assign I2C1SDAOEN = i2c_sda_oe;

    // I2C half-period for simulation (100 ns → 5 MHz SCL).
    // Full READ sequence: 5 bytes × ~1.8-2.7 µs ≈ 12 µs << test's 50 µs window.
    localparam integer I2C_HALF = 100;

    // Named event + latch: WB write to CR triggers I2C execution
    event   i2c_cr_event;
    reg [7:0] i2c_cr_cmd;

    // -------------------------------------------------------------------------
    // Wishbone slave — registered ACK, one cycle after STB qualifies.
    // Handles TXDR writes, CR write (triggers I2C), SR/RXDR/TRACEID reads.
    // -------------------------------------------------------------------------
    always @(posedge WBCLKI or posedge WBRSTI) begin
        if (WBRSTI) begin
            wbdato_r <= 8'h00;
            WBACKO   <= 1'b0;
            i2c_txdr <= 8'h00;
        end else begin
            WBACKO <= WBCYCI & WBSTBI & ~WBACKO;
            if (WBCYCI & WBSTBI & ~WBACKO) begin
                if (WBWEI) begin
                    // Write transaction
                    case (wbadri_bus)
                        WB_I2C_TXDR: i2c_txdr <= wbdati_bus;
                        WB_I2C_CR: begin
                            // Latch CR value immediately (blocking) then fire event
                            i2c_cr_cmd = wbdati_bus;
                            -> i2c_cr_event;
                        end
                    endcase
                end else begin
                    // Read transaction
                    case (wbadri_bus)
                        WB_I2C_SR:   wbdato_r <= {6'h0, i2c_tip, 1'b0};  // TIP at bit[1] per EFB TN1204
                        WB_I2C_RXDR: wbdato_r <= i2c_rxdr;
                        8'h58: wbdato_r <= TRACE_ID[ 7: 0];  // LSB
                        8'h59: wbdato_r <= TRACE_ID[15: 8];
                        8'h5A: wbdato_r <= TRACE_ID[23:16];
                        8'h5B: wbdato_r <= TRACE_ID[31:24];  // MSB of 32-bit ESN
                        8'h70: wbdato_r <= 8'h00;
                        default: wbdato_r <= 8'h00;
                    endcase
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // I2C master tasks
    // -------------------------------------------------------------------------

    // START condition: SCL=1, SDA falls 1→0.
    // Precondition: any SCL/SDA state.  After: SCL=0, SDA=0.
    task automatic i2c_start;
        begin
            i2c_sda_oe = 1'b1; i2c_scl_oe = 1'b1; #(I2C_HALF); // ensure idle
            i2c_sda_oe = 1'b0;                      #(I2C_HALF); // SDA falls → START
            i2c_scl_oe = 1'b0;                      #(I2C_HALF); // SCL falls
        end
    endtask

    // Write one byte MSB-first; return got_ack=1 if slave ACKed.
    // Precondition: SCL=0.  After: SCL=0, SDA=1 (released).
    task automatic i2c_write_byte;
        input [7:0] data;
        output      got_ack;
        integer j;
        begin
            for (j = 7; j >= 0; j = j - 1) begin
                // Data bit: set SDA while SCL=0
                // sda_oe=data[j]: 1 → release (tri1=HIGH), 0 → drive LOW
                i2c_sda_oe = data[j]; #(I2C_HALF);
                i2c_scl_oe = 1'b1;    #(I2C_HALF);  // SCL high — slave samples
                i2c_scl_oe = 1'b0;    #(I2C_HALF);  // SCL low  — slave advances
            end
            // ACK slot: release SDA, raise SCL, sample, lower SCL
            i2c_sda_oe = 1'b1;        #(I2C_HALF);
            i2c_scl_oe = 1'b1;        #(I2C_HALF);
            got_ack    = ~I2C1SDAI;                  // 1 = slave pulled SDA low
            i2c_scl_oe = 1'b0;        #(I2C_HALF);
        end
    endtask

    // Read one byte MSB-first from slave.
    // send_nack=1 → master sends NACK (last byte); send_nack=0 → ACK.
    // Precondition: SCL=0, slave already driving first bit.
    // After: SCL=0, SDA=1 (released).
    task automatic i2c_read_byte;
        input      send_nack;
        output reg [7:0] rxd;
        integer j;
        begin
            i2c_sda_oe = 1'b1;  // release SDA for slave to drive
            for (j = 7; j >= 0; j = j - 1) begin
                i2c_scl_oe = 1'b1; #(I2C_HALF); // SCL high — SDA stable from slave
                rxd[j]     = I2C1SDAI;            // sample
                i2c_scl_oe = 1'b0; #(I2C_HALF);  // SCL low  — slave sets next bit
            end
            // ACK/NACK: 0=drive low=ACK, 1=release=NACK
            i2c_sda_oe = send_nack ? 1'b1 : 1'b0;
            #1; // let sda_prev (#1 lag) settle before SCL rises to avoid false START
            i2c_scl_oe = 1'b1; #(I2C_HALF);
            i2c_scl_oe = 1'b0; #(I2C_HALF);
            i2c_sda_oe = 1'b1;   // release after ACK/NACK
        end
    endtask

    // STOP condition: SCL=1, SDA rises 0→1.
    // Precondition: SCL=0.  After: SCL=1, SDA=1 (bus idle).
    task automatic i2c_stop;
        begin
            i2c_sda_oe = 1'b0; #(I2C_HALF);  // ensure SDA low
            i2c_scl_oe = 1'b1; #(I2C_HALF);  // raise SCL
            i2c_sda_oe = 1'b1; #(I2C_HALF);  // SDA rises → STOP
        end
    endtask

    // Execute one EFB I2C1 CR register command.
    // CR bits: STA[7] STO[6] RD[5] WR[4] ACK[3] (ACK=1→NACK after read)
    task automatic i2c_do_cr;
        input [7:0] cr;
        reg got_ack;
        reg [7:0] rx;
        begin
            i2c_tip = 1'b1;
            if (cr[7]) i2c_start();
            if (cr[4]) i2c_write_byte(i2c_txdr, got_ack);
            if (cr[5]) begin
                i2c_read_byte(cr[3], rx);
                i2c_rxdr = rx;
            end
            if (cr[6]) i2c_stop();
            i2c_tip = 1'b0;
        end
    endtask

    // -------------------------------------------------------------------------
    // I2C master process — waits on i2c_cr_event then executes CR command
    // -------------------------------------------------------------------------
    initial begin
        i2c_tip    = 1'b0;
        i2c_rxdr   = 8'h00;
        i2c_scl_oe = 1'b1;   // bus idle: SCL=H
        i2c_sda_oe = 1'b1;   // bus idle: SDA=H
        forever @(i2c_cr_event) begin
            i2c_do_cr(i2c_cr_cmd);
        end
    end

    // -------------------------------------------------------------------------
    // Tie-off all unused outputs
    // -------------------------------------------------------------------------
    assign PLL0DATO   = 8'h00;
    assign PLL0STBO   = 1'b0;
    assign PLL1DATO   = 8'h00;
    assign PLL1STBO   = 1'b0;
    assign PLLCLKO    = 1'b0;
    assign PLLRSTO    = 1'b0;
    assign PLLWEO     = 1'b0;
    assign {PLLADRO4, PLLADRO3, PLLADRO2, PLLADRO1, PLLADRO0} = 5'b0;
    assign {PLLDATO7, PLLDATO6, PLLDATO5, PLLDATO4,
            PLLDATO3, PLLDATO2, PLLDATO1, PLLDATO0} = 8'h00;
    assign I2C1SCLO   = 1'b1;
    assign I2C1SDAO   = 1'b1;
    assign I2C1IRQ    = 1'b0;
    assign I2C2SCLO   = 1'b1;
    assign I2C2SDAO   = 1'b1;
    assign I2C2SCLOEN = 1'b0;
    assign I2C2SDAOEN = 1'b0;
    assign I2C2IRQ    = 1'b0;
    assign SPIMISOO   = 1'b1;
    assign SPIIRQ     = 1'b0;
    assign SPICSNO    = 1'b1;
    assign SPIMOSIO   = 1'b0;
    assign SPICSO     = 1'b0;
    assign SPICLKO    = 1'b0;
    assign TCINT      = 1'b0;
    assign TCOC       = 1'b0;
    assign TCOCOE     = 1'b0;
    assign TCLIN      = 1'b0;
    assign TCGSR      = 1'b0;
    assign WBINT      = 1'b0;
    assign WBCUFMIRQ  = 1'b0;
    assign CFGWAKE    = 1'b0;
    assign CFGSTDBY   = 1'b0;

endmodule
/* verilator lint_on CASEINCOMPLETE */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on BLKSEQ */
/* verilator lint_on DECLFILENAME */
