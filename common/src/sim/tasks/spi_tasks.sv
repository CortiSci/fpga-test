// SPI task library for direct Tail FPGA testing (TU tests).
// Re-exports the spi_master_bfm task API as convenience functions and
// adds higher-level Tail FPGA command sequences.
// The spi_master_bfm instance is tb_top.spi_bfm[ch] in the testbench.
`timescale 1ns/1ps

package spi_tasks_pkg;

    // -------------------------------------------------------------------------
    // ASIC data generation mode (shared with ucsd_asic_model and test tasks)
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        MODE_RAMP         = 3'd0,
        MODE_CONSTANT     = 3'd1,
        MODE_SINE         = 3'd2,
        MODE_PATTERN_FILE = 3'd3,
        MODE_IDLE         = 3'd4,
        MODE_UNIQUE       = 3'd5
    } asic_mode_t;

    // -------------------------------------------------------------------------
    // CDC FIFO read-buffer result returned by send_cmd_read_buffer
    // C-05: unpacked struct not supported by Icarus — guarded.
    // -------------------------------------------------------------------------
`ifndef ICARUS
    typedef struct {
        logic [7:0]  status;    // raw STATUS byte (bits[7:6] = frame type)
        int          n_words;   // 16 when STATUS[7]==1, 0 otherwise
        logic [15:0] words [0:15];
    } tail_buf_status_t;
`endif

    // Tail FPGA SPI command opcodes (from user_spi_slave.v)
    localparam logic [7:0] SPI_CMD_READ_BUFFER      = 8'h01;
    localparam logic [7:0] SPI_CMD_READ_BUFFER_ENA  = 8'h04;  // byte=0→enable, byte=1→reset
    localparam logic [7:0] SPI_CMD_ASIC_SPI_CS0     = 8'h10;
    localparam logic [7:0] SPI_CMD_ASIC_SPI_CS1 = 8'h11;
    localparam logic [7:0] SPI_CMD_SET_CTRL      = 8'h20;
    localparam logic [7:0] SPI_CMD_READ_ADC      = 8'h30;
    localparam logic [7:0] SPI_CMD_I2C_WRITE     = 8'h40;
    localparam logic [7:0] SPI_CMD_I2C_READ      = 8'h50;
    localparam logic [7:0] SPI_CMD_PING          = 8'hAA;
    localparam logic [7:0] SPI_CMD_FPGA_RESET    = 8'hFF;

    // ctrl_val bits for CMD_SET_CTRL data byte
    // Mapping: ctrl_reg[4]=RO_RST, [3]=MCLK_EN, [2]=TEST_SIG, [1]=AMP_SHDN, [0]=LED_ON
    // Power-on default: RO_RST=1 (ASIC held in reset), AMP_SHDN=1.
    // Write CTRL_MCLK_EN alone (5'b0_1000) to release ASIC reset AND enable MCLK.
    localparam logic [4:0] CTRL_RO_RST    = 5'b1_0000;  // 1=assert ASIC reset (RO_RSTn=0); 0=release
    localparam logic [4:0] CTRL_MCLK_EN   = 5'b0_1000;
    localparam logic [4:0] CTRL_TEST_SIG  = 5'b0_0100;
    localparam logic [4:0] CTRL_AMP_SHDN  = 5'b0_0010;  // active-high shuts down amp; default on
    localparam logic [4:0] CTRL_LED_ON    = 5'b0_0001;

endpackage
