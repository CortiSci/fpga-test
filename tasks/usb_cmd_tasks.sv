// USB command task library — wrapper around ft600q_tlm task API.
// Provides named helpers for the Consolidator register map operations.
// Import usb_cmd_pkg and reference the ft600q_tlm instance u_ft600q from tb_top.
`timescale 1ns/1ps

// Consolidator command frame format (from cmd_decoder.v):
//   word 0: magic (0xAA55)
//   word 1: flags  [0]=R/W (1=write), [15:1]=reserved
//   word 2: address (16-bit register address)
//   word 3: data    (write value / ignored for reads)
//
// Response frame format:
//   word 0: magic (0x55AA)
//   word 1: flags (echo)
//   word 2: address (echo)
//   word 3: data (read value / write echo)

package usb_cmd_pkg;

    localparam logic [15:0] CMD_MAGIC = 16'hAA55;
    localparam logic [15:0] RSP_MAGIC = 16'h55AA;

    // -------------------------------------------------------------------------
    // Register addresses (from reg_map.v)
    // -------------------------------------------------------------------------
    // System
    localparam logic [15:0] REG_SYS_CMD_RST    = 16'h0000;  // WO: [0]=soft reset pulse
    localparam logic [15:0] REG_FAULT_STATUS   = 16'h0001;  // RO: [0]=fault_stat [1]=fault_latch
    localparam logic [15:0] REG_SPI_EN_MASK    = 16'h0002;  // RW: [3:0]=Ch5-8 enables
    localparam logic [15:0] REG_DEVICE_CTRL   = 16'h0003;  // RW: [0]=LED_ON [1]=MCLK_EN
    localparam logic [15:0] REG_TOKEN_HI       = 16'h0004;  // RW: magic token [31:16]
    localparam logic [15:0] REG_TOKEN_LO       = 16'h0005;  // RW: magic token [15:0]
    localparam logic [15:0] REG_FW_VERSION     = 16'h0006;  // RO: [15:8]=major [7:0]=minor

    // GPIO / daisy chain
    localparam logic [15:0] REG_GPIO_DAISY     = 16'h0010;  // RW: [0]=daisy_out_val [1]=daisy_in_raw(RO)
    localparam logic [15:0] REG_USB_GPIO       = 16'h0011;  // RW: wr[1:0]=out_val [3:2]=oe; rd[1:0]=pin_state [3:2]=oe
    localparam logic [15:0] REG_RESERVE_MISO   = 16'h0012;  // RO: [0]=MISO_LEG1 [1]=LEG2 [2]=LEG3 [3]=LEG9

    // ESN (read-only)
    localparam logic [15:0] REG_ESN_HIGH       = 16'h0020;  // RO: TRACEID[31:16]
    localparam logic [15:0] REG_ESN_LOW        = 16'h0021;  // RO: TRACEID[15:0]

    // SPI config path — TX data registers
    localparam logic [15:0] REG_SPI_CFG_DATA   = 16'h0030;  // RW: [15:8]=opcode(byte0) [7:0]=byte1
    localparam logic [15:0] REG_SPI_CFG_CTRL   = 16'h0031;  // WR: [7:5]=n_bytes [3]=go [2]=rw [1:0]=ch_sel
                                                             // RD: [7:5]=n_bytes [3]=busy [2]=rw [1:0]=ch_sel
    localparam logic [15:0] REG_SPI_CFG_DATA2  = 16'h0032;  // RW: [15:8]=byte2 [7:0]=byte3
    localparam logic [15:0] REG_SPI_CFG_DATA3  = 16'h0033;  // RW: [15:8]=byte4 [7:0]=byte5
    // SPI config path — RX capture registers (updated when BUSY clears)
    localparam logic [15:0] REG_SPI_CFG_RD01   = 16'h0034;  // RO: [15:8]=rx_byte0(wd_sts) [7:0]=rx_byte1(wd_sts)
    localparam logic [15:0] REG_SPI_CFG_RD23   = 16'h0035;  // RO: [15:8]=rx_byte2(response) [7:0]=rx_byte3
    localparam logic [15:0] REG_SPI_CFG_RD45   = 16'h0036;  // RO: [15:8]=rx_byte4 [7:0]=rx_byte5

    // Per-channel registers: base + (ch × 0x10), ch = 0..3 (LEG5..LEG8)
    localparam logic [15:0] REG_CH0_CTRL       = 16'h0100;  // RW: [0]=run [1]=local_rst
    localparam logic [15:0] REG_CH0_STATUS     = 16'h0101;  // RO: [0]=empty [1]=full [7]=overflow
    localparam logic [15:0] REG_CH0_WORD_CNT   = 16'h0102;  // RO: [9:0]=word count
    localparam logic [15:0] REG_CH1_CTRL       = 16'h0110;
    localparam logic [15:0] REG_CH1_STATUS     = 16'h0111;
    localparam logic [15:0] REG_CH1_WORD_CNT   = 16'h0112;
    localparam logic [15:0] REG_CH2_CTRL       = 16'h0120;
    localparam logic [15:0] REG_CH2_STATUS     = 16'h0121;
    localparam logic [15:0] REG_CH2_WORD_CNT   = 16'h0122;
    localparam logic [15:0] REG_CH3_CTRL       = 16'h0130;
    localparam logic [15:0] REG_CH3_STATUS     = 16'h0131;
    localparam logic [15:0] REG_CH3_WORD_CNT   = 16'h0132;
    localparam logic [15:0] REG_ACQ_ALL_RUN    = 16'h0140;  // WO: [3:0]=ch_run[3:0] broadcast

    // Convenience array for channel-N ctrl/status/word_cnt
    function automatic logic [15:0] ch_ctrl_addr(input int ch);
        return 16'h0100 + (ch * 16'h0010);
    endfunction
    function automatic logic [15:0] ch_status_addr(input int ch);
        return 16'h0101 + (ch * 16'h0010);
    endfunction
    function automatic logic [15:0] ch_word_cnt_addr(input int ch);
        return 16'h0102 + (ch * 16'h0010);
    endfunction

    // -------------------------------------------------------------------------
    // Flags field helpers
    // -------------------------------------------------------------------------
    function automatic logic [15:0] flags_wr(); return 16'h0001; endfunction
    function automatic logic [15:0] flags_rd(); return 16'h0000; endfunction

    // -------------------------------------------------------------------------
    // SPI CFG register word helpers
    // -------------------------------------------------------------------------

    // SPI_CFG_CTRL write word.
    // n_bytes: transaction length 1-6; default 0 aliases to 2 in hardware (backward compat).
    // Existing call sites that pass only ch_sel/rw/go (3 args) continue to work unchanged
    // because SV default arguments default n_bytes to 3'd0 → hardware treats as n_bytes=2.
    function automatic logic [15:0] spi_cfg_ctrl_word(
        input logic [1:0] ch_sel,
        input logic       rw,
        input logic       go,
        input logic [2:0] n_bytes = 3'd0  // 0 = backward-compat alias for 2-byte transaction
    );
        return {8'h0, n_bytes, 1'b0, go, rw, ch_sel};
    endfunction

    // SPI_CFG_DATA word helper (TX bytes 0 and 1)
    function automatic logic [15:0] spi_cfg_data_word(
        input logic [7:0] reg_addr,
        input logic [7:0] wr_data
    );
        return {reg_addr, wr_data};
    endfunction

    // SPI_CFG_DATA2 word helper (TX bytes 2 and 3)
    function automatic logic [15:0] spi_cfg_data2_word(
        input logic [7:0] byte2,
        input logic [7:0] byte3
    );
        return {byte2, byte3};
    endfunction

    // SPI_CFG_DATA3 word helper (TX bytes 4 and 5)
    function automatic logic [15:0] spi_cfg_data3_word(
        input logic [7:0] byte4,
        input logic [7:0] byte5
    );
        return {byte4, byte5};
    endfunction

endpackage
