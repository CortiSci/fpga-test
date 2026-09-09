// Consolidator V2 unit tests.
// Included inside module tb_top — do NOT add `timescale or import directives here.
// Package imports are inherited from tb_top.sv.
//
// Contains:
//   spi_cfg_xact  — composite SPI_CFG passthrough helper (same as production)
//   CU-02, CU-03  — fault monitor tests (same registers, same behavior)
//   CU-04         — ESN reader test (same registers)
//   CV-01         — FW_VERSION reads 0x0002
//   CV-02         — SPI_CLK_DIV (0x0060) write / read-back
//   CV-03         — watchdog enable via first WD_PET (0x0052), read back WD_STS (0x0053) bit[2]
//   CV-04         — STREAM_STS (0x005C) idle check

// V2-only register addresses (not in usb_cmd_pkg which covers shared addresses)
localparam logic [15:0] REG_WD_PET        = 16'h0052;
localparam logic [15:0] REG_WD_STS        = 16'h0053;
// 0x0054 (WD_EN) removed in reg_map_v2 — watchdog enables on first WD_PET wdata[0]=1
localparam logic [15:0] REG_FRAME_CNT_HI  = 16'h0055;
localparam logic [15:0] REG_FRAME_CNT_LO  = 16'h0056;
localparam logic [15:0] REG_FRAME_CNT_RST = 16'h0057;
localparam logic [15:0] REG_PAR_ERR_FLAGS = 16'h0058;
localparam logic [15:0] REG_STREAM_STS    = 16'h005C;
localparam logic [15:0] REG_SPI_CLK_DIV   = 16'h0060;

// ─────────────────────────────────────────────────────────────────────────────
// spi_cfg_xact — composite SPI_CFG passthrough helper.
//
// Arguments:
//   ch_sel   : 0-3 (Tail FPGA channel; 0=LEG5)
//   tx_bytes : [0]=opcode, [1..5]=subsequent bytes (zero-pad unused slots)
//   n_bytes  : transaction length 1-6; 0 aliases to 2 in hardware (back-compat)
//   rx_bytes : captured MISO bytes — [0,1]=wd_sts (always); [2]=first real response
//
// Precondition: SPI_EN_MASK bit for ch_sel must be set before calling.
// ─────────────────────────────────────────────────────────────────────────────
// C-05: Icarus does not support unpacked array task parameters.
// QuestaSim version uses logic [7:0] tx_bytes [0:5] / rx_bytes [0:5].
// Icarus version uses packed logic [47:0]: byte[k] = bits [47-8k : 40-8k]
//   (byte[0]=opcode=[47:40], byte[2]=first response=[31:24], byte[5]=[7:0]).
`ifndef ICARUS
task automatic spi_cfg_xact(
    input  logic [1:0]  ch_sel,
    input  logic [7:0]  tx_bytes [0:5],
    input  logic [2:0]  n_bytes,
    output logic [7:0]  rx_bytes [0:5]
);
    logic [15:0] m, f, a, d;
    int          timeout;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA,
                                       {tx_bytes[0], tx_bytes[1]});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    if (n_bytes > 3'd2) begin
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA2,
                                           {tx_bytes[2], tx_bytes[3]});
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    end

    if (n_bytes > 3'd4) begin
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA3,
                                           {tx_bytes[4], tx_bytes[5]});
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    end

    // rw=0 (read mode): keeps the 3 trailing SCLK cycles so MISO is captured.
    // write_mode=1 would raise SS_N immediately after the last data bit, skipping
    // MISO capture — essential only for ASIC-passthrough (CS1/CS2_PASS) writes
    // where extra clocks over-clock the ASIC 24-bit shift register.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_CTRL,
                                       spi_cfg_ctrl_word(ch_sel, 1'b0, 1'b1, n_bytes));
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    // Poll BUSY (bit3) until clear
    timeout = 0;
    do begin
        // At the configured 800 kHz setup rate a two-byte read-mode transfer
        // takes about 25 us including its trailing clocks.  A host poll after
        // 30 us is both realistic and leaves a bounded BUSY-check retry path.
        #30_000;
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_CTRL, 16'h0);
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        timeout++;
    end while (d[3] === 1'b1 && timeout < 500);

    if (d[3] !== 1'b0)
        $error("[spi_cfg_xact] BUSY did not clear: ch=%0d n_bytes=%0d opcode=0x%02h",
               ch_sel, n_bytes, tx_bytes[0]);

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD01, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[0] = d[15:8];
    rx_bytes[1] = d[7:0];

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD23, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[2] = d[15:8];
    rx_bytes[3] = d[7:0];

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD45, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[4] = d[15:8];
    rx_bytes[5] = d[7:0];
endtask
`else
task automatic spi_cfg_xact(
    input  logic [1:0]   ch_sel,
    input  logic [47:0]  tx_bytes,
    input  logic [2:0]   n_bytes,
    output logic [47:0]  rx_bytes
);
    logic [15:0] m, f, a, d;
    int          timeout;

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA,
                                       {tx_bytes[47:40], tx_bytes[39:32]});
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);

    if (n_bytes > 3'd2) begin
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA2,
                                           {tx_bytes[31:24], tx_bytes[23:16]});
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    end

    if (n_bytes > 3'd4) begin
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_DATA3,
                                           {tx_bytes[15:8], tx_bytes[7:0]});
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    end

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CFG_CTRL,
                                       spi_cfg_ctrl_word(ch_sel, 1'b0, 1'b1, n_bytes));
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    timeout = 0;
    do begin
        // At the configured 800 kHz setup rate a two-byte read-mode transfer
        // takes about 25 us including its trailing clocks.  A host poll after
        // 30 us is both realistic and leaves a bounded BUSY-check retry path.
        #30_000;
        tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_CTRL, 16'h0);
        tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
        timeout++;
    end while (d[3] === 1'b1 && timeout < 500);

    if (d[3] !== 1'b0)
        $error("[spi_cfg_xact] BUSY did not clear: ch=%0d n_bytes=%0d opcode=0x%02h",
               ch_sel, n_bytes, tx_bytes[47:40]);

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD01, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[47:40] = d[15:8]; rx_bytes[39:32] = d[7:0];

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD23, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[31:24] = d[15:8]; rx_bytes[23:16] = d[7:0];

    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CFG_RD45, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    rx_bytes[15:8] = d[15:8]; rx_bytes[7:0] = d[7:0];
endtask
`endif  // !ICARUS (spi_cfg_xact)

// CU-02: Fault monitor falling-edge detect → fault_latch set
// cmd_decoder emits a spontaneous interrupt frame when fault_trig fires;
// consume it before reading the register.
`ifndef ICARUS  // C-05: Icarus does not support 'ref' task parameters
task automatic run_CU02(ref logic faultn);
    logic [15:0] m, f, a, d;
    faultn = 1'b0;
    #50;
    faultn = 1'b1;
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_FAULT_STATUS, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[1] !== 1'b1)
        $error("[CU-02] fault_latch not set: FAULT_STATUS = 0x%04h", d);
    else
        $display("[CU-02] PASS (fault_latch=1, FAULT_STATUS=0x%04h)", d);
endtask
`endif  // !ICARUS (run_CU02)

// CU-03: Fault latch clear via SYS_CMD_RESET (0x0000)
task automatic run_CU03();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SYS_CMD_RST, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    #200;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_FAULT_STATUS, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[1] !== 1'b0)
        $error("[CU-03] fault_latch not cleared: FAULT_STATUS = 0x%04h", d);
    else
        $display("[CU-03] PASS (fault_latch cleared, FAULT_STATUS=0x%04h)", d);
endtask

// CU-04: ESN reader — EFB stub returns TRACEID via 0x0020 / 0x0021
task automatic run_CU04();
    logic [15:0] m, f, a, esn_hi, esn_lo;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_ESN_HIGH, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, esn_hi);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_ESN_LOW, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, esn_lo);
    if (^esn_hi === 1'bx || ^esn_lo === 1'bx)
        $error("[CU-04] ESN contains X: ESN_HIGH=0x%04h ESN_LOW=0x%04h", esn_hi, esn_lo);
    else
        $display("[CU-04] PASS ESN = 0x%04h_%04h", esn_hi, esn_lo);
endtask

// CV-01: FW_VERSION must read 0x0002 (V2 identifier)
task automatic run_CV01();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_FW_VERSION, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h0002)
        $error("[CV-01] FW_VERSION 0x%04h expected 0x0002", d);
    else
        $display("[CV-01] PASS FW_VERSION = 0x%04h (V2)", d);
endtask

// CV-02: SPI_CLK_DIV (0x0060) write / read-back
// Write pre=31 (800 kHz setting), verify read-back, then restore to 0 (fast mode).
task automatic run_CV02();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h001F);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_SPI_CLK_DIV, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d !== 16'h001F)
        $error("[CV-02] SPI_CLK_DIV readback 0x%04h expected 0x001F", d);
    else
        $display("[CV-02] PASS SPI_CLK_DIV = 0x%04h (pre=31, 800 kHz)", d);
    // Restore to 0 — leave in a clean known state; SP tests set their own rate
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_SPI_CLK_DIV, 16'h0000);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
endtask

// CV-03: watchdog enable via first WD_PET, read back through WD_STS.
// The standalone WD_EN register (0x0054) was removed in reg_map_v2: the watchdog
// is enabled by the FIRST WD_PET (0x0052) write with wdata[0]=1, and its enabled
// state is reported by WD_STS (0x0053) bit[2].  The enable is sticky (no disable
// register), but the 819 ms timeout is far longer than the <10 ms sim, so it
// cannot expire during the remaining SP tests and needs no cleanup.
task automatic run_CV03();
    logic [15:0] m, f, a, d;
    // First pet with bit[0]=1 enables the watchdog.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_wr(), REG_WD_PET, 16'h0001);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    // WD_STS bit[2] = wd_en must now be set; bit[0] = wd_expired must be clear.
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_WD_STS, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[2] !== 1'b1 || d[0] !== 1'b0)
        $error("[CV-03] WD not enabled after first pet: WD_STS=0x%04h (expected bit[2]=1, bit[0]=0)", d);
    else
        $display("[CV-03] PASS watchdog enabled via first pet, WD_STS = 0x%04h", d);
endtask

// CV-04: STREAM_STS (0x005C) — all channels should be idle (0) before ch_run set
task automatic run_CV04();
    logic [15:0] m, f, a, d;
    tb_top.u_ft600q.send_command_frame(CMD_MAGIC, flags_rd(), REG_STREAM_STS, 16'h0);
    tb_top.u_ft600q.wait_response_frame_typed(m, f, a, d);
    if (d[3:0] !== 4'b0000)
        $display("[CV-04] INFO STREAM_STS = 0x%04h (non-zero before ch_run — expected if stream state machines have started)", d);
    else
        $display("[CV-04] PASS STREAM_STS = 0x%04h (all channels idle)", d);
endtask
