// Generic SPI Master Bus Functional Model
// Mode 0 (CPOL=0, CPHA=0), MSB-first, byte-oriented.
// Default clock period = 19.531 ns (51.2 MHz) — matches Tail FPGA spi_clk domain.
// Connect to Tail FPGA FPGA_SPI_{SCLK,MOSI,MISO,SS} for TU tests.
//
// Protocol note — user_spi_slave ss_n_d initialization:
//   The Tail FPGA user_spi_slave registers ss_n_d on SCLK posedges.
//   The first SCLK posedge after SS_N falls fires an "SS_N just fell" branch
//   that samples MOSI (opcode MSB) and initialises bit_cnt=6.  The BFM sets
//   MOSI to opcode[7] before asserting SS_N so the ss_n_d branch captures
//   the correct MSB; no dummy pulse is needed.
//     MOSI=opcode[7] → SS_N=0 → opcode byte (8 posedges) → ...
//
// Pipeline depth — trailing SCLK pulses:
//   spi_cmd_decoder and ctrl_output_regs are both clocked by FPGA_SPI_SCLK.
//   rx_valid from user_spi_slave is registered (visible one cycle after the
//   8th data bit).  Each downstream register stage needs its own SCLK pulse:
//     T1 → spi_cmd_decoder samples rx_valid, schedules ctrl_wr_en<=1
//     T2 → ctrl_output_regs samples ctrl_wr_en=1, schedules ctrl_reg<=ctrl_data
//   Opcode-only commands need one more cycle through S_DECODE → action state.
`timescale 1ns/1ps

// C-05: tail_buf_status_t is an unpacked struct, not supported by Icarus.
`ifndef ICARUS
module spi_master_bfm
    import spi_tasks_pkg::tail_buf_status_t;
#(
`else
module spi_master_bfm
#(
`endif
    parameter real CLK_PERIOD_NS = 19.531 // 51.2 MHz default
) (
    output reg sclk,
    output reg mosi,
    input  wire miso,
    output reg ss_n
);

    initial begin
        sclk = 1'b0;
        mosi = 1'b0;
        ss_n = 1'b1;
    end

    // =========================================================================
    // Internal helpers
    // =========================================================================

    // Transfer one byte; rdata returns the byte shifted in from MISO.
    //
    // MISO is sampled on the FALLING edge (after sclk=0) rather than the
    // rising edge.  user_spi_slave drives miso via NBA at the posedge, so
    // reading in the active region of the same posedge captures the *previous*
    // cycle's value — a 1-bit right-shift of every response byte.  Sampling
    // at the negedge gives the fully-settled NBA value. (MOSI is still set
    // before the posedge; user_spi_slave captures it on the rising edge. ✓)
    task automatic spi_send_byte(
        input  logic [7:0] data,
        output logic [7:0] rdata
    );
        automatic int i;
        for (i = 7; i >= 0; i--) begin
            mosi = data[i];
            #(CLK_PERIOD_NS / 2.0);
            sclk = 1'b1;
            #(CLK_PERIOD_NS / 2.0);
            sclk = 1'b0;
            rdata[i] = miso;   // sample after falling edge — NBA has settled
        end
    endtask

    // Generate N SCLK pulses with SS_N=1 (idle bus).  Used in fork alongside
    // inject_adc_result so drdy_monitor's 3-FF synchroniser sees sclk posedges
    // while DRDY_N is pulsing low.
    task automatic idle_clocks(input int n);
        automatic int i;
        for (i = 0; i < n; i++) begin
            #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        end
    endtask

    // =========================================================================
    // Task API
    // =========================================================================

    // Full SPI transaction: assert SS, send opcode, send/receive data bytes,
    // two trailing pulses, deassert SS.
    // C-05: dynamic array params not supported by Icarus — guarded.
`ifndef ICARUS
    task automatic spi_transaction(
        input  logic [7:0] cmd,
        input  logic [7:0] tx_data [],
        output logic [7:0] rx_data []
    );
        automatic int i;
        automatic logic [7:0] dummy;
        ss_n = 1'b0;
        spi_send_byte(cmd, dummy);
        for (i = 0; i < tx_data.size(); i++) begin
            spi_send_byte(tx_data[i], rx_data[i]);
        end
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
    endtask

    // CMD_SET_CTRL (0x20): set Tail FPGA control outputs
    // C-05: dynamic array literals not supported by Icarus — guarded.
    task automatic send_cmd_set_ctrl(input logic [4:0] ctrl_val);
        automatic logic [7:0] tx[] = '{8'(ctrl_val)};
        automatic logic [7:0] rx[] = new[1];
        spi_transaction(8'h20, tx, rx);
    endtask

    // CMD_READ_BUFFER (0x01): fixed 35-byte burst (opcode + 34 dummy bytes).
    //   MISO layout: byte0=wd_sts, byte1=wd_sts, byte2=STATUS[7:6], bytes3-34=16×HI/LO.
    //   STATUS[7:6]: 2'b10=valid-continue, 2'b11=new-frame → 16 words captured.
    //                2'b00=empty, 2'b01=overflow              → n_words=0.
    // C-05: struct output param not supported by Icarus — guarded.
    task automatic send_cmd_read_buffer(
        output tail_buf_status_t status_out
    );
        automatic logic [7:0] dummy;
        automatic logic [7:0] status_byte;
        automatic logic [7:0] hi, lo;
        automatic logic       burst_valid;
        automatic int i;

        ss_n = 1'b0;
        spi_send_byte(8'h01, dummy);        // byte 0: opcode; MISO = wd_sts (discard)
        spi_send_byte(8'h00, dummy);        // byte 1: dummy;  MISO = wd_sts (discard)
        spi_send_byte(8'h00, status_byte);  // byte 2: dummy;  MISO = STATUS
        burst_valid = status_byte[7];       // 1 for 2'b10 (valid) and 2'b11 (new-frame)
        for (i = 0; i < 16; i++) begin      // bytes 3-34: 16 HI/LO pairs
            spi_send_byte(8'h00, hi);
            spi_send_byte(8'h00, lo);
            if (burst_valid)
                status_out.words[i] = {hi, lo};
        end
        status_out.status  = status_byte;
        status_out.n_words = burst_valid ? 16 : 0;
        // Two trailing pulses so spi_cmd_decoder FSM returns to S_WAIT_RX.
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
    endtask
`endif  // !ICARUS (spi_transaction / send_cmd_set_ctrl / send_cmd_read_buffer)

    // CMD_ASIC_SPI_CS0/1 (0x10/0x11): relay SPI bytes to the UCSD ASIC.
    // C-05: dynamic array params not supported by Icarus — guarded.
`ifndef ICARUS
    task automatic send_cmd_asic_spi(
        input  logic        cs_sel,
        input  logic [7:0]  tx_data [],
        output logic [7:0]  rx_data []
    );
        automatic logic [7:0] cmd = cs_sel ? 8'h11 : 8'h10;
        automatic logic [7:0] dummy;
        automatic int n = tx_data.size();
        automatic int i;

        // 1. Stage all bytes into the decoder staging buffer via CMD_ASIC_BUF.
        ss_n = 1'b0;
        spi_send_byte(8'h18 | ((n - 1) & 7), dummy);  // CMD_ASIC_BUF for n bytes
        for (i = 0; i < n; i++)
            spi_send_byte(tx_data[i], rx_data[i]);     // stage; readback = spa_rd_data
        // T1, T2: extra margin (decoder is already in S_WAIT_RX after last byte).
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);

        // 2. Trigger ASIC SPI execution.
        // T1: S_DECODE → S_ASPI_TRIG (exec_byte_cnt=buf_ptr, exec_trigger_tog)
        // T2: S_ASPI_TRIG → S_WAIT_RX
        ss_n = 1'b0;
        spi_send_byte(cmd, dummy);
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;  // T1
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;  // T2
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;

        // 3. 65 idle sclk pulses so asic_spi_ctrl executes and the exec_busy_s
        //    toggle propagates through the spi_clk 2-FF synchroniser.
        for (i = 0; i < 65; i++) begin
            #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        end
        #(CLK_PERIOD_NS);
    endtask

    // CMD_READ_ADC (0x30): read last ADC result from Tail FPGA
    task automatic send_cmd_read_adc(output logic [15:0] result);
        automatic logic [7:0] dummy, hi, lo;
        ss_n = 1'b0;
        spi_send_byte(8'h30, dummy);   // opcode
        // Flush byte: decoder pipeline reaches S_TX_BYTE and loads spi_tx_byte=hi
        // at posedge 12 (3 cycles after opcode RELOAD).  S_ADC_LO then waits for
        // THIS byte's RELOAD (posedge 16) before overwriting spi_tx_byte with lo,
        // so RELOAD-2 picks up hi and RELOAD-3 picks up lo correctly.
        spi_send_byte(8'h00, dummy);   // flush — slave sends garbage, decoder pipelines hi
        spi_send_byte(8'h00, hi);      // slave sends hi (RELOADed at flush RELOAD)
        spi_send_byte(8'h00, lo);      // slave sends lo (RELOADed at hi-read RELOAD)
        result = {hi, lo};
        // Two trailing pulses — FSM already at S_WAIT_RX; pulses just for timing margin.
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
    endtask

    // CMD_PING (0xAA): confirm SPI link; Tail FPGA returns 0x55 in the
    // response byte.  Extra inter-byte pulses let spi_cmd_decoder load
    // tx_byte=0x55 before the slave shifts it out.
    task automatic send_cmd_ping(output logic [7:0] response);
        automatic logic [7:0] dummy;
        ss_n = 1'b0;
        spi_send_byte(8'hAA, dummy);
        // 8 inter-byte pulses needed:
        //   P1: S_WAIT_RX → S_DECODE (opcode rx_valid seen)
        //   P2: S_DECODE  → S_TX_BYTE (0xAA decoded, tx_pending=0x55 set)
        //   P3: S_TX_BYTE → S_WAIT_RX (spi_tx_byte=0x55 latched)
        //   P4-P7: bit_cnt decrements 7→4 (bit_cnt→0 NBA fires at end of P7)
        //   P8: bit_cnt IS 0 → RELOAD fires, shift_tx=0x55 loaded; miso=0x55[7]
        //       before BFM byte-2 first posedge. With only 7 pulses, RELOAD and
        //       byte-2 posedge-1 coincide, so BFM reads the pre-RELOAD miso (0).
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        spi_send_byte(8'h00, response);
        // Two trailing pulses so FSM returns to S_WAIT_RX.
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
    endtask

    // CMD_READ_BUFFER_ENA (0x04): enable or reset the CDC read buffer.
    // C-05: dynamic array literals not supported by Icarus — guarded.
    task automatic send_cmd_read_buffer_ena(input logic reset);
        automatic logic [7:0] tx[] = '{reset ? 8'h01 : 8'h00};
        automatic logic [7:0] rx[] = new[1];
        spi_transaction(8'h04, tx, rx);
    endtask
`endif  // !ICARUS (send_cmd_asic_spi / send_cmd_read_buffer_ena)

    // CMD_FPGA_RESET (0xFF): soft-reset the Tail FPGA
    task automatic send_cmd_fpga_reset();
        automatic logic [7:0] dummy;
        ss_n = 1'b0;
        spi_send_byte(8'hFF, dummy);
        // 3 trailing pulses (opcode-only command, 3-stage pipeline):
        //   T1: spi_cmd_decoder samples rx_valid → S_DECODE
        //   T2: S_DECODE → S_RESET
        //   T3: S_RESET fires soft_reset → S_IDLE
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
        ss_n = 1'b1;
        #(CLK_PERIOD_NS / 2.0); sclk = 1'b1; #(CLK_PERIOD_NS / 2.0); sclk = 1'b0;
        #(CLK_PERIOD_NS);
    endtask

endmodule
