// ADS122C14IRTER I2C Slave Stub — address 0x40
// Implements: WREG (0x4x), RREG (0x2x), RDATA (0x10).
// Drives drdy_n low for one I2C byte period after inject_adc_result().
// I2C open-drain modelled with pull-up: slave can only drive lines low.
`timescale 1ns/1ps

module ads122c14_i2c_model (
    inout wire scl,
    inout wire sda,
    output reg drdy_n
);

    // Pull-ups (model as 1 when not driven)
    wire scl_in = scl;
    wire sda_in = sda;
    reg  sda_drive = 1'b0;  // 1 = driving sda low
    assign sda = sda_drive ? 1'b0 : 1'bz;

    // Internal register file: 4 config registers + 1 result register
    reg [7:0] cfg_reg [0:3];
    reg [15:0] result_reg = 16'h0;

    localparam I2C_ADDR = 7'h40;

    // I2C state machine
    typedef enum logic [3:0] {
        I2C_IDLE       = 4'd0,
        I2C_ADDR_RX    = 4'd1,
        I2C_ADDR_ACK   = 4'd2,
        I2C_CMD_RX     = 4'd3,
        I2C_CMD_ACK    = 4'd4,
        I2C_DATA_TX    = 4'd5,
        I2C_DATA_ACK   = 4'd6,
        I2C_DATA_RX    = 4'd7,
        I2C_DATA_WACK  = 4'd8,
        // Absorbs the phantom SCL negedge produced by efb_stub i2c_start()
        // (scl_oe=0 at the end of i2c_start fires @negedge scl_in before any
        // address bit is driven).  The next real SCL negedge enters I2C_ADDR_RX.
        I2C_START_SKIP = 4'd9
    } i2c_state_t;

    i2c_state_t state = I2C_IDLE;

    reg [7:0]  shift;
    reg [2:0]  bit_cnt;
    reg        rw_bit;
    reg [7:0]  cmd_byte;
    reg [7:0]  tx_byte = 8'h00;  // initialised so MSB is valid before first load
    reg        tx_phase;    // 0 = high byte, 1 = low byte
    int        data_byte_idx;

    // Start condition detection
    // sda_prev tracks SDA with 1ns lag so SCL rising with SDA already-low
    // (data bit 0) does not trigger a spurious START condition.
    reg scl_prev = 1'b1;
    reg sda_prev = 1'b1;
    always @(sda_in) #1 sda_prev <= sda_in;
    wire start_cond = scl_in && sda_prev && !sda_in;
    wire stop_cond  = scl_in && !sda_prev && sda_in;

    always @(negedge scl_in or posedge start_cond) begin
        if (start_cond) begin
            state     <= I2C_START_SKIP;  // absorb phantom SCL negedge from i2c_start()
            bit_cnt   <= 3'd7;
            sda_drive <= 1'b0;
        end else begin
            case (state)
                I2C_ADDR_RX: begin
                    shift <= {shift[6:0], sda_in};
                    if (bit_cnt == 3'd0) begin
                        rw_bit  <= sda_in;
                        bit_cnt <= 3'd7;
                        if (shift[6:0] == I2C_ADDR[6:0]) begin
                            state     <= I2C_ADDR_ACK;
                            sda_drive <= 1'b1;  // ACK
                        end else begin
                            state <= I2C_IDLE;
                        end
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end

                I2C_ADDR_ACK: begin
                    sda_drive <= 1'b0;
                    if (rw_bit == 1'b0) begin
                        state   <= I2C_CMD_RX;
                        bit_cnt <= 3'd7;
                    end else begin
                        // Read direction — begin transmitting
                        tx_byte   <= result_reg[15:8];
                        tx_phase  <= 1'b0;
                        state     <= I2C_DATA_TX;
                        bit_cnt   <= 3'd7;
                        sda_drive <= ~tx_byte[7];
                    end
                end

                I2C_CMD_RX: begin
                    shift <= {shift[6:0], sda_in};
                    if (bit_cnt == 3'd0) begin
                        cmd_byte <= {shift[6:0], sda_in};
                        bit_cnt  <= 3'd7;
                        state    <= I2C_CMD_ACK;
                        sda_drive <= 1'b1;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end

                I2C_CMD_ACK: begin
                    sda_drive    <= 1'b0;
                    data_byte_idx <= 0;
                    // RDATA (0x10): next SCL neg → begin read transaction path
                    // WREG (0x4x): receive data byte
                    // RREG (0x2x): receive register index then enter TX
                    if (cmd_byte == 8'h10) begin
                        // RDATA — host will send repeated-start + read address
                        state <= I2C_IDLE;
                    end else begin
                        state   <= I2C_DATA_RX;
                        bit_cnt <= 3'd7;
                    end
                end

                I2C_DATA_TX: begin
                    if (bit_cnt == 3'd0) begin
                        bit_cnt   <= 3'd7;
                        state     <= I2C_DATA_ACK;
                        sda_drive <= 1'b0;
                    end else begin
                        bit_cnt   <= bit_cnt - 3'd1;
                        sda_drive <= ~tx_byte[bit_cnt - 1];
                    end
                end

                I2C_DATA_ACK: begin
                    // Master ACK → continue; NACK → stop
                    if (!sda_in) begin
                        if (!tx_phase) begin
                            tx_byte  <= result_reg[7:0];
                            tx_phase <= 1'b1;
                            state    <= I2C_DATA_TX;
                            bit_cnt  <= 3'd7;
                            sda_drive <= ~result_reg[7];
                        end else begin
                            state <= I2C_IDLE;
                        end
                    end else begin
                        state <= I2C_IDLE;
                    end
                end

                I2C_DATA_RX: begin
                    shift <= {shift[6:0], sda_in};
                    if (bit_cnt == 3'd0) begin
                        // WREG: store to config reg
                        if (cmd_byte[7:4] == 4'h4)
                            cfg_reg[cmd_byte[1:0]] <= {shift[6:0], sda_in};
                        bit_cnt   <= 3'd7;
                        state     <= I2C_DATA_WACK;
                        sda_drive <= 1'b1;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end

                I2C_DATA_WACK: begin
                    sda_drive <= 1'b0;
                    state     <= I2C_IDLE;
                end

                I2C_START_SKIP: begin
                    // Phantom SCL negedge from i2c_start() — discard it.
                    state <= I2C_ADDR_RX;
                end

                default: state <= I2C_IDLE;
            endcase
        end
    end

    // DRDY default high
    initial begin
        drdy_n = 1'b1;
    end

    // =========================================================================
    // Task API
    // =========================================================================

    // Inject a 16-bit ADC result and pulse DRDY_N low for ~1 ms simulated
    task automatic inject_adc_result(input logic [15:0] val);
        result_reg = val;
        drdy_n     = 1'b0;
        #1000;   // 1 μs DRDY pulse (Tail FPGA drdy_monitor detects falling edge)
        drdy_n = 1'b1;
    endtask

endmodule
