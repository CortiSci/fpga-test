// UCSD Recording ASIC Behavioral Model
// 1024 ADCs organised as 64 slices of 16 ADCs each.
// 16 SD lines (SD0..SD15); each SD line carries 4 slices = 64 ADC values per frame.
// Slice-to-SD mapping: slice s goes to SD lane (s % 16).
// Within each SD line, output order = slices 0, 16, 32, 48 in sequence,
// each slice outputs its 16 ADC values MSB-first.
//
// At lane k, sample time s (0..SAMPLES_PER_FRAME-1):
//   local_slice = (s / 16) * 16 + k   -- which slice
//   local_adc   = s % 16              -- which ADC within that slice
//   pattern_mem index = local_slice * 16 + local_adc = k*16 + (s/16)*256 + s%16
//   Simplified: pattern_mem[k + (s%16)*1 + (s/16)*256]  ... see gen_samples for exact
//
// SD_CLK (ro1_clk) is generated internally at mclk/8 = 2.56 MHz.
// The Tail FPGA receives ro1_clk as a source-synchronous input.
// ro1_frame is asserted high for exactly 1 SD_CLK cycle at the start of each frame.
`timescale 1ns/1ps

module ucsd_asic_model
    import spi_tasks_pkg::asic_mode_t,
           spi_tasks_pkg::MODE_RAMP,
           spi_tasks_pkg::MODE_CONSTANT,
           spi_tasks_pkg::MODE_SINE,
           spi_tasks_pkg::MODE_PATTERN_FILE,
           spi_tasks_pkg::MODE_UNIQUE,
           spi_tasks_pkg::MODE_IDLE;
#(
    // Samples per frame = ADCs per SD line = 64 for 2.5 kHz at 2.56 MHz SD_CLK.
    // 256 → legacy behaviour retained for any existing tests that set it explicitly.
    parameter int SAMPLES_PER_FRAME = 256,
    // MCLK divide ratio (log2): 3 = ÷8 (2.56 MHz default), 4 = ÷16 (1.28 MHz)
    parameter int MCLK_DIVLOG2 = 3,
    // Cycles of mclk to wait after mclk_en before asserting the first frame_valid.
    // Default 0 = no delay (preserves behaviour for all existing tests).
    // Set to 1000 in tb_functional so the host has time to arm buf_ena before
    // the first frame boundary; 1000 × 48.8 ns ≈ 49 µs at 20.48 MHz.
    parameter int STARTUP_DELAY_MCLK = 0,
    // Board leg identity used only by MODE_UNIQUE simulation stimulus.
    parameter logic [1:0] LEG_ID = 2'd0
) (
    // Master clock input from Tail FPGA (20.48 MHz).
    // ASIC divides by 8 to produce SD_CLK = 2.56 MHz.
    input  wire        mclk,
    input  wire        mclk_en,     // enable SD_CLK and data output (from Tail FPGA ctrl reg)
    input  wire        ro_rst_n,    // active-low reset from Tail FPGA
    output wire        ro1_clk,     // SD_CLK output to Tail FPGA (mclk/8)
    output reg  [15:0] ro1_sd,      // 16-lane serial data, MSB first
    output reg         ro1_frame,   // high for exactly 1 SD_CLK at frame start

    // SPI configuration slave (Mode 0, two chip-selects)
    input  wire        spi_sclk,
    input  wire        spi_mosi,
    output reg         spi_miso,
    input  wire        spi_ss0_n,
    input  wire        spi_ss1_n
);

    // -------------------------------------------------------------------------
    // SD_CLK generation: mclk / 8 = 2.56 MHz
    // mclk_div[2] toggles every 4 mclk cycles → full period = 8 mclk cycles.
    // Gate with mclk_en so the ASIC stays silent until the Tail FPGA enables it.
    // -------------------------------------------------------------------------
    reg [MCLK_DIVLOG2-1:0] mclk_div;
    wire                   sd_clk_int;

    always @(posedge mclk or negedge ro_rst_n) begin
        if (!ro_rst_n) mclk_div <= '0;
        else           mclk_div <= mclk_div + 1;
    end

    assign sd_clk_int = mclk_en ? mclk_div[MCLK_DIVLOG2-1] : 1'b0;
    assign ro1_clk    = sd_clk_int;

    always @(mclk_en) begin
`ifdef VERBOSE
        $display("[ASIC_MDL %0t ns] mclk_en -> %b (sd_clk_int will %s)",
                 $time, mclk_en, mclk_en ? "RUN" : "STOP");
`endif
    end

    // -------------------------------------------------------------------------
    // Data generation mode (type imported from spi_tasks_pkg)
    // -------------------------------------------------------------------------
    asic_mode_t data_mode = MODE_IDLE;
    reg [15:0]  constant_val = 16'h0;
    reg [15:0]  pattern_mem [0:4095];
    int         pattern_len  = 0;
    int         pattern_idx  = 0;   // kept for API compatibility (not used in output path)

    // -------------------------------------------------------------------------
    // Startup delay: suppress frame_valid for STARTUP_DELAY_MCLK mclk cycles
    // after mclk_en asserts. Counts in the mclk domain; resets to zero whenever
    // mclk_en is low so the delay repeats on every MCLK_EN re-assertion.
    // When STARTUP_DELAY_MCLK==0 startup_done is permanently 1 (no delay).
    // -------------------------------------------------------------------------
    reg [9:0]  startup_cnt;
    reg        startup_done;

    always @(posedge mclk or negedge ro_rst_n) begin
        if (!ro_rst_n) begin
            startup_cnt  <= 10'd0;
            startup_done <= (STARTUP_DELAY_MCLK == 0);
        end else if (!mclk_en) begin
            startup_cnt  <= 10'd0;
            startup_done <= (STARTUP_DELAY_MCLK == 0);
        end else if (!startup_done) begin
            if (startup_cnt == STARTUP_DELAY_MCLK - 1)
                startup_done <= 1'b1;
            else
                startup_cnt <= startup_cnt + 10'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 16 shift registers — one per lane, 16-bit deep
    // -------------------------------------------------------------------------
    reg [15:0]  sr [0:15];
    reg [15:0]  next_sample [0:15];
    int         bit_cnt    = 0;
    int         sample_cnt = 0;
    int         frame_cnt  = 0;
    reg         frame_start_d = 1'b0;  // posedge captures; negedge outputs ro1_frame

    // Self-inverse unique-data transform.  The pre-encoder sample is
    // {frame_cnt[3:0], sensor[9:0], LEG_ID[1:0]}; XOR with this deterministic
    // 16-step LFSR mask preserves a simple, independently reproducible oracle.
    function automatic [15:0] unique_lfsr_mask(input [15:0] seed);
        reg [15:0] lfsr;
        integer b;
        begin
            lfsr = seed ^ 16'h1D0F;
            for (b = 0; b < 16; b = b + 1)
                lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            unique_lfsr_mask = lfsr;
        end
    endfunction

    // -------------------------------------------------------------------------
    // SINE table (12-bit, 256 entries)
    // -------------------------------------------------------------------------
    reg [11:0] sine_lut [0:255];
    integer si;
    initial begin
        for (si = 0; si < 256; si = si + 1)
            sine_lut[si] = $rtoi(2047.5 + 2047.5 * $sin(2.0 * 3.14159265 * si / 256.0));
    end

    // -------------------------------------------------------------------------
    // Data generation
    //
    // Blocked layout: each SD lane k carries SAMPLES_PER_FRAME ADC values.
    // The 64 ADC slices (16 ADCs each) are distributed round-robin across the
    // 16 SD lanes.  At sample time s within a frame:
    //   slice_group = s / 16          (which group of 16 slices, 0..3 for SPF=64)
    //   local_adc   = s % 16          (which ADC within the slice)
    //   pattern_mem_index for lane k  = k*16 + local_adc + slice_group*256
    //
    // Equivalently:
    //   index = k*16 + (s % 16) + (s / 16)*256
    // -------------------------------------------------------------------------
    task automatic gen_samples();
        automatic int lane;
        automatic int local_s  = sample_cnt % SAMPLES_PER_FRAME;
        automatic int grp      = local_s / 16;   // slice group (0..3 for SPF=64)
        automatic int ladc     = local_s % 16;   // ADC within slice
        case (data_mode)
            MODE_RAMP: begin
                // Ramp: value = local ADC index within the ASIC (0..1023)
                for (lane = 0; lane < 16; lane = lane + 1)
                    next_sample[lane] = 16'(lane * 16 + ladc + grp * 256);
            end
            MODE_CONSTANT: begin
                for (lane = 0; lane < 16; lane = lane + 1)
                    next_sample[lane] = constant_val;
            end
            MODE_SINE: begin
                for (lane = 0; lane < 16; lane = lane + 1)
                    next_sample[lane] = {4'h0, sine_lut[(local_s + lane * 4) % 256]};
            end
            MODE_PATTERN_FILE: begin
                for (lane = 0; lane < 16; lane = lane + 1) begin
                    if (pattern_len > 0) begin
                        automatic int pidx = lane * 16 + ladc + grp * 256;
                        next_sample[lane] = pattern_mem[pidx % pattern_len];
                    end else
                        next_sample[lane] = 16'h0;
                end
            end
            MODE_UNIQUE: begin
                for (lane = 0; lane < 16; lane = lane + 1) begin
                    automatic logic [9:0] sensor = lane * 16 + ladc + grp * 256;
                    automatic logic [15:0] pre = {frame_cnt[3:0], sensor, LEG_ID};
                    next_sample[lane] = pre ^ unique_lfsr_mask(pre);
                end
            end
            default: begin  // IDLE
                for (lane = 0; lane < 16; lane = lane + 1)
                    next_sample[lane] = 16'h0;
            end
        endcase
    endtask

    // -------------------------------------------------------------------------
    // Bit-serial output engine — clocked on SD_CLK (sd_clk_int = mclk/8)
    //
    // Split into posedge (state) + negedge (output) to avoid a simulation
    // race: both this model and asic_readout_if sample on posedge sd_clk_int.
    // Driving ro1_sd/ro1_frame via NBA on posedge means asic_readout_if reads
    // the pre-NBA (stale) value in the same active region.  Driving on the
    // NEGEDGE guarantees the value is stable 4 sys_clk cycles before the next
    // posedge where asic_readout_if captures it.
    // -------------------------------------------------------------------------
    integer lane_i;

    // Posedge: update shift registers and counters only.
    always @(posedge sd_clk_int or negedge ro_rst_n) begin
        if (!ro_rst_n) begin
            bit_cnt       <= 0;
            sample_cnt    <= 0;
            frame_cnt     <= 0;
            frame_start_d <= 1'b0;
            for (lane_i = 0; lane_i < 16; lane_i = lane_i + 1)
                sr[lane_i] <= 16'h0;
        end else begin
            if (bit_cnt == 0) begin
                gen_samples();
                for (lane_i = 0; lane_i < 16; lane_i = lane_i + 1)
                    sr[lane_i] <= next_sample[lane_i];
            end else begin
                for (lane_i = 0; lane_i < 16; lane_i = lane_i + 1)
                    sr[lane_i] <= {sr[lane_i][14:0], 1'b0};
            end
            // Capture whether this posedge begins a new ASIC frame so the
            // negedge output block can assert ro1_frame without reading
            // bit_cnt/sample_cnt across clock edges.
            frame_start_d <= startup_done &&
                             (bit_cnt == 0) && (sample_cnt % SAMPLES_PER_FRAME == 0);
            if (bit_cnt == 15) begin
                bit_cnt    <= 0;
                sample_cnt <= sample_cnt + 1;
                if ((sample_cnt + 1) % SAMPLES_PER_FRAME == 0)
                    frame_cnt <= frame_cnt + 1;
            end else begin
                bit_cnt <= bit_cnt + 1;
            end
        end
    end

    // Negedge: drive serial outputs from post-posedge-NBA shift-register state.
    // asic_readout_if captures on the following posedge — values are stable.
    always @(negedge sd_clk_int or negedge ro_rst_n) begin
        if (!ro_rst_n) begin
            ro1_sd    <= 16'h0;
            ro1_frame <= 1'b0;
        end else begin
            for (lane_i = 0; lane_i < 16; lane_i = lane_i + 1)
                ro1_sd[lane_i] <= sr[lane_i][15];
            ro1_frame <= frame_start_d;
        end
    end

    // -------------------------------------------------------------------------
    // SPI configuration slave (Mode 0, 8-bit) — unchanged
    // -------------------------------------------------------------------------
    reg [7:0]  asic_regs [0:15];
    reg [7:0]  spi_shift;
    reg [2:0]  spi_bit_cnt;
    reg [7:0]  spi_tx_byte;
    reg        spi_first_byte;
    reg [7:0]  spi_cmd_reg;

    initial begin
        for (si = 0; si < 16; si = si + 1) asic_regs[si] = 8'h0;
        spi_miso     = 1'b1;
        spi_bit_cnt  = 3'd0;
        spi_first_byte = 1'b1;
    end

    // Add negedge ro_rst_n so the SPI shift state resets when the ASIC is held in
    // reset — an ASIC in reset must not respond to SPI transactions.
    always @(posedge spi_sclk or negedge ro_rst_n) begin
        if (!ro_rst_n) begin
            spi_shift      <= 8'h0;
            spi_bit_cnt    <= 3'd0;
            spi_first_byte <= 1'b1;
            spi_tx_byte    <= 8'h0;
        end else if (!spi_ss0_n || !spi_ss1_n) begin
            spi_shift <= {spi_shift[6:0], spi_mosi};
            if (spi_bit_cnt == 3'd7) begin
                spi_bit_cnt <= 3'd0;
                if (spi_first_byte) begin
`ifdef VERBOSE
                    $display("[ASIC_MODEL %0t ns] cmd_byte=0x%02h", $time, {spi_shift[6:0], spi_mosi});
`endif
                    spi_cmd_reg  <= {spi_shift[6:0], spi_mosi};
                    spi_tx_byte  <= asic_regs[{spi_shift[3:0], spi_mosi} % 16];
                    spi_first_byte <= 1'b0;
                end else begin
`ifdef VERBOSE
                    $display("[ASIC_MODEL %0t ns] write reg[%0d]=0x%02h (cmd=0x%02h)",
                             $time, spi_cmd_reg[3:0], {spi_shift[6:0], spi_mosi}, spi_cmd_reg);
`endif
                    asic_regs[spi_cmd_reg[3:0]] <= {spi_shift[6:0], spi_mosi};
                end
            end else begin
                spi_bit_cnt <= spi_bit_cnt + 3'd1;
            end
        end
    end

    always @(negedge spi_sclk or negedge ro_rst_n) begin
        if (!ro_rst_n)
            spi_miso <= 1'b1;   // tri-state equivalent: release MISO when ASIC in reset
        else if (!spi_ss0_n || !spi_ss1_n)
            spi_miso <= spi_tx_byte[7 - spi_bit_cnt];
        else
            spi_miso <= 1'b1;
    end

    always @(posedge spi_ss0_n, posedge spi_ss1_n) begin
        spi_first_byte <= 1'b1;
        spi_bit_cnt    <= 3'd0;
        spi_miso       <= 1'b1;
    end

    // Mode 0 (CPHA=0): slave must pre-drive MISO with MSB of response before the first
    // rising edge.  Without this, the exec FSM samples spi_miso=1 (idle-high pullup)
    // on the first rising edge, corrupting bit 7 of every received first byte.
    always @(negedge spi_ss0_n, negedge spi_ss1_n) begin
        spi_miso <= spi_tx_byte[7];
    end

    // =========================================================================
    // Task API
    // =========================================================================

    task automatic set_data_mode(
        input asic_mode_t mode,
        input logic [15:0] c_val = 16'h0
    );
        data_mode    = mode;
        constant_val = c_val;
        pattern_idx  = 0;
    endtask

    task automatic load_pattern_file(input string filename);
        $readmemh(filename, pattern_mem);
        pattern_len = 0;
        begin : count_lp
            automatic int pi;
            for (pi = 0; pi < 4096; pi = pi + 1) begin
                if (^pattern_mem[pi] !== 1'bx) pattern_len = pi + 1;
                else disable count_lp;
            end
        end
        pattern_idx = 0;
    endtask

    task automatic set_asic_register(
        input logic [7:0] addr,
        input logic [7:0] val
    );
        asic_regs[addr[3:0]] = val;
    endtask

    task automatic read_asic_register(
        input  logic [7:0] addr,
        output logic [7:0] val
    );
        val = asic_regs[addr[3:0]];
    endtask

    task automatic wait_n_frames(input int n);
        automatic int target = frame_cnt + n;
        while (frame_cnt < target) @(posedge sd_clk_int);
    endtask

endmodule
