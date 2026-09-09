`timescale 1ns / 1ps
// End-to-end harness for the V3 telemetry engine (main-branch redesign):
//
//   4x asic_stream_tx (tail, bit-plane raw words, frame-sync gate)
//     -> 4x spi_ch_stream (lane reassembly: assembled sensor samples)
//     -> leg_quad_fifo -> telem_engine_v3 -> crc32
//
// Wired the way §11 of telemetry_v3_spec.md prescribes for the top level.
// The C++ TB drives the board's clock relationships and checks the emitted
// 4103-word frames against THE SPEC, not against the RTL.
module v3_e2e_wrap (
    input  wire        clk_48m,
    input  wire [3:0]  spi_sclk,     // per-leg tail SCLK (TB gates on strm_active)
    input  wire        ro1_clk,      // shared ASIC readout clock
    input  wire        ro1_frame,    // shared sweep-start strobe
    input  wire        rst_n,
    input  wire [3:0]  run,          // consolidator ch_run & spi_enable
    input  wire [3:0]  telem_en,     // per-tail streaming enable
    input  wire [63:0] ro1_sd,       // per-tail 16-bit raw bus (packed)
    input  wire        telem_start,  // one-cycle strobe (spec §9.3)
    input  wire        tx_ready,

    output wire [3:0]  strm_active,
    output wire [15:0] tx_data,
    output wire        tx_valid,
    output wire        framer_busy,
    output wire [3:0]  quad_ovf,
    output wire [3:0]  par_err
);
    wire [3:0] mosi, ss_n;
    wire [3:0] miso_pre;
    reg  [3:0] miso_ff;

    wire [15:0] w_data [0:3];
    wire [3:0]  w_valid;

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : legs
            asic_stream_tx u_tx (
                .ro1_clk         (ro1_clk),
                .rst_n           (rst_n),
                .ro1_sd          (ro1_sd[g*16 +: 16]),
                .ro1_frame       (ro1_frame),
                .sclk            (spi_sclk[g]),
                .telem_en        (telem_en[g]),
                .imp_en          (1'b0),
                .lane_sel        (1'b0),
                .mosi            (mosi[g]),
                .ss_n            (ss_n[g]),
                .stream_miso_pre (miso_pre[g])
            );

            always @(posedge spi_sclk[g] or negedge rst_n) begin
                if (!rst_n) miso_ff[g] <= 1'b0;
                else        miso_ff[g] <= miso_pre[g];
            end

            spi_ch_stream u_rx (
                .sclk        (clk_48m),
                .rst_n       (rst_n),
                .miso_in     (miso_ff[g]),
                .run         (run[g]),
                .cfg_hold    (1'b0),
                .sclk_d1     (strm_active[g]),
                .mosi_out    (mosi[g]),
                .ss_n_out    (ss_n[g]),
                .word_data   (w_data[g]),
                .word_valid  (w_valid[g]),
                .par_err_flag(par_err[g]),
                .resync_arm  (wrap_resync_arm),
                .fifo_full   (wrap_full[g]),
                .quad_flush  (wrap_flush[g]),
                .w0_dist     (wrap_hp[g*10 +: 10]),
                .phase_known (wrap_pk[g])
            );
        end
    endgenerate

    // ---- V3 data path ------------------------------------------------------
    wire        tick_req, tick_valid;
    wire [15:0] tick_d0, tick_d1, tick_d2, tick_d3;
    wire [3:0]  tick_undf, fill_ge4;
    wire [3:0]  fifo_full_w, fifo_empty_w;
    wire        wrap_resync_arm;
    wire [3:0]  wrap_flush;
    wire [39:0] wrap_hp;
    wire [3:0]  wrap_pk;
    wire [19:0] wrap_wcnt = fifo_wc_w;
    wire [3:0]  wrap_full = fifo_full_w;
    wire [19:0] fifo_wc_w;

    leg_quad_fifo quad (
        .clk          (clk_48m),
        .rst_n        (rst_n),
        .sync_rst     (telem_start),
        .wd_data_0    (w_data[0]),
        .wd_data_1    (w_data[1]),
        .wd_data_2    (w_data[2]),
        .wd_data_3    (w_data[3]),
        .wd_valid     (w_valid),
        .tick_req     (tick_req),
        .tick_data_0  (tick_d0),
        .tick_data_1  (tick_d1),
        .tick_data_2  (tick_d2),
        .tick_data_3  (tick_d3),
        .tick_valid   (tick_valid),
        .tick_undf    (tick_undf),
        .fill_ge4     (fill_ge4),
        .fifo_full    (fifo_full_w),
        .fifo_empty   (fifo_empty_w),
        .fifo_ovf     (quad_ovf),
        .fifo_word_cnt(fifo_wc_w),
        .ch_local_rst (wrap_flush)
    );

    wire        crc_init_w, crc_valid_w, crc_last_w;
    wire [15:0] crc_data_w;
    wire [31:0] crc_result_w;
    wire        crc_result_valid_w;

    crc32 crc (
        .clk       (clk_48m),
        .rst_n     (rst_n),
        .init      (crc_init_w),
        .valid     (crc_valid_w),
        .last      (crc_last_w),
        .data_in   (crc_data_w),
        .crc_out   (crc_result_w),
        .crc_valid (crc_result_valid_w)
    );

    // Free-running 2.5 kHz frame counter, as frame_counter.v provides.
    // Match frame_counter.v's complete 29-bit external-counter contract,
    // even though only bits [12:0] are sent in the telemetry header.
    reg [28:0] frame_cnt;
    reg [14:0] fc_div;
    always @(posedge clk_48m or negedge rst_n) begin
        if (!rst_n) begin
            frame_cnt <= 29'd0;
            fc_div    <= 15'd0;
        end else if (fc_div == 15'd20479) begin
            fc_div    <= 15'd0;
            frame_cnt <= frame_cnt + 29'd1;
        end else begin
            fc_div <= fc_div + 15'd1;
        end
    end

    telem_engine_v3 engine (
        .clk             (clk_48m),
        .rst_n           (rst_n),
        .telem_start     (telem_start),
        .sw_reset        (1'b0),
        .run_any         (|run),
        .frame_aligned   (1'b1),
        .ext_frame_cnt   (frame_cnt),
        .tick_req        (tick_req),
        .tick_data_0     (tick_d0),
        .tick_data_1     (tick_d1),
        .tick_data_2     (tick_d2),
        .tick_data_3     (tick_d3),
        .tick_valid      (tick_valid),
        .tick_undf       (tick_undf),
        .fill_ge4        (fill_ge4),
        .par_err_in      (par_err),
        .ovfl_in         (quad_ovf),
        .crc_init        (crc_init_w),
        .crc_valid       (crc_valid_w),
        .crc_last        (crc_last_w),
        .crc_data        (crc_data_w),
        .crc_result      (crc_result_w),
        .crc_result_valid(crc_result_valid_w),
        .telem_tx_data   (tx_data),
        .telem_tx_valid  (tx_valid),
        .telem_tx_ready  (tx_ready),
        .framer_busy     (framer_busy),
        .resync_arm      (wrap_resync_arm),
        .w0_dist_0       (wrap_hp[9:0]),
        .w0_dist_1       (wrap_hp[19:10]),
        .w0_dist_2       (wrap_hp[29:20]),
        .w0_dist_3       (wrap_hp[39:30]),
        .leg_fill        (wrap_wcnt),
        .phase_known     (wrap_pk)
    );
endmodule
