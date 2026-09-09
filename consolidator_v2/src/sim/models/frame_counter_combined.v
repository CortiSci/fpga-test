// Simulation-only combined frame_counter stub.
//
// The V2 consolidator and the production Tail FPGA both instantiate a module
// named "frame_counter", but with completely different port lists.  Compiling
// both real implementations into the same work library causes a name collision.
//
// This stub declares ALL ports from both versions (names are disjoint — no
// overlap) and implements both behaviours in separate always blocks.  Each
// block is gated by its own clock/reset pair:
//
//   V2 consolidator ports: sclk, rst_n, cnt_rst, frame_cnt[15:0], tick_2k5
//   Tail FPGA ports:       sys_clk, sys_rst_n, frame_valid_pulse,
//                          frame_num[31:0], frame_num_tog, free_cnt_o[31:0]
//
// When the V2 consolidator instantiates this module with named connections,
// the tail ports are unconnected (driven to Z).  A Z input has no posedge/
// negedge events, so the tail always block stays dormant.  The same applies
// in reverse for the tail's instantiation.
//
// COMPILE INSTEAD OF:
//   Tail FPGA/src/rtl/frame_counter.v
//   consolidator_v2/src/rtl/frame_counter.v
// (see test_spi_ping_prod_tail.do)
`timescale 1ns/1ps

module frame_counter (
    // ---- V2 consolidator ports (51.2 MHz free-running, 2.5 kHz tick) --------
    input  wire        sclk,
    input  wire        rst_n,
    input  wire        cnt_rst,
    output reg  [15:0] frame_cnt,
    output reg         tick_2k5,

    // ---- Production Tail FPGA ports (20.48 MHz ASIC frame counter) ----------
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        frame_valid_pulse,
    output reg  [31:0] frame_num,
    output reg         frame_num_tog,
    output wire [31:0] free_cnt_o
);

    // =========================================================================
    // V2 consolidator behaviour — 51.2 MHz / 20480 = 2.5 kHz tick
    // =========================================================================
    localparam [14:0] V2_TICK_PERIOD = 15'd20479;
    reg [14:0] v2_tick_cnt;

    always @(posedge sclk or negedge rst_n) begin
        if (!rst_n) begin
            v2_tick_cnt <= V2_TICK_PERIOD;
            tick_2k5    <= 1'b0;
            frame_cnt   <= 16'd0;
        end else begin
            tick_2k5 <= 1'b0;
            if (cnt_rst) begin
                frame_cnt   <= 16'd0;
                v2_tick_cnt <= V2_TICK_PERIOD;
            end else if (v2_tick_cnt == 15'd0) begin
                v2_tick_cnt <= V2_TICK_PERIOD;
                tick_2k5    <= 1'b1;
                frame_cnt   <= frame_cnt + 16'd1;
            end else begin
                v2_tick_cnt <= v2_tick_cnt - 15'd1;
            end
        end
    end

    // =========================================================================
    // Production Tail FPGA behaviour — 20.48 MHz / 8192 = 2.5 kHz divider.
    // Simplified: no stall logic (not needed for SPI ping regression).
    // =========================================================================
    reg [12:0] t_div_cnt;
    reg [31:0] t_free_cnt;
    assign free_cnt_o = t_free_cnt;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            t_div_cnt     <= 13'd0;
            t_free_cnt    <= 32'd0;
            frame_num     <= 32'd0;
            frame_num_tog <= 1'b0;
        end else begin
            if (&t_div_cnt) begin
                t_div_cnt  <= 13'd0;
                t_free_cnt <= t_free_cnt + 32'd1;
            end else begin
                t_div_cnt <= t_div_cnt + 13'd1;
            end
            if (frame_valid_pulse) begin
                frame_num     <= t_free_cnt;
                frame_num_tog <= ~frame_num_tog;
            end
        end
    end

endmodule
