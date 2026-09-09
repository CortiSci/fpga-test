// Application-level HAL: reassembles Consolidator telemetry frames into a
// sensor-ordered data buffer and compares against an expected scoreboard.
//
// ASIC physical layout (blocked):
//   1024 ADCs per ASIC arranged as 64 slices of 16 ADCs each.
//   The 64 slices are distributed across 16 SD lanes in round-robin:
//     slice s  → SD lane (s % 16)
//   Output order on each SD lane per frame:
//     slices 0,16,32,48 (for 16 lanes, SPF=64): 4 groups × 16 ADCs = 64 values
//   At sample time s (0..63) on lane k:
//     slice_group = s / 16   (0..3)
//     local_adc   = s % 16   (0..15)
//     pattern_mem index = k*16 + local_adc + slice_group*256
//     global sensor index (within ASIC) = k*16 + local_adc + slice_group*256
//                                       = k*16 + (s%16) + (s/16)*256
//
// Tail FPGA FIFO push order: lanes sr[0..15] after each 16-bit sample.
//   FIFO word at position p = 16*s + k carries ADC at (k*16 + (s%16) + (s/16)*256)
//
// Consolidator frame f (0..127), round r (0..7):
//   FIFO position p = f*8 + r  →  s = p/16, k = p%16
//   global sensor index = k*16 + (s%16) + (s/16)*256
//                       = (p%16)*16 + ((p/16)%16) + (p/256)*256
//
// 128 Consolidator frames = 1 ASIC frame (1024 FIFO words per channel).
`timescale 1ns/1ps

package frame_assembler_pkg;

    localparam int CON_FRAMES_PER_ASIC = 128;   // 1024 CDC words / 8 per con-frame
    localparam int SENSORS_TOTAL       = 4096;
    localparam int CHANNELS            = 4;
    localparam int ROUNDS              = 8;
    localparam int LANES               = 16;
    localparam int SAMPLES_PER_LANE    = 64;    // = CON_FRAMES_PER_ASIC * ROUNDS / LANES

    // -------------------------------------------------------------------------
    // assemble_asic_frame
    // Input : 128 Consolidator telemetry frames, each 37 words
    //         con_frames[f][0]     = header
    //         con_frames[f][1..32] = payload (8 rounds × 4 channels)
    //         con_frames[f][33..36]= token + CRC (not examined here)
    // Output: sensor_data[0:4095] in global-sensor-index order
    //         asic_frame_number derived from header of first frame
    // -------------------------------------------------------------------------
    function automatic void assemble_asic_frame(
        input  logic [15:0] con_frames[0:CON_FRAMES_PER_ASIC-1][0:36],
        output logic [15:0] sensor_data[0:SENSORS_TOTAL-1],
        output int          asic_frame_number
    );
        automatic int f, ch, round, pos, lane, samp, sg, la, sensor_idx;
        // asic_frame_number: frame_cnt field in header bits [15:3], divided by 128
        asic_frame_number = int'(con_frames[0][0] >> 3) / CON_FRAMES_PER_ASIC;

        for (f = 0; f < CON_FRAMES_PER_ASIC; f++) begin
            for (round = 0; round < ROUNDS; round++) begin
                for (ch = 0; ch < CHANNELS; ch++) begin
                    // FIFO position consumed by this payload word
                    pos   = f * ROUNDS + round;     // 0..1023
                    lane  = pos % LANES;             // SD lane (0..15)
                    samp  = pos / LANES;             // sample time (0..63)
                    sg    = samp / LANES;            // slice group (0..3)
                    la    = samp % LANES;            // local ADC within slice (0..15)
                    // Blocked geometry: global sensor index within this ASIC
                    sensor_idx = ch * 1024 + lane * 16 + la + sg * 256;
                    sensor_data[sensor_idx] = con_frames[f][1 + 4*round + ch];
                end
            end
        end
    endfunction

    // -------------------------------------------------------------------------
    // compare_sensor_frames
    // Returns number of mismatches; logs each (first 16 capped).
    // -------------------------------------------------------------------------
    function automatic int compare_sensor_frames(
        input logic [15:0] received[0:SENSORS_TOTAL-1],
        input logic [15:0] expected[0:SENSORS_TOTAL-1],
        input string       test_name,
        input int          asic_frame_number
    );
        automatic int mismatches = 0;
        automatic int i;
        for (i = 0; i < SENSORS_TOTAL; i++) begin
            if (received[i] !== expected[i]) begin
                mismatches++;
                if (mismatches <= 16)
                    $error("[%s] ASIC frame %0d sensor[%0d]: got 0x%04h expected 0x%04h",
                           test_name, asic_frame_number, i, received[i], expected[i]);
            end
        end
        if (mismatches == 0) begin
`ifdef VERBOSE
            $display("[%s] ASIC frame %0d: all %0d sensors MATCH",
                     test_name, asic_frame_number, SENSORS_TOTAL);
`endif
        end
        else
            $error("[%s] ASIC frame %0d: %0d mismatch(es) (first 16 shown above)",
                   test_name, asic_frame_number, mismatches);
        return mismatches;
    endfunction

    // -------------------------------------------------------------------------
    // build_count_scoreboard
    // For count frames: pattern_mem[i] = i (per ASIC), and with blocked geometry
    // sensor_index = ASIC_offset + i, so expected[sensor_index] = sensor_index.
    // -------------------------------------------------------------------------
    function automatic void build_count_scoreboard(
        output logic [15:0] expected[0:SENSORS_TOTAL-1]
    );
        automatic int i;
        for (i = 0; i < SENSORS_TOTAL; i++)
            expected[i] = i[15:0];
    endfunction

    // -------------------------------------------------------------------------
    // check_frame_count_monotonic
    // Returns 1 if a gap or duplicate is detected in the 13-bit frame_cnt field
    // (header bits [15:3]).
    // -------------------------------------------------------------------------
    function automatic int check_frame_count_monotonic(
        input logic [15:0] prev_hdr,
        input logic [15:0] curr_hdr,
        input string       test_name
    );
        automatic int prev_fc = int'(prev_hdr >> 3);
        automatic int curr_fc = int'(curr_hdr >> 3);
        automatic int expected_fc = (prev_fc + 1) & 13'h1FFF;
        if (curr_fc !== expected_fc) begin
            $error("[%s] frame_cnt gap: prev=%0d, got=%0d (expected %0d)",
                   test_name, prev_fc, curr_fc, expected_fc);
            return 1;
        end
        return 0;
    endfunction

endpackage
