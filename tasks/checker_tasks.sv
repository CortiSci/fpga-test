// Checker tasks: software reference models for CRC-32, 3DES, and frame validation.
// Included as a package imported into tb_top.sv and individual test files.
`timescale 1ns/1ps

package checker_pkg;

    // =========================================================================
    // CRC-32 reference (IEEE 802.3, reflected polynomial 0xEDB88320)
    // Operates on an array of 16-bit words, processed high-byte first (big-endian).
    // Matches telemetry_framer.v which byte-swaps crc_data before feeding crc32.v,
    // and usb_acq_pipeline.cpp frame_crc() which packs big-endian (word>>8 first).
    // =========================================================================
    // C-05: dynamic/unpacked array params not supported by Icarus — guarded.
`ifndef ICARUS
    function automatic logic [31:0] ref_crc32(
        input logic [15:0] words [],
        input int          n
    );
        automatic logic [31:0] crc = 32'hFFFF_FFFF;
        automatic logic [31:0] poly = 32'hEDB8_8320;
        automatic int w, b, bit_i;
        automatic logic [7:0] byte_val;
        automatic logic       xor_bit;

        for (w = 0; w < n; w++) begin
            // Process high byte then low byte (big-endian word order)
            for (b = 0; b < 2; b++) begin
                byte_val = (b == 0) ? words[w][15:8] : words[w][7:0];
                for (bit_i = 0; bit_i < 8; bit_i++) begin
                    xor_bit = crc[0] ^ byte_val[bit_i];
                    crc     = crc >> 1;
                    if (xor_bit) crc = crc ^ poly;
                end
            end
        end
        return crc ^ 32'hFFFF_FFFF;
    endfunction
`endif

    // =========================================================================
    // 3DES-EDE reference implementation (software, for checker only)
    // Uses the standard DES S-boxes and key schedule.
    // K1 encrypt → K2 decrypt → K3 encrypt.
    // Bit numbering: DES standard (bit 1 = MSB of 64-bit word).
    // =========================================================================

    // C-05: localparam arrays with unpacked dimensions are not supported by Icarus.
    // All 3DES table localparams are guarded — they are only needed by the 3DES
    // functions (also guarded) and are never called during the RUN_ACED test.
`ifndef ICARUS
    // Initial permutation table (0-indexed, value = src bit position in 64-bit block)
    localparam logic [5:0] IP_TBL [0:63] = '{
        6'd57, 6'd49, 6'd41, 6'd33, 6'd25, 6'd17, 6'd9,  6'd1,
        6'd59, 6'd51, 6'd43, 6'd35, 6'd27, 6'd19, 6'd11, 6'd3,
        6'd61, 6'd53, 6'd45, 6'd37, 6'd29, 6'd21, 6'd13, 6'd5,
        6'd63, 6'd55, 6'd47, 6'd39, 6'd31, 6'd23, 6'd15, 6'd7,
        6'd56, 6'd48, 6'd40, 6'd32, 6'd24, 6'd16, 6'd8,  6'd0,
        6'd58, 6'd50, 6'd42, 6'd34, 6'd26, 6'd18, 6'd10, 6'd2,
        6'd60, 6'd52, 6'd44, 6'd36, 6'd28, 6'd20, 6'd12, 6'd4,
        6'd62, 6'd54, 6'd46, 6'd38, 6'd30, 6'd22, 6'd14, 6'd6
    };

    // Final permutation (inverse IP)
    localparam logic [5:0] FP_TBL [0:63] = '{
        6'd39, 6'd7,  6'd47, 6'd15, 6'd55, 6'd23, 6'd63, 6'd31,
        6'd38, 6'd6,  6'd46, 6'd14, 6'd54, 6'd22, 6'd62, 6'd30,
        6'd37, 6'd5,  6'd45, 6'd13, 6'd53, 6'd21, 6'd61, 6'd29,
        6'd36, 6'd4,  6'd44, 6'd12, 6'd52, 6'd20, 6'd60, 6'd28,
        6'd35, 6'd3,  6'd43, 6'd11, 6'd51, 6'd19, 6'd59, 6'd27,
        6'd34, 6'd2,  6'd42, 6'd10, 6'd50, 6'd18, 6'd58, 6'd26,
        6'd33, 6'd1,  6'd41, 6'd9,  6'd49, 6'd17, 6'd57, 6'd25,
        6'd32, 6'd0,  6'd40, 6'd8,  6'd48, 6'd16, 6'd56, 6'd24
    };

    // Expansion permutation E (32→48 bits)
    localparam logic [4:0] E_TBL [0:47] = '{
        5'd31, 5'd0,  5'd1,  5'd2,  5'd3,  5'd4,
        5'd3,  5'd4,  5'd5,  5'd6,  5'd7,  5'd8,
        5'd7,  5'd8,  5'd9,  5'd10, 5'd11, 5'd12,
        5'd11, 5'd12, 5'd13, 5'd14, 5'd15, 5'd16,
        5'd15, 5'd16, 5'd17, 5'd18, 5'd19, 5'd20,
        5'd19, 5'd20, 5'd21, 5'd22, 5'd23, 5'd24,
        5'd23, 5'd24, 5'd25, 5'd26, 5'd27, 5'd28,
        5'd27, 5'd28, 5'd29, 5'd30, 5'd31, 5'd0
    };

    // S-boxes (8 × 64 entries, 4-bit output)
    localparam logic [3:0] SBOX [0:7][0:63] = '{
        // S1
        '{ 4'd14,4'd4, 4'd13,4'd1, 4'd2, 4'd15,4'd11,4'd8,  4'd3, 4'd10,4'd6, 4'd12,4'd5, 4'd9, 4'd0, 4'd7,
           4'd0, 4'd15,4'd7, 4'd4, 4'd14,4'd2, 4'd13,4'd1,  4'd10,4'd6, 4'd12,4'd11,4'd9, 4'd5, 4'd3, 4'd8,
           4'd4, 4'd1, 4'd14,4'd8, 4'd13,4'd6, 4'd2, 4'd11, 4'd15,4'd12,4'd9, 4'd7, 4'd3, 4'd10,4'd5, 4'd0,
           4'd15,4'd12,4'd8, 4'd2, 4'd4, 4'd9, 4'd1, 4'd7,  4'd5, 4'd11,4'd3, 4'd14,4'd10,4'd0, 4'd6, 4'd13 },
        // S2
        '{ 4'd15,4'd1, 4'd8, 4'd14,4'd6, 4'd11,4'd3, 4'd4,  4'd9, 4'd7, 4'd2, 4'd13,4'd12,4'd0, 4'd5, 4'd10,
           4'd3, 4'd13,4'd4, 4'd7, 4'd15,4'd2, 4'd8, 4'd14, 4'd12,4'd0, 4'd1, 4'd10,4'd6, 4'd9, 4'd11,4'd5,
           4'd0, 4'd14,4'd7, 4'd11,4'd10,4'd4, 4'd13,4'd1,  4'd5, 4'd8, 4'd12,4'd6, 4'd9, 4'd3, 4'd2, 4'd15,
           4'd13,4'd8, 4'd10,4'd1, 4'd3, 4'd15,4'd4, 4'd2,  4'd11,4'd6, 4'd7, 4'd12,4'd0, 4'd5, 4'd14,4'd9 },
        // S3
        '{ 4'd10,4'd0, 4'd9, 4'd14,4'd6, 4'd3, 4'd15,4'd5,  4'd1, 4'd13,4'd12,4'd7, 4'd11,4'd4, 4'd2, 4'd8,
           4'd13,4'd7, 4'd0, 4'd9, 4'd3, 4'd4, 4'd6, 4'd10, 4'd2, 4'd8, 4'd5, 4'd14,4'd12,4'd11,4'd15,4'd1,
           4'd13,4'd6, 4'd4, 4'd9, 4'd8, 4'd15,4'd3, 4'd0,  4'd11,4'd1, 4'd2, 4'd12,4'd5, 4'd10,4'd14,4'd7,
           4'd1, 4'd10,4'd13,4'd0, 4'd6, 4'd9, 4'd8, 4'd7,  4'd4, 4'd15,4'd14,4'd3, 4'd11,4'd5, 4'd2, 4'd12 },
        // S4
        '{ 4'd7, 4'd13,4'd14,4'd3, 4'd0, 4'd6, 4'd9, 4'd10, 4'd1, 4'd2, 4'd8, 4'd5, 4'd11,4'd12,4'd4, 4'd15,
           4'd13,4'd8, 4'd11,4'd5, 4'd6, 4'd15,4'd0, 4'd3,  4'd4, 4'd7, 4'd2, 4'd12,4'd1, 4'd10,4'd14,4'd9,
           4'd10,4'd6, 4'd9, 4'd0, 4'd12,4'd11,4'd7, 4'd13, 4'd15,4'd1, 4'd3, 4'd14,4'd5, 4'd2, 4'd8, 4'd4,
           4'd3, 4'd15,4'd0, 4'd6, 4'd10,4'd1, 4'd13,4'd8,  4'd9, 4'd4, 4'd5, 4'd11,4'd12,4'd7, 4'd2, 4'd14 },
        // S5
        '{ 4'd2, 4'd12,4'd4, 4'd1, 4'd7, 4'd10,4'd11,4'd6,  4'd8, 4'd5, 4'd3, 4'd15,4'd13,4'd0, 4'd14,4'd9,
           4'd14,4'd11,4'd2, 4'd12,4'd4, 4'd7, 4'd13,4'd1,  4'd5, 4'd0, 4'd15,4'd10,4'd3, 4'd9, 4'd8, 4'd6,
           4'd4, 4'd2, 4'd1, 4'd11,4'd10,4'd13,4'd7, 4'd8,  4'd15,4'd9, 4'd12,4'd5, 4'd6, 4'd3, 4'd0, 4'd14,
           4'd11,4'd8, 4'd12,4'd7, 4'd1, 4'd14,4'd2, 4'd13, 4'd6, 4'd15,4'd0, 4'd9, 4'd10,4'd4, 4'd5, 4'd3 },
        // S6
        '{ 4'd12,4'd1, 4'd10,4'd15,4'd9, 4'd2, 4'd6, 4'd8,  4'd0, 4'd13,4'd3, 4'd4, 4'd14,4'd7, 4'd5, 4'd11,
           4'd10,4'd15,4'd4, 4'd2, 4'd7, 4'd12,4'd9, 4'd5,  4'd6, 4'd1, 4'd13,4'd14,4'd0, 4'd11,4'd3, 4'd8,
           4'd9, 4'd14,4'd15,4'd5, 4'd2, 4'd8, 4'd12,4'd3,  4'd7, 4'd0, 4'd4, 4'd10,4'd1, 4'd13,4'd11,4'd6,
           4'd4, 4'd3, 4'd2, 4'd12,4'd9, 4'd5, 4'd15,4'd10, 4'd11,4'd14,4'd1, 4'd7, 4'd6, 4'd0, 4'd8, 4'd13 },
        // S7
        '{ 4'd4, 4'd11,4'd2, 4'd14,4'd15,4'd0, 4'd8, 4'd13, 4'd3, 4'd12,4'd9, 4'd7, 4'd5, 4'd10,4'd6, 4'd1,
           4'd13,4'd0, 4'd11,4'd7, 4'd4, 4'd9, 4'd1, 4'd10, 4'd14,4'd3, 4'd5, 4'd12,4'd2, 4'd15,4'd8, 4'd6,
           4'd1, 4'd4, 4'd11,4'd13,4'd12,4'd3, 4'd7, 4'd14, 4'd10,4'd15,4'd6, 4'd8, 4'd0, 4'd5, 4'd9, 4'd2,
           4'd6, 4'd11,4'd13,4'd8, 4'd1, 4'd4, 4'd10,4'd7,  4'd9, 4'd5, 4'd0, 4'd15,4'd14,4'd2, 4'd3, 4'd12 },
        // S8
        '{ 4'd13,4'd2, 4'd8, 4'd4, 4'd6, 4'd15,4'd11,4'd1,  4'd10,4'd9, 4'd3, 4'd14,4'd5, 4'd0, 4'd12,4'd7,
           4'd1, 4'd15,4'd13,4'd8, 4'd10,4'd3, 4'd7, 4'd4,  4'd12,4'd5, 4'd6, 4'd11,4'd0, 4'd14,4'd9, 4'd2,
           4'd7, 4'd11,4'd4, 4'd1, 4'd9, 4'd12,4'd14,4'd2,  4'd0, 4'd6, 4'd10,4'd13,4'd15,4'd3, 4'd5, 4'd8,
           4'd2, 4'd1, 4'd14,4'd7, 4'd4, 4'd10,4'd8, 4'd13, 4'd15,4'd12,4'd9, 4'd0, 4'd3, 4'd5, 4'd6, 4'd11 }
    };

    // P permutation (32 bits)
    localparam logic [4:0] P_TBL [0:31] = '{
        5'd15, 5'd6,  5'd19, 5'd20, 5'd28, 5'd11, 5'd27, 5'd16,
        5'd0,  5'd14, 5'd22, 5'd25, 5'd4,  5'd17, 5'd30, 5'd9,
        5'd1,  5'd7,  5'd23, 5'd13, 5'd31, 5'd26, 5'd2,  5'd8,
        5'd18, 5'd12, 5'd29, 5'd5,  5'd21, 5'd10, 5'd3,  5'd24
    };

    // PC1 permutation (64→56 bits)
    localparam logic [5:0] PC1_TBL [0:55] = '{
        6'd56, 6'd48, 6'd40, 6'd32, 6'd24, 6'd16, 6'd8,
        6'd0,  6'd57, 6'd49, 6'd41, 6'd33, 6'd25, 6'd17,
        6'd9,  6'd1,  6'd58, 6'd50, 6'd42, 6'd34, 6'd26,
        6'd18, 6'd10, 6'd2,  6'd59, 6'd51, 6'd43, 6'd35,
        6'd62, 6'd54, 6'd46, 6'd38, 6'd30, 6'd22, 6'd14,
        6'd6,  6'd61, 6'd53, 6'd45, 6'd37, 6'd29, 6'd21,
        6'd13, 6'd5,  6'd60, 6'd52, 6'd44, 6'd36, 6'd28,
        6'd20, 6'd12, 6'd4,  6'd27, 6'd19, 6'd11, 6'd3
    };

    // PC2 permutation (56→48 bits)
    localparam logic [5:0] PC2_TBL [0:47] = '{
        6'd13, 6'd16, 6'd10, 6'd23, 6'd0,  6'd4,
        6'd2,  6'd27, 6'd14, 6'd5,  6'd20, 6'd9,
        6'd22, 6'd18, 6'd11, 6'd3,  6'd25, 6'd7,
        6'd15, 6'd6,  6'd26, 6'd19, 6'd12, 6'd1,
        6'd40, 6'd51, 6'd30, 6'd36, 6'd46, 6'd54,
        6'd29, 6'd39, 6'd50, 6'd44, 6'd32, 6'd47,
        6'd43, 6'd48, 6'd38, 6'd55, 6'd33, 6'd52,
        6'd45, 6'd41, 6'd49, 6'd35, 6'd28, 6'd31
    };

    // Key rotation schedule (1 = left rotate by 1, 2 = left rotate by 2)
    localparam logic [1:0] KEY_ROT [0:15] = '{
        2'd1,2'd1,2'd2,2'd2,2'd2,2'd2,2'd2,2'd2,
        2'd1,2'd2,2'd2,2'd2,2'd2,2'd2,2'd2,2'd1
    };

    // Permute a word: out[i] = in[src_bits[i]]
    function automatic logic [63:0] permute64(
        input logic [63:0] data,
        input logic [5:0]  tbl [],
        input int          out_width
    );
        automatic logic [63:0] result = '0;
        automatic int i;
        for (i = 0; i < out_width; i++)
            result[out_width-1-i] = data[63 - tbl[i]];
        return result;
    endfunction

    // DES key schedule: generate 16 round keys (48-bit each, packed into 64-bit)
    function automatic void des_ks (input logic [63:0] key, output logic [47:0] ks [0:15]);
        automatic logic [27:0] C, D;
        automatic logic [55:0] CD;
        automatic int          r;

        // PC1
        begin : pc1_blk
            automatic logic [63:0] cd64 = permute64(key, PC1_TBL, 56);
            C = cd64[55:28];
            D = cd64[27:0];
        end

        for (r = 0; r < 16; r++) begin
            if (KEY_ROT[r] == 2'd1) begin
                C = {C[26:0], C[27]};
                D = {D[26:0], D[27]};
            end else begin
                C = {C[25:0], C[27:26]};
                D = {D[25:0], D[27:26]};
            end
            CD = {C, D};
            begin : pc2_blk
                automatic int bi;
                for (bi = 0; bi < 48; bi++)
                    ks[r][47-bi] = CD[55 - PC2_TBL[bi]];
            end
        end
    endfunction

    // DES F function: expand R (32→48), XOR subkey, S-box, P permutation
    function automatic logic [31:0] des_f(
        input logic [31:0] R,
        input logic [47:0] subkey
    );
        automatic logic [47:0] expanded;
        automatic logic [47:0] xored;
        automatic logic [31:0] sout;
        automatic logic [31:0] fout;
        automatic int i;

        // Expand
        for (i = 0; i < 48; i++)
            expanded[47-i] = R[31 - E_TBL[i]];

        xored = expanded ^ subkey;

        // S-boxes
        begin : sbox_blk
            automatic int s;
            for (s = 0; s < 8; s++) begin
                automatic logic [5:0] b = xored[47 - s*6 -: 6];
                automatic logic [5:0] idx = {b[5], b[0], b[4:1]};
                sout[31 - s*4 -: 4] = SBOX[s][idx];
            end
        end

        // P permutation
        for (i = 0; i < 32; i++)
            fout[31-i] = sout[31 - P_TBL[i]];

        return fout;
    endfunction

    // Single DES encrypt or decrypt
    function automatic logic [63:0] des_crypt(
        input logic [63:0] data,
        input logic [63:0] key,
        input logic        encrypt  // 1=encrypt, 0=decrypt
    );
        automatic logic [47:0] ks [0:15];
        automatic logic [31:0] L, R, tmp;
        automatic logic [63:0] blk;
        automatic int          r;

        des_ks(key, ks);
        blk = permute64(data, IP_TBL, 64);
        L   = blk[63:32];
        R   = blk[31:0];

        for (r = 0; r < 16; r++) begin
            automatic int ki = encrypt ? r : (15 - r);
            tmp = R;
            R   = L ^ des_f(R, ks[ki]);
            L   = tmp;
        end

        blk = {R, L};   // swap before FP
        return permute64(blk, FP_TBL, 64);
    endfunction

    // 3DES-EDE: K1 encrypt, K2 decrypt, K3 encrypt
    function automatic logic [63:0] ref_des3(
        input logic [63:0] k1,
        input logic [63:0] k2,
        input logic [63:0] k3,
        input logic [63:0] plaintext
    );
        automatic logic [63:0] t1, t2;
        t1 = des_crypt(plaintext, k1, 1'b1);
        t2 = des_crypt(t1,        k2, 1'b0);
        return des_crypt(t2,      k3, 1'b1);
    endfunction
`endif  // !ICARUS (permute64 .. ref_des3 block)

    // =========================================================================
    // Frame checkers
    // =========================================================================

    // Validate a Consolidator telemetry frame (37 words) — CRC-32 only.
    // C-05: unpacked array input params not supported by Icarus — guarded.
`ifndef ICARUS
    task automatic check_telemetry_frame_crc(
        input logic [15:0] words     [0:36],
        input string       test_name
    );
        automatic logic [31:0] crc_exp;
        automatic logic [15:0] crc_words [0:32];
        automatic logic [63:0] token_got;
        automatic int i;

        for (i = 0; i <= 32; i++) crc_words[i] = words[i];
        crc_exp   = ref_crc32(crc_words, 33);
        token_got = {words[33], words[34], words[35], words[36]};

        if (token_got[31:0] !== crc_exp) begin
            $error("[%s] Telemetry CRC mismatch: frame CRC=0x%08h, token[31:0]=0x%08h",
                   test_name, crc_exp, token_got[31:0]);
        end else begin
`ifdef VERBOSE
            $display("[%s] Telemetry frame PASS (CRC=0x%08h, token=0x%016h)",
                     test_name, crc_exp, token_got);
`endif
        end
    endtask

    // Validate a Consolidator control response frame (4 words)
    // C-05: unpacked array input params not supported by Icarus — guarded.
    task automatic check_response_frame(
        input logic [15:0] words [0:3],
        input logic [15:0] exp_magic,
        input logic [15:0] exp_flags,
        input logic [15:0] exp_addr,
        input logic [15:0] exp_data,
        input string       test_name
    );
        if (words[0] !== exp_magic)
            $error("[%s] Response magic mismatch: got 0x%04h, exp 0x%04h",
                   test_name, words[0], exp_magic);
        if (words[1] !== exp_flags)
            $error("[%s] Response flags mismatch: got 0x%04h, exp 0x%04h",
                   test_name, words[1], exp_flags);
        if (words[2] !== exp_addr)
            $error("[%s] Response addr mismatch: got 0x%04h, exp 0x%04h",
                   test_name, words[2], exp_addr);
        if (words[3] !== exp_data)
            $error("[%s] Response data mismatch: got 0x%04h, exp 0x%04h",
                   test_name, words[3], exp_data);
        if (words[0] === exp_magic && words[1] === exp_flags &&
            words[2] === exp_addr  && words[3] === exp_data) begin
`ifdef VERBOSE
            $display("[%s] Response frame PASS", test_name);
`endif
        end
    endtask
`endif  // !ICARUS (checker tasks)

endpackage
