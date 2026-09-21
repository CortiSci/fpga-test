// V3 end-to-end telemetry regression for software-transpose payloads.
#include "Vv3_e2e_wrap.h"
#include "verilated.h"
#include <cstdio>
#include <vector>
#include <cstring>

double sc_time_stamp() { return 0; }

static uint16_t enc(int leg, int pos) {
    return uint16_t(((leg + 1) << 12) | (pos & 0x3FF));
}

// Expected raw bit-plane for one leg. Consecutive 16 planes reconstruct the
// sixteen lane samples for one ASIC sample group in host software.
static uint16_t raw_plane(int leg, int raw_pos) {
    const int group = (raw_pos >> 4) & 63;
    const int bit = raw_pos & 15;
    uint16_t plane = 0;
    for (int lane = 0; lane < 16; ++lane)
        plane |= uint16_t(((enc(leg, group * 16 + lane) >> (15 - bit)) & 1) << lane);
    return plane;
}

static uint32_t crc32_be_words(const uint16_t* w, int n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < n; ++i) {
        const uint8_t bytes[2] = {uint8_t(w[i] >> 8), uint8_t(w[i])};
        for (int b = 0; b < 2; ++b) {
            crc ^= bytes[b];
            for (int j = 0; j < 8; ++j)
                crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char mode = (argc > 1) ? argv[1][0] : ' ';
    uint8_t start_en = 0xF;
    long late_at = -1, die_at = -1;
    int late_leg = -1, die_leg = -1;
    bool stalls = false;
    const char* desc = "all four legs from t=0";
    switch (mode) {
        case 'o': start_en = 0x1; desc = "LEG5 only"; break;
        case 'k': start_en = 0xD; late_leg = 1; late_at = 35000; desc = "LEG6 late"; break;
        case 'd': die_leg = 2; die_at = 50000; desc = "LEG7 dies"; break;
        case 's': stalls = true; desc = "25% TX stalls"; break;
        default: break;
    }

    Vv3_e2e_wrap* top = new Vv3_e2e_wrap;
    top->clk_48m = 0; top->spi_sclk = 0; top->ro1_clk = 0; top->ro1_frame = 0;
    top->rst_n = 0; top->run = 0; top->telem_en = 0; top->ro1_sd = 0;
    top->telem_start = 0; top->tx_ready = 1;
    for (int n = 0; n < 20; ++n) {
        top->clk_48m = 1; top->eval(); top->clk_48m = 0; top->eval();
    }
    top->rst_n = 1;
    for (int n = 0; n < 20; ++n) {
        top->clk_48m = 1; top->eval(); top->clk_48m = 0; top->eval();
    }
    top->run = start_en;
    top->telem_en = start_en;

    // 2.56 MHz ASIC clock against the production 50 MHz core.
    int ro1_acc = 0, ro1_lvl = 0;
    long cap = 0;
    uint32_t rng = 0x12345678u;
    auto rnd = [&]() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; };
    std::vector<uint16_t> stream;
    uint8_t alive_en = start_en;
    bool started = false;

    for (long t = 0; t < 400000; ++t) {
        if (t == late_at) { alive_en |= (1u << late_leg); top->run = alive_en; top->telem_en = alive_en; }
        if (t == die_at) { alive_en &= ~(1u << die_leg); top->telem_en = alive_en; }
        top->telem_start = (!started && t == 2000);
        if (top->telem_start) started = true;

        ro1_acc += 5120000;
        if (ro1_acc >= 50000000) {
            ro1_acc -= 50000000;
            ro1_lvl = !ro1_lvl;
            if (ro1_lvl) {
                const int group = int(cap >> 4), bit = int(cap & 15);
                uint64_t sd = 0;
                for (int leg = 0; leg < 4; ++leg)
                    sd |= uint64_t(raw_plane(leg, group * 16 + bit)) << (leg * 16);
                top->ro1_sd = sd;
                if (++cap >= 1024) cap = 0;
                top->ro1_clk = 1; top->eval();
            } else {
                top->ro1_frame = (cap == 0);
                top->ro1_clk = 0; top->eval();
            }
        }
        if (stalls) top->tx_ready = ((rnd() & 3) != 0);
        top->clk_48m = 1; top->eval();
        if (top->tx_valid) stream.push_back(top->tx_data);
        top->clk_48m = 0; top->eval();
        const uint8_t active = top->strm_active;
        if (active) { top->spi_sclk = active; top->eval(); top->spi_sclk = 0; top->eval(); }
    }

    // Negative control corrupts payload while preserving CRC, so the sample
    // identity oracle must reject it independently of the CRC check.
    if (argc > 2 && std::strcmp(argv[2], "--corrupt-payload") == 0 && stream.size() >= 4105) {
        stream[3] ^= 1;
        const uint32_t crc = crc32_be_words(stream.data(), 4103);
        stream[4103] = uint16_t(crc >> 16); stream[4104] = uint16_t(crc);
    }
    int bad = 0, nframes = 0;
    long plane_errors[4] = {}, zero_fill[4] = {}, valid_planes[4] = {};
    int clean[4] = {}, absent[4] = {}, flagged[4] = {}, last_clean[4] = {-1,-1,-1,-1};
    for (size_t i = 0; i + 4105 <= stream.size(); i += 4105, ++nframes) {
        const uint16_t* frame = stream.data() + i;
        if (frame[0] != 1 || (frame[2] & 0xE000)) ++bad;
        const uint32_t crc = crc32_be_words(frame, 4103);
        if (frame[4103] != uint16_t(crc >> 16) || frame[4104] != uint16_t(crc)) ++bad;
        for (int leg = 0; leg < 4; ++leg) {
            const uint16_t word = frame[4099 + leg];
            const int phase = word & 0x3FF;
            const bool fault = (word & 0x7000) != 0;
            if ((word & 0x8C00) || (phase > 63 && phase != 1023)) ++bad;
            if (fault) ++flagged[leg];
            if (phase == 1023) ++absent[leg];
            else if (!fault) { ++clean[leg]; last_clean[leg] = nframes; }
            for (int tick = 0; tick < 1024; ++tick) {
                const uint16_t got = frame[3 + tick * 4 + leg];
                if (phase == 1023 && (!fault || (!(start_en & (1u << leg)) && leg != late_leg))) {
                    if (got == 0) ++zero_fill[leg]; else ++plane_errors[leg];
                } else if (!fault) {
                    // Phase is the live group index, not a session tick offset.
                    const int raw_pos = (((tick / 16 + phase) & 63) * 16) + tick % 16;
                    if (got != raw_plane(leg, raw_pos)) ++plane_errors[leg];
                    ++valid_planes[leg];
                }
                // Faulted partial frames have no sample-integrity guarantee.
                // Recovery assertions below prevent all-faulted runs passing.
            }
        }
    }
    if (nframes < 10) ++bad;
    for (int leg = 0; leg < 4; ++leg) {
        const bool enabled = (start_en >> leg) & 1;
        const bool late = leg == late_leg, died = leg == die_leg;
        if (plane_errors[leg]) ++bad;
        if ((enabled || late) && clean[leg] < 2) ++bad;
        if ((enabled || late) && !died && last_clean[leg] < nframes - 3) ++bad;
        if (late && absent[leg] == 0) ++bad;
        if (!enabled && !late && absent[leg] != nframes) ++bad;
        if (died && (flagged[leg] == 0 || absent[leg] == 0)) ++bad;
        if (late_leg < 0 && die_leg < 0 && enabled && flagged[leg]) ++bad;
        std::printf("leg%d: raw=%ld zero-fill=%ld errors=%ld clean=%d absent=%d flagged=%d last-clean=%d\n",
                    leg, valid_planes[leg], zero_fill[leg], plane_errors[leg], clean[leg], absent[leg], flagged[leg], last_clean[leg]);
    }
    std::printf("scenario: %s; frames: %d; result: %s\n", desc, nframes, bad ? "FAIL" : "PASS");
    delete top;
    return bad ? 1 : 0;
}
