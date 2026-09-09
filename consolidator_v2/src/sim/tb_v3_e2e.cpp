// V3 end-to-end telemetry regression for software-transpose payloads.
#include "Vv3_e2e_wrap.h"
#include "verilated.h"
#include <cstdio>
#include <vector>

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

    const int RO1_HALF = 10;
    int ro1_div = 0, ro1_lvl = 0;
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

        if (++ro1_div >= RO1_HALF) {
            ro1_div = 0;
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

    int bad = 0, nframes = 0;
    int first_phase[4] = {-1,-1,-1,-1};
    int raw_pos[4] = {0,0,0,0};
    long plane_errors[4] = {0,0,0,0};
    long zero_fill[4] = {0,0,0,0};
    long valid_planes[4] = {0,0,0,0};
    bool stream_started[4] = {false,false,false,false};
    bool stop_checking[4] = {false,false,false,false};
    uint8_t last_phase_undf = 0;

    size_t i = 0;
    while (i + 4105 <= stream.size()) {
        const uint16_t* frame = stream.data() + i;
        if ((frame[0] & 7) != 1) { ++bad; break; }
        const uint32_t crc = crc32_be_words(frame, 4103);
        if (frame[4103] != uint16_t(crc >> 16) || frame[4104] != uint16_t(crc)) ++bad;

        for (int leg = 0; leg < 4; ++leg) {
            const int phase = frame[4099 + leg] & 0x3FF;
            const bool underflow = (frame[4099 + leg] & 0x1000) != 0;
            if (!stream_started[leg] && phase != 1023) {
                stream_started[leg] = true;
                raw_pos[leg] = 0;
            }
            if (first_phase[leg] < 0) first_phase[leg] = phase;
            for (int tick = 0; tick < 1024; ++tick) {
                const uint16_t got = frame[3 + tick * 4 + leg];
                const bool valid = stream_started[leg] &&
                    (nframes != 0 || tick >= phase) && !stop_checking[leg];
                if (!valid) {
                    if (got == 0) ++zero_fill[leg]; else ++plane_errors[leg];
                } else {
                    if (got != raw_plane(leg, raw_pos[leg])) ++plane_errors[leg];
                    raw_pos[leg] = (raw_pos[leg] + 1) & 0x3FF;
                    ++valid_planes[leg];
                }
            }
            if (underflow) stop_checking[leg] = true;
            last_phase_undf |= uint8_t(underflow << leg);
        }
        ++nframes;
        i += 4105;
    }

    if (nframes < 2) ++bad;
    for (int leg = 0; leg < 4; ++leg) {
        const bool enabled = (start_en >> leg) & 1;
        const bool late = leg == late_leg;
        const bool died = leg == die_leg;
        if (!died && plane_errors[leg]) ++bad;
        if (enabled && !late && !died && first_phase[leg] != 0) ++bad;
        if (late && first_phase[leg] == 0) ++bad;
        if (!enabled && !late && valid_planes[leg] != 0) ++bad;
        if (died && !(last_phase_undf & (1u << leg))) ++bad;
        std::printf("leg%d: raw=%ld zero-fill=%ld errors=%ld phase=%d\n",
                    leg, valid_planes[leg], zero_fill[leg], plane_errors[leg], first_phase[leg]);
    }
    std::printf("scenario: %s; frames: %d; result: %s\n", desc, nframes, bad ? "FAIL" : "PASS");
    delete top;
    return bad ? 1 : 0;
}
