// Host-side unit tests for the sensor node's Arduino-free parsers.
// Build and run:  g++ -std=c++11 -Wall -Wextra -Werror -I Firmware/sensor_node_esp32s3
//     Firmware/tests/host/test_parsers.cpp -o /tmp/test_parsers && /tmp/test_parsers
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "radar_protocol.h"
#include "rd03d_parser.h"
#include "ld19_parser.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

// ------------------------------------------------------------------ RD-03D
static void test_rd03d_decode() {
    // Datasheet example: 0E 03 -> -782, B1 86 -> 1713, 10 00 -> -16
    CHECK(rd03dDecode(0x030E) == -782);
    CHECK(rd03dDecode(0x86B1) == 1713);
    CHECK(rd03dDecode(0x0010) == -16);
    CHECK(rd03dDecode(0x0000) == 0);
    CHECK(rd03dDecode(0x8000) == 0);
    CHECK(rd03dDecode(0xFFFF) == 32767);
    CHECK(rd03dDecode(0x7FFF) == -32767);
}

static std::vector<uint8_t> rd03d_frame(const uint8_t t0[8], const uint8_t t1[8], const uint8_t t2[8]) {
    std::vector<uint8_t> f = {0xAA, 0xFF, 0x03, 0x00};
    f.insert(f.end(), t0, t0 + 8); f.insert(f.end(), t1, t1 + 8); f.insert(f.end(), t2, t2 + 8);
    f.push_back(0x55); f.push_back(0xCC);
    return f;
}

static const uint8_t ZERO8[8] = {0};

static void test_rd03d_parser_and_process() {
    const uint8_t example[8] = {0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x68, 0x01};
    std::vector<uint8_t> frame = rd03d_frame(example, ZERO8, ZERO8);
    CHECK(frame.size() == RD03D_FRAME_LEN);

    Rd03dParser p; Rd03dFrame out; int got = 0;
    // garbage, a false header start, then the frame, then a frame with a bad tail
    std::vector<uint8_t> stream = {0x00, 0xAA, 0xAA, 0x12};
    stream.insert(stream.end(), frame.begin(), frame.end());
    std::vector<uint8_t> bad = frame; bad[28] = 0x00;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), frame.begin(), frame.end());
    for (uint8_t c : stream) if (p.feed(c, out)) got++;
    CHECK(got == 2);
    CHECK(p.frames == 2 && p.badFrames == 1);

    Rd03dOptions o = {100, 8000, 1000, true};
    RlRadarPacket pkt = {};
    rd03dProcess(out, o, pkt);
    CHECK(pkt.target[0].valid == 1 && pkt.target[0].x_mm == -782 && pkt.target[0].y_mm == 1713 && pkt.target[0].speed_cm_s == -16);
    CHECK(pkt.target[1].valid == 0 && pkt.target[2].valid == 0);
}

static void put_target(uint8_t *b, int16_t x, int16_t y, int16_t spd) {
    auto enc = [](int16_t v) -> uint16_t { return v >= 0 ? (uint16_t)(0x8000 | v) : (uint16_t)(-v); };
    uint16_t ex = enc(x), ey = enc(y), es = enc(spd);
    b[0] = ex & 0xFF; b[1] = ex >> 8; b[2] = ey & 0xFF; b[3] = ey >> 8; b[4] = es & 0xFF; b[5] = es >> 8; b[6] = 0x68; b[7] = 0x01;
}

static void test_rd03d_filters() {
    Rd03dFrame f; std::memset(&f, 0, sizeof f);
    put_target(f.payload + 0,  500,  2000,  30);   // good
    put_target(f.payload + 8,  600,  2300, -80);   // 0.32 m from slot 0 -> clusters into it, faster magnitude wins
    put_target(f.payload + 16, 100,  9000,  10);   // beyond 8 m -> dropped
    Rd03dOptions o = {100, 8000, 1000, true};
    RlRadarPacket pkt = {};
    rd03dProcess(f, o, pkt);
    CHECK(pkt.target[0].valid == 1 && pkt.target[0].x_mm == 550 && pkt.target[0].y_mm == 2150 && pkt.target[0].speed_cm_s == -80);
    CHECK(pkt.target[1].valid == 0 && pkt.target[2].valid == 0);

    // sentinel speeds are noise
    std::memset(&f, 0, sizeof f);
    put_target(f.payload + 0, 500, 2000, 248);
    put_target(f.payload + 8, -500, 2000, -256);
    put_target(f.payload + 16, 0, 3000, 0);
    rd03dProcess(f, o, pkt);
    CHECK(!pkt.target[0].valid && !pkt.target[1].valid && !pkt.target[2].valid);
    o.filterSentinels = false;
    rd03dProcess(f, o, pkt);
    CHECK(pkt.target[0].valid && pkt.target[1].valid && pkt.target[2].valid);

    // clustering off keeps both
    std::memset(&f, 0, sizeof f);
    put_target(f.payload + 0, 500, 2000, 30);
    put_target(f.payload + 8, 600, 2300, -80);
    o.clusterDistMm = 0;
    rd03dProcess(f, o, pkt);
    CHECK(pkt.target[0].valid && pkt.target[1].valid);

    // minimum range gate
    std::memset(&f, 0, sizeof f);
    put_target(f.payload + 0, 30, 40, 30);   // 50 mm
    o.clusterDistMm = 1000;
    rd03dProcess(f, o, pkt);
    CHECK(!pkt.target[0].valid);
}

// -------------------------------------------------------------------- LD19
static void test_ld19_crc_table() {
    for (int i = 0; i < 256; i++) {
        uint8_t c = (uint8_t)i;
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
        CHECK(LD19_CRC_TABLE[i] == c);
    }
}

static std::vector<uint8_t> ld19_frame(uint16_t speed, uint16_t start, uint16_t end, const uint16_t *dist, const uint8_t *inten, uint16_t ts) {
    std::vector<uint8_t> f(LD19_FRAME_LEN);
    f[0] = LD19_HEADER; f[1] = LD19_VERLEN;
    f[2] = speed & 0xFF; f[3] = speed >> 8; f[4] = start & 0xFF; f[5] = start >> 8;
    for (unsigned i = 0; i < LD19_POINTS; i++) { f[6 + i * 3] = dist[i] & 0xFF; f[7 + i * 3] = dist[i] >> 8; f[8 + i * 3] = inten[i]; }
    f[42] = end & 0xFF; f[43] = end >> 8; f[44] = ts & 0xFF; f[45] = ts >> 8;
    f[46] = ld19Crc8(f.data(), 46);
    return f;
}

static void test_ld19_parser() {
    uint16_t dist[12]; uint8_t inten[12];
    for (unsigned i = 0; i < 12; i++) { dist[i] = (uint16_t)(1000 + i); inten[i] = (uint8_t)(10 * i); }
    dist[5] = 0;  // no return
    std::vector<uint8_t> frame = ld19_frame(3600, 0, 1100, dist, inten, 1234);

    Ld19Parser p; Ld19Frame out; int got = 0;
    std::vector<uint8_t> stream = {0x11, 0x54, 0x00, 0x54, 0x54};   // noise, false starts, header-byte repeated
    stream.insert(stream.end(), frame.begin() + 1, frame.end());      // the last 0x54 above is the real header
    std::vector<uint8_t> bad = frame; bad[20] ^= 0x01;                 // corrupt a distance byte
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), frame.begin(), frame.end());
    for (uint8_t c : stream) if (p.feed(c, out)) got++;
    CHECK(got == 2);
    CHECK(p.frames == 2 && p.crcErrors == 1);
    CHECK(out.speed_deg_s == 3600 && out.start_angle_cdeg == 0 && out.end_angle_cdeg == 1100 && out.timestamp_ms == 1234);
    CHECK(out.dist_mm[0] == 1000 && out.dist_mm[11] == 1011 && out.dist_mm[5] == 0 && out.intensity[3] == 30);

    for (unsigned i = 0; i < 12; i++) CHECK(ld19PointAngleCdeg(out.start_angle_cdeg, out.end_angle_cdeg, i) == i * 100);
    // wrap across 360: 359.00 -> 6.00 is a 7 degree span
    CHECK(ld19PointAngleCdeg(35900, 600, 0) == 35900);
    CHECK(ld19PointAngleCdeg(35900, 600, 11) == 600);
    CHECK(ld19PointAngleCdeg(35900, 600, 2) == (35900 + 700 * 2 / 11) % 36000);
}

// --------------------------------------------------------------- protocol
static void test_lidar_packet_fits_giga_buffer() {
    CHECK(RL_LIDAR_PACKET_LEN(RL_LIDAR_MAX_POINTS) <= RL_MAX_PACKET);
    CHECK(8u * LD19_POINTS == RL_LIDAR_MAX_POINTS);
}

int main() {
    test_rd03d_decode();
    test_rd03d_parser_and_process();
    test_rd03d_filters();
    test_ld19_crc_table();
    test_ld19_parser();
    test_lidar_packet_fits_giga_buffer();
    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("all parser tests passed\n");
    return 0;
}
