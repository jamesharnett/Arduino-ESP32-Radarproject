/* Host-side companion for test_sensors.py.
 *
 * Reads a raw byte stream from a file, runs it through the C++ parser named on
 * the command line (rd03d_parser.h or ld19_parser.h, exactly as the firmware
 * does) and prints one line per decoded frame plus the counters, so the Python
 * test can compare uconsole/radar_sensors.py against the firmware's own code.
 *
 *   g++ -std=c++11 -Wall -Wextra -Werror -I<dir of rd03d_parser.h> sensor_vectors.cpp -o sensor_vectors
 *   ./sensor_vectors radar stream.bin     # frame <offset> | x y spd valid x3 (after rd03dProcess, default options)
 *   ./sensor_vectors lidar stream.bin     # frame <offset> speed start end ts | 12 x (angle dist intensity)
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "radar_protocol.h"
#include "rd03d_parser.h"
#include "ld19_parser.h"

static std::vector<uint8_t> read_file(const char *path) {
    std::vector<uint8_t> data;
    FILE *f = std::fopen(path, "rb");
    if (!f) { std::perror(path); std::exit(2); }
    uint8_t buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) data.insert(data.end(), buf, buf + n);
    std::fclose(f);
    return data;
}

int main(int argc, char **argv) {
    if (argc != 3) { std::fprintf(stderr, "usage: %s radar|lidar FILE\n", argv[0]); return 2; }
    std::vector<uint8_t> data = read_file(argv[2]);

    if (!std::strcmp(argv[1], "radar")) {
        Rd03dParser p; Rd03dFrame f;
        Rd03dOptions o = {100, 8000, 1000, true};
        for (size_t i = 0; i < data.size(); i++) {
            if (!p.feed(data[i], f)) continue;
            RlRadarPacket pkt = {};
            rd03dProcess(f, o, pkt);
            std::printf("frame %zu |", i + 1 - RD03D_FRAME_LEN);
            for (unsigned s = 0; s < RL_RADAR_SLOTS; s++)
                std::printf(" %d %d %d %u", pkt.target[s].x_mm, pkt.target[s].y_mm, pkt.target[s].speed_cm_s, pkt.target[s].valid);
            std::printf("\n");
        }
        std::printf("frames=%lu bad=%lu\n", (unsigned long)p.frames, (unsigned long)p.badFrames);
        return 0;
    }
    if (!std::strcmp(argv[1], "lidar")) {
        Ld19Parser p; Ld19Frame f;
        for (size_t i = 0; i < data.size(); i++) {
            if (!p.feed(data[i], f)) continue;
            std::printf("frame %zu %u %u %u %u |", i + 1 - LD19_FRAME_LEN, f.speed_deg_s, f.start_angle_cdeg, f.end_angle_cdeg, f.timestamp_ms);
            for (unsigned k = 0; k < LD19_POINTS; k++)
                std::printf(" %u %u %u", ld19PointAngleCdeg(f.start_angle_cdeg, f.end_angle_cdeg, k), f.dist_mm[k], f.intensity[k]);
            std::printf("\n");
        }
        std::printf("frames=%lu crc=%lu\n", (unsigned long)p.frames, (unsigned long)p.crcErrors);
        return 0;
    }
    std::fprintf(stderr, "unknown sensor %s\n", argv[1]);
    return 2;
}
