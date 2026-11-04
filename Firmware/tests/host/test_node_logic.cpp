// Host-side unit tests for the sensor node's console parser and self-diagnosis.
// Build and run:  g++ -std=c++11 -Wall -Wextra -Werror -I Firmware/sensor_node_esp32s3
//     Firmware/tests/host/test_node_logic.cpp -o /tmp/test_node_logic && /tmp/test_node_logic
#include <cstdio>
#include <cstring>
#include <string>
#include "node_console.h"
#include "node_diag.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

// ---------------------------------------------------------------- console
static int feed_line(ConsoleLine &c, const char *s, ConsoleCommand &out) {
    int completed = 0;
    for (const char *p = s; *p; p++) if (c.feed(*p, out)) completed++;
    return completed;
}

static void test_console_basic() {
    ConsoleLine c; ConsoleCommand cmd;
    CHECK(feed_line(c, "help\n", cmd) == 1 && cmd.cmd == CON_HELP);
    CHECK(feed_line(c, "?\n", cmd) == 1 && cmd.cmd == CON_HELP);
    CHECK(feed_line(c, "S\r\n", cmd) == 1 && cmd.cmd == CON_STAT);          // CRLF = one command
    CHECK(feed_line(c, "diag\r", cmd) == 1 && cmd.cmd == CON_DIAG);         // bare CR terminates too
    CHECK(feed_line(c, "\n", cmd) == 0);                                     // the LF of a CRLF pair, and empty lines
    CHECK(feed_line(c, "\n\n   \n", cmd) == 0);
    CHECK(feed_line(c, "subs\n", cmd) == 1 && cmd.cmd == CON_SUBS);
    CHECK(feed_line(c, "wifi\n", cmd) == 1 && cmd.cmd == CON_WIFI);
    CHECK(feed_line(c, "reconfig\n", cmd) == 1 && cmd.cmd == CON_RECONFIG);
    CHECK(feed_line(c, "reboot\n", cmd) == 1 && cmd.cmd == CON_REBOOT);
    CHECK(feed_line(c, "  stat   \n", cmd) == 1 && cmd.cmd == CON_STAT);    // surrounding blanks
}

static void test_console_arguments() {
    ConsoleLine c; ConsoleCommand cmd;
    CHECK(feed_line(c, "RAW LIDAR 1000\r\n", cmd) == 1);
    CHECK(cmd.cmd == CON_RAW && !std::strcmp(cmd.arg, "lidar") && cmd.num == 1000);
    CHECK(feed_line(c, "raw radar\n", cmd) == 1);
    CHECK(cmd.cmd == CON_RAW && !std::strcmp(cmd.arg, "radar") && cmd.num == -1);
    CHECK(feed_line(c, "raw 600\n", cmd) == 1);                              // number without a source
    CHECK(cmd.cmd == CON_RAW && cmd.arg[0] == '\0' && cmd.num == 600);
    CHECK(feed_line(c, "raw\n", cmd) == 1 && cmd.cmd == CON_RAW && cmd.arg[0] == '\0' && cmd.num == -1);
    CHECK(feed_line(c, "log off\n", cmd) == 1 && cmd.cmd == CON_LOG && !std::strcmp(cmd.arg, "off"));
    CHECK(feed_line(c, "log ON\n", cmd) == 1 && cmd.cmd == CON_LOG && !std::strcmp(cmd.arg, "on"));
    CHECK(feed_line(c, "raw lidar abc\n", cmd) == 1 && cmd.cmd == CON_RAW && cmd.num == -1);   // non-numeric count ignored
}

static void test_console_editing_and_limits() {
    ConsoleLine c; ConsoleCommand cmd;
    CHECK(feed_line(c, "sta\x08\x08tat\n", cmd) == 1 && cmd.cmd == CON_STAT);   // backspace: "s" + "tat"
    CHECK(feed_line(c, "x\x7f\x7f\x7fhelp\n", cmd) == 1 && cmd.cmd == CON_HELP); // deleting past the start is harmless
    CHECK(feed_line(c, "frobnicate now\n", cmd) == 1 && cmd.cmd == CON_UNKNOWN && !std::strcmp(cmd.raw, "frobnicate now"));
    std::string longline(200, 'a'); longline += "\n";                        // longer than the buffer: truncated, not overflowed
    CHECK(feed_line(c, longline.c_str(), cmd) == 1 && cmd.cmd == CON_UNKNOWN && std::strlen(cmd.raw) == CONSOLE_LINE_MAX);
    CHECK(feed_line(c, "\x1b[A\n", cmd) == 1 && cmd.cmd == CON_UNKNOWN);      // escape sequence: ESC dropped, "[A" unknown
    CHECK(feed_line(c, "stat\n", cmd) == 1 && cmd.cmd == CON_STAT);           // still usable afterwards
}

// ------------------------------------------------------------------- diag
static DiagInput healthy() {
    DiagInput in = {};
    in.uptime_ms = 60000;
    in.radar_bytes = 36000; in.radar_frames = 1200; in.radar_bad_frames = 3; in.radar_fps = 20;
    in.radar_last_frame_age_ms = 40; in.radar_configured = true;
    in.lidar_enabled = true; in.lidar_bytes = 1000000; in.lidar_frames = 22000; in.lidar_crc_errors = 10;
    in.lidar_fps = 375; in.lidar_pps = 4100; in.lidar_last_frame_age_ms = 3; in.lidar_speed_deg_s = 3610;
    in.ap_up = true; in.stations = 2; in.subscribers = 2;
    in.free_heap = 250000; in.min_free_heap = 200000;
    return in;
}

static const DiagLine *find(const DiagLine *l, size_t n, const char *sub) {
    for (size_t i = 0; i < n; i++) if (!std::strcmp(l[i].subsystem, sub)) return &l[i];
    return nullptr;
}

static bool has(const DiagLine *l, const char *needle) { return l && std::strstr(l->text, needle) != nullptr; }

static void test_diag_healthy() {
    DiagLine lines[DIAG_MAX_LINES];
    size_t n = nodeDiagnose(healthy(), lines, DIAG_MAX_LINES);
    CHECK(n == 4);
    CHECK(find(lines, n, "radar") && find(lines, n, "radar")->level == DIAG_OK);
    CHECK(find(lines, n, "lidar") && find(lines, n, "lidar")->level == DIAG_OK && has(find(lines, n, "lidar"), "10.0 Hz"));
    CHECK(find(lines, n, "wifi") && find(lines, n, "wifi")->level == DIAG_OK);
    CHECK(find(lines, n, "heap") && find(lines, n, "heap")->level == DIAG_INFO);
    CHECK(diagWorst(lines, n) == DIAG_OK);
}

static void test_diag_boot_and_wiring() {
    DiagLine l[DIAG_MAX_LINES]; size_t n;
    DiagInput in = healthy(); in.uptime_ms = 1000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_INFO && find(l, n, "lidar")->level == DIAG_INFO);

    in = healthy(); in.radar_bytes = 0; in.radar_frames = 0; in.radar_bad_frames = 0; in.radar_fps = 0; in.radar_last_frame_age_ms = DIAG_NEVER;
    in.lidar_bytes = 0; in.lidar_frames = 0; in.lidar_crc_errors = 0; in.lidar_fps = 0; in.lidar_pps = 0; in.lidar_last_frame_age_ms = DIAG_NEVER;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_FAIL && has(find(l, n, "radar"), "not powered") && has(find(l, n, "radar"), "D0"));
    CHECK(find(l, n, "lidar")->level == DIAG_FAIL && has(find(l, n, "lidar"), "D2"));
    CHECK(diagWorst(l, n) == DIAG_FAIL);

    // bytes but never a header: baud rate
    in.radar_bytes = 5000; in.lidar_bytes = 50000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_FAIL && has(find(l, n, "radar"), "256000"));
    CHECK(find(l, n, "lidar")->level == DIAG_FAIL && has(find(l, n, "lidar"), "230400"));

    // a few bytes only: wait
    in.radar_bytes = 20;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_INFO);

    // headers but every frame bad
    in.radar_bytes = 5000; in.radar_bad_frames = 40; in.lidar_crc_errors = 900;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_FAIL && has(find(l, n, "radar"), "tail"));
    CHECK(find(l, n, "lidar")->level == DIAG_FAIL && has(find(l, n, "lidar"), "CRC"));

    // configuring in progress is never a failure
    in = healthy(); in.radar_configuring = true; in.radar_bytes = 0;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_INFO);
}

static void test_diag_quality() {
    DiagLine l[DIAG_MAX_LINES]; size_t n;
    DiagInput in = healthy(); in.radar_bad_frames = 300;                     // 25 % bad
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_WARN && has(find(l, n, "radar"), "25% bad"));

    in = healthy(); in.radar_last_frame_age_ms = 4000; in.radar_reconfigs = 2;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_WARN && has(find(l, n, "radar"), "stopped 4000 ms ago"));

    in = healthy(); in.radar_fps = 2;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_WARN && has(find(l, n, "radar"), "frame rate"));

    in = healthy(); in.radar_reconfigs = 1;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "radar")->level == DIAG_WARN && has(find(l, n, "radar"), "silent 1 time"));

    in = healthy(); in.lidar_crc_errors = 2200;                              // 10 %
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_WARN && has(find(l, n, "lidar"), "CRC errors"));

    in = healthy(); in.lidar_speed_deg_s = 1200;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_WARN && has(find(l, n, "lidar"), "scan speed 1200"));

    in = healthy(); in.lidar_fps = 120;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_WARN && has(find(l, n, "lidar"), "120 frames/s"));

    in = healthy(); in.lidar_pps = 400;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_INFO && has(find(l, n, "lidar"), "beyond range"));

    in = healthy(); in.lidar_last_frame_age_ms = 5000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_WARN && has(find(l, n, "lidar"), "stopped"));

    in = healthy(); in.lidar_enabled = false;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "lidar")->level == DIAG_INFO && has(find(l, n, "lidar"), "disabled"));
    CHECK(diagWorst(l, n) == DIAG_OK);
}

static void test_diag_wifi_and_heap() {
    DiagLine l[DIAG_MAX_LINES]; size_t n;
    DiagInput in = healthy(); in.ap_up = false;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "wifi")->level == DIAG_FAIL && has(find(l, n, "wifi"), "8..63"));

    in = healthy(); in.stations = 0; in.subscribers = 0; in.uptime_ms = 5000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "wifi")->level == DIAG_INFO);                            // early: nobody expected yet
    in.uptime_ms = 120000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "wifi")->level == DIAG_WARN);                            // two minutes: that is a problem

    in = healthy(); in.stations = 1; in.subscribers = 0;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "wifi")->level == DIAG_WARN && has(find(l, n, "wifi"), "HELLO"));

    in = healthy(); in.min_free_heap = 12000;
    n = nodeDiagnose(in, l, DIAG_MAX_LINES);
    CHECK(find(l, n, "heap")->level == DIAG_WARN && has(find(l, n, "heap"), "12000"));
    CHECK(diagWorst(l, n) == DIAG_WARN);

    // the line limit is respected
    CHECK(nodeDiagnose(healthy(), l, 2) == 2);
    CHECK(nodeDiagnose(healthy(), l, 0) == 0);
}

static void test_percent() {
    CHECK(diagPercent(0, 0) == 0);
    CHECK(diagPercent(5, 0) == 100);
    CHECK(diagPercent(1, 3) == 33);
    CHECK(diagPercent(4000000000u, 4000000000u) == 100);                      // no 32-bit overflow
}

int main() {
    test_console_basic();
    test_console_arguments();
    test_console_editing_and_limits();
    test_diag_healthy();
    test_diag_boot_and_wiring();
    test_diag_quality();
    test_diag_wifi_and_heap();
    test_percent();
    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("test_node_logic: all checks passed\n");
    return 0;
}
