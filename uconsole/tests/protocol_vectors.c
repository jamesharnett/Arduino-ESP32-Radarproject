/* Host-side companion for test_protocol.py.
 *
 * Encodes fixed test vectors with the C header, round-trips them through the C
 * decoders (asserting equality), and prints each datagram as a hex line so the
 * Python test can compare bytes and decode them independently.
 * Build: gcc -std=c99 -Wall -Wextra -Werror -I<dir of radar_protocol.h> protocol_vectors.c
 */
#include <stdio.h>
#include <stdlib.h>
#include "radar_protocol.h"

static void print_hex(const char *name, const uint8_t *b, size_t n) {
    printf("%s ", name);
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #cond); exit(1); } } while (0)

int main(void) {
    uint8_t buf[RL_MAX_PACKET];

    /* RADAR: datasheet example target in slot 0, another in slot 2, slot 1 empty */
    RlRadarPacket r = {0};
    r.frame_count = 1234;
    r.target[0].x_mm = -782;  r.target[0].y_mm = 1713; r.target[0].speed_cm_s = -16;  r.target[0].valid = 1;
    r.target[2].x_mm = 2500;  r.target[2].y_mm = 7000; r.target[2].speed_cm_s = 120;  r.target[2].valid = 1;
    r.target[1].x_mm = 999;   r.target[1].valid = 0;   /* garbage in an invalid slot must encode as zeros */
    size_t n = rl_encode_radar(buf, 7, &r);
    CHECK(n == RL_RADAR_PACKET_LEN);
    { uint8_t t; uint16_t s; CHECK(rl_read_header(buf, n, &t, &s)); CHECK(t == RL_TYPE_RADAR && s == 7); }
    { RlRadarPacket d; CHECK(rl_decode_radar(buf, n, &d));
      CHECK(d.frame_count == 1234 && d.target[0].x_mm == -782 && d.target[0].y_mm == 1713 &&
            d.target[0].speed_cm_s == -16 && d.target[0].valid == 1 && d.target[1].valid == 0 &&
            d.target[1].x_mm == 0 && d.target[2].x_mm == 2500 && d.target[2].speed_cm_s == 120); }
    print_hex("radar", buf, n);

    /* LIDAR: 3 points, then a full 96-point packet */
    RlLidarPoint pts[RL_LIDAR_MAX_POINTS];
    pts[0] = (RlLidarPoint){0, 1500, 200};
    pts[1] = (RlLidarPoint){9000, 12000, 7};
    pts[2] = (RlLidarPoint){35999, 20, 255};
    n = rl_encode_lidar(buf, 65535, 3600, pts, 3);
    CHECK(n == RL_LIDAR_PACKET_LEN(3));
    { RlLidarHeader h; CHECK(rl_decode_lidar_header(buf, n, &h)); CHECK(h.count == 3 && h.scan_speed_deg_s == 3600);
      RlLidarPoint p; rl_decode_lidar_point(buf, 2, &p); CHECK(p.angle_cdeg == 35999 && p.dist_mm == 20 && p.intensity == 255); }
    print_hex("lidar3", buf, n);
    for (unsigned i = 0; i < RL_LIDAR_MAX_POINTS; i++) pts[i] = (RlLidarPoint){(uint16_t)(i * 375), (uint16_t)(100 + i * 7), (uint8_t)i};
    n = rl_encode_lidar(buf, 1, 3598, pts, RL_LIDAR_MAX_POINTS);
    CHECK(n == 492 && n <= RL_MAX_PACKET);
    { RlLidarHeader h; CHECK(rl_decode_lidar_header(buf, n, &h)); CHECK(h.count == RL_LIDAR_MAX_POINTS);
      CHECK(!rl_decode_lidar_header(buf, n - 1, &h)); /* truncated must fail */ }
    print_hex("lidar96", buf, n);

    /* STATUS */
    RlStatusPacket st = {0};
    st.uptime_ms = 123456789; st.radar_fps = 20; st.lidar_fps = 375; st.lidar_pps = 4321;
    st.subscribers = 2; st.flags = RL_STATUS_RADAR_OK | RL_STATUS_LIDAR_OK | RL_STATUS_LIDAR_ENABLED;
    st.radar_bad_frames = 3; st.lidar_crc_errors = 4; st.reserved = 0;
    n = rl_encode_status(buf, 300, &st);
    CHECK(n == RL_STATUS_PACKET_LEN);
    { RlStatusPacket d; CHECK(rl_decode_status(buf, n, &d)); CHECK(d.uptime_ms == 123456789 && d.lidar_fps == 375 && d.flags == 0x0B && d.lidar_crc_errors == 4); }
    print_hex("status", buf, n);

    /* HELLO */
    n = rl_encode_hello(buf, 2, RL_RECEIVER_GIGA, RL_WANT_RADAR | RL_WANT_STATUS);
    CHECK(n == RL_HELLO_PACKET_LEN);
    { RlHelloPacket h; CHECK(rl_decode_hello(buf, n, &h)); CHECK(h.receiver_kind == RL_RECEIVER_GIGA && h.wants == 0x05); }
    print_hex("hello", buf, n);

    /* header rejection + seq arithmetic */
    { uint8_t t; CHECK(!rl_read_header(buf, 7, &t, NULL)); buf[5] = 2; CHECK(!rl_read_header(buf, n, &t, NULL)); }
    CHECK(rl_seq_newer(1, 0) && rl_seq_newer(0, 65535) && !rl_seq_newer(5, 5) && !rl_seq_newer(65535, 0) && !rl_seq_newer(10, 20));
    printf("ok\n");
    return 0;
}
