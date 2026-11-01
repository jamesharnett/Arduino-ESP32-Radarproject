/*
 * radar_protocol.h — RadarLink v1 wire format (shared by every node)
 *
 * One UDP port (RL_PORT) carries every message. Every datagram starts with the
 * same 8-byte header; the payload layout depends on the type byte. All fields
 * are little-endian and are written/read byte-by-byte, so this header produces
 * identical bytes on Xtensa (ESP32-S3), Cortex-M7 (GIGA R1) and an x86 host —
 * no struct packing or alignment assumptions anywhere.
 *
 * The canonical copy lives in Firmware/sensor_node_esp32s3/. The copy in
 * Firmware/giga_display/ must be byte-identical (tools/check_protocol_sync.py
 * enforces this) and uconsole/radar_protocol.py mirrors it for Python.
 * The human-readable description is docs/PROTOCOL.md.
 *
 *   Offset  Size  Field
 *   0       4     magic      RL_MAGIC ("RDL1" as ASCII bytes 52 44 4C 31)
 *   4       1     type       RL_TYPE_*
 *   5       1     version    RL_VERSION (1)
 *   6       2     seq        per-type sequence counter of the sender (wraps)
 *   8       ...   payload    see rl_encode_* / rl_decode_* below
 *
 * Datagrams never exceed RL_MAX_PACKET (508 bytes): that is the size of the
 * receive buffer in the Arduino Mbed WiFiUDP implementation on the GIGA R1 and
 * anything longer would be silently truncated there.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- constants */
#define RL_PORT                 4210u        /* UDP port used by every node          */
#define RL_MAGIC                0x314C4452u  /* "RDL1" little-endian                  */
#define RL_VERSION              1u
#define RL_HEADER_LEN           8u
#define RL_MAX_PACKET           508u         /* Arduino Mbed WiFiUDP buffer size      */

#define RL_TYPE_RADAR           0x01u        /* sensor node -> receivers              */
#define RL_TYPE_LIDAR           0x02u        /* sensor node -> receivers              */
#define RL_TYPE_STATUS          0x03u        /* sensor node -> receivers (1 Hz)       */
#define RL_TYPE_HELLO           0x10u        /* receiver -> sensor node (1 Hz)        */

#define RL_RADAR_SLOTS          3u           /* RD-03D tracks at most 3 targets       */
#define RL_RADAR_PAYLOAD_LEN    (4u + RL_RADAR_SLOTS * 8u)              /* 28     */
#define RL_RADAR_PACKET_LEN     (RL_HEADER_LEN + RL_RADAR_PAYLOAD_LEN)   /* 36     */

#define RL_LIDAR_POINT_LEN      5u
#define RL_LIDAR_HDR_LEN        4u           /* scan_speed + count + flags            */
#define RL_LIDAR_MAX_POINTS     96u          /* 8 LD19 frames; 12 + 5*96 = 492 bytes  */
#define RL_LIDAR_PACKET_LEN(n)  (RL_HEADER_LEN + RL_LIDAR_HDR_LEN + RL_LIDAR_POINT_LEN * (size_t)(n))

#define RL_STATUS_PAYLOAD_LEN   24u
#define RL_STATUS_PACKET_LEN    (RL_HEADER_LEN + RL_STATUS_PAYLOAD_LEN)  /* 32     */

#define RL_HELLO_PAYLOAD_LEN    4u
#define RL_HELLO_PACKET_LEN     (RL_HEADER_LEN + RL_HELLO_PAYLOAD_LEN)   /* 12     */

/* receiver_kind in HELLO */
#define RL_RECEIVER_GIGA        1u
#define RL_RECEIVER_UCONSOLE    2u
#define RL_RECEIVER_OTHER       3u

/* wants bitmask in HELLO */
#define RL_WANT_RADAR           0x01u
#define RL_WANT_LIDAR           0x02u
#define RL_WANT_STATUS          0x04u
#define RL_WANT_ALL             (RL_WANT_RADAR | RL_WANT_LIDAR | RL_WANT_STATUS)

/* flags in STATUS */
#define RL_STATUS_RADAR_OK          0x01u    /* radar frames seen in the last second   */
#define RL_STATUS_LIDAR_OK          0x02u    /* lidar frames seen in the last second   */
#define RL_STATUS_RADAR_CONFIGURED  0x04u    /* multi-target mode command acknowledged */
#define RL_STATUS_LIDAR_ENABLED     0x08u    /* firmware built with LIDAR support      */

/* timing conventions shared by all nodes */
#define RL_HELLO_INTERVAL_MS        1000u    /* receivers send HELLO this often         */
#define RL_SUBSCRIBER_TIMEOUT_MS    5000u    /* sender forgets a receiver after this    */
#define RL_LINK_TIMEOUT_MS          3000u    /* receiver declares the link dead         */

/* ------------------------------------------------------------ decoded forms */
typedef struct {
    int16_t x_mm;        /* lateral, +x = right of the radar's forward axis   */
    int16_t y_mm;        /* forward distance                                  */
    int16_t speed_cm_s;  /* radial speed, sign as reported by the RD-03D      */
    uint8_t valid;       /* 0 = slot empty                                    */
} RlRadarTarget;

typedef struct {
    uint32_t      frame_count;
    RlRadarTarget target[RL_RADAR_SLOTS];
} RlRadarPacket;

typedef struct {
    uint16_t angle_cdeg; /* 0..35999, hundredths of a degree, LD19 convention */
    uint16_t dist_mm;    /* 0 never appears: invalid points are dropped       */
    uint8_t  intensity;
} RlLidarPoint;

typedef struct {
    uint16_t scan_speed_deg_s;
    uint8_t  count;
    uint8_t  flags;      /* reserved, 0                                       */
} RlLidarHeader;

typedef struct {
    uint32_t uptime_ms;
    uint16_t radar_fps;         /* RD-03D frames parsed in the last second    */
    uint16_t lidar_fps;         /* LD19 frames parsed in the last second      */
    uint16_t lidar_pps;         /* lidar points sent in the last second       */
    uint8_t  subscribers;
    uint8_t  flags;             /* RL_STATUS_*                                */
    uint32_t radar_bad_frames;  /* cumulative                                 */
    uint32_t lidar_crc_errors;  /* cumulative                                 */
    uint32_t reserved;
} RlStatusPacket;

typedef struct {
    uint8_t  receiver_kind;     /* RL_RECEIVER_*                              */
    uint8_t  wants;             /* RL_WANT_*                                  */
    uint16_t reserved;
} RlHelloPacket;

/* ------------------------------------------------------- byte-level helpers */
static inline void rl_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void rl_put_i16(uint8_t *p, int16_t v)  { rl_put_u16(p, (uint16_t)v); }
static inline void rl_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t rl_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline int16_t  rl_get_i16(const uint8_t *p) { return (int16_t)rl_get_u16(p); }
static inline uint32_t rl_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------- header */
static inline void rl_write_header(uint8_t *buf, uint8_t type, uint16_t seq) {
    rl_put_u32(buf, RL_MAGIC);
    buf[4] = type;
    buf[5] = (uint8_t)RL_VERSION;
    rl_put_u16(buf + 6, seq);
}

/* Validates magic + version and returns the type/seq. False = not ours. */
static inline bool rl_read_header(const uint8_t *buf, size_t len, uint8_t *type, uint16_t *seq) {
    if (len < RL_HEADER_LEN) return false;
    if (rl_get_u32(buf) != RL_MAGIC) return false;
    if (buf[5] != RL_VERSION) return false;
    if (type) *type = buf[4];
    if (seq)  *seq  = rl_get_u16(buf + 6);
    return true;
}

/* True when `seq` is newer than `last` on a wrapping 16-bit counter. */
static inline bool rl_seq_newer(uint16_t seq, uint16_t last) {
    return (uint16_t)(seq - last) != 0 && (uint16_t)(seq - last) < 0x8000u;
}

/* -------------------------------------------------------------------- RADAR */
static inline size_t rl_encode_radar(uint8_t *buf, uint16_t seq, const RlRadarPacket *r) {
    rl_write_header(buf, RL_TYPE_RADAR, seq);
    uint8_t *p = buf + RL_HEADER_LEN;
    rl_put_u32(p, r->frame_count); p += 4;
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        const RlRadarTarget *t = &r->target[i];
        rl_put_i16(p + 0, t->valid ? t->x_mm : 0);
        rl_put_i16(p + 2, t->valid ? t->y_mm : 0);
        rl_put_i16(p + 4, t->valid ? t->speed_cm_s : 0);
        p[6] = t->valid ? 1u : 0u;
        p[7] = 0;
        p += 8;
    }
    return RL_RADAR_PACKET_LEN;
}

static inline bool rl_decode_radar(const uint8_t *buf, size_t len, RlRadarPacket *r) {
    if (len < RL_RADAR_PACKET_LEN) return false;
    const uint8_t *p = buf + RL_HEADER_LEN;
    r->frame_count = rl_get_u32(p); p += 4;
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        r->target[i].x_mm       = rl_get_i16(p + 0);
        r->target[i].y_mm       = rl_get_i16(p + 2);
        r->target[i].speed_cm_s = rl_get_i16(p + 4);
        r->target[i].valid      = p[6] ? 1u : 0u;
        p += 8;
    }
    return true;
}

/* -------------------------------------------------------------------- LIDAR */
/* Encodes up to RL_LIDAR_MAX_POINTS points. Returns the datagram length. */
static inline size_t rl_encode_lidar(uint8_t *buf, uint16_t seq, uint16_t scan_speed_deg_s,
                                     const RlLidarPoint *pts, unsigned n) {
    if (n > RL_LIDAR_MAX_POINTS) n = RL_LIDAR_MAX_POINTS;
    rl_write_header(buf, RL_TYPE_LIDAR, seq);
    uint8_t *p = buf + RL_HEADER_LEN;
    rl_put_u16(p, scan_speed_deg_s);
    p[2] = (uint8_t)n;
    p[3] = 0;
    p += RL_LIDAR_HDR_LEN;
    for (unsigned i = 0; i < n; i++) {
        rl_put_u16(p + 0, pts[i].angle_cdeg);
        rl_put_u16(p + 2, pts[i].dist_mm);
        p[4] = pts[i].intensity;
        p += RL_LIDAR_POINT_LEN;
    }
    return RL_LIDAR_PACKET_LEN(n);
}

/* Reads the lidar header; returns false if the length does not match count. */
static inline bool rl_decode_lidar_header(const uint8_t *buf, size_t len, RlLidarHeader *h) {
    if (len < RL_HEADER_LEN + RL_LIDAR_HDR_LEN) return false;
    const uint8_t *p = buf + RL_HEADER_LEN;
    h->scan_speed_deg_s = rl_get_u16(p);
    h->count = p[2];
    h->flags = p[3];
    if (h->count > RL_LIDAR_MAX_POINTS) return false;
    if (len < RL_LIDAR_PACKET_LEN(h->count)) return false;
    return true;
}

/* Reads point i (0-based) after a successful rl_decode_lidar_header. */
static inline void rl_decode_lidar_point(const uint8_t *buf, unsigned i, RlLidarPoint *pt) {
    const uint8_t *p = buf + RL_HEADER_LEN + RL_LIDAR_HDR_LEN + RL_LIDAR_POINT_LEN * i;
    pt->angle_cdeg = rl_get_u16(p + 0);
    pt->dist_mm    = rl_get_u16(p + 2);
    pt->intensity  = p[4];
}

/* ------------------------------------------------------------------- STATUS */
static inline size_t rl_encode_status(uint8_t *buf, uint16_t seq, const RlStatusPacket *s) {
    rl_write_header(buf, RL_TYPE_STATUS, seq);
    uint8_t *p = buf + RL_HEADER_LEN;
    rl_put_u32(p + 0,  s->uptime_ms);
    rl_put_u16(p + 4,  s->radar_fps);
    rl_put_u16(p + 6,  s->lidar_fps);
    rl_put_u16(p + 8,  s->lidar_pps);
    p[10] = s->subscribers;
    p[11] = s->flags;
    rl_put_u32(p + 12, s->radar_bad_frames);
    rl_put_u32(p + 16, s->lidar_crc_errors);
    rl_put_u32(p + 20, s->reserved);
    return RL_STATUS_PACKET_LEN;
}

static inline bool rl_decode_status(const uint8_t *buf, size_t len, RlStatusPacket *s) {
    if (len < RL_STATUS_PACKET_LEN) return false;
    const uint8_t *p = buf + RL_HEADER_LEN;
    s->uptime_ms        = rl_get_u32(p + 0);
    s->radar_fps        = rl_get_u16(p + 4);
    s->lidar_fps        = rl_get_u16(p + 6);
    s->lidar_pps        = rl_get_u16(p + 8);
    s->subscribers      = p[10];
    s->flags            = p[11];
    s->radar_bad_frames = rl_get_u32(p + 12);
    s->lidar_crc_errors = rl_get_u32(p + 16);
    s->reserved         = rl_get_u32(p + 20);
    return true;
}

/* -------------------------------------------------------------------- HELLO */
static inline size_t rl_encode_hello(uint8_t *buf, uint16_t seq, uint8_t receiver_kind, uint8_t wants) {
    rl_write_header(buf, RL_TYPE_HELLO, seq);
    uint8_t *p = buf + RL_HEADER_LEN;
    p[0] = receiver_kind;
    p[1] = wants;
    p[2] = 0;
    p[3] = 0;
    return RL_HELLO_PACKET_LEN;
}

static inline bool rl_decode_hello(const uint8_t *buf, size_t len, RlHelloPacket *h) {
    if (len < RL_HELLO_PACKET_LEN) return false;
    const uint8_t *p = buf + RL_HEADER_LEN;
    h->receiver_kind = p[0];
    h->wants         = p[1];
    h->reserved      = rl_get_u16(p + 2);
    return true;
}

#ifdef __cplusplus
}
#endif
#endif /* RADAR_PROTOCOL_H */
