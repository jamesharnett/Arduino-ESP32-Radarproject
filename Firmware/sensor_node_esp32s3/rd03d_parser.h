/*
 * rd03d_parser.h — Ai-Thinker RD-03D frame parser and target filter (no Arduino dependency)
 *
 * Report frame (30 bytes, 256000 baud):
 *   AA FF 03 00 | target 1 (8) | target 2 (8) | target 3 (8) | 55 CC
 * Each target: int16 x, int16 y, int16 speed, uint16 distance resolution, all
 * little-endian, with the RD-03D's sign convention: bit 15 set = positive,
 * clear = negative, bits 0..14 = magnitude. x and y are millimetres, speed is
 * centimetres per second. An all-zero target is an empty slot.
 *
 * Datasheet example: 0E 03 B1 86 10 00 68 01 -> x = -782 mm, y = 1713 mm,
 * speed = -16 cm/s, resolution 360 mm.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef RD03D_PARSER_H
#define RD03D_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "radar_protocol.h"

#define RD03D_HEADER_LEN   4u
#define RD03D_PAYLOAD_LEN  24u
#define RD03D_TAIL_LEN     2u
#define RD03D_FRAME_LEN    (RD03D_HEADER_LEN + RD03D_PAYLOAD_LEN + RD03D_TAIL_LEN)

static const uint8_t RD03D_HEADER[RD03D_HEADER_LEN] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t RD03D_TAIL[RD03D_TAIL_LEN]     = {0x55, 0xCC};

/* Configuration commands: FD FC FB FA | len | cmd | value | 04 03 02 01 */
static const uint8_t RD03D_CMD_ENABLE_CONFIG[14] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
static const uint8_t RD03D_CMD_MULTI_TARGET[12]  = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01};
static const uint8_t RD03D_CMD_END_CONFIG[12]    = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};

struct Rd03dFrame {
    uint8_t payload[RD03D_PAYLOAD_LEN];
};

struct Rd03dOptions {
    int  minDistMm;        /* targets closer than this are noise            */
    int  maxDistMm;        /* ... and farther than this                     */
    int  clusterDistMm;    /* merge two returns closer than this; 0 = off   */
    bool filterSentinels;  /* drop |speed| 0 / 248 / 256 cm/s (noise slots) */
};

static inline int16_t rd03dDecode(uint16_t raw) {
    return (raw & 0x8000u) ? (int16_t)(raw & 0x7FFFu) : (int16_t)(-(int32_t)(raw & 0x7FFFu));
}

/* The RD-03D firmware reports these speeds in slots that hold no real target. */
static inline bool rd03dIsSentinelSpeed(int16_t s) {
    int a = s < 0 ? -s : s;
    return a == 0 || a == 248 || a == 256;
}

static inline float rd03dDistance(int16_t x, int16_t y) {
    return sqrtf((float)x * (float)x + (float)y * (float)y);
}

/* Byte-wise state machine. feed() returns true when `out` holds a frame with a valid tail. */
class Rd03dParser {
public:
    uint32_t frames = 0;
    uint32_t badFrames = 0;

    bool feed(uint8_t c, Rd03dFrame &out) {
        if (!_inFrame) {
            if (c == RD03D_HEADER[_hdrIdx]) {
                if (++_hdrIdx == RD03D_HEADER_LEN) { _hdrIdx = 0; _inFrame = true; _idx = 0; }
            } else {
                _hdrIdx = (c == RD03D_HEADER[0]) ? 1 : 0;
            }
            return false;
        }
        _buf[_idx++] = c;
        if (_idx < RD03D_PAYLOAD_LEN + RD03D_TAIL_LEN) return false;
        _inFrame = false;
        _idx = 0;
        if (_buf[RD03D_PAYLOAD_LEN] == RD03D_TAIL[0] && _buf[RD03D_PAYLOAD_LEN + 1] == RD03D_TAIL[1]) {
            memcpy(out.payload, _buf, RD03D_PAYLOAD_LEN);
            frames++;
            return true;
        }
        badFrames++;
        return false;
    }

    void reset() { _inFrame = false; _idx = 0; _hdrIdx = 0; }

private:
    uint8_t  _buf[RD03D_PAYLOAD_LEN + RD03D_TAIL_LEN];
    unsigned _idx = 0;
    unsigned _hdrIdx = 0;
    bool     _inFrame = false;
};

/* Decodes, gates, filters and clusters one frame into the wire-format target slots.
 * Does not touch out.frame_count. */
static inline void rd03dProcess(const Rd03dFrame &f, const Rd03dOptions &o, RlRadarPacket &out) {
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        const uint8_t *b = f.payload + i * 8;
        RlRadarTarget &t = out.target[i];
        t.x_mm = 0; t.y_mm = 0; t.speed_cm_s = 0; t.valid = 0;

        bool empty = true;
        for (unsigned j = 0; j < 8; j++) if (b[j]) { empty = false; break; }
        if (empty) continue;

        int16_t x   = rd03dDecode((uint16_t)(b[0] | (b[1] << 8)));
        int16_t y   = rd03dDecode((uint16_t)(b[2] | (b[3] << 8)));
        int16_t spd = rd03dDecode((uint16_t)(b[4] | (b[5] << 8)));
        float dist  = rd03dDistance(x, y);
        if (dist <= (float)o.minDistMm || dist > (float)o.maxDistMm) continue;
        if (o.filterSentinels && rd03dIsSentinelSpeed(spd)) continue;
        t.x_mm = x; t.y_mm = y; t.speed_cm_s = spd; t.valid = 1;
    }

    if (o.clusterDistMm <= 0) return;
    /* One person often appears as two returns (torso + swinging arm). Merge
     * pairs closer than clusterDistMm into the lower slot, keeping the speed
     * with the larger magnitude, which is what the alarm cares about. */
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        if (!out.target[i].valid) continue;
        for (unsigned j = i + 1; j < RL_RADAR_SLOTS; j++) {
            if (!out.target[j].valid) continue;
            float dx = (float)out.target[i].x_mm - (float)out.target[j].x_mm;
            float dy = (float)out.target[i].y_mm - (float)out.target[j].y_mm;
            if (sqrtf(dx * dx + dy * dy) >= (float)o.clusterDistMm) continue;
            out.target[i].x_mm = (int16_t)(((int32_t)out.target[i].x_mm + out.target[j].x_mm) / 2);
            out.target[i].y_mm = (int16_t)(((int32_t)out.target[i].y_mm + out.target[j].y_mm) / 2);
            int ai = out.target[i].speed_cm_s < 0 ? -out.target[i].speed_cm_s : out.target[i].speed_cm_s;
            int aj = out.target[j].speed_cm_s < 0 ? -out.target[j].speed_cm_s : out.target[j].speed_cm_s;
            if (aj > ai) out.target[i].speed_cm_s = out.target[j].speed_cm_s;
            out.target[j].x_mm = 0; out.target[j].y_mm = 0; out.target[j].speed_cm_s = 0; out.target[j].valid = 0;
        }
    }
}

#endif /* RD03D_PARSER_H */
