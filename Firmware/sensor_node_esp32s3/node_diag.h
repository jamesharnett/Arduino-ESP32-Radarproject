/*
 * node_diag.h — the sensor node diagnoses itself from its own counters (no Arduino dependency)
 *
 * The serial console's "diag" command turns the counters the sketch already
 * keeps into the verdicts a builder would otherwise look up in the tables of
 * docs/TESTING.md: "bytes but no frames" means the wrong baud rate, "no bytes"
 * means no power or the wrong pin, and so on. The same rules live in
 * tools/analyze_log.py for logs captured earlier. Host tests:
 * Firmware/tests/host/test_node_logic.cpp.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef NODE_DIAG_H
#define NODE_DIAG_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>

enum DiagLevel : uint8_t { DIAG_OK = 0, DIAG_INFO = 1, DIAG_WARN = 2, DIAG_FAIL = 3 };

/* Everything the verdicts are based on. Ages are milliseconds since the last
 * good frame; DIAG_NEVER means no frame has ever been decoded. */
#define DIAG_NEVER 0xFFFFFFFFu

struct DiagInput {
    uint32_t uptime_ms;
    /* radar */
    uint32_t radar_bytes;              /* raw bytes received on the radar UART     */
    uint32_t radar_frames;             /* frames with a valid header and tail      */
    uint32_t radar_bad_frames;         /* header seen, tail wrong                  */
    uint32_t radar_reconfigs;          /* mode re-sent after 5 s of silence        */
    uint32_t radar_fps;                /* frames in the last whole second          */
    uint32_t radar_last_frame_age_ms;
    bool     radar_configured;
    bool     radar_configuring;
    /* lidar */
    bool     lidar_enabled;
    uint32_t lidar_bytes;
    uint32_t lidar_frames;
    uint32_t lidar_crc_errors;
    uint32_t lidar_fps;
    uint32_t lidar_pps;
    uint32_t lidar_last_frame_age_ms;
    uint16_t lidar_speed_deg_s;
    /* wifi */
    bool     ap_up;
    uint8_t  stations;                 /* associated Wi-Fi clients                 */
    uint8_t  subscribers;              /* clients that sent a HELLO recently       */
    /* memory */
    uint32_t free_heap;
    uint32_t min_free_heap;
};

struct DiagLine {
    DiagLevel   level;
    const char *subsystem;             /* "radar" | "lidar" | "wifi" | "heap"      */
    char        text[160];
};

#define DIAG_MAX_LINES 8

/* Thresholds, shared with tools/analyze_log.py (keep them in step). */
#define DIAG_SETTLE_MS            3000   /* nothing is judged before this uptime   */
#define DIAG_RADAR_FPS_MIN        5      /* RD-03D reports 10..20 frames/s         */
#define DIAG_RADAR_FPS_MAX        40
#define DIAG_RADAR_BAD_PCT_MAX    10     /* bad frames as % of good ones           */
#define DIAG_RADAR_STALE_MS       1500
#define DIAG_LIDAR_FPS_MIN        300    /* LD19 streams ~375 frames/s regardless of speed */
#define DIAG_LIDAR_SPEED_MIN      2500   /* deg/s; 3600 = 10 Hz default            */
#define DIAG_LIDAR_SPEED_MAX      4700
#define DIAG_LIDAR_CRC_PCT_MAX    2
#define DIAG_LIDAR_PPS_LOW        1000   /* fewer points/s: most of the room out of range */
#define DIAG_LIDAR_STALE_MS       1500
#define DIAG_HEAP_MIN_BYTES       20000

static inline const char *diagLevelName(DiagLevel l) {
    switch (l) {
        case DIAG_OK:   return "OK  ";
        case DIAG_INFO: return "INFO";
        case DIAG_WARN: return "WARN";
        default:        return "FAIL";
    }
}

static inline unsigned diagPercent(uint32_t part, uint32_t whole) {
    if (whole == 0) return part ? 100u : 0u;
    uint64_t p = (uint64_t)part * 100u / whole;
    return p > 100u ? 100u : (unsigned)p;
}

static inline void diagEmit(DiagLine *out, size_t &n, size_t max, DiagLevel level, const char *subsystem, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));

static inline void diagEmit(DiagLine *out, size_t &n, size_t max, DiagLevel level, const char *subsystem, const char *fmt, ...) {
    if (n >= max) return;
    DiagLine &l = out[n++];
    l.level = level;
    l.subsystem = subsystem;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(l.text, sizeof(l.text), fmt, ap);
    va_end(ap);
}

static inline void diagRadar(const DiagInput &in, DiagLine *out, size_t &n, size_t max) {
    if (in.radar_configuring || in.uptime_ms < DIAG_SETTLE_MS) {
        diagEmit(out, n, max, DIAG_INFO, "radar", "still configuring the RD-03D; ask again in a few seconds");
        return;
    }
    if (in.radar_bytes == 0) {
        diagEmit(out, n, max, DIAG_FAIL, "radar", "no bytes received at all: RD-03D not powered (needs 5 V), its TX not on D0/GPIO1, or no common ground");
        return;
    }
    if (in.radar_frames == 0) {
        if (in.radar_bad_frames == 0) {
            if (in.radar_bytes < 60) {
                diagEmit(out, n, max, DIAG_INFO, "radar", "only %lu bytes so far, no frame yet; wait a second and ask again",
                         (unsigned long)in.radar_bytes);
                return;
            }
            diagEmit(out, n, max, DIAG_FAIL, "radar", "%lu bytes but no AA FF 03 00 header: wrong baud rate (must be 256000) or line noise; run 'raw radar' and decode it",
                     (unsigned long)in.radar_bytes);
            return;
        }
        diagEmit(out, n, max, DIAG_FAIL, "radar", "headers found but all %lu frames fail the 55 CC tail check: byte loss or an unexpected frame format; run 'raw radar'",
                 (unsigned long)in.radar_bad_frames);
        return;
    }
    unsigned badPct = diagPercent(in.radar_bad_frames, in.radar_frames);
    if (in.radar_last_frame_age_ms != DIAG_NEVER && in.radar_last_frame_age_ms > DIAG_RADAR_STALE_MS) {
        diagEmit(out, n, max, DIAG_WARN, "radar", "frames stopped %lu ms ago after %lu good ones (reconfigured %lu times): check the 5 V rail under load, the module may be browning out",
                 (unsigned long)in.radar_last_frame_age_ms, (unsigned long)in.radar_frames, (unsigned long)in.radar_reconfigs);
        return;
    }
    if (badPct > DIAG_RADAR_BAD_PCT_MAX) {
        diagEmit(out, n, max, DIAG_WARN, "radar", "%u%% bad frames (%lu of %lu): supply ripple or lost bytes; add 220 uF + 10 uF at the module and shorten the leads",
                 badPct, (unsigned long)in.radar_bad_frames, (unsigned long)in.radar_frames);
        return;
    }
    if (in.radar_fps < DIAG_RADAR_FPS_MIN || in.radar_fps > DIAG_RADAR_FPS_MAX) {
        diagEmit(out, n, max, DIAG_WARN, "radar", "frame rate %lu/s is outside the normal %d..%d: module in a different reporting mode or frames being lost",
                 (unsigned long)in.radar_fps, DIAG_RADAR_FPS_MIN, DIAG_RADAR_FPS_MAX);
        return;
    }
    if (in.radar_reconfigs > 0) {
        diagEmit(out, n, max, DIAG_WARN, "radar", "running (%lu frames/s, %u%% bad) but the module went silent %lu time(s) since boot: supply dips, check the boost converter",
                 (unsigned long)in.radar_fps, badPct, (unsigned long)in.radar_reconfigs);
        return;
    }
    diagEmit(out, n, max, DIAG_OK, "radar", "%lu frames/s, %lu frames, %u%% bad, multi-target mode %s",
             (unsigned long)in.radar_fps, (unsigned long)in.radar_frames, badPct, in.radar_configured ? "configured" : "NOT configured");
}

static inline void diagLidar(const DiagInput &in, DiagLine *out, size_t &n, size_t max) {
    if (!in.lidar_enabled) {
        diagEmit(out, n, max, DIAG_INFO, "lidar", "disabled in this build (ENABLE_LIDAR 0)");
        return;
    }
    if (in.uptime_ms < DIAG_SETTLE_MS) {
        diagEmit(out, n, max, DIAG_INFO, "lidar", "motor still spinning up; ask again in a few seconds");
        return;
    }
    if (in.lidar_bytes == 0) {
        diagEmit(out, n, max, DIAG_FAIL, "lidar", "no bytes received: LD19 not powered (5 V, 180 mA+), its TX not on D2/GPIO3, or no common ground. Is the motor turning?");
        return;
    }
    if (in.lidar_frames == 0) {
        if (in.lidar_crc_errors == 0) {
            diagEmit(out, n, max, DIAG_FAIL, "lidar", "%lu bytes but no 54 2C header: wrong baud rate (must be 230400) or another device on that pin; run 'raw lidar'",
                     (unsigned long)in.lidar_bytes);
            return;
        }
        diagEmit(out, n, max, DIAG_FAIL, "lidar", "headers found but all %lu frames fail the CRC: baud mismatch, lost bytes (host stalling the USB serial?) or noise; run 'raw lidar'",
                 (unsigned long)in.lidar_crc_errors);
        return;
    }
    unsigned crcPct = diagPercent(in.lidar_crc_errors, in.lidar_frames);
    if (in.lidar_last_frame_age_ms != DIAG_NEVER && in.lidar_last_frame_age_ms > DIAG_LIDAR_STALE_MS) {
        diagEmit(out, n, max, DIAG_WARN, "lidar", "frames stopped %lu ms ago after %lu good ones: power dropped or the motor stalled",
                 (unsigned long)in.lidar_last_frame_age_ms, (unsigned long)in.lidar_frames);
        return;
    }
    if (crcPct > DIAG_LIDAR_CRC_PCT_MAX) {
        diagEmit(out, n, max, DIAG_WARN, "lidar", "%u%% CRC errors (%lu of %lu frames): supply ripple or a long unshielded lead; add 220 uF at the sensor",
                 crcPct, (unsigned long)in.lidar_crc_errors, (unsigned long)in.lidar_frames);
        return;
    }
    if (in.lidar_speed_deg_s < DIAG_LIDAR_SPEED_MIN || in.lidar_speed_deg_s > DIAG_LIDAR_SPEED_MAX) {
        diagEmit(out, n, max, DIAG_WARN, "lidar", "scan speed %u deg/s is outside %d..%d (3600 = 10 Hz): leave the PWM pin open, check the 5 V supply",
                 (unsigned)in.lidar_speed_deg_s, DIAG_LIDAR_SPEED_MIN, DIAG_LIDAR_SPEED_MAX);
        return;
    }
    if (in.lidar_fps < DIAG_LIDAR_FPS_MIN) {
        diagEmit(out, n, max, DIAG_WARN, "lidar", "only %lu frames/s (expect ~375): frames are being lost, usually the UART buffer overflowing while USB serial output blocks",
                 (unsigned long)in.lidar_fps);
        return;
    }
    if (in.lidar_pps < DIAG_LIDAR_PPS_LOW) {
        diagEmit(out, n, max, DIAG_INFO, "lidar", "healthy stream (%lu frames/s, %u%% CRC errors) but only %lu returns/s: most directions are beyond range or absorb the laser",
                 (unsigned long)in.lidar_fps, crcPct, (unsigned long)in.lidar_pps);
        return;
    }
    diagEmit(out, n, max, DIAG_OK, "lidar", "%lu frames/s, %lu points/s, %u%% CRC errors, %u deg/s (%u.%u Hz)",
             (unsigned long)in.lidar_fps, (unsigned long)in.lidar_pps, crcPct, (unsigned)in.lidar_speed_deg_s,
             (unsigned)(in.lidar_speed_deg_s / 360), (unsigned)((in.lidar_speed_deg_s % 360) * 10 / 360));
}

static inline void diagWifi(const DiagInput &in, DiagLine *out, size_t &n, size_t max) {
    if (!in.ap_up) {
        diagEmit(out, n, max, DIAG_FAIL, "wifi", "access point is DOWN: softAP() failed; WIFI_PASS must be 8..63 characters, WIFI_CHANNEL 1..13");
        return;
    }
    if (in.stations == 0) {
        diagEmit(out, n, max, in.uptime_ms < 20000 ? DIAG_INFO : DIAG_WARN, "wifi",
                 "AP up, no receiver has joined: power the GIGA / connect the uConsole, check SSID and password, antenna plugged in?");
        return;
    }
    if (in.subscribers == 0) {
        diagEmit(out, n, max, DIAG_WARN, "wifi", "%u station(s) joined but none sent a HELLO on UDP 4210: receiver firmware not running RadarLink v2, or its UDP port differs",
                 (unsigned)in.stations);
        return;
    }
    diagEmit(out, n, max, DIAG_OK, "wifi", "AP up, %u station(s), %u subscriber(s) receiving data", (unsigned)in.stations, (unsigned)in.subscribers);
}

static inline void diagHeap(const DiagInput &in, DiagLine *out, size_t &n, size_t max) {
    if (in.min_free_heap < DIAG_HEAP_MIN_BYTES) {
        diagEmit(out, n, max, DIAG_WARN, "heap", "free heap dipped to %lu bytes (now %lu): memory is tight, report this",
                 (unsigned long)in.min_free_heap, (unsigned long)in.free_heap);
        return;
    }
    diagEmit(out, n, max, DIAG_INFO, "heap", "%lu KB free, lowest %lu KB since boot",
             (unsigned long)(in.free_heap / 1024), (unsigned long)(in.min_free_heap / 1024));
}

/* Fills `out` (at most `max` lines) and returns the number written. */
static inline size_t nodeDiagnose(const DiagInput &in, DiagLine *out, size_t max) {
    size_t n = 0;
    diagRadar(in, out, n, max);
    diagLidar(in, out, n, max);
    diagWifi(in, out, n, max);
    diagHeap(in, out, n, max);
    return n;
}

static inline DiagLevel diagWorst(const DiagLine *lines, size_t n) {
    DiagLevel w = DIAG_OK;
    for (size_t i = 0; i < n; i++) if (lines[i].level > w && lines[i].level != DIAG_INFO) w = lines[i].level;
    return w;
}

#endif /* NODE_DIAG_H */
