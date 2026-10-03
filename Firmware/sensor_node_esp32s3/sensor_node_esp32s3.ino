/*
 * sensor_node_esp32s3.ino — RadarLink sensor node (Seeed XIAO ESP32-S3)
 *
 * Reads an Ai-Thinker RD-03D 24 GHz radar and, when ENABLE_LIDAR is set in
 * config.h, an LDROBOT LD19 360-degree 2D LIDAR. Hosts the Wi-Fi access point
 * and unicasts every measurement to each receiver (Arduino GIGA display,
 * uConsole viewer) that announces itself with a HELLO packet once a second.
 * Wire format: radar_protocol.h / docs/PROTOCOL.md.
 *
 * Design rules, all learned from the v1 firmware's weak spots:
 *   - nothing blocks: no delay() in loop(), the radar configuration sequence
 *     is a timed state machine, Wi-Fi comes up once and stays up (AP mode)
 *   - every datagram is encoded byte by byte (no raw structs on the wire)
 *   - lidar datagrams never exceed 508 bytes (the GIGA's UDP buffer)
 *   - diagnostics on the USB serial port every DEBUG_INTERVAL_MS, and in the
 *     STATUS packet so the displays can show them too
 *
 * Board: "XIAO_ESP32S3" (esp32 core 3.x), USB CDC On Boot = Enabled (default).
 * Credits: RD-03D handling after Sudo_solder's Arduino-ESP32-Radarproject,
 * LD19 handling after the same author's Lidar-Radar-combination-Raspberry (MIT).
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

#include "config.h"
#include "radar_protocol.h"
#include "rd03d_parser.h"
#if ENABLE_LIDAR
#include "ld19_parser.h"
#endif

static_assert(sizeof(WIFI_PASS) - 1 >= 8 && sizeof(WIFI_PASS) - 1 <= 63,
              "WIFI_PASS must be 8..63 characters (WPA2)");
static_assert(WIFI_MAX_CLIENTS >= 1 && WIFI_MAX_CLIENTS <= 10, "WIFI_MAX_CLIENTS out of range");
#if ENABLE_LIDAR
static_assert(LIDAR_BATCH_FRAMES * LD19_POINTS <= RL_LIDAR_MAX_POINTS,
              "LIDAR_BATCH_FRAMES too large for a 508-byte datagram");
#endif

/* ------------------------------------------------------------------ state */
HardwareSerial RadarSerial(1);
#if ENABLE_LIDAR
HardwareSerial LidarSerial(2);
#endif
WiFiUDP udp;

static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);
static bool     apUp = false;
static uint32_t apLastTryMs = 0;

static uint8_t  txBuf[RL_MAX_PACKET];
static uint16_t radarSeq = 0, statusSeq = 0;
#if ENABLE_LIDAR
static uint16_t lidarSeq = 0;
#endif

/* radar */
static Rd03dParser   radarParser;
static Rd03dFrame    radarFrame;
static RlRadarPacket radarPkt;
static const Rd03dOptions radarOpts = {RADAR_MIN_DIST_MM, RADAR_MAX_DIST_MM, RADAR_CLUSTER_DIST_MM, RADAR_FILTER_SENTINELS != 0};
static uint32_t radarFrameCount = 0, radarBytes = 0, radarFramesThisSec = 0, radarFps = 0;
static uint32_t lastRadarFrameMs = 0, radarReconfigs = 0;

enum RadarCfgState { CFG_IDLE, CFG_WAIT_BOOT, CFG_SENT_ENABLE, CFG_SENT_MULTI, CFG_SENT_END, CFG_DONE };
static RadarCfgState cfgState = CFG_IDLE;
static uint32_t cfgDueMs = 0;
static bool     radarConfigured = false;

/* lidar */
#if ENABLE_LIDAR
static Ld19Parser   lidarParser;
static Ld19Frame    lidarFrame;
static RlLidarPoint lidarBatch[RL_LIDAR_MAX_POINTS];
static unsigned     lidarBatchCount = 0, lidarBatchFrames = 0;
static uint32_t     lidarBatchStartMs = 0;
static uint16_t     lidarScanSpeed = 0;
static uint32_t     lidarBytes = 0, lidarFramesThisSec = 0, lidarPointsThisSec = 0, lidarFps = 0, lidarPps = 0;
static uint32_t     lastLidarFrameMs = 0;
#endif

/* subscribers */
struct Subscriber {
    IPAddress ip;
    uint16_t  port;
    uint8_t   kind;
    uint8_t   wants;
    uint32_t  lastHelloMs;
    bool      used;
};
static Subscriber subs[WIFI_MAX_CLIENTS];

/* timing */
static uint32_t lastStatusMs = 0, lastDebugMs = 0;

/* ------------------------------------------------------------ subscribers */
static uint8_t subscriberCount() {
    uint8_t n = 0;
    for (auto &s : subs) if (s.used) n++;
    return n;
}

static void subscriberHello(const IPAddress &ip, uint16_t port, const RlHelloPacket &h, uint32_t now) {
    int freeSlot = -1, oldest = 0;
    for (int i = 0; i < WIFI_MAX_CLIENTS; i++) {
        if (subs[i].used && subs[i].ip == ip && subs[i].port == port) {
            subs[i].lastHelloMs = now;
            subs[i].wants = h.wants;
            subs[i].kind = h.receiver_kind;
            return;
        }
        if (!subs[i].used && freeSlot < 0) freeSlot = i;
        if (subs[i].lastHelloMs < subs[oldest].lastHelloMs) oldest = i;
    }
    int slot = freeSlot >= 0 ? freeSlot : oldest;
    subs[slot] = {ip, port, h.receiver_kind, h.wants, now, true};
    Serial.printf("[subs] + %s:%u kind=%u wants=0x%02X (%u total)\n",
                  ip.toString().c_str(), port, h.receiver_kind, h.wants, subscriberCount());
}

static void subscriberExpire(uint32_t now) {
    for (auto &s : subs) {
        if (s.used && (now - s.lastHelloMs) > RL_SUBSCRIBER_TIMEOUT_MS) {
            Serial.printf("[subs] - %s:%u timed out\n", s.ip.toString().c_str(), s.port);
            s.used = false;
        }
    }
}

static void sendToSubscribers(const uint8_t *buf, size_t len, uint8_t wantMask) {
    for (auto &s : subs) {
        if (!s.used || !(s.wants & wantMask)) continue;
        udp.beginPacket(s.ip, s.port);
        udp.write(buf, len);
        udp.endPacket();
    }
}

static void pollHello(uint32_t now) {
    int sz;
    while ((sz = udp.parsePacket()) > 0) {
        uint8_t buf[64];
        int n = udp.read(buf, sizeof(buf));
        if (n <= 0) continue;
        uint8_t type; uint16_t seq;
        if (!rl_read_header(buf, (size_t)n, &type, &seq)) continue;
        if (type != RL_TYPE_HELLO) continue;
        RlHelloPacket h;
        if (!rl_decode_hello(buf, (size_t)n, &h)) continue;
        subscriberHello(udp.remoteIP(), udp.remotePort(), h, now);
    }
}

/* ------------------------------------------------------------------ radar */
static void drainRadar() { while (RadarSerial.available()) RadarSerial.read(); }

static void radarConfigStart(uint32_t now) {
    cfgState = CFG_WAIT_BOOT;
    cfgDueMs = now + 300;          /* let the module finish booting first */
    radarConfigured = false;
}

static bool radarConfiguring() { return cfgState != CFG_IDLE && cfgState != CFG_DONE; }

/* Non-blocking replacement for the v1 enable/multi/end sequence with 650 ms of delay(). */
static void radarConfigRun(uint32_t now) {
    if (!radarConfiguring() || (int32_t)(now - cfgDueMs) < 0) return;
    switch (cfgState) {
        case CFG_WAIT_BOOT:
            drainRadar();
            RadarSerial.write(RD03D_CMD_ENABLE_CONFIG, sizeof(RD03D_CMD_ENABLE_CONFIG));
            cfgState = CFG_SENT_ENABLE; cfgDueMs = now + 200;
            break;
        case CFG_SENT_ENABLE:
            drainRadar();
            RadarSerial.write(RD03D_CMD_MULTI_TARGET, sizeof(RD03D_CMD_MULTI_TARGET));
            cfgState = CFG_SENT_MULTI; cfgDueMs = now + 200;
            break;
        case CFG_SENT_MULTI:
            drainRadar();
            RadarSerial.write(RD03D_CMD_END_CONFIG, sizeof(RD03D_CMD_END_CONFIG));
            cfgState = CFG_SENT_END; cfgDueMs = now + 200;
            break;
        case CFG_SENT_END:
            drainRadar();
            radarParser.reset();
            radarConfigured = true;
            lastRadarFrameMs = now;          /* grace period before the silence check */
            cfgState = CFG_DONE;
            Serial.println("[radar] multi-target mode configured");
            break;
        default:
            break;
    }
}

static void readRadar(uint32_t now) {
    if (radarConfiguring()) { drainRadar(); return; }   /* ACK frames, not reports */
    while (RadarSerial.available()) {
        uint8_t c = (uint8_t)RadarSerial.read();
        radarBytes++;
        if (!radarParser.feed(c, radarFrame)) continue;
        rd03dProcess(radarFrame, radarOpts, radarPkt);
        radarPkt.frame_count = ++radarFrameCount;
        lastRadarFrameMs = now;
        radarFramesThisSec++;
        size_t n = rl_encode_radar(txBuf, radarSeq++, &radarPkt);
        sendToSubscribers(txBuf, n, RL_WANT_RADAR);
    }
    /* A radar that stops reporting is usually one that lost its mode after a
     * brown-out. Re-send the configuration, at most once per silence period. */
    if (radarConfigured && (now - lastRadarFrameMs) > RADAR_SILENCE_RECONFIG_MS) {
        radarReconfigs++;
        Serial.printf("[radar] no frames for %lu ms, re-sending configuration (#%lu)\n",
                      (unsigned long)(now - lastRadarFrameMs), (unsigned long)radarReconfigs);
        radarConfigStart(now);
    }
}

/* ------------------------------------------------------------------ lidar */
#if ENABLE_LIDAR
static void lidarFlush() {
    if (lidarBatchCount == 0) { lidarBatchFrames = 0; return; }
    size_t n = rl_encode_lidar(txBuf, lidarSeq++, lidarScanSpeed, lidarBatch, lidarBatchCount);
    sendToSubscribers(txBuf, n, RL_WANT_LIDAR);
    lidarPointsThisSec += lidarBatchCount;
    lidarBatchCount = 0;
    lidarBatchFrames = 0;
}

static void readLidar(uint32_t now) {
    while (LidarSerial.available()) {
        uint8_t c = (uint8_t)LidarSerial.read();
        lidarBytes++;
        if (!lidarParser.feed(c, lidarFrame)) continue;
        lastLidarFrameMs = now;
        lidarFramesThisSec++;
        lidarScanSpeed = lidarFrame.speed_deg_s;
        for (unsigned i = 0; i < LD19_POINTS; i++) {
            uint16_t d = lidarFrame.dist_mm[i];
            if (d < LIDAR_MIN_DIST_MM) continue;                 /* 0 = no return */
            if (lidarBatchCount == 0) lidarBatchStartMs = now;
            lidarBatch[lidarBatchCount].angle_cdeg = ld19PointAngleCdeg(lidarFrame.start_angle_cdeg, lidarFrame.end_angle_cdeg, i);
            lidarBatch[lidarBatchCount].dist_mm    = d;
            lidarBatch[lidarBatchCount].intensity  = lidarFrame.intensity[i];
            if (++lidarBatchCount >= RL_LIDAR_MAX_POINTS) lidarFlush();
        }
        if (++lidarBatchFrames >= LIDAR_BATCH_FRAMES) lidarFlush();
    }
    if (lidarBatchCount && (now - lidarBatchStartMs) >= LIDAR_BATCH_MAX_AGE_MS) lidarFlush();
}
#endif

/* ----------------------------------------------------------------- status */
static void sendStatus(uint32_t now) {
    RlStatusPacket s = {};
    s.uptime_ms   = now;
    s.radar_fps   = (uint16_t)radarFramesThisSec;
    s.subscribers = subscriberCount();
    s.flags       = 0;
    if (lastRadarFrameMs && (now - lastRadarFrameMs) < 1000) s.flags |= RL_STATUS_RADAR_OK;
    if (radarConfigured) s.flags |= RL_STATUS_RADAR_CONFIGURED;
    s.radar_bad_frames = radarParser.badFrames;
#if ENABLE_LIDAR
    s.lidar_fps = (uint16_t)lidarFramesThisSec;
    s.lidar_pps = (uint16_t)(lidarPointsThisSec > 65535 ? 65535 : lidarPointsThisSec);
    if (lastLidarFrameMs && (now - lastLidarFrameMs) < 1000) s.flags |= RL_STATUS_LIDAR_OK;
    s.flags |= RL_STATUS_LIDAR_ENABLED;
    s.lidar_crc_errors = lidarParser.crcErrors;
    lidarFps = lidarFramesThisSec; lidarPps = lidarPointsThisSec;
    lidarFramesThisSec = 0; lidarPointsThisSec = 0;
#endif
    radarFps = radarFramesThisSec;
    radarFramesThisSec = 0;

    size_t n = rl_encode_status(txBuf, statusSeq++, &s);
    sendToSubscribers(txBuf, n, RL_WANT_STATUS);
}

static void printDebug(uint32_t now) {
    Serial.printf("[stat] up=%lus ap=%s sta=%d subs=%u | radar fps=%lu bytes=%lu bad=%lu cfg=%d reconf=%lu",
                  (unsigned long)(now / 1000), apUp ? "up" : "DOWN", WiFi.softAPgetStationNum(), subscriberCount(),
                  (unsigned long)radarFps, (unsigned long)radarBytes, (unsigned long)radarParser.badFrames,
                  radarConfigured ? 1 : 0, (unsigned long)radarReconfigs);
#if ENABLE_LIDAR
    Serial.printf(" | lidar fps=%lu pps=%lu bytes=%lu crc=%lu speed=%u",
                  (unsigned long)lidarFps, (unsigned long)lidarPps, (unsigned long)lidarBytes,
                  (unsigned long)lidarParser.crcErrors, lidarScanSpeed);
#endif
    Serial.printf(" | heap=%lu\n", (unsigned long)ESP.getFreeHeap());
}

/* -------------------------------------------------------------------- LED */
static void updateLed(uint32_t now) {
    /* short blink every 2 s while serving receivers, every 0.5 s while alone */
    uint32_t period = subscriberCount() ? 2000 : 500;
    bool on = (now % period) < 60;
    digitalWrite(STATUS_LED_PIN, (STATUS_LED_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

/* ------------------------------------------------------------------ Wi-Fi */
static void onWifiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:    Serial.println("[wifi] station joined"); break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: Serial.println("[wifi] station left"); break;
        default: break;
    }
}

static void startAccessPoint(uint32_t now) {
    apLastTryMs = now;
    WiFi.mode(WIFI_AP);
    delay(100);                                   /* setup-time only; avoids auth failures on some modules */
    WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
    apUp = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, WIFI_MAX_CLIENTS);
    if (apUp) {
        esp_wifi_set_ps(WIFI_PS_NONE);            /* lowest latency; we are mains/LiPo powered anyway */
        Serial.printf("[wifi] AP \"%s\" up on channel %d, IP %s, UDP port %u\n",
                      WIFI_SSID, WIFI_CHANNEL, WiFi.softAPIP().toString().c_str(), (unsigned)RL_PORT);
    } else {
        Serial.println("[wifi] softAP() FAILED, retrying in 5 s");
    }
}

/* ------------------------------------------------------------------ setup */
void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    Serial.begin(DEBUG_BAUD);
    delay(200);                                   /* let USB CDC enumerate; never wait for a host */
    Serial.println();
    Serial.println("RadarLink sensor node — RD-03D" 
#if ENABLE_LIDAR
                   " + LD19"
#endif
                   );
    Serial.printf("radar UART1 rx=%d tx=%d @%d", RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD);
#if ENABLE_LIDAR
    Serial.printf(" | lidar UART2 rx=%d tx=%d @%d", LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUD);
#endif
    Serial.println();

    RadarSerial.setRxBufferSize(RADAR_RX_BUFFER);
    RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#if ENABLE_LIDAR
    LidarSerial.setRxBufferSize(LIDAR_RX_BUFFER);
    LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
#endif

    WiFi.onEvent(onWifiEvent);
    startAccessPoint(millis());
    udp.begin(RL_PORT);

    radarConfigStart(millis());
    lastStatusMs = lastDebugMs = millis();
    Serial.println("[boot] ready; waiting for receivers to say HELLO");
}

/* ------------------------------------------------------------------- loop */
void loop() {
    uint32_t now = millis();

    if (!apUp && (now - apLastTryMs) > 5000) startAccessPoint(now);

    pollHello(now);
    subscriberExpire(now);

    radarConfigRun(now);
    readRadar(now);
#if ENABLE_LIDAR
    readLidar(now);
#endif

    if ((now - lastStatusMs) >= 1000) { lastStatusMs = now; sendStatus(now); }
    if ((now - lastDebugMs) >= DEBUG_INTERVAL_MS) { lastDebugMs = now; printDebug(now); }
    updateLed(now);
}
