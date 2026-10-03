/*
 * giga_display.ino — RadarLink display node (Arduino GIGA R1 WiFi + GIGA Display Shield)
 *
 * Joins the sensor node's Wi-Fi network as a client, announces itself with a
 * HELLO packet once a second and draws what the sensor node sends: a 360-degree
 * LIDAR outline of the room (LD19, 720 angle buckets) with the radar's moving
 * targets overlaid in the +/-60-degree sector in front of the sensor. Passive
 * buzzer pings speed up and rise in pitch as the closest target approaches.
 * Touch buttons: MUTE, ZOOM (4 / 8 / 12 m), LIDAR on/off.
 *
 * Changes versus the v1 receiver, all from the audit of that code:
 *   - client (STA) mode: the sensor node is the access point, so a second
 *     display (uConsole) can receive the same stream; boot order no longer matters
 *   - every pending UDP datagram is drained each loop (v1 read one per loop
 *     while each dot erase cost ~125k trig calls, so it fell behind and stayed behind)
 *   - whole frame redrawn with cheap primitives into the GFX canvas at ~15 fps,
 *     no per-pixel trig loops, no 10-second full-screen flicker
 *   - packets accepted only from the sensor node's address, sequence numbers checked
 *   - #include <WiFiUdp.h> with the case the core actually ships (v1 only
 *     compiled on case-insensitive file systems)
 *   - a failed Wi-Fi connection is shown on screen and retried, never silent
 *
 * Board: "Arduino GIGA R1 WiFi" from the Arduino Mbed OS GIGA Boards core 4.x.
 * Libraries: Arduino_GigaDisplay_GFX (+ Adafruit GFX Library), Arduino_GigaDisplayTouch.
 * Credits: v1 display, buzzer and touch handling by Sudo_solder (MIT).
 *
 * SPDX-License-Identifier: MIT
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

#include "config.h"
#include "radar_protocol.h"

/* ------------------------------------------------------------- constants */
#define SCREEN_W   800
#define SCREEN_H   480
static const int16_t PLOT_CX = 400;
static const int16_t PLOT_CY = 240;
static const int16_t PLOT_R  = 222;
static const int16_t HUD_LEFT_X  = 6;
static const int16_t HUD_RIGHT_X = 636;
static const int16_t HUD_RIGHT_W = 158;

/* RGB565 */
#define C_BG       0x0000
#define C_GRID     0x01E0
#define C_GRID_DIM 0x00C0
#define C_SECTOR   0x0320
#define C_GREEN    0x07E0
#define C_GDIM     0x02E0
#define C_RED      0xF800
#define C_AMBER    0xFD20
#define C_CYAN     0x07FF
#define C_WHITE    0xFFFF
#define C_GREY     0x8410
#define C_LIDAR0   0xAFE5   /* fresh point */
#define C_LIDAR1   0x5E04
#define C_LIDAR2   0x2C83   /* oldest */
#define C_BTN_ON   0x0320
#define C_BTN_OFF  0x4000

static const uint16_t SLOT_COLOUR[RL_RADAR_SLOTS] = {C_RED, C_AMBER, C_CYAN};
static const float    ZOOM_RANGES_M[] = ZOOM_LEVELS_M;
static const uint8_t  ZOOM_COUNT = sizeof(ZOOM_RANGES_M) / sizeof(ZOOM_RANGES_M[0]);
#define LIDAR_BUCKETS 720                     /* 0.5 degree each */

/* --------------------------------------------------------------- objects */
GigaDisplay_GFX          display;
Arduino_GigaDisplayTouch touch;
WiFiUDP                  udp;

/* -------------------------------------------------------------- link state */
static bool      linkUp = false;
static IPAddress senderIP;
static uint32_t  lastConnectAttemptMs = 0, lastHelloMs = 0, lastPacketMs = 0;
static uint16_t  helloSeq = 0;
static uint16_t  lastSeq[4];          /* indexed by packet type 1..3 */
static bool      haveSeq[4];
static uint8_t   oldStreak[4];
static uint32_t  rxPackets = 0, rxDropped = 0, rxForeign = 0, rxThisSec = 0, rxPerSec = 0;
static uint8_t   rxBuf[RL_MAX_PACKET];
static int       lastWifiResult = -1;

/* ----------------------------------------------------------------- lidar */
struct LidarBucket { uint16_t dist_mm; uint8_t intensity; uint32_t ts; };
static LidarBucket buckets[LIDAR_BUCKETS];
static float       bucketSin[LIDAR_BUCKETS], bucketCos[LIDAR_BUCKETS];
static bool        lidarVisible = LIDAR_SHOWN_AT_BOOT != 0;
static uint32_t    lastLidarMs = 0;

/* ----------------------------------------------------------------- radar */
static RlRadarPacket radar;
static uint32_t      lastRadarMs = 0;
struct Track { int16_t x, y; uint32_t stillSinceMs; bool have; };
static Track track[RL_RADAR_SLOTS];
static bool  targetShown[RL_RADAR_SLOTS];

/* ---------------------------------------------------------------- status */
static RlStatusPacket nodeStatus;
static bool           haveStatus = false;

/* -------------------------------------------------------------------- UI */
struct Button { int16_t x, y, w, h; };
static const Button BTN_LIDAR = {HUD_RIGHT_X, 324, HUD_RIGHT_W, 42};
static const Button BTN_MUTE  = {HUD_RIGHT_X, 376, HUD_RIGHT_W, 42};
static const Button BTN_ZOOM  = {HUD_RIGHT_X, 428, HUD_RIGHT_W, 42};
static uint8_t  zoomIdx = ZOOM_DEFAULT_INDEX;
static bool     buzzerMuted = BUZZER_MUTED_AT_BOOT != 0;
static bool     touchDownLast = false;
static uint32_t lastTouchToggleMs = 0;

/* ---------------------------------------------------------------- buzzer */
static uint32_t lastBeepMs = 0, beepStartMs = 0;
static bool     beepActive = false;

/* ---------------------------------------------------------------- timing */
static uint32_t lastFrameMs = 0, lastSecondMs = 0, lastDebugMs = 0, framesThisSec = 0, fps = 0;

/* ================================================================ helpers */
static inline bool alive(uint32_t now) { return linkUp && lastPacketMs && (now - lastPacketMs) < RL_LINK_TIMEOUT_MS; }
static inline float rangeM() { return ZOOM_RANGES_M[zoomIdx]; }
static inline float pxPerMm() { return (float)PLOT_R / (rangeM() * 1000.0f); }
static inline bool inButton(const Button &b, int16_t x, int16_t y) { return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h; }

static void resetSeqTracking() {
    for (int i = 0; i < 4; i++) { haveSeq[i] = false; oldStreak[i] = 0; lastSeq[i] = 0; }
}

static void buildBucketTables() {
    for (int i = 0; i < LIDAR_BUCKETS; i++) {
        float deg = (i + 0.5f) * (360.0f / LIDAR_BUCKETS) + LIDAR_ANGLE_OFFSET_DEG;
        if (!LIDAR_CLOCKWISE) deg = -deg;
        float rad = deg * PI / 180.0f;
        bucketSin[i] = sinf(rad);     /* screen x grows with sin, y shrinks with cos: 0 deg = up */
        bucketCos[i] = cosf(rad);
    }
}

/* ============================================================== Wi-Fi link */
static void drawConnectingScreen(const char *line2) {
    display.fillScreen(C_BG);
    display.setTextColor(C_GREEN); display.setTextSize(3);
    display.setCursor(150, 180); display.print("RadarLink display");
    display.setTextSize(2); display.setTextColor(C_GDIM);
    display.setCursor(150, 230); display.print("Joining \""); display.print(WIFI_SSID); display.print("\" ...");
    display.setCursor(150, 260); display.print(line2);
}

static void connectWifi(uint32_t now) {
    lastConnectAttemptMs = now;
    char buf[64];
    if (lastWifiResult < 0) snprintf(buf, sizeof buf, "first attempt");
    else snprintf(buf, sizeof buf, "last attempt failed (code %d), retrying", lastWifiResult);
    drawConnectingScreen(buf);
    Serial.print("[wifi] connecting to "); Serial.println(WIFI_SSID);

    WiFi.setTimeout(WIFI_CONNECT_TIMEOUT_MS);
    int st = WiFi.begin(WIFI_SSID, WIFI_PASS);     /* blocking, bounded by the timeout above */
    lastWifiResult = st;
    if (st != WL_CONNECTED) {
        Serial.print("[wifi] connect failed, status "); Serial.println(st);
        return;
    }
    senderIP = WiFi.gatewayIP();                    /* the sensor node is the AP */
    udp.stop();
    udp.begin(RL_PORT);
    resetSeqTracking();
    linkUp = true;
    lastPacketMs = now;                             /* grace period before "no data" */
    lastHelloMs = 0;                                /* send HELLO immediately */
    Serial.print("[wifi] connected, IP "); Serial.print(WiFi.localIP());
    Serial.print(", sensor node "); Serial.print(senderIP);
    Serial.print(", RSSI "); Serial.println(WiFi.RSSI());
}

static void checkLink(uint32_t now) {
    if (linkUp) {
        bool silent = (now - lastPacketMs) > WIFI_RECONNECT_AFTER_SILENCE_MS;
        if (WiFi.status() != WL_CONNECTED || silent) {
            Serial.println(silent ? "[wifi] no packets for a long time, reconnecting" : "[wifi] association lost, reconnecting");
            WiFi.disconnect();
            linkUp = false;
            lastConnectAttemptMs = now - WIFI_RETRY_MS;   /* retry on the next loop */
        }
        return;
    }
    if ((now - lastConnectAttemptMs) >= WIFI_RETRY_MS) connectWifi(now);
}

static void sendHello(uint32_t now) {
    if (!linkUp || (now - lastHelloMs) < RL_HELLO_INTERVAL_MS) return;
    lastHelloMs = now;
    uint8_t buf[RL_HELLO_PACKET_LEN];
    size_t n = rl_encode_hello(buf, helloSeq++, RL_RECEIVER_GIGA, RECEIVER_WANTS);
    udp.beginPacket(senderIP, RL_PORT);
    udp.write(buf, n);
    udp.endPacket();
}

/* ============================================================ packet input */
static bool acceptSeq(uint8_t type, uint16_t seq) {
    if (type > 3) return false;
    if (!haveSeq[type] || rl_seq_newer(seq, lastSeq[type])) {
        haveSeq[type] = true; lastSeq[type] = seq; oldStreak[type] = 0;
        return true;
    }
    /* Several "old" packets in a row means the sensor node rebooted and its
     * counters restarted: resynchronise instead of dropping for minutes. */
    if (++oldStreak[type] >= 5) { lastSeq[type] = seq; oldStreak[type] = 0; return true; }
    return false;
}

static void handleRadar(const uint8_t *buf, size_t len, uint32_t now) {
    RlRadarPacket p;
    if (!rl_decode_radar(buf, len, &p)) return;
    radar = p;
    lastRadarMs = now;
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        RlRadarTarget &t = radar.target[i];
        if (!t.valid) { track[i].have = false; continue; }
        bool moved = !track[i].have ||
                     abs(t.x_mm - track[i].x) > RADAR_MOVE_THRESH_MM ||
                     abs(t.y_mm - track[i].y) > RADAR_MOVE_THRESH_MM;
        if (moved) track[i].stillSinceMs = now;
        track[i].x = t.x_mm; track[i].y = t.y_mm; track[i].have = true;
    }
}

static void handleLidar(const uint8_t *buf, size_t len, uint32_t now) {
    RlLidarHeader h;
    if (!rl_decode_lidar_header(buf, len, &h)) return;
    lastLidarMs = now;
    for (unsigned i = 0; i < h.count; i++) {
        RlLidarPoint pt;
        rl_decode_lidar_point(buf, i, &pt);
        unsigned idx = (pt.angle_cdeg % 36000u) / (36000u / LIDAR_BUCKETS);
        buckets[idx].dist_mm = pt.dist_mm;
        buckets[idx].intensity = pt.intensity;
        buckets[idx].ts = now;
    }
}

static void receivePackets(uint32_t now) {
    if (!linkUp) return;
    int sz;
    while ((sz = udp.parsePacket()) > 0) {
        if (udp.remoteIP() != senderIP) { rxForeign++; continue; }
        int n = udp.read(rxBuf, sizeof rxBuf);
        if (n <= 0) continue;
        uint8_t type; uint16_t seq;
        if (!rl_read_header(rxBuf, (size_t)n, &type, &seq)) { rxForeign++; continue; }
        if (type != RL_TYPE_RADAR && type != RL_TYPE_LIDAR && type != RL_TYPE_STATUS) { rxForeign++; continue; }
        if (!acceptSeq(type, seq)) { rxDropped++; continue; }
        rxPackets++; rxThisSec++;
        lastPacketMs = now;
        switch (type) {
            case RL_TYPE_RADAR:  handleRadar(rxBuf, (size_t)n, now); break;
            case RL_TYPE_LIDAR:  handleLidar(rxBuf, (size_t)n, now); break;
            case RL_TYPE_STATUS: if (rl_decode_status(rxBuf, (size_t)n, &nodeStatus)) haveStatus = true; break;
            default: break;
        }
    }
}

/* ============================================================== radar logic */
static void updateTargets(uint32_t now) {
    bool hold = lastRadarMs && (now - lastRadarMs) < RADAR_HOLD_MS && alive(now);
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        targetShown[i] = false;
        if (!hold || !radar.target[i].valid) continue;
        if (RADAR_STILL_TIMEOUT_MS > 0 && track[i].have && (now - track[i].stillSinceMs) > (uint32_t)RADAR_STILL_TIMEOUT_MS) continue;
        targetShown[i] = true;
    }
}

static float closestShownTargetMm() {
    float best = -1.0f;
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        if (!targetShown[i]) continue;
        float d = sqrtf((float)radar.target[i].x_mm * radar.target[i].x_mm + (float)radar.target[i].y_mm * radar.target[i].y_mm);
        if (best < 0 || d < best) best = d;
    }
    return best;
}

static void updateBuzzer(uint32_t now) {
    if (buzzerMuted) { if (beepActive) { noTone(BUZZER_PIN); beepActive = false; } return; }
    if (beepActive && (now - beepStartMs) >= BEEP_DUR_MS) beepActive = false;
    float dist = closestShownTargetMm();
    if (dist < 0) return;
    float d = constrain(dist, BEEP_DIST_MIN_MM, BEEP_DIST_MAX_MM);
    float t = (d - BEEP_DIST_MIN_MM) / (BEEP_DIST_MAX_MM - BEEP_DIST_MIN_MM);     /* 0 near .. 1 far */
    uint32_t interval = BEEP_INTERVAL_MIN_MS + (uint32_t)(t * (BEEP_INTERVAL_MAX_MS - BEEP_INTERVAL_MIN_MS));
    uint16_t freq     = BEEP_FREQ_MAX_HZ - (uint16_t)(t * (BEEP_FREQ_MAX_HZ - BEEP_FREQ_MIN_HZ));
    if (!beepActive && (now - lastBeepMs) >= interval) {
        tone(BUZZER_PIN, freq, BEEP_DUR_MS);
        beepActive = true; beepStartMs = now; lastBeepMs = now;
    }
}

/* ================================================================= drawing */
static void drawButton(const Button &b, const char *label, bool on, uint16_t onColour) {
    display.fillRect(b.x, b.y, b.w, b.h, on ? C_BTN_ON : C_BTN_OFF);
    display.drawRect(b.x, b.y, b.w, b.h, on ? onColour : C_RED);
    display.setTextColor(C_WHITE); display.setTextSize(2);
    display.setCursor(b.x + 10, b.y + 13); display.print(label);
}

static void drawGrid() {
    float range = rangeM();
    float step  = range > 8.0f ? 2.0f : 1.0f;
    for (float m = step; m <= range + 0.01f; m += step) {
        int16_t r = (int16_t)(m * 1000.0f * pxPerMm());
        display.drawCircle(PLOT_CX, PLOT_CY, r, (fmodf(m, 2.0f * step) < 0.01f) ? C_GRID : C_GRID_DIM);
        display.setTextColor(C_GDIM); display.setTextSize(1);
        display.setCursor(PLOT_CX + 3, PLOT_CY - r - 9);
        display.print((int)m); display.print("m");
    }
    display.drawCircle(PLOT_CX, PLOT_CY, PLOT_R, C_GRID);
    display.drawFastHLine(PLOT_CX - PLOT_R, PLOT_CY, 2 * PLOT_R, C_GRID_DIM);
    display.drawFastVLine(PLOT_CX, PLOT_CY - PLOT_R, 2 * PLOT_R, C_GRID_DIM);

    /* radar field of view: +/-60 degrees forward, arc at the radar's maximum range */
    float rr = RADAR_MAX_RANGE_M * 1000.0f * pxPerMm();
    if (rr > PLOT_R) rr = PLOT_R;
    float a = RADAR_FOV_HALF_DEG * PI / 180.0f;
    display.drawLine(PLOT_CX, PLOT_CY, PLOT_CX + (int16_t)(rr * sinf(a)),  PLOT_CY - (int16_t)(rr * cosf(a)), C_SECTOR);
    display.drawLine(PLOT_CX, PLOT_CY, PLOT_CX - (int16_t)(rr * sinf(a)),  PLOT_CY - (int16_t)(rr * cosf(a)), C_SECTOR);
    int16_t px = PLOT_CX - (int16_t)(rr * sinf(a)), py = PLOT_CY - (int16_t)(rr * cosf(a));
    for (float deg = -RADAR_FOV_HALF_DEG + 5.0f; deg <= RADAR_FOV_HALF_DEG + 0.1f; deg += 5.0f) {
        float r2 = deg * PI / 180.0f;
        int16_t nx = PLOT_CX + (int16_t)(rr * sinf(r2)), ny = PLOT_CY - (int16_t)(rr * cosf(r2));
        display.drawLine(px, py, nx, ny, C_SECTOR);
        px = nx; py = ny;
    }
    display.fillCircle(PLOT_CX, PLOT_CY, 4, C_GREEN);
}

static void drawLidar(uint32_t now) {
    if (!lidarVisible) return;
    float k = pxPerMm();
    for (int i = 0; i < LIDAR_BUCKETS; i++) {
        const LidarBucket &b = buckets[i];
        if (!b.ts) continue;
        uint32_t age = now - b.ts;
        if (age > LIDAR_POINT_TTL_MS) continue;
        float r = b.dist_mm * k;
        if (r > PLOT_R) continue;
        int16_t x = PLOT_CX + (int16_t)(r * bucketSin[i]);
        int16_t y = PLOT_CY - (int16_t)(r * bucketCos[i]);
        uint16_t c = age < LIDAR_POINT_TTL_MS / 3 ? C_LIDAR0 : (age < 2 * LIDAR_POINT_TTL_MS / 3 ? C_LIDAR1 : C_LIDAR2);
        display.fillRect(x - 1, y - 1, 3, 3, c);
    }
}

static void drawTargets() {
    float k = pxPerMm();
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        if (!targetShown[i]) continue;
        const RlRadarTarget &t = radar.target[i];
        float xs = RADAR_MIRROR_X ? -(float)t.x_mm : (float)t.x_mm;
        int16_t x = PLOT_CX + (int16_t)(xs * k);
        int16_t y = PLOT_CY - (int16_t)((float)t.y_mm * k);
        uint16_t c = SLOT_COLOUR[i];
        display.fillCircle(x, y, 6, c);
        display.drawCircle(x, y, 13, c);
        display.drawCircle(x, y, 20, c);
        float d = sqrtf((float)t.x_mm * t.x_mm + (float)t.y_mm * t.y_mm) / 1000.0f;
        char buf[24];
        snprintf(buf, sizeof buf, "%.2fm", (double)d);
        display.setTextColor(C_WHITE); display.setTextSize(1);
        display.setCursor(x + 24, y - 8); display.print(buf);
        snprintf(buf, sizeof buf, "%dcm/s", (int)abs(t.speed_cm_s));
        display.setCursor(x + 24, y + 2); display.print(buf);
    }
}

static void hudLine(int16_t y, const char *text, uint16_t colour) {
    display.setTextColor(colour); display.setTextSize(1);
    display.setCursor(HUD_LEFT_X, y); display.print(text);
}

static void drawHud(uint32_t now) {
    char buf[40];
    bool dataOk = alive(now);
    display.setTextColor(C_GREEN); display.setTextSize(2);
    display.setCursor(HUD_LEFT_X, 6); display.print("RadarLink");

    snprintf(buf, sizeof buf, "WIFI  %s", linkUp ? "OK" : "----");       hudLine(32, buf, linkUp ? C_GREEN : C_RED);
    snprintf(buf, sizeof buf, "DATA  %s", dataOk ? "OK" : "----");       hudLine(44, buf, dataOk ? C_GREEN : C_RED);
    if (haveStatus && dataOk) {
        snprintf(buf, sizeof buf, "node  up %lus  subs %u", (unsigned long)(nodeStatus.uptime_ms / 1000), nodeStatus.subscribers);
        hudLine(56, buf, C_GDIM);
        bool rOk = nodeStatus.flags & RL_STATUS_RADAR_OK;
        snprintf(buf, sizeof buf, "radar %s %u fps bad %lu", rOk ? "OK " : "---", nodeStatus.radar_fps, (unsigned long)nodeStatus.radar_bad_frames);
        hudLine(68, buf, rOk ? C_GDIM : C_AMBER);
        if (nodeStatus.flags & RL_STATUS_LIDAR_ENABLED) {
            bool lOk = nodeStatus.flags & RL_STATUS_LIDAR_OK;
            snprintf(buf, sizeof buf, "lidar %s %u fps %u pps", lOk ? "OK " : "---", nodeStatus.lidar_fps, nodeStatus.lidar_pps);
            hudLine(80, buf, lOk ? C_GDIM : C_AMBER);
            snprintf(buf, sizeof buf, "      crc errors %lu", (unsigned long)nodeStatus.lidar_crc_errors);
            hudLine(92, buf, C_GDIM);
        } else {
            hudLine(80, "lidar not fitted", C_GDIM);
        }
    } else {
        hudLine(56, linkUp ? "waiting for sensor node" : "joining network...", C_AMBER);
    }
    snprintf(buf, sizeof buf, "rx %lu/s  drop %lu  fps %lu", (unsigned long)rxPerSec, (unsigned long)rxDropped, (unsigned long)fps);
    hudLine(108, buf, C_GDIM);
    if (linkUp) {
        snprintf(buf, sizeof buf, "rssi %ld dBm", (long)WiFi.RSSI());
        hudLine(120, buf, C_GDIM);
    }

    /* target list */
    int16_t y = 400;
    for (unsigned i = 0; i < RL_RADAR_SLOTS; i++) {
        if (!targetShown[i]) continue;
        const RlRadarTarget &t = radar.target[i];
        float d = sqrtf((float)t.x_mm * t.x_mm + (float)t.y_mm * t.y_mm) / 1000.0f;
        snprintf(buf, sizeof buf, "T%u %.2fm %dcm/s", i + 1, (double)d, (int)abs(t.speed_cm_s));
        display.setTextColor(SLOT_COLOUR[i]); display.setTextSize(2);
        display.setCursor(HUD_LEFT_X, y); display.print(buf);
        y += 20;
    }

    /* right column */
    display.setTextColor(C_GREEN); display.setTextSize(2);
    display.setCursor(HUD_RIGHT_X, 6); display.print("RANGE "); display.print((int)rangeM()); display.print("m");
    snprintf(buf, sizeof buf, "LIDAR %s", lidarVisible ? "ON" : "OFF");
    drawButton(BTN_LIDAR, buf, lidarVisible, C_GREEN);
    drawButton(BTN_MUTE, buzzerMuted ? "MUTED" : "SOUND ON", !buzzerMuted, C_GREEN);
    drawButton(BTN_ZOOM, "ZOOM", true, C_GREEN);

    if (!dataOk) {
        display.setTextColor(C_RED); display.setTextSize(2);
        display.setCursor(PLOT_CX - 42, PLOT_CY + PLOT_R - 20); display.print(linkUp ? "NO DATA" : "NO LINK");
    }
}

static void renderFrame(uint32_t now) {
    display.startBuffering();
    display.fillScreen(C_BG);
    drawGrid();
    drawLidar(now);
    drawTargets();
    drawHud(now);
    display.endBuffering();           /* one DMA2D copy to the panel per frame */
    framesThisSec++;
}

/* =================================================================== touch */
static void checkTouch(uint32_t now) {
    GDTpoint_t points[5];
    uint8_t contacts = touch.getTouchPoints(points);
    bool touching = contacts > 0;
    if (touching && !touchDownLast && (now - lastTouchToggleMs) > 300) {
        /* The controller reports native portrait coordinates (480x800); map to
         * the rotated 800x480 landscape frame the sketch draws in. */
        int16_t tx = (int16_t)points[0].y;
        int16_t ty = (int16_t)(480 - points[0].x);
        if (inButton(BTN_MUTE, tx, ty)) {
            buzzerMuted = !buzzerMuted; lastTouchToggleMs = now;
        } else if (inButton(BTN_ZOOM, tx, ty)) {
            zoomIdx = (uint8_t)((zoomIdx + 1) % ZOOM_COUNT); lastTouchToggleMs = now;
        } else if (inButton(BTN_LIDAR, tx, ty)) {
            lidarVisible = !lidarVisible; lastTouchToggleMs = now;
        }
    }
    touchDownLast = touching;
}

/* ============================================================= diagnostics */
static void printDebug(uint32_t now) {
    Serial.print("[stat] link="); Serial.print(linkUp ? "up" : "down");
    Serial.print(" data="); Serial.print(alive(now) ? "ok" : "none");
    Serial.print(" rx/s="); Serial.print(rxPerSec);
    Serial.print(" total="); Serial.print(rxPackets);
    Serial.print(" drop="); Serial.print(rxDropped);
    Serial.print(" foreign="); Serial.print(rxForeign);
    Serial.print(" fps="); Serial.print(fps);
    if (haveStatus) {
        Serial.print(" | node radar_fps="); Serial.print(nodeStatus.radar_fps);
        Serial.print(" lidar_fps="); Serial.print(nodeStatus.lidar_fps);
        Serial.print(" subs="); Serial.print(nodeStatus.subscribers);
    }
    Serial.println();
}

/* =================================================================== setup */
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);

    display.begin();
    display.setRotation(1);
    touch.begin();
    buildBucketTables();
    memset(buckets, 0, sizeof buckets);
    resetSeqTracking();

    drawConnectingScreen("starting");
    Serial.println("RadarLink GIGA display starting");
    lastFrameMs = lastSecondMs = lastDebugMs = millis();
}

/* ==================================================================== loop */
void loop() {
    uint32_t now = millis();

    checkLink(now);
    sendHello(now);
    receivePackets(now);
    updateTargets(now);
    updateBuzzer(now);
    checkTouch(now);

    if (linkUp && (now - lastFrameMs) >= FRAME_INTERVAL_MS) {
        lastFrameMs = now;
        renderFrame(now);
    }
    if ((now - lastSecondMs) >= 1000) {
        lastSecondMs = now;
        rxPerSec = rxThisSec; rxThisSec = 0;
        fps = framesThisSec; framesThisSec = 0;
    }
    if ((now - lastDebugMs) >= DEBUG_INTERVAL_MS) { lastDebugMs = now; printDebug(now); }
}
