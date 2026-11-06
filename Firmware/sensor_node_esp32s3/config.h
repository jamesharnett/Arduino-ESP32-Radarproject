/*
 * config.h — build-time settings for the sensor node (ESP32-S3)
 *
 * Edit this file, not the sketch. Everything a builder normally changes is here:
 * Wi-Fi credentials, which sensors are fitted, which pins they use.
 */
#ifndef SENSOR_NODE_CONFIG_H
#define SENSOR_NODE_CONFIG_H

/* ---------------------------------------------------------------- Wi-Fi AP */
/* The sensor node hosts the network. Both displays join it as clients.
 * CHANGE THE PASSWORD before you deploy: anyone who knows it can join the
 * network and inject fake targets. WPA2 requires 8 to 63 characters.          */
#define WIFI_SSID          "RadarSystem"
#define WIFI_PASS          "change-me-before-use"
#define WIFI_CHANNEL       6          /* 1..13; pick a quiet one in your area   */
#define WIFI_MAX_CLIENTS   4          /* GIGA + uConsole + two spare            */

/* ---------------------------------------------------------------- features */
#define ENABLE_LIDAR       1          /* 0 = radar-only node (build level 1)    */

/* ------------------------------------------------- pins (Seeed XIAO ESP32-S3) */
/* XIAO pad -> GPIO: D0=1 D1=2 D2=3 D3=4 D4=5 D5=6 D8=7 D9=8 D10=9.
 * D6/D7 (GPIO43/44) are UART0 and stay free for debugging.                    */
#define RADAR_RX_PIN       1          /* D0  <- RD-03D TX (OT1)                 */
#define RADAR_TX_PIN       2          /* D1  -> RD-03D RX                       */
#define LIDAR_RX_PIN       3          /* D2  <- LD19 TX                         */
#define LIDAR_TX_PIN       4          /* D3: UART2 TX, unused (the LD19 has no RX; tie its PWM pin to GND) */
#define STATUS_LED_PIN     LED_BUILTIN /* XIAO user LED, GPIO21, active LOW     */
#define STATUS_LED_ACTIVE_LOW 1

/* ------------------------------------------------------------------ serial */
#define RADAR_BAUD         256000
#define LIDAR_BAUD         230400
#define RADAR_RX_BUFFER    1024       /* bytes; default 256 overflows easily    */
#define LIDAR_RX_BUFFER    4096       /* 375 frames/s x 47 bytes                */
#define DEBUG_BAUD         115200
#define SERIAL_TX_BUFFER   4096       /* USB CDC transmit ring; default 256 drops multi-line replies */

/* ---------------------------------------------------------- radar handling */
#define RADAR_MIN_DIST_MM          100
#define RADAR_MAX_DIST_MM          8000
#define RADAR_CLUSTER_DIST_MM      1000   /* merge two returns closer than this; 0 = off */
#define RADAR_FILTER_SENTINELS     1      /* drop |speed| of 0, 248, 256 cm/s (RD-03D noise) */
#define RADAR_SILENCE_RECONFIG_MS  5000   /* re-send multi-target mode if no frames for this long */

/* ---------------------------------------------------------- lidar handling */
#define LIDAR_BATCH_FRAMES         8      /* 8 x 12 points = 96 = RL_LIDAR_MAX_POINTS */
#define LIDAR_BATCH_MAX_AGE_MS     40     /* flush a partial batch after this    */
#define LIDAR_MIN_DIST_MM          20     /* LD19 minimum range                  */

/* ------------------------------------------------------------- diagnostics */
#define DEBUG_INTERVAL_MS          2000   /* serial statistics period            */

/* Serial console "raw radar|lidar [N]": bytes are buffered in RAM while the
 * parsers keep running, then printed as hex a few lines per loop() pass.       */
#define RAW_CAPTURE_MAX            6144   /* buffer: 20 s of radar frames or 130 LD19 frames */
#define RAW_DEFAULT_RADAR_BYTES    600    /* 20 RD-03D frames (2 s)              */
#define RAW_DEFAULT_LIDAR_BYTES    4700   /* 100 LD19 frames (0.27 s)            */
#define RAW_CAPTURE_TIMEOUT_MS     3000   /* base timeout; 1.5x the healthy transfer time is added per request */
#define RAW_DUMP_BYTES_PER_LINE    32
#define RAW_DUMP_LINES_PER_LOOP    4

#endif /* SENSOR_NODE_CONFIG_H */
