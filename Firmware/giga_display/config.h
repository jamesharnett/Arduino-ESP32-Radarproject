/*
 * config.h — build-time settings for the GIGA R1 display node
 *
 * Edit this file, not the sketch.
 */
#ifndef GIGA_DISPLAY_CONFIG_H
#define GIGA_DISPLAY_CONFIG_H

/* ------------------------------------------------------------------- Wi-Fi */
/* Must match Firmware/sensor_node_esp32s3/config.h. The sensor node hosts
 * the network; this display joins it as a client.                            */
#define WIFI_SSID                        "RadarSystem"
#define WIFI_PASS                        "change-me-before-use"
#define WIFI_CONNECT_TIMEOUT_MS          8000   /* one blocking connect attempt     */
#define WIFI_RETRY_MS                    3000   /* pause between attempts           */
#define WIFI_RECONNECT_AFTER_SILENCE_MS  15000  /* no packets at all -> reconnect   */

/* What this display asks the sensor node to send (RL_WANT_* bits). Clear the
 * LIDAR bit for a radar-only display.                                         */
#define RECEIVER_WANTS   (RL_WANT_RADAR | RL_WANT_LIDAR | RL_WANT_STATUS)

/* ------------------------------------------------------------------- LIDAR */
/* Rotate the point cloud so the sensor node's "forward" is at the top of the
 * screen. The LD19's zero angle is wherever its connector points; measure it
 * once (docs/TESTING.md, "Calibration") and put the result here.              */
#define LIDAR_ANGLE_OFFSET_DEG   0.0f
#define LIDAR_CLOCKWISE          1      /* LD19 angles increase clockwise from above */
#define LIDAR_POINT_TTL_MS       500    /* a bucket not refreshed for this long fades */
#define LIDAR_SHOWN_AT_BOOT      1

/* ------------------------------------------------------------------- radar */
#define RADAR_MIRROR_X           0      /* 1 if "walk left" moves the dot right  */
#define RADAR_HOLD_MS            400    /* keep showing the last targets this long */
#define RADAR_STILL_TIMEOUT_MS   2000   /* hide a target that stopped moving; 0 = off */
#define RADAR_MOVE_THRESH_MM     15
#define RADAR_FOV_HALF_DEG       60.0f  /* RD-03D field of view is +/-60 degrees */
#define RADAR_MAX_RANGE_M        8.0f

/* --------------------------------------------------------------------- view */
#define ZOOM_LEVELS_M            {4.0f, 8.0f, 12.0f}   /* tap ZOOM to cycle     */
#define ZOOM_DEFAULT_INDEX       1
#define FRAME_INTERVAL_MS        66     /* ~15 fps full redraw                   */

/* ------------------------------------------------------------------ buzzer */
/* Passive piezo on D9. Power it from 3V3: GIGA pins are not 5 V tolerant.      */
#define BUZZER_PIN               9
#define BUZZER_MUTED_AT_BOOT     0
#define BEEP_DUR_MS              70
#define BEEP_DIST_MIN_MM         300.0f /* fastest, highest ping                 */
#define BEEP_DIST_MAX_MM         8000.0f
#define BEEP_INTERVAL_MIN_MS     120
#define BEEP_INTERVAL_MAX_MS     900
#define BEEP_FREQ_MIN_HZ         700
#define BEEP_FREQ_MAX_HZ         1800

/* -------------------------------------------------------------- diagnostics */
#define DEBUG_INTERVAL_MS        2000

#endif /* GIGA_DISPLAY_CONFIG_H */
