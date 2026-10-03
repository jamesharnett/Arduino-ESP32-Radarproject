# RadarLink v1 — UDP wire protocol

Every node in the system speaks one protocol on one UDP port. The sensor node
(ESP32-S3 with the RD-03D radar and the optional LD19 LIDAR) is the Wi-Fi access
point and the only sender of measurement data. Receivers (the Arduino GIGA R1
display and the uConsole viewer) announce themselves with a `HELLO` once per
second and the sensor node unicasts every data packet to each receiver that has
said hello within the last five seconds. There is no broadcast: the original
author measured unreliable delivery of broadcast frames under LIDAR load, and
unicast frames are acknowledged and retransmitted by the Wi-Fi layer.

The canonical machine-readable definition is
`Firmware/sensor_node_esp32s3/radar_protocol.h`. The copy in
`Firmware/giga_display/` must stay byte-identical (`tools/check_protocol_sync.py`),
and `uconsole/radar_protocol.py` is the Python mirror, verified against the C
header by `uconsole/tests/test_protocol.py`.

## Conventions

* UDP port **4210** for everything, in both directions.
* All multi-byte integers are **little-endian**.
* Every field is written byte by byte. No struct is ever sent raw, so compiler
  padding can never change the format (the v1 firmware had exactly that trap:
  an unpacked 7-byte struct became 8 bytes on the wire and a third-party decoder
  written for 29 bytes misread targets 2 and 3).
* No datagram is longer than **508 bytes**, because that is the receive buffer
  of the Arduino Mbed `WiFiUDP` implementation on the GIGA R1. Anything longer
  would be truncated there without an error.
* Receivers accept data only from the sensor node's address (the Wi-Fi gateway)
  and drop packets whose sequence number is not newer than the last one seen for
  that type. Wi-Fi security is WPA2 with the pre-shared key in `config.h`; change
  it before deploying.

## Common header (8 bytes)

| Offset | Size | Field     | Value |
|-------:|-----:|-----------|-------|
| 0 | 4 | `magic`   | `0x314C4452`, the ASCII bytes `52 44 4C 31` ("RDL1") |
| 4 | 1 | `type`    | see below |
| 5 | 1 | `version` | `1` |
| 6 | 2 | `seq`     | per-type sequence counter of the sender, wraps at 65535 |

| Type | Name     | Direction            | Rate |
|-----:|----------|----------------------|------|
| `0x01` | `RADAR`  | sensor node → receivers | one per RD-03D frame, about 10 to 20 per second |
| `0x02` | `LIDAR`  | sensor node → receivers | about 47 per second with the LD19 at 10 Hz |
| `0x03` | `STATUS` | sensor node → receivers | 1 per second |
| `0x10` | `HELLO`  | receiver → sensor node  | 1 per second |

## RADAR (type 0x01), 36 bytes

| Offset | Size | Field |
|-------:|-----:|-------|
| 8  | 4 | `frame_count`, RD-03D frames parsed since boot |
| 12 | 24 | three target slots of 8 bytes each (see below) |

Each slot:

| Offset in slot | Size | Field |
|-------:|-----:|-------|
| 0 | 2 | `x_mm`, `int16`, lateral position, positive to the right of the radar's forward axis |
| 2 | 2 | `y_mm`, `int16`, forward distance |
| 4 | 2 | `speed_cm_s`, `int16`, radial speed with the RD-03D's sign convention |
| 6 | 1 | `valid`, `1` if the slot holds a target, otherwise `0` and the other fields are `0` |
| 7 | 1 | reserved, `0` |

The slot index is stable while the RD-03D keeps tracking the same person, which
is why receivers colour targets by slot. The sensor node has already applied the
range gate (100 mm to 8000 mm), the sentinel-speed filter (`0`, `±248`, `±256`
cm/s are noise in the RD-03D firmware) and the clustering of two returns closer
than 1 m into one target.

## LIDAR (type 0x02), 12 + 5·N bytes, N ≤ 96

| Offset | Size | Field |
|-------:|-----:|-------|
| 8  | 2 | `scan_speed_deg_s`, motor speed reported by the LD19 |
| 10 | 1 | `count` N, number of points that follow, at most 96 |
| 11 | 1 | `flags`, reserved, `0` |
| 12 | 5·N | points |

Each point:

| Offset in point | Size | Field |
|-------:|-----:|-------|
| 0 | 2 | `angle_cdeg`, `uint16`, 0 to 35999, hundredths of a degree, increasing clockwise seen from above, in the LD19's own reference |
| 2 | 2 | `dist_mm`, `uint16`, never `0` (points without a return are dropped at the sensor node) |
| 4 | 1 | `intensity` |

A packet carries up to eight LD19 frames of twelve points. The receivers apply
the mounting offset `LIDAR_ANGLE_OFFSET_DEG` from their own configuration, so
the sensor node never needs recalibrating for a different display.

## STATUS (type 0x03), 32 bytes

| Offset | Size | Field |
|-------:|-----:|-------|
| 8  | 4 | `uptime_ms` |
| 12 | 2 | `radar_fps`, RD-03D frames parsed in the last second |
| 14 | 2 | `lidar_fps`, LD19 frames parsed in the last second |
| 16 | 2 | `lidar_pps`, LIDAR points sent in the last second |
| 18 | 1 | `subscribers`, receivers currently registered |
| 19 | 1 | `flags`: bit 0 radar frames seen in the last second, bit 1 lidar frames seen, bit 2 radar multi-target mode configured, bit 3 firmware built with LIDAR support |
| 20 | 4 | `radar_bad_frames`, cumulative frames with a bad tail |
| 24 | 4 | `lidar_crc_errors`, cumulative |
| 28 | 4 | reserved |

Receivers show these numbers in their HUD. They make the difference between
"the radar is not wired" and "the Wi-Fi link is down" visible at a glance.

## HELLO (type 0x10), 12 bytes

| Offset | Size | Field |
|-------:|-----:|-------|
| 8  | 1 | `receiver_kind`: `1` GIGA display, `2` uConsole, `3` other |
| 9  | 1 | `wants` bitmask: bit 0 radar, bit 1 lidar, bit 2 status |
| 10 | 2 | reserved |

The sensor node records the source address and port of each `HELLO` and sends
to that address until no `HELLO` has arrived for 5 s. Up to four receivers are
served at once. A receiver that is only interested in radar can clear bit 1 and
save the LIDAR bandwidth.

## Timing constants

| Constant | Value | Meaning |
|---|---:|---|
| `RL_HELLO_INTERVAL_MS` | 1000 | receivers send `HELLO` this often |
| `RL_SUBSCRIBER_TIMEOUT_MS` | 5000 | sensor node forgets a silent receiver |
| `RL_LINK_TIMEOUT_MS` | 3000 | receiver declares the link dead and clears the screen |

## Worked example

A radar packet with one target at x = −782 mm, y = 1713 mm, −16 cm/s in slot 0,
sequence 7, frame 1234:

```
52 44 4C 31  01 01  07 00   | magic "RDL1", type RADAR, version 1, seq 7
D2 04 00 00                  | frame_count 1234
F2 FC  B1 06  F0 FF  01 00   | slot 0: x=-782 y=1713 speed=-16 valid=1 pad
00 00  00 00  00 00  00 00   | slot 1 empty
00 00  00 00  00 00  00 00   | slot 2 empty
```
