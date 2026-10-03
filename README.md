# RadarLink — ESP32 radar + LIDAR sensor node with an Arduino GIGA display and a uConsole viewer

A wireless motion-tracking and room-outline system for remote surveillance. One
battery-powered **sensor node** (Seeed XIAO ESP32-S3) reads an Ai-Thinker
**RD-03D 24 GHz radar** and, optionally, an LDROBOT **LD19 360° LIDAR**, hosts
its own Wi-Fi network and streams the measurements to any number of displays:
an **Arduino GIGA R1 WiFi with the GIGA Display Shield** and a **ClockworkPi
uConsole** (or any Linux laptop) running the Python viewer, at the same time.

This is a merged and reworked descendant of three projects by Sudo_solder
(GitHub `Stevee87`): the original GIGA radar display, its uConsole variant and
the LD19 + radar uConsole tracker. See *Credits* at the end.

-------------------------------------------------------------------------------

## Safety and disclaimer

This is a hobby project, shared as-is for anyone who wants to build their own.

* **LiPo batteries.** Incorrect wiring, reverse polarity or physical damage to a
  LiPo cell can cause fire. Double-check polarity before powering anything on,
  use a charging module rated for the cell, and never leave a charging LiPo
  unattended.
* **RF hardware.** The RD-03D (24.0–24.25 GHz ISM) and the Wi-Fi modules operate
  under their respective regulatory approvals for their intended region. Verify
  this applies where you are before use.
* **Laser.** The LD19 is a Class 1 laser product. Do not open it or stare into
  the window at close range.
* **Surveillance and privacy.** Monitoring areas used by other people can fall
  under local privacy or surveillance law even without a camera. Know your rules.
* **No warranty.** Firmware, wiring, enclosures and this README are provided
  without any warranty. Verify every connection yourself; the diagrams and
  code may contain mistakes. Building and operating this project is entirely
  your own responsibility.

If you spot an error, please open an issue.

-------------------------------------------------------------------------------

## What it does, honestly

| | RD-03D radar | LD19 LIDAR |
|---|---|---|
| Measures | X/Y position and radial speed of up to **3 moving** people | distance to the nearest surface in **one horizontal plane**, 360°, ~4500 points/s |
| Range | 8 m rated, expect 5–7 m indoors | 12 m |
| Field of view | ±60° in front of the module | 360° |
| Sees stationary people | **no** (Doppler only; the display hides a target that stops for 2 s) | yes, as part of the room outline |
| Through thin walls or doors | partly, with much reduced range | no, line of sight only |
| Imaging | none, three dots | flat room outline, no height |

Neither sensor is 3D. The result is a top-down plan view: the LIDAR draws the
walls and furniture around the node, the radar marks who is moving where and how
fast. A real 3D LIDAR (Unitree L2, Livox Mid-360 class) cannot pass through an
ESP32 Wi-Fi link or be rendered by a GIGA; it would plug into the uConsole
directly. That is the planned third stage, see *Roadmap*.

Two build levels share the same firmware and displays:

* **Level 1, radar only.** Set `ENABLE_LIDAR 0` in the sensor node's `config.h`.
  This is the original project's function with two displays and the fixes below.
* **Level 2, radar + LIDAR.** Add the LD19 and keep `ENABLE_LIDAR 1` (default).

-------------------------------------------------------------------------------

## Architecture

```
                 sensor node (XIAO ESP32-S3, Wi-Fi access point "RadarSystem")
                 ┌──────────────────────────────────────────────────────────┐
 RD-03D ──UART1──▶  parse, gate, filter, cluster  ──▶ RADAR  36 B, ~20/s   │
 LD19   ──UART2──▶  parse, CRC, batch ≤96 points  ──▶ LIDAR  ≤492 B, ~47/s │──▶ unicast UDP :4210
                 │  1 Hz health counters            ──▶ STATUS 32 B, 1/s     │    to every receiver
                 └──────────────────────────────────────────────────────────┘    that said HELLO
                                 ▲ HELLO 12 B, 1/s                 ▲ HELLO
                                 │                                  │
                ┌────────────────┴──────────────┐   ┌───────────────┴───────────────┐
                │ Arduino GIGA R1 WiFi + Display │   │ uConsole (Linux, pygame)      │
                │ 360° plan view, radar sector,  │   │ same view, sonar audio,       │
                │ buzzer pings, touch buttons    │   │ record/replay, zoom           │
                └───────────────────────────────┘   └───────────────────────────────┘
```

* The sensor node is the access point, so it never depends on a display being
  on first. Displays join, send `HELLO` once a second and are served while they
  keep doing so. Boot order no longer matters.
* Everything is unicast. The original author found broadcast unreliable at
  LIDAR packet rates, and unicast frames are acknowledged by the Wi-Fi layer.
* The wire format is documented in [docs/PROTOCOL.md](docs/PROTOCOL.md) and
  implemented once in `radar_protocol.h`, mirrored in `uconsole/radar_protocol.py`
  and cross-checked by tests.

### Repository layout

```
Firmware/sensor_node_esp32s3/   ESP32-S3 sketch: config.h, radar_protocol.h, rd03d_parser.h, ld19_parser.h
Firmware/giga_display/          GIGA R1 sketch: config.h, radar_protocol.h (identical copy)
Firmware/tests/host/            parser unit tests that run on your PC with g++
Firmware/legacy/                the original v1 sketches, reference only (incompatible protocol)
uconsole/                       radar_viewer.py, radar_protocol.py, radar_state.py, fake_sensor_node.py, tests/
docs/PROTOCOL.md                wire format
docs/TESTING.md                 bring-up, expected serial output, calibration, troubleshooting
tools/check_protocol_sync.py    fails if the two radar_protocol.h copies differ
CAD files/                      3D-printable enclosures from the v1 build (see Enclosures)
```

-------------------------------------------------------------------------------

## Bill of materials

Approximate 2026 street prices in USD before shipping. Everything is a common
hobby part; the exact vendor does not matter.

### Sensor node (Level 1: radar only)

| Qty | Part | ~USD | Notes |
|---:|---|---:|---|
| 1 | Seeed Studio XIAO ESP32-S3 (plain, not Sense) | 7.5 | attach the included u.FL antenna |
| 1 | Ai-Thinker RD-03D 24 GHz radar module | 7 | 5 V supply, 3.3 V UART, 1.25 mm 4-pin cable usually included |
| 1 | MT3608 step-up converter module | 1.5 | trimmed to 5.0 V, feeds the sensors only |
| 1 | TP4056/BW4056 USB-C charger module with protection (DW01) | 1 | one per cell |
| 1 | 3.7 V LiPo, 1300 mAh (602560 fits the v1 case) | 8 | about 4 h radar-only |
| 1 | Mini toggle switch | 1.2 | |
| 1 | 1S Li-ion 4-LED battery indicator board | 2 | optional, wires across the cell |
| 1 | 220 µF electrolytic + 10 µF ceramic | 0.5 | decoupling at the sensors' 5 V |
| – | Wire, heat-set inserts, M3×10 screws, magnets, filament | 10 | |
| | **Subtotal** | **~40** | |

### Sensor node additions for Level 2 (radar + LIDAR)

| Qty | Part | ~USD | Notes |
|---:|---|---:|---|
| 1 | LDROBOT LD19 / D300 / D500 kit (same sensor family) | 70–105 | 5 V, 180 mA typical (300 mA at start), 3.3 V UART TX |
| 1 | Larger LiPo, ≥ 3000 mAh (e.g. 2 × 18650 in parallel or a 103450 pouch) | 10–15 | about 4 h with both sensors; the 1300 mAh cell gives under 2 h |

### GIGA display unit

| Qty | Part | ~USD | Notes |
|---:|---|---:|---|
| 1 | Arduino GIGA R1 WiFi | 73–80 | attach the included u.FL antenna; no antenna on the board |
| 1 | Arduino GIGA Display Shield | 64 | 800×480 touch |
| 1 | 3.7 V LiPo ≥ 5000 mAh (1160100 fits the v1 case) | 18 | about 6 h |
| 1 | TP4056/BW4056 USB-C charger module | 1 | |
| 1 | 5 V boost or power-bank module (see power options) | 2–5 | or a second MT3608 set to 7.0–7.5 V |
| 1 | Passive piezo buzzer module, 3-pin | 2 | passive, not active |
| 1 | Toggle switch, battery indicator, 220 µF capacitor | 4 | |
| – | Enclosure prints, inserts, screws, magnets | 10 | |
| | **Subtotal** | **~180** | |

### uConsole viewer

| Qty | Part | ~USD | Notes |
|---:|---|---:|---|
| 1 | ClockworkPi uConsole with a **wireless** Raspberry Pi core (CM4 kit with CM4104000, or CM5 Lite wireless) | ~250 | a non-wireless CM4/CM5 has no Wi-Fi in the uConsole; expect months of lead time |
| 2 | 18650 cells | 15 | not included with the kit |
| 1 | microSD 32 GB+ if your core has no eMMC, USB-C charger | 15 | |
| 1 | 4G module (optional) | 45–50 | only for viewing from outside Wi-Fi range (Roadmap) |

Nothing is wired to the uConsole; it only needs its Wi-Fi radio. Any Linux
laptop with Wi-Fi and Python 3.9+ runs the same viewer.

-------------------------------------------------------------------------------

## Wiring

### Sensor node

```
 LiPo ──▶ BW4056 B+/B- ──▶ OUT+/OUT- ──▶ toggle switch ──┬──▶ XIAO BAT+ / BAT- pads (back of the board)
                                                          │
                                                          └──▶ MT3608 IN ──▶ OUT trimmed to 5.0 V ──┬──▶ RD-03D VCC
                                                                 (220 µF + 10 µF across OUT)        └──▶ LD19 P5V
 All grounds meet at the BW4056 OUT- (the protection IC switches the negative rail).
```

Trim the MT3608 to 5.0 V with a meter **before** connecting the sensors.

| Signal | XIAO pin | GPIO | Notes |
|---|---|---:|---|
| RD-03D TX (OT1) | D0 | 1 | UART1 RX, 256000 baud |
| RD-03D RX | D1 | 2 | UART1 TX, carries the multi-target command |
| RD-03D VCC | – | – | 5.0 V from the MT3608, **not** from the XIAO's 5V pin |
| RD-03D GND | GND | – | |
| LD19 TX | D2 | 3 | UART2 RX, 230400 baud |
| (LD19 has no RX) | D3 | 4 | reserved as UART2 TX, leave unconnected |
| LD19 PWM | – | – | leave unconnected: the LD19 then regulates its own motor at 10 Hz |
| LD19 P5V | – | – | 5.0 V from the MT3608 |
| LD19 GND | GND | – | |

LD19 connector, as printed on the sensor: Tx, PWM, GND, P5V. Both sensors use
3.3 V logic on their UART, so no level shifting is needed. Pins D6/D7 (GPIO43/44)
are UART0 and stay free for debugging.

Why this differs from the v1 wiring: the v1 README fed the 5 V boost output
into the XIAO's `BAT+` pad, which Seeed rates for a 4.2 V cell, and then also
expected the XIAO to charge that cell over USB, which cannot work through a
boost converter. The cell now sits directly on the BAT pads as designed and the
boost feeds only the two 5 V sensors.

Charging: either through the BW4056's USB-C (1 A, set its Rprog to ~2 kΩ for a
1300 mAh cell) or through the XIAO's USB-C (100 mA, slow). Never both at once.
Switch the battery off while flashing over USB.

Antenna: plug the XIAO's u.FL antenna in and route it away from the LiPo and the
magnets. Without it the Wi-Fi range is a few metres at best. Keep a plastic,
metal-free window in front of the RD-03D and the LD19's optical ring clear all
round.

### GIGA display

Power, pick one:

* **A (simplest):** LiPo → BW4056 → switch → 5 V boost / power-bank module →
  the GIGA's **USB-C** port. Avoids the VIN minimum and any trimming.
* **B:** LiPo → BW4056 → switch → MT3608 trimmed to **7.0–7.5 V** (not the 6.0 V
  of the v1 README: 6 V is the bottom of the 6–24 V VIN range and browns out as
  the cell sags) → GIGA `VIN`, with 220 µF across the converter output.

| Buzzer module | GIGA R1 |
|---|---|
| VCC | **3V3** (GIGA pins are not 5 V tolerant; many 3-pin modules pull the signal line up to VCC) |
| I/O | D9 |
| GND | GND |

Use a **passive** piezo: an active buzzer has its own oscillator and ignores
the frequency the sketch sends. Plug in the GIGA's u.FL antenna. The Display
Shield mounts on the GIGA's headers; the battery indicator wires across the cell.

### uConsole

Nothing to wire.

-------------------------------------------------------------------------------

## Software

### Toolchain versions (pin them)

| Target | Core / package | Version | Notes |
|---|---|---|---|
| Sensor node | `esp32` by Espressif (Boards Manager URL `https://espressif.github.io/arduino-esp32/package_esp32_index.json`) | 3.3.x | board **XIAO_ESP32S3**, *USB CDC On Boot: Enabled* (default) |
| GIGA display | **Arduino Mbed OS GIGA Boards** | 4.6.x | not the Zephyr-based GIGA core: different Wi-Fi API |
| GIGA display | Arduino_GigaDisplay_GFX, Arduino_GigaDisplayTouch | 1.2.0, 1.1.2 | pulls in Adafruit GFX Library and Adafruit BusIO |
| uConsole | Python 3, pygame ≥ 2.1, numpy (audio only) | | |

Both sketch folders contain a `sketch.yaml` with these pins for `arduino-cli`.

### 1. Flash the sensor node

1. Edit `Firmware/sensor_node_esp32s3/config.h`: set `WIFI_SSID` and a new
   `WIFI_PASS` (8–63 characters; the shipped value is a placeholder and the
   build refuses passwords shorter than 8). Set `ENABLE_LIDAR 0` for Level 1.
2. Open `Firmware/sensor_node_esp32s3/sensor_node_esp32s3.ino`, select
   *XIAO_ESP32S3*, upload. Or: `arduino-cli compile --profile xiao_esp32s3 -u -p /dev/ttyACM0 Firmware/sensor_node_esp32s3`.
3. Open the serial monitor at 115200. You should see the AP come up and, every
   2 s, a `[stat]` line with radar (and lidar) frame rates. Expected values are
   in [docs/TESTING.md](docs/TESTING.md).

### 2. Flash the GIGA display

1. If this GIGA has never run Wi-Fi: *File → Examples → STM32H747_System →
   WiFiFirmwareUpdater*, upload once, wait for it to finish.
2. Edit `Firmware/giga_display/config.h`: same `WIFI_SSID`/`WIFI_PASS` as the
   node. Leave `LIDAR_ANGLE_OFFSET_DEG` at 0 for now.
3. Open `Firmware/giga_display/giga_display.ino`, board *Arduino GIGA R1 WiFi*
   (Mbed core), upload. The screen shows the join attempts; a failed attempt
   is printed with its status code and retried every 3 s.

### 3. Set up the uConsole

```bash
# join the sensor node's network and keep any LTE/other uplink as default route
nmcli dev wifi connect RadarSystem password 'your-password'
nmcli con modify RadarSystem ipv4.never-default yes ipv4.ignore-auto-dns yes 802-11-wireless.powersave 2
# (powersave 2 = off: a sleeping Wi-Fi client delays small UDP packets)

sudo apt install python3-pygame python3-numpy        # Raspberry Pi OS / Debian
git clone https://github.com/jamesharnett/Arduino-ESP32-Radarproject
cd Arduino-ESP32-Radarproject/uconsole
python3 radar_viewer.py                              # fullscreen; ESC quits
```

Keys: `UP/DOWN` zoom (2/4/8/12 m), `M` mute, `L` LIDAR on/off, `F` fullscreen,
`A` arm/disarm intrusion detection, `B` learn the empty-room baseline (10 s),
`R` record raw datagrams to a `.rdl` file. `--windowed`, `--no-audio`,
`--node IP` and `--offset DEG` are the useful options; `--help` lists the rest.
To start it at boot, see `uconsole/radar-viewer.service`.

### 4. Intrusion detection (uConsole)

The viewer turns the room outline into an alarm. With the room empty, learn a
**baseline** (press `B`, or start with `--learn 10`): the median LIDAR distance
per 0.5° bucket, with flickering buckets excluded and open directions (a long
corridor beyond range) remembered as open. Then **arm** (`A`, or `--armed`).
While armed, three things raise an alert:

| Alert kind | Trigger | Confidence |
|---|---|---|
| `fused` | LIDAR returns at least 30 cm closer than the baseline over ≥ 1.5° for ≥ 0.5 s **and** a moving radar target within 0.9 m of them | 0.95 |
| `lidar` | the same LIDAR change without radar motion: something was placed or someone is holding still | 0.6–0.7 |
| `radar` | a moving radar target with no LIDAR change: motion behind a thin door or beyond the LIDAR's range | 0.5 |

Alerts are drawn in red on the plot, listed in the HUD, sounded as a two-tone
siren (unless muted), appended to `alerts.log` as CSV and, with
`--alert-cmd`, handed to a shell command on each start, for example

```bash
python3 radar_viewer.py --armed --alert-cmd 'curl -s -d "RadarLink: $ALERT_KIND $ALERT_DIST_M m at $ALERT_ANGLE deg" ntfy.sh/your-topic'
```

`ALERT_KIND`, `ALERT_DIST_M`, `ALERT_ANGLE`, `ALERT_X_M`, `ALERT_Y_M`,
`ALERT_CONF` and `ALERT_ID` are in the command's environment. Tune with
`--delta-mm`, `--dwell-s` and `--min-buckets`; re-learn the baseline whenever
furniture moves. The detector lives in `uconsole/radar_intrusion.py` and is
unit tested without hardware.

### Testing without hardware

```bash
cd uconsole
python3 fake_sensor_node.py &                        # simulated node on UDP 4210
python3 radar_viewer.py --node 127.0.0.1 --windowed  # draws a 6 x 4 m room and two walkers
python3 fake_sensor_node.py --write demo.rdl --seconds 20   # or write a recording
python3 radar_viewer.py --replay demo.rdl --windowed --loop
python3 fake_sensor_node.py --quiet-room --intruder-after 15 &          # empty room, then an intruder
python3 radar_viewer.py --node 127.0.0.1 --windowed --learn 8         # learns, arms, alerts at ~15 s
python3 -m unittest discover -s tests                # protocol + state tests
g++ -std=c++11 -Wall -Wextra -Werror -I ../Firmware/sensor_node_esp32s3 ../Firmware/tests/host/test_parsers.cpp -o /tmp/t && /tmp/t
```

-------------------------------------------------------------------------------

## Calibration

* **LIDAR rotation.** The LD19's zero angle points wherever its connector
  points. Stand in front of the node: the matching LIDAR return must appear at
  the top of the plot. If it does not, read the angle where it appears, clockwise
  from the top, and set `LIDAR_ANGLE_OFFSET_DEG` in the GIGA `config.h` and
  `--offset` (or `LIDAR_ANGLE_OFFSET_DEG`) in the viewer to **minus** that angle:
  a return at 3 o'clock is +90°, so the offset is −90. The procedure is in
  `docs/TESTING.md`.
* **Radar mirror.** Walk to the left of the node; the dot must move left. If it
  moves right the module is mounted upside down: flip it or set `RADAR_MIRROR_X 1`.
* The radar's forward direction is always "up" on both displays; mount the
  RD-03D upright, 1.5–2 m high, antenna side out.

-------------------------------------------------------------------------------

## Security

The sensor node is an open target for anyone within Wi-Fi range who knows the
password, which is why the v1 defaults (`RadarNet` / `radar12345`, published in
the README) must not be used.

* Change `WIFI_PASS` in both `config.h` files before the first deployment.
  WPA2-PSK then keeps outsiders off the network.
* Receivers accept data only from the sensor node's address (the Wi-Fi
  gateway) and drop reordered or replayed sequence numbers.
* There is no per-packet authentication. If you need protection against someone
  who has the Wi-Fi password, add an HMAC to the protocol (it changes the wire
  format; do it before the viewer and display depend on it) or run the viewer's
  traffic over a VPN such as Tailscale when relaying off-site.

-------------------------------------------------------------------------------

## Known limitations

* The RD-03D only detects moving targets; a person standing still vanishes
  after about 2 s on both displays (the display's "still" filter, configurable).
  Ghost targets from wall reflections happen; the node drops the RD-03D's
  noise-slot speeds (0, ±248, ±256 cm/s) and merges returns closer than 1 m.
* The LD19 sees one plane at its mounting height: a table top above it is
  invisible, a chair leg at that height is a dot.
* Wi-Fi range is the sensor node's own: tens of metres indoors, roughly 100 m
  outdoors with line of sight. Walls cost range for the Wi-Fi and far more for
  the 24 GHz radar.
* The GIGA's Mbed `WiFiUDP` buffer is 508 bytes, which is why LIDAR datagrams
  carry at most 96 points. Its `WiFi.begin()` is blocking: a network scan, up to
  8 s to join and, if the join succeeds but DHCP does not answer, up to 60 s
  more. The screen shows the attempt and the display retries by itself.
* The GIGA renders at about 15 fps; the uConsole at 30. The GIGA's Wi-Fi
  receive path queues only about five datagrams, so anything that stalls its
  loop for more than ~70 ms drops LIDAR packets silently (visible as a lower
  `rx/s`, not as `drop`).
* Runtime on the suggested cells: about 4 h for the radar-only node, about 4 h
  for the LIDAR node on 3000 mAh, about 6 h for the GIGA display on 5000 mAh.
  The DW01 protection only cuts off at ~2.4 V, too deep for LiPo health, so
  switch off when the indicator shows one LED.

-------------------------------------------------------------------------------

## Enclosures

The four STL files in `CAD files/` are the v1 enclosures by the original author
(binary STL, Autodesk Fusion exports, low triangle counts so curves print faceted).
Measured bounding boxes:

| File | W × D × H (mm) | Holds |
|---|---|---|
| `body display.stl` | 34 × 105 × 113 | GIGA R1 + Display Shield + 1160100 LiPo |
| `top display.stl` | 8 × 85 × 111 | display lid |
| `top radarmodule.stl` | 47 × 31 × 87 | XIAO + RD-03D + 602560 LiPo |
| `bottom radarmodule.stl` | 47 × 6 × 67 | radar node base plate |

They fit the Level 1 radar node and the GIGA display. **Nothing yet
accommodates the LD19** (about 38 mm diameter, 35 mm tall, needs a clear 360°
view) or a larger cell; a Level 2 enclosure is an open item. Check the GIGA body
in your slicer before printing: the board is 101.5 mm long inside a 105 mm body.

-------------------------------------------------------------------------------

## Roadmap

1. **Level 2 enclosure** for the LD19 and a ≥ 3000 mAh cell.
2. ~~Intrusion logic on the uConsole~~ — done, see *Intrusion detection* above.
   Open items: per-zone arming (ignore a pet corridor), scheduled arming, and
   persistence of alert history beyond the CSV log.
3. **Off-site viewing:** the uConsole's 4G module plus Tailscale (or an MQTT
   relay) forwarding the UDP stream to a phone or laptop. The radio layer on the
   ESP32 stays as it is.
4. **True 3D (stage three):** a Unitree L2 or Livox Mid-360 connected to the
   uConsole over Ethernet/USB with ROS 2 or the vendor SDK; the ESP32 node then
   carries only the radar as the low-power motion trigger. Not an ESP32 or GIGA
   job: those sensors emit 64 000–200 000 points per second.

-------------------------------------------------------------------------------

## What changed from v1

Firmware and documentation were audited before this rewrite. Fixed here:

* Sensor node hosts the access point and serves any number of receivers that
  send `HELLO`; v1 unicast to the GIGA only and the boot order mattered.
* One documented, byte-exact protocol with sequence numbers; v1 sent a raw
  struct whose compiler padding made the author's own uConsole decoder misread
  targets 2 and 3.
* Non-blocking radar configuration and Wi-Fi handling; v1 blocked for 650 ms
  every minute and reboot-looped the XIAO when the AP was absent.
* GIGA receiver drains all pending datagrams and redraws the whole frame with
  cheap primitives at ~15 fps; v1 read one datagram per loop while spending
  ~125 000 trig calls per erased dot, so it lagged and flickered.
* `#include <WiFiUdp.h>` with the header name the core actually ships; v1 only
  compiled on case-insensitive file systems.
* Wi-Fi join failures are shown on screen and retried; v1 ignored the return
  value, so a missing GIGA Wi-Fi firmware looked like a boot-order problem.
* Corrected power wiring (cell on the XIAO BAT pads, boost only for the sensors,
  GIGA via USB-C or 7–7.5 V VIN), antennas and buzzer voltage documented.
* README paths, enclosure file names and sizes, the GIGA's real AP address in
  the v1 design (192.168.3.1, not 192.168.4.1), BW4056/TP4056 naming, stale
  "R4" labels.
* Sentinel-speed filter, clustering keeps the faster speed, `fb[12]` overflow
  after 10⁸ frames gone, toolchain versions pinned, tests added.

-------------------------------------------------------------------------------

## Credits and license

Original design, firmware, enclosures and build photos: Sudo_solder
([Stevee87](https://github.com/Stevee87)) in `Arduino-ESP32-Radarproject`,
`Radar-project-Uconsole` and `Lidar-Radar-combination-Raspberry`. LD19 frame
handling follows the LDROBOT SDK (CRC-8/0x4D table). RD-03D frame format from
the Ai-Thinker manual.

MIT — see [LICENSE](LICENSE).
