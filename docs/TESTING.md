# Bring-up, testing and calibration

Work through this in order the first time. Every stage has an expected output,
so a radar wiring fault, a LIDAR fault and a Wi-Fi fault each look different.

## 0. Bench test the software without hardware

```bash
cd uconsole
python3 -m unittest discover -s tests            # 19 tests: protocol bytes vs the C header, state logic
python3 fake_sensor_node.py &                    # simulated sensor node on UDP 4210
python3 radar_viewer.py --node 127.0.0.1 --windowed
```

You should see a 6 × 4 m rectangular room outline with a gap (a "doorway") and
one or two walkers in the radar sector, `DATA OK` in the HUD and roughly
`rx 70/s`. Quit with ESC, stop the fake node with `kill %1`.

Parser tests for the firmware run on your PC too:

```bash
g++ -std=c++11 -Wall -Wextra -Werror -I Firmware/sensor_node_esp32s3 Firmware/tests/host/test_parsers.cpp -o /tmp/t && /tmp/t
```

## 1. Sensor node on USB power, radar only

Battery switch **off**, `ENABLE_LIDAR 0` for this step if you like, XIAO on USB,
serial monitor at 115200 baud. Expected:

```
RadarLink sensor node — RD-03D + LD19
radar UART1 rx=1 tx=2 @256000 | lidar UART2 rx=3 tx=4 @230400
[wifi] AP "RadarSystem" up on channel 6, IP 192.168.4.1, UDP port 4210
[boot] ready; waiting for receivers to say HELLO
[radar] multi-target mode configured
[stat] up=2s ap=up sta=0 subs=0 | radar fps=20 bytes=1234 bad=0 cfg=1 reconf=0 | lidar fps=0 pps=0 bytes=0 crc=0 speed=0 | heap=...
```

With the RD-03D powered from the boost converter (trimmed to 5.0 V first):

| What you see | Meaning |
|---|---|
| `radar fps=10..20 bad=0`, `bytes` growing | radar wired and configured. Normal. |
| `bytes` growing, `fps=0`, `bad` growing | framing wrong: wrong baud (must be 256000) or a swapped TX/RX pair that still receives garbage |
| `bytes=0` | no data at all: RD-03D not powered (needs 5 V), TX not on D0, or GND missing |
| `reconf` incrementing every 5 s | radar silent for 5 s each time; the node re-sends the mode command. Check the 5 V rail with a meter under load |

The LED blinks briefly every 0.5 s while no receiver is subscribed and every
2 s once one is.

## 2. Add the LD19

Connect the LD19 (Tx → D2, P5V from the boost, GND common, PWM open). The motor
spins up within a second. Expected in `[stat]`:

```
lidar fps=370..380 pps=3500..4500 crc=0 speed=3500..3700
```

| What you see | Meaning |
|---|---|
| `fps≈375 crc=0` | good. `pps` is lower than 4500 when part of the room is beyond 12 m or absorbs the laser |
| `bytes` growing, `fps=0`, `crc` growing | wrong baud (must be 230400) or noise on the line |
| `bytes=0` | LD19 TX not on D2, or no power. Listen for the motor |
| `crc` growing slowly with `fps≈370` | occasional corrupt frames: supply ripple or a long unshielded lead. Add the 220 µF + 10 µF at the sensors |

## 3. GIGA display

Serial monitor at 115200. Expected:

```
RadarLink GIGA display starting
[wifi] connecting to RadarSystem
[wifi] connected, IP 192.168.4.2, sensor node 192.168.4.1, RSSI -45
[stat] link=up data=ok rx/s=70 total=140 drop=0 foreign=0 fps=15 | node radar_fps=20 lidar_fps=375 subs=1
```

The screen shows the plot once joined; `WIFI OK`, `DATA OK` and the node's
counters appear in the top-left HUD. On the sensor node you will see
`[subs] + 192.168.4.2:4210 kind=1 wants=0x07 (1 total)` and `[wifi] station joined`.

| What you see | Meaning |
|---|---|
| `connect failed, status 4` repeating, screen says "last attempt failed (code 4)" | wrong SSID/password, node not powered, or the GIGA's antenna is missing |
| `connect failed, status 6`, every attempt | the GIGA cannot see the SSID at all: antenna, range, or the node is still booting |
| "Failed to mount the filesystem containing the WiFi firmware" on serial | run *File → Examples → STM32H747_System → WiFiFirmwareUpdater* once |
| link up, `data=none`, HUD "waiting for sensor node" | packets not arriving: wrong `RECEIVER_WANTS`, or the node shows `subs=0` (HELLO not received: both on the same channel? check node log) |
| `drop` counting up | packets arriving out of order; harmless unless large |
| `fps` well below 15 | the display is CPU bound; disable LIDAR drawing (touch LIDAR OFF) to confirm |

## 4. uConsole

```bash
nmcli dev wifi connect RadarSystem password '...'
python3 radar_viewer.py --windowed
```

Console output every 2 s:

```
[net] listening on UDP 4210, sensor node 192.168.4.1:4210
[stat] data=ok rx/s=70 total=1400 drop=0 foreign=0 fps=30 | node radar_fps=20 lidar_fps=375 subs=2
```

`subs=2` confirms both displays are being served at once. If `data=none`
although the GIGA works: check that the viewer's node address is the Wi-Fi
gateway (`ip route`), that Wi-Fi power save is off (`iw dev wlan0 get power_save`),
and that no firewall blocks UDP 4210 inbound (`sudo ufw allow 4210/udp`).

## 5. Battery test

Switch to battery on each node, repeat stages 1–4, then leave everything running
for an hour and watch `reconf`, `crc` and `drop`. Rising `reconf` or `crc` only
on battery points at the boost converter: add capacitance or lower its load.

## 6. Boot order

Power the displays first, then the node; then the node first, then the displays;
then power-cycle the node while the displays run. All three must recover within
about 15 s: the displays retry every 3 s, drop a dead link after 15 s of silence
and the node accepts a `HELLO` at any time.

## Calibration

### LIDAR rotation offset

1. Stand 2 m in front of the node, in line with the radar's forward axis (the
   radar dot should sit on the vertical centre line at the top).
2. Look where the matching LIDAR return (a short arc at 2 m) appears. Read its
   angle clockwise from the top of the plot, e.g. 90° if it is at 3 o'clock.
3. Subtract: set `LIDAR_ANGLE_OFFSET_DEG` (GIGA `config.h`) and `--offset`
   (viewer, or edit `LIDAR_ANGLE_OFFSET_DEG` in `radar_viewer.py`) to minus that
   angle (−90 in the example). Re-flash / restart and confirm the arc moved up.
4. If the room appears mirrored (left/right swapped) set `LIDAR_CLOCKWISE 0`.

### Radar left/right

Walk from the centre line to your left. The dot must move left on both displays.
If it moves right, the RD-03D is mounted rotated 180°: turn it, or set
`RADAR_MIRROR_X 1` in the GIGA `config.h` (the viewer has the same constant).

### Radar range check

Walk away along the centre line. Targets should track to 6–8 m indoors. Loss
before 5 m usually means the module faces plastic thicker than 2 mm, a metal
object within 20 cm, or supply ripple.

## Recording a session

```bash
python3 radar_viewer.py --record walk1.rdl     # or press R while running
python3 radar_viewer.py --replay walk1.rdl --windowed --loop --speed 2
```

Recordings are raw datagrams with timestamps; a 10-minute session is about
15 MB. Attach one to an issue if you report a decoding or display problem.
