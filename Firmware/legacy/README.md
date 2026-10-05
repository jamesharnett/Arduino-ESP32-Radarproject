# Legacy v1.1 firmware (reference only)

These are the original author's two sketches in their current upstream form,
**Firmware 1.1** (Stevee87/Arduino-ESP32-Radarproject, October 2026): the GIGA R1
hosts the access point, the XIAO is a client, and the data is a 32-byte
packed-struct packet on UDP 4210. They are **not** compatible with the RadarLink
sensor node, display or uConsole viewer in this repository, which use the
protocol in `docs/PROTOCOL.md`. They are kept so the first-generation build can
still be flashed and compared against.

Firmware 1.1 already contains the fixes contributed from this fork (merged
upstream as pull requests #2 to #5): the `WiFiUdp.h` include name, the checked
access-point start, draining the UDP queue every loop, the `fb[16]` buffer,
the non-blocking Wi-Fi connection and radar configuration, and the clustering
speed fix. Upstream ships them in folders named `rd03d_giga_receiver v1.1` and
`rd03d_xiao_s3_sender v1.1`, which the Arduino IDE cannot open in place because
a sketch folder must carry the sketch's own name; here they sit in folders the
IDE accepts.

Remaining known issue in this code: `tone()` in the Arduino Mbed core leaks a
`DigitalOut` on every call (`Tone::stop()` nulls the pointer before the
destructor deletes it), so the distance-reactive buzzer exhausts the heap after
an hour or two of pinging. The RadarLink GIGA display drives its buzzer with a
sketch-owned ticker instead.
