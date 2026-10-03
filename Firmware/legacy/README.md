# Legacy v1 firmware (reference only)

These are the original two sketches by Sudo_solder, kept so the first-generation
build (GIGA R1 as access point, XIAO as client, 32-byte packed-struct packets on
UDP 4210) can still be flashed and compared against. They are **not** compatible
with the current sensor node, display or uConsole viewer, which use the RadarLink
protocol in `docs/PROTOCOL.md`.

The only change made here is the `WiFiUdp.h` include name in the receiver, which
the Mbed core ships in that case and which failed to compile on Linux as `WiFiUDP.h`.

Known issues in this code, found in the audit that led to the rewrite:

* the transmitter unicasts to the Wi-Fi gateway only, so a second viewer gets nothing;
* the GIGA reads one UDP datagram per loop while each dot erase re-walks the whole
  sector with ~125k trig calls, so the display lags under load and never catches up;
* a blocking 20 s Wi-Fi connect followed by `ESP.restart()` reboot-loops the XIAO;
* a 650 ms blocking re-configuration of the radar every 60 s;
* `WiFi.beginAP()` failure is never checked, so a missing Wi-Fi firmware looks like a boot-order problem;
* the README's `192.168.4.1` is wrong for this core (the AP is `192.168.3.1`);
* `char fb[12]` overflows after 100 million frames (about 116 days).
