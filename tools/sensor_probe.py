#!/usr/bin/env python3
"""Read an RD-03D or LD19 directly from a PC over a USB-UART adapter.

The quickest way to find out whether a sensor works at all, before it meets the
ESP32: wire the sensor's TX to the adapter's RX (3.3 V logic), power it from a
5 V pin, and run

    python3 tools/sensor_probe.py --port /dev/ttyUSB0 --sensor radar --configure
    python3 tools/sensor_probe.py --port COM5 --sensor lidar --seconds 10
    python3 tools/sensor_probe.py --port /dev/ttyUSB0 --sensor radar --save radar.bin --hex

``--configure`` sends the RD-03D the same enable / multi-target / end sequence
the node sends at boot (adapter TX -> module RX needed for that). Live output
shows each radar frame's targets, or once a second the LIDAR's frame rate, scan
speed and CRC errors; the final report is the same as tools/decode_capture.py.

Needs pyserial: ``pip install pyserial`` or ``sudo apt install python3-serial``.
Not every adapter can do 256000 baud (RD-03D): CP2102, CH340 and FT232 can,
some clones cannot; the LD19's 230400 works everywhere.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "uconsole"))
import radar_sensors as sn  # noqa: E402

CONFIG_SEQUENCE = ((sn.RD03D_CMD_ENABLE_CONFIG, 0.2), (sn.RD03D_CMD_MULTI_TARGET, 0.2), (sn.RD03D_CMD_END_CONFIG, 0.2))


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial is not installed: pip install pyserial  (Debian: sudo apt install python3-serial)")
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except (OSError, serial.SerialException) as exc:  # type: ignore[attr-defined]
        sys.exit(f"cannot open {port} at {baud} baud: {exc}")


class Probe:
    """Reads from anything with .read(n) (a pyserial port, or a fake in the tests)."""

    def __init__(self, sensor: str, port, out=sys.stdout, clock=time.monotonic, sleep=time.sleep) -> None:
        self.sensor = sensor
        self.port = port
        self.out = out
        self.clock = clock
        self.sleep = sleep
        self.data = bytearray()
        self.radar = sn.Rd03dParser()
        self.lidar = sn.Ld19Parser()

    def say(self, msg: str) -> None:
        print(msg, file=self.out, flush=True)

    def configure_radar(self) -> None:
        for cmd, gap in CONFIG_SEQUENCE:
            self.port.write(cmd)
            self.sleep(gap)
            self.data += self.port.read(4096)          # the replies land in the capture too
        acks = sn.rd03d_find_acks(bytes(self.data))
        if not acks:
            self.say("no reply to the configuration commands (adapter TX not wired to the module's RX, or 3.3 V/5 V level issue); continuing")
        for a in acks:
            self.say(f"module replied: {a.name} {'OK' if a.ok else 'FAILED status %d' % a.status}")

    def run(self, seconds: float, hex_lines: bool = False, verbose: bool = True) -> sn.CaptureReport:
        start = self.clock()
        last_report = start
        pending = bytearray()
        radar_frames_sec = lidar_frames_sec = lidar_crc_sec = 0
        while self.clock() - start < seconds:
            chunk = self.port.read(4096)
            if chunk:
                self.data += chunk
                pending += chunk
                if hex_lines:
                    while len(pending) >= 32:
                        self.say(f"[raw {self.sensor}] " + " ".join(f"{b:02X}" for b in pending[:32]))
                        del pending[:32]
                if self.sensor == "radar":
                    for f in self.radar.feed_bytes(chunk):
                        radar_frames_sec += 1
                        if verbose:
                            kept = [t for t in sn.rd03d_process(f) if t.valid]
                            self.say(f"frame {self.radar.frames:5d}  "
                                     + (" ".join(f"[{t.x_mm:+5d},{t.y_mm:+5d} mm {t.distance_mm / 1000:4.2f} m {t.speed_cm_s:+4d} cm/s]" for t in kept)
                                        or "no target"))
                else:
                    before_f, before_c = self.lidar.frames, self.lidar.crc_errors
                    self.lidar.feed_bytes(chunk)
                    lidar_frames_sec += self.lidar.frames - before_f
                    lidar_crc_sec += self.lidar.crc_errors - before_c
            now = self.clock()
            if now - last_report >= 1.0:
                if verbose:
                    if self.sensor == "lidar":
                        rep = sn.analyze_capture(bytes(self.data[-47 * 40:]), "lidar")
                        self.say(f"lidar: {lidar_frames_sec} frames/s, {lidar_crc_sec} CRC errors, speed field "
                                 f"{rep.lidar_speed_min}..{rep.lidar_speed_max} deg/s, {len(self.data)} bytes so far")
                    elif radar_frames_sec == 0:
                        self.say(f"radar: no frame this second ({len(self.data)} bytes so far, {self.radar.bad_frames} bad frames)")
                radar_frames_sec = lidar_frames_sec = lidar_crc_sec = 0
                last_report = now
        if hex_lines and pending:
            self.say(f"[raw {self.sensor}] " + " ".join(f"{b:02X}" for b in pending))
        return sn.analyze_capture(bytes(self.data), self.sensor)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0 or COM5")
    p.add_argument("--sensor", choices=("radar", "lidar"), required=True)
    p.add_argument("--baud", type=int, default=0, help="override the sensor's default (256000 radar, 230400 lidar)")
    p.add_argument("--seconds", type=float, default=5.0, help="how long to listen")
    p.add_argument("--configure", action="store_true", help="radar: send the enable / multi-target / end sequence first")
    p.add_argument("--save", metavar="FILE", help="write the raw bytes to FILE (decode later with decode_capture.py --binary)")
    p.add_argument("--hex", action="store_true", help="also print the bytes as [raw ...] hex lines")
    p.add_argument("--quiet", action="store_true", help="no per-frame output, just the final report")
    a = p.parse_args(argv)
    baud = a.baud or (sn.RD03D_BAUD if a.sensor == "radar" else sn.LD19_BAUD)
    port = open_serial(a.port, baud)
    probe = Probe(a.sensor, port)
    probe.say(f"listening on {a.port} at {baud} baud for {a.seconds:g} s ({a.sensor})")
    try:
        if a.configure and a.sensor == "radar":
            probe.configure_radar()
        rep = probe.run(a.seconds, hex_lines=a.hex, verbose=not a.quiet)
    except KeyboardInterrupt:
        rep = sn.analyze_capture(bytes(probe.data), a.sensor)
    finally:
        port.close()
    if a.save:
        with open(a.save, "wb") as fh:
            fh.write(probe.data)
        probe.say(f"saved {len(probe.data)} bytes to {a.save}")
    for line in rep.summary_lines():
        probe.say(line)
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
