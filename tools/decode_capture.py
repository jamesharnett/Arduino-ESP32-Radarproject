#!/usr/bin/env python3
"""Decode a raw RD-03D or LD19 byte capture and say what is in it.

Input is the hex the sensor node prints for ``raw radar`` / ``raw lidar`` (copy
everything between the "capturing" line and "end" into a file, or pipe the
serial monitor's output straight in), a plain hex dump, a C byte array, or a
binary file (``--binary``, e.g. ``tools/sensor_probe.py --save``).

    python3 tools/decode_capture.py capture.txt
    python3 tools/decode_capture.py --sensor lidar --verbose capture.txt
    python3 tools/decode_capture.py --binary --sensor radar probe.bin
    python3 tools/decode_capture.py -                              # paste the hex, then press Ctrl-D

The output is a summary (frames found, corrupt frames, bytes outside frames,
the module's configuration replies, which target slots were used), a line per
frame with ``--verbose``, and a hint when the stream looks like one of the
classic bring-up faults (wrong baud rate, idle line, lost bytes).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "uconsole"))
import radar_sensors as sn  # noqa: E402


def load(path: str, binary: bool):
    if binary:
        data = sys.stdin.buffer.read() if path == "-" else open(path, "rb").read()
        return None, data
    text = sys.stdin.read() if path == "-" else open(path, "r", errors="replace").read()
    return sn.parse_hex_dump(text)


def frame_lines(rep: sn.CaptureReport, options: sn.Rd03dOptions) -> List[str]:
    out = []
    if rep.sensor == "radar":
        for f in rep.radar_frames:
            raw = " ".join("empty" if s.empty else f"({s.x_mm:+d},{s.y_mm:+d} mm {s.speed_cm_s:+d} cm/s)" for s in f.slots())
            sent = [t for t in sn.rd03d_process(f, options) if t.valid]
            kept = " ".join(f"[{t.x_mm:+d},{t.y_mm:+d} {t.distance_mm / 1000:.2f} m {t.speed_cm_s:+d}]" for t in sent) or "nothing"
            out.append(f"@{f.offset:6d}  raw: {raw}  -> node sends: {kept}")
    else:
        for f in rep.lidar_frames:
            returns = sum(1 for d in f.dist_mm if d)
            out.append(f"@{f.offset:6d}  {f.speed_deg_s:4d} deg/s  {f.start_angle_cdeg / 100:6.2f} -> {f.end_angle_cdeg / 100:6.2f} deg  "
                       f"ts {f.timestamp_ms:5d}  {returns:2d}/12 returns  "
                       + " ".join(f"{d}" if d else "-" for d in f.dist_mm))
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("files", nargs="*", default=["-"], help="capture file(s); '-' or nothing reads stdin")
    p.add_argument("--sensor", choices=("auto", "radar", "lidar"), default="auto",
                   help="which parser; auto uses the [raw ...] prefix or whichever decodes more frames")
    p.add_argument("--binary", action="store_true", help="input is raw bytes, not hex text")
    p.add_argument("--verbose", "-v", action="store_true", help="print every decoded frame")
    p.add_argument("--json", action="store_true", help="machine-readable summary")
    p.add_argument("--min-dist", type=int, default=100, help="node's RADAR_MIN_DIST_MM for the 'node sends' column")
    p.add_argument("--max-dist", type=int, default=8000, help="node's RADAR_MAX_DIST_MM")
    p.add_argument("--cluster", type=int, default=1000, help="node's RADAR_CLUSTER_DIST_MM (0 = off)")
    p.add_argument("--keep-sentinels", action="store_true", help="node built with RADAR_FILTER_SENTINELS 0")
    a = p.parse_args(argv)
    options = sn.Rd03dOptions(a.min_dist, a.max_dist, a.cluster, not a.keep_sentinels)

    status = 0
    for path in a.files:
        source, data = load(path, a.binary)
        sensor = a.sensor if a.sensor != "auto" else (source or sn.guess_sensor(data))
        rep = sn.analyze_capture(data, sensor)
        if a.json:
            print(json.dumps({"file": path, "sensor": sensor, "bytes": rep.n_bytes, "frames": rep.frames,
                              "bad_frames": rep.bad_frames, "skipped_bytes": rep.skipped_bytes,
                              "slot_usage": rep.slot_usage, "sentinel_slots": rep.sentinel_slots,
                              "acks": [{"name": x.name, "ok": x.ok, "status": x.status} for x in rep.acks],
                              "lidar_speed": [rep.lidar_speed_min, rep.lidar_speed_max],
                              "lidar_implied_speed": round(rep.lidar_implied_speed),
                              "lidar_points": rep.lidar_points, "lidar_zero_points": rep.lidar_zero_points,
                              "hints": rep.hints, "ok": rep.ok}))
        else:
            if len(a.files) > 1 or path != "-":
                print(f"== {path}" + (" (auto-detected %s)" % sensor if a.sensor == "auto" else ""))
            if a.verbose:
                for line in frame_lines(rep, options):
                    print(line)
            for line in rep.summary_lines():
                print(line)
        if not rep.ok:
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
