#!/usr/bin/env python3
"""Analyse RadarLink serial logs and viewer recordings and say what went wrong.

Give it anything you saved during bring-up (docs/TESTING.md):

* the sensor node's serial output (``[stat] up=... | radar fps=... | lidar ...``)
* the GIGA display's serial output (``[stat] link=up data=ok rx/s=...``)
* the uConsole viewer's or relay's console output (``[stat] data=ok rx/s=...``)
* a ``.rdl`` recording made with ``radar_viewer.py --record`` or ``R``

    python3 tools/analyze_log.py node.log
    python3 tools/analyze_log.py node.log giga.log walk.rdl --plot report.png
    python3 tools/analyze_log.py node.log --json > node.json

Serial-monitor timestamps (Arduino IDE ``12:34:56.789 -> ``, ``[12:34:56]``, ISO
dates, journalctl prefixes) are understood but optional. For each file you get
the measured rates, how they varied, the events (joins, timeouts, reconfigurations,
reconnects, alerts), ASCII histograms of what the sensors saw, and a diagnosis
with the same rules as the node's own ``diag`` console command. ``--plot``
needs matplotlib; everything else is standard library.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "uconsole"))
import radar_protocol as rp  # noqa: E402
import radar_state as rs     # noqa: E402

# Same thresholds as Firmware/sensor_node_esp32s3/node_diag.h (keep in step).
DIAG_SETTLE_S = 3
RADAR_FPS_MIN, RADAR_FPS_MAX = 5, 40
RADAR_BAD_PCT_MAX = 10
LIDAR_FPS_MIN = 300
LIDAR_SPEED_MIN, LIDAR_SPEED_MAX = 2500, 4700
LIDAR_CRC_PCT_MAX = 2
LIDAR_PPS_LOW = 1000
HEAP_MIN_BYTES = 20000
STAT_INTERVAL_S = 2.0            # DEBUG_INTERVAL_MS on every speaker

WIFI_STATUS = {               # Arduino Mbed core WiFi.begin() return values
    0: "no Wi-Fi module answered (status 0): run the WiFiFirmwareUpdater example once",
    1: "SSID not found (status 1): node not powered or out of range",
    4: "connect failed (status 4): SSID not seen in the scan or the join was refused; node off, out of range, antenna, or wrong password",
    5: "connection lost (status 5)",
    6: "disconnected (status 6): usually a wrong password",
    255: "no Wi-Fi shield (status 255)",
}

LEVELS = ("OK", "INFO", "WARN", "FAIL")


# ----------------------------------------------------------------- helpers
def pct(part: float, whole: float) -> float:
    return 100.0 * part / whole if whole else (100.0 if part else 0.0)


def stats(values: List[float]) -> str:
    if not values:
        return "no samples"
    if len(values) == 1:
        return f"{values[0]:g}"
    return f"min {min(values):g} / median {statistics.median(values):g} / max {max(values):g} ({len(values)} samples)"


def slope_per_hour(xs: List[float], ys: List[float]) -> Optional[float]:
    """Least-squares slope in units per hour; None when the span is under ten minutes."""
    if len(xs) < 3 or xs[-1] - xs[0] < 600:
        return None
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx * 3600.0


def bars(rows: List[Tuple[str, float]], width: int = 40, unit: str = "") -> List[str]:
    top = max((v for _, v in rows), default=0)
    out = []
    for label, v in rows:
        n = int(round(width * v / top)) if top else 0
        out.append(f"  {label:>10} {'#' * n:<{width}} {v:g}{unit}")
    return out


from radar_sensors import split_timestamp  # noqa: E402  (the same prefixes decode_capture accepts)


def kv(text: str) -> Dict[str, str]:
    return dict(re.findall(r"([A-Za-z_/]+)=([^\s|]+)", text))


def to_int(s: Optional[str], default: int = 0) -> int:
    try:
        return int(s) if s is not None else default
    except ValueError:
        return default


# ----------------------------------------------------------------- parsing
@dataclass
class NodeSample:
    t: float
    up: int
    ap_up: bool
    sta: int
    subs: int
    radar_fps: int
    radar_bytes: int
    radar_bad: int
    cfg: int
    reconf: int
    heap: int
    lidar: Optional[Dict[str, int]] = None


@dataclass
class RxSample:
    t: float
    link: Optional[bool]
    data_ok: bool
    rx_s: int
    total: int
    drop: int
    foreign: int
    fps: int
    node_radar_fps: Optional[int]
    node_lidar_fps: Optional[int]
    node_subs: Optional[int]


@dataclass
class Event:
    t: float
    kind: str
    text: str
    clock: bool = False           # t came from a timestamp on the line, not from a line count


@dataclass
class TextLog:
    path: str
    node: List[NodeSample] = field(default_factory=list)
    giga: List[RxSample] = field(default_factory=list)
    viewer: List[RxSample] = field(default_factory=list)
    relay: List[RxSample] = field(default_factory=list)
    events: List[Event] = field(default_factory=list)
    diag_lines: List[str] = field(default_factory=list)
    lines: int = 0
    unparsed: int = 0


NODE_EVENTS = [
    ("node_boot", re.compile(r"^RadarLink sensor node")),
    ("node_ready", re.compile(r"^\[boot\] ready")),
    ("ap_up", re.compile(r"^\[wifi\] AP \"")),
    ("ap_failed", re.compile(r"^\[wifi\] softAP\(\) FAILED")),
    ("sta_joined", re.compile(r"^\[wifi\] station joined")),
    ("sta_left", re.compile(r"^\[wifi\] station left")),
    ("sub_add", re.compile(r"^\[subs\] \+ ")),
    ("sub_timeout", re.compile(r"^\[subs\] - .* timed out")),
    ("radar_reconf", re.compile(r"^\[radar\] no frames for (\d+) ms")),
    ("radar_cfg", re.compile(r"^\[radar\] multi-target mode configured")),
]
RX_EVENTS = [
    ("giga_boot", re.compile(r"^RadarLink GIGA display starting")),
    ("connecting", re.compile(r"^\[wifi\] connecting to")),
    ("connect_failed", re.compile(r"^\[wifi\] connect failed, status (\d+)")),
    ("connected", re.compile(r"^\[wifi\] connected, IP (\S+), sensor node (\S+), RSSI (-?\d+)")),
    ("reconnect_silent", re.compile(r"^\[wifi\] no packets for a long time")),
    ("reconnect_lost", re.compile(r"^\[wifi\] association lost")),
    ("listening", re.compile(r"^\[net\] listening on UDP")),
    ("hello_failed", re.compile(r"^\[(?:net|relay)\] hello failed")),
    ("recv_error", re.compile(r"^\[(?:net|relay)\] recv error")),
    ("remote_add", re.compile(r"^\[relay\] \+ remote viewer")),
    ("remote_timeout", re.compile(r"^\[relay\] - remote viewer")),
    ("alert_start", re.compile(r"^\[alert\] START #(\d+) (.*)")),
    ("alert_end", re.compile(r"^\[alert\] END #(\d+)")),
    ("detect", re.compile(r"^\[detect\] ")),
    ("record", re.compile(r"^\[record\] ")),
]


def parse_text_log(path: str, text: str) -> TextLog:
    log = TextLog(path)
    idx = 0
    for raw in text.splitlines():
        log.lines += 1
        t_clock, line = split_timestamp(raw.rstrip("\r"))
        line = line.strip()
        if not line:
            continue
        idx += 1
        t = t_clock if t_clock is not None else idx * STAT_INTERVAL_S
        if line.startswith("[diag]"):
            log.diag_lines.append(line)
            continue
        if line.startswith("[stat]"):
            d = kv(line)
            if "up" in d:                                   # sensor node
                lidar = None
                if "crc" in d:
                    seg = line.split("| lidar", 1)[1].split("|")[0] if "| lidar" in line else ""
                    ld = kv(seg)
                    lidar = {"fps": to_int(ld.get("fps")), "pps": to_int(ld.get("pps")), "bytes": to_int(ld.get("bytes")),
                             "crc": to_int(ld.get("crc")), "speed": to_int(ld.get("speed"))}
                radar_seg = kv(line.split("| radar", 1)[1].split("|")[0]) if "| radar" in line else {}
                up = to_int(d["up"].rstrip("s"))
                log.node.append(NodeSample(t_clock if t_clock is not None else float(up), up, d.get("ap") == "up",
                                           to_int(d.get("sta")), to_int(d.get("subs")), to_int(radar_seg.get("fps")),
                                           to_int(radar_seg.get("bytes")), to_int(radar_seg.get("bad")),
                                           to_int(radar_seg.get("cfg")), to_int(radar_seg.get("reconf")),
                                           to_int(d.get("heap")), lidar))
                continue
            if "data" in d:
                node_part = kv(line.split("| node", 1)[1]) if "| node" in line else {}
                s = RxSample(t, (d["link"] == "up") if "link" in d else None, d["data"] == "ok", to_int(d.get("rx/s")),
                             to_int(d.get("total", d.get("fwd"))), to_int(d.get("drop")), to_int(d.get("foreign")),
                             to_int(d.get("fps")),
                             to_int(node_part.get("radar_fps"), -1) if node_part else None,
                             to_int(node_part.get("lidar_fps"), -1) if node_part else None,
                             to_int(node_part.get("subs"), -1) if node_part else None)
                if "link" in d:
                    log.giga.append(s)
                elif "fwd" in d or "remote" in line:
                    log.relay.append(s)
                else:
                    log.viewer.append(s)
                continue
        matched = False
        for kind, pat in NODE_EVENTS + RX_EVENTS:
            m = pat.match(line)
            if m:
                log.events.append(Event(t, kind, line, t_clock is not None))
                matched = True
                break
        if not matched and not line.startswith(("[", "radar UART", "RadarLink")):
            log.unparsed += 1
    return log


# ---------------------------------------------------------------- analysis
@dataclass
class Report:
    title: str
    sections: List[Tuple[str, List[str]]] = field(default_factory=list)
    issues: List[Tuple[str, str]] = field(default_factory=list)     # (level, text)
    metrics: Dict[str, object] = field(default_factory=dict)
    series: Dict[str, Tuple[List[float], List[float]]] = field(default_factory=dict)

    def say(self, level: str, text: str) -> None:
        self.issues.append((level, text))

    @property
    def worst(self) -> str:
        order = {lv: i for i, lv in enumerate(LEVELS)}
        return max((lv for lv, _ in self.issues), key=lambda lv: order[lv], default="OK")

    def render(self) -> List[str]:
        out = [f"=== {self.title}"]
        for name, lines in self.sections:
            out.append(f"--- {name}")
            out += [f"  {l}" for l in lines]
        out.append("--- diagnosis")
        if not self.issues:
            out.append("  OK   nothing to report")
        for lv, text in self.issues:
            out.append(f"  {lv:<4} {text}")
        out.append(f"--- result: {self.worst}")
        return out


def analyze_node(log: TextLog, rep: Report) -> None:
    ns = log.node
    if not ns:
        return
    ups = [s.up for s in ns]
    resets = sum(1 for a, b in zip(ups, ups[1:]) if b < a)
    banners = sum(1 for e in log.events if e.kind == "node_boot")
    reboots = max(resets, banners - 1 if banners else 0)       # the first banner is the boot the log starts with
    settled = [s for s in ns if s.up >= DIAG_SETTLE_S]
    last = ns[-1]
    span = max(ups) - min(ups)
    lines = [f"{len(ns)} samples, uptime {min(ups)}..{max(ups)} s ({span} s covered), {reboots} reboot(s) seen"]
    # radar. bytes, bad frames, CRC errors and reconfigurations are counters since
    # boot, so everything is measured as the growth over the lines in this log
    # (positive steps only: a reboot restarts the counters).
    fps = [s.radar_fps for s in settled]
    frames_est = sum(s.radar_fps for s in ns) * STAT_INTERVAL_S          # fps is a one-second count sampled every 2 s
    zero = sum(1 for f in fps if f == 0)
    bytes_delta = sum(max(0, b.radar_bytes - a.radar_bytes) for a, b in zip(ns, ns[1:])) if len(ns) > 1 else last.radar_bytes
    bad_delta = sum(max(0, b.radar_bad - a.radar_bad) for a, b in zip(ns, ns[1:])) if len(ns) > 1 else last.radar_bad
    reconf_delta = sum(max(0, b.reconf - a.reconf) for a, b in zip(ns, ns[1:])) if len(ns) > 1 else last.reconf
    bad_pct = pct(bad_delta, frames_est)
    lines.append(f"radar frames/s: {stats(fps)}; {zero} sample(s) at 0 fps; {bytes_delta} bytes received during the log; "
                 f"bad frames +{bad_delta} (~{bad_pct:.1f}% of ~{frames_est:.0f} frames; {last.radar_bad} since boot); "
                 f"reconfigured {reconf_delta}x during the log ({last.reconf} since boot); "
                 f"configured in {sum(1 for s in settled if s.cfg)}/{len(settled)} samples")
    rep.series["radar fps"] = ([s.up for s in ns], [s.radar_fps for s in ns])
    # lidar
    has_lidar = any(s.lidar for s in ns)
    if has_lidar:
        lf = [s.lidar["fps"] for s in settled if s.lidar]
        lp = [s.lidar["pps"] for s in settled if s.lidar]
        sp = [s.lidar["speed"] for s in settled if s.lidar and s.lidar["speed"]]
        lsamples = [s for s in ns if s.lidar]
        crc_delta = (sum(max(0, b.lidar["crc"] - a.lidar["crc"]) for a, b in zip(lsamples, lsamples[1:]))
                     if len(lsamples) > 1 else lsamples[-1].lidar["crc"])
        lbytes_delta = (sum(max(0, b.lidar["bytes"] - a.lidar["bytes"]) for a, b in zip(lsamples, lsamples[1:]))
                        if len(lsamples) > 1 else lsamples[-1].lidar["bytes"])
        lframes = sum(s.lidar["fps"] for s in lsamples) * STAT_INTERVAL_S
        lines.append(f"lidar frames/s: {stats(lf)}; points/s: {stats(lp)}; scan speed deg/s: {stats(sp)}; "
                     f"CRC errors +{crc_delta} (~{pct(crc_delta, lframes):.2f}% of ~{lframes:.0f} frames; "
                     f"{lsamples[-1].lidar['crc']} since boot); {lbytes_delta} bytes during the log")
        rep.series["lidar fps"] = ([s.up for s in ns if s.lidar], [s.lidar["fps"] for s in ns if s.lidar])
        rep.series["lidar pps"] = ([s.up for s in ns if s.lidar], [s.lidar["pps"] for s in ns if s.lidar])
    # wifi
    joins = sum(1 for e in log.events if e.kind == "sta_joined")
    leaves = sum(1 for e in log.events if e.kind == "sta_left")
    adds = sum(1 for e in log.events if e.kind == "sub_add")
    timeouts = sum(1 for e in log.events if e.kind == "sub_timeout")
    orphan = sum(1 for s in settled if s.sta > 0 and s.subs == 0)
    lines.append(f"wifi: AP down in {sum(1 for s in ns if not s.ap_up)} samples; stations {stats([s.sta for s in ns])}; "
                 f"subscribers {stats([s.subs for s in ns])}; {joins} join(s), {leaves} leave(s), {adds} HELLO registration(s), "
                 f"{timeouts} subscriber timeout(s); {orphan} sample(s) with stations but no subscriber")
    rep.series["subscribers"] = ([s.up for s in ns], [s.subs for s in ns])
    # heap
    heaps = [s.heap for s in ns if s.heap]
    if heaps:
        sl = slope_per_hour([s.up for s in ns if s.heap], heaps)
        lines.append(f"heap: first {heaps[0]} last {heaps[-1]} min {min(heaps)} bytes"
                     + (f"; trend {sl:+.0f} bytes/h" if sl is not None else "; trend needs 10+ min of log"))
        rep.series["heap"] = ([s.up for s in ns if s.heap], heaps)
    rep.sections.append(("sensor node", lines))
    if log.diag_lines:
        rep.sections.append(("node's own diag output", log.diag_lines))
    rep.metrics.update(node_samples=len(ns), node_reboots=reboots, radar_fps_median=statistics.median(fps) if fps else None,
                       radar_bad=bad_delta, radar_bad_pct=round(bad_pct, 2), radar_reconf=reconf_delta,
                       lidar=bool(has_lidar), subscriber_timeouts=timeouts, heap_min=min(heaps) if heaps else None)

    # diagnosis with the node_diag.h rules, applied to the end of the log
    if reboots >= 1:
        rep.say("WARN", f"the node rebooted {reboots} time{'s' if reboots > 1 else ''} during this log: brown-outs (check the "
                        "5 V/battery path) or crashes; note the last lines before each 'RadarLink sensor node' banner")
    if last.up < DIAG_SETTLE_S or not settled:
        rep.say("INFO", "log too short to judge the sensors (under 3 s of uptime)")
    else:
        if bytes_delta == 0:
            rep.say("FAIL", "radar: no bytes arrived during this log: RD-03D not powered (needs 5 V), TX not on D0/GPIO1, or no common ground")
        elif frames_est == 0 and bad_delta == 0:
            rep.say("FAIL", "radar: bytes arrive but no frame was decoded: wrong baud (must be 256000) or line noise; "
                            "capture with 'raw radar' and run tools/decode_capture.py")
        elif frames_est == 0:
            rep.say("FAIL", f"radar: {bad_delta} frames with a bad tail and none good: lost bytes or an unexpected frame layout")
        else:
            if bad_pct > RADAR_BAD_PCT_MAX:
                rep.say("WARN", f"radar: {bad_pct:.0f}% bad frames: supply ripple or lost bytes; add 220 uF + 10 uF at the module")
            if zero and settled:
                rep.say("WARN" if zero > len(settled) * 0.1 else "INFO",
                        f"radar: {zero} of {len(settled)} samples had no frame in the counted second"
                        + ("" if zero <= len(settled) * 0.1 else ": intermittent supply or wiring"))
            if reconf_delta:
                rep.say("WARN", f"radar: the node re-sent the configuration {reconf_delta} time(s) during this log: the module went silent for 5 s each time (supply dips)")
            med = statistics.median(fps) if fps else 0
            if fps and (med < RADAR_FPS_MIN or med > RADAR_FPS_MAX):
                rep.say("WARN", f"radar: median frame rate {med:g}/s is outside {RADAR_FPS_MIN}..{RADAR_FPS_MAX}")
            if not any(lv == "FAIL" or (lv == "WARN" and t.startswith("radar")) for lv, t in rep.issues):
                rep.say("OK", f"radar: {med:g} frames/s, {bad_pct:.1f}% bad frames")
        if has_lidar and last.lidar:
            lmed = statistics.median([s.lidar["fps"] for s in settled if s.lidar]) if settled else 0
            if lbytes_delta == 0:
                rep.say("FAIL", "lidar: no bytes arrived during this log: LD19 not powered (5 V, 180 mA+), TX not on D2/GPIO3, or no common ground; is the motor turning?")
            elif lframes == 0 and crc_delta == 0:
                rep.say("FAIL", "lidar: bytes but never a frame: wrong baud (must be 230400) or another device on that pin")
            elif lframes == 0:
                rep.say("FAIL", f"lidar: {crc_delta} CRC failures and no good frame: baud mismatch, lost bytes or noise")
            else:
                crc_pct = pct(crc_delta, lframes)
                if crc_pct > LIDAR_CRC_PCT_MAX:
                    rep.say("WARN", f"lidar: {crc_pct:.1f}% CRC errors: supply ripple or a long unshielded lead")
                if sp and (statistics.median(sp) < LIDAR_SPEED_MIN or statistics.median(sp) > LIDAR_SPEED_MAX):
                    rep.say("WARN", f"lidar: scan speed median {statistics.median(sp):g} deg/s outside {LIDAR_SPEED_MIN}..{LIDAR_SPEED_MAX}: PWM pin open? supply?")
                if lmed < LIDAR_FPS_MIN:
                    rep.say("WARN", f"lidar: median {lmed:g} frames/s (expect ~375): frames lost, usually a stalled USB serial host blocking the loop")
                lp_med = statistics.median(lp) if lp else 0
                if lp_med < LIDAR_PPS_LOW:
                    rep.say("INFO", f"lidar: only {lp_med:g} returns/s: most directions beyond range or absorbing the laser")
                if not any(t.startswith("lidar") and lv in ("WARN", "FAIL") for lv, t in rep.issues):
                    rep.say("OK", f"lidar: {lmed:g} frames/s, {lp_med:g} points/s, {crc_pct:.2f}% CRC errors")
        if not last.ap_up:
            rep.say("FAIL", "wifi: the access point is down at the end of the log")
        elif orphan > 2:
            rep.say("WARN", f"wifi: {orphan} sample(s) had a station joined but no subscriber: the receiver is not sending HELLO "
                            "(old firmware, wrong UDP port 4210)")
        elif last.sta == 0 and last.up > 20:
            rep.say("INFO", "wifi: no receiver joined the access point during this log")
        if timeouts > max(2, adds // 2) and adds:
            rep.say("WARN", f"wifi: {timeouts} subscriber timeouts for {adds} registrations: HELLOs are getting lost (weak link, "
                            "receiver power save) so data stops for up to 5 s each time")
        if heaps and min(heaps) < HEAP_MIN_BYTES:
            rep.say("WARN", f"heap: free memory dipped to {min(heaps)} bytes")
        sl = slope_per_hour([s.up for s in ns if s.heap], heaps) if heaps else None
        if sl is not None and sl < -2000:
            rep.say("WARN", f"heap: shrinking by {-sl:.0f} bytes/h: a memory leak, report this with the log")


def analyze_receiver(samples: List[RxSample], events: List[Event], name: str, rep: Report) -> None:
    if not samples and not any(e.kind in ("giga_boot", "connecting", "connect_failed", "connected", "listening") for e in events):
        return
    lines = []
    if samples:
        last = samples[-1]
        ok = sum(1 for s in samples if s.data_ok)
        lines.append(f"{len(samples)} samples; data ok in {ok} ({pct(ok, len(samples)):.0f}%); rx/s {stats([s.rx_s for s in samples])}; "
                     f"received {last.total}, dropped {last.drop} ({pct(last.drop, last.total + last.drop):.2f}%), foreign {last.foreign}; "
                     f"render fps {stats([s.fps for s in samples])}")
        if any(s.link is not None for s in samples):
            up = sum(1 for s in samples if s.link)
            lines.append(f"link up in {up} of {len(samples)} samples")
        node_seen = [s for s in samples if s.node_radar_fps is not None and s.node_radar_fps >= 0]
        if node_seen:
            lines.append(f"node status seen: radar fps {stats([s.node_radar_fps for s in node_seen])}, "
                         f"lidar fps {stats([s.node_lidar_fps for s in node_seen])}, subscribers {stats([s.node_subs for s in node_seen])}")
        rep.series[f"{name} rx/s"] = ([s.t for s in samples], [s.rx_s for s in samples])
    attempts = [e for e in events if e.kind == "connecting"]
    failures = [e for e in events if e.kind == "connect_failed"]
    connected = [e for e in events if e.kind == "connected"]
    codes = Counter(re.search(r"status (\d+)", e.text).group(1) for e in failures)
    if attempts or failures or connected:
        lines.append(f"wifi: {len(attempts)} attempt(s), {len(failures)} failure(s) {dict(codes) if codes else ''}, {len(connected)} success(es)")
        rssi = [int(re.search(r"RSSI (-?\d+)", e.text).group(1)) for e in connected]
        if rssi:
            lines.append(f"RSSI at connect: {stats(rssi)} dBm")
            if min(rssi) < -75:
                rep.say("WARN", f"{name}: weak signal ({min(rssi)} dBm): move closer, plug the u.FL antenna in, change WIFI_CHANNEL")
        # how long each attempt took: from its "connecting" line to the next failed/connected line
        outcomes = sorted([e for e in events if e.kind in ("connect_failed", "connected")], key=lambda e: e.t)
        slow = 0
        for a in attempts:
            if not a.clock:
                continue                                       # a line count is not a duration
            end = next((o for o in outcomes if o.t > a.t), None)
            if end is not None and end.t - a.t > 50:
                slow += 1
        if slow:
            rep.say("WARN", f"{name}: {slow} join attempt(s) took about a minute: the join worked but DHCP did not answer; power-cycle the node")
        for code, n in codes.items():
            meaning = WIFI_STATUS.get(int(code), f"status {code}")
            rep.say("FAIL" if not connected else "INFO", f"{name}: {n}x {meaning}")
    silent = sum(1 for e in events if e.kind == "reconnect_silent")
    lost = sum(1 for e in events if e.kind == "reconnect_lost")
    if silent or lost:
        lines.append(f"reconnects: {silent} after silence, {lost} after association loss")
        rep.say("WARN", f"{name}: {silent + lost} reconnect(s): the node stopped sending or the link dropped; check the node log at the same time")
    hello_failed = sum(1 for e in events if e.kind == "hello_failed")
    if hello_failed:
        rep.say("WARN", f"{name}: {hello_failed} HELLO send failure(s): no route to the node (not joined to RadarSystem, wrong --node address)")
    alerts = [e for e in events if e.kind == "alert_start"]
    if alerts or any(e.kind == "detect" for e in events):
        kinds = Counter(re.search(r"START #\d+ (\w+)", e.text).group(1) for e in alerts if re.search(r"START #\d+ (\w+)", e.text))
        lines.append(f"intrusion: {len(alerts)} alert(s) {dict(kinds) if kinds else ''}; "
                     + "; ".join(e.text for e in events if e.kind == "detect")[:300])
    rep.sections.append((name, lines))
    rep.metrics[f"{name}_samples"] = len(samples)
    if samples:
        last = samples[-1]
        rep.metrics[f"{name}_drop_pct"] = round(pct(last.drop, last.total + last.drop), 3)
        rep.metrics[f"{name}_data_ok_pct"] = round(pct(sum(1 for s in samples if s.data_ok), len(samples)), 1)
        if not last.data_ok:
            if last.link is False:
                rep.say("FAIL", f"{name}: not joined to the node's Wi-Fi at the end of the log")
            elif last.total == 0:
                rep.say("FAIL", f"{name}: joined but never received a packet: check RECEIVER_WANTS / the node's 'subs' output; "
                                "on the uConsole check the --node address, Wi-Fi power save and the firewall (UDP 4210)")
            else:
                rep.say("WARN", f"{name}: data stopped before the end of the log (last rx/s {last.rx_s})")
        if last.drop and pct(last.drop, last.total + last.drop) > 2:
            rep.say("WARN", f"{name}: {pct(last.drop, last.total + last.drop):.1f}% of datagrams dropped (out of order or lost): weak link or power save")
        if last.foreign:
            rep.say("INFO", f"{name}: {last.foreign} datagram(s) from another address were ignored")
        fps_med = statistics.median([s.fps for s in samples])
        if name == "GIGA display" and fps_med < 10:
            rep.say("WARN", f"GIGA display: median {fps_med:g} fps: CPU bound; try LIDAR OFF to confirm")
        if not any(t.startswith(name) and lv in ("WARN", "FAIL") for lv, t in rep.issues) and last.data_ok:
            rep.say("OK", f"{name}: receiving {last.rx_s} datagrams/s, {pct(last.drop, last.total + last.drop):.2f}% dropped")


# -------------------------------------------------------------- recordings
def analyze_recording(path: str, data: bytes, rep: Report) -> None:
    import io
    try:
        replay = rs.Replayer(io.BytesIO(data))
    except ValueError as exc:
        rep.say("FAIL", f"not a recording: {exc}")
        return
    recs = replay.records
    if not recs:
        rep.say("FAIL", "empty recording")
        return
    duration = recs[-1][0] - recs[0][0]
    counts: Counter = Counter()
    nbytes: Counter = Counter()
    bad = 0
    seq_last: Dict[int, int] = {}
    seq_gaps: Counter = Counter()
    seq_reorder: Counter = Counter()
    radar_frames = 0
    radar_with_target = 0
    slot_use = [0, 0, 0]
    range_hist: Counter = Counter()
    speed_hist: Counter = Counter()
    lidar_points = 0
    lidar_zero = 0
    buckets_seen = [0] * 36                 # 10-degree sectors
    bucket_dist: Dict[int, List[int]] = defaultdict(list)
    scan_speeds: List[int] = []
    status: List[Tuple[float, rp.StatusPacket]] = []
    rx_per_sec: Counter = Counter()
    for t, dgram in recs:
        decoded = rp.decode(dgram)
        if decoded is None:
            bad += 1
            continue
        ptype, seq, pkt = decoded
        counts[ptype] += 1
        nbytes[ptype] += len(dgram)
        rx_per_sec[int(t)] += 1
        if ptype in seq_last:
            last = seq_last[ptype]
            if rp.seq_newer(seq, last):
                gap = (seq - last - 1) & 0xFFFF
                if gap:
                    seq_gaps[ptype] += gap
                seq_last[ptype] = seq
            else:
                seq_reorder[ptype] += 1
        else:
            seq_last[ptype] = seq
        if ptype == rp.TYPE_RADAR:
            radar_frames += 1
            valid = [x for x in pkt.targets if x.valid]
            if valid:
                radar_with_target += 1
            for i, x in enumerate(pkt.targets):
                if x.valid:
                    slot_use[i] += 1
                    range_hist[min(16, int(x.distance_mm // 500))] += 1
                    speed_hist[max(-6, min(6, int(x.speed_cm_s // 50)))] += 1
        elif ptype == rp.TYPE_LIDAR:
            scan_speeds.append(pkt.scan_speed_deg_s)
            for pnt in pkt.points:
                lidar_points += 1
                if pnt.dist_mm == 0:
                    lidar_zero += 1
                    continue
                b = (pnt.angle_cdeg % 36000) // 1000
                buckets_seen[b] += 1
                bucket_dist[b].append(pnt.dist_mm)
        elif ptype == rp.TYPE_STATUS:
            status.append((t, pkt))
    names = {rp.TYPE_RADAR: "radar", rp.TYPE_LIDAR: "lidar", rp.TYPE_STATUS: "status", rp.TYPE_HELLO: "hello"}
    lines = [f"{len(recs)} datagrams over {duration:.1f} s, {bad} undecodable"]
    for ptype in sorted(counts):
        n = counts[ptype]
        lines.append(f"{names.get(ptype, hex(ptype))}: {n} datagrams ({n / duration if duration else 0:.1f}/s, {nbytes[ptype] / 1024:.1f} KB), "
                     f"{seq_gaps[ptype]} missing by sequence ({pct(seq_gaps[ptype], n + seq_gaps[ptype]):.2f}%), {seq_reorder[ptype]} reordered")
    rep.metrics.update(recording_seconds=round(duration, 1), datagrams=len(recs),
                       loss_pct={names.get(k, str(k)): round(pct(v, counts[k] + v), 3) for k, v in seq_gaps.items()})
    rep.sections.append(("recording", lines))
    rep.series["datagrams/s"] = (sorted(rx_per_sec), [rx_per_sec[k] for k in sorted(rx_per_sec)])
    if radar_frames:
        rl = [f"{radar_frames} frames, {radar_with_target} with a target ({pct(radar_with_target, radar_frames):.0f}%), "
              f"slot usage {slot_use}"]
        rl.append("range histogram (m):")
        rl += bars([(f"{k * 0.5:.1f}-{k * 0.5 + 0.5:.1f}" if k < 16 else "8.0+", range_hist[k]) for k in sorted(range_hist)])
        rl.append("speed histogram (cm/s, negative = approaching):")
        rl += bars([(f"{k * 50:+d}..{k * 50 + 50:+d}", speed_hist[k]) for k in sorted(speed_hist)])
        rep.sections.append(("radar", rl))
        if radar_with_target == 0:
            rep.say("INFO", "radar: no target in the whole recording: nothing moved in the sector, or the module faces an obstacle")
        rep.metrics.update(radar_frames=radar_frames, radar_target_pct=round(pct(radar_with_target, radar_frames), 1))
    if counts[rp.TYPE_LIDAR]:
        covered = sum(1 for b in buckets_seen if b)
        ll = [f"{lidar_points} points ({lidar_points / duration if duration else 0:.0f}/s), {lidar_zero} without return, "
              f"scan speed {stats(scan_speeds)} deg/s, {covered}/36 sectors of 10 deg have returns"]
        ll.append("median distance per 30-degree sector (m), clockwise from the LIDAR's zero:")
        rows = []
        for s in range(12):
            ds = [d for b in range(s * 3, s * 3 + 3) for d in bucket_dist.get(b, [])]
            rows.append((f"{s * 30:3d}-{s * 30 + 30:3d}", round(statistics.median(ds) / 1000, 2) if ds else 0.0))
        ll += bars(rows, unit=" m")
        rep.sections.append(("lidar", ll))
        if covered < 18:
            rep.say("WARN", f"lidar: only {covered} of 36 sectors ever returned: the LIDAR is boxed in, or the room is beyond 12 m / absorbing")
        med_speed = statistics.median(scan_speeds) if scan_speeds else 0
        if scan_speeds and not (LIDAR_SPEED_MIN <= med_speed <= LIDAR_SPEED_MAX):
            rep.say("WARN", f"lidar: scan speed {med_speed:g} deg/s outside {LIDAR_SPEED_MIN}..{LIDAR_SPEED_MAX}")
        rep.metrics.update(lidar_points=lidar_points, lidar_sectors=covered)
    if status:
        ups = [s.uptime_ms for _, s in status]
        resets = sum(1 for a, b in zip(ups, ups[1:]) if b < a)
        radar_ok = sum(1 for _, s in status if s.flags & rp.STATUS_RADAR_OK)
        lidar_on = any(s.flags & rp.STATUS_LIDAR_ENABLED for _, s in status)
        lidar_ok = sum(1 for _, s in status if s.flags & rp.STATUS_LIDAR_OK)
        sl = [f"{len(status)} status packets; node uptime {ups[0] / 1000:.0f}..{ups[-1] / 1000:.0f} s, {resets} reset(s); "
              f"radar OK in {radar_ok}, lidar OK in {lidar_ok} ({'enabled' if lidar_on else 'disabled'}); "
              f"node radar fps {stats([s.radar_fps for _, s in status])}, lidar fps {stats([s.lidar_fps for _, s in status])}, "
              f"subscribers {stats([s.subscribers for _, s in status])}, bad radar frames {status[-1][1].radar_bad_frames}, "
              f"lidar CRC errors {status[-1][1].lidar_crc_errors}"]
        rep.sections.append(("node status (as received)", sl))
        rep.series["node radar fps"] = ([t for t, _ in status], [s.radar_fps for _, s in status])
        if resets:
            rep.say("WARN", f"node rebooted {resets} time(s) during the recording (uptime went backwards)")
        if lidar_on and lidar_ok < len(status) * 0.9:
            rep.say("WARN", f"node reported the LIDAR healthy in only {lidar_ok} of {len(status)} status packets")
        if radar_ok < len(status) * 0.9:
            rep.say("WARN", f"node reported the radar healthy in only {radar_ok} of {len(status)} status packets")
    for ptype, gap in seq_gaps.items():
        p = pct(gap, counts[ptype] + gap)
        if p > 2:
            rep.say("WARN", f"{names.get(ptype, ptype)}: {p:.1f}% of datagrams never arrived: weak link, power save on the receiver, or a busy channel")
    if bad:
        rep.say("FAIL" if bad == len(recs) else "WARN",
                f"{bad} of {len(recs)} datagrams are not RadarLink v1 packets: a protocol mismatch or a damaged file")
    if not rep.issues:
        rep.say("OK", "recording looks healthy")


# -------------------------------------------------------------------- main
def analyze_file(path: str) -> Report:
    rep = Report(path)
    with open(path, "rb") as fh:
        data = fh.read()
    if data.startswith(rs.RECORD_MAGIC):
        analyze_recording(path, data, rep)
        return rep
    log = parse_text_log(path, data.decode("utf-8", errors="replace"))
    analyze_node(log, rep)
    rx_events = [e for e in log.events if e.kind in dict(RX_EVENTS)]
    # a serial log holds one speaker; GIGA-style events without any [stat] line still belong to the GIGA
    analyze_receiver(log.giga, rx_events if (log.giga or not (log.viewer or log.relay)) else [], "GIGA display", rep)
    analyze_receiver(log.viewer, rx_events if log.viewer else [], "uConsole viewer", rep)
    analyze_receiver(log.relay, rx_events if log.relay else [], "relay", rep)
    if not (log.node or log.giga or log.viewer or log.relay):
        rep.say("FAIL", f"no RadarLink log lines recognised in {log.lines} lines (is this a serial log of the node, GIGA or viewer?)")
    elif log.unparsed > log.lines * 0.5:
        rep.say("INFO", f"{log.unparsed} of {log.lines} lines were not RadarLink output")
    rep.metrics["lines"] = log.lines
    return rep


def plot(reports: List[Report], path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    panels = [(r.title, k, v) for r in reports for k, v in r.series.items() if v[0]]
    if not panels:
        return
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 2.2 * len(panels)), squeeze=False)
    for ax, (title, name, (xs, ys)) in zip(axes[:, 0], panels):
        ax.plot(xs, ys, linewidth=1)
        ax.set_title(f"{os.path.basename(title)}: {name}", fontsize=9, loc="left")
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("seconds")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("files", nargs="+", help="serial logs and/or .rdl recordings")
    p.add_argument("--json", action="store_true", help="metrics and diagnosis as JSON")
    p.add_argument("--plot", metavar="PNG", help="write time-series plots (needs matplotlib)")
    a = p.parse_args(argv)
    reports = [analyze_file(f) for f in a.files]
    if a.json:
        print(json.dumps([{"file": r.title, "result": r.worst, "metrics": r.metrics,
                           "issues": [{"level": lv, "text": t} for lv, t in r.issues]} for r in reports], indent=1, default=str))
    else:
        for r in reports:
            print("\n".join(r.render()))
            print()
    if a.plot:
        try:
            plot(reports, a.plot)
            print(f"plots written to {a.plot}")
        except ImportError:
            print("matplotlib is not installed; no plot written (pip install matplotlib)", file=sys.stderr)
    order = {lv: i for i, lv in enumerate(LEVELS)}
    return 1 if any(order[r.worst] >= order["FAIL"] for r in reports) else 0


if __name__ == "__main__":
    sys.exit(main())
