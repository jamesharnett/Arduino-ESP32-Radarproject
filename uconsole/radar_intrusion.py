"""Intrusion detection on top of the LIDAR room outline and the radar tracks.

Pure Python, no pygame, unit-tested in ``tests/test_intrusion.py``.

* ``ReferenceScan`` learns the empty room: the median LIDAR distance per
  0.5-degree bucket over a learning window, with each bucket classified as
  *reliable* (consistent return), *open* (consistently no return, e.g. a long
  corridor beyond the LIDAR's range) or *unreliable* (glass, black surfaces,
  flicker), which detection ignores.
* ``IntrusionDetector`` flags buckets whose live distance is shorter than the
  reference by more than ``delta_mm`` for at least ``dwell_s``, clusters
  adjacent buckets into blobs, tracks blobs over time and fuses them with the
  radar: a blob with a moving radar target nearby is a *fused* alert (high
  confidence), a blob alone is a *lidar* alert (something appeared, maybe a
  parcel), a radar target with no LIDAR change is a *radar* alert (motion the
  LIDAR cannot see, e.g. behind a thin door). Alerts start and end as events the
  viewer logs, sounds and can hand to a shell command.

Frames: all angles are degrees clockwise from the sensor node's forward
direction (the top of the plot); radar x is to the right and y forward, so a
radar target is at angle atan2(x, y). The LIDAR mounting offset is applied here
so the detector and the display agree.
"""
from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import radar_protocol as rp
import radar_state as rs

BUCKETS = rs.BUCKETS
DEG_PER_BUCKET = 360.0 / BUCKETS
POINTS_PER_BUCKET_PER_S = 4500.0 / BUCKETS      # LD19 at 10 Hz: about 6 returns per bucket per second


def bucket_angle_deg(i: int) -> float:
    return (i + 0.5) * DEG_PER_BUCKET


def polar_to_xy(angle_deg: float, dist_mm: float) -> Tuple[float, float]:
    a = math.radians(angle_deg)
    return dist_mm * math.sin(a), dist_mm * math.cos(a)


def xy_to_polar(x_mm: float, y_mm: float) -> Tuple[float, float]:
    return math.degrees(math.atan2(x_mm, y_mm)) % 360.0, math.hypot(x_mm, y_mm)


# ------------------------------------------------------------- reference scan
class ReferenceScan:
    def __init__(self) -> None:
        self.ref_mm = [0] * BUCKETS
        self.reliable = [False] * BUCKETS
        self.open = [False] * BUCKETS
        self.learned_at = 0.0
        self.learning_since: Optional[float] = None
        self._acc: Optional[List[List[int]]] = None

    # -- learning -----------------------------------------------------------
    def start(self, now: float) -> None:
        self._acc = [[] for _ in range(BUCKETS)]
        self.learning_since = now

    def learning(self) -> bool:
        return self._acc is not None

    def feed(self, pkt: rp.LidarPacket) -> None:
        if self._acc is None:
            return
        for p in pkt.points:
            self._acc[(p.angle_cdeg % 36000) // rs.CDEG_PER_BUCKET].append(p.dist_mm)

    def finish(self, now: float, spread_mm: int = 150) -> int:
        """Classify every bucket; returns the number of reliable buckets."""
        assert self._acc is not None and self.learning_since is not None
        duration = max(0.5, now - self.learning_since)
        expected = POINTS_PER_BUCKET_PER_S * duration
        reliable = 0
        for i, samples in enumerate(self._acc):
            n = len(samples)
            if n == 0 or n < 0.1 * expected:
                self.ref_mm[i], self.reliable[i], self.open[i] = 0, False, True
                continue
            med = int(statistics.median(samples))
            mad = statistics.median(abs(s - med) for s in samples)
            ok = n >= 0.35 * expected and mad <= spread_mm
            self.ref_mm[i], self.reliable[i], self.open[i] = med, ok, False
            reliable += ok
        self._acc = None
        self.learning_since = None
        self.learned_at = now
        return reliable

    def coverage(self) -> int:
        return sum(self.reliable) + sum(self.open)

    def has_baseline(self) -> bool:
        return self.learned_at > 0 and self.coverage() > 0

    # -- persistence --------------------------------------------------------
    def save(self, path: str) -> None:
        with open(path, "w") as fh:
            json.dump({"version": 1, "learned_at": self.learned_at, "ref_mm": self.ref_mm,
                       "reliable": self.reliable, "open": self.open}, fh)

    @classmethod
    def load(cls, path: str) -> "ReferenceScan":
        with open(path) as fh:
            d = json.load(fh)
        if d.get("version") != 1 or len(d["ref_mm"]) != BUCKETS:
            raise ValueError("not a RadarLink baseline file")
        r = cls()
        r.ref_mm, r.reliable, r.open, r.learned_at = d["ref_mm"], d["reliable"], d["open"], d["learned_at"]
        return r


# -------------------------------------------------------------------- alerts
@dataclass
class Alert:
    id: int
    kind: str                     # 'lidar' | 'fused' | 'radar'
    angle_deg: float
    dist_mm: float
    x_mm: float
    y_mm: float
    span_deg: float
    since: float
    last_seen: float
    confidence: float
    buckets: List[int] = field(default_factory=list)
    radar_slot: Optional[int] = None

    def describe(self) -> str:
        return f"{self.kind} {self.dist_mm / 1000:.1f}m @ {self.angle_deg:.0f}deg conf {self.confidence:.2f}"


class IntrusionDetector:
    def __init__(self, ref: ReferenceScan, lidar_offset_deg: float = 0.0, *, delta_mm: int = 300,
                 dwell_s: float = 0.5, min_buckets: int = 3, hold_s: float = 1.0, gap_s: float = 0.6,
                 fuse_radius_mm: float = 900.0, match_radius_mm: float = 700.0, max_range_mm: int = 12000) -> None:
        self.ref = ref
        self.lidar_offset_deg = lidar_offset_deg
        self.delta_mm = delta_mm
        self.dwell_s = dwell_s
        self.min_buckets = min_buckets
        self.hold_s = hold_s
        self.gap_s = gap_s
        self.fuse_radius_mm = fuse_radius_mm
        self.match_radius_mm = match_radius_mm
        self.max_range_mm = max_range_mm
        self.armed = False
        self.zones: List["IgnoreZone"] = []
        self.alerts: Dict[int, Alert] = {}
        self.events: List[Tuple[str, Alert]] = []     # ('start'|'end', alert), drained by the caller
        self._first_closer = [-1.0] * BUCKETS     # -1 = not currently 'closer'
        self._last_closer = [-1.0] * BUCKETS
        self._next_id = 1
        self.active_buckets: List[int] = []

    # -- helpers -------------------------------------------------------------
    def clear(self) -> None:
        for a in list(self.alerts.values()):
            self.events.append(("end", a))
        self.alerts.clear()
        self._first_closer = [-1.0] * BUCKETS
        self._last_closer = [-1.0] * BUCKETS
        self.active_buckets = []

    def drain_events(self) -> List[Tuple[str, Alert]]:
        ev, self.events = self.events, []
        return ev

    def _closer_buckets(self, now: float, lidar: rs.LidarStore) -> List[int]:
        active = []
        ref = self.ref
        for i in range(BUCKETS):
            ts = lidar.ts[i]
            closer = False
            if ts and now - ts < lidar.ttl_s:
                d = lidar.dist[i]
                if ref.reliable[i]:
                    closer = d < ref.ref_mm[i] - self.delta_mm
                elif ref.open[i]:
                    closer = d < self.max_range_mm
                if closer and self.zones:
                    a = (bucket_angle_deg(i) + self.lidar_offset_deg) % 360.0
                    if any(z.contains(a, d) for z in self.zones):
                        closer = False
            if closer:
                if self._first_closer[i] < 0.0 or now - self._last_closer[i] > self.gap_s:
                    self._first_closer[i] = now
                self._last_closer[i] = now
                if now - self._first_closer[i] >= self.dwell_s - 1e-9:
                    active.append(i)
            elif self._first_closer[i] >= 0.0 and now - self._last_closer[i] > self.gap_s:
                self._first_closer[i] = -1.0
        return active

    @staticmethod
    def _cluster(active: Sequence[int], gap: int = 1) -> List[List[int]]:
        """Group adjacent buckets (circular, tolerating ``gap`` missing buckets)."""
        if not active:
            return []
        s = sorted(set(active))
        groups: List[List[int]] = [[s[0]]]
        for b in s[1:]:
            if b - groups[-1][-1] <= gap + 1:
                groups[-1].append(b)
            else:
                groups.append([b])
        if len(groups) > 1 and (s[0] + BUCKETS) - groups[-1][-1] <= gap + 1:   # wrap around 360
            groups[0] = groups.pop() + groups[0]
        return groups

    def _blob_geometry(self, buckets: List[int], lidar: rs.LidarStore) -> Tuple[float, float, float, float, float]:
        sx = sy = 0.0
        for b in buckets:
            a = (bucket_angle_deg(b) + self.lidar_offset_deg) % 360.0
            x, y = polar_to_xy(a, lidar.dist[b])
            sx += x
            sy += y
        n = len(buckets)
        cx, cy = sx / n, sy / n
        angle, dist = xy_to_polar(cx, cy)
        return angle, dist, cx, cy, n * DEG_PER_BUCKET

    # -- main ----------------------------------------------------------------
    def update(self, now: float, lidar: rs.LidarStore, radar: rs.RadarTracks) -> None:
        if not self.armed or self.ref.learning() or not self.ref.has_baseline():
            if self.alerts or self.active_buckets:
                self.clear()
            return

        self.active_buckets = self._closer_buckets(now, lidar)
        blobs = [g for g in self._cluster(self.active_buckets, gap=2) if len(g) >= self.min_buckets]
        targets = [(slot, float(t.x_mm), float(t.y_mm)) for slot, t in radar.shown(now)]
        if self.zones:
            targets = [(slot, x, y) for slot, x, y in targets if not any(z.contains(*xy_to_polar(x, y)) for z in self.zones)]

        # geometry of every blob and the radar target (if any) moving inside it
        geoms = []
        for g in blobs:
            angle, dist, cx, cy, span = self._blob_geometry(g, lidar)
            near = sorted((math.hypot(cx - x, cy - y), slot) for slot, x, y in targets)
            slot = near[0][1] if near and near[0][0] <= self.fuse_radius_mm else None
            geoms.append((g, cx, cy, slot))

        # assign blobs to existing alerts: nearest within match radius, or the radar
        # alert of the same target (that is how a radar-first alert becomes fused).
        # Several fragments of one body may land on the same alert and are merged.
        assigned: Dict[int, list] = {}
        fresh = []
        for geom in geoms:
            g, cx, cy, slot = geom
            best = None
            for a in self.alerts.values():
                d = math.hypot(a.x_mm - cx, a.y_mm - cy)
                if a.kind == "radar" and slot is not None and a.radar_slot == slot:
                    d = 0.0
                if d <= self.match_radius_mm and (best is None or d < best[0]):
                    best = (d, a)
            if best is None:
                fresh.append(geom)
            else:
                assigned.setdefault(best[1].id, []).append(geom)

        seen_ids = set()
        used_slots = set()

        def apply(a: Alert, buckets: List[int], slot: Optional[int]) -> None:
            angle, dist, cx, cy, span = self._blob_geometry(buckets, lidar)
            kind = "fused" if slot is not None else "lidar"
            conf = 0.95 if slot is not None else (0.7 if len(buckets) >= 2 * self.min_buckets else 0.6)
            a.kind, a.angle_deg, a.dist_mm, a.x_mm, a.y_mm, a.span_deg = kind, angle, dist, cx, cy, span
            a.confidence = max(a.confidence, conf)
            a.buckets, a.radar_slot, a.last_seen = buckets, slot, now
            if slot is not None:
                used_slots.add(slot)
            seen_ids.add(a.id)

        for aid, parts in assigned.items():
            buckets = sorted({b for part in parts for b in part[0]})
            slots = [part[3] for part in parts if part[3] is not None]
            apply(self.alerts[aid], buckets, slots[0] if slots else None)

        for g, cx, cy, slot in fresh:
            a = Alert(self._next_id, "lidar", 0.0, 0.0, cx, cy, 0.0, now, now, 0.0, list(g), slot)
            self._next_id += 1
            apply(a, list(g), slot)
            self.alerts[a.id] = a
            self.events.append(("start", a))

        # radar targets with no LIDAR change nearby: motion the LIDAR cannot see
        for slot, x, y in targets:
            if slot in used_slots:
                continue
            angle, dist = xy_to_polar(x, y)
            existing = next((a for a in self.alerts.values() if a.kind == "radar" and a.radar_slot == slot), None)
            if existing is None:
                a = Alert(self._next_id, "radar", angle, dist, x, y, 0.0, now, now, 0.5, [], slot)
                self._next_id += 1
                self.alerts[a.id] = a
                self.events.append(("start", a))
                seen_ids.add(a.id)
            else:
                existing.angle_deg, existing.dist_mm, existing.x_mm, existing.y_mm, existing.last_seen = angle, dist, x, y, now
                seen_ids.add(existing.id)

        for a in list(self.alerts.values()):
            if a.id not in seen_ids and now - a.last_seen > self.hold_s:
                self.events.append(("end", a))
                del self.alerts[a.id]

    def active(self) -> List[Alert]:
        return sorted(self.alerts.values(), key=lambda a: a.since)


# --------------------------------------------------------------- ignore zones
@dataclass
class IgnoreZone:
    """A wedge (angles clockwise from forward, may wrap through 360) and range band to ignore."""
    angle_from_deg: float
    angle_to_deg: float
    dist_min_mm: float = 0.0
    dist_max_mm: float = 1e9

    def contains(self, angle_deg: float, dist_mm: float) -> bool:
        a, f, t = angle_deg % 360.0, self.angle_from_deg % 360.0, self.angle_to_deg % 360.0
        in_angle = (f <= a <= t) if f <= t else (a >= f or a <= t)
        return in_angle and self.dist_min_mm <= dist_mm <= self.dist_max_mm

    @classmethod
    def parse(cls, text: str) -> "IgnoreZone":
        """``A1:A2`` or ``A1:A2:DMIN:DMAX`` with angles in degrees and distances in metres."""
        parts = text.split(":")
        if len(parts) not in (2, 4):
            raise ValueError(f"ignore zone must be A1:A2 or A1:A2:DMIN:DMAX, got {text!r}")
        a1, a2 = float(parts[0]), float(parts[1])
        if len(parts) == 2:
            return cls(a1, a2)
        return cls(a1, a2, float(parts[2]) * 1000.0, float(parts[3]) * 1000.0)

    def describe(self) -> str:
        rng = "" if self.dist_max_mm >= 1e8 else f" {self.dist_min_mm / 1000:g}-{self.dist_max_mm / 1000:g} m"
        return f"{self.angle_from_deg:g}-{self.angle_to_deg:g} deg{rng}"


# --------------------------------------------------------------- arm schedule
class ArmSchedule:
    """Daily arming window ``HH:MM-HH:MM`` in local time; may wrap past midnight."""

    def __init__(self, text: str) -> None:
        try:
            start, end = text.split("-")
            sh, sm = (int(x) for x in start.split(":"))
            eh, em = (int(x) for x in end.split(":"))
        except ValueError as exc:
            raise ValueError(f"arm schedule must be HH:MM-HH:MM, got {text!r}") from exc
        if not (0 <= sh < 24 and 0 <= eh < 24 and 0 <= sm < 60 and 0 <= em < 60):
            raise ValueError(f"arm schedule out of range: {text!r}")
        self.start_min = sh * 60 + sm
        self.end_min = eh * 60 + em
        self.text = text
        self._last: Optional[bool] = None

    def active(self, when) -> bool:
        m = when.hour * 60 + when.minute
        if self.start_min <= self.end_min:
            return self.start_min <= m < self.end_min
        return m >= self.start_min or m < self.end_min

    def poll(self, when) -> Optional[bool]:
        """The new armed state when the window boundary was crossed since the last poll, else None."""
        state = self.active(when)
        if state != self._last:
            self._last = state
            return state
        return None


# ------------------------------------------------------------------ snapshot
def snapshot(now: float, lidar: rs.LidarStore, radar: rs.RadarTracks, alert: Alert, lidar_offset_deg: float) -> dict:
    """Everything needed to review an alert later: live buckets, radar targets, the alert."""
    buckets = [[round(bucket_angle_deg(i) + lidar_offset_deg, 2), d, inten, round(age, 3)]
               for i, d, inten, age in lidar.visible(now)]
    targets = [{"slot": slot, "x_mm": t.x_mm, "y_mm": t.y_mm, "speed_cm_s": t.speed_cm_s} for slot, t in radar.shown(now)]
    a = {k: v for k, v in alert.__dict__.items() if k != "buckets"}
    a["buckets"] = list(alert.buckets)
    return {"version": 1, "monotonic": now, "alert": a, "lidar": buckets, "radar": targets}


# ------------------------------------------------------- detection runtime
class DetectionRuntime:
    """Everything the viewer and the headless relay share: baseline lifecycle,
    arming (manual and scheduled), the detector, alert logging, snapshots and
    the alert command. Audio and drawing stay with the caller."""

    @staticmethod
    def add_arguments(parser) -> None:
        import argparse  # local import keeps this module importable without argparse users
        g = parser.add_argument_group("intrusion detection")
        g.add_argument("--baseline", default="baseline.json", metavar="FILE", help="empty-room reference scan to load/save ('' to disable)")
        g.add_argument("--learn", type=float, default=0.0, metavar="S", help="learn the baseline for S seconds after data arrives, then arm")
        g.add_argument("--armed", action="store_true", help="start armed (needs a baseline)")
        g.add_argument("--arm-schedule", metavar="HH:MM-HH:MM", help="arm automatically inside this daily window (local time)")
        g.add_argument("--ignore-zone", action="append", default=[], metavar="A1:A2[:DMIN:DMAX]",
                       help="ignore changes in this wedge (degrees clockwise from forward, metres); repeatable")
        g.add_argument("--alert-log", default="alerts.log", metavar="FILE", help="append alert start/end rows as CSV ('' to disable)")
        g.add_argument("--alert-cmd", metavar="CMD", help="shell command run on each alert start; ALERT_KIND, ALERT_DIST_M, ALERT_ANGLE, ... in its environment")
        g.add_argument("--snapshot-dir", metavar="DIR", help="write a JSON snapshot of the scene at every alert start")
        g.add_argument("--delta-mm", type=int, default=300, help="a return must be this much closer than the baseline")
        g.add_argument("--dwell-s", type=float, default=0.5, help="... for at least this long")
        g.add_argument("--min-buckets", type=int, default=3, help="minimum width of a change in 0.5-degree buckets")

    def __init__(self, args, lidar_offset_deg: float, log=print) -> None:
        import os
        self.args = args
        self.log = log
        self.lidar_offset_deg = lidar_offset_deg
        self.ref = ReferenceScan()
        if args.baseline and os.path.exists(args.baseline):
            try:
                self.ref = ReferenceScan.load(args.baseline)
                self.log(f"[detect] baseline loaded from {args.baseline}: {sum(self.ref.reliable)} reliable, {sum(self.ref.open)} open buckets")
            except (OSError, ValueError, KeyError) as exc:
                self.log(f"[detect] could not load {args.baseline}: {exc}")
        self.detector = IntrusionDetector(self.ref, lidar_offset_deg=lidar_offset_deg, delta_mm=args.delta_mm,
                                          dwell_s=args.dwell_s, min_buckets=args.min_buckets)
        self.detector.zones = [IgnoreZone.parse(z) for z in args.ignore_zone]
        for z in self.detector.zones:
            self.log(f"[detect] ignoring {z.describe()}")
        self.schedule = ArmSchedule(args.arm_schedule) if args.arm_schedule else None
        self.detector.armed = bool(args.armed and self.ref.has_baseline())
        if args.armed and not self.ref.has_baseline():
            self.log("[detect] --armed ignored: no baseline yet (use --learn or press B)")
        self.learn_pending = args.learn if args.learn and args.learn > 0 else 0.0
        self.learn_duration = 0.0
        self.arm_after_learn = False
        self.alert_log = None
        if args.alert_log:
            new = not os.path.exists(args.alert_log) or os.path.getsize(args.alert_log) == 0
            self.alert_log = open(args.alert_log, "a")
            if new:
                self.alert_log.write("time,event,id,kind,angle_deg,dist_m,x_m,y_m,span_deg,confidence\n")
        if args.snapshot_dir:
            os.makedirs(args.snapshot_dir, exist_ok=True)
        self.last_schedule_check = 0.0
        self.events_seen = 0

    # -- lifecycle -------------------------------------------------------------
    @property
    def armed(self) -> bool:
        return self.detector.armed

    def set_armed(self, state: bool) -> bool:
        if state and not self.ref.has_baseline():
            self.log("[detect] cannot arm: no baseline yet (learn one first)")
            return False
        self.detector.armed = state
        self.log(f"[detect] {'ARMED' if state else 'disarmed'}")
        return True

    def toggle_armed(self) -> None:
        self.set_armed(not self.detector.armed)

    def start_learning(self, now: float, seconds: float, arm_after: bool) -> None:
        if self.ref.learning():
            return
        self.ref.start(now)
        self.learn_duration = seconds
        self.arm_after_learn = arm_after
        self.log(f"[detect] learning the empty room for {seconds:g} s - keep it empty")

    def learning_left(self, now: float) -> Optional[float]:
        if not self.ref.learning() or self.ref.learning_since is None:
            return None
        return max(0.0, self.learn_duration - (now - self.ref.learning_since))

    def on_lidar_packet(self, pkt: rp.LidarPacket) -> None:
        if self.ref.learning():
            self.ref.feed(pkt)

    def tick(self, now: float, data_alive: bool, lidar: rs.LidarStore, radar: rs.RadarTracks) -> List[Tuple[str, Alert]]:
        """Advance learning, scheduling and detection; returns the alert events of this tick."""
        import datetime
        if self.learn_pending and data_alive:
            self.start_learning(now, self.learn_pending, arm_after=True)
            self.learn_pending = 0.0
        if self.ref.learning() and self.ref.learning_since is not None and now - self.ref.learning_since >= self.learn_duration:
            n = self.ref.finish(now)
            self.log(f"[detect] baseline learned: {n} reliable, {sum(self.ref.open)} open buckets")
            if self.args.baseline:
                try:
                    self.ref.save(self.args.baseline)
                    self.log(f"[detect] baseline saved to {self.args.baseline}")
                except OSError as exc:
                    self.log(f"[detect] could not save baseline: {exc}")
            if self.arm_after_learn or (self.schedule is not None and self.schedule.active(datetime.datetime.now())):
                self.set_armed(True)          # a schedule that wanted to arm before the baseline existed
        if self.schedule is not None and now - self.last_schedule_check >= 1.0:
            self.last_schedule_check = now
            change = self.schedule.poll(datetime.datetime.now())
            if change is not None and change != self.detector.armed:
                self.log(f"[detect] schedule {self.schedule.text}: {'arming' if change else 'disarming'}")
                self.set_armed(change)
        self.detector.update(now, lidar, radar)
        events = self.detector.drain_events()
        for event, alert in events:
            self.on_alert_event(event, alert, now, lidar, radar)
        return events

    def on_alert_event(self, event: str, a: Alert, now: float, lidar: rs.LidarStore, radar: rs.RadarTracks) -> None:
        import datetime
        import json as _json
        import os
        import subprocess
        self.events_seen += 1
        stamp = datetime.datetime.now().isoformat(timespec="seconds")
        self.log(f"[alert] {event.upper()} #{a.id} {a.describe()}")
        if self.alert_log is not None:
            self.alert_log.write(f"{stamp},{event},{a.id},{a.kind},{a.angle_deg:.1f},{a.dist_mm / 1000:.2f},"
                                 f"{a.x_mm / 1000:.2f},{a.y_mm / 1000:.2f},{a.span_deg:.1f},{a.confidence:.2f}\n")
            self.alert_log.flush()
        if event != "start":
            return
        if self.args.snapshot_dir:
            path = os.path.join(self.args.snapshot_dir, f"alert-{stamp.replace(':', '')}-{a.id}.json")
            try:
                with open(path, "w") as fh:
                    _json.dump(snapshot(now, lidar, radar, a, self.lidar_offset_deg), fh)
            except OSError as exc:
                self.log(f"[alert] snapshot failed: {exc}")
        if self.args.alert_cmd:
            env = dict(os.environ, ALERT_EVENT=event, ALERT_ID=str(a.id), ALERT_KIND=a.kind,
                       ALERT_ANGLE=f"{a.angle_deg:.1f}", ALERT_DIST_M=f"{a.dist_mm / 1000:.2f}",
                       ALERT_X_M=f"{a.x_mm / 1000:.2f}", ALERT_Y_M=f"{a.y_mm / 1000:.2f}",
                       ALERT_CONF=f"{a.confidence:.2f}")
            try:
                subprocess.Popen(self.args.alert_cmd, shell=True, env=env)
            except OSError as exc:
                self.log(f"[alert] command failed: {exc}")

    def status_text(self, now: float) -> Tuple[str, str]:
        """(text, level) for a HUD line; level is 'warn', 'alert' or 'dim'."""
        left = self.learning_left(now)
        if left is not None:
            return f"LEARNING BASELINE {left:4.1f}s  keep the room empty", "warn"
        if self.ref.has_baseline():
            sched = f"  schedule {self.schedule.text}" if self.schedule else ""
            return (f"{'ARMED' if self.detector.armed else 'disarmed'}  baseline {sum(self.ref.reliable)} buckets{sched}",
                    "alert" if self.detector.armed else "dim")
        return "no baseline: press B with the room empty", "dim"

    def close(self) -> None:
        if self.alert_log is not None:
            self.alert_log.close()
            self.alert_log = None
