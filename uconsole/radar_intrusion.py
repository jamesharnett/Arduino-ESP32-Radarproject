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
