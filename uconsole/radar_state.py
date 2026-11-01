"""Pure-Python state handling for the RadarLink viewer (no pygame, unit-testable).

* ``SeqTracker``  drops reordered datagrams and resynchronises after a sender reboot
* ``LidarStore``  keeps the newest LIDAR return per 0.5-degree bucket with its age
* ``RadarTracks`` applies the hold time and the "stopped moving" ghost filter
* ``Recorder`` / ``Replayer`` write and read raw datagram captures for offline work
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import BinaryIO, Dict, Iterator, List, Optional, Tuple

import radar_protocol as rp

BUCKETS = 720                       # 0.5 degree per bucket
CDEG_PER_BUCKET = 36000 // BUCKETS


class SeqTracker:
    """Per-type sequence check with the same rules as the GIGA display."""

    def __init__(self, resync_after: int = 5) -> None:
        self.resync_after = resync_after
        self.last: Dict[int, int] = {}
        self.streak: Dict[int, int] = {}
        self.dropped = 0

    def reset(self) -> None:
        self.last.clear()
        self.streak.clear()

    def accept(self, ptype: int, seq: int) -> bool:
        last = self.last.get(ptype)
        if last is None or rp.seq_newer(seq, last):
            self.last[ptype] = seq
            self.streak[ptype] = 0
            return True
        self.streak[ptype] = self.streak.get(ptype, 0) + 1
        if self.streak[ptype] >= self.resync_after:      # sender rebooted, counters restarted
            self.last[ptype] = seq
            self.streak[ptype] = 0
            return True
        self.dropped += 1
        return False


class LidarStore:
    def __init__(self, ttl_s: float = 0.5) -> None:
        self.ttl_s = ttl_s
        self.dist = [0] * BUCKETS
        self.intensity = [0] * BUCKETS
        self.ts = [0.0] * BUCKETS
        self.last_update = 0.0
        self.points_received = 0

    def clear(self) -> None:
        self.ts = [0.0] * BUCKETS

    def update(self, pkt: rp.LidarPacket, now: float) -> None:
        self.last_update = now
        for p in pkt.points:
            idx = (p.angle_cdeg % 36000) // CDEG_PER_BUCKET
            self.dist[idx] = p.dist_mm
            self.intensity[idx] = p.intensity
            self.ts[idx] = now
        self.points_received += len(pkt.points)

    def visible(self, now: float) -> Iterator[Tuple[int, int, int, float]]:
        """Yield ``(bucket, dist_mm, intensity, age_fraction)`` for live buckets; age 0 = fresh, 1 = about to expire."""
        ttl = self.ttl_s
        for i in range(BUCKETS):
            ts = self.ts[i]
            if not ts:
                continue
            age = now - ts
            if age > ttl:
                continue
            yield i, self.dist[i], self.intensity[i], age / ttl


@dataclass
class Track:
    x_mm: int = 0
    y_mm: int = 0
    still_since: float = 0.0
    have: bool = False


class RadarTracks:
    def __init__(self, hold_s: float = 0.4, still_timeout_s: float = 2.0, move_thresh_mm: int = 15) -> None:
        self.hold_s = hold_s
        self.still_timeout_s = still_timeout_s
        self.move_thresh_mm = move_thresh_mm
        self.packet: Optional[rp.RadarPacket] = None
        self.last_update = 0.0
        self.tracks = [Track() for _ in range(rp.RADAR_SLOTS)]

    def update(self, pkt: rp.RadarPacket, now: float) -> None:
        self.packet = pkt
        self.last_update = now
        for i, t in enumerate(pkt.targets):
            tr = self.tracks[i]
            if not t.valid:
                tr.have = False
                continue
            moved = (not tr.have or abs(t.x_mm - tr.x_mm) > self.move_thresh_mm
                     or abs(t.y_mm - tr.y_mm) > self.move_thresh_mm)
            if moved:
                tr.still_since = now
            tr.x_mm, tr.y_mm, tr.have = t.x_mm, t.y_mm, True

    def shown(self, now: float) -> List[Tuple[int, rp.RadarTarget]]:
        """Targets to draw: within the hold time and, if enabled, not frozen for too long."""
        if self.packet is None or now - self.last_update > self.hold_s:
            return []
        out = []
        for i, t in enumerate(self.packet.targets):
            if not t.valid:
                continue
            tr = self.tracks[i]
            if self.still_timeout_s > 0 and tr.have and now - tr.still_since > self.still_timeout_s:
                continue
            out.append((i, t))
        return out

    def closest_mm(self, now: float) -> Optional[float]:
        shown = self.shown(now)
        if not shown:
            return None
        return min(t.distance_mm for _, t in shown)


# ------------------------------------------------------------ record / replay
RECORD_MAGIC = b"RDLREC1\n"
_RECORD_HDR = struct.Struct("<dH")     # seconds since start, datagram length


class Recorder:
    def __init__(self, fh: BinaryIO, start: float) -> None:
        self.fh = fh
        self.start = start
        self.count = 0
        fh.write(RECORD_MAGIC)

    def write(self, now: float, data: bytes) -> None:
        self.fh.write(_RECORD_HDR.pack(now - self.start, len(data)))
        self.fh.write(data)
        self.count += 1

    def close(self) -> None:
        self.fh.close()


class Replayer:
    """Hands back datagrams on the original timeline, scaled by ``speed``."""

    def __init__(self, fh: BinaryIO, speed: float = 1.0, loop: bool = False) -> None:
        if fh.read(len(RECORD_MAGIC)) != RECORD_MAGIC:
            raise ValueError("not a RadarLink recording")
        self.records: List[Tuple[float, bytes]] = []
        while True:
            hdr = fh.read(_RECORD_HDR.size)
            if len(hdr) < _RECORD_HDR.size:
                break
            t, n = _RECORD_HDR.unpack(hdr)
            data = fh.read(n)
            if len(data) < n:
                break
            self.records.append((t, data))
        self.speed = speed
        self.loop = loop
        self.pos = 0
        self.t0: Optional[float] = None
        self.finished = not self.records

    def due(self, now: float) -> List[bytes]:
        if self.finished:
            return []
        if self.t0 is None:
            self.t0 = now
        out = []
        elapsed = (now - self.t0) * self.speed
        while self.pos < len(self.records) and self.records[self.pos][0] <= elapsed:
            out.append(self.records[self.pos][1])
            self.pos += 1
        if self.pos >= len(self.records):
            if self.loop:
                self.pos = 0
                self.t0 = now
            else:
                self.finished = True
        return out
