"""Python mirror of the sensor node's UART parsers (``rd03d_parser.h``, ``ld19_parser.h``).

Lets a PC decode what the sensors send without an ESP32 in between:

* ``tools/sensor_probe.py`` reads an RD-03D or LD19 straight from a USB-UART adapter
* ``tools/decode_capture.py`` decodes the hex dump the node prints for ``raw radar`` / ``raw lidar``
* ``uconsole/tests/test_sensors.py`` feeds the same byte streams to this module and to
  the C++ headers and compares every decoded value, so the two stay in step.

Only the standard library is used.
"""
from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import radar_protocol as rp

# ------------------------------------------------------------------ RD-03D
RD03D_BAUD = 256000
RD03D_HEADER = bytes((0xAA, 0xFF, 0x03, 0x00))
RD03D_TAIL = bytes((0x55, 0xCC))
RD03D_PAYLOAD_LEN = 24
RD03D_FRAME_LEN = len(RD03D_HEADER) + RD03D_PAYLOAD_LEN + len(RD03D_TAIL)       # 30
RD03D_SLOTS = 3

RD03D_CMD_HEADER = bytes((0xFD, 0xFC, 0xFB, 0xFA))
RD03D_CMD_TAIL = bytes((0x04, 0x03, 0x02, 0x01))
RD03D_CMD_ENABLE_CONFIG = RD03D_CMD_HEADER + bytes((0x04, 0x00, 0xFF, 0x00, 0x01, 0x00)) + RD03D_CMD_TAIL
RD03D_CMD_MULTI_TARGET = RD03D_CMD_HEADER + bytes((0x02, 0x00, 0x90, 0x00)) + RD03D_CMD_TAIL
RD03D_CMD_SINGLE_TARGET = RD03D_CMD_HEADER + bytes((0x02, 0x00, 0x80, 0x00)) + RD03D_CMD_TAIL
RD03D_CMD_END_CONFIG = RD03D_CMD_HEADER + bytes((0x02, 0x00, 0xFE, 0x00)) + RD03D_CMD_TAIL
RD03D_CMD_NAMES = {0x00FF: "enable configuration", 0x00FE: "end configuration",
                   0x0090: "multi-target mode", 0x0080: "single-target mode"}
RD03D_SENTINEL_SPEEDS = (0, 248, 256)


def rd03d_decode(raw: int) -> int:
    """Sign-magnitude: bit 15 set means positive."""
    return (raw & 0x7FFF) if raw & 0x8000 else -(raw & 0x7FFF)


def rd03d_encode(value: int) -> int:
    if not -0x7FFF <= value <= 0x7FFF:
        raise ValueError("out of range")
    return (0x8000 | value) if value >= 0 else -value


def rd03d_is_sentinel_speed(speed: int) -> bool:
    return abs(speed) in RD03D_SENTINEL_SPEEDS


@dataclass
class Rd03dSlot:
    """One of the three 8-byte target slots, decoded but not filtered."""
    x_mm: int = 0
    y_mm: int = 0
    speed_cm_s: int = 0
    resolution_mm: int = 0
    empty: bool = True

    @property
    def distance_mm(self) -> float:
        return math.hypot(self.x_mm, self.y_mm)


@dataclass
class Rd03dFrame:
    payload: bytes                       # the 24 bytes between header and tail
    offset: int = 0                      # byte offset of the header in the stream (informational)

    def slots(self) -> List[Rd03dSlot]:
        out = []
        for i in range(RD03D_SLOTS):
            b = self.payload[i * 8:(i + 1) * 8]
            if not any(b):
                out.append(Rd03dSlot())
                continue
            out.append(Rd03dSlot(rd03d_decode(b[0] | b[1] << 8), rd03d_decode(b[2] | b[3] << 8),
                                 rd03d_decode(b[4] | b[5] << 8), b[6] | b[7] << 8, False))
        return out


@dataclass
class Rd03dAck:
    """Reply to a configuration command: FD FC FB FA | len | cmd|0x0100 | status | data | 04 03 02 01."""
    command: int
    status: int
    data: bytes
    offset: int = 0

    @property
    def name(self) -> str:
        return RD03D_CMD_NAMES.get(self.command & 0x00FF, f"command 0x{self.command & 0xFF:02X}")

    @property
    def ok(self) -> bool:
        return self.status == 0


@dataclass
class Rd03dOptions:
    min_dist_mm: int = 100
    max_dist_mm: int = 8000
    cluster_dist_mm: int = 1000          # 0 = off
    filter_sentinels: bool = True


def _c_half(v: int) -> int:
    """Integer division by two with C truncation toward zero."""
    return -((-v) // 2) if v < 0 else v // 2


def _f32(v: float) -> float:
    """Round to IEEE single precision, the way the firmware's float arithmetic does."""
    return struct.unpack("f", struct.pack("f", v))[0]


def f32_distance(x: int, y: int) -> float:
    """sqrtf((float)x*(float)x + (float)y*(float)y) exactly as rd03d_parser.h computes it.

    On the 8 m boundary the single-precision sum can round down (for example
    (1, 8000) sums to 64 000 000 instead of 64 000 001), which keeps a target the
    double-precision hypot would have dropped; the mirror must agree with the node.
    """
    return _f32(math.sqrt(_f32(_f32(float(x) * float(x)) + _f32(float(y) * float(y)))))


def rd03d_process(frame: Rd03dFrame, options: Rd03dOptions = Rd03dOptions()) -> List[rp.RadarTarget]:
    """What the node puts on the wire for this frame (mirror of ``rd03dProcess``)."""
    targets = [rp.RadarTarget() for _ in range(RD03D_SLOTS)]
    for i, slot in enumerate(frame.slots()):
        if slot.empty:
            continue
        dist = f32_distance(slot.x_mm, slot.y_mm)
        if dist <= options.min_dist_mm or dist > options.max_dist_mm:
            continue
        if options.filter_sentinels and rd03d_is_sentinel_speed(slot.speed_cm_s):
            continue
        targets[i] = rp.RadarTarget(slot.x_mm, slot.y_mm, slot.speed_cm_s, True)
    if options.cluster_dist_mm <= 0:
        return targets
    for i in range(RD03D_SLOTS):
        if not targets[i].valid:
            continue
        for j in range(i + 1, RD03D_SLOTS):
            if not targets[j].valid:
                continue
            if f32_distance(targets[i].x_mm - targets[j].x_mm, targets[i].y_mm - targets[j].y_mm) >= options.cluster_dist_mm:
                continue
            targets[i].x_mm = _c_half(targets[i].x_mm + targets[j].x_mm)
            targets[i].y_mm = _c_half(targets[i].y_mm + targets[j].y_mm)
            if abs(targets[j].speed_cm_s) > abs(targets[i].speed_cm_s):
                targets[i].speed_cm_s = targets[j].speed_cm_s
            targets[j] = rp.RadarTarget()
    return targets


class Rd03dParser:
    """Byte-wise state machine, identical to the C++ one (including its resync rule)."""

    def __init__(self) -> None:
        self.frames = 0
        self.bad_frames = 0
        self.acks = 0
        self.bytes_fed = 0
        self._in_frame = False
        self._hdr_idx = 0
        self._buf = bytearray()
        self._ack = bytearray()          # separate, tolerant matcher for configuration replies
        self._frame_start = 0

    def reset(self) -> None:
        self._in_frame = False
        self._hdr_idx = 0
        self._buf = bytearray()
        self._ack = bytearray()

    def feed(self, c: int) -> Optional[Rd03dFrame]:
        self.bytes_fed += 1
        if not self._in_frame:
            if c == RD03D_HEADER[self._hdr_idx]:
                self._hdr_idx += 1
                if self._hdr_idx == len(RD03D_HEADER):
                    self._hdr_idx = 0
                    self._in_frame = True
                    self._buf = bytearray()
                    self._frame_start = self.bytes_fed - len(RD03D_HEADER)
            else:
                self._hdr_idx = 1 if c == RD03D_HEADER[0] else 0
            return None
        self._buf.append(c)
        if len(self._buf) < RD03D_PAYLOAD_LEN + len(RD03D_TAIL):
            return None
        self._in_frame = False
        if bytes(self._buf[RD03D_PAYLOAD_LEN:]) == RD03D_TAIL:
            self.frames += 1
            return Rd03dFrame(bytes(self._buf[:RD03D_PAYLOAD_LEN]), self._frame_start)
        self.bad_frames += 1
        return None

    def feed_bytes(self, data: Iterable[int]) -> List[Rd03dFrame]:
        out = []
        for c in data:
            f = self.feed(c)
            if f is not None:
                out.append(f)
        return out


def rd03d_find_acks(data: bytes) -> List[Rd03dAck]:
    """Configuration replies anywhere in a byte stream (the node drains them, a direct probe sees them)."""
    out = []
    pos = 0
    while True:
        i = data.find(RD03D_CMD_HEADER, pos)
        if i < 0 or i + 6 > len(data):
            break
        length = data[i + 4] | data[i + 5] << 8
        end = i + 6 + length
        if length >= 4 and end + len(RD03D_CMD_TAIL) <= len(data) and data[end:end + 4] == RD03D_CMD_TAIL:
            body = data[i + 6:end]
            out.append(Rd03dAck(body[0] | body[1] << 8, body[2] | body[3] << 8, bytes(body[4:]), i))
            pos = end + 4
        else:
            pos = i + 1
    return out


def rd03d_build_slot(x_mm: int, y_mm: int, speed_cm_s: int, resolution_mm: int = 360) -> bytes:
    ex, ey, es = rd03d_encode(x_mm), rd03d_encode(y_mm), rd03d_encode(speed_cm_s)
    return bytes((ex & 0xFF, ex >> 8, ey & 0xFF, ey >> 8, es & 0xFF, es >> 8, resolution_mm & 0xFF, resolution_mm >> 8))


def rd03d_build_frame(slots: Sequence[Optional[Tuple[int, int, int]]]) -> bytes:
    """A 30-byte report; ``slots`` holds up to three ``(x, y, speed)`` or ``None`` for empty."""
    payload = b"".join(rd03d_build_slot(*s) if s else bytes(8) for s in list(slots) + [None] * (RD03D_SLOTS - len(slots)))
    return RD03D_HEADER + payload + RD03D_TAIL


# -------------------------------------------------------------------- LD19
LD19_BAUD = 230400
LD19_FRAME_LEN = 47
LD19_POINTS = 12
LD19_HEADER = 0x54
LD19_VERLEN = 0x2C
LD19_FULL_CIRCLE = 36000


def _crc_table() -> Tuple[int, ...]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return tuple(table)


LD19_CRC_TABLE = _crc_table()


def ld19_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = LD19_CRC_TABLE[crc ^ b]
    return crc


def ld19_point_angle_cdeg(start_cdeg: int, end_cdeg: int, i: int) -> int:
    s = start_cdeg % LD19_FULL_CIRCLE
    e = end_cdeg % LD19_FULL_CIRCLE
    span = (e + LD19_FULL_CIRCLE - s) % LD19_FULL_CIRCLE
    return (s + span * i // (LD19_POINTS - 1)) % LD19_FULL_CIRCLE


@dataclass
class Ld19Frame:
    speed_deg_s: int
    start_angle_cdeg: int
    end_angle_cdeg: int
    timestamp_ms: int
    dist_mm: List[int] = field(default_factory=list)
    intensity: List[int] = field(default_factory=list)
    offset: int = 0

    def points(self) -> List[Tuple[int, int, int]]:
        """``(angle_cdeg, dist_mm, intensity)`` for all 12 points, including zero-distance ones."""
        return [(ld19_point_angle_cdeg(self.start_angle_cdeg, self.end_angle_cdeg, i), self.dist_mm[i], self.intensity[i])
                for i in range(LD19_POINTS)]

    @property
    def span_cdeg(self) -> int:
        return (self.end_angle_cdeg - self.start_angle_cdeg) % LD19_FULL_CIRCLE


class Ld19Parser:
    def __init__(self) -> None:
        self.frames = 0
        self.crc_errors = 0
        self.bytes_fed = 0
        self._buf = bytearray()
        self._frame_start = 0

    def reset(self) -> None:
        self._buf = bytearray()

    def feed(self, c: int) -> Optional[Ld19Frame]:
        self.bytes_fed += 1
        n = len(self._buf)
        if n == 0:
            if c == LD19_HEADER:
                self._buf.append(c)
                self._frame_start = self.bytes_fed - 1
            return None
        if n == 1:
            if c == LD19_VERLEN:
                self._buf.append(c)
            elif c == LD19_HEADER:
                self._frame_start = self.bytes_fed - 1
            else:
                self._buf = bytearray()
            return None
        self._buf.append(c)
        if len(self._buf) < LD19_FRAME_LEN:
            return None
        buf = bytes(self._buf)
        self._buf = bytearray()
        if ld19_crc8(buf[:-1]) != buf[-1]:
            self.crc_errors += 1
            return None
        self.frames += 1
        return Ld19Frame(buf[2] | buf[3] << 8, buf[4] | buf[5] << 8, buf[42] | buf[43] << 8, buf[44] | buf[45] << 8,
                         [buf[6 + 3 * i] | buf[7 + 3 * i] << 8 for i in range(LD19_POINTS)],
                         [buf[8 + 3 * i] for i in range(LD19_POINTS)], self._frame_start)

    def feed_bytes(self, data: Iterable[int]) -> List[Ld19Frame]:
        out = []
        for c in data:
            f = self.feed(c)
            if f is not None:
                out.append(f)
        return out


def ld19_build_frame(speed_deg_s: int, start_cdeg: int, end_cdeg: int, dist_mm: Sequence[int],
                     intensity: Sequence[int], timestamp_ms: int, corrupt_crc: bool = False) -> bytes:
    if len(dist_mm) != LD19_POINTS or len(intensity) != LD19_POINTS:
        raise ValueError("12 points required")
    f = bytearray((LD19_HEADER, LD19_VERLEN, speed_deg_s & 0xFF, speed_deg_s >> 8, start_cdeg & 0xFF, start_cdeg >> 8))
    for d, i in zip(dist_mm, intensity):
        f += bytes((d & 0xFF, d >> 8, i & 0xFF))
    f += bytes((end_cdeg & 0xFF, end_cdeg >> 8, timestamp_ms & 0xFF, timestamp_ms >> 8))
    crc = ld19_crc8(bytes(f))
    f.append((crc ^ 0xFF) if corrupt_crc else crc)
    return bytes(f)


# -------------------------------------------------------------- hex dumps
_HEX_PAIR = re.compile(r"^(?:0x)?[0-9a-fA-F]{2},?$")
_RAW_PREFIX = re.compile(r"^\s*\[raw (radar|lidar)\]\s*(.*)$")

# Timestamp prefixes serial monitors and journals put in front of a line. Shared
# with tools/analyze_log.py so both accept the same pastes.
TIMESTAMP_PATTERNS = [
    re.compile(r"^(\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?\s*->\s*"),               # Arduino IDE
    re.compile(r"^\[(\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?\]\s*"),                 # [12:34:56.789]
    re.compile(r"^\d{4}-\d{2}-\d{2}[T ](\d{2}):(\d{2}):(\d{2})(?:[.,](\d{1,6}))?(?:Z|[+-]\d{2}:?\d{2})?\s*"),  # ISO
    re.compile(r"^\w{3} +\d{1,2} (\d{2}):(\d{2}):(\d{2}) \S+ \S+: "),                # journalctl
    re.compile(r"^(\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?\s+"),                      # plain clock
]


def split_timestamp(line: str) -> Tuple[Optional[float], str]:
    """Seconds since midnight (or None) and the line without its timestamp prefix."""
    for pat in TIMESTAMP_PATTERNS:
        m = pat.match(line)
        if m:
            g = m.groups()
            frac = g[3] if len(g) > 3 and g[3] else "0"
            t = int(g[0]) * 3600 + int(g[1]) * 60 + int(g[2]) + float("0." + frac)
            return t, line[m.end():]
    return None, line


def parse_hex_dump(text: str) -> Tuple[Optional[str], bytes]:
    """Bytes from a ``raw radar`` / ``raw lidar`` console paste, a plain hex dump or a C array.

    Returns ``(source, data)`` where source is ``"radar"``, ``"lidar"`` or ``None`` when the
    paste carried no ``[raw ...]`` prefix. Lines that are not hex bytes (the capture's own
    status lines, other log output) are skipped.
    """
    source: Optional[str] = None
    out = bytearray()
    for line in text.splitlines():
        _, line = split_timestamp(line.rstrip("\r"))
        m = _RAW_PREFIX.match(line)
        if m:
            source = source or m.group(1)
            line = m.group(2)
        line = line.split("//")[0].split("#")[0].strip().strip("{},;")
        if not line:
            continue
        tokens = line.replace(",", " ").split()
        if tokens and all(_HEX_PAIR.match(t) for t in tokens):
            out += bytes(int(t.rstrip(","), 16) for t in tokens)
        elif len(tokens) == 1 and re.fullmatch(r"[0-9a-fA-F]+", tokens[0]) and len(tokens[0]) % 2 == 0 and len(tokens[0]) >= 4:
            out += bytes.fromhex(tokens[0])      # one long hex string per line
    return source, bytes(out)


def format_hex_lines(data: bytes, prefix: str = "", width: int = 32) -> List[str]:
    return [prefix + " ".join(f"{b:02X}" for b in data[i:i + width]) for i in range(0, len(data), width)]


# ------------------------------------------------------- stream analysis
@dataclass
class CaptureReport:
    sensor: str                         # "radar" | "lidar"
    n_bytes: int
    frames: int
    bad_frames: int                     # radar: tail mismatch; lidar: CRC failures
    acks: List[Rd03dAck]
    radar_frames: List[Rd03dFrame]
    lidar_frames: List[Ld19Frame]
    skipped_bytes: int                  # bytes not part of any good frame
    hints: List[str]
    slot_usage: List[int] = field(default_factory=lambda: [0, 0, 0])
    sentinel_slots: int = 0
    lidar_zero_points: int = 0
    lidar_points: int = 0
    lidar_speed_min: int = 0
    lidar_speed_max: int = 0
    lidar_implied_speed: float = 0.0    # deg/s from the angle span per frame, assuming 4500 points/s

    @property
    def ok(self) -> bool:
        return self.frames > 0 and self.bad_frames * 10 <= self.frames

    def summary_lines(self) -> List[str]:
        lines = [f"{self.sensor}: {self.n_bytes} bytes, {self.frames} good frame(s), {self.bad_frames} bad, "
                 f"{self.skipped_bytes} byte(s) outside frames"]
        if self.sensor == "radar":
            if self.acks:
                lines.append("configuration replies: " + "; ".join(f"{a.name} {'OK' if a.ok else 'FAILED status ' + str(a.status)}" for a in self.acks))
            if self.frames:
                lines.append(f"slot usage: 1={self.slot_usage[0]} 2={self.slot_usage[1]} 3={self.slot_usage[2]} "
                             f"of {self.frames} frames; sentinel-speed slots {self.sentinel_slots}")
        else:
            if self.frames:
                lines.append(f"scan speed field {self.lidar_speed_min}..{self.lidar_speed_max} deg/s "
                             f"(implied by angle spans: {self.lidar_implied_speed:.0f} deg/s), "
                             f"{self.lidar_points} points of which {self.lidar_zero_points} without return")
        lines += [f"hint: {h}" for h in self.hints]
        return lines


def _byte_hints(data: bytes) -> List[str]:
    hints = []
    if not data:
        return ["empty capture: the UART was silent (no power, TX on another pin, or no common ground)"]
    idle = sum(1 for b in data if b in (0x00, 0xFF))
    if idle * 10 >= len(data) * 9:
        hints.append(f"{idle * 100 // len(data)}% of the bytes are 00/FF: the line is idle or stuck, "
                     "the sensor's TX is probably not on this pin (or it has no power)")
    elif len(set(data)) < 8 and len(data) > 32:
        hints.append("only a handful of distinct byte values: a baud-rate mismatch or a floating input")
    return hints


def analyze_capture(data: bytes, sensor: str) -> CaptureReport:
    """Decode a raw byte stream and say what it contains and what the problems look like."""
    if sensor not in ("radar", "lidar"):
        raise ValueError("sensor must be 'radar' or 'lidar'")
    hints: List[str] = []
    if sensor == "radar":
        p = Rd03dParser()
        frames = p.feed_bytes(data)
        acks = rd03d_find_acks(data)
        good_bytes = len(frames) * RD03D_FRAME_LEN + sum(6 + len(a.data) + 4 + 4 for a in acks)
        rep = CaptureReport("radar", len(data), p.frames, p.bad_frames, acks, frames, [], max(0, len(data) - good_bytes), hints)
        for f in frames:
            for i, s in enumerate(f.slots()):
                if not s.empty:
                    rep.slot_usage[i] += 1
                    if rd03d_is_sentinel_speed(s.speed_cm_s):
                        rep.sentinel_slots += 1
        if not frames:
            hints += _byte_hints(data)
            if data and not hints:
                if p.bad_frames:
                    hints.append("headers found but every frame fails the 55 CC tail check: bytes are being lost "
                                 "between header and tail (buffer overflow) or the module uses another frame layout")
                elif acks and not frames:
                    hints.append("only configuration replies, no reports: the module is still in configuration mode "
                                 "(send the end-configuration command) or nothing has moved yet")
                else:
                    hints.append("no AA FF 03 00 header in the data: wrong baud rate (RD-03D uses 256000), "
                                 "TX/RX swapped, or this is not the RD-03D's output")
        else:
            if p.bad_frames * 10 > p.frames:
                hints.append(f"{p.bad_frames} of {p.frames + p.bad_frames} frames corrupt: supply ripple or dropped bytes")
            if sum(rep.slot_usage) == 0:
                hints.append("frames are valid but every target slot is empty: nothing moving in front of the module, "
                             "or it faces metal/thick plastic")
            elif rep.slot_usage[1] == 0 and rep.slot_usage[2] == 0 and rep.slot_usage[0] > 5:
                hints.append("only slot 1 is ever used: one target, or the module is still in single-target mode "
                             "(the node sends the multi-target command at boot)")
            if rep.sentinel_slots and rep.sentinel_slots * 2 >= sum(rep.slot_usage):
                hints.append("many slots report speed 0 / 248 / 256 cm/s: the RD-03D's noise sentinels, filtered by the node")
            for a in acks:
                if not a.ok:
                    hints.append(f"the module rejected '{a.name}' (status {a.status})")
        return rep

    p2 = Ld19Parser()
    lframes = p2.feed_bytes(data)
    rep = CaptureReport("lidar", len(data), p2.frames, p2.crc_errors, [], [], lframes,
                        max(0, len(data) - len(lframes) * LD19_FRAME_LEN), hints)
    if lframes:
        speeds = [f.speed_deg_s for f in lframes]
        rep.lidar_speed_min, rep.lidar_speed_max = min(speeds), max(speeds)
        spans = [f.span_cdeg for f in lframes if 0 < f.span_cdeg < 9000]
        if spans:
            # 12 points per frame at the LD19's fixed 4500 points/s -> 375 frames/s -> span * 375 / 100 deg/s
            rep.lidar_implied_speed = sum(spans) / len(spans) / 100.0 * (4500.0 / LD19_POINTS)
        rep.lidar_points = len(lframes) * LD19_POINTS
        rep.lidar_zero_points = sum(1 for f in lframes for d in f.dist_mm if d == 0)
        if p2.crc_errors * 50 > p2.frames:
            hints.append(f"{p2.crc_errors} CRC errors in {p2.frames + p2.crc_errors} frames: supply ripple, a long lead, or dropped bytes")
        if rep.lidar_speed_max < 2500 or rep.lidar_speed_min > 4700:
            hints.append("scan speed outside 2500..4700 deg/s: tie the PWM pin to GND (10 Hz self-regulation), check the 5 V supply")
        if rep.lidar_points and rep.lidar_zero_points * 10 >= rep.lidar_points * 9:
            hints.append("almost no returns: the LIDAR is covered, facing beyond 12 m, or looking at black/glass surfaces")
    else:
        hints += _byte_hints(data)
        if data and not hints:
            if p2.crc_errors:
                hints.append("54 2C headers found but every CRC fails: baud mismatch close to 230400, dropped bytes or noise")
            else:
                hints.append("no 54 2C header in the data: wrong baud rate (LD19 uses 230400), TX on another pin, or not an LD19")
    return rep


def guess_sensor(data: bytes) -> str:
    """Pick the parser that finds more valid frames (radar on a tie)."""
    r = Rd03dParser(); r.feed_bytes(data)
    l = Ld19Parser(); l.feed_bytes(data)
    return "lidar" if l.frames > r.frames else "radar"
