"""Python mirror of ``Firmware/sensor_node_esp32s3/radar_protocol.h`` (RadarLink v1).

Every value is little-endian and every packet starts with the same 8-byte
header.  ``uconsole/tests/test_protocol.py`` compiles the C header on the host
and checks that both implementations produce identical bytes, so keep the two
files in step when you change the protocol.  The prose description is in
``docs/PROTOCOL.md``.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union

PORT = 4210
MAGIC = 0x314C4452          # "RDL1"
VERSION = 1
HEADER_LEN = 8
MAX_PACKET = 508            # Arduino Mbed WiFiUDP receive buffer on the GIGA R1

TYPE_RADAR = 0x01
TYPE_LIDAR = 0x02
TYPE_STATUS = 0x03
TYPE_HELLO = 0x10

RADAR_SLOTS = 3
LIDAR_MAX_POINTS = 96

RECEIVER_GIGA = 1
RECEIVER_UCONSOLE = 2
RECEIVER_OTHER = 3

WANT_RADAR = 0x01
WANT_LIDAR = 0x02
WANT_STATUS = 0x04
WANT_ALL = WANT_RADAR | WANT_LIDAR | WANT_STATUS

STATUS_RADAR_OK = 0x01
STATUS_LIDAR_OK = 0x02
STATUS_RADAR_CONFIGURED = 0x04
STATUS_LIDAR_ENABLED = 0x08

HELLO_INTERVAL_MS = 1000
SUBSCRIBER_TIMEOUT_MS = 5000
LINK_TIMEOUT_MS = 3000

_HEADER = struct.Struct("<IBBH")        # magic, type, version, seq
_RADAR_FRAME = struct.Struct("<I")      # frame_count
_RADAR_SLOT = struct.Struct("<hhhBx")   # x, y, speed, valid, pad
_LIDAR_HDR = struct.Struct("<HBB")      # scan_speed, count, flags
_LIDAR_POINT = struct.Struct("<HHB")    # angle_cdeg, dist_mm, intensity
_STATUS = struct.Struct("<IHHHBBIII")   # see StatusPacket
_HELLO = struct.Struct("<BBH")          # receiver_kind, wants, reserved

RADAR_PACKET_LEN = HEADER_LEN + _RADAR_FRAME.size + RADAR_SLOTS * _RADAR_SLOT.size   # 36
STATUS_PACKET_LEN = HEADER_LEN + _STATUS.size                                        # 32
HELLO_PACKET_LEN = HEADER_LEN + _HELLO.size                                          # 12

assert _RADAR_SLOT.size == 8 and _LIDAR_POINT.size == 5 and _LIDAR_HDR.size == 4
assert _STATUS.size == 24 and _HELLO.size == 4
assert HEADER_LEN + _LIDAR_HDR.size + LIDAR_MAX_POINTS * _LIDAR_POINT.size <= MAX_PACKET


def lidar_packet_len(count: int) -> int:
    return HEADER_LEN + _LIDAR_HDR.size + _LIDAR_POINT.size * count


# ----------------------------------------------------------------- dataclasses
@dataclass
class RadarTarget:
    x_mm: int = 0
    y_mm: int = 0
    speed_cm_s: int = 0
    valid: bool = False

    @property
    def distance_mm(self) -> float:
        return (self.x_mm ** 2 + self.y_mm ** 2) ** 0.5


@dataclass
class RadarPacket:
    frame_count: int = 0
    targets: List[RadarTarget] = field(default_factory=lambda: [RadarTarget() for _ in range(RADAR_SLOTS)])


@dataclass
class LidarPoint:
    angle_cdeg: int
    dist_mm: int
    intensity: int


@dataclass
class LidarPacket:
    scan_speed_deg_s: int = 0
    flags: int = 0
    points: List[LidarPoint] = field(default_factory=list)


@dataclass
class StatusPacket:
    uptime_ms: int = 0
    radar_fps: int = 0
    lidar_fps: int = 0
    lidar_pps: int = 0
    subscribers: int = 0
    flags: int = 0
    radar_bad_frames: int = 0
    lidar_crc_errors: int = 0
    reserved: int = 0


@dataclass
class HelloPacket:
    receiver_kind: int = RECEIVER_OTHER
    wants: int = WANT_ALL
    reserved: int = 0


Packet = Union[RadarPacket, LidarPacket, StatusPacket, HelloPacket]


# ---------------------------------------------------------------------- header
def encode_header(ptype: int, seq: int) -> bytes:
    return _HEADER.pack(MAGIC, ptype & 0xFF, VERSION, seq & 0xFFFF)


def decode_header(data: bytes) -> Optional[Tuple[int, int]]:
    """Return ``(type, seq)`` or ``None`` when this is not a RadarLink v1 datagram."""
    if len(data) < HEADER_LEN:
        return None
    magic, ptype, version, seq = _HEADER.unpack_from(data, 0)
    if magic != MAGIC or version != VERSION:
        return None
    return ptype, seq


def seq_newer(seq: int, last: int) -> bool:
    """True when ``seq`` is ahead of ``last`` on the wrapping 16-bit counter."""
    diff = (seq - last) & 0xFFFF
    return diff != 0 and diff < 0x8000


# ----------------------------------------------------------------------- radar
def encode_radar(seq: int, pkt: RadarPacket) -> bytes:
    out = bytearray(encode_header(TYPE_RADAR, seq))
    out += _RADAR_FRAME.pack(pkt.frame_count & 0xFFFFFFFF)
    for i in range(RADAR_SLOTS):
        t = pkt.targets[i] if i < len(pkt.targets) else RadarTarget()
        if t.valid:
            out += _RADAR_SLOT.pack(t.x_mm, t.y_mm, t.speed_cm_s, 1)
        else:
            out += _RADAR_SLOT.pack(0, 0, 0, 0)
    return bytes(out)


def decode_radar(data: bytes) -> Optional[RadarPacket]:
    if len(data) < RADAR_PACKET_LEN:
        return None
    (frame_count,) = _RADAR_FRAME.unpack_from(data, HEADER_LEN)
    targets = []
    off = HEADER_LEN + _RADAR_FRAME.size
    for _ in range(RADAR_SLOTS):
        x, y, spd, valid = _RADAR_SLOT.unpack_from(data, off)
        targets.append(RadarTarget(x, y, spd, bool(valid)))
        off += _RADAR_SLOT.size
    return RadarPacket(frame_count, targets)


# ----------------------------------------------------------------------- lidar
def encode_lidar(seq: int, pkt: LidarPacket) -> bytes:
    pts = pkt.points[:LIDAR_MAX_POINTS]
    out = bytearray(encode_header(TYPE_LIDAR, seq))
    out += _LIDAR_HDR.pack(pkt.scan_speed_deg_s & 0xFFFF, len(pts), 0)
    for p in pts:
        out += _LIDAR_POINT.pack(p.angle_cdeg & 0xFFFF, p.dist_mm & 0xFFFF, p.intensity & 0xFF)
    return bytes(out)


def decode_lidar(data: bytes) -> Optional[LidarPacket]:
    if len(data) < HEADER_LEN + _LIDAR_HDR.size:
        return None
    scan_speed, count, flags = _LIDAR_HDR.unpack_from(data, HEADER_LEN)
    if count > LIDAR_MAX_POINTS or len(data) < lidar_packet_len(count):
        return None
    off = HEADER_LEN + _LIDAR_HDR.size
    points = []
    for _ in range(count):
        angle, dist, intensity = _LIDAR_POINT.unpack_from(data, off)
        points.append(LidarPoint(angle, dist, intensity))
        off += _LIDAR_POINT.size
    return LidarPacket(scan_speed, flags, points)


# ---------------------------------------------------------------------- status
def encode_status(seq: int, s: StatusPacket) -> bytes:
    return encode_header(TYPE_STATUS, seq) + _STATUS.pack(
        s.uptime_ms & 0xFFFFFFFF, s.radar_fps & 0xFFFF, s.lidar_fps & 0xFFFF, s.lidar_pps & 0xFFFF,
        s.subscribers & 0xFF, s.flags & 0xFF,
        s.radar_bad_frames & 0xFFFFFFFF, s.lidar_crc_errors & 0xFFFFFFFF, s.reserved & 0xFFFFFFFF)


def decode_status(data: bytes) -> Optional[StatusPacket]:
    if len(data) < STATUS_PACKET_LEN:
        return None
    return StatusPacket(*_STATUS.unpack_from(data, HEADER_LEN))


# ----------------------------------------------------------------------- hello
def encode_hello(seq: int, receiver_kind: int = RECEIVER_UCONSOLE, wants: int = WANT_ALL) -> bytes:
    return encode_header(TYPE_HELLO, seq) + _HELLO.pack(receiver_kind & 0xFF, wants & 0xFF, 0)


def decode_hello(data: bytes) -> Optional[HelloPacket]:
    if len(data) < HELLO_PACKET_LEN:
        return None
    return HelloPacket(*_HELLO.unpack_from(data, HEADER_LEN))


# -------------------------------------------------------------------- dispatch
_DECODERS = {
    TYPE_RADAR: decode_radar,
    TYPE_LIDAR: decode_lidar,
    TYPE_STATUS: decode_status,
    TYPE_HELLO: decode_hello,
}


def decode(data: bytes) -> Optional[Tuple[int, int, Packet]]:
    """Decode any RadarLink datagram into ``(type, seq, packet)``.

    Returns ``None`` for foreign, truncated or unknown-type datagrams.
    """
    hdr = decode_header(data)
    if hdr is None:
        return None
    ptype, seq = hdr
    decoder = _DECODERS.get(ptype)
    if decoder is None:
        return None
    pkt = decoder(data)
    if pkt is None:
        return None
    return ptype, seq, pkt
