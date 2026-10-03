#!/usr/bin/env python3
"""Simulated RadarLink sensor node for testing receivers without hardware.

Behaves like the ESP32 firmware on the network: listens for HELLO on UDP 4210,
remembers each receiver for 5 s and unicasts synthetic RADAR (20 Hz), LIDAR
(about 47 datagrams/s, a 6 x 4 m room) and STATUS (1 Hz) packets to it.

    python3 fake_sensor_node.py                 # serve on 0.0.0.0:4210
    python3 fake_sensor_node.py --write demo.rdl --seconds 10   # write a recording instead
    python3 fake_sensor_node.py --quiet-room --intruder-after 15  # empty room, then someone walks in

Then, on the same machine:  python3 radar_viewer.py --node 127.0.0.1 --windowed
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radar_protocol as rp  # noqa: E402
import radar_state as rs     # noqa: E402

ROOM_HALF_W, ROOM_HALF_D = 3000.0, 2000.0      # mm, sensor sits at the room centre
RADAR_HZ, LIDAR_SCAN_HZ, LIDAR_FRAMES_PER_SCAN = 20.0, 10.0, 37
STATUS_HZ = 1.0


INTRUDER_RADIUS_MM = 250.0


class Scene:
    """Two walkers in the radar sector plus a ray-cast rectangular room for the LIDAR.

    With ``quiet_room`` the walkers are gone and the room is empty, which is what
    the intrusion detector needs to learn its baseline. With ``intruder_after``
    set, a person enters through the doorway at 50 degrees that many seconds in,
    walks towards the centre, lingers, and leaves again after ``intruder_stay``
    seconds; the LIDAR sees the body, the radar sees the motion.
    """

    def __init__(self, quiet_room: bool = False, intruder_after: Optional[float] = None,
                 intruder_stay: float = 20.0, intruder_speed_mm_s: float = 600.0) -> None:
        self.frame = 0
        self.radar_seq = self.lidar_seq = self.status_seq = 0
        self.scan_angle = 0.0
        self.start = time.monotonic()
        self.bad_frames = 0
        self.quiet_room = quiet_room
        self.intruder_after = intruder_after
        self.intruder_stay = intruder_stay
        self.intruder_speed = intruder_speed_mm_s
        self.t0: Optional[float] = None
        self.last_intruder: Optional[Tuple[float, float]] = None

    def intruder_xy(self, t: float) -> Optional[Tuple[float, float]]:
        """Intruder position in mm (x right, y forward) or None when nobody is there."""
        if self.intruder_after is None:
            return None
        if self.t0 is None:
            self.t0 = t
        dt = t - self.t0 - self.intruder_after
        if dt < 0 or dt > self.intruder_stay:
            return None
        door_x, door_y = 3100.0 * math.sin(math.radians(50)), 3100.0 * math.cos(math.radians(50))
        goal_x, goal_y = -800.0, 1400.0
        path = math.hypot(goal_x - door_x, goal_y - door_y)
        walk_t = path / self.intruder_speed
        if dt < walk_t:
            f = dt / walk_t
            return door_x + (goal_x - door_x) * f, door_y + (goal_y - door_y) * f
        # lingering: slow circle so the radar keeps seeing motion
        return goal_x + 300.0 * math.sin((dt - walk_t) * 1.2), goal_y + 300.0 * math.cos((dt - walk_t) * 1.2)

    def radar_packet(self, t: float) -> bytes:
        self.frame += 1
        targets = []
        if self.quiet_room:
            targets += [rp.RadarTarget(), rp.RadarTarget()]
        else:
            # walker 1: back and forth 1..6 m ahead, slightly left
            y1 = 3500 + 2500 * math.sin(t * 0.5)
            targets.append(rp.RadarTarget(int(-600 + 300 * math.sin(t * 0.9)), int(y1), int(125 * math.cos(t * 0.5)), True))
            # walker 2: appears every other 10 s, crossing the sector
            if int(t // 10) % 2 == 0:
                x2 = 2500 * math.sin(t * 0.3)
                targets.append(rp.RadarTarget(int(x2), 2200, int(75 * math.cos(t * 0.3)), True))
            else:
                targets.append(rp.RadarTarget())
        pos = self.intruder_xy(t)
        if pos is not None and abs(math.degrees(math.atan2(pos[0], pos[1]))) <= 60 and math.hypot(*pos) <= 8000:
            spd = 0
            if self.last_intruder is not None:
                spd = int(min(200, math.hypot(pos[0] - self.last_intruder[0], pos[1] - self.last_intruder[1]) / 10 * RADAR_HZ))
            targets.append(rp.RadarTarget(int(pos[0]), int(pos[1]), spd if spd else 5, True))
        else:
            targets.append(rp.RadarTarget())
        self.last_intruder = pos
        data = rp.encode_radar(self.radar_seq, rp.RadarPacket(self.frame, targets))
        self.radar_seq = (self.radar_seq + 1) & 0xFFFF
        return data

    @staticmethod
    def wall_distance(angle_deg: float) -> float:
        a = math.radians(angle_deg)
        dx, dy = math.sin(a), math.cos(a)
        best = 1e9
        if abs(dx) > 1e-9:
            best = min(best, ROOM_HALF_W / abs(dx))
        if abs(dy) > 1e-9:
            best = min(best, ROOM_HALF_D / abs(dy))
        return best

    def lidar_packets(self, t: float) -> List[bytes]:
        """One scan's worth of points split into <=96-point datagrams (8 LD19 frames each)."""
        out = []
        pts: List[rp.LidarPoint] = []
        step = 360.0 / (LIDAR_FRAMES_PER_SCAN * 12)
        pos = self.intruder_xy(t)
        for i in range(LIDAR_FRAMES_PER_SCAN * 12):
            ang = (self.scan_angle + i * step) % 360.0
            d = self.wall_distance(ang) + 25 * math.sin(i * 0.7 + t)      # a little noise
            hit = False
            if pos is not None:                                           # ray against the intruder's body
                dx, dy = math.sin(math.radians(ang)), math.cos(math.radians(ang))
                cd = pos[0] * dx + pos[1] * dy
                disc = cd * cd - (pos[0] ** 2 + pos[1] ** 2) + INTRUDER_RADIUS_MM ** 2
                if disc >= 0 and cd > 0 and cd - math.sqrt(disc) < d:
                    d, hit = cd - math.sqrt(disc), True
            if 40 < ang < 60 and not hit:                                 # a doorway: no return
                continue
            pts.append(rp.LidarPoint(int(ang * 100) % 36000, int(d), 120))
            if len(pts) == 96:
                out.append(rp.encode_lidar(self.lidar_seq, rp.LidarPacket(3600, 0, pts)))
                self.lidar_seq = (self.lidar_seq + 1) & 0xFFFF
                pts = []
        if pts:
            out.append(rp.encode_lidar(self.lidar_seq, rp.LidarPacket(3600, 0, pts)))
            self.lidar_seq = (self.lidar_seq + 1) & 0xFFFF
        self.scan_angle = (self.scan_angle + 0.37) % 360.0
        return out

    def status_packet(self, t: float, subscribers: int) -> bytes:
        st = rp.StatusPacket(int((time.monotonic() - self.start) * 1000), int(RADAR_HZ), LIDAR_FRAMES_PER_SCAN * 10,
                             LIDAR_FRAMES_PER_SCAN * 12 * 10, subscribers,
                             rp.STATUS_RADAR_OK | rp.STATUS_LIDAR_OK | rp.STATUS_RADAR_CONFIGURED | rp.STATUS_LIDAR_ENABLED,
                             self.bad_frames, 0, 0)
        data = rp.encode_status(self.status_seq, st)
        self.status_seq = (self.status_seq + 1) & 0xFFFF
        return data


def serve(port: int, verbose: bool, scene: Scene) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.setblocking(False)
    port = sock.getsockname()[1]
    subs: Dict[Tuple[str, int], Tuple[float, int]] = {}
    next_radar = next_lidar = next_status = time.monotonic()
    print(f"[fake-node] serving on UDP {port}; waiting for HELLO", flush=True)
    while True:
        now = time.monotonic()
        while True:
            try:
                data, addr = sock.recvfrom(256)
            except BlockingIOError:
                break
            dec = rp.decode(data)
            if dec and dec[0] == rp.TYPE_HELLO:
                if addr not in subs:
                    print(f"[fake-node] + subscriber {addr[0]}:{addr[1]} wants=0x{dec[2].wants:02x}")
                subs[addr] = (now, dec[2].wants)
        for addr, (ts, _) in list(subs.items()):
            if now - ts > rp.SUBSCRIBER_TIMEOUT_MS / 1000.0:
                print(f"[fake-node] - subscriber {addr[0]}:{addr[1]} timed out")
                del subs[addr]

        def send(data: bytes, want: int) -> None:
            for addr, (_, wants) in subs.items():
                if wants & want:
                    sock.sendto(data, addr)

        if now >= next_radar:
            next_radar += 1.0 / RADAR_HZ
            send(scene.radar_packet(now), rp.WANT_RADAR)
        if now >= next_lidar:
            next_lidar += 1.0 / LIDAR_SCAN_HZ
            for d in scene.lidar_packets(now):
                send(d, rp.WANT_LIDAR)
        if now >= next_status:
            next_status += 1.0 / STATUS_HZ
            send(scene.status_packet(now, len(subs)), rp.WANT_STATUS)
            if verbose:
                print(f"[fake-node] subscribers={len(subs)}")
        time.sleep(0.002)


def write_recording(path: str, seconds: float, scene: Scene) -> None:
    t0 = 0.0
    with open(path, "wb") as fh:
        rec = rs.Recorder(fh, t0)
        t = t0
        next_radar = next_lidar = next_status = t0
        while t < seconds:
            if t >= next_radar:
                rec.write(t, scene.radar_packet(t)); next_radar += 1.0 / RADAR_HZ
            if t >= next_lidar:
                for d in scene.lidar_packets(t):
                    rec.write(t, d)
                next_lidar += 1.0 / LIDAR_SCAN_HZ
            if t >= next_status:
                rec.write(t, scene.status_packet(t, 1)); next_status += 1.0 / STATUS_HZ
            t += 0.005
        print(f"[fake-node] wrote {rec.count} datagrams covering {seconds:g} s to {path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=rp.PORT)
    p.add_argument("--write", metavar="FILE", help="write a recording instead of serving")
    p.add_argument("--seconds", type=float, default=10.0, help="length of the recording")
    p.add_argument("-v", "--verbose", action="store_true")
    p.add_argument("--quiet-room", action="store_true", help="no walkers: an empty room (for learning a baseline)")
    p.add_argument("--intruder-after", type=float, metavar="S", help="someone walks in through the door after S seconds")
    p.add_argument("--intruder-stay", type=float, default=20.0, metavar="S", help="how long the intruder stays")
    a = p.parse_args()
    scene = Scene(quiet_room=a.quiet_room, intruder_after=a.intruder_after, intruder_stay=a.intruder_stay)
    if a.write:
        write_recording(a.write, a.seconds, scene)
        return 0
    try:
        serve(a.port, a.verbose, scene)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
