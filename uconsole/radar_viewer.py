#!/usr/bin/env python3
"""RadarLink viewer for the ClockworkPi uConsole (or any Linux box with pygame).

Joins nothing itself: connect the machine to the sensor node's Wi-Fi first
(``nmcli dev wifi connect RadarSystem password ...``). The viewer then sends a
HELLO to the sensor node once a second and draws the LIDAR room outline and the
radar's moving targets, with optional sonar-style audio.

Keys: ESC/Q quit · UP/DOWN or +/- zoom · M mute · L lidar on/off · F fullscreen
      SPACE pause (replay) · R start/stop recording

Examples:
    python3 radar_viewer.py                          # fullscreen, node at the default gateway
    python3 radar_viewer.py --windowed --node 192.168.4.1
    python3 radar_viewer.py --record walk.rdl        # capture raw datagrams
    python3 radar_viewer.py --replay walk.rdl --windowed --loop
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time
from typing import List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radar_protocol as rp  # noqa: E402
import radar_state as rs     # noqa: E402

# ------------------------------------------------------------------ defaults
DEFAULT_NODE = "192.168.4.1"
LIDAR_ANGLE_OFFSET_DEG = 0.0      # rotate the point cloud so the sensor's "forward" is up
LIDAR_CLOCKWISE = True            # LD19 angles increase clockwise seen from above
LIDAR_TTL_S = 0.5
RADAR_HOLD_S = 0.4
RADAR_MIRROR_X = False        # True if "walk left" moves the dot right (module mounted rotated 180 degrees)
RADAR_STILL_TIMEOUT_S = 2.0
RADAR_FOV_HALF_DEG = 60.0
RADAR_MAX_RANGE_M = 8.0
ZOOM_LEVELS_M = (2.0, 4.0, 8.0, 12.0)
BEEP = dict(dist_min=300.0, dist_max=8000.0, interval_min=0.12, interval_max=0.9, freq_min=700.0, freq_max=1800.0)

SLOT_COLOURS = [(255, 70, 70), (255, 180, 0), (0, 230, 255)]
C_BG, C_GRID, C_GRID_DIM, C_SECTOR = (6, 10, 6), (0, 95, 0), (0, 50, 0), (0, 110, 30)
C_TEXT, C_DIM, C_OK, C_BAD, C_WARN = (230, 255, 230), (110, 150, 110), (80, 255, 80), (255, 80, 80), (255, 190, 0)


def default_gateway() -> Optional[str]:
    """Default IPv4 gateway from /proc/net/route (Linux only)."""
    try:
        with open("/proc/net/route") as fh:
            next(fh)
            for line in fh:
                f = line.split()
                if f[1] == "00000000" and int(f[3], 16) & 2:
                    g = int(f[2], 16)
                    return f"{g & 0xFF}.{(g >> 8) & 0xFF}.{(g >> 16) & 0xFF}.{(g >> 24) & 0xFF}"
    except (OSError, StopIteration, ValueError, IndexError):
        pass
    return None


# -------------------------------------------------------------------- audio
class Sonar:
    """Synthesised pings (numpy + pygame.mixer). Silently disabled when either is missing."""

    def __init__(self, enabled: bool) -> None:
        self.ok = False
        self.last_ping = 0.0
        self.cache = {}
        if not enabled:
            return
        try:
            import numpy as np  # noqa: F401
            import pygame
            pygame.mixer.pre_init(44100, -16, 1, 512)
            pygame.mixer.init()
            self.np = np
            self.pygame = pygame
            self.ok = True
        except Exception as exc:  # pragma: no cover - depends on host audio
            print(f"[audio] disabled: {exc}")

    def _sound(self, freq: float):
        key = int(freq // 25)
        snd = self.cache.get(key)
        if snd is None:
            np, pygame = self.np, self.pygame
            sr, dur = 44100, 0.09
            t = np.arange(int(sr * dur)) / sr
            env = np.exp(-t * 45.0) * np.minimum(1.0, t * 400.0)
            wave = 0.7 * np.sin(2 * np.pi * freq * t) + 0.25 * np.sin(2 * np.pi * 2 * freq * t)
            pcm = (wave * env * 32767 * 0.6).astype(np.int16)
            snd = pygame.sndarray.make_sound(np.ascontiguousarray(pcm))
            self.cache[key] = snd
        return snd

    def update(self, now: float, closest_mm: Optional[float], muted: bool) -> None:
        if not self.ok or muted or closest_mm is None:
            return
        d = min(max(closest_mm, BEEP["dist_min"]), BEEP["dist_max"])
        t = (d - BEEP["dist_min"]) / (BEEP["dist_max"] - BEEP["dist_min"])
        interval = BEEP["interval_min"] + t * (BEEP["interval_max"] - BEEP["interval_min"])
        freq = BEEP["freq_max"] - t * (BEEP["freq_max"] - BEEP["freq_min"])
        if now - self.last_ping >= interval:
            self.last_ping = now
            self._sound(freq).play()


# ------------------------------------------------------------------- viewer
class Viewer:
    def __init__(self, args: argparse.Namespace) -> None:
        import pygame
        self.pygame = pygame
        self.args = args
        self.node_ip = args.node
        self.replay: Optional[rs.Replayer] = None
        self.recorder: Optional[rs.Recorder] = None
        self.sock: Optional[socket.socket] = None
        self.hello_seq = 0
        self.last_hello = 0.0
        self.last_packet = 0.0
        self.rx_total = self.rx_foreign = 0
        self.rx_this_sec = 0
        self.rx_per_sec = 0
        self.last_sec = 0.0
        self.seq = rs.SeqTracker()
        self.lidar = rs.LidarStore(LIDAR_TTL_S)
        self.radar = rs.RadarTracks(RADAR_HOLD_S, 0.0 if args.no_still_filter else RADAR_STILL_TIMEOUT_S)
        self.status: Optional[rp.StatusPacket] = None
        self.status_at = 0.0
        self.zoom = max(0, min(len(ZOOM_LEVELS_M) - 1, ZOOM_LEVELS_M.index(args.range) if args.range in ZOOM_LEVELS_M else 2))
        self.muted = args.mute
        self.lidar_visible = not args.no_lidar
        self.mirror_x = RADAR_MIRROR_X or args.mirror_radar
        self.paused = False
        self.frames = 0
        self.fps = 0
        self.frames_this_sec = 0
        self.wants = rp.WANT_RADAR | rp.WANT_STATUS | (0 if args.no_lidar else rp.WANT_LIDAR)
        offset = args.offset if args.offset is not None else LIDAR_ANGLE_OFFSET_DEG
        sign = 1.0 if LIDAR_CLOCKWISE else -1.0
        self.bucket_trig = [(math.sin(math.radians(sign * ((i + 0.5) * 360.0 / rs.BUCKETS + offset))),
                             math.cos(math.radians(sign * ((i + 0.5) * 360.0 / rs.BUCKETS + offset))))
                            for i in range(rs.BUCKETS)]

        if args.replay:
            self.replay = rs.Replayer(open(args.replay, "rb"), speed=args.speed, loop=args.loop)
            print(f"[replay] {len(self.replay.records)} datagrams from {args.replay}")
        else:
            self.open_socket()
        if args.record:
            self.recorder = rs.Recorder(open(args.record, "wb"), time.monotonic())
            print(f"[record] writing {args.record}")

        pygame.init()
        flags = 0 if args.windowed else pygame.FULLSCREEN
        size = tuple(args.size) if args.windowed else (0, 0)
        self.screen = pygame.display.set_mode(size, flags)
        pygame.display.set_caption("RadarLink viewer")
        pygame.mouse.set_visible(args.windowed)
        self.w, self.h = self.screen.get_size()
        self.font = pygame.font.SysFont("monospace", max(14, self.h // 40))
        self.font_big = pygame.font.SysFont("monospace", max(18, self.h // 24), bold=True)
        self.cx, self.cy = self.w // 2, self.h // 2
        self.plot_r = min(self.w // 2, self.h // 2) - 12
        self.sonar = Sonar(not args.no_audio and not args.replay_silent)
        self.clock = pygame.time.Clock()

    # ------------------------------------------------------------ network
    def open_socket(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # No SO_REUSEADDR: on Linux it would let two processes share the port
        # and split the datagrams between them. A second viewer on the same
        # machine simply gets an ephemeral port; the sensor node replies to
        # whatever port the HELLO came from.
        try:
            s.bind(("", rp.PORT))
        except OSError:
            s.bind(("", 0))
        s.setblocking(False)
        self.sock = s
        print(f"[net] listening on UDP {s.getsockname()[1]}, sensor node {self.node_ip}:{rp.PORT}")

    def send_hello(self, now: float) -> None:
        if self.sock is None or now - self.last_hello < rp.HELLO_INTERVAL_MS / 1000.0:
            return
        self.last_hello = now
        try:
            self.sock.sendto(rp.encode_hello(self.hello_seq, rp.RECEIVER_UCONSOLE, self.wants), (self.node_ip, rp.PORT))
        except OSError as exc:
            print(f"[net] hello failed: {exc}")
        self.hello_seq = (self.hello_seq + 1) & 0xFFFF

    def poll_network(self, now: float) -> None:
        if self.replay is not None:
            if not self.paused:
                for data in self.replay.due(now):
                    self.handle(data, now)
            return
        assert self.sock is not None
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"[net] recv error: {exc}")
                break
            if addr[0] != self.node_ip:
                self.rx_foreign += 1
                continue
            if self.recorder is not None:
                self.recorder.write(now, data)
            self.handle(data, now)

    def handle(self, data: bytes, now: float) -> None:
        decoded = rp.decode(data)
        if decoded is None:
            self.rx_foreign += 1
            return
        ptype, seq, pkt = decoded
        if ptype not in (rp.TYPE_RADAR, rp.TYPE_LIDAR, rp.TYPE_STATUS):
            self.rx_foreign += 1          # e.g. another receiver's HELLO
            return
        if not self.seq.accept(ptype, seq):
            return
        self.rx_total += 1
        self.rx_this_sec += 1
        self.last_packet = now
        if ptype == rp.TYPE_RADAR:
            self.radar.update(pkt, now)
        elif ptype == rp.TYPE_LIDAR:
            self.lidar.update(pkt, now)
        elif ptype == rp.TYPE_STATUS:
            self.status, self.status_at = pkt, now

    # ------------------------------------------------------------ drawing
    def alive(self, now: float) -> bool:
        return bool(self.last_packet) and now - self.last_packet < rp.LINK_TIMEOUT_MS / 1000.0

    def px_per_mm(self) -> float:
        return self.plot_r / (ZOOM_LEVELS_M[self.zoom] * 1000.0)

    def draw_grid(self) -> None:
        pg, scr = self.pygame, self.screen
        rng = ZOOM_LEVELS_M[self.zoom]
        step = 2.0 if rng > 8 else (0.5 if rng <= 2 else 1.0)
        k = self.px_per_mm()
        m = step
        while m <= rng + 1e-6:
            r = int(m * 1000 * k)
            pg.draw.circle(scr, C_GRID if abs((m / step) % 2) < 1e-6 else C_GRID_DIM, (self.cx, self.cy), r, 1)
            scr.blit(self.font.render(f"{m:g}m", True, C_DIM), (self.cx + 4, self.cy - r - self.font.get_height()))
            m += step
        pg.draw.line(scr, C_GRID_DIM, (self.cx - self.plot_r, self.cy), (self.cx + self.plot_r, self.cy), 1)
        pg.draw.line(scr, C_GRID_DIM, (self.cx, self.cy - self.plot_r), (self.cx, self.cy + self.plot_r), 1)
        rr = min(self.plot_r, RADAR_MAX_RANGE_M * 1000 * k)
        pts = [(self.cx + rr * math.sin(math.radians(a)), self.cy - rr * math.cos(math.radians(a)))
               for a in range(-int(RADAR_FOV_HALF_DEG), int(RADAR_FOV_HALF_DEG) + 1, 5)]
        pg.draw.lines(scr, C_SECTOR, False, [(self.cx, self.cy)] + pts + [(self.cx, self.cy)], 1)
        pg.draw.circle(scr, C_OK, (self.cx, self.cy), 4)

    def draw_lidar(self, now: float) -> None:
        if not self.lidar_visible:
            return
        k = self.px_per_mm()
        scr = self.screen
        for idx, dist, _inten, age in self.lidar.visible(now):
            r = dist * k
            if r > self.plot_r:
                continue
            s, c = self.bucket_trig[idx]
            x, y = int(self.cx + r * s), int(self.cy - r * c)
            g = int(255 - 150 * age)
            scr.fill((int(60 + 60 * (1 - age)), g, int(80 * (1 - age))), (x - 1, y - 1, 3, 3))

    def draw_targets(self, now: float) -> None:
        pg, scr = self.pygame, self.screen
        k = self.px_per_mm()
        sign = -1.0 if self.mirror_x else 1.0
        for slot, t in self.radar.shown(now):
            x = int(self.cx + sign * t.x_mm * k)
            y = int(self.cy - t.y_mm * k)
            col = SLOT_COLOURS[slot]
            pg.draw.circle(scr, col, (x, y), 7)
            pg.draw.circle(scr, col, (x, y), 15, 2)
            pg.draw.circle(scr, col, (x, y), 23, 1)
            label = self.font.render(f"{t.distance_mm / 1000:.2f}m {abs(t.speed_cm_s)}cm/s", True, C_TEXT)
            scr.blit(label, (x + 26, y - label.get_height() // 2))

    def draw_hud(self, now: float) -> None:
        scr, f = self.screen, self.font
        lh = f.get_height() + 2
        ok = self.alive(now)
        lines: List[Tuple[str, Tuple[int, int, int]]] = []
        if self.replay is not None:
            lines.append((f"REPLAY {'paused' if self.paused else ''}", C_WARN))
        lines.append((f"DATA  {'OK' if ok else '----'}", C_OK if ok else C_BAD))
        st = self.status
        if st is not None and ok:
            lines.append((f"node  up {st.uptime_ms // 1000}s  subs {st.subscribers}", C_DIM))
            r_ok = bool(st.flags & rp.STATUS_RADAR_OK)
            lines.append((f"radar {'OK ' if r_ok else '---'} {st.radar_fps} fps  bad {st.radar_bad_frames}", C_DIM if r_ok else C_WARN))
            if st.flags & rp.STATUS_LIDAR_ENABLED:
                l_ok = bool(st.flags & rp.STATUS_LIDAR_OK)
                lines.append((f"lidar {'OK ' if l_ok else '---'} {st.lidar_fps} fps {st.lidar_pps} pps  crc {st.lidar_crc_errors}",
                              C_DIM if l_ok else C_WARN))
            else:
                lines.append(("lidar not fitted", C_DIM))
        elif self.replay is None:
            lines.append(("waiting for sensor node", C_WARN))
        lines.append((f"rx {self.rx_per_sec}/s  drop {self.seq.dropped}  foreign {self.rx_foreign}  fps {self.fps}", C_DIM))
        lines.append((f"range {ZOOM_LEVELS_M[self.zoom]:g} m   lidar {'on' if self.lidar_visible else 'off'}   "
                      f"sound {'muted' if self.muted else ('on' if self.sonar.ok else 'n/a')}", C_DIM))
        if self.recorder is not None:
            lines.append((f"REC {self.recorder.count} datagrams", C_BAD))
        scr.blit(self.font_big.render("RadarLink", True, C_OK), (8, 6))
        y = 8 + self.font_big.get_height() + 4
        for text, col in lines:
            scr.blit(f.render(text, True, col), (8, y))
            y += lh

        y = self.h - lh * (len(self.radar.shown(now)) + 1) - 6
        for slot, t in self.radar.shown(now):
            scr.blit(f.render(f"T{slot + 1} {t.distance_mm / 1000:.2f}m {abs(t.speed_cm_s)}cm/s", True, SLOT_COLOURS[slot]), (8, y))
            y += lh
        help_text = "ESC quit  UP/DOWN zoom  M mute  L lidar  F fullscreen" + ("  R record" if self.replay is None else "  SPACE pause")
        hs = f.render(help_text, True, C_DIM)
        scr.blit(hs, (self.w - hs.get_width() - 8, self.h - lh - 4))
        if not ok:
            msg = self.font_big.render("NO DATA", True, C_BAD)
            scr.blit(msg, (self.cx - msg.get_width() // 2, self.cy + self.plot_r - msg.get_height() - 6))

    def render(self, now: float) -> None:
        self.screen.fill(C_BG)
        self.draw_grid()
        self.draw_lidar(now)
        self.draw_targets(now)
        self.draw_hud(now)
        self.pygame.display.flip()
        self.frames += 1
        self.frames_this_sec += 1

    # ------------------------------------------------------------- input
    def handle_events(self) -> bool:
        pg = self.pygame
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                return False
            if ev.type != pg.KEYDOWN:
                continue
            if ev.key in (pg.K_ESCAPE, pg.K_q):
                return False
            if ev.key in (pg.K_UP, pg.K_PLUS, pg.K_KP_PLUS, pg.K_EQUALS):
                self.zoom = max(0, self.zoom - 1)
            elif ev.key in (pg.K_DOWN, pg.K_MINUS, pg.K_KP_MINUS):
                self.zoom = min(len(ZOOM_LEVELS_M) - 1, self.zoom + 1)
            elif ev.key == pg.K_m:
                self.muted = not self.muted
            elif ev.key == pg.K_l:
                self.lidar_visible = not self.lidar_visible
            elif ev.key == pg.K_f:
                pg.display.toggle_fullscreen()
            elif ev.key == pg.K_SPACE and self.replay is not None:
                self.paused = not self.paused
            elif ev.key == pg.K_r and self.replay is None:
                if self.recorder is None:
                    name = time.strftime("radarlink-%Y%m%d-%H%M%S.rdl")
                    self.recorder = rs.Recorder(open(name, "wb"), time.monotonic())
                    print(f"[record] writing {name}")
                else:
                    self.recorder.close()
                    print(f"[record] stopped after {self.recorder.count} datagrams")
                    self.recorder = None
        return True

    # -------------------------------------------------------------- main
    def run(self) -> int:
        running = True
        last_log = time.monotonic()
        while running:
            now = time.monotonic()
            running = self.handle_events()
            self.send_hello(now)
            self.poll_network(now)
            self.sonar.update(now, self.radar.closest_mm(now), self.muted)
            self.render(now)
            if now - self.last_sec >= 1.0:
                self.last_sec = now
                self.rx_per_sec, self.rx_this_sec = self.rx_this_sec, 0
                self.fps, self.frames_this_sec = self.frames_this_sec, 0
            if now - last_log >= 2.0:
                last_log = now
                st = self.status
                print(f"[stat] data={'ok' if self.alive(now) else 'none'} rx/s={self.rx_per_sec} total={self.rx_total} "
                      f"drop={self.seq.dropped} foreign={self.rx_foreign} fps={self.fps}"
                      + (f" | node radar_fps={st.radar_fps} lidar_fps={st.lidar_fps} subs={st.subscribers}" if st else ""))
            if self.args.max_frames and self.frames >= self.args.max_frames:
                running = False
            if self.replay is not None and self.replay.finished and not self.args.loop and self.args.exit_on_replay_end:
                running = False
            self.clock.tick(self.args.fps)
        if self.recorder is not None:
            self.recorder.close()
        self.pygame.quit()
        return 0


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--node", default=None, help=f"sensor node IP (default: Wi-Fi gateway, else {DEFAULT_NODE})")
    p.add_argument("--windowed", action="store_true", help="run in a window instead of fullscreen")
    p.add_argument("--size", nargs=2, type=int, default=(1280, 720), metavar=("W", "H"), help="window size with --windowed")
    p.add_argument("--range", type=float, default=8.0, help="initial range in metres: 2, 4, 8 or 12")
    p.add_argument("--offset", type=float, default=None, help="LIDAR mounting angle offset in degrees (see docs/TESTING.md)")
    p.add_argument("--no-lidar", action="store_true", help="do not request or draw LIDAR data")
    p.add_argument("--no-audio", action="store_true", help="disable the sonar pings")
    p.add_argument("--mute", action="store_true", help="start muted")
    p.add_argument("--no-still-filter", action="store_true", help="keep showing targets that stopped moving")
    p.add_argument("--mirror-radar", action="store_true", help="flip radar left/right (module mounted rotated 180 degrees)")
    p.add_argument("--fps", type=int, default=30, help="render frame rate limit")
    p.add_argument("--record", metavar="FILE", help="record raw datagrams to FILE (.rdl)")
    p.add_argument("--replay", metavar="FILE", help="replay a recording instead of listening")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    p.add_argument("--loop", action="store_true", help="loop the replay")
    p.add_argument("--max-frames", type=int, default=0, help="exit after N frames (testing)")
    p.add_argument("--exit-on-replay-end", action="store_true", help="exit when the replay finishes")
    p.add_argument("--replay-silent", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.node is None:
        args.node = default_gateway() or DEFAULT_NODE
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    try:
        return Viewer(args).run()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
