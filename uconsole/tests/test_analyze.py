"""tools/analyze_log.py against synthetic node / GIGA / viewer logs and a synthetic recording."""
import io
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(ROOT / "tools"))

import radar_protocol as rp   # noqa: E402
import radar_state as rs      # noqa: E402
import analyze_log as al      # noqa: E402


def node_log(seconds=60, radar_fps=20, lidar=True, bytes0=False, reconf_at=None, heap_leak=0, join_at=10, boot_banner=True):
    lines = []
    if boot_banner:
        lines += ["RadarLink sensor node — RD-03D + LD19", "radar UART1 rx=1 tx=2 @256000 | lidar UART2 rx=3 tx=4 @230400",
                  '[wifi] AP "RadarSystem" up on channel 6, IP 192.168.4.1, UDP port 4210',
                  "[boot] ready; waiting for receivers to say HELLO. Type ? for the serial console (diag, raw, subs, wifi)",
                  "[radar] multi-target mode configured"]
    rbytes = 0
    bad = 0
    reconf = 0
    for up in range(2, seconds + 1, 2):
        fps = 0 if (up < 2 or bytes0) else radar_fps
        rbytes = 0 if bytes0 else rbytes + fps * 30 * 2
        if reconf_at and up == reconf_at:
            lines.append(f"[radar] no frames for 5002 ms, re-sending configuration (#{reconf + 1})")
            reconf += 1
            fps = 0
        if up % 20 == 0 and not bytes0:
            bad += 1
        sta = 1 if up >= join_at else 0
        subs = 1 if up >= join_at + 2 else 0
        if up == join_at:
            lines.append("[wifi] station joined")
        if up == join_at + 2:
            lines.append("[subs] + 192.168.4.2:4210 kind=1 wants=0x07 (1 total)")
        heap = 250000 - heap_leak * up
        line = (f"[stat] up={up}s ap=up sta={sta} subs={subs} | radar fps={fps} bytes={rbytes} bad={bad} cfg=1 reconf={reconf}")
        if lidar:
            line += f" | lidar fps=375 pps=4100 bytes={up * 17625} crc={up // 30} speed=3600"
        lines.append(line + f" | heap={heap}")
    return "\n".join(lines) + "\n"


def giga_log(failures=3, connect=True, rssi=-48, samples=20, drop=0, data=True):
    lines = ["RadarLink GIGA display starting"]
    for i in range(failures):
        lines += ["[wifi] connecting to RadarSystem", "[wifi] connect failed, status 4"]
    if connect:
        lines += ["[wifi] connecting to RadarSystem", f"[wifi] connected, IP 192.168.4.2, sensor node 192.168.4.1, RSSI {rssi}"]
        for i in range(1, samples + 1):
            lines.append(f"[stat] link=up data={'ok' if data else 'none'} rx/s={70 if data else 0} total={70 * 2 * i if data else 0} "
                         f"drop={drop * i} foreign=0 fps=15 | node radar_fps=20 lidar_fps=375 subs=1")
    return "\n".join(lines) + "\n"


def viewer_log(samples=10, alert=True):
    lines = ["[net] listening on UDP 4210, sensor node 192.168.4.1:4210"]
    for i in range(1, samples + 1):
        lines.append(f"12:00:{i * 2:02d}.000 -> [stat] data=ok rx/s=70 total={140 * i} drop=0 foreign=0 fps=30 | node radar_fps=20 lidar_fps=375 subs=2")
    if alert:
        lines += ["[detect] ARMED", "[alert] START #1 radar at 2.1 m, 45 deg, conf 0.50", "[alert] END #1 after 3.2 s"]
    return "\n".join(lines) + "\n"


def recording(seconds=10, drop_every=0, reboot_at=None):
    buf = io.BytesIO()
    rec = rs.Recorder(buf, 0.0)
    radar_seq = lidar_seq = status_seq = 0
    t = 0.0
    while t < seconds:
        if int(t * 20) % 1 == 0:
            x = int(1000 * (t - seconds / 2))
            pkt = rp.RadarPacket(int(t * 20), [rp.RadarTarget(x, 3000, -60, True), rp.RadarTarget(), rp.RadarTarget()])
            radar_seq += 1
            if not (drop_every and radar_seq % drop_every == 0):
                rec.write(t, rp.encode_radar(radar_seq & 0xFFFF, pkt))
        if int(t * 20) % 1 == 0:
            base = int(t * 20) * 1800 % 36000
            pts = [rp.LidarPoint((base + i * 19) % 36000, 2500 + (i % 7) * 100, 100) for i in range(96)]
            lidar_seq += 1
            rec.write(t, rp.encode_lidar(lidar_seq & 0xFFFF, rp.LidarPacket(3600, 0, pts)))
        if abs(t - round(t)) < 1e-9:
            up = int(t * 1000) if not (reboot_at and t >= reboot_at) else int((t - reboot_at) * 1000)
            st = rp.StatusPacket(up, 20, 375, 4100, 2, rp.STATUS_RADAR_OK | rp.STATUS_LIDAR_OK | rp.STATUS_LIDAR_ENABLED | rp.STATUS_RADAR_CONFIGURED, 0, 0, 0)
            status_seq += 1
            rec.write(t, rp.encode_status(status_seq & 0xFFFF, st))
        t = round(t + 0.05, 6)
    return buf.getvalue()


def analyze_text(text: str, suffix=".log") -> al.Report:
    with tempfile.NamedTemporaryFile("w", suffix=suffix, delete=False) as fh:
        fh.write(text)
        path = fh.name
    try:
        return al.analyze_file(path)
    finally:
        os.unlink(path)


def analyze_bytes(data: bytes) -> al.Report:
    with tempfile.NamedTemporaryFile("wb", suffix=".rdl", delete=False) as fh:
        fh.write(data)
        path = fh.name
    try:
        return al.analyze_file(path)
    finally:
        os.unlink(path)


def issue_texts(rep, level=None):
    return [t for lv, t in rep.issues if level is None or lv == level]


class NodeLogs(unittest.TestCase):
    def test_healthy_node(self):
        rep = analyze_text(node_log())
        self.assertEqual(rep.worst, "OK", rep.issues)
        self.assertTrue(any(t.startswith("radar: 20 frames/s") for t in issue_texts(rep, "OK")), rep.issues)
        self.assertTrue(any(t.startswith("lidar: 375 frames/s") for t in issue_texts(rep, "OK")), rep.issues)
        self.assertEqual(rep.metrics["node_reboots"], 1)
        self.assertEqual(rep.metrics["radar_fps_median"], 20)
        self.assertIn("sensor node", dict(rep.sections))
        text = "\n".join(rep.render())
        self.assertIn("1 join(s)", text)
        self.assertIn("1 HELLO registration(s)", text)

    def test_radar_unpowered(self):
        rep = analyze_text(node_log(bytes0=True, lidar=False))
        self.assertEqual(rep.worst, "FAIL")
        self.assertTrue(any("no bytes at all" in t for t in issue_texts(rep, "FAIL")), rep.issues)
        self.assertFalse(rep.metrics["lidar"])

    def test_reconfig_and_leak(self):
        rep = analyze_text(node_log(seconds=1200, reconf_at=600, heap_leak=10))
        warns = issue_texts(rep, "WARN")
        self.assertTrue(any("re-sent the configuration 1 time" in t for t in warns), rep.issues)
        self.assertTrue(any("shrinking by" in t for t in warns), rep.issues)
        self.assertEqual(rep.worst, "WARN")

    def test_station_without_hello(self):
        text = node_log(join_at=10).replace("[subs] + 192.168.4.2:4210 kind=1 wants=0x07 (1 total)", "")
        text = "\n".join(l.replace("subs=1", "subs=0") for l in text.splitlines())
        rep = analyze_text(text)
        self.assertTrue(any("not sending HELLO" in t for t in issue_texts(rep, "WARN")), rep.issues)

    def test_reboots_and_short_log(self):
        rep = analyze_text(node_log(seconds=20) + node_log(seconds=20))
        self.assertTrue(any("rebooted 3 times" in t or "rebooted" in t for t in issue_texts(rep, "WARN")), rep.issues)
        rep = analyze_text("[stat] up=2s ap=up sta=0 subs=0 | radar fps=0 bytes=0 bad=0 cfg=0 reconf=0 | heap=250000\n")
        self.assertTrue(any("too short" in t for t in issue_texts(rep, "INFO")), rep.issues)

    def test_diag_lines_pass_through(self):
        rep = analyze_text(node_log() + "[diag] OK   radar: 20 frames/s\n[diag] result: all OK\n")
        self.assertIn("node's own diag output", dict(rep.sections))


class ReceiverLogs(unittest.TestCase):
    def test_giga_healthy_after_retries(self):
        rep = analyze_text(giga_log(failures=2))
        self.assertEqual(rep.worst, "INFO", rep.issues)                 # the two failures become INFO once connected
        self.assertTrue(any("status 4" in t for t in issue_texts(rep, "INFO")), rep.issues)
        self.assertTrue(any(t.startswith("GIGA display: receiving 70") for t in issue_texts(rep, "OK")), rep.issues)
        self.assertEqual(rep.metrics["GIGA display_drop_pct"], 0)

    def test_giga_never_connects(self):
        rep = analyze_text(giga_log(failures=5, connect=False))
        self.assertEqual(rep.worst, "FAIL")
        self.assertTrue(any("5x connect failed (status 4)" in t for t in issue_texts(rep, "FAIL")), rep.issues)

    def test_giga_no_data_weak_signal_drops(self):
        rep = analyze_text(giga_log(failures=0, rssi=-82, data=False))
        self.assertTrue(any("never received a packet" in t for t in issue_texts(rep, "FAIL")), rep.issues)
        self.assertTrue(any("weak signal" in t for t in issue_texts(rep, "WARN")), rep.issues)
        rep = analyze_text(giga_log(failures=0, drop=10))
        self.assertTrue(any("dropped" in t for t in issue_texts(rep, "WARN")), rep.issues)

    def test_viewer_with_timestamps_and_alerts(self):
        rep = analyze_text(viewer_log())
        self.assertEqual(rep.worst, "OK", rep.issues)
        sec = dict(rep.sections)["uConsole viewer"]
        self.assertTrue(any("1 alert(s)" in l and "radar" in l for l in sec), sec)
        xs, _ = rep.series["uConsole viewer rx/s"]
        self.assertEqual(xs[0], 12 * 3600 + 2)                           # the timestamp prefix was parsed

    def test_unknown_file(self):
        rep = analyze_text("hello\nworld\n")
        self.assertEqual(rep.worst, "FAIL")
        self.assertIn("no RadarLink log lines", issue_texts(rep, "FAIL")[0])


class Recordings(unittest.TestCase):
    def test_healthy_recording(self):
        rep = analyze_bytes(recording())
        self.assertEqual(rep.worst, "OK", rep.issues)
        self.assertEqual(rep.metrics["recording_seconds"], 9.9)
        self.assertEqual(rep.metrics["radar_target_pct"], 100.0)
        self.assertGreaterEqual(rep.metrics["lidar_sectors"], 30)
        text = "\n".join(rep.render())
        self.assertIn("range histogram", text)
        self.assertIn("median distance per 30-degree sector", text)

    def test_loss_and_reboot(self):
        rep = analyze_bytes(recording(drop_every=20, reboot_at=5))
        self.assertAlmostEqual(rep.metrics["loss_pct"]["radar"], 5.0, delta=0.6)
        warns = issue_texts(rep, "WARN")
        self.assertTrue(any("never arrived" in t for t in warns), rep.issues)
        self.assertTrue(any("rebooted 1 time" in t for t in warns), rep.issues)

    def test_not_a_recording(self):
        rep = analyze_bytes(rs.RECORD_MAGIC + b"\x00")
        self.assertEqual(rep.worst, "FAIL")


class Cli(unittest.TestCase):
    def test_json_and_plot(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node.log")
            pathlib.Path(log).write_text(node_log())
            rec = os.path.join(tmp, "walk.rdl")
            pathlib.Path(rec).write_bytes(recording())
            r = subprocess.run([sys.executable, str(ROOT / "tools" / "analyze_log.py"), log, rec, "--json"], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            out = json.loads(r.stdout)
            self.assertEqual([o["result"] for o in out], ["OK", "OK"])
            try:
                import matplotlib  # noqa: F401
            except ImportError:
                return
            png = os.path.join(tmp, "p.png")
            r = subprocess.run([sys.executable, str(ROOT / "tools" / "analyze_log.py"), log, rec, "--plot", png], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertGreater(os.path.getsize(png), 5000)

    def test_exit_code_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node.log")
            pathlib.Path(log).write_text(node_log(bytes0=True))
            r = subprocess.run([sys.executable, str(ROOT / "tools" / "analyze_log.py"), log], capture_output=True, text=True)
            self.assertEqual(r.returncode, 1)
            self.assertIn("FAIL radar: no bytes at all", r.stdout)


if __name__ == "__main__":
    unittest.main()
