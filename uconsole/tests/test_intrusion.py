import json
import math
import os
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import radar_protocol as rp     # noqa: E402
import radar_state as rs        # noqa: E402
import radar_intrusion as ri    # noqa: E402


def room_distance(angle_deg: float, half_w=3000.0, half_d=2000.0) -> float:
    a = math.radians(angle_deg)
    dx, dy = math.sin(a), math.cos(a)
    best = 1e9
    if abs(dx) > 1e-9:
        best = min(best, half_w / abs(dx))
    if abs(dy) > 1e-9:
        best = min(best, half_d / abs(dy))
    return best


def scan_packets(now_override=None, intruder=None, door=(40, 60)):
    """One full LIDAR scan of the test room as 8-frame packets. intruder=(x,y,r) in mm."""
    pts = []
    for i in range(ri.BUCKETS):
        ang = ri.bucket_angle_deg(i)
        d = room_distance(ang)
        hit = False
        if intruder is not None:
            cx, cy, r = intruder
            dx, dy = math.sin(math.radians(ang)), math.cos(math.radians(ang))
            cd = cx * dx + cy * dy
            disc = cd * cd - (cx * cx + cy * cy) + r * r
            if disc >= 0 and cd > 0:
                d, hit = min(d, cd - math.sqrt(disc)), True
        if door[0] <= ang < door[1] and not hit:
            continue                                  # open doorway: no return unless someone stands in it
        pts.append(rp.LidarPoint(int(ang * 100) % 36000, int(d), 100))
    return [rp.LidarPacket(3600, 0, pts[i:i + 96]) for i in range(0, len(pts), 96)]


def learn_reference(seconds=3.0, step=0.1):
    ref = ri.ReferenceScan()
    ref.start(0.0)
    t = 0.0
    while t < seconds:
        for pkt in scan_packets():
            ref.feed(pkt)
        t += step
    reliable = ref.finish(seconds)
    return ref, reliable


class ReferenceScanTests(unittest.TestCase):
    def test_learns_walls_and_open_door(self):
        ref, reliable = learn_reference()
        self.assertGreater(reliable, 600)
        door_bucket = int(50 / ri.DEG_PER_BUCKET)
        self.assertTrue(ref.open[door_bucket])
        self.assertFalse(ref.reliable[door_bucket])
        front = int(0.25 / ri.DEG_PER_BUCKET)
        self.assertTrue(ref.reliable[front])
        self.assertAlmostEqual(ref.ref_mm[front], 2000, delta=5)
        self.assertTrue(ref.has_baseline())

    def test_flicker_is_unreliable(self):
        ref = ri.ReferenceScan()
        ref.start(0.0)
        for k in range(30):                       # 3 s at 10 scans: bucket 10 flickers between two distances
            pts = [rp.LidarPoint(10 * 50 + 25, 1000 if k % 2 else 3000, 50)]
            pts += [rp.LidarPoint(20 * 50 + 25, 1500, 50)] * 2
            ref.feed(rp.LidarPacket(3600, 0, pts))
        ref.finish(3.0)
        self.assertFalse(ref.reliable[10])
        self.assertFalse(ref.open[10])
        self.assertTrue(ref.reliable[20])

    def test_save_load_roundtrip(self):
        ref, _ = learn_reference()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "baseline.json")
            ref.save(path)
            back = ri.ReferenceScan.load(path)
        self.assertEqual(back.ref_mm, ref.ref_mm)
        self.assertEqual(back.reliable, ref.reliable)
        self.assertEqual(back.open, ref.open)
        with self.assertRaises(ValueError):
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
                json.dump({"version": 9}, fh)
            ri.ReferenceScan.load(fh.name)


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.ref, _ = learn_reference()
        self.lidar = rs.LidarStore(ttl_s=0.5)
        self.radar = rs.RadarTracks(hold_s=0.4, still_timeout_s=0)
        self.det = ri.IntrusionDetector(self.ref, dwell_s=0.5, min_buckets=3, hold_s=1.0)
        self.det.armed = True

    def feed(self, t, intruder=None, radar_xy=None):
        for pkt in scan_packets(intruder=intruder):
            self.lidar.update(pkt, t)
        targets = [rp.RadarTarget(), rp.RadarTarget(), rp.RadarTarget()]
        if radar_xy is not None:
            targets[0] = rp.RadarTarget(int(radar_xy[0]), int(radar_xy[1]), 40, True)
        self.radar.update(rp.RadarPacket(1, targets), t)
        self.det.update(t, self.lidar, self.radar)

    def test_empty_room_no_alert(self):
        for k in range(20):
            self.feed(k * 0.1)
        self.assertEqual(self.det.active(), [])
        self.assertEqual(self.det.drain_events(), [])

    def test_lidar_only_object_after_dwell(self):
        box = (-1000.0, 1200.0, 250.0)          # a parcel left of centre, 1.2 m ahead
        self.feed(0.0)
        self.feed(0.1, intruder=box)
        self.feed(0.3, intruder=box)
        self.assertEqual(self.det.active(), [], "dwell not reached yet")
        self.feed(0.7, intruder=box)
        active = self.det.active()
        self.assertEqual(len(active), 1)
        a = active[0]
        self.assertEqual(a.kind, "lidar")
        self.assertAlmostEqual(a.x_mm, -1000, delta=300)
        self.assertAlmostEqual(a.y_mm, 1200, delta=300)
        self.assertGreater(a.span_deg, 5)
        ev = self.det.drain_events()
        self.assertEqual([e[0] for e in ev], ["start"])
        # object removed: the alert survives the 1 s hold, then ends
        self.feed(1.0)
        self.feed(1.5)
        self.assertEqual(len(self.det.active()), 1, "still within hold_s")
        self.feed(2.0)
        self.assertEqual(self.det.active(), [])
        self.assertEqual([e[0] for e in self.det.drain_events()], ["end"])

    def test_fused_with_radar_and_tracking(self):
        person = (500.0, 1500.0, 250.0)       # inside the 6 x 4 m room (back wall at y = 2 m)
        for t in (0.0, 0.2, 0.4, 0.7):
            self.feed(t, intruder=person, radar_xy=(520, 1480))
        a = self.det.active()
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0].kind, "fused")
        self.assertGreaterEqual(a[0].confidence, 0.9)
        first_id = a[0].id
        # the person walks: the same alert id follows the blob
        for t in (0.9, 1.1, 1.3):
            moved = (500.0 + (t - 0.7) * 400, 1500.0 - (t - 0.7) * 800, 250.0)
            self.feed(t, intruder=moved, radar_xy=(moved[0], moved[1]))
        a = self.det.active()
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0].id, first_id)
        self.assertEqual(len([e for e in self.det.drain_events() if e[0] == "start"]), 1)

    def test_radar_only_motion(self):
        for t in (0.0, 0.2, 0.4):
            self.feed(t, radar_xy=(0, 1500))      # moving target, room outline unchanged
        a = self.det.active()
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0].kind, "radar")
        self.assertAlmostEqual(a[0].dist_mm, 1500, delta=1)
        self.feed(2.0)
        self.assertEqual(self.det.active(), [])

    def test_intruder_in_open_doorway(self):
        person = (*ri.polar_to_xy(50.0, 2200.0), 250.0)   # standing in the open door direction
        for t in (0.0, 0.3, 0.6, 0.9):
            self.feed(t, intruder=person)
        self.assertEqual(len(self.det.active()), 1)

    def test_single_bucket_flicker_ignored(self):
        self.feed(0.0)
        for t in (0.1, 0.4, 0.7, 1.0):
            for pkt in scan_packets():
                self.lidar.update(pkt, t)
            self.lidar.update(rp.LidarPacket(3600, 0, [rp.LidarPoint(100 * 50 + 25, 900, 90)]), t)
            self.det.update(t, self.lidar, self.radar)
        self.assertEqual(self.det.active(), [], "one bucket never reaches min_buckets")

    def test_disarmed_and_learning_clear(self):
        box = (-1000.0, 1200.0, 250.0)
        for t in (0.0, 0.3, 0.6, 0.9):
            self.feed(t, intruder=box)
        self.assertEqual(len(self.det.active()), 1)
        self.det.armed = False
        self.det.update(1.0, self.lidar, self.radar)
        self.assertEqual(self.det.active(), [])
        self.assertEqual([e[0] for e in self.det.drain_events()], ["start", "end"])

    def test_cluster_wraps_around_north(self):
        groups = ri.IntrusionDetector._cluster([718, 719, 0, 1, 2, 300, 301])
        self.assertEqual(len(groups), 2)
        self.assertIn(719, max(groups, key=len))
        self.assertIn(2, max(groups, key=len))


if __name__ == "__main__":
    unittest.main()


class ZoneAndScheduleTests(unittest.TestCase):
    def test_zone_parse_and_contains(self):
        z = ri.IgnoreZone.parse("350:10:1:3")
        self.assertTrue(z.contains(355, 2000))
        self.assertTrue(z.contains(5, 1000))
        self.assertFalse(z.contains(20, 2000), "outside the wedge")
        self.assertFalse(z.contains(0, 4000), "outside the range band")
        z2 = ri.IgnoreZone.parse("90:180")
        self.assertTrue(z2.contains(135, 12000))
        with self.assertRaises(ValueError):
            ri.IgnoreZone.parse("1:2:3")

    def test_zone_suppresses_lidar_and_radar(self):
        ref, _ = learn_reference()
        lidar = rs.LidarStore(ttl_s=0.5)
        radar = rs.RadarTracks(hold_s=0.4, still_timeout_s=0)
        det = ri.IntrusionDetector(ref, dwell_s=0.5, min_buckets=3)
        det.armed = True
        det.zones = [ri.IgnoreZone(300, 340)]          # the corner where the parcel will appear
        box = (-1000.0, 1200.0, 250.0)                  # at about 320 degrees
        for t in (0.0, 0.3, 0.6, 0.9):
            for pkt in scan_packets(intruder=box):
                lidar.update(pkt, t)
            radar.update(rp.RadarPacket(1, [rp.RadarTarget(-1000, 1200, 30, True), rp.RadarTarget(), rp.RadarTarget()]), t)
            det.update(t, lidar, radar)
        self.assertEqual(det.active(), [], "changes inside an ignore zone raise nothing")
        det.zones = []
        det.update(1.0, lidar, radar)
        self.assertEqual(len(det.active()), 1, "same scene without the zone alerts")

    def test_schedule_window_and_wrap(self):
        import datetime as dt
        s = ri.ArmSchedule("22:00-06:00")
        self.assertTrue(s.active(dt.datetime(2026, 1, 1, 23, 30)))
        self.assertTrue(s.active(dt.datetime(2026, 1, 1, 5, 59)))
        self.assertFalse(s.active(dt.datetime(2026, 1, 1, 6, 0)))
        self.assertFalse(s.active(dt.datetime(2026, 1, 1, 12, 0)))
        d = ri.ArmSchedule("09:00-17:00")
        self.assertTrue(d.active(dt.datetime(2026, 1, 1, 9, 0)))
        self.assertFalse(d.active(dt.datetime(2026, 1, 1, 17, 0)))
        self.assertEqual(d.poll(dt.datetime(2026, 1, 1, 8, 0)), False)
        self.assertIsNone(d.poll(dt.datetime(2026, 1, 1, 8, 30)))
        self.assertEqual(d.poll(dt.datetime(2026, 1, 1, 9, 5)), True)
        with self.assertRaises(ValueError):
            ri.ArmSchedule("25:00-06:00")

    def test_snapshot_contents(self):
        ref, _ = learn_reference()
        lidar = rs.LidarStore(ttl_s=0.5)
        radar = rs.RadarTracks(still_timeout_s=0)
        for pkt in scan_packets(intruder=(500.0, 1500.0, 250.0)):
            lidar.update(pkt, 1.0)
        radar.update(rp.RadarPacket(1, [rp.RadarTarget(500, 1500, 40, True), rp.RadarTarget(), rp.RadarTarget()]), 1.0)
        a = ri.Alert(7, "fused", 18.4, 1580.0, 500.0, 1500.0, 12.0, 0.5, 1.0, 0.95, [30, 31, 32], 0)
        snap = ri.snapshot(1.1, lidar, radar, a, lidar_offset_deg=0.0)
        self.assertEqual(snap["alert"]["id"], 7)
        self.assertEqual(snap["alert"]["buckets"], [30, 31, 32])
        self.assertEqual(snap["radar"][0]["x_mm"], 500)
        self.assertGreater(len(snap["lidar"]), 600)
        json.dumps(snap)   # must be serialisable


class RuntimeTests(unittest.TestCase):
    def test_runtime_learns_arms_logs_and_snapshots(self):
        import argparse
        parser = argparse.ArgumentParser()
        ri.DetectionRuntime.add_arguments(parser)
        with tempfile.TemporaryDirectory() as d:
            args = parser.parse_args(["--baseline", os.path.join(d, "b.json"), "--learn", "2",
                                      "--alert-log", os.path.join(d, "a.log"), "--snapshot-dir", os.path.join(d, "snaps"),
                                      "--ignore-zone", "100:120"])
            logs = []
            rt = ri.DetectionRuntime(args, lidar_offset_deg=0.0, log=logs.append)
            lidar = rs.LidarStore(ttl_s=0.5)
            radar = rs.RadarTracks(still_timeout_s=0)
            empty = rp.RadarPacket(1, [rp.RadarTarget(), rp.RadarTarget(), rp.RadarTarget()])
            t = 0.0
            while t < 2.6:                               # empty room: learning completes at 2 s
                for pkt in scan_packets():
                    lidar.update(pkt, t)
                    rt.on_lidar_packet(pkt)
                radar.update(empty, t)
                rt.tick(t, True, lidar, radar)
                t += 0.1
            self.assertTrue(rt.armed)
            self.assertTrue(os.path.exists(os.path.join(d, "b.json")))
            person = (500.0, 1500.0, 250.0)
            events = []
            for t in (3.0, 3.2, 3.4, 3.7):
                for pkt in scan_packets(intruder=person):
                    lidar.update(pkt, t)
                radar.update(rp.RadarPacket(2, [rp.RadarTarget(520, 1480, 40, True), rp.RadarTarget(), rp.RadarTarget()]), t)
                events += rt.tick(t, True, lidar, radar)
            self.assertTrue(any(e == "start" for e, _ in events))
            rt.close()
            rows = open(os.path.join(d, "a.log")).read().splitlines()
            self.assertEqual(rows[0].split(",")[1], "event")
            self.assertTrue(any(",start," in r for r in rows[1:]))
            self.assertEqual(len(os.listdir(os.path.join(d, "snaps"))), 1)
            self.assertTrue(any("ARMED" in m for m in logs))
