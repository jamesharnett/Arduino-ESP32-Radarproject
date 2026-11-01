import io
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import radar_protocol as rp  # noqa: E402
import radar_state as rs     # noqa: E402


class SeqTrackerTests(unittest.TestCase):
    def test_in_order_and_wrap(self):
        s = rs.SeqTracker()
        self.assertTrue(s.accept(rp.TYPE_RADAR, 65534))
        self.assertTrue(s.accept(rp.TYPE_RADAR, 65535))
        self.assertTrue(s.accept(rp.TYPE_RADAR, 0))
        self.assertFalse(s.accept(rp.TYPE_RADAR, 65535), "late packet dropped")
        self.assertEqual(s.dropped, 1)

    def test_types_independent(self):
        s = rs.SeqTracker()
        self.assertTrue(s.accept(rp.TYPE_RADAR, 100))
        self.assertTrue(s.accept(rp.TYPE_LIDAR, 5))

    def test_resync_after_sender_reboot(self):
        s = rs.SeqTracker(resync_after=5)
        self.assertTrue(s.accept(rp.TYPE_RADAR, 1000))
        results = [s.accept(rp.TYPE_RADAR, i) for i in range(6)]   # sender restarted at 0 (1000 -> 0 is 'old')
        self.assertEqual(results[:4], [False] * 4)
        self.assertTrue(results[4], "fifth old packet resynchronises")
        self.assertTrue(results[5])


class LidarStoreTests(unittest.TestCase):
    def test_bucket_and_expiry(self):
        st = rs.LidarStore(ttl_s=0.5)
        pkt = rp.LidarPacket(3600, 0, [rp.LidarPoint(0, 1000, 10), rp.LidarPoint(49, 1001, 11),
                                       rp.LidarPoint(50, 2000, 20), rp.LidarPoint(35999, 3000, 30)])
        st.update(pkt, 10.0)
        vis = {i: (d, inten, age) for i, d, inten, age in st.visible(10.1)}
        self.assertEqual(set(vis), {0, 1, 719})
        self.assertEqual(vis[0][0], 1001, "later point in the same bucket wins")
        self.assertEqual(vis[1][0], 2000)
        self.assertAlmostEqual(vis[0][2], 0.2, places=5)
        self.assertEqual(list(st.visible(10.6)), [], "expired after ttl")
        self.assertEqual(st.points_received, 4)


class RadarTracksTests(unittest.TestCase):
    def pkt(self, x, y, spd=50):
        return rp.RadarPacket(1, [rp.RadarTarget(x, y, spd, True), rp.RadarTarget(), rp.RadarTarget()])

    def test_hold_and_still_filter(self):
        tr = rs.RadarTracks(hold_s=0.4, still_timeout_s=2.0, move_thresh_mm=15)
        tr.update(self.pkt(500, 2000), 0.0)
        self.assertEqual(len(tr.shown(0.1)), 1)
        self.assertEqual(tr.shown(0.5), [], "hold time elapsed")
        for t in (0.5, 1.0, 1.5, 2.0, 2.5):
            tr.update(self.pkt(503, 2004), t)              # jitter below threshold = still
        self.assertEqual(tr.shown(2.6), [], "frozen target hidden after 2 s")
        tr.update(self.pkt(700, 2004), 2.7)                 # moved again
        self.assertEqual(len(tr.shown(2.8)), 1)
        self.assertAlmostEqual(tr.closest_mm(2.8), (700 ** 2 + 2004 ** 2) ** 0.5)

    def test_still_filter_disabled(self):
        tr = rs.RadarTracks(still_timeout_s=0)
        for t in (0.0, 1.0, 2.0, 3.0):
            tr.update(self.pkt(500, 2000), t)
        self.assertEqual(len(tr.shown(3.1)), 1)


class RecordReplayTests(unittest.TestCase):
    def test_roundtrip_timing(self):
        buf = io.BytesIO()
        rec = rs.Recorder(buf, start=100.0)
        a = rp.encode_hello(1)
        b = rp.encode_status(2, rp.StatusPacket(uptime_ms=5))
        rec.write(100.0, a)
        rec.write(100.25, b)
        buf.seek(0)
        rep = rs.Replayer(buf, speed=1.0)
        self.assertEqual(rep.due(50.0), [a])
        self.assertEqual(rep.due(50.1), [])
        self.assertEqual(rep.due(50.3), [b])
        self.assertTrue(rep.finished)

    def test_rejects_other_files(self):
        with self.assertRaises(ValueError):
            rs.Replayer(io.BytesIO(b"nope"))


if __name__ == "__main__":
    unittest.main()
