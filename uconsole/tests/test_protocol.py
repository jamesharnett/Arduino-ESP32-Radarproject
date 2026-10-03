"""Cross-checks uconsole/radar_protocol.py against the C header.

Run from the repository root:  python3 -m unittest discover -s uconsole/tests
"""
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))

import radar_protocol as rp  # noqa: E402

C_HEADER_DIR = ROOT / "Firmware" / "sensor_node_esp32s3"


def radar_vector() -> rp.RadarPacket:
    return rp.RadarPacket(1234, [
        rp.RadarTarget(-782, 1713, -16, True),
        rp.RadarTarget(999, 0, 0, False),      # invalid slot must encode as zeros
        rp.RadarTarget(2500, 7000, 120, True),
    ])


def lidar3_vector() -> rp.LidarPacket:
    return rp.LidarPacket(3600, 0, [rp.LidarPoint(0, 1500, 200), rp.LidarPoint(9000, 12000, 7),
                                    rp.LidarPoint(35999, 20, 255)])


def lidar96_vector() -> rp.LidarPacket:
    return rp.LidarPacket(3598, 0, [rp.LidarPoint(i * 375, 100 + i * 7, i) for i in range(96)])


def status_vector() -> rp.StatusPacket:
    return rp.StatusPacket(123456789, 20, 375, 4321, 2,
                           rp.STATUS_RADAR_OK | rp.STATUS_LIDAR_OK | rp.STATUS_LIDAR_ENABLED, 3, 4, 0)


class PythonRoundTrip(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(rp.RADAR_PACKET_LEN, 36)
        self.assertEqual(rp.STATUS_PACKET_LEN, 32)
        self.assertEqual(rp.HELLO_PACKET_LEN, 12)
        self.assertEqual(rp.lidar_packet_len(96), 492)
        self.assertLessEqual(rp.lidar_packet_len(rp.LIDAR_MAX_POINTS), rp.MAX_PACKET)

    def test_radar(self):
        data = rp.encode_radar(7, radar_vector())
        self.assertEqual(len(data), 36)
        self.assertEqual(data[:4], b"RDL1")
        ptype, seq, pkt = rp.decode(data)
        self.assertEqual((ptype, seq), (rp.TYPE_RADAR, 7))
        self.assertEqual(pkt.frame_count, 1234)
        self.assertEqual((pkt.targets[0].x_mm, pkt.targets[0].y_mm, pkt.targets[0].speed_cm_s), (-782, 1713, -16))
        self.assertFalse(pkt.targets[1].valid)
        self.assertEqual(pkt.targets[1].x_mm, 0)
        self.assertTrue(pkt.targets[2].valid)

    def test_lidar(self):
        pkt = lidar96_vector()
        data = rp.encode_lidar(1, pkt)
        self.assertEqual(len(data), 492)
        _, _, back = rp.decode(data)
        self.assertEqual(back.points, pkt.points)
        self.assertEqual(back.scan_speed_deg_s, 3598)
        self.assertIsNone(rp.decode(data[:-1]), "truncated lidar packet must be rejected")

    def test_status_hello(self):
        s = status_vector()
        self.assertEqual(rp.decode(rp.encode_status(300, s))[2], s)
        h = rp.decode(rp.encode_hello(2, rp.RECEIVER_GIGA, rp.WANT_RADAR | rp.WANT_STATUS))[2]
        self.assertEqual((h.receiver_kind, h.wants), (rp.RECEIVER_GIGA, 0x05))

    def test_rejects_foreign(self):
        self.assertIsNone(rp.decode(b"\x00" * 40))
        bad_version = bytearray(rp.encode_hello(1))
        bad_version[5] = 2
        self.assertIsNone(rp.decode(bytes(bad_version)))
        self.assertIsNone(rp.decode(rp.encode_header(0x7F, 1) + b"\x00" * 8), "unknown type")
        self.assertIsNone(rp.decode(rp.encode_radar(1, radar_vector())[:35]), "short radar")

    def test_seq_newer(self):
        self.assertTrue(rp.seq_newer(1, 0))
        self.assertTrue(rp.seq_newer(0, 65535))
        self.assertFalse(rp.seq_newer(5, 5))
        self.assertFalse(rp.seq_newer(65535, 0))
        self.assertFalse(rp.seq_newer(10, 20))


@unittest.skipUnless(shutil.which("gcc"), "gcc not available")
class CrossLanguage(unittest.TestCase):
    """The C header and the Python module must produce identical bytes."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        exe = os.path.join(cls.tmp, "vectors")
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        "-I", str(C_HEADER_DIR), str(HERE / "protocol_vectors.c"), "-o", exe], check=True)
        out = subprocess.run([exe], check=True, capture_output=True, text=True).stdout.split()
        cls.vectors = {}
        it = iter(out)
        for name in it:
            if name == "ok":
                break
            cls.vectors[name] = bytes.fromhex(next(it))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_radar_bytes_match(self):
        self.assertEqual(rp.encode_radar(7, radar_vector()), self.vectors["radar"])

    def test_lidar_bytes_match(self):
        self.assertEqual(rp.encode_lidar(65535, lidar3_vector()), self.vectors["lidar3"])
        self.assertEqual(rp.encode_lidar(1, lidar96_vector()), self.vectors["lidar96"])

    def test_status_hello_bytes_match(self):
        self.assertEqual(rp.encode_status(300, status_vector()), self.vectors["status"])
        self.assertEqual(rp.encode_hello(2, rp.RECEIVER_GIGA, rp.WANT_RADAR | rp.WANT_STATUS), self.vectors["hello"])

    def test_python_decodes_c_bytes(self):
        ptype, seq, pkt = rp.decode(self.vectors["radar"])
        self.assertEqual((ptype, seq, pkt), (rp.TYPE_RADAR, 7, radar_vector_normalised()))
        _, _, lid = rp.decode(self.vectors["lidar96"])
        self.assertEqual(lid, lidar96_vector())
        _, _, st = rp.decode(self.vectors["status"])
        self.assertEqual(st, status_vector())

    def test_header_copies_identical(self):
        giga = (ROOT / "Firmware" / "giga_display" / "radar_protocol.h").read_bytes()
        node = (C_HEADER_DIR / "radar_protocol.h").read_bytes()
        self.assertEqual(giga, node, "run tools/check_protocol_sync.py --fix")


def radar_vector_normalised() -> rp.RadarPacket:
    """What a decoder must return: invalid slots come back as all-zero."""
    v = radar_vector()
    v.targets[1] = rp.RadarTarget(0, 0, 0, False)
    return v


if __name__ == "__main__":
    unittest.main()
