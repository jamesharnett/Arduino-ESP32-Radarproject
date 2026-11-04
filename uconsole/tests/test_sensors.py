"""radar_sensors.py (Python) against rd03d_parser.h / ld19_parser.h (C++), byte for byte.

Builds a deterministic stream of good frames, corrupt frames and garbage for
each sensor, decodes it with both implementations and compares every value.
The C++ side is skipped (not failed) when no g++ is installed.
"""
import os
import pathlib
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))

import radar_protocol as rp  # noqa: E402
import radar_sensors as sn   # noqa: E402

C_HEADER_DIR = ROOT / "Firmware" / "sensor_node_esp32s3"


# ------------------------------------------------------------- stream builders
def radar_stream(seed: int = 1) -> bytes:
    rnd = random.Random(seed)
    out = bytearray(b"\x00\xAA\xAA\x12")                      # garbage and a false header start
    out += sn.RD03D_CMD_ENABLE_CONFIG                         # a command on the line (as a probe would see its own TX)
    out += sn.RD03D_CMD_HEADER + bytes((0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00)) + sn.RD03D_CMD_TAIL  # ACK
    for _ in range(120):
        slots = []
        for _ in range(3):
            kind = rnd.random()
            if kind < 0.3:
                slots.append(None)
            elif kind < 0.4:
                slots.append((rnd.randint(-8000, 8000), rnd.randint(0, 9000), rnd.choice((0, 248, -256))))   # sentinel speeds
            elif kind < 0.5:
                slots.append((rnd.randint(-60, 60), rnd.randint(0, 70), rnd.randint(-50, 50)))                 # too close
            else:
                slots.append((rnd.randint(-5000, 5000), rnd.randint(0, 8500), rnd.randint(-300, 300)))
        # make clustering happen often: slot 2 near slot 1
        if rnd.random() < 0.4 and slots[0]:
            slots[1] = (slots[0][0] + rnd.randint(-600, 600), slots[0][1] + rnd.randint(-600, 600), rnd.randint(-300, 300))
        frame = bytearray(sn.rd03d_build_frame(slots))
        r = rnd.random()
        if r < 0.08:
            frame[28] ^= 0xFF                                  # bad tail
        elif r < 0.12:
            frame = frame[:rnd.randint(5, 28)]                 # truncated frame (next header resyncs)
        elif r < 0.16:
            out += bytes(rnd.randint(0, 255) for _ in range(rnd.randint(1, 5)))   # noise between frames
        out += frame
    # datasheet example last, so it is always present
    out += sn.RD03D_HEADER + bytes((0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x68, 0x01)) + bytes(16) + sn.RD03D_TAIL
    return bytes(out)


def lidar_stream(seed: int = 2) -> bytes:
    rnd = random.Random(seed)
    out = bytearray(b"\x54\x00\x54\x54\x2C")                  # false starts: 54 then not 2C; 54 54 2C is a real start of garbage
    out += bytes(rnd.randint(0, 255) for _ in range(44))      # ... which fails the CRC
    angle = 0
    for n in range(150):
        speed = 3600 + rnd.randint(-100, 100)
        span = rnd.randint(900, 1000)
        start = angle % 36000
        end = (angle + span) % 36000
        angle += span
        dist = [0 if rnd.random() < 0.1 else rnd.randint(20, 12000) for _ in range(12)]
        inten = [rnd.randint(0, 255) for _ in range(12)]
        corrupt = rnd.random() < 0.07
        frame = sn.ld19_build_frame(speed, start, end, dist, inten, (n * 3) % 30000, corrupt_crc=corrupt)
        if rnd.random() < 0.05:
            frame = frame[:rnd.randint(2, 46)]                 # truncated
        if rnd.random() < 0.05:
            out += bytes(rnd.randint(0, 255) for _ in range(rnd.randint(1, 3)))
        out += frame
    return bytes(out)


def have_gxx() -> bool:
    return shutil.which("g++") is not None


def run_cpp(sensor: str, stream: bytes) -> str:
    with tempfile.TemporaryDirectory() as tmp:
        exe = os.path.join(tmp, "sensor_vectors")
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", f"-I{C_HEADER_DIR}",
                        str(HERE / "sensor_vectors.cpp"), "-o", exe], check=True)
        path = os.path.join(tmp, "stream.bin")
        pathlib.Path(path).write_bytes(stream)
        return subprocess.run([exe, sensor, path], check=True, capture_output=True, text=True).stdout


# --------------------------------------------------------------------- tests
class Rd03dPython(unittest.TestCase):
    def test_decode_datasheet_example(self):
        self.assertEqual(sn.rd03d_decode(0x030E), -782)
        self.assertEqual(sn.rd03d_decode(0x86B1), 1713)
        self.assertEqual(sn.rd03d_decode(0x0010), -16)
        self.assertEqual(sn.rd03d_decode(0x8000), 0)
        self.assertEqual(sn.rd03d_decode(0x0000), 0)
        for v in (-782, 1713, -16, 0, 32767, -32767):
            self.assertEqual(sn.rd03d_decode(sn.rd03d_encode(v)), v)

    def test_parser_resync_and_counts(self):
        good = sn.rd03d_build_frame([(500, 2000, 30)])
        bad = bytearray(good); bad[28] = 0
        p = sn.Rd03dParser()
        frames = p.feed_bytes(b"\x00\xAA\xAA\x12" + good + bytes(bad) + good)
        self.assertEqual(len(frames), 2)
        self.assertEqual((p.frames, p.bad_frames), (2, 1))
        self.assertEqual(frames[0].offset, 4)
        self.assertEqual(frames[1].offset, 4 + 60)

    def test_process_matches_firmware_rules(self):
        f = sn.Rd03dFrame(sn.rd03d_build_frame([(500, 2000, 30), (600, 2300, -80), (100, 9000, 10)])[4:28])
        t = sn.rd03d_process(f)
        self.assertEqual((t[0].x_mm, t[0].y_mm, t[0].speed_cm_s, t[0].valid), (550, 2150, -80, True))
        self.assertFalse(t[1].valid or t[2].valid)
        f = sn.Rd03dFrame(sn.rd03d_build_frame([(500, 2000, 248), (-500, 2000, -256), (0, 3000, 0)])[4:28])
        self.assertFalse(any(x.valid for x in sn.rd03d_process(f)))
        self.assertTrue(all(x.valid for x in sn.rd03d_process(f, sn.Rd03dOptions(filter_sentinels=False))))
        # C truncation toward zero when averaging negative coordinates
        f = sn.Rd03dFrame(sn.rd03d_build_frame([(-501, 2000, 30), (-600, 2300, 10)])[4:28])
        self.assertEqual(sn.rd03d_process(f)[0].x_mm, -550)
        f = sn.Rd03dFrame(sn.rd03d_build_frame([(-501, 2001, 30), (-600, 2300, 10)])[4:28])
        self.assertEqual((sn.rd03d_process(f)[0].x_mm, sn.rd03d_process(f)[0].y_mm), (-550, 2150))

    def test_acks(self):
        data = b"junk" + sn.RD03D_CMD_HEADER + bytes((0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00)) + sn.RD03D_CMD_TAIL \
            + sn.RD03D_CMD_HEADER + bytes((0x04, 0x00, 0x90, 0x01, 0x01, 0x00)) + sn.RD03D_CMD_TAIL
        acks = sn.rd03d_find_acks(data)
        self.assertEqual(len(acks), 2)
        self.assertEqual((acks[0].name, acks[0].ok, acks[0].data), ("enable configuration", True, bytes((0x01, 0x00, 0x40, 0x00))))
        self.assertEqual((acks[1].name, acks[1].ok, acks[1].status), ("multi-target mode", False, 1))


class Ld19Python(unittest.TestCase):
    def test_crc_table_and_frame(self):
        self.assertEqual(sn.LD19_CRC_TABLE[:4], (0x00, 0x4D, 0x9A, 0xD7))
        self.assertEqual(sn.LD19_CRC_TABLE[255], 0xA8)
        f = sn.ld19_build_frame(3600, 100, 1050, [1000 + i for i in range(12)], list(range(12)), 1234)
        p = sn.Ld19Parser()
        frames = p.feed_bytes(b"\x54\x00" + f + bytes(f[:-1]) + bytes((f[-1] ^ 1,)) + f)
        self.assertEqual((p.frames, p.crc_errors), (2, 1))
        self.assertEqual((frames[0].speed_deg_s, frames[0].start_angle_cdeg, frames[0].end_angle_cdeg, frames[0].timestamp_ms), (3600, 100, 1050, 1234))
        self.assertEqual(frames[0].points()[0], (100, 1000, 0))
        self.assertEqual(frames[0].points()[11], (1050, 1011, 11))
        self.assertEqual(frames[0].offset, 2)

    def test_angle_wrap(self):
        self.assertEqual(sn.ld19_point_angle_cdeg(35900, 300, 0), 35900)
        self.assertEqual(sn.ld19_point_angle_cdeg(35900, 300, 11), 300)
        self.assertEqual(sn.ld19_point_angle_cdeg(0, 0, 5), 0)


@unittest.skipUnless(have_gxx(), "g++ not installed")
class CrossCheck(unittest.TestCase):
    def test_radar_against_cpp(self):
        stream = radar_stream()
        out = run_cpp("radar", stream)
        lines = out.strip().splitlines()
        counts = lines.pop()
        p = sn.Rd03dParser()
        py = []
        for f in p.feed_bytes(stream):
            t = sn.rd03d_process(f)
            py.append(f"frame {f.offset} |" + "".join(f" {x.x_mm} {x.y_mm} {x.speed_cm_s} {1 if x.valid else 0}" for x in t))
        self.assertEqual(counts, f"frames={p.frames} bad={p.bad_frames}")
        self.assertGreater(p.frames, 90)
        self.assertGreater(p.bad_frames, 3)
        self.assertEqual(py, lines)
        self.assertTrue(lines[-1].startswith("frame ") and " -782 1713 -16 1" in lines[-1])

    def test_lidar_against_cpp(self):
        stream = lidar_stream()
        out = run_cpp("lidar", stream)
        lines = out.strip().splitlines()
        counts = lines.pop()
        p = sn.Ld19Parser()
        py = []
        for f in p.feed_bytes(stream):
            py.append(f"frame {f.offset} {f.speed_deg_s} {f.start_angle_cdeg} {f.end_angle_cdeg} {f.timestamp_ms} |"
                      + "".join(f" {a} {d} {i}" for a, d, i in f.points()))
        self.assertEqual(counts, f"frames={p.frames} crc={p.crc_errors}")
        self.assertGreater(p.frames, 120)
        self.assertGreater(p.crc_errors, 5)
        self.assertEqual(py, lines)


class HexDumpAndAnalysis(unittest.TestCase):
    def test_parse_hex_dump_formats(self):
        frame = sn.rd03d_build_frame([(-782, 1713, -16)])
        console = "\n".join(["[raw radar] capturing 600 bytes (gives up after 10000 ms) ...",
                             "[stat] up=12s ap=up ...",
                             "[raw radar] 30 bytes in 50 ms; paste everything from here to 'end' into tools/decode_capture.py"]
                            + sn.format_hex_lines(frame, "[raw radar] ", 16) + ["[raw radar] end"])
        src, data = sn.parse_hex_dump(console)
        self.assertEqual((src, data), ("radar", frame))
        src, data = sn.parse_hex_dump("0xAA, 0xFF, 0x03, 0x00,\n0x55,0xCC // comment")
        self.assertEqual((src, data), (None, bytes((0xAA, 0xFF, 0x03, 0x00, 0x55, 0xCC))))
        src, data = sn.parse_hex_dump("aaff0300\n55cc\n")
        self.assertEqual(data, bytes((0xAA, 0xFF, 0x03, 0x00, 0x55, 0xCC)))

    def test_analysis_hints(self):
        rep = sn.analyze_capture(b"", "radar")
        self.assertEqual(rep.frames, 0)
        self.assertIn("silent", rep.hints[0])
        rep = sn.analyze_capture(bytes(500), "lidar")
        self.assertTrue(any("idle" in h for h in rep.hints))
        rnd = random.Random(5)
        rep = sn.analyze_capture(bytes(rnd.randint(0, 255) for _ in range(600)), "radar")
        self.assertTrue(any("256000" in h for h in rep.hints) or any("tail" in h for h in rep.hints), rep.hints)
        rep = sn.analyze_capture(bytes((0x55,)) * 600, "radar")                 # one stuck value
        self.assertTrue(any("distinct byte values" in h for h in rep.hints), rep.hints)
        good = sn.rd03d_build_frame([(500, 2000, 30)]) * 20
        rep = sn.analyze_capture(good, "radar")
        self.assertEqual((rep.frames, rep.bad_frames, rep.skipped_bytes), (20, 0, 0))
        self.assertEqual(rep.slot_usage, [20, 0, 0])
        self.assertTrue(any("single-target" in h for h in rep.hints))
        self.assertTrue(rep.ok)
        rep = sn.analyze_capture(sn.rd03d_build_frame([]) * 10, "radar")
        self.assertTrue(any("empty" in h for h in rep.hints))
        f = sn.ld19_build_frame(3600, 0, 960, [500] * 12, [10] * 12, 0)
        rep = sn.analyze_capture(f * 50, "lidar")
        self.assertEqual((rep.frames, rep.bad_frames), (50, 0))
        self.assertAlmostEqual(rep.lidar_implied_speed, 3600, delta=5)
        self.assertEqual(rep.hints, [])
        self.assertEqual(sn.guess_sensor(f * 5), "lidar")
        self.assertEqual(sn.guess_sensor(good), "radar")


if __name__ == "__main__":
    unittest.main()
