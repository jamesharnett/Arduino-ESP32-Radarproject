"""tools/decode_capture.py and tools/sensor_probe.py against synthetic captures (no hardware)."""
import io
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
TOOLS = ROOT / "tools"
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(TOOLS))

import radar_sensors as sn      # noqa: E402
import sensor_probe             # noqa: E402


def run_tool(*args, stdin: str = None):
    return subprocess.run([sys.executable, str(TOOLS / args[0]), *args[1:]], input=stdin, capture_output=True, text=True)


class DecodeCapture(unittest.TestCase):
    def test_console_paste_radar(self):
        frames = b"".join(sn.rd03d_build_frame([(500 + i, 2000, 30 + i), (600, 2300, -80)]) for i in range(10))
        paste = "\n".join(["[boot] ready", "[raw radar] capturing 300 bytes (gives up after 10000 ms) ...",
                           "[raw radar] 300 bytes in 480 ms; paste everything from here to 'end' into tools/decode_capture.py"]
                          + sn.format_hex_lines(frames, "[raw radar] ") + ["[raw radar] end", "[stat] up=20s ap=up sta=0 subs=0"])
        r = run_tool("decode_capture.py", "-", "--verbose", stdin=paste)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("radar: 300 bytes, 10 good frame(s), 0 bad, 0 byte(s) outside frames", r.stdout)
        self.assertIn("slot usage: 1=10 2=10 3=0", r.stdout)
        self.assertIn("node sends: [+550,+2150 2.22 m -80]", r.stdout)       # clustered as the firmware does

    def test_binary_lidar_and_json(self):
        data = b"".join(sn.ld19_build_frame(3600, (i * 960) % 36000, (i * 960 + 960) % 36000, [1000 + 10 * k for k in range(12)],
                                            [50] * 12, i, corrupt_crc=(i % 25 == 0)) for i in range(100))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "l.bin")
            pathlib.Path(path).write_bytes(data)
            r = run_tool("decode_capture.py", "--binary", "--sensor", "lidar", "--json", path)
            self.assertEqual(r.returncode, 0, r.stderr)
            import json
            rep = json.loads(r.stdout)
            self.assertEqual((rep["sensor"], rep["frames"], rep["bad_frames"]), ("lidar", 96, 4))
            self.assertEqual(rep["lidar_speed"], [3600, 3600])
            self.assertTrue(rep["ok"])
            r = run_tool("decode_capture.py", "--binary", path)                  # auto-detects lidar
            self.assertIn("(auto-detected lidar)", r.stdout)

    def test_bad_capture_exits_nonzero_with_hint(self):
        r = run_tool("decode_capture.py", "--sensor", "radar", "-", stdin="00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\n" * 20)
        self.assertEqual(r.returncode, 1)
        self.assertIn("hint:", r.stdout)
        self.assertIn("idle", r.stdout)


class FakePort:
    """Hands out a byte stream in chunks, one per read() call, with optional replies to commands."""

    def __init__(self, stream: bytes, chunk: int = 97, reply: bytes = b"") -> None:
        self.stream, self.pos, self.chunk, self.reply = stream, 0, chunk, reply
        self.written = bytearray()
        self.pending_reply = b""

    def read(self, n: int) -> bytes:
        if self.pending_reply:
            out, self.pending_reply = self.pending_reply, b""
            return out
        out = self.stream[self.pos:self.pos + self.chunk]
        self.pos += len(out)
        return out

    def write(self, data: bytes) -> None:
        self.written += data
        self.pending_reply = self.reply

    def close(self) -> None:
        pass


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        self.t += 0.02
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s


class SensorProbe(unittest.TestCase):
    def test_radar_probe_with_configuration(self):
        stream = b"".join(sn.rd03d_build_frame([(1000, 3000, -40)]) for _ in range(40))
        ack = sn.RD03D_CMD_HEADER + bytes((0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00)) + sn.RD03D_CMD_TAIL
        port = FakePort(stream, reply=ack)
        out = io.StringIO()
        clock = FakeClock()
        probe = sensor_probe.Probe("radar", port, out=out, clock=clock, sleep=clock.sleep)
        probe.configure_radar()
        rep = probe.run(3.0)
        text = out.getvalue()
        self.assertEqual(port.written, sn.RD03D_CMD_ENABLE_CONFIG + sn.RD03D_CMD_MULTI_TARGET + sn.RD03D_CMD_END_CONFIG)
        self.assertIn("module replied: enable configuration OK", text)
        self.assertIn("[+1000,+3000 mm 3.16 m  -40 cm/s]", text)
        numbers = [int(l.split()[1]) for l in text.splitlines() if l.startswith("frame ")]
        self.assertEqual(numbers, list(range(1, 41)))                                # every frame numbered in order
        self.assertEqual((rep.frames, rep.bad_frames), (40, 0))
        self.assertEqual(len(rep.acks), 3)                       # one reply per command
        self.assertTrue(rep.ok)

    def test_lidar_probe_hex_and_report(self):
        stream = b"".join(sn.ld19_build_frame(3550, (i * 950) % 36000, (i * 950 + 950) % 36000, [800] * 12, [9] * 12, i) for i in range(60))
        port = FakePort(stream, chunk=500)
        out = io.StringIO()
        clock = FakeClock()
        probe = sensor_probe.Probe("lidar", port, out=out, clock=clock, sleep=clock.sleep)
        rep = probe.run(2.0, hex_lines=True)
        text = out.getvalue()
        self.assertEqual(rep.frames, 60)
        self.assertIn("[raw lidar] 54 2C", text)
        src, data = sn.parse_hex_dump(text)                       # the hex lines round-trip through the decoder
        self.assertEqual((src, data), ("lidar", stream))
        self.assertIn("lidar:", text)                              # the once-a-second line

    def test_cli_without_pyserial_port(self):
        r = run_tool("sensor_probe.py", "--port", "/dev/does-not-exist-radarlink", "--sensor", "radar", "--seconds", "0.1")
        self.assertNotEqual(r.returncode, 0)
        self.assertTrue("cannot open" in r.stderr or "pyserial is not installed" in r.stderr, r.stderr)


if __name__ == "__main__":
    unittest.main()
