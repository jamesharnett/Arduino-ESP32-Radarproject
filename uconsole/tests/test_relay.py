"""End-to-end test of radar_relay.py over loopback: fake node -> relay -> remote viewer (this test).

Runs the simulated sensor node and the relay as subprocesses on ephemeral ports,
then acts as a remote viewer: HELLO to the relay, expect forwarded RADAR/LIDAR/
STATUS datagrams with intact sequence numbers; fetch the web state and drive
arm/disarm/learn through the HTTP endpoints.
"""
import json
import os
import pathlib
import re
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request

HERE = pathlib.Path(__file__).resolve().parent
UCONSOLE = HERE.parent
sys.path.insert(0, str(UCONSOLE))

import radar_protocol as rp  # noqa: E402


def wait_for(proc, pattern, timeout=10.0):
    """Read the process's stdout until a line matches pattern; returns the match."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                raise AssertionError("process exited early")
            time.sleep(0.05)
            continue
        m = re.search(pattern, line)
        if m:
            return m
    raise AssertionError(f"timeout waiting for {pattern!r}")


class RelayLoopbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        # fake node on an ephemeral port: the script prints the port it bound
        cls.node = subprocess.Popen([sys.executable, str(UCONSOLE / "fake_sensor_node.py"), "--port", "0", "--quiet-room",
                                     "--intruder-after", "4", "--intruder-stay", "30"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        cls.node_port = int(wait_for(cls.node, r"serving on UDP (\d+)").group(1))
        cls.relay = subprocess.Popen([sys.executable, str(UCONSOLE / "radar_relay.py"), "--node", "127.0.0.1",
                                      "--node-port", str(cls.node_port), "--serve-port", "0", "--web", "0",
                                      "--web-token", "t0k", "--learn", "2", "--baseline", os.path.join(cls.tmp, "b.json"),
                                      "--alert-log", os.path.join(cls.tmp, "a.log"), "--max-seconds", "40"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        cls.web_port = int(wait_for(cls.relay, r"web view on http://0\.0\.0\.0:(\d+)/").group(1))
        cls.serve_port = int(wait_for(cls.relay, r"serving remote viewers on UDP (\d+)").group(1))

    @classmethod
    def tearDownClass(cls):
        for p in (cls.relay, cls.node):
            p.kill()
            p.wait()
            if p.stdout:
                p.stdout.close()

    def test_forwarding_and_web(self):
        # --- act as a remote viewer
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 0))
        sock.settimeout(0.5)
        seen = {}
        last_seq = {}
        deadline = time.monotonic() + 8.0
        next_hello = 0.0
        while time.monotonic() < deadline and len(seen) < 3:
            if time.monotonic() >= next_hello:
                sock.sendto(rp.encode_hello(1, rp.RECEIVER_OTHER, rp.WANT_ALL), ("127.0.0.1", self.serve_port))
                next_hello = time.monotonic() + 0.5
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            dec = rp.decode(data)
            self.assertIsNotNone(dec, "relay forwarded something that is not RadarLink")
            ptype, seq, pkt = dec
            if ptype in last_seq:
                self.assertTrue(rp.seq_newer(seq, last_seq[ptype]), "sequence numbers must stay in order")
            last_seq[ptype] = seq
            seen[ptype] = pkt
        self.assertEqual(set(seen), {rp.TYPE_RADAR, rp.TYPE_LIDAR, rp.TYPE_STATUS})
        self.assertGreater(len(seen[rp.TYPE_LIDAR].points), 0)

        # --- web state
        base = f"http://127.0.0.1:{self.web_port}"
        page = urllib.request.urlopen(base + "/", timeout=5).read().decode()
        self.assertIn("<canvas", page)
        state = json.loads(urllib.request.urlopen(base + "/state.json", timeout=5).read())
        self.assertTrue(state["alive"])
        self.assertGreater(len(state["lidar"]), 100)
        self.assertGreaterEqual(state["remote_viewers"], 1)

        # --- control endpoints: token enforced, then disarm/arm round trip once the baseline exists
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(urllib.request.Request(base + "/arm", method="POST"), timeout=5)
        self.assertEqual(cm.exception.code, 403)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            state = json.loads(urllib.request.urlopen(base + "/state.json", timeout=5).read())
            if state["baseline_buckets"] > 0 and state["learning_left"] is None:
                break
            time.sleep(0.25)
        self.assertGreater(state["baseline_buckets"], 500, "relay learned a baseline from the empty room")
        self.assertTrue(state["armed"], "--learn arms when learning completes")
        urllib.request.urlopen(urllib.request.Request(base + "/disarm?token=t0k", method="POST"), timeout=5)
        time.sleep(0.4)
        self.assertFalse(json.loads(urllib.request.urlopen(base + "/state.json", timeout=5).read())["armed"])
        urllib.request.urlopen(urllib.request.Request(base + "/arm?token=t0k", method="POST"), timeout=5)
        # --- the intruder walks in at 4 s: expect an alert within a few seconds
        deadline = time.monotonic() + 15.0
        alerts = []
        while time.monotonic() < deadline and not alerts:
            state = json.loads(urllib.request.urlopen(base + "/state.json", timeout=5).read())
            alerts = state["alerts"]
            time.sleep(0.25)
        self.assertTrue(alerts, "intruder should raise an alert")
        self.assertIn(alerts[0]["kind"], ("radar", "fused", "lidar"))
        sock.close()


if __name__ == "__main__":
    unittest.main()
