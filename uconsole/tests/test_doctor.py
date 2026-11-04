"""doctor.py with mocked system commands, plus a live HELLO/STATUS exchange with the fake sensor node."""
import pathlib
import re
import subprocess
import sys
import time
import unittest

HERE = pathlib.Path(__file__).resolve().parent
UCONSOLE = HERE.parent
sys.path.insert(0, str(UCONSOLE))

import doctor as dr  # noqa: E402

NMCLI_WIFI_OK = "yes:RadarSystem:72:6:wlan0\nno:HomeNet:55:11:wlan0\n"
NMCLI_WIFI_OTHER = "no:RadarSystem:72:6:wlan0\nyes:Home\\:Net:55:11:wlan0\n"
IP_ADDR_OK = '[{"ifname":"wlan0","addr_info":[{"family":"inet","local":"192.168.4.7","prefixlen":24}]}]'
IP_ROUTE_OTHER = '[{"dst":"default","gateway":"10.0.0.1","dev":"wwan0"}]'
IP_ROUTE_NODE = '[{"dst":"default","gateway":"192.168.4.1","dev":"wlan0"},{"dst":"default","gateway":"10.0.0.1","dev":"wwan0"}]'


def make_runner(table):
    """table maps a command prefix (space-joined) to (rc, output); None rc = tool missing."""
    calls = []

    def runner(argv, timeout=5.0):
        calls.append(argv)
        joined = " ".join(argv)
        for prefix, result in table.items():
            if joined.startswith(prefix):
                return result
        return None, ""
    runner.calls = calls
    return runner


def levels(d):
    return {(c.name, c.level) for c in d.checks}


class MockedSystem(unittest.TestCase):
    def doctor(self, table, env=None, **kw):
        d = dr.Doctor(runner=make_runner(table), env=env or {"WAYLAND_DISPLAY": "wayland-1", "XDG_SESSION_TYPE": "wayland"},
                      home="/nonexistent", **kw)
        for name in ("check_session", "check_wifi", "check_address", "check_route", "check_powersave", "check_firewall", "check_services"):
            getattr(d, name)()
        return d

    def test_all_good(self):
        d = self.doctor({"nmcli -t -f ACTIVE": (0, NMCLI_WIFI_OK), "nmcli -t -f NAME": (0, "RadarSystem\nHomeNet\n"),
                         "ip -j -4 addr": (0, IP_ADDR_OK), "ip -j route": (0, IP_ROUTE_OTHER),
                         "iw dev wlan0 get power_save": (0, "Power save: off\n"), "ufw status": (0, "Status: inactive\n"),
                         "systemctl --user is-active": (3, "inactive\n"), "systemctl is-active": (3, "inactive\n")})
        self.assertEqual(d.worst, "OK", d.render())
        self.assertIn(("wifi", "OK"), levels(d))
        self.assertIn(("address", "OK"), levels(d))
        self.assertIn(("route", "OK"), levels(d))
        self.assertIn(("powersave", "OK"), levels(d))

    def test_wrong_network_and_powersave(self):
        d = self.doctor({"nmcli -t -f ACTIVE": (0, NMCLI_WIFI_OTHER), "nmcli -t -f NAME": (0, "Home:Net\n"),
                         "ip -j -4 addr": (0, '[{"ifname":"wlan0","addr_info":[{"family":"inet","local":"192.168.1.20"}]}]'),
                         "ip -j route": (0, IP_ROUTE_NODE), "iw dev wlan0 get power_save": (0, "Power save: on\n"),
                         "ufw status": (0, "Status: active\n\nTo   Action  From\n22/tcp  ALLOW  Anywhere\n")})
        self.assertEqual(d.worst, "FAIL")
        wifi = [c for c in d.checks if c.name == "wifi"]
        self.assertEqual(wifi[0].level, "FAIL")
        self.assertIn("'Home:Net'", wifi[0].text)                       # escaped colon in nmcli output handled
        self.assertIn("nmcli con up 'RadarSystem'", wifi[0].fix)
        self.assertTrue(any(c.name == "wifi" and "no saved connection profile" in c.text for c in d.checks))
        self.assertIn(("address", "WARN"), levels(d))
        self.assertIn(("route", "WARN"), levels(d))
        self.assertIn(("powersave", "WARN"), levels(d))
        self.assertIn(("firewall", "WARN"), levels(d))
        self.assertTrue(any("ufw allow 4210/udp" in c.fix for c in d.checks))

    def test_iw_fallback_and_no_display(self):
        d = self.doctor({"iw dev wlan0 link": (0, "Connected to aa:bb:cc:dd:ee:ff (on wlan0)\n\tSSID: RadarSystem\n\tsignal: -80 dBm\n"),
                         "ip -j -4 addr": (0, "not json"), "ip -4 addr": (0, "    inet 192.168.4.9/24 brd ...\n"),
                         "ip -j route": (0, "default via 192.168.4.1 dev wlan0\n"),
                         "iw dev wlan0 get power_save": (0, "Power save: off\n")},
                        env={"XDG_SESSION_TYPE": "tty"})
        self.assertIn(("display", "WARN"), levels(d))
        wifi = [c for c in d.checks if c.name == "wifi"][0]
        self.assertEqual(wifi.level, "WARN")                             # weak signal
        self.assertIn(("address", "OK"), levels(d))
        self.assertIn(("route", "INFO"), levels(d))                      # the node is the only default route
        self.assertIn(("firewall", "OK"), levels(d))                     # ufw missing

    def test_no_address(self):
        d = self.doctor({"nmcli -t -f ACTIVE": (0, "no:RadarSystem:72:6:wlan0\n"), "nmcli -t -f NAME": (0, "RadarSystem\n"),
                         "ip -j -4 addr": (0, '[{"ifname":"wlan0","addr_info":[]}]'), "ip -j route": (0, "[]")})
        self.assertIn(("wifi", "FAIL"), levels(d))
        self.assertIn(("address", "FAIL"), levels(d))
        self.assertTrue(any("no default route" in c.text for c in d.checks))


class LiveNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = subprocess.Popen([sys.executable, str(UCONSOLE / "fake_sensor_node.py"), "--port", "0", "--quiet-room"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 10
        cls.port = None
        while time.time() < deadline and cls.port is None:
            line = cls.node.stdout.readline()
            m = re.search(r"serving on UDP (\d+)", line)
            if m:
                cls.port = int(m.group(1))
        if cls.port is None:
            cls.node.kill()
            raise RuntimeError("fake node did not start")

    @classmethod
    def tearDownClass(cls):
        cls.node.kill()
        cls.node.wait()

    def test_hello_status_exchange(self):
        d = dr.Doctor(node="127.0.0.1", port=self.port, timeout=5.0, runner=make_runner({}))
        d.check_node()
        node = [c for c in d.checks if c.name == "node"][0]
        self.assertEqual(node.level, "OK", node.text)
        self.assertIn("127.0.0.1 answered", node.text)
        self.assertIn("radar OK", node.text)
        self.assertIn("subscriber(s) incl. this check", node.text)

    def test_no_node_there(self):
        d = dr.Doctor(node="127.0.0.1", port=1, timeout=0.6, runner=make_runner({"ping": (0, "ok")}))
        d.check_node()
        node = [c for c in d.checks if c.name == "node"][0]
        self.assertEqual(node.level, "FAIL")
        self.assertIn("answers ping but sent no STATUS", node.text)

    def test_port_check(self):
        d = dr.Doctor(port=self.port, runner=make_runner({"ss -ulpn": (0, 'UNCONN 0 0 0.0.0.0:%d 0.0.0.0:* users:(("python3",pid=4242,fd=3))' % self.port)}))
        d.check_port()
        self.assertEqual(d.checks[0].level, "INFO")
        self.assertIn("python3 pid 4242", d.checks[0].text)

    def test_cli(self):
        r = subprocess.run([sys.executable, str(UCONSOLE / "doctor.py"), "--node", "127.0.0.1", "--port", str(self.port), "--json"],
                           capture_output=True, text=True, timeout=30)
        import json
        out = json.loads(r.stdout)
        self.assertIn(out["result"], ("OK", "INFO", "WARN", "FAIL"))
        node = [c for c in out["checks"] if c["name"] == "node"][0]
        self.assertEqual(node["level"], "OK", node)


if __name__ == "__main__":
    unittest.main()
