"""uconsole/install.sh with the system tools replaced by shims (no root, no NetworkManager needed)."""
import os
import pathlib
import stat
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
UCONSOLE = HERE.parent
INSTALL = UCONSOLE / "install.sh"

SHIM = """#!/usr/bin/env bash
echo "{name} $*" >> "$SHIM_LOG"
{body}
"""


def make_shims(bindir: pathlib.Path, nm_profiles: str = "", ufw_status: str = "Status: inactive") -> None:
    shims = {
        "sudo": 'exec "$@"',
        "apt-get": "exit 0",
        "systemctl": "exit 0",
        "ufw": f'if [ "$1" = status ]; then echo "{ufw_status}"; fi; exit 0',
        "nmcli": f'if [ "$1" = -t ] && [ "$3" = NAME ]; then printf "%s" "{nm_profiles}"; fi; exit 0',
        "hostname": "echo 192.168.4.9",
    }
    for name, body in shims.items():
        p = bindir / name
        p.write_text(SHIM.format(name=name, body=body))
        p.chmod(p.stat().st_mode | stat.S_IEXEC)


class Installer(unittest.TestCase):
    def run_install(self, *args, nm_profiles="", ufw_status="Status: inactive", env_extra=None):
        tmp = tempfile.mkdtemp()
        self.addCleanup(lambda: subprocess.run(["rm", "-rf", tmp]))
        bindir = pathlib.Path(tmp, "bin")
        bindir.mkdir()
        make_shims(bindir, nm_profiles, ufw_status)
        home = pathlib.Path(tmp, "home")
        home.mkdir()
        log = pathlib.Path(tmp, "shim.log")
        log.write_text("")
        env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}", HOME=str(home), SHIM_LOG=str(log), SUDO_USER="")
        env.pop("SUDO_USER")
        if env_extra:
            env.update(env_extra)
        r = subprocess.run(["bash", str(INSTALL), "--prefix", tmp, *args], capture_output=True, text=True, env=env, timeout=60)
        return r, tmp, home, log.read_text()

    def test_viewer_autostart_and_wifi(self):
        r, tmp, home, log = self.run_install("--password", "s3cret-pass", "--viewer-autostart", "--node", "192.168.4.1")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        desktop = home / ".config" / "autostart" / "radar-viewer.desktop"
        self.assertTrue(desktop.exists(), r.stdout)
        text = desktop.read_text()
        self.assertIn("radar_viewer.py --node 192.168.4.1", text)
        self.assertIn(f"Path={UCONSOLE}", text)
        self.assertIn("apt-get install -y --no-install-recommends python3 python3-pygame python3-numpy iw network-manager", log)
        self.assertIn("nmcli con add type wifi con-name RadarSystem ifname wlan0 ssid RadarSystem", log)
        self.assertIn("ipv4.never-default yes", log)
        self.assertIn("802-11-wireless.powersave 2", log)
        self.assertIn("connection.autoconnect-retries 0", log)
        self.assertIn("wifi-sec.psk s3cret-pass", log)                              # the shim saw the real password
        self.assertNotIn("s3cret-pass", r.stdout)                                   # but the console output masks it
        self.assertIn("********", r.stdout)
        self.assertIn("doctor.py", r.stdout)

    def test_existing_profile_is_modified_not_added(self):
        r, tmp, home, log = self.run_install("--password", "x", nm_profiles="RadarSystem\nHomeNet\n")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertNotIn("nmcli con add", log)
        self.assertIn("nmcli con modify RadarSystem ipv4.method auto", log)
        self.assertIn("profile exists, updating it", r.stdout)

    def test_relay_service_and_firewall(self):
        r, tmp, home, log = self.run_install("--no-wifi", "--no-apt", "--relay", "--web-port", "8090", "--web-token", "tok", ufw_status="Status: active")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        unit = pathlib.Path(tmp, "etc/systemd/system/radar-relay.service")
        self.assertTrue(unit.exists(), r.stdout)
        text = unit.read_text()
        self.assertIn("--web 8090 --web-token tok --armed", text)
        self.assertIn(f"WorkingDirectory={UCONSOLE}", text)
        self.assertIn(f"--baseline {home}/radarlink/baseline.json", text)
        self.assertTrue((home / "radarlink").is_dir())
        self.assertIn("ufw allow 4210/udp", log)
        self.assertIn("ufw allow 8090/tcp", log)
        self.assertNotIn("nmcli", log)
        self.assertNotIn("apt-get", log)
        self.assertIn("systemd not touched", r.stdout)                              # prefix mode never calls systemctl enable

    def test_dry_run_writes_nothing(self):
        r, tmp, home, log = self.run_install("--dry-run", "--password", "pw", "--viewer-autostart", "--relay")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertFalse((home / ".config").exists())
        self.assertFalse(pathlib.Path(tmp, "etc").exists())
        for mutating in ("con add", "con modify", "apt-get", "systemctl", "ufw allow", "mkdir"):
            self.assertNotIn(mutating, log, log)                                   # only read-only queries ran ...
        self.assertIn("+ nmcli con add", r.stdout)                                  # ... but every step was printed
        self.assertIn("+ write", r.stdout)
        self.assertIn("DRY RUN", r.stdout)

    def test_uninstall(self):
        r, tmp, home, log = self.run_install("--password", "pw", "--viewer-autostart", "--relay", "--no-apt")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        desktop = home / ".config" / "autostart" / "radar-viewer.desktop"
        self.assertTrue(desktop.exists())
        bindir = pathlib.Path(tmp, "bin")
        env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}", HOME=str(home), SHIM_LOG=str(pathlib.Path(tmp, "shim.log")))
        env.pop("SUDO_USER", None)
        make_shims(bindir, nm_profiles="RadarSystem\n")
        r2 = subprocess.run(["bash", str(INSTALL), "--prefix", tmp, "--uninstall"], capture_output=True, text=True, env=env, timeout=60)
        self.assertEqual(r2.returncode, 0, r2.stdout + r2.stderr)
        self.assertFalse(desktop.exists())
        self.assertFalse(pathlib.Path(tmp, "etc/systemd/system/radar-relay.service").exists())
        self.assertIn("nmcli con delete RadarSystem", pathlib.Path(tmp, "shim.log").read_text())

    def test_help_and_bad_option(self):
        r = subprocess.run(["bash", str(INSTALL), "--help"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0)
        self.assertIn("--viewer-autostart", r.stdout)
        r = subprocess.run(["bash", str(INSTALL), "--bogus"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 2)

    def test_shell_syntax(self):
        self.assertEqual(subprocess.run(["bash", "-n", str(INSTALL)]).returncode, 0)


if __name__ == "__main__":
    unittest.main()
