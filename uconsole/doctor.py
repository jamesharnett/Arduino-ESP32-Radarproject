#!/usr/bin/env python3
"""RadarLink doctor: is this uConsole (or any Linux box) ready to show the sensor node?

Checks, in order: the Python modules, the graphical session, the Wi-Fi link to
the node's network, the address and default route, Wi-Fi power save, the
firewall, whether UDP 4210 is free, and finally talks to the node itself
(HELLO -> STATUS) and prints what the node reports. Every finding comes with
the command that fixes it. Exit status 1 when something is broken.

    python3 doctor.py                          # SSID RadarSystem, node 192.168.4.1, interface wlan0
    python3 doctor.py --ssid MyRadar --node 192.168.4.1 --iface wlan0
    python3 doctor.py --json
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radar_protocol as rp  # noqa: E402

LEVELS = ("OK", "INFO", "WARN", "FAIL")
RunResult = Tuple[Optional[int], str]           # (exit status or None when the tool is missing, stdout+stderr)


SBIN_PATH = os.environ.get("PATH", "") + ":/usr/sbin:/sbin:/usr/local/sbin"   # iw and ufw live in sbin on Debian


def run_command(argv: List[str], timeout: float = 5.0) -> RunResult:
    exe = shutil.which(argv[0]) or shutil.which(argv[0], path=SBIN_PATH)
    if exe is None:
        return None, ""
    try:
        r = subprocess.run([exe] + list(argv[1:]), capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    return r.returncode, r.stdout + r.stderr


def nmcli_fields(line: str) -> List[str]:
    """Split one line of ``nmcli -t`` output: ':' separates fields, '\\' escapes the next character."""
    fields, cur, i = [], [], 0
    while i < len(line):
        c = line[i]
        if c == "\\" and i + 1 < len(line):
            cur.append(line[i + 1])
            i += 2
            continue
        if c == ":":
            fields.append("".join(cur))
            cur = []
        else:
            cur.append(c)
        i += 1
    fields.append("".join(cur))
    return fields


@dataclass
class Check:
    level: str
    name: str
    text: str
    fix: str = ""


@dataclass
class Doctor:
    ssid: str = "RadarSystem"
    node: str = "192.168.4.1"
    port: int = rp.PORT
    iface: str = "wlan0"
    timeout: float = 3.0
    runner: Callable[..., RunResult] = run_command
    env: Dict[str, str] = field(default_factory=lambda: dict(os.environ))
    home: str = field(default_factory=lambda: os.path.expanduser("~"))
    checks: List[Check] = field(default_factory=list)

    # ---------------------------------------------------------------- utils
    def add(self, level: str, name: str, text: str, fix: str = "") -> None:
        self.checks.append(Check(level, name, text, fix))

    @property
    def worst(self) -> str:
        """The verdict: INFO lines are notes, not problems, so a healthy run says OK."""
        order = {lv: i for i, lv in enumerate(LEVELS)}
        return max((c.level for c in self.checks if c.level != "INFO"), key=lambda lv: order[lv], default="OK")

    def node_net(self) -> str:
        return ".".join(self.node.split(".")[:3]) + "."

    # --------------------------------------------------------------- checks
    def check_python(self) -> None:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")     # keep pygame's banner out of --json output
        try:
            import pygame  # type: ignore
            self.add("OK", "python", f"pygame {pygame.version.ver} on Python {sys.version.split()[0]}")
        except Exception as exc:  # ImportError or a broken SDL
            self.add("FAIL", "python", f"pygame cannot be imported ({exc}); the viewer needs it (the relay does not)",
                     "sudo apt install python3-pygame   # or: pip install pygame")
        try:
            import numpy  # type: ignore  # noqa: F401
            self.add("OK", "python", "numpy present (sonar audio)")
        except Exception:
            self.add("INFO", "python", "numpy missing: the viewer runs without its sonar pings", "sudo apt install python3-numpy")

    def check_session(self) -> None:
        stype = self.env.get("XDG_SESSION_TYPE", "")
        wl, x11 = self.env.get("WAYLAND_DISPLAY"), self.env.get("DISPLAY")
        if wl:
            self.add("OK", "display", f"Wayland session ({wl}); pygame/SDL picks it up by itself"
                     + (" (SDL_VIDEODRIVER=%s set)" % self.env["SDL_VIDEODRIVER"] if self.env.get("SDL_VIDEODRIVER") else ""))
        elif x11:
            self.add("OK", "display", f"X11 display {x11}")
        elif stype == "tty" or not stype:
            self.add("WARN", "display", "no graphical session in this shell (no DISPLAY or WAYLAND_DISPLAY): the viewer window cannot open "
                     "from here; run it from the desktop's terminal, or install the headless relay instead",
                     "python3 radar_relay.py --web 8080   # then open http://<this host>:8080/ in a browser")
        else:
            self.add("INFO", "display", f"session type {stype}, no DISPLAY/WAYLAND_DISPLAY variable")

    def _nmcli_rows(self, args: List[str]) -> Optional[List[List[str]]]:
        rc, out = self.runner(["nmcli", "-t"] + args)
        if rc is None or rc != 0:
            return None
        rows = []
        for line in out.splitlines():
            if line.strip():
                rows.append(nmcli_fields(line))
        return rows

    def check_wifi(self) -> None:
        rows = self._nmcli_rows(["-f", "ACTIVE,SSID,SIGNAL,CHAN,DEVICE", "dev", "wifi"])
        if rows is not None:
            active = [r for r in rows if r and r[0] == "yes"]
            if not active:
                self.add("FAIL", "wifi", "not connected to any Wi-Fi network",
                         f"nmcli con up '{self.ssid}'   # or: nmcli dev wifi connect '{self.ssid}' password '...'")
                return
            for r in active:
                ssid, signal, chan, dev = (r + ["", "", "", ""])[1:5]
                if ssid == self.ssid:
                    try:
                        sig = int(signal)
                    except ValueError:
                        sig = -1
                    lvl = "OK" if sig >= 40 or sig < 0 else "WARN"
                    self.add(lvl, "wifi", f"connected to {ssid} on {dev}, channel {chan}, signal {signal}%"
                             + ("" if lvl == "OK" else ": weak, move closer or check the node's antenna"))
                else:
                    self.add("FAIL", "wifi", f"connected to '{ssid}' on {dev}, not to the node's network '{self.ssid}'",
                             f"nmcli con up '{self.ssid}'")
            profiles = self._nmcli_rows(["-f", "NAME", "con", "show"]) or []
            if not any(p and p[0] == self.ssid for p in profiles):
                self.add("INFO", "wifi", f"no saved connection profile named '{self.ssid}' yet",
                         f"uconsole/install.sh --ssid '{self.ssid}' --ask-password --no-apt")
            return
        rc, out = self.runner(["iw", "dev", self.iface, "link"])
        if rc is None:
            self.add("INFO", "wifi", "neither nmcli nor iw found: cannot check the Wi-Fi link")
            return
        m = re.search(r"SSID:\s*(.+)", out)
        sig = re.search(r"signal:\s*(-?\d+)", out)
        if not m:
            self.add("FAIL", "wifi", f"{self.iface} is not associated with any network", f"nmcli con up '{self.ssid}'")
        elif m.group(1).strip() != self.ssid:
            self.add("FAIL", "wifi", f"{self.iface} is on '{m.group(1).strip()}', not '{self.ssid}'", f"nmcli con up '{self.ssid}'")
        else:
            dbm = int(sig.group(1)) if sig else None
            lvl = "WARN" if dbm is not None and dbm < -75 else "OK"
            self.add(lvl, "wifi", f"associated with {self.ssid}" + (f", {dbm} dBm" if dbm is not None else ""))

    def check_address(self) -> None:
        rc, out = self.runner(["ip", "-j", "-4", "addr", "show", "dev", self.iface])
        addrs: List[str] = []
        if rc not in (0, None) and "does not exist" in out:
            self.add("FAIL", "address", f"interface {self.iface} does not exist on this machine",
                     "ip link   # find the Wi-Fi interface, then: doctor.py --iface <name>")
            return
        if rc == 0 and out.strip().startswith("["):
            try:
                for entry in json.loads(out):
                    addrs += [a["local"] for a in entry.get("addr_info", []) if a.get("family") == "inet"]
            except (ValueError, KeyError):
                pass
        elif rc is not None:
            rc, out = self.runner(["ip", "-4", "addr", "show", "dev", self.iface])
            addrs = re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", out)
        else:
            self.add("INFO", "address", "ip command not found: cannot check the address")
            return
        if not addrs:
            self.add("FAIL", "address", f"{self.iface} has no IPv4 address: the node's DHCP did not answer (node still booting, wrong "
                     "password, or not joined)", f"nmcli con up '{self.ssid}'")
        elif not any(a.startswith(self.node_net()) for a in addrs):
            self.add("WARN", "address", f"{self.iface} has {', '.join(addrs)}, not an address in the node's {self.node_net()}0/24: "
                     "connected to another access point, or the node's AP_IP was changed")
        else:
            self.add("OK", "address", f"{self.iface} has {', '.join(a for a in addrs if a.startswith(self.node_net()))}")

    def check_route(self) -> None:
        rc, out = self.runner(["ip", "-j", "route", "show", "default"])
        if rc is None:
            return
        gws: List[Tuple[str, str]] = []
        if rc == 0 and out.strip().startswith("["):
            try:
                for r in json.loads(out):
                    for hop in r.get("nexthops", [r]):               # multipath routes list their hops
                        gws.append((hop.get("gateway", "-"), hop.get("dev", "?")))
            except ValueError:
                gws = []
        else:
            if rc != 0:                                               # iproute2 without JSON support
                rc, out = self.runner(["ip", "route", "show", "default"])
            gws = re.findall(r"default via (\S+) dev (\S+)", out)
            gws += [("-", d) for d in re.findall(r"default dev (\S+)", out)]
        if not gws:
            self.add("INFO", "route", "no default route: fine for the viewer; the relay needs another uplink for off-site access")
        elif any(g == self.node for g, _ in gws):
            others = [f"{g} via {d}" for g, d in gws if g != self.node]
            self.add("INFO" if not others else "WARN", "route",
                     "the sensor node is the default route" + (" next to " + ", ".join(others) if others else "")
                     + ": no internet through it; keep your other uplink as default",
                     f"nmcli con modify '{self.ssid}' ipv4.never-default yes ipv4.ignore-auto-dns yes && nmcli con up '{self.ssid}'")
        else:
            self.add("OK", "route", "default route via " + ", ".join(f"{g} ({d})" for g, d in gws) + "; the node is reached directly")

    def check_powersave(self) -> None:
        rc, out = self.runner(["iw", "dev", self.iface, "get", "power_save"])
        if rc is None:
            self.add("INFO", "powersave", "iw not found: cannot check Wi-Fi power save (sudo apt install iw)")
            return
        if rc != 0:
            self.add("INFO", "powersave", f"could not query power save on {self.iface}: {out.strip()[:80]}")
        elif "on" in out.lower():
            self.add("WARN", "powersave", f"Wi-Fi power save is ON on {self.iface}: small UDP packets get delayed and dropped",
                     f"nmcli con modify '{self.ssid}' 802-11-wireless.powersave 2 && nmcli con up '{self.ssid}'")
        else:
            self.add("OK", "powersave", f"Wi-Fi power save is off on {self.iface}")

    def check_firewall(self) -> None:
        rc, out = self.runner(["ufw", "status"])
        if rc is None:
            self.add("OK", "firewall", "ufw not installed; nothing blocks UDP 4210 unless you configured nftables yourself")
            return
        if rc != 0:                                                   # ufw refuses to answer non-root users
            rc2, out2 = self.runner(["sudo", "-n", "ufw", "status"])
            if rc2 == 0:
                rc, out = rc2, out2
            else:
                self.add("INFO", "firewall", f"cannot query ufw as this user ({out.strip().splitlines()[0][:60] if out.strip() else 'no output'}); "
                         "if ufw is active it must allow UDP 4210", "sudo ufw status   # and if active: sudo ufw allow 4210/udp")
                return
        if "Status: active" not in out:
            self.add("OK", "firewall", "ufw is inactive")
            return
        if re.search(rf"{self.port}/udp\s+ALLOW", out):
            self.add("OK", "firewall", f"ufw active and UDP {self.port} allowed")
        else:
            self.add("WARN", "firewall", f"ufw is active and has no rule for UDP {self.port}: the node's datagrams are dropped",
                     f"sudo ufw allow {self.port}/udp")

    def check_port(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", self.port))
            self.add("OK", "udp", f"UDP {self.port} is free for the viewer")
        except OSError:
            rc, out = self.runner(["ss", "-ulpn", f"sport = :{self.port}"])
            who = ""
            if rc == 0:
                m = re.search(r'users:\(\("([^"]+)",pid=(\d+)', out)
                who = f" ({m.group(1)} pid {m.group(2)})" if m else ""
            self.add("INFO", "udp", f"UDP {self.port} is already in use{who}: a viewer or relay is running; a second viewer "
                     "needs the relay's port or --node-port")
        finally:
            s.close()

    def check_node(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.25)
        hello = rp.encode_hello(0, rp.RECEIVER_UCONSOLE, rp.WANT_STATUS)
        deadline = time.monotonic() + self.timeout
        last_hello = 0.0
        status: Optional[rp.StatusPacket] = None
        sender = ""
        try:
            while time.monotonic() < deadline:
                if time.monotonic() - last_hello >= 1.0:
                    try:
                        s.sendto(hello, (self.node, self.port))
                    except OSError as exc:
                        self.add("FAIL", "node", f"cannot send to {self.node}:{self.port}: {exc}",
                                 f"nmcli con up '{self.ssid}'")
                        return
                    last_hello = time.monotonic()
                try:
                    data, addr = s.recvfrom(1024)
                except socket.timeout:
                    continue
                dec = rp.decode(data)
                if dec and dec[0] == rp.TYPE_STATUS:
                    status, sender = dec[2], addr[0]
                    break
        finally:
            s.close()
        if status is None:
            rc, _ = self.runner(["ping", "-c", "1", "-W", "1", self.node])
            if rc == 0:
                self.add("FAIL", "node", f"{self.node} answers ping but sent no STATUS to our HELLO within {self.timeout:g} s: "
                         "it is not running RadarLink v2 firmware, or UDP is blocked", "python3 uconsole/doctor.py --node <ip> --port <port>")
            else:
                self.add("FAIL", "node", f"no answer from {self.node}:{self.port} and ping fails: not joined to the node's Wi-Fi, "
                         "node not powered, or another address (the node prints its IP on its serial port)",
                         f"nmcli con up '{self.ssid}'")
            return
        f = status.flags
        bits = []
        bits.append("radar " + ("OK" if f & rp.STATUS_RADAR_OK else "NO DATA") + (" configured" if f & rp.STATUS_RADAR_CONFIGURED else " not configured"))
        if f & rp.STATUS_LIDAR_ENABLED:
            bits.append("lidar " + ("OK" if f & rp.STATUS_LIDAR_OK else "NO DATA"))
        else:
            bits.append("lidar disabled in the node's build")
        level = "OK" if (f & rp.STATUS_RADAR_OK) else "WARN"
        self.add(level, "node", f"{sender} answered: up {status.uptime_ms / 1000:.0f} s, {'; '.join(bits)}; radar {status.radar_fps} fps, "
                 f"lidar {status.lidar_fps} fps / {status.lidar_pps} pts/s, {status.subscribers} subscriber(s) incl. this check, "
                 f"bad radar frames {status.radar_bad_frames}, lidar CRC errors {status.lidar_crc_errors}"
                 + ("" if level == "OK" else " -- the node has no radar frames: see its serial 'diag' output"))

    def check_services(self) -> None:
        for scope, unit in ((["--user"], "radar-viewer"), ([], "radar-relay")):
            rc, out = self.runner(["systemctl"] + scope + ["is-active", unit])
            if rc is None:
                break
            state = out.strip().splitlines()[0] if out.strip() else "unknown"
            if rc == 0:
                self.add("INFO", "service", f"{unit} service is {state}")
        desktop = os.path.join(self.home, ".config", "autostart", "radar-viewer.desktop")
        if os.path.exists(desktop):
            self.add("INFO", "service", "viewer autostart entry installed (~/.config/autostart/radar-viewer.desktop)")

    # ------------------------------------------------------------------ run
    def run(self, with_node: bool = True) -> List[Check]:
        self.check_python()
        self.check_session()
        self.check_wifi()
        self.check_address()
        self.check_route()
        self.check_powersave()
        self.check_firewall()
        self.check_port()
        if with_node:
            self.check_node()
        self.check_services()
        return self.checks

    def render(self) -> List[str]:
        out = [f"RadarLink doctor: node {self.node}:{self.port}, network '{self.ssid}', interface {self.iface}"]
        for c in self.checks:
            out.append(f"{c.level:<4} {c.name:<9} {c.text}")
            if c.fix and c.level in ("WARN", "FAIL"):
                out.append(f"     fix:      {c.fix}")
        out.append(f"result: {self.worst}")
        return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ssid", default="RadarSystem", help="the sensor node's Wi-Fi network (WIFI_SSID in config.h)")
    p.add_argument("--node", default="192.168.4.1", help="sensor node address")
    p.add_argument("--port", type=int, default=rp.PORT)
    p.add_argument("--iface", default="wlan0", help="Wi-Fi interface")
    p.add_argument("--timeout", type=float, default=3.0, help="seconds to wait for the node's STATUS")
    p.add_argument("--no-node", action="store_true", help="skip talking to the node")
    p.add_argument("--json", action="store_true")
    a = p.parse_args(argv)
    d = Doctor(a.ssid, a.node, a.port, a.iface, a.timeout)
    d.run(with_node=not a.no_node)
    if a.json:
        print(json.dumps({"result": d.worst, "checks": [c.__dict__ for c in d.checks]}, indent=1))
    else:
        print("\n".join(d.render()))
    return 1 if d.worst == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main())
