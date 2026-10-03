#!/usr/bin/env python3
"""Headless RadarLink hub: relay, alarm daemon and web view. No display needed.

Upstream it behaves like any receiver: it sends HELLO to the sensor node once a
second. Downstream it behaves like the sensor node: remote viewers send HELLO to
this machine and receive the same datagrams forwarded unchanged (sequence
numbers intact), so ``radar_viewer.py --node <this host>`` works from anywhere
this host is reachable, for example over Tailscale across the uConsole's 4G
link, while the sensor node itself stays on its private Wi-Fi.

It also runs the intrusion detector without a screen (same options as the
viewer: --learn, --armed, --arm-schedule, --ignore-zone, --alert-log,
--alert-cmd, --snapshot-dir) and serves a phone-friendly web view:

    GET  /            live plot (canvas), status, arm/disarm/learn buttons
    GET  /state.json  the same data as JSON for your own tooling
    POST /arm  /disarm  /learn?seconds=10   control (add --web-token to require ?token=)

Examples:
    python3 radar_relay.py --web 8080                       # relay + web view on this machine
    python3 radar_relay.py --armed --alert-cmd '...' --web 8080 --web-token s3cret
    # on a laptop anywhere on the tailnet:
    python3 radar_viewer.py --node 100.101.102.103 --windowed
"""
from __future__ import annotations

import argparse
import http.server
import json
import os
import queue
import select
import socket
import sys
import threading
import time
import urllib.parse
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radar_protocol as rp     # noqa: E402
import radar_state as rs        # noqa: E402
import radar_intrusion as ri    # noqa: E402

DEFAULT_NODE = "192.168.4.1"
LIDAR_TTL_S = 0.5
RADAR_HOLD_S = 0.4
RADAR_STILL_TIMEOUT_S = 2.0
LEARN_SECONDS = 10.0
FORWARD_TYPES = {rp.TYPE_RADAR: rp.WANT_RADAR, rp.TYPE_LIDAR: rp.WANT_LIDAR, rp.TYPE_STATUS: rp.WANT_STATUS}


class Relay:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.node = (args.node, args.node_port)
        self.up = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.up.bind(("", 0))
        self.up.setblocking(False)
        self.down = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.down.bind(("", args.serve_port))
        self.down.setblocking(False)
        self.serve_port = self.down.getsockname()[1]
        self.subs: Dict[Tuple[str, int], Tuple[float, int]] = {}
        self.hello_seq = 0
        self.last_hello = 0.0
        self.last_packet = 0.0
        self.seq = rs.SeqTracker()
        self.lidar = rs.LidarStore(LIDAR_TTL_S)
        self.radar = rs.RadarTracks(RADAR_HOLD_S, 0.0 if args.no_still_filter else RADAR_STILL_TIMEOUT_S)
        self.status: Optional[rp.StatusPacket] = None
        self.rx_total = self.rx_this_sec = self.rx_per_sec = self.fwd_total = self.foreign = 0
        self.last_sec = 0.0
        self.offset = args.offset
        self.detect = ri.DetectionRuntime(args, lidar_offset_deg=self.offset, log=self.log)
        self.commands: "queue.Queue[Tuple[str, dict]]" = queue.Queue()
        self.web_lock = threading.Lock()
        self.web_json = b"{}"
        self.web_port: Optional[int] = None
        self.httpd: Optional[http.server.ThreadingHTTPServer] = None
        if args.web is not None:
            self.start_web(args.web)
        self.log(f"[relay] upstream node {self.node[0]}:{self.node[1]} via UDP {self.up.getsockname()[1]}; "
                 f"serving remote viewers on UDP {self.serve_port}")

    @staticmethod
    def log(msg: str) -> None:
        print(msg, flush=True)

    # ------------------------------------------------------------ upstream
    def send_hello(self, now: float) -> None:
        if now - self.last_hello < rp.HELLO_INTERVAL_MS / 1000.0:
            return
        self.last_hello = now
        try:
            self.up.sendto(rp.encode_hello(self.hello_seq, rp.RECEIVER_OTHER, rp.WANT_ALL), self.node)
        except OSError as exc:
            self.log(f"[relay] hello failed: {exc}")
        self.hello_seq = (self.hello_seq + 1) & 0xFFFF

    def alive(self, now: float) -> bool:
        return bool(self.last_packet) and now - self.last_packet < rp.LINK_TIMEOUT_MS / 1000.0

    def handle_upstream(self, data: bytes, addr: Tuple[str, int], now: float) -> None:
        if addr[0] != self.node[0]:
            self.foreign += 1
            return
        decoded = rp.decode(data)
        if decoded is None or decoded[0] not in FORWARD_TYPES:
            self.foreign += 1
            return
        ptype, seq, pkt = decoded
        self.forward(data, FORWARD_TYPES[ptype])
        if not self.seq.accept(ptype, seq):
            return
        self.rx_total += 1
        self.rx_this_sec += 1
        self.last_packet = now
        if ptype == rp.TYPE_RADAR:
            self.radar.update(pkt, now)
        elif ptype == rp.TYPE_LIDAR:
            self.lidar.update(pkt, now)
            self.detect.on_lidar_packet(pkt)
        elif ptype == rp.TYPE_STATUS:
            self.status = pkt

    # ---------------------------------------------------------- downstream
    def handle_downstream(self, data: bytes, addr: Tuple[str, int], now: float) -> None:
        decoded = rp.decode(data)
        if decoded is None or decoded[0] != rp.TYPE_HELLO:
            return
        if addr not in self.subs:
            self.log(f"[relay] + remote viewer {addr[0]}:{addr[1]} wants=0x{decoded[2].wants:02x}")
        self.subs[addr] = (now, decoded[2].wants)

    def expire_subs(self, now: float) -> None:
        for addr, (ts, _) in list(self.subs.items()):
            if now - ts > rp.SUBSCRIBER_TIMEOUT_MS / 1000.0:
                self.log(f"[relay] - remote viewer {addr[0]}:{addr[1]} timed out")
                del self.subs[addr]

    def forward(self, data: bytes, want: int) -> None:
        for addr, (_, wants) in self.subs.items():
            if wants & want:
                try:
                    self.down.sendto(data, addr)
                    self.fwd_total += 1
                except OSError:
                    pass

    # ----------------------------------------------------------------- web
    def start_web(self, port: int) -> None:
        relay = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a) -> None:  # quiet
                pass

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:
                path = urllib.parse.urlparse(self.path).path
                if path == "/state.json":
                    with relay.web_lock:
                        body = relay.web_json
                    self._send(200, body, "application/json")
                elif path in ("/", "/index.html"):
                    self._send(200, WEB_PAGE.encode(), "text/html; charset=utf-8")
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self) -> None:
                url = urllib.parse.urlparse(self.path)
                q = urllib.parse.parse_qs(url.query)
                if relay.args.web_token and q.get("token", [None])[0] != relay.args.web_token:
                    self._send(403, b"bad token", "text/plain")
                    return
                if url.path in ("/arm", "/disarm", "/learn"):
                    try:
                        seconds = float(q.get("seconds", [LEARN_SECONDS])[0])
                    except ValueError:
                        seconds = LEARN_SECONDS
                    relay.commands.put((url.path[1:], {"seconds": max(1.0, min(120.0, seconds))}))
                    self._send(200, b"ok", "text/plain")
                else:
                    self._send(404, b"not found", "text/plain")

        self.httpd = http.server.ThreadingHTTPServer(("", port), Handler)
        self.httpd.daemon_threads = True
        self.web_port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.log(f"[relay] web view on http://0.0.0.0:{self.web_port}/")

    def build_web_state(self, now: float) -> None:
        det = self.detect.detector
        st = self.status
        state = {
            "t": round(now, 3),
            "alive": self.alive(now),
            "armed": self.detect.armed,
            "learning_left": self.detect.learning_left(now),
            "baseline_buckets": sum(self.detect.ref.reliable) if self.detect.ref.has_baseline() else 0,
            "status": None if st is None else {"uptime_s": st.uptime_ms // 1000, "radar_fps": st.radar_fps, "lidar_fps": st.lidar_fps,
                                                "lidar_pps": st.lidar_pps, "subscribers": st.subscribers, "flags": st.flags,
                                                "radar_bad_frames": st.radar_bad_frames, "lidar_crc_errors": st.lidar_crc_errors},
            "rx_per_sec": self.rx_per_sec,
            "remote_viewers": len(self.subs),
            "lidar": [[round((ri.bucket_angle_deg(i) + self.offset) % 360.0, 1), d, round(age, 2)] for i, d, _, age in self.lidar.visible(now)],
            "radar": [{"slot": slot, "x_mm": t.x_mm, "y_mm": t.y_mm, "speed_cm_s": t.speed_cm_s} for slot, t in self.radar.shown(now)],
            "alerts": [{"id": a.id, "kind": a.kind, "angle_deg": round(a.angle_deg, 1), "dist_mm": round(a.dist_mm),
                        "x_mm": round(a.x_mm), "y_mm": round(a.y_mm), "confidence": a.confidence, "since_s": round(now - a.since, 1)}
                       for a in det.active()] if self.detect.armed else [],
            "zones": [[z.angle_from_deg, z.angle_to_deg, z.dist_min_mm, min(z.dist_max_mm, 99999)] for z in det.zones],
        }
        body = json.dumps(state, separators=(",", ":")).encode()
        with self.web_lock:
            self.web_json = body

    def run_commands(self, now: float) -> None:
        while True:
            try:
                cmd, params = self.commands.get_nowait()
            except queue.Empty:
                return
            if cmd == "arm":
                self.detect.set_armed(True)
            elif cmd == "disarm":
                self.detect.set_armed(False)
            elif cmd == "learn":
                self.detect.start_learning(now, params.get("seconds", LEARN_SECONDS), arm_after=self.detect.armed)

    # ---------------------------------------------------------------- main
    def run(self) -> int:
        start = time.monotonic()
        last_log = last_web = start
        while True:
            now = time.monotonic()
            self.send_hello(now)
            ready, _, _ = select.select([self.up, self.down], [], [], 0.01)
            for sock in ready:
                for _ in range(64):
                    try:
                        data, addr = sock.recvfrom(2048)
                    except BlockingIOError:
                        break
                    except OSError as exc:
                        self.log(f"[relay] recv error: {exc}")
                        break
                    if sock is self.up:
                        self.handle_upstream(data, addr, now)
                    else:
                        self.handle_downstream(data, addr, now)
            self.expire_subs(now)
            self.run_commands(now)
            self.detect.tick(now, self.alive(now), self.lidar, self.radar)
            if now - self.last_sec >= 1.0:
                self.last_sec = now
                self.rx_per_sec, self.rx_this_sec = self.rx_this_sec, 0
            if self.httpd is not None and now - last_web >= 0.1:
                last_web = now
                self.build_web_state(now)
            if now - last_log >= 2.0:
                last_log = now
                self.log(f"[stat] data={'ok' if self.alive(now) else 'none'} rx/s={self.rx_per_sec} fwd={self.fwd_total} "
                         f"viewers={len(self.subs)} armed={self.detect.armed} alerts={len(self.detect.detector.alerts)}")
            if self.args.max_seconds and now - start >= self.args.max_seconds:
                break
        self.detect.close()
        if self.httpd is not None:
            self.httpd.shutdown()
        return 0


WEB_PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RadarLink</title>
<style>
 body{margin:0;background:#060a06;color:#cfe;font:14px/1.3 monospace}
 #top{display:flex;gap:8px;align-items:center;padding:6px 8px;flex-wrap:wrap}
 button{background:#123;color:#cfe;border:1px solid #4a8;padding:6px 10px;font:inherit}
 canvas{display:block;width:100vw;height:calc(100vh - 110px);touch-action:none}
 #hud{padding:4px 8px;white-space:pre-wrap;min-height:48px}
 .alert{color:#f55} .ok{color:#5f5} .warn{color:#fc3}
</style></head><body>
<div id="top"><b>RadarLink</b>
 <button onclick="post('arm')">Arm</button><button onclick="post('disarm')">Disarm</button>
 <button onclick="post('learn')">Learn 10 s</button>
 <button onclick="zoom(-1)">+</button><button onclick="zoom(1)">&minus;</button><span id="range"></span></div>
<canvas id="c"></canvas><div id="hud">connecting…</div>
<script>
const ranges=[2,4,8,12];let zi=2;const token=new URLSearchParams(location.search).get('token');
function zoom(d){zi=Math.max(0,Math.min(ranges.length-1,zi+d));}
function post(p){fetch('/'+p+(token?'?token='+encodeURIComponent(token):''),{method:'POST'});}
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
function draw(s){
 const W=cv.clientWidth,H=cv.clientHeight;cv.width=W;cv.height=H;
 const cx=W/2,cy=H/2,R=Math.min(W,H)/2-10,range=ranges[zi],k=R/(range*1000);
 document.getElementById('range').textContent=range+' m';
 ctx.fillStyle='#060a06';ctx.fillRect(0,0,W,H);
 for(const z of s.zones||[]){const a1=z[0]*Math.PI/180,span=((((z[1]-z[0])%360)+360)%360)*Math.PI/180||2*Math.PI;
  const ri_=Math.min(R,z[2]*k),ro=Math.min(R,z[3]*k);ctx.fillStyle='#2a1e1e';ctx.beginPath();
  ctx.arc(cx,cy,ro,a1-Math.PI/2,a1-Math.PI/2+span);ctx.arc(cx,cy,ri_,a1-Math.PI/2+span,a1-Math.PI/2,true);ctx.fill();}
 ctx.strokeStyle='#0a3a0a';for(let m=1;m<=range;m+=(range>8?2:1)){ctx.beginPath();ctx.arc(cx,cy,m*1000*k,0,7);ctx.stroke();}
 ctx.strokeStyle='#0d4d1d';ctx.beginPath();const rr=Math.min(R,8000*k);ctx.moveTo(cx,cy);
 ctx.arc(cx,cy,rr,-Math.PI/2-Math.PI/3,-Math.PI/2+Math.PI/3);ctx.closePath();ctx.stroke();
 for(const p of s.lidar){const r=p[1]*k;if(r>R)continue;const a=p[0]*Math.PI/180;const g=Math.round(255-150*p[2]);
  ctx.fillStyle='rgb('+Math.round(60+60*(1-p[2]))+','+g+','+Math.round(80*(1-p[2]))+')';ctx.fillRect(cx+r*Math.sin(a)-1,cy-r*Math.cos(a)-1,3,3);}
 const cols=['#f44','#fb0','#0ef'];
 for(const t of s.radar){const x=cx+t.x_mm*k,y=cy-t.y_mm*k;ctx.strokeStyle=ctx.fillStyle=cols[t.slot]||'#fff';
  ctx.beginPath();ctx.arc(x,y,7,0,7);ctx.fill();ctx.beginPath();ctx.arc(x,y,15,0,7);ctx.stroke();
  ctx.fillStyle='#eee';ctx.fillText((Math.hypot(t.x_mm,t.y_mm)/1000).toFixed(2)+'m '+Math.abs(t.speed_cm_s)+'cm/s',x+20,y+4);}
 for(const a of s.alerts){const x=cx+a.x_mm*k,y=cy-a.y_mm*k;ctx.strokeStyle='#f33';ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,26,0,7);ctx.stroke();ctx.lineWidth=1;
  ctx.fillStyle='#f33';ctx.fillText('ALERT '+a.kind+' '+(a.dist_mm/1000).toFixed(1)+'m',x-40,y-32);}
 ctx.fillStyle='#5f5';ctx.beginPath();ctx.arc(cx,cy,4,0,7);ctx.fill();
 let hud=(s.alive?'<span class=ok>DATA OK</span>':'<span class=alert>NO DATA</span>')+'  rx '+s.rx_per_sec+'/s  remote viewers '+s.remote_viewers;
 if(s.status)hud+='\nnode up '+s.status.uptime_s+'s  radar '+s.status.radar_fps+' fps  lidar '+s.status.lidar_fps+' fps '+s.status.lidar_pps+' pps  subs '+s.status.subscribers;
 if(s.learning_left!=null)hud+='\n<span class=warn>LEARNING '+s.learning_left.toFixed(1)+' s - keep the room empty</span>';
 else hud+='\n'+(s.armed?'<span class=alert>ARMED</span>':'disarmed')+'  baseline '+s.baseline_buckets+' buckets';
 for(const a of s.alerts)hud+='\n<span class=alert>ALERT #'+a.id+' '+a.kind+' '+(a.dist_mm/1000).toFixed(1)+'m @ '+a.angle_deg+'deg conf '+a.confidence+' ('+a.since_s+'s)</span>';
 document.getElementById('hud').innerHTML=hud;}
async function tick(){try{const r=await fetch('/state.json',{cache:'no-store'});draw(await r.json());}catch(e){document.getElementById('hud').textContent='relay unreachable';}}
setInterval(tick,250);tick();
</script></body></html>
"""


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--node", default=DEFAULT_NODE, help=f"sensor node IP (default {DEFAULT_NODE})")
    p.add_argument("--node-port", type=int, default=rp.PORT, help="sensor node UDP port")
    p.add_argument("--serve-port", type=int, default=rp.PORT, help="UDP port remote viewers send HELLO to (0 = ephemeral)")
    p.add_argument("--web", type=int, metavar="PORT", help="serve the web view on this TCP port (0 = ephemeral)")
    p.add_argument("--web-token", metavar="TOKEN", help="require ?token=TOKEN on the arm/disarm/learn endpoints")
    p.add_argument("--offset", type=float, default=0.0, help="LIDAR mounting angle offset in degrees (same as the viewer)")
    p.add_argument("--no-still-filter", action="store_true", help="keep radar targets that stopped moving")
    p.add_argument("--max-seconds", type=float, default=0.0, help="exit after this long (testing)")
    ri.DetectionRuntime.add_arguments(p)
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    try:
        return Relay(args).run()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
