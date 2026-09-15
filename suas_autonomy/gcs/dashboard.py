"""Dependency-free local GCS dashboard for simulation and CSV replay.

The server binds to loopback by default.  It is a development display, not a
replacement for Mission Planner or a flight-certified safety interface.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


@dataclass(frozen=True)
class AircraftState:
    latitude: float
    longitude: float
    altitude_msl_ft: float
    altitude_agl_ft: float
    groundspeed_kt: float
    heading_deg: float
    mission_phase: str
    completed_laps: int
    camera_ok: bool
    telemetry_age_s: float


class SyntheticStateSource:
    def __init__(self, origin_lat: float = -35.363262, origin_lon: float = 149.165237):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.started = time.monotonic()

    def latest(self) -> AircraftState:
        elapsed = time.monotonic() - self.started
        angle = elapsed / 18.0
        north_m = 280 * math.sin(angle)
        east_m = 180 * math.sin(angle) * math.cos(angle)
        lat = self.origin_lat + north_m / 111_320.0
        lon = self.origin_lon + east_m / (111_320.0 * math.cos(math.radians(self.origin_lat)))
        return AircraftState(lat, lon, 2150.0, 200.0, 35.0, math.degrees(angle) % 360, "waypoint_lap", int(elapsed // 90), True, 0.0)


class CsvReplaySource:
    """Replay the CSV produced by autonomous_mission_sitl.monitor."""

    def __init__(self, path: Path, rate_hz: float = 2.0):
        with path.open(newline="", encoding="utf-8") as stream:
            self.rows = list(csv.DictReader(stream))
        if not self.rows:
            raise ValueError(f"replay CSV contains no rows: {path}")
        self.period = 1.0 / rate_hz
        self.started = time.monotonic()

    def latest(self) -> AircraftState:
        index = min(int((time.monotonic() - self.started) / self.period), len(self.rows) - 1)
        row = self.rows[index]
        groundspeed_m_s = float(row.get("groundspeed_m_s") or 0)
        relative_alt_m = float(row.get("relative_alt_m") or 0)
        return AircraftState(
            latitude=float(row.get("lat") or 0),
            longitude=float(row.get("lon") or 0),
            altitude_msl_ft=relative_alt_m * 3.28084,
            altitude_agl_ft=relative_alt_m * 3.28084,
            groundspeed_kt=groundspeed_m_s * 1.94384,
            heading_deg=float(row.get("heading_deg") or 0),
            mission_phase=row.get("event") or row.get("mode") or "replay",
            completed_laps=0,
            camera_ok=True,
            telemetry_age_s=0.0,
        )


class StateStore:
    def __init__(self, source, boundary: list[list[float]] | None = None):
        self.source = source
        self.boundary = boundary or []
        self._lock = threading.Lock()
        self._trail: list[list[float]] = []

    def snapshot(self) -> dict[str, object]:
        state = self.source.latest()
        with self._lock:
            point = [state.latitude, state.longitude]
            if not self._trail or self._trail[-1] != point:
                self._trail.append(point)
                self._trail = self._trail[-500:]
            trail = list(self._trail)
        return {"aircraft": asdict(state), "boundary": self.boundary, "trail": trail}


HTML = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>SUAS Development GCS</title>
<style>body{margin:0;background:#10151c;color:#e8eef5;font:15px system-ui;display:grid;grid-template-columns:1fr 320px;height:100vh}canvas{width:100%;height:100%;background:#17212b}.panel{padding:22px}.value{font-size:26px;margin-bottom:16px}.ok{color:#71e6a2}.warn{color:#ffb86b}small{color:#91a4b7}</style></head>
<body><canvas id="map"></canvas><div class="panel"><h1>SUAS Development GCS</h1><small>Simulation/replay only</small><div id="data"></div></div>
<script>
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d');let state;
function resize(){canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=canvas.clientHeight*devicePixelRatio}addEventListener('resize',resize);resize();
function project(p,points){let xs=points.map(x=>x[1]),ys=points.map(x=>x[0]),minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys),pad=60;return[pad+(p[1]-minx)/(maxx-minx||1)*(canvas.width-2*pad),canvas.height-pad-(p[0]-miny)/(maxy-miny||1)*(canvas.height-2*pad)]}
function line(points,color,width){if(points.length<2)return;ctx.beginPath();points.forEach((p,i)=>{let q=project(p,[...state.boundary,...state.trail]);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke()}
function draw(){if(!state)return;ctx.clearRect(0,0,canvas.width,canvas.height);line([...state.boundary,state.boundary[0]],'#f4c95d',3);line(state.trail,'#55c2ff',3);let a=state.aircraft,q=project([a.latitude,a.longitude],[...state.boundary,...state.trail]);ctx.beginPath();ctx.arc(...q,10,0,Math.PI*2);ctx.fillStyle='#ff5c77';ctx.fill()}
async function update(){state=await(await fetch('/api/state')).json();let a=state.aircraft;document.getElementById('data').innerHTML=`<h2>${a.mission_phase}</h2><div class=value>${a.groundspeed_kt.toFixed(1)} kt</div><div class=value>${a.altitude_msl_ft.toFixed(0)} ft MSL</div><div>${a.latitude.toFixed(7)}, ${a.longitude.toFixed(7)}</div><p>Heading: ${a.heading_deg.toFixed(0)}°</p><p>Laps: ${a.completed_laps}</p><p class=${a.camera_ok?'ok':'warn'}>Camera: ${a.camera_ok?'OK':'FAULT'}</p>`;draw()}setInterval(update,500);update();
</script></body></html>"""


def make_handler(store: StateStore):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state":
                payload = json.dumps(store.snapshot()).encode()
                content_type = "application/json"
            elif path == "/":
                payload = HTML.encode()
                content_type = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format, *args):
            return

    return Handler


def load_boundary(path: Path | None) -> list[list[float]]:
    if path is None:
        return []
    document = json.loads(path.read_text(encoding="utf-8"))
    return document.get("flight_boundary", document.get("boundary", []))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--rules", type=Path, default=Path(__file__).parents[1] / "config" / "suas_2026.json")
    args = parser.parse_args()
    source = CsvReplaySource(args.replay) if args.replay else SyntheticStateSource()
    store = StateStore(source, load_boundary(args.rules))
    server = ThreadingHTTPServer((args.host, args.port), make_handler(store))
    print(f"Development GCS: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
