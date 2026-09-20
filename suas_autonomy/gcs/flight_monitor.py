"""Receive-only flight telemetry, onboard logging, and a wireless browser display."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class Telemetry:
    def __init__(self, demo=False, system=1, component=1):
        self.demo, self.system, self.component = demo, system, component
        self.lock = threading.Lock()
        self.values, self.updated = {}, {}
        self.trail, self.messages = [], []
        self.error = None
        self.logging_error = None

    def ingest(self, msg, now=None):
        if msg.get_srcSystem() != self.system or msg.get_srcComponent() != self.component:
            return
        kind = msg.get_type()
        values = {}
        if kind == 'HEARTBEAT':
            # Accept only ArduPilot fixed-wing autopilot heartbeats.
            if msg.autopilot != 3 or msg.type != 1:
                return
            from pymavlink import mavutil
            values = dict(mode=mavutil.mode_string_v10(msg), armed=bool(msg.base_mode & 128))
        elif kind == 'ATTITUDE':
            values = dict(roll_deg=math.degrees(msg.roll), pitch_deg=math.degrees(msg.pitch))
        elif kind == 'GLOBAL_POSITION_INT':
            values = dict(latitude=msg.lat / 1e7, longitude=msg.lon / 1e7,
                          altitude_msl_m=msg.alt / 1000, altitude_relative_m=msg.relative_alt / 1000)
        elif kind == 'VFR_HUD':
            values = dict(airspeed_m_s=msg.airspeed, groundspeed_m_s=msg.groundspeed,
                          heading_deg=msg.heading, throttle_pct=msg.throttle, climb_m_s=msg.climb)
        elif kind == 'SYS_STATUS':
            values = dict(battery_v=None if msg.voltage_battery == 65535 else msg.voltage_battery / 1000,
                          battery_a=None if msg.current_battery == -1 else msg.current_battery / 100,
                          battery_pct=None if msg.battery_remaining == -1 else msg.battery_remaining)
        elif kind == 'GPS_RAW_INT':
            values = dict(gps_fix=msg.fix_type, satellites=None if msg.satellites_visible == 255 else msg.satellites_visible)
        elif kind == 'RC_CHANNELS':
            values = dict(rc_rssi_raw=None if msg.rssi == 255 else msg.rssi)
        elif kind == 'STATUSTEXT':
            text = msg.text.decode(errors='replace') if isinstance(msg.text, bytes) else msg.text
            with self.lock:
                self.messages.append(dict(utc=utc_now(), severity=msg.severity, text=text))
                self.messages = self.messages[-20:]
        if values:
            self.update(values, now)

    def update(self, values, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            for key, value in values.items():
                self.values[key] = value if not isinstance(value, float) or math.isfinite(value) else None
                self.updated[key] = now
            if 'latitude' in values and self.values.get('gps_fix', 0) >= 3:
                point = [self.values['latitude'], self.values['longitude']]
                if all(v is not None for v in point) and (not self.trail or point != self.trail[-1]):
                    self.trail.append(point)
                    self.trail = self.trail[-1000:]

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            ages = {key: max(0, now - stamp) for key, stamp in self.updated.items()}
            connected = ages.get('mode', float('inf')) <= 3
            return dict(utc=utc_now(), source='DEMO' if self.demo else 'AIRCRAFT',
                        connected=connected, values=dict(self.values), ages=ages,
                        trail=list(self.trail), messages=list(self.messages),
                        connection_error=self.error, logging_error=self.logging_error)


class FlightLog:
    def __init__(self, root, demo):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = Path(root) / (('demo_' if demo else 'flight_') + stamp)
        self.path.mkdir(parents=True)
        self.json_file = (self.path / 'telemetry.jsonl').open('x', encoding='utf-8')
        self.csv_file = (self.path / 'telemetry.csv').open('x', newline='', encoding='utf-8')
        self.fields = ['utc', 'source', 'connected', 'mode', 'armed', 'latitude', 'longitude',
                       'altitude_relative_m', 'altitude_msl_m', 'airspeed_m_s', 'groundspeed_m_s',
                       'roll_deg', 'pitch_deg', 'heading_deg', 'battery_v', 'battery_a',
                       'battery_pct', 'gps_fix', 'satellites']
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fields)
        self.writer.writeheader()

    def write(self, snapshot):
        record = {k: v for k, v in snapshot.items() if k != 'trail'}
        self.json_file.write(json.dumps(record, allow_nan=False) + '\n')
        row = {k: snapshot.get(k) for k in self.fields[:3]}
        # CSV blanks stale values; JSONL retains last values with explicit ages.
        row.update({k: snapshot['values'].get(k) if snapshot['ages'].get(k, 999) <= 3 else None
                    for k in self.fields[3:]})
        self.writer.writerow(row)
        self.json_file.flush()
        self.csv_file.flush()

    def close(self):
        self.json_file.close()
        self.csv_file.close()


def demo_tick(state, elapsed):
    angle = elapsed / 15
    state.update(dict(mode='FBWA', armed=True, gps_fix=3, satellites=14,
                      latitude=-35.363262 + .001 * math.sin(angle),
                      longitude=149.165237 + .001 * math.cos(angle),
                      altitude_relative_m=45 + 3 * math.sin(angle), altitude_msl_m=630,
                      airspeed_m_s=14, groundspeed_m_s=15, heading_deg=math.degrees(angle) % 360,
                      roll_deg=12 * math.sin(angle), pitch_deg=3, throttle_pct=45,
                      climb_m_s=.2 * math.cos(angle), battery_v=11.8, battery_a=9,
                      battery_pct=80, rc_rssi_raw=190))


def receive(state, stop, connection, baud):
    from pymavlink import mavutil
    while not stop.is_set():
        link = None
        try:
            link = mavutil.mavlink_connection(connection, baud=baud)
            with state.lock:
                state.error = None
            while not stop.is_set():
                msg = link.recv_match(blocking=True, timeout=.5)
                if msg is not None:
                    state.ingest(msg)
        except Exception as exc:
            with state.lock:
                state.error = str(exc)
            stop.wait(2)
        finally:
            if link is not None:
                link.close()


def record(state, stop, log):
    started = time.monotonic()
    while not stop.is_set():
        if state.demo:
            demo_tick(state, time.monotonic() - started)
        try:
            log.write(state.snapshot())
        except (OSError, ValueError) as exc:
            with state.lock:
                state.logging_error = str(exc)
            print(f'LOGGING FAILED: {exc}', flush=True)
            return
        stop.wait(.5)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/api/state':
                payload = json.dumps(state.snapshot(), allow_nan=False).encode()
                content_type = 'application/json'
            elif self.path == '/':
                payload = Path(__file__).with_name('flight_monitor.html').read_bytes()
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--demo', action='store_true')
    source.add_argument('--connect', help='e.g. /dev/serial/by-id/... or udpin:0.0.0.0:14550')
    parser.add_argument('--baud', type=int, default=57600)
    parser.add_argument('--system', type=int, default=1)
    parser.add_argument('--component', type=int, default=1)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--logs', type=Path, default=Path('logs/manual_flight'))
    args = parser.parse_args()
    if not args.demo:
        try:
            import pymavlink  # noqa: F401
        except ImportError:
            parser.error('Live telemetry requires: python3 -m pip install pymavlink')
    state = Telemetry(args.demo, args.system, args.component)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    log = FlightLog(args.logs, args.demo)
    stop = threading.Event()
    workers = [threading.Thread(target=record, args=(state, stop, log), daemon=True)]
    if not args.demo:
        workers.append(threading.Thread(target=receive, args=(state, stop, args.connect, args.baud), daemon=True))
    for worker in workers:
        worker.start()
    print(f"{'DEMO' if args.demo else 'RECEIVE-ONLY'} dashboard: http://{args.host}:{args.port}", flush=True)
    print(f'Onboard logs: {log.path.resolve()}', flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=3)
        log.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
