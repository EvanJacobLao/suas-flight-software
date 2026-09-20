import csv
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen, Request
from http.server import ThreadingHTTPServer
from pymavlink.dialects.v20 import ardupilotmega as mav

from suas_autonomy.gcs.flight_monitor import Telemetry, FlightLog, make_handler, record, receive
from unittest.mock import patch


def message(kind, *args, system=1, component=1):
    msg = getattr(mav, 'MAVLink_' + kind + '_message')(*args)
    msg._header.srcSystem = system
    msg._header.srcComponent = component
    return msg


class MonitorTests(unittest.TestCase):
    def test_heartbeat_filter_and_staleness(self):
        state = Telemetry()
        hb = message('heartbeat', 1, 3, 129, 5, 4, 3)
        state.ingest(hb, now=10)
        snapshot = state.snapshot(now=11)
        self.assertTrue(snapshot['connected'])
        self.assertEqual(snapshot['values']['mode'], 'FBWA')
        self.assertTrue(snapshot['values']['armed'])
        state.ingest(message('heartbeat', 1, 3, 0, 0, 4, 3, system=2), now=13)
        state.ingest(message('heartbeat', 6, 8, 0, 0, 4, 3), now=13)
        self.assertFalse(state.snapshot(now=14)['connected'])
        self.assertTrue(state.snapshot(now=14)['values']['armed'])

    def test_units_and_unknown_battery(self):
        state = Telemetry()
        state.ingest(message('global_position_int', 100, -353632620, 1491652370, 630000, 45000, 0, 0, 0, 0), now=10)
        self.assertEqual(state.snapshot(now=10)['values']['altitude_relative_m'], 45)
        self.assertAlmostEqual(state.snapshot(now=10)['values']['latitude'], -35.363262)
        state.ingest(message('sys_status', 0, 0, 0, 0, 65535, -1, -1, 0, 0, 0, 0, 0, 0), now=10)
        self.assertIsNone(state.snapshot(now=10)['values']['battery_v'])
        state.ingest(message('sys_status', 0, 0, 0, 0, 11800, 950, 70, 0, 0, 0, 0, 0, 0), now=11)
        self.assertEqual(state.snapshot(now=11)['values']['battery_a'], 9.5)
        state.update({'roll_deg': float('nan')}, now=11)
        json.dumps(state.snapshot(now=11), allow_nan=False)

    def test_logging_without_browser_and_stale_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Telemetry(demo=True)
            log = FlightLog(tmp, True)
            stop = threading.Event()
            worker = threading.Thread(target=record, args=(state, stop, log))
            worker.start()
            time.sleep(.65)
            stop.set()
            worker.join(2)
            log.write(state.snapshot(now=time.monotonic() + 10))
            log.close()
            records = [json.loads(line) for line in (log.path / 'telemetry.jsonl').read_text().splitlines()]
            self.assertGreaterEqual(len(records), 3)
            self.assertEqual(records[0]['source'], 'DEMO')
            with (log.path / 'telemetry.csv').open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[-1]['mode'], '')
            self.assertEqual(rows[-1]['connected'], 'False')

    def test_http_has_no_command_endpoint(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(Telemetry()))
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(base + '/api/state') as response:
                self.assertFalse(json.load(response)['connected'])
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
            with urlopen(base) as response:
                self.assertIn(b'LAPTOP LINK LOST', response.read())
            with self.assertRaises(HTTPError) as caught:
                urlopen(Request(base + '/arm', data=b'{}', method='POST'))
            self.assertEqual(caught.exception.code, 501)
        finally:
            server.shutdown()
            worker.join()
            server.server_close()

    def test_receiver_never_writes_to_link(self):
        stop = threading.Event()
        class ReceiveOnlyLink:
            def recv_match(self, **kwargs):
                stop.set()
                return message('heartbeat', 1, 3, 128, 5, 4, 3)
            def close(self):
                pass
        with patch('pymavlink.mavutil.mavlink_connection', return_value=ReceiveOnlyLink()):
            state = Telemetry()
            receive(state, stop, 'fake', 57600)
            self.assertTrue(state.snapshot()['connected'])

    def test_logging_failure_is_visible(self):
        class BrokenLog:
            def write(self, snapshot):
                raise OSError('disk full')
        state = Telemetry(demo=True)
        record(state, threading.Event(), BrokenLog())
        self.assertIn('disk full', state.snapshot()['logging_error'])


if __name__ == '__main__':
    unittest.main()
