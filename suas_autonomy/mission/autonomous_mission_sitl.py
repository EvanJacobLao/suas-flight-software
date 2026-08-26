"""Upload and monitor a safety-gated autonomous ArduPlane SITL mission.

This is a development tool, not flight-qualified aircraft software. It refuses
non-loopback MAVLink endpoints so it cannot accidentally command real hardware.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
import time

from pymavlink import mavutil

try:
    from .model import FT_TO_M, GeoPoint, MissionPoint, offset_point, validate_mission
except ImportError:
    from model import FT_TO_M, GeoPoint, MissionPoint, offset_point, validate_mission


DEFAULT_CONNECTION = "tcp:127.0.0.1:5762"
LOOPBACK_MARKERS = ("127.0.0.1", "localhost")

COMMANDS = {
    "TAKEOFF": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    "WAYPOINT": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    "LAND": mavutil.mavlink.MAV_CMD_NAV_LAND,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default=DEFAULT_CONNECTION)
    parser.add_argument(
        "--mission",
        type=Path,
        default=Path(__file__).parents[1] / "config" / "sample_sitl_mission.json",
    )
    parser.add_argument(
        "--rules",
        type=Path,
        default=Path(__file__).parents[1] / "config" / "suas_2026.json",
    )
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--heartbeat-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true", help="validate without connecting")
    parser.add_argument(
        "--start",
        action="store_true",
        help="required acknowledgement before arming and entering AUTO",
    )
    return parser.parse_args()


def load_inputs(mission_path: Path, rules_path: Path):
    mission_data = json.loads(mission_path.read_text(encoding="utf-8"))
    rules = json.loads(rules_path.read_text(encoding="utf-8"))
    points = [MissionPoint(**item) for item in mission_data["points"]]
    errors = validate_mission(
        points,
        min_cruise_altitude_m=rules["minimum_cruise_altitude_ft"] * FT_TO_M,
        max_altitude_m=rules["maximum_altitude_ft"] * FT_TO_M,
        runway_exception_radius_m=rules["runway_exception_radius_ft"] * FT_TO_M,
    )
    waypoint_count = sum(point.command == "WAYPOINT" for point in points)
    if waypoint_count > rules["maximum_waypoints_per_lap"]:
        errors.append(
            f"mission contains {waypoint_count} waypoints; profile maximum is "
            f"{rules['maximum_waypoints_per_lap']}"
        )
    return mission_data, rules, points, errors


def require_sitl_endpoint(connection: str) -> None:
    if not connection.startswith("tcp:") or not any(marker in connection for marker in LOOPBACK_MARKERS):
        raise SystemExit(
            "Refusing connection: this milestone is SITL-only and accepts only a loopback TCP endpoint."
        )


def wait_for_message(mav, message_type: str, timeout: float):
    message = mav.recv_match(type=message_type, blocking=True, timeout=timeout)
    if message is None:
        raise TimeoutError(f"timed out waiting for {message_type}")
    return message


def command_and_require_ack(mav, command: int, *params: float, timeout: float = 10.0) -> None:
    padded = list(params) + [0.0] * (7 - len(params))
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        command,
        0,
        *padded[:7],
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack and ack.command == command:
            if ack.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                raise RuntimeError(f"command {command} rejected with MAV_RESULT {ack.result}")
            return
    raise TimeoutError(f"no COMMAND_ACK for command {command}")


def current_home(mav, timeout: float) -> GeoPoint:
    command_and_require_ack(
        mav,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
    )
    home = wait_for_message(mav, "HOME_POSITION", timeout)
    return GeoPoint(home.latitude / 1e7, home.longitude / 1e7)


def upload_mission(mav, home: GeoPoint, points: list[MissionPoint]) -> None:
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    clear_ack = mav.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if clear_ack is None or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError("autopilot did not accept mission clear")
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(points))
    sent: set[int] = set()
    deadline = time.monotonic() + 30

    while len(sent) < len(points) and time.monotonic() < deadline:
        request = mav.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True, timeout=2)
        if request is None:
            continue
        sequence = request.seq
        if not 0 <= sequence < len(points):
            raise RuntimeError(f"autopilot requested invalid mission sequence {sequence}")
        point = points[sequence]
        location = offset_point(home, point.north_m, point.east_m)
        # ArduPlane recommends 10-15 degrees for NAV_TAKEOFF. A zero pitch can
        # produce an ineffective runway takeoff even though AUTO is active.
        command_param1 = 12.0 if point.command == "TAKEOFF" else 0.0
        mav.mav.mission_item_int_send(
            mav.target_system,
            mav.target_component,
            sequence,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            COMMANDS[point.command],
            0,
            1,
            command_param1, 0, 0, 0,
            int(location.lat * 1e7),
            int(location.lon * 1e7),
            point.altitude_m,
        )
        sent.add(sequence)

    ack = mav.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if len(sent) != len(points) or ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError("mission upload was not accepted")


def set_and_verify_wp_radius(mav, radius_m: float) -> None:
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        b"WP_RADIUS",
        radius_m,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        value = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if value and value.param_id.rstrip("\x00") == "WP_RADIUS":
            if value.param_value > radius_m + 0.5:
                raise RuntimeError("autopilot WP_RADIUS is wider than the handbook threshold")
            return
    raise TimeoutError("autopilot did not confirm WP_RADIUS")


def set_mode(mav, name: str) -> None:
    mapping = mav.mode_mapping()
    if name not in mapping:
        raise RuntimeError(f"flight mode {name} is unavailable")
    mav.set_mode(mapping[name])
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat and mavutil.mode_string_v10(heartbeat) == name:
            return
    raise TimeoutError(f"autopilot did not enter {name}")


def monitor(mav, log_path: Path, stop_requested) -> str:
    fields = [
        "utc", "mission_seq", "lat", "lon", "relative_alt_m", "groundspeed_m_s",
        "mode", "armed", "event",
    ]
    state = {field: "" for field in fields}
    last_write = 0.0
    last_heartbeat = time.monotonic()
    completed = False

    with log_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        while not stop_requested():
            message = mav.recv_match(blocking=True, timeout=0.5)
            now = time.monotonic()
            state["event"] = ""
            if message:
                kind = message.get_type()
                if kind == "HEARTBEAT":
                    # Ignore heartbeats from a GCS or companion component on
                    # the shared MAVLink stream; their custom_mode is not the
                    # aircraft flight mode.
                    if (
                        message.get_srcSystem() != mav.target_system
                        or message.get_srcComponent() != mav.target_component
                    ):
                        continue
                    last_heartbeat = now
                    state["mode"] = mavutil.mode_string_v10(message)
                    state["armed"] = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif kind == "GLOBAL_POSITION_INT":
                    state["lat"] = message.lat / 1e7
                    state["lon"] = message.lon / 1e7
                    state["relative_alt_m"] = message.relative_alt / 1000
                elif kind == "VFR_HUD":
                    state["groundspeed_m_s"] = message.groundspeed
                elif kind == "MISSION_CURRENT":
                    state["mission_seq"] = message.seq
                    if getattr(message, "mission_state", None) == mavutil.mavlink.MISSION_STATE_COMPLETE:
                        completed = True
                        state["event"] = "mission_complete"
                elif kind == "STATUSTEXT":
                    state["event"] = message.text

            if now - last_heartbeat > 5:
                print("Companion link heartbeat lost; ArduPilot remains responsible for its configured failsafe.")
                return "heartbeat_timeout"

            if now - last_write >= 1:
                state["utc"] = datetime.now(timezone.utc).isoformat()
                writer.writerow(state)
                stream.flush()
                print(
                    f"seq={state['mission_seq']} mode={state['mode']} "
                    f"alt={state['relative_alt_m']}m speed={state['groundspeed_m_s']}m/s"
                )
                last_write = now
            if completed:
                return "complete"
    return "operator_stop"


def main() -> int:
    args = parse_args()
    mission_data, rules, points, errors = load_inputs(args.mission, args.rules)
    if errors:
        print("Mission rejected:")
        for error in errors:
            print(f"  - {error}")
        return 2

    print(f"Validated: {mission_data['name']}")
    print(
        f"Envelope: {rules['minimum_cruise_altitude_ft']}–{rules['maximum_altitude_ft']} ft AGL; "
        f"waypoint threshold {rules['waypoint_acceptance_radius_ft']} ft"
    )
    if args.dry_run:
        return 0
    require_sitl_endpoint(args.connect)
    if not args.start:
        print("Validation complete. Re-run with --start to upload, arm, and enter AUTO in SITL.")
        return 0

    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / f"sitl_mission_{datetime.now():%Y%m%d_%H%M%S}.csv"
    stopping = False

    def request_stop(_signal=None, _frame=None):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(f"Connecting to {args.connect} ...")
    mav = mavutil.mavlink_connection(args.connect)
    mav.wait_heartbeat(timeout=args.heartbeat_timeout)
    heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if heartbeat is None:
        raise TimeoutError("heartbeat stream stopped after initial connection")
    if mavutil.mavlink.MAV_TYPE_FIXED_WING != heartbeat.type:
        raise RuntimeError("connected vehicle is not an ArduPlane fixed-wing vehicle")

    home = current_home(mav, args.heartbeat_timeout)
    upload_mission(mav, home, points)
    set_and_verify_wp_radius(mav, rules["waypoint_acceptance_radius_ft"] * FT_TO_M)
    # Entering AUTO while disarmed can leave an automatic takeoff waiting for
    # a transition that already occurred. Arm in a stabilized mode first, then
    # make the explicit AUTO transition that starts NAV_TAKEOFF.
    set_mode(mav, "FBWA")
    command_and_require_ack(mav, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    set_mode(mav, "AUTO")
    print(f"Mission started; telemetry log: {log_path}")
    result = monitor(mav, log_path, lambda: stopping)

    if result in {"operator_stop", "heartbeat_timeout"}:
        print("Requesting RTL before exit...")
        try:
            set_mode(mav, "RTL")
        except Exception as exc:
            print(f"WARNING: RTL request was not confirmed: {exc}", file=sys.stderr)
    print(f"Mission monitor ended: {result}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
