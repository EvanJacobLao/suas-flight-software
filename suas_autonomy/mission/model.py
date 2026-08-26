"""Pure mission geometry and validation helpers.

This module deliberately has no pymavlink dependency so its safety rules can
be unit tested without a running simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


EARTH_RADIUS_M = 6_378_137.0
FT_TO_M = 0.3048


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class MissionPoint:
    command: str
    north_m: float
    east_m: float
    altitude_m: float


def offset_point(origin: GeoPoint, north_m: float, east_m: float) -> GeoPoint:
    """Return a WGS84 approximation suitable for a local mission area."""
    lat_rad = math.radians(origin.lat)
    d_lat = north_m / EARTH_RADIUS_M
    d_lon = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    return GeoPoint(origin.lat + math.degrees(d_lat), origin.lon + math.degrees(d_lon))


def distance_m(a: GeoPoint, b: GeoPoint) -> float:
    """Great-circle distance between two WGS84 points."""
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    d_lat = lat2 - lat1
    d_lon = math.radians(b.lon - a.lon)
    h = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def point_in_polygon(point: GeoPoint, polygon: Sequence[GeoPoint]) -> bool:
    """Ray-casting containment test; boundary points are treated as inside."""
    if len(polygon) < 3:
        return False

    x, y = point.lon, point.lat
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous.lon, previous.lat
        x2, y2 = current.lon, current.lat

        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) < 1e-12 and min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            return True

        if (y1 > y) != (y2 > y):
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x <= intersection_x:
                inside = not inside
        previous = current
    return inside


def validate_mission(
    points: Iterable[MissionPoint],
    *,
    min_cruise_altitude_m: float,
    max_altitude_m: float,
    runway_exception_radius_m: float,
) -> list[str]:
    """Validate the handbook altitude envelope for a relative mission.

    Altitudes are relative to the launch point. TAKEOFF and LAND are allowed
    below the minimum. Other points below the minimum are allowed only inside
    the runway exception radius (500 ft in the 2026 handbook).
    """
    errors: list[str] = []
    materialized = list(points)
    if not materialized:
        return ["mission has no points"]
    if materialized[0].command != "TAKEOFF":
        errors.append("first mission item must be TAKEOFF")
    if materialized[-1].command != "LAND":
        errors.append("last mission item must be LAND")

    allowed = {"TAKEOFF", "WAYPOINT", "LAND"}
    for index, item in enumerate(materialized, start=1):
        if item.command not in allowed:
            errors.append(f"item {index}: unsupported command {item.command!r}")
        if item.altitude_m < 0:
            errors.append(f"item {index}: altitude cannot be negative")
        if item.altitude_m > max_altitude_m:
            errors.append(f"item {index}: altitude exceeds {max_altitude_m:.2f} m")
        radius = math.hypot(item.north_m, item.east_m)
        if (
            item.command == "WAYPOINT"
            and radius > runway_exception_radius_m
            and item.altitude_m < min_cruise_altitude_m
        ):
            errors.append(
                f"item {index}: below {min_cruise_altitude_m:.2f} m outside runway exception"
            )
    return errors
