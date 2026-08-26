"""Camera-footprint, capture-spacing, and survey-line planning."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Sequence


EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class LocalPoint:
    east_m: float
    north_m: float


@dataclass(frozen=True)
class CameraGeometry:
    horizontal_fov_deg: float
    image_width_px: int
    image_height_px: int

    @property
    def vertical_fov_deg(self) -> float:
        half_horizontal = math.radians(self.horizontal_fov_deg) / 2
        half_vertical = math.atan(math.tan(half_horizontal) * self.image_height_px / self.image_width_px)
        return math.degrees(2 * half_vertical)


@dataclass(frozen=True)
class CaptureGeometry:
    altitude_m: float
    footprint_width_m: float
    footprint_height_m: float
    along_track_spacing_m: float
    cross_track_spacing_m: float
    capture_interval_s: float


@dataclass(frozen=True)
class SurveyLine:
    index: int
    start: GeoPoint
    end: GeoPoint
    length_m: float
    capture_points: tuple[GeoPoint, ...]


def capture_geometry(
    camera: CameraGeometry,
    altitude_m: float,
    forward_overlap: float,
    side_overlap: float,
    groundspeed_m_s: float,
) -> CaptureGeometry:
    if altitude_m <= 0 or groundspeed_m_s <= 0:
        raise ValueError("altitude and groundspeed must be positive")
    if not 0 <= forward_overlap < 1 or not 0 <= side_overlap < 1:
        raise ValueError("overlap fractions must be in [0, 1)")
    width = 2 * altitude_m * math.tan(math.radians(camera.horizontal_fov_deg) / 2)
    height = 2 * altitude_m * math.tan(math.radians(camera.vertical_fov_deg) / 2)
    along = height * (1 - forward_overlap)
    cross = width * (1 - side_overlap)
    return CaptureGeometry(altitude_m, width, height, along, cross, along / groundspeed_m_s)


class LocalProjection:
    def __init__(self, points: Sequence[GeoPoint]):
        if not points:
            raise ValueError("projection needs at least one point")
        self.origin = GeoPoint(
            sum(point.lat for point in points) / len(points),
            sum(point.lon for point in points) / len(points),
        )
        self.cos_lat = math.cos(math.radians(self.origin.lat))

    def to_local(self, point: GeoPoint) -> LocalPoint:
        north = math.radians(point.lat - self.origin.lat) * EARTH_RADIUS_M
        east = math.radians(point.lon - self.origin.lon) * EARTH_RADIUS_M * self.cos_lat
        return LocalPoint(east, north)

    def to_geo(self, point: LocalPoint) -> GeoPoint:
        lat = self.origin.lat + math.degrees(point.north_m / EARTH_RADIUS_M)
        lon = self.origin.lon + math.degrees(point.east_m / (EARTH_RADIUS_M * self.cos_lat))
        return GeoPoint(lat, lon)


def _to_survey(point: LocalPoint, heading_deg: float) -> tuple[float, float]:
    """Return along-track and cross-track coordinates."""
    heading = math.radians(heading_deg)
    along = point.north_m * math.cos(heading) + point.east_m * math.sin(heading)
    cross = -point.north_m * math.sin(heading) + point.east_m * math.cos(heading)
    return along, cross


def _from_survey(along: float, cross: float, heading_deg: float) -> LocalPoint:
    heading = math.radians(heading_deg)
    north = along * math.cos(heading) - cross * math.sin(heading)
    east = along * math.sin(heading) + cross * math.cos(heading)
    return LocalPoint(east, north)


def _line_intersections(polygon: Sequence[tuple[float, float]], cross: float) -> list[float]:
    intersections: list[float] = []
    previous = polygon[-1]
    for current in polygon:
        along1, cross1 = previous
        along2, cross2 = current
        if (cross1 <= cross < cross2) or (cross2 <= cross < cross1):
            ratio = (cross - cross1) / (cross2 - cross1)
            intersections.append(along1 + ratio * (along2 - along1))
        previous = current
    return sorted(intersections)


def _capture_positions(start: float, end: float, spacing: float) -> list[float]:
    length = abs(end - start)
    count = max(2, math.ceil(length / spacing) + 1)
    return [start + (end - start) * index / (count - 1) for index in range(count)]


def plan_survey(
    boundary: Sequence[GeoPoint],
    heading_deg: float,
    along_spacing_m: float,
    cross_spacing_m: float,
) -> list[SurveyLine]:
    if len(boundary) < 3:
        raise ValueError("survey boundary must contain at least three points")
    if along_spacing_m <= 0 or cross_spacing_m <= 0:
        raise ValueError("capture and line spacing must be positive")

    projection = LocalProjection(boundary)
    survey_polygon = [_to_survey(projection.to_local(point), heading_deg) for point in boundary]
    cross_min = min(point[1] for point in survey_polygon)
    cross_max = max(point[1] for point in survey_polygon)
    line_count = max(1, math.ceil((cross_max - cross_min) / cross_spacing_m))
    actual_spacing = (cross_max - cross_min) / line_count if line_count else 0
    cross_values = [cross_min + actual_spacing * (index + 0.5) for index in range(line_count)]

    lines: list[SurveyLine] = []
    for cross in cross_values:
        intersections = _line_intersections(survey_polygon, cross)
        if len(intersections) < 2:
            continue
        # The official SUAS search regions are convex. For a future concave
        # region, the longest interior segment remains a safe useful baseline.
        pairs = list(zip(intersections[0::2], intersections[1::2]))
        along_start, along_end = max(pairs, key=lambda pair: abs(pair[1] - pair[0]))
        if len(lines) % 2:
            along_start, along_end = along_end, along_start
        captures = _capture_positions(along_start, along_end, along_spacing_m)
        start_local = _from_survey(along_start, cross, heading_deg)
        end_local = _from_survey(along_end, cross, heading_deg)
        capture_geo = tuple(projection.to_geo(_from_survey(value, cross, heading_deg)) for value in captures)
        lines.append(
            SurveyLine(
                index=len(lines),
                start=projection.to_geo(start_local),
                end=projection.to_geo(end_local),
                length_m=abs(along_end - along_start),
                capture_points=capture_geo,
            )
        )
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rules", type=Path, default=Path(__file__).parents[1] / "config" / "suas_2026.json")
    parser.add_argument("--config", type=Path, default=Path(__file__).parents[1] / "config" / "mapping_defaults.json")
    parser.add_argument("--search-boundary", choices=["1", "2"], default="1")
    parser.add_argument("--altitude-m", type=float)
    parser.add_argument("--groundspeed-m-s", type=float)
    parser.add_argument("--forward-overlap", type=float)
    parser.add_argument("--side-overlap", type=float)
    parser.add_argument("--heading-deg", type=float)
    parser.add_argument("--horizontal-fov-deg", type=float)
    parser.add_argument("--resolution")
    parser.add_argument("--output", type=Path, default=Path("output") / "mapping" / "survey_plan.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    defaults = json.loads(args.config.read_text(encoding="utf-8"))
    camera_defaults = defaults["camera"]
    mapping_defaults = defaults["mapping"]
    resolution = args.resolution or camera_defaults["resolution"]
    altitude_m = args.altitude_m if args.altitude_m is not None else mapping_defaults["altitude_m"]
    groundspeed_m_s = args.groundspeed_m_s if args.groundspeed_m_s is not None else mapping_defaults["groundspeed_m_s"]
    forward_overlap = args.forward_overlap if args.forward_overlap is not None else mapping_defaults["forward_overlap"]
    side_overlap = args.side_overlap if args.side_overlap is not None else mapping_defaults["side_overlap"]
    heading_deg = args.heading_deg if args.heading_deg is not None else mapping_defaults["survey_heading_deg"]
    horizontal_fov_deg = args.horizontal_fov_deg if args.horizontal_fov_deg is not None else camera_defaults["horizontal_fov_deg"]
    width, height = (int(part) for part in resolution.lower().split("x", 1))
    rules = json.loads(args.rules.read_text(encoding="utf-8"))
    boundary = [GeoPoint(*values) for values in rules["search_boundaries"][args.search_boundary]]
    geometry = capture_geometry(
        CameraGeometry(horizontal_fov_deg, width, height),
        altitude_m,
        forward_overlap,
        side_overlap,
        groundspeed_m_s,
    )
    lines = plan_survey(boundary, heading_deg, geometry.along_track_spacing_m, geometry.cross_track_spacing_m)
    document = {
        "schema_version": 1,
        "profile": rules["profile"],
        "search_boundary": args.search_boundary,
        "camera": asdict(CameraGeometry(horizontal_fov_deg, width, height)),
        "capture_geometry": asdict(geometry),
        "survey_heading_deg": heading_deg % 360,
        "fixed_wing_warning": "Survey lines do not include turn arcs; turns must remain inside the flight boundary.",
        "line_count": len(lines),
        "capture_count": sum(len(line.capture_points) for line in lines),
        "lines": [asdict(line) for line in lines],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2), encoding="utf-8")
    print(
        f"Planned {document['line_count']} survey lines and {document['capture_count']} captures; "
        f"capture every {geometry.capture_interval_s:.2f}s at {groundspeed_m_s:.1f}m/s"
    )
    print(f"Plan written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
