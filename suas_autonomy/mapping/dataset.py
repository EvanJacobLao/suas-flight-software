"""Validate, manifest, and replay captured mapping datasets."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Iterator

import cv2


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class ValidationIssue:
    level: str
    code: str
    file: str
    detail: str


@dataclass(frozen=True)
class CaptureEntry:
    image_path: Path
    metadata_path: Path
    metadata: dict


@dataclass(frozen=True)
class ValidationReport:
    dataset_dir: str
    image_count: int
    valid_count: int
    issues: tuple[ValidationIssue, ...]

    @property
    def failed(self) -> bool:
        return any(issue.level == "ERROR" for issue in self.issues)


def discover_entries(dataset_dir: Path) -> tuple[list[CaptureEntry], list[ValidationIssue]]:
    entries: list[CaptureEntry] = []
    issues: list[ValidationIssue] = []
    images = sorted(path for path in dataset_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    image_stems = {path.stem for path in images}

    for image_path in images:
        metadata_path = image_path.with_suffix(".json")
        if not metadata_path.exists():
            issues.append(ValidationIssue("ERROR", "MISSING_METADATA", image_path.name, "matching JSON file is absent"))
            continue
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            issues.append(ValidationIssue("ERROR", "INVALID_METADATA", metadata_path.name, str(exc)))
            continue
        entries.append(CaptureEntry(image_path, metadata_path, metadata))

    for metadata_path in sorted(dataset_dir.glob("*.json")):
        if metadata_path.name in {
            "validation_report.json",
            "orthomosaic_manifest.json",
            "coverage_report.json",
            "stitch_report.json",
        }:
            continue
        if metadata_path.stem not in image_stems:
            issues.append(ValidationIssue("WARN", "ORPHAN_METADATA", metadata_path.name, "matching image is absent"))
    return entries, issues


def validate_dataset(dataset_dir: Path, max_telemetry_age_s: float = 0.5) -> tuple[ValidationReport, list[CaptureEntry]]:
    if not dataset_dir.is_dir():
        issue = ValidationIssue("ERROR", "MISSING_DATASET", str(dataset_dir), "directory does not exist")
        return ValidationReport(str(dataset_dir), 0, 0, (issue,)), []

    entries, issues = discover_entries(dataset_dir)
    valid: list[CaptureEntry] = []
    seen_sequences: set[int] = set()
    total_images = sum(1 for path in dataset_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)

    for entry in entries:
        entry_errors = 0
        metadata = entry.metadata

        required = {"schema_version", "image_file", "image_sha256", "width_px", "height_px", "telemetry", "quality"}
        missing = sorted(required - metadata.keys())
        if missing:
            issues.append(ValidationIssue("ERROR", "MISSING_FIELDS", entry.metadata_path.name, ", ".join(missing)))
            entry_errors += 1

        try:
            digest = hashlib.sha256(entry.image_path.read_bytes()).hexdigest()
            if digest != metadata.get("image_sha256"):
                issues.append(ValidationIssue("ERROR", "HASH_MISMATCH", entry.image_path.name, "image content changed"))
                entry_errors += 1
        except OSError as exc:
            issues.append(ValidationIssue("ERROR", "IMAGE_READ", entry.image_path.name, str(exc)))
            entry_errors += 1

        image = cv2.imread(str(entry.image_path), cv2.IMREAD_COLOR)
        if image is None:
            issues.append(ValidationIssue("ERROR", "IMAGE_DECODE", entry.image_path.name, "OpenCV could not decode image"))
            entry_errors += 1
        elif image.shape[1] != metadata.get("width_px") or image.shape[0] != metadata.get("height_px"):
            issues.append(ValidationIssue("ERROR", "DIMENSION_MISMATCH", entry.image_path.name, f"decoded {image.shape[1]}x{image.shape[0]}"))
            entry_errors += 1

        telemetry = metadata.get("telemetry", {})
        for field in ("latitude", "longitude", "agl_m", "roll_deg", "pitch_deg", "yaw_deg"):
            if not isinstance(telemetry.get(field), (int, float)):
                issues.append(ValidationIssue("ERROR", "INVALID_TELEMETRY", entry.metadata_path.name, f"{field} missing or non-numeric"))
                entry_errors += 1
        if not -90 <= telemetry.get("latitude", -999) <= 90 or not -180 <= telemetry.get("longitude", -999) <= 180:
            issues.append(ValidationIssue("ERROR", "GPS_RANGE", entry.metadata_path.name, "latitude or longitude is outside WGS84 range"))
            entry_errors += 1

        frame_time = metadata.get("frame_monotonic_time")
        telemetry_time = telemetry.get("monotonic_time")
        if isinstance(frame_time, (int, float)) and isinstance(telemetry_time, (int, float)):
            age = abs(frame_time - telemetry_time)
            if age > max_telemetry_age_s:
                issues.append(ValidationIssue("WARN", "STALE_TELEMETRY", entry.metadata_path.name, f"age {age:.3f}s"))
        else:
            issues.append(ValidationIssue("WARN", "UNMEASURED_LATENCY", entry.metadata_path.name, "frame/telemetry timing unavailable"))

        if not metadata.get("quality", {}).get("acceptable", False):
            issues.append(ValidationIssue("WARN", "IMAGE_QUALITY", entry.image_path.name, "capture failed configured quality threshold"))

        sequence = metadata.get("source_sequence")
        if isinstance(sequence, int):
            if sequence in seen_sequences:
                issues.append(ValidationIssue("WARN", "DUPLICATE_SEQUENCE", entry.metadata_path.name, str(sequence)))
            seen_sequences.add(sequence)

        if entry_errors == 0:
            valid.append(entry)

    return ValidationReport(str(dataset_dir), total_images, len(valid), tuple(issues)), valid


def write_report(report: ValidationReport, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(report), indent=2), encoding="utf-8")


def write_orthomosaic_manifest(entries: list[CaptureEntry], path: Path) -> None:
    captures = []
    for entry in entries:
        telemetry = entry.metadata["telemetry"]
        captures.append(
            {
                "image": str(entry.image_path.resolve()),
                "latitude": telemetry["latitude"],
                "longitude": telemetry["longitude"],
                "altitude_agl_m": telemetry["agl_m"],
                "roll_deg": telemetry["roll_deg"],
                "pitch_deg": telemetry["pitch_deg"],
                "yaw_deg": telemetry["yaw_deg"],
                "gimbal": entry.metadata.get("gimbal"),
                "camera": entry.metadata.get("camera"),
                "quality": entry.metadata.get("quality"),
            }
        )
    document = {
        "schema_version": 1,
        "purpose": "engine-neutral orthomosaic input manifest",
        "capture_count": len(captures),
        "captures": captures,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2), encoding="utf-8")


class DatasetReplay:
    def __init__(self, entries: list[CaptureEntry], speed: float = 0.0):
        if speed < 0:
            raise ValueError("replay speed cannot be negative")
        self.entries = sorted(entries, key=lambda entry: entry.metadata.get("frame_monotonic_time", 0))
        self.speed = speed

    def __iter__(self) -> Iterator[CaptureEntry]:
        previous_time = None
        for entry in self.entries:
            current_time = entry.metadata.get("frame_monotonic_time")
            if self.speed > 0 and previous_time is not None and isinstance(current_time, (int, float)):
                delay = max(0.0, (current_time - previous_time) / self.speed)
                time.sleep(delay)
            yield entry
            if isinstance(current_time, (int, float)):
                previous_time = current_time


def _distance_m(first: tuple[float, float], second: tuple[float, float]) -> float:
    import math

    lat1, lon1 = map(math.radians, first)
    lat2, lon2 = map(math.radians, second)
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1
    value = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(value)))


def coverage_against_plan(entries: list[CaptureEntry], plan: dict, tolerance_m: float) -> dict:
    planned = []
    for line in plan.get("lines", []):
        planned.extend((point["lat"], point["lon"]) for point in line.get("capture_points", []))
    actual = [
        (entry.metadata["telemetry"]["latitude"], entry.metadata["telemetry"]["longitude"])
        for entry in entries
    ]
    distances = [min((_distance_m(point, capture) for capture in actual), default=float("inf")) for point in planned]
    covered = sum(distance <= tolerance_m for distance in distances)
    return {
        "schema_version": 1,
        "planned_capture_count": len(planned),
        "actual_capture_count": len(actual),
        "covered_planned_points": covered,
        "coverage_fraction": covered / len(planned) if planned else 0.0,
        "tolerance_m": tolerance_m,
        "maximum_nearest_distance_m": max(distances) if distances and actual else None,
        "note": "Position coverage estimate only; it does not prove image footprint, overlap, or orthomosaic quality.",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "manifest", "replay", "coverage"):
        child = subparsers.add_parser(name)
        child.add_argument("dataset", type=Path)
    subparsers.choices["validate"].add_argument("--report", type=Path)
    subparsers.choices["manifest"].add_argument("--output", type=Path)
    subparsers.choices["replay"].add_argument("--speed", type=float, default=0.0, help="0 means no timing delay")
    subparsers.choices["coverage"].add_argument("plan", type=Path)
    subparsers.choices["coverage"].add_argument("--tolerance-m", type=float, default=15.0)
    subparsers.choices["coverage"].add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report, valid = validate_dataset(args.dataset)

    if args.command == "validate":
        report_path = args.report or args.dataset / "validation_report.json"
        write_report(report, report_path)
        print(f"Validated {report.image_count} images: {report.valid_count} structurally valid, {len(report.issues)} issues")
        for issue in report.issues:
            print(f"{issue.level:<5} {issue.code:<20} {issue.file}: {issue.detail}")
        print(f"Report written to {report_path}")
    elif args.command == "manifest":
        output = args.output or args.dataset / "orthomosaic_manifest.json"
        write_orthomosaic_manifest(valid, output)
        print(f"Manifest written with {len(valid)} captures: {output}")
    elif args.command == "replay":
        for index, entry in enumerate(DatasetReplay(valid, args.speed)):
            telemetry = entry.metadata["telemetry"]
            print(f"{index:05d} {entry.image_path.name} {telemetry['latitude']:.7f},{telemetry['longitude']:.7f}")
        print(f"Replayed {len(valid)} captures")
    else:
        plan = json.loads(args.plan.read_text(encoding="utf-8"))
        coverage = coverage_against_plan(valid, plan, args.tolerance_m)
        output = args.output or args.dataset / "coverage_report.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(coverage, indent=2), encoding="utf-8")
        print(
            f"Position coverage: {coverage['covered_planned_points']}/{coverage['planned_capture_count']} "
            f"({coverage['coverage_fraction']:.1%}); report written to {output}"
        )

    return 1 if report.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
