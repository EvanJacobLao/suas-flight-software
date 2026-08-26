"""Prepare a validated capture dataset for OpenDroneMap processing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

from .dataset import validate_dataset


def prepare_odm_project(dataset: Path, project_root: Path, project_name: str) -> dict:
    report, entries = validate_dataset(dataset)
    if report.failed or not entries:
        raise RuntimeError("dataset has structural errors or no valid captures; run dataset validate first")

    project = project_root / project_name
    images_dir = project / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    geo_lines = ["EPSG:4326"]
    copied = []

    for entry in entries:
        destination = images_dir / entry.image_path.name
        shutil.copy2(entry.image_path, destination)
        copied.append(destination.name)
        telemetry = entry.metadata["telemetry"]
        longitude = telemetry["longitude"]
        latitude = telemetry["latitude"]
        altitude_msl = telemetry.get("altitude_msl_m")
        if isinstance(altitude_msl, (int, float)):
            geo_lines.append(
                f"{destination.name} {longitude:.10f} {latitude:.10f} {altitude_msl:.3f} "
                f"{telemetry['yaw_deg']:.3f} {telemetry['pitch_deg']:.3f} {telemetry['roll_deg']:.3f}"
            )
        else:
            # AGL is not a valid substitute for WGS84/MSL camera elevation.
            geo_lines.append(f"{destination.name} {longitude:.10f} {latitude:.10f}")

    (project / "geo.txt").write_text("\n".join(geo_lines) + "\n", encoding="utf-8")
    handoff = {
        "schema_version": 1,
        "project_root": str(project_root.resolve()),
        "project_name": project_name,
        "image_count": len(copied),
        "images": copied,
        "geo_file": str((project / "geo.txt").resolve()),
        "altitude_note": "AGL is intentionally omitted when MSL/WGS84 camera elevation is unavailable.",
        "docker_command": (
            f"docker run --rm -v {project_root.resolve()}:/datasets "
            f"opendronemap/odm --project-path /datasets {project_name} --orthophoto-png"
        ),
    }
    (project / "handoff.json").write_text(json.dumps(handoff, indent=2), encoding="utf-8")
    return handoff


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--project-root", type=Path, default=Path("output") / "odm_projects")
    parser.add_argument("--project-name", default="suas_mapping")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    handoff = prepare_odm_project(args.dataset, args.project_root, args.project_name)
    print(f"Prepared {handoff['image_count']} images in {handoff['project_root']}/{handoff['project_name']}")
    print("ODM is not installed or run automatically. When ready, review and run:")
    print(handoff["docker_command"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

