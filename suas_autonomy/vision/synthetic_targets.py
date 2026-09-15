"""Generate and score deterministic synthetic aerial target scenes.

The simple colour detector is a test oracle, not a competition detector.  It
verifies capture -> detection -> pixel-to-ground -> GPS plumbing while a real
trained model and real aerial datasets are developed behind the same result
format.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random

import cv2
import numpy as np


@dataclass(frozen=True)
class BoundingBox:
    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True)
class TargetTruth:
    label: str
    box: BoundingBox
    latitude: float
    longitude: float


@dataclass(frozen=True)
class Detection:
    box: BoundingBox
    latitude: float
    longitude: float
    confidence: float


def pixel_to_geo(
    x: float,
    y: float,
    *,
    width: int,
    height: int,
    metres_per_pixel: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    east_m = (x - width / 2) * metres_per_pixel
    north_m = (height / 2 - y) * metres_per_pixel
    lat = origin_lat + north_m / 111_320.0
    lon = origin_lon + east_m / (111_320.0 * math.cos(math.radians(origin_lat)))
    return lat, lon


def generate_scene(
    width: int = 1280,
    height: int = 720,
    target_count: int = 4,
    seed: int = 7,
    metres_per_pixel: float = 0.08,
    origin_lat: float = -35.363262,
    origin_lon: float = 149.165237,
) -> tuple[np.ndarray, list[TargetTruth]]:
    if width < 160 or height < 120 or target_count < 0:
        raise ValueError("scene dimensions or target count are invalid")
    rng = np.random.default_rng(seed)
    py_rng = random.Random(seed)
    base = rng.normal(118, 20, (height, width, 3)).clip(25, 220).astype(np.uint8)
    base = cv2.GaussianBlur(base, (0, 0), 3)
    truths: list[TargetTruth] = []
    labels = ["person", "car", "airplane", "stop_sign", "sports_ball", "suitcase"]

    for index in range(target_count):
        size = py_rng.randint(max(14, min(width, height) // 35), max(20, min(width, height) // 18))
        cx = py_rng.randint(size + 10, width - size - 10)
        cy = py_rng.randint(size + 10, height - size - 10)
        angle = py_rng.uniform(0, 360)
        rectangle = ((cx, cy), (size * 1.5, size), angle)
        points = cv2.boxPoints(rectangle).astype(np.int32)
        cv2.fillConvexPoly(base, points, (20, 20, 235))
        cv2.polylines(base, [points], True, (245, 245, 245), max(2, size // 8))
        x, y, w, h = cv2.boundingRect(points)
        lat, lon = pixel_to_geo(
            cx,
            cy,
            width=width,
            height=height,
            metres_per_pixel=metres_per_pixel,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )
        truths.append(TargetTruth(labels[index % len(labels)], BoundingBox(x, y, w, h), lat, lon))
    return base, truths


def detect_test_targets(
    image: np.ndarray,
    *,
    metres_per_pixel: float,
    origin_lat: float,
    origin_lon: float,
) -> list[Detection]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_a = cv2.inRange(hsv, (0, 100, 90), (12, 255, 255))
    mask_b = cv2.inRange(hsv, (168, 100, 90), (179, 255, 255))
    mask = cv2.morphologyEx(mask_a | mask_b, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[Detection] = []
    height, width = image.shape[:2]
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 80:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        cx, cy = x + w / 2, y + h / 2
        lat, lon = pixel_to_geo(
            cx,
            cy,
            width=width,
            height=height,
            metres_per_pixel=metres_per_pixel,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )
        confidence = min(1.0, area / max(1.0, w * h))
        detections.append(Detection(BoundingBox(x, y, w, h), lat, lon, round(confidence, 4)))
    return sorted(detections, key=lambda item: (item.box.y, item.box.x))


def centre_error_px(truth: TargetTruth, detection: Detection) -> float:
    tx = truth.box.x + truth.box.width / 2
    ty = truth.box.y + truth.box.height / 2
    dx = detection.box.x + detection.box.width / 2
    dy = detection.box.y + detection.box.height / 2
    return math.hypot(tx - dx, ty - dy)


def evaluate(truths: list[TargetTruth], detections: list[Detection]) -> dict[str, float | int]:
    remaining = list(detections)
    errors: list[float] = []
    for truth in truths:
        if not remaining:
            break
        nearest = min(remaining, key=lambda item: centre_error_px(truth, item))
        error = centre_error_px(truth, nearest)
        if error <= max(truth.box.width, truth.box.height):
            errors.append(error)
            remaining.remove(nearest)
    return {
        "truth_count": len(truths),
        "detection_count": len(detections),
        "matched_count": len(errors),
        "precision": round(len(errors) / len(detections), 4) if detections else 0.0,
        "recall": round(len(errors) / len(truths), 4) if truths else 1.0,
        "mean_centre_error_px": round(float(np.mean(errors)), 3) if errors else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output") / "vision" / "synthetic_scene.jpg")
    parser.add_argument("--targets", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    image, truths = generate_scene(target_count=args.targets, seed=args.seed)
    detections = detect_test_targets(
        image, metres_per_pixel=0.08, origin_lat=-35.363262, origin_lon=149.165237
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), image)
    report = {
        "image": args.output.name,
        "ground_truth": [asdict(item) for item in truths],
        "detections": [asdict(item) for item in detections],
        "evaluation": evaluate(truths, detections),
    }
    report_path = args.output.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["evaluation"], indent=2))
    return 0 if report["evaluation"]["recall"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
