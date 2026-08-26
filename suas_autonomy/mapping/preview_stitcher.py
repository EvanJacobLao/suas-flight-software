"""Folder-based planar preview stitcher with feature-match reporting.

This is a fast integration preview, not a georeferenced orthomosaic engine.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .dataset import IMAGE_SUFFIXES


@dataclass(frozen=True)
class PairMatch:
    first: str
    second: str
    keypoints_first: int
    keypoints_second: int
    good_matches: int
    homography_inliers: int
    inlier_ratio: float
    acceptable: bool


def load_images(folder: Path, max_dimension: int) -> tuple[list[Path], list[np.ndarray], list[float]]:
    paths = sorted(path for path in folder.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    images: list[np.ndarray] = []
    scales: list[float] = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot decode {path}")
        largest = max(image.shape[:2])
        scale = min(1.0, max_dimension / largest) if max_dimension > 0 else 1.0
        if scale < 1:
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        images.append(image)
        scales.append(scale)
    return paths, images, scales


def match_pair(first_path: Path, second_path: Path, first: np.ndarray, second: np.ndarray) -> PairMatch:
    detector = cv2.ORB_create(nfeatures=5000)
    gray_first = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
    gray_second = cv2.cvtColor(second, cv2.COLOR_BGR2GRAY)
    keypoints_first, descriptors_first = detector.detectAndCompute(gray_first, None)
    keypoints_second, descriptors_second = detector.detectAndCompute(gray_second, None)
    good = []
    inliers = 0

    if descriptors_first is not None and descriptors_second is not None:
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        for candidates in matcher.knnMatch(descriptors_first, descriptors_second, k=2):
            if len(candidates) == 2 and candidates[0].distance < 0.75 * candidates[1].distance:
                good.append(candidates[0])
        if len(good) >= 8:
            source = np.float32([keypoints_first[item.queryIdx].pt for item in good]).reshape(-1, 1, 2)
            destination = np.float32([keypoints_second[item.trainIdx].pt for item in good]).reshape(-1, 1, 2)
            _, mask = cv2.findHomography(source, destination, cv2.RANSAC, 4.0)
            if mask is not None:
                inliers = int(mask.sum())

    ratio = inliers / len(good) if good else 0.0
    acceptable = len(good) >= 12 and inliers >= 8 and ratio >= 0.25
    return PairMatch(
        first_path.name,
        second_path.name,
        len(keypoints_first),
        len(keypoints_second),
        len(good),
        inliers,
        round(ratio, 4),
        acceptable,
    )


def analyze_pairs(paths: list[Path], images: list[np.ndarray]) -> list[PairMatch]:
    return [match_pair(paths[index], paths[index + 1], images[index], images[index + 1]) for index in range(len(images) - 1)]


def stitch_images(images: list[np.ndarray]) -> tuple[int, np.ndarray | None, float]:
    if len(images) < 2:
        raise ValueError("at least two images are required")
    stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
    stitcher.setPanoConfidenceThresh(0.3)
    started = time.perf_counter()
    status, panorama = stitcher.stitch(images)
    return status, panorama, time.perf_counter() - started


def status_name(status: int) -> str:
    names = {
        cv2.Stitcher_OK: "OK",
        cv2.Stitcher_ERR_NEED_MORE_IMGS: "NEED_MORE_IMAGES",
        cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL: "HOMOGRAPHY_ESTIMATION_FAILED",
        cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "CAMERA_ADJUSTMENT_FAILED",
    }
    return names.get(status, f"UNKNOWN_{status}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, default=Path("output") / "mapping" / "preview_panorama.jpg")
    parser.add_argument("--report", type=Path, default=Path("output") / "mapping" / "stitch_report.json")
    parser.add_argument("--max-dimension", type=int, default=1600, help="resize input for preview; 0 disables")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths, images, scales = load_images(args.input, args.max_dimension)
    if len(images) < 2:
        raise SystemExit("preview stitching requires at least two decodable images")
    pair_matches = analyze_pairs(paths, images)
    status, panorama, elapsed = stitch_images(images)

    report = {
        "schema_version": 1,
        "purpose": "non-georeferenced integration preview",
        "input_count": len(images),
        "input_files": [path.name for path in paths],
        "input_scales": scales,
        "pair_matches": [asdict(result) for result in pair_matches],
        "weak_pair_count": sum(not result.acceptable for result in pair_matches),
        "stitch_status": status_name(status),
        "elapsed_s": round(elapsed, 4),
        "output": str(args.output) if panorama is not None else None,
        "warning": "Preview only; use a photogrammetry engine for the competition orthomosaic.",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if status != cv2.Stitcher_OK or panorama is None:
        print(f"Stitch failed: {status_name(status)}; report written to {args.report}")
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), panorama, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise RuntimeError(f"failed to write {args.output}")
    print(f"Stitched {len(images)} images into {args.output} in {elapsed:.2f}s")
    print(f"Match report written to {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

