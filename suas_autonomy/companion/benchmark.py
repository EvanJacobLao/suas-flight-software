"""Driver-independent frame-processing benchmark for laptop or Jetson."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import platform
import statistics
import time

import cv2

from .capture_pipeline import SyntheticCamera, image_quality


@dataclass(frozen=True)
class ResolutionResult:
    width: int
    height: int
    frames: int
    quality_fps: float
    jpeg_fps: float
    jpeg_mean_mib: float
    feature_fps: float
    orb_keypoints_mean: float


def benchmark_resolution(width: int, height: int, frames: int) -> ResolutionResult:
    camera = SyntheticCamera(width, height, frame_rate=1000)
    images = [camera.read().image for _ in range(frames)]
    camera.close()

    started = time.perf_counter()
    for image in images:
        image_quality(image)
    quality_elapsed = max(time.perf_counter() - started, 1e-6)

    sizes = []
    started = time.perf_counter()
    for image in images:
        success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not success:
            raise RuntimeError("JPEG benchmark encoding failed")
        sizes.append(encoded.nbytes / 1024**2)
    jpeg_elapsed = max(time.perf_counter() - started, 1e-6)

    detector = cv2.ORB_create(nfeatures=4000)
    keypoint_counts = []
    started = time.perf_counter()
    for image in images:
        keypoints = detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
        keypoint_counts.append(len(keypoints))
    feature_elapsed = max(time.perf_counter() - started, 1e-6)

    return ResolutionResult(
        width,
        height,
        frames,
        round(frames / quality_elapsed, 2),
        round(frames / jpeg_elapsed, 2),
        round(statistics.mean(sizes), 3),
        round(frames / feature_elapsed, 2),
        round(statistics.mean(keypoint_counts), 1),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--resolutions", nargs="+", default=["1280x720", "1920x1080", "3840x2160"])
    parser.add_argument("--output", type=Path, default=Path("output") / "orin_frame_benchmark.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.frames <= 0:
        raise SystemExit("--frames must be positive")
    results = []
    for value in args.resolutions:
        width, height = (int(part) for part in value.lower().split("x", 1))
        result = benchmark_resolution(width, height, args.frames)
        results.append(result)
        print(
            f"{width}x{height}: quality={result.quality_fps:.2f}fps "
            f"jpeg={result.jpeg_fps:.2f}fps ORB={result.feature_fps:.2f}fps"
        )
    document = {
        "schema_version": 1,
        "platform": platform.platform(),
        "opencv": cv2.__version__,
        "results": [asdict(result) for result in results],
        "note": "Synthetic CPU/OpenCV baseline; it does not measure SIYI decoding, CUDA, TensorRT, or YOLO.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2), encoding="utf-8")
    print(f"Benchmark written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

