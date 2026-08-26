"""Camera-independent mapping capture pipeline.

Synthetic mode runs with no sensors. The same pipeline can pair synthetic or
recorded frames with live MAVLink telemetry, and a future SIYI adapter only
needs to implement the FrameSource protocol.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Protocol

import cv2
import numpy as np


@dataclass(frozen=True)
class FramePacket:
    image: np.ndarray
    monotonic_time: float
    source_sequence: int
    source: str


@dataclass(frozen=True)
class TelemetrySample:
    monotonic_time: float
    latitude: float
    longitude: float
    agl_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    groundspeed_m_s: float
    source: str


class FrameSource(Protocol):
    def read(self) -> FramePacket | None: ...
    def close(self) -> None: ...


class TelemetrySource(Protocol):
    def latest(self, at_time: float) -> TelemetrySample: ...
    def close(self) -> None: ...


class SyntheticCamera:
    """Deterministic moving aerial-style scene for hardware-free testing."""

    def __init__(self, width: int, height: int, frame_rate: float = 5.0):
        self.width = width
        self.height = height
        self.period = 1.0 / frame_rate
        self.sequence = 0
        self.next_frame_time = time.monotonic()

    def read(self) -> FramePacket:
        delay = self.next_frame_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        captured = time.monotonic()
        image = self._render(self.sequence)
        packet = FramePacket(image, captured, self.sequence, "synthetic")
        self.sequence += 1
        self.next_frame_time = captured + self.period
        return packet

    def _render(self, sequence: int) -> np.ndarray:
        x = np.linspace(0, 1, self.width, dtype=np.float32)
        y = np.linspace(0, 1, self.height, dtype=np.float32)[:, None]
        blue = np.broadcast_to((80 + 80 * x).astype(np.uint8), (self.height, self.width))
        green = np.broadcast_to((100 + 90 * y).astype(np.uint8), (self.height, self.width))
        red = ((blue.astype(np.uint16) + green.astype(np.uint16)) // 3).astype(np.uint8)
        image = np.dstack((blue, green, red)).copy()

        grid = max(40, min(self.width, self.height) // 10)
        offset = (sequence * 7) % grid
        for px in range(-grid + offset, self.width, grid):
            cv2.line(image, (px, 0), (px, self.height - 1), (90, 90, 90), 2)
        for py in range(-grid + offset, self.height, grid):
            cv2.line(image, (0, py), (self.width - 1, py), (90, 90, 90), 2)

        centre = (int(self.width * 0.55), int(self.height * 0.48))
        cv2.circle(image, centre, max(10, grid // 3), (20, 20, 230), -1)
        cv2.putText(image, f"SYNTHETIC {sequence:04d}", (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return image

    def close(self) -> None:
        return None


class VideoFileCamera:
    def __init__(self, path: Path):
        self.capture = cv2.VideoCapture(str(path))
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open video {path}")
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.period = 1.0 / fps if fps and fps > 0 else 0.04
        self.sequence = 0
        self.next_frame_time = time.monotonic()

    def read(self) -> FramePacket | None:
        delay = self.next_frame_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        success, frame = self.capture.read()
        if not success:
            return None
        captured = time.monotonic()
        packet = FramePacket(frame, captured, self.sequence, f"video:{self.capture.getBackendName()}")
        self.sequence += 1
        self.next_frame_time = captured + self.period
        return packet

    def close(self) -> None:
        self.capture.release()


class SyntheticTelemetry:
    def __init__(self, origin_lat: float = -35.363262, origin_lon: float = 149.165237):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.started = time.monotonic()

    def latest(self, at_time: float) -> TelemetrySample:
        elapsed = max(0.0, at_time - self.started)
        north_m = elapsed * 18.0
        east_m = 20.0 * math.sin(elapsed / 8.0)
        latitude = self.origin_lat + north_m / 111_320.0
        longitude = self.origin_lon + east_m / (111_320.0 * math.cos(math.radians(self.origin_lat)))
        return TelemetrySample(
            monotonic_time=at_time,
            latitude=latitude,
            longitude=longitude,
            agl_m=60.0,
            roll_deg=3.0 * math.sin(elapsed),
            pitch_deg=1.0,
            yaw_deg=353.0,
            groundspeed_m_s=18.0,
            source="synthetic",
        )

    def close(self) -> None:
        return None


class MavlinkTelemetry:
    """Background MAVLink state cache; importing pymavlink is deferred."""

    def __init__(self, connection: str, heartbeat_timeout: float = 20.0):
        from pymavlink import mavutil

        self.mavutil = mavutil
        self.connection = mavutil.mavlink_connection(connection)
        self.connection.wait_heartbeat(timeout=heartbeat_timeout)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sample: TelemetrySample | None = None
        self._position = None
        self._attitude = None
        self._hud = None
        self._thread = threading.Thread(target=self._receive, name="mavlink-telemetry", daemon=True)
        self._thread.start()

    def _receive(self) -> None:
        while not self._stop.is_set():
            message = self.connection.recv_match(
                type=["GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD"], blocking=True, timeout=0.5
            )
            if message is None:
                continue
            with self._lock:
                kind = message.get_type()
                if kind == "GLOBAL_POSITION_INT":
                    self._position = message
                elif kind == "ATTITUDE":
                    self._attitude = message
                elif kind == "VFR_HUD":
                    self._hud = message
                if self._position is not None and self._attitude is not None:
                    self._sample = TelemetrySample(
                        monotonic_time=time.monotonic(),
                        latitude=self._position.lat / 1e7,
                        longitude=self._position.lon / 1e7,
                        agl_m=self._position.relative_alt / 1000,
                        roll_deg=math.degrees(self._attitude.roll),
                        pitch_deg=math.degrees(self._attitude.pitch),
                        yaw_deg=math.degrees(self._attitude.yaw) % 360,
                        groundspeed_m_s=float(self._hud.groundspeed) if self._hud else 0.0,
                        source="mavlink",
                    )

    def latest(self, at_time: float) -> TelemetrySample:
        with self._lock:
            sample = self._sample
        if sample is None:
            raise RuntimeError("MAVLink connected but no complete position/attitude sample received")
        return sample

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.connection.close()


def image_quality(image: np.ndarray) -> dict[str, float | bool]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    brightness = float(np.mean(gray))
    blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    dark_fraction = float(np.mean(gray < 10))
    bright_fraction = float(np.mean(gray > 245))
    acceptable = 25 <= brightness <= 235 and blur_score >= 20 and dark_fraction < 0.5 and bright_fraction < 0.5
    return {
        "brightness_mean": round(brightness, 3),
        "blur_laplacian_variance": round(blur_score, 3),
        "dark_fraction": round(dark_fraction, 5),
        "bright_fraction": round(bright_fraction, 5),
        "acceptable": acceptable,
    }


def write_capture(output_dir: Path, capture_index: int, packet: FramePacket, telemetry: TelemetrySample) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"capture_{capture_index:05d}"
    image_path = output_dir / f"{stem}.jpg"
    metadata_path = output_dir / f"{stem}.json"
    temporary_image = output_dir / f".{stem}.jpg.tmp"
    temporary_metadata = output_dir / f".{stem}.json.tmp"

    success, encoded = cv2.imencode(".jpg", packet.image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not success:
        raise RuntimeError("OpenCV failed to encode JPEG")
    payload = encoded.tobytes()
    temporary_image.write_bytes(payload)

    metadata = {
        "schema_version": 1,
        "capture_utc": datetime.now(timezone.utc).isoformat(),
        "image_file": image_path.name,
        "image_sha256": hashlib.sha256(payload).hexdigest(),
        "width_px": int(packet.image.shape[1]),
        "height_px": int(packet.image.shape[0]),
        "source_sequence": packet.source_sequence,
        "frame_monotonic_time": packet.monotonic_time,
        "telemetry": asdict(telemetry),
        "gimbal": {"pitch_deg": -90.0, "yaw_deg": 0.0, "source": "configured_placeholder"},
        "camera": {"model": packet.source, "calibration_id": None},
        "quality": image_quality(packet.image),
    }
    temporary_metadata.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    temporary_image.replace(image_path)
    temporary_metadata.replace(metadata_path)
    return image_path, metadata_path


def run_pipeline(
    camera: FrameSource,
    telemetry: TelemetrySource,
    output_dir: Path,
    capture_count: int,
    capture_interval_s: float,
) -> list[tuple[Path, Path]]:
    captures: list[tuple[Path, Path]] = []
    next_capture = time.monotonic()
    try:
        while len(captures) < capture_count:
            packet = camera.read()
            if packet is None:
                break
            if packet.monotonic_time < next_capture:
                continue
            sample = telemetry.latest(packet.monotonic_time)
            paths = write_capture(output_dir, len(captures), packet, sample)
            captures.append(paths)
            quality = image_quality(packet.image)
            print(
                f"saved {paths[0].name}: lat={sample.latitude:.7f} lon={sample.longitude:.7f} "
                f"agl={sample.agl_m:.1f}m quality={'PASS' if quality['acceptable'] else 'WARN'}"
            )
            next_capture = packet.monotonic_time + capture_interval_s
    finally:
        camera.close()
        telemetry.close()
    return captures


def parse_resolution(value: str) -> tuple[int, int]:
    try:
        width, height = (int(part) for part in value.lower().split("x", 1))
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("resolution must look like 1920x1080") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("resolution dimensions must be positive")
    return width, height


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=["synthetic", "video"], default="synthetic")
    parser.add_argument("--video", type=Path, help="required when --camera video")
    parser.add_argument("--telemetry", choices=["synthetic", "mavlink"], default="synthetic")
    parser.add_argument("--mavlink", default="tcp:127.0.0.1:5762")
    parser.add_argument("--resolution", type=parse_resolution, default=(1280, 720))
    parser.add_argument("--frames", type=int, default=10, help="number of mapping captures")
    parser.add_argument("--capture-interval", type=float, default=1.0)
    parser.add_argument("--output", type=Path, default=Path("output") / "captures")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.frames <= 0 or args.capture_interval < 0:
        raise SystemExit("--frames must be positive and --capture-interval cannot be negative")

    if args.camera == "synthetic":
        camera: FrameSource = SyntheticCamera(*args.resolution)
    else:
        if args.video is None:
            raise SystemExit("--video is required with --camera video")
        camera = VideoFileCamera(args.video)

    telemetry: TelemetrySource
    if args.telemetry == "synthetic":
        telemetry = SyntheticTelemetry()
    else:
        telemetry = MavlinkTelemetry(args.mavlink)

    captures = run_pipeline(camera, telemetry, args.output, args.frames, args.capture_interval)
    print(f"Capture pipeline complete: {len(captures)} image/metadata pairs in {args.output}")
    return 0 if len(captures) == args.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
