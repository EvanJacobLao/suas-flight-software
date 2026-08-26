"""Hardware-tolerant readiness checks for the SUAS Jetson companion computer.

The default run needs only Python. Jetson-only and disconnected-device checks
are reported as SKIPPED instead of making development on another computer fail.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import tempfile
import time
from typing import Callable


@dataclass(frozen=True)
class CheckResult:
    name: str
    status: str
    detail: str


def _read_text(path: str | Path) -> str | None:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").replace("\x00", "").strip()
    except (OSError, PermissionError):
        return None


def _command(command: list[str], timeout: float = 5.0) -> tuple[int | None, str]:
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        output = (completed.stdout or completed.stderr).strip()
        return completed.returncode, output
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)


def check_platform() -> CheckResult:
    model = _read_text("/proc/device-tree/model")
    detail = model or f"{platform.system()} {platform.machine()}"
    status = "PASS" if model and "NVIDIA" in model.upper() else "WARN"
    return CheckResult("Platform", status, detail)


def check_operating_system() -> CheckResult:
    os_release = _read_text("/etc/os-release")
    if not os_release:
        return CheckResult("Operating system", "WARN", platform.platform())
    values = {}
    for line in os_release.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip('"')
    return CheckResult("Operating system", "PASS", values.get("PRETTY_NAME", "Linux"))


def check_jetson_linux() -> CheckResult:
    release = _read_text("/etc/nv_tegra_release")
    if release:
        return CheckResult("Jetson Linux (L4T)", "PASS", release.splitlines()[0])
    return CheckResult("Jetson Linux (L4T)", "SKIPPED", "NVIDIA L4T release file not found")


def check_memory(minimum_gib: float = 12.0) -> CheckResult:
    meminfo = _read_text("/proc/meminfo")
    if not meminfo:
        return CheckResult("System memory", "WARN", "unable to read /proc/meminfo")
    first = next((line for line in meminfo.splitlines() if line.startswith("MemTotal:")), "")
    try:
        gib = int(first.split()[1]) / 1024 / 1024
    except (IndexError, ValueError):
        return CheckResult("System memory", "WARN", first or "unknown")
    status = "PASS" if gib >= minimum_gib else "WARN"
    return CheckResult("System memory", status, f"{gib:.1f} GiB detected")


def check_storage(path: Path, minimum_free_gib: float = 10.0) -> CheckResult:
    try:
        usage = shutil.disk_usage(path)
    except OSError as exc:
        return CheckResult("Storage space", "FAIL", str(exc))
    free_gib = usage.free / 1024**3
    total_gib = usage.total / 1024**3
    status = "PASS" if free_gib >= minimum_free_gib else "WARN"
    return CheckResult("Storage space", status, f"{free_gib:.1f} GiB free of {total_gib:.1f} GiB at {path}")


def check_storage_write(path: Path, size_mb: int) -> CheckResult:
    if size_mb <= 0:
        return CheckResult("Storage write", "SKIPPED", "disabled")
    chunk = b"\0" * (1024 * 1024)
    try:
        path.mkdir(parents=True, exist_ok=True)
        started = time.perf_counter()
        with tempfile.NamedTemporaryFile(dir=path, prefix="orin_write_test_", delete=True) as stream:
            for _ in range(size_mb):
                stream.write(chunk)
            stream.flush()
            os.fsync(stream.fileno())
        elapsed = max(time.perf_counter() - started, 1e-6)
    except OSError as exc:
        return CheckResult("Storage write", "FAIL", str(exc))
    return CheckResult("Storage write", "PASS", f"{size_mb / elapsed:.1f} MiB/s ({size_mb} MiB test)")


def check_python_module(module_name: str, display_name: str | None = None) -> CheckResult:
    display_name = display_name or module_name
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        return CheckResult(display_name, "WARN", f"not importable: {exc}")
    version = getattr(module, "__version__", "installed")
    return CheckResult(display_name, "PASS", str(version))


def check_opencv() -> CheckResult:
    try:
        cv2 = importlib.import_module("cv2")
    except Exception as exc:
        return CheckResult("OpenCV", "WARN", f"not importable: {exc}")
    build = cv2.getBuildInformation()
    gstreamer_line = next((line.strip() for line in build.splitlines() if "GStreamer:" in line), "GStreamer: unknown")
    status = "PASS" if "YES" in gstreamer_line else "WARN"
    return CheckResult("OpenCV", status, f"{cv2.__version__}; {gstreamer_line}")


def check_command_version(executable: str, arguments: list[str], display_name: str) -> CheckResult:
    if shutil.which(executable) is None:
        return CheckResult(display_name, "WARN", f"{executable} not found")
    code, output = _command([executable, *arguments])
    first_line = output.splitlines()[0] if output else "available"
    return CheckResult(display_name, "PASS" if code == 0 else "WARN", first_line)


def check_power_mode() -> CheckResult:
    if shutil.which("nvpmodel") is None:
        return CheckResult("Jetson power mode", "SKIPPED", "nvpmodel not found")
    code, output = _command(["nvpmodel", "-q"])
    return CheckResult("Jetson power mode", "PASS" if code == 0 else "WARN", output or "unknown")


def check_thermal() -> CheckResult:
    thermal_root = Path("/sys/class/thermal")
    readings: list[tuple[str, float]] = []
    if thermal_root.exists():
        for zone in thermal_root.glob("thermal_zone*"):
            raw = _read_text(zone / "temp")
            if raw is None:
                continue
            try:
                value = float(raw)
                celsius = value / 1000 if value > 1000 else value
            except ValueError:
                continue
            name = _read_text(zone / "type") or zone.name
            readings.append((name, celsius))
    if not readings:
        return CheckResult("Temperatures", "SKIPPED", "no readable thermal zones")
    hottest = max(value for _, value in readings)
    status = "PASS" if hottest < 80 else "WARN"
    detail = ", ".join(f"{name}={value:.1f}C" for name, value in readings[:8])
    return CheckResult("Temperatures", status, detail)


def check_optional_endpoint(name: str, host: str | None, port: int) -> CheckResult:
    if not host:
        return CheckResult(name, "SKIPPED", "not requested / hardware not connected")
    try:
        with socket.create_connection((host, port), timeout=2):
            pass
    except OSError as exc:
        return CheckResult(name, "WARN", f"{host}:{port} unreachable: {exc}")
    return CheckResult(name, "PASS", f"{host}:{port} reachable")


def run_checks(storage_path: Path, write_test_mb: int, camera_host: str | None) -> list[CheckResult]:
    checks: list[Callable[[], CheckResult]] = [
        check_platform,
        check_operating_system,
        check_jetson_linux,
        check_memory,
        lambda: check_storage(storage_path),
        lambda: check_storage_write(storage_path, write_test_mb),
        lambda: check_python_module("numpy", "NumPy"),
        check_opencv,
        lambda: check_python_module("pymavlink", "pymavlink"),
        lambda: check_python_module("tensorrt", "TensorRT Python"),
        lambda: check_python_module("torch", "PyTorch"),
        lambda: check_command_version("gst-launch-1.0", ["--version"], "GStreamer"),
        lambda: check_command_version("nvcc", ["--version"], "CUDA compiler"),
        check_power_mode,
        check_thermal,
        lambda: check_optional_endpoint("SIYI RTSP port", camera_host, 8554),
    ]
    return [check() for check in checks]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--storage-path", type=Path, default=Path.cwd())
    parser.add_argument("--write-test-mb", type=int, default=16)
    parser.add_argument("--camera-host", help="optional SIYI camera IP, e.g. 192.168.144.25")
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = run_checks(args.storage_path, args.write_test_mb, args.camera_host)
    width = max(len(result.name) for result in results)
    for result in results:
        print(f"{result.name:<{width}}  {result.status:<7} {result.detail}")

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps({"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "checks": [asdict(r) for r in results]}, indent=2),
            encoding="utf-8",
        )
        print(f"Report written to {args.json_output}")

    return 1 if any(result.status == "FAIL" for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
