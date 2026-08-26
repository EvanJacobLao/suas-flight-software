# SUAS autonomy development package

This package develops the fixed-wing mission, Jetson companion, and mapping
workflow independently of unavailable aircraft hardware. The default capture
and telemetry sources are synthetic; later SIYI and Pixhawk adapters plug into
the same interfaces.

## Structure

```text
suas_autonomy/
  companion/   Orin checks, capture services, and processing benchmarks
  mapping/     Survey planning, dataset QA, replay, stitching, and ODM handoff
  mission/     ArduPlane mission geometry and SITL mission runner
  config/      SUAS rules and hardware-independent mapping defaults
  tests/       Automated regression and synthetic integration tests
```

The four small modules at the package root are compatibility entry points, so
commands given during earlier development continue to work after the
reorganization.

## Dependencies

The companion and mapping tools require Python 3, NumPy, and OpenCV. Live
MAVLink also requires `pymavlink`. On a Jetson, prefer the OpenCV/GStreamer and
CUDA packages supplied for its YUAN/NVIDIA JetPack image instead of replacing
them with generic packages before the carrier software is identified.

## 1. Validate an ArduPlane SITL mission

```bash
python3 -m suas_autonomy.autonomous_mission_sitl --dry-run
```

With ArduPlane SITL running on the secondary TCP port:

```bash
python3 -m suas_autonomy.autonomous_mission_sitl --start
```

The runner is loopback-TCP-only and cannot connect to a real Pixhawk. Real
aircraft integration requires separate onboard geofence, RC/GCS/battery
failsafes, flight termination, and safety testing.

## 2. Check Orin readiness

No camera or Pixhawk is needed:

```bash
python3 -m suas_autonomy.orin_diagnostics \
  --json-output output/orin_diagnostics.json
```

When the SIYI is connected, add `--camera-host 192.168.144.25` to check whether
its RTSP TCP port is reachable. A missing optional component is reported as
`WARN` or `SKIPPED`, not as a false hardware failure.

## 3. Capture images without hardware

Generate image/JSON pairs with synthetic frames and telemetry:

```bash
python3 -m suas_autonomy.capture_pipeline \
  --frames 10 \
  --output output/captures
```

Exercise 4K frame handling:

```bash
python3 -m suas_autonomy.capture_pipeline \
  --resolution 3840x2160 \
  --frames 10 \
  --output output/captures_4k
```

Use a recorded video:

```bash
python3 -m suas_autonomy.capture_pipeline \
  --camera video \
  --video sample.mp4 \
  --frames 10
```

Pair synthetic frames with live SITL telemetry:

```bash
python3 -m suas_autonomy.capture_pipeline \
  --telemetry mavlink \
  --mavlink tcp:127.0.0.1:5762 \
  --frames 10
```

Every capture contains a JPEG and JSON metadata with its hash, timing, GPS,
AGL altitude, aircraft attitude, gimbal placeholder, and quality measurements.

## 4. Plan an SUAS mapping survey

The default planner uses the SIYI A8 Mini's published 81-degree horizontal
field of view, 4096x2160 resolution, 60 m AGL, 18 m/s groundspeed, 75% forward
overlap, 65% side overlap, and official 2026 search boundary 1:

```bash
python3 -m suas_autonomy.mapping.planning \
  --output output/mapping/survey_plan.json
```

Override settings at the command line or edit `config/mapping_defaults.json`.
The output contains alternating survey-line endpoints and every planned image
capture coordinate. It does not create fixed-wing turn arcs; those must be
validated against the larger flight boundary and the aircraft's turn radius.

## 5. Validate, replay, and measure coverage

```bash
python3 -m suas_autonomy.mapping.dataset validate output/captures

python3 -m suas_autonomy.mapping.dataset manifest output/captures

python3 -m suas_autonomy.mapping.dataset replay output/captures --speed 0

python3 -m suas_autonomy.mapping.dataset coverage \
  output/captures \
  output/mapping/survey_plan.json
```

Validation checks pair completeness, JSON structure, SHA-256 integrity, image
decoding and dimensions, GPS/attitude, telemetry timing, duplicate sequences,
and image quality. Coverage compares actual GPS capture positions with planned
capture positions; it does not claim that image footprints or stitching are
correct.

## 6. Create a quick preview stitch

```bash
python3 -m suas_autonomy.mapping.preview_stitcher \
  output/captures \
  --output output/mapping/preview_panorama.jpg \
  --report output/mapping/stitch_report.json
```

This performs planar OpenCV stitching with pairwise ORB feature, homography,
and inlier reporting. It is only a fast visual/integration preview, not a
georeferenced competition orthomosaic.

## 7. Prepare an OpenDroneMap project

```bash
python3 -m suas_autonomy.mapping.odm_handoff \
  output/captures \
  --project-root output/odm_projects \
  --project-name suas_mapping
```

This validates and copies accepted images into ODM's expected project layout,
then writes `geo.txt` and a reviewable Docker command. AGL altitude is not
incorrectly used as MSL/WGS84 camera elevation; elevation is omitted until the
Pixhawk metadata provides the appropriate value. ODM is not installed or run
automatically.

## 8. Benchmark frame processing

```bash
python3 -m suas_autonomy.companion.benchmark \
  --frames 5 \
  --output output/orin_frame_benchmark.json
```

The benchmark measures synthetic 720p, 1080p, and 4K image-quality analysis,
JPEG encoding, and ORB feature detection. It intentionally does not claim to
measure SIYI decoding, CUDA, TensorRT, or YOLO.

## Tests

Run all tests from the repository root in an environment containing OpenCV,
NumPy, and pymavlink:

```bash
python3 -m unittest discover -s suas_autonomy/tests -p "test_*.py"
```

