# SUAS flight software

Development tools for a fixed-wing SUAS aircraft, including manual control,
SITL mission planning, companion-camera capture, mapping, and synthetic vision.

- [Autonomy tools and usage](suas_autonomy/README.md)
- [Manual control tools](manual_control/README.md)
- [Jetson manual-flight telemetry monitor](manual_control/FLIGHT_MONITOR.md) — RC-piloted FBWA, wireless laptop display, onboard logs, and hardware-free demo.

## Development setup

Use Python 3.11 or newer. From the repository root:

```bash
python -m venv .venv
```

Activate the environment:

```powershell
# Windows PowerShell
.venv\Scripts\Activate.ps1
```

```bash
# Linux
source .venv/bin/activate
```

Then install and test:

```bash
python -m pip install -e ".[dev]"
python -m pytest
```

These dependency constraints target laptop and simulation development. For
Jetson deployment, follow the carrier vendor's supported Python, OpenCV, and
JetPack configuration; see the autonomy guide before installing packages.

## Current implementation status

| Area | Available code | Limits |
| --- | --- | --- |
| Manual control | Keyboard/controller RC overrides and telemetry-based trajectory display for ArduPlane SITL | Live SITL execution and controller configuration still need verification |
| Mission development | Mission validation, loopback-only SITL upload/start runner, competition-state logic | Latest checks do not establish end-to-end autonomous operation |
| Capture and mapping | Synthetic/video/webcam capture, synthetic or MAVLink telemetry, survey planning, dataset validation, preview stitching, ODM handoff | Default telemetry is synthetic; preview stitching is not a georeferenced orthomosaic; ODM is not run automatically |
| Ground station | Synthetic/replay dashboard and a separate receive-only MAVLink monitor with logging | Hardware integration remains unverified |
| Vision | Synthetic target tests and pretrained YOLO webcam prototype | No verified SUAS fine-tuning or real aerial detection results |

## YOLO webcam prototype

[object_detection.py](suas_autonomy/vision/object_detection.py) reads webcam 0,
runs pretrained YOLO inference, prints class names/confidence, and displays
annotated frames. Press **Q** in the video window to quit. It uses pretrained
classes, not a mannequin/tent model fine-tuned on the local SUAS dataset.

The core package uses headless OpenCV, which does not support display windows.
Use a separate GUI-enabled environment for this prototype. From the repository
root on Windows:

```powershell
python -m venv .venv-vision
.\.venv-vision\Scripts\python.exe -m pip install ultralytics opencv-python
.\.venv-vision\Scripts\python.exe suas_autonomy/vision/object_detection.py
```

Before running, obtain pretrained `yolo26n.pt` weights and place them at
`models/yolo26n.pt`. Weights are not included in Git. Run from the repository
root because the script uses a working-directory-relative model path. These
optional dependencies are not pinned. Local import checks used Ultralytics
8.4.183 and OpenCV 4.14.0; webcam inference was not executed during the review.

## SUAS dataset preparation (work in progress)

The local dataset lives under `datasets/suas_v1/`, with image/label folders
for `train`, `valid`, and `test`, and classes `mannequin` and `tent`.
The dataset is not included in Git. A local file-count check found:

| Split | Images | Label files |
| --- | ---: | ---: |
| Train | 5,172 | 5,172 |
| Validation | 639 | 639 |
| Test | 642 | 642 |

The export records [Roboflow project provenance](https://app.roboflow.com/evan-jacob-lao/suas_2026-v2-iorpl/1)
and CC BY 4.0 licensing in its YAML metadata. Preserve the attribution when
using the dataset. The source YAML root path is specific to the local machine.

[prepare_detection_dataset.py](suas_autonomy/vision/prepare_detection_dataset.py)
preserves existing five-field bounding-box rows and converts polygon rows to
enclosing boxes under `datasets/suas_v1_detection/`. It is unfinished: image
copying/linking and derived dataset YAML creation are missing. Running it
writes derived labels, not a complete training dataset or a trained model.
Both vision scripts execute at module level; do not import them as libraries.

Remaining work includes completing and validating the derived dataset,
checking overlays and split leakage, then adding training and independent
evaluation. No SUAS-trained checkpoint or verified training results are provided.

## Verification and local files

On 4 November 2026, the existing suite passed **27 tests** in the local Windows
environment. Both new vision scripts passed syntax checks, and OpenCV and
Ultralytics imported successfully. This does not verify webcam inference,
dataset conversion correctness, live SITL, Jetson performance, or aircraft use.
The mission runner's `--dry-run` also validated successfully without a simulator
or aircraft connection.

Datasets, weights, virtual environments, captures, generated outputs, logs,
and personal handoff notes are kept local and ignored by Git.

The GCS dashboard and synthetic detector are development tools. Passing unit
tests does not establish aircraft integration or flight readiness. Camera
captures, logs, virtual environments, and personal troubleshooting notes stay
local and are excluded from version control.
