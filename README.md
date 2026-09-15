# SUAS flight software

Development tools for a fixed-wing SUAS aircraft, including manual control,
SITL mission planning, companion-camera capture, mapping, and synthetic vision.

- [Autonomy tools and usage](suas_autonomy/README.md)
- [Manual control tools](manual_control/README.md)

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

## Validation scope

The GCS dashboard and synthetic detector are development tools. Passing unit
tests does not establish aircraft integration or flight readiness. Camera
captures, logs, virtual environments, and personal troubleshooting notes stay
local and are excluded from version control.
