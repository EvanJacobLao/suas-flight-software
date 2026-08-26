"""Compatibility entry point for the reorganized mission runner."""

from .mission.autonomous_mission_sitl import *  # noqa: F401,F403
from .mission.autonomous_mission_sitl import main


if __name__ == "__main__":
    raise SystemExit(main())

