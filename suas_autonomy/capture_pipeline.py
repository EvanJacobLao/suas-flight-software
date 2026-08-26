"""Compatibility entry point for the reorganized capture pipeline."""

from .companion.capture_pipeline import *  # noqa: F401,F403
from .companion.capture_pipeline import main


if __name__ == "__main__":
    raise SystemExit(main())

