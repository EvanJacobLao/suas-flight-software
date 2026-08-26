"""Compatibility entry point for the reorganized Orin diagnostics."""

from .companion.orin_diagnostics import *  # noqa: F401,F403
from .companion.orin_diagnostics import main


if __name__ == "__main__":
    raise SystemExit(main())

