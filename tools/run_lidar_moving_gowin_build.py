"""Moving-world-map revision: isolated original-profile Gowin build only.

No serial/device access, flashing or motor commands. The shared builder keeps
the original project/constraints/IP/profile unchanged and records full timing.
"""
from pathlib import Path
import run_diagnostic_gowin_build as build

if __name__ == '__main__':
    build.DEST = Path(__file__).resolve().parents[1] / 'tmp/lidar-moving-20261103/builds'
    build.DEST.mkdir(parents=True, exist_ok=True)
    raise SystemExit(build.main())
