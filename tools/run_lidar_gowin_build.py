"""2026-11-03 lidar build: identical original profile, isolated outputs only."""
from pathlib import Path
import run_diagnostic_gowin_build as build

if __name__=='__main__':
    build.DEST=Path(__file__).resolve().parents[1]/'tmp/lidar-20261103/builds'
    build.DEST.mkdir(parents=True,exist_ok=True)
    raise SystemExit(build.main())
