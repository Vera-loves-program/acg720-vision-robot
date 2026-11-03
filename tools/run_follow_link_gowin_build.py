"""Build RX timing/diagnostic revision in isolation, without device actions."""
from pathlib import Path
import run_diagnostic_gowin_build as build

if __name__ == '__main__':
    build.DEST = Path(__file__).resolve().parents[1] / 'tmp/follow-link-20261103/builds'
    build.DEST.mkdir(parents=True, exist_ok=True)
    raise SystemExit(build.main())
