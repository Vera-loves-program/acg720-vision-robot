#!/usr/bin/env python3
"""One entry point for the Orange Pi client; accepts R4/R5 UI observations."""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ACG720 Orange Pi client: view video, capture photos, or inspect UDP.",
        epilog="Examples: python3 client.py view; python3 client.py capture; python3 client.py burst --class dog_plush",
    )
    parser.add_argument("action", choices=("view", "capture", "burst", "probe"),
                        help="view=video window; capture=5-second photo bursts; burst=one burst without GUI; probe=UDP statistics")
    args, extra = parser.parse_known_args()
    if args.action == "view":
        script = "udp_video_viewer.py"
        defaults = ["--bind", "192.168.10.3"]
    elif args.action in ("capture", "burst"):
        script = "capture_dataset.py"
        defaults = ["--bind", "192.168.10.3", "--fps", "3", "--duration", "5",
                    "--fpga-profile", "unconfirmed", "--output", str(ROOT / "dataset" / "captures")]
        if args.action == "burst":
            defaults.append("--burst")
    else:
        script = "udp_probe.py"
        defaults = ["--bind", "192.168.10.3", "--seconds", "10"]
    target = ROOT / "pc" / script
    if not target.is_file():
        print(f"Missing client file: {target}. Extract the complete acg720-orangepi.zip.", file=sys.stderr)
        return 1
    try:
        return subprocess.call([sys.executable, str(target), *defaults, *extra], cwd=ROOT)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
