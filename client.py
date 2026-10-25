#!/usr/bin/env python3
"""One entry point for the Orange Pi client; accepts R4/R5 UI observations."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ACG720 Orange Pi client: FPGA UDP tools, USB human/dog and calibrated gimbal.",
        epilog="Examples: python3 client.py view; python3 client.py capture; python3 client.py burst --class dog_plush",
    )
    parser.add_argument("action", choices=("view", "capture", "burst", "probe", "detect", "gimbal", "camera-calibrate"),
                        help="Old UDP tools; gimbal=USB/browser/PWM; camera-calibrate=RAW chessboard images")
    # Parse only the action here: --help and all subsequent flags belong to
    # the selected tool, including the USB tool's independent diagnostics.
    args = parser.parse_args(sys.argv[1:2])
    extra = sys.argv[2:]
    if args.action == "gimbal":
        script, defaults = "usb_gimbal.py", []
    elif args.action == "camera-calibrate":
        script, defaults = "calibrate_usb_camera.py", []
    elif args.action == "detect":
        script = "detect_dog.py"
        defaults = ["--bind", "192.168.10.3", "--model", str(ROOT / "models/dog_plush/dog_plush_best.onnx")]
    elif args.action == "view":
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
    if args.action == 'gimbal':
        # Own one process: TERM must reach the camera/UDP/HTTP owner rather
        # than orphaning it and leaving6102/8080 occupied on restart.
        os.chdir(ROOT)
        os.execv(sys.executable,[sys.executable,str(target),*defaults,*extra])
    try:
        return subprocess.call([sys.executable, str(target), *defaults, *extra], cwd=ROOT)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
