"""Execute the real planner RTL on synthetic maps with Icarus Verilog.

No source-string check or Python planner substitute decides reachability.
Windows may use the already installed WSL Ubuntu Icarus; no installer runs.
The current Git HEAD is also simulated as an R7 negative control when it has
the old interface. Use --legacy-source to select another preserved baseline.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", choices=("auto", "native", "wsl"), default="auto")
    parser.add_argument("--wsl-distribution", default="Ubuntu-22.04")
    parser.add_argument("--legacy-source", type=Path)
    args = parser.parse_args()
    mode = args.simulator
    if mode == "auto":
        mode = "native" if shutil.which("iverilog") and shutil.which("vvp") else "wsl"
    if mode == "wsl" and (sys.platform != "win32" or not shutil.which("wsl")):
        raise SystemExit("Icarus is required: install independently or specify an available simulator.")
    output = ROOT / "tmp" / "planner-20261103"
    output.mkdir(parents=True, exist_ok=True)

    def run(words: list[str], log: Path) -> None:
        if mode == "wsl":
            drive = ROOT.drive.rstrip(":").lower()
            linux_root = f"/mnt/{drive}/{ROOT.as_posix()[3:]}"
            command = ["wsl", "-d", args.wsl_distribution, "--", "sh", "-lc",
                       f"cd {shlex.quote(linux_root)} && {shlex.join(words)}"]
        else:
            command = words
        result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, encoding="utf-8",
                                errors="replace", timeout=300, check=False)
        log.write_text(result.stdout, encoding="utf-8")
        print(result.stdout.replace("\x00", ""), end="", flush=True)
        if result.returncode:
            raise SystemExit(f"HDL command failed ({result.returncode}); see {log}")

    def simulate(source: str, name: str, legacy: bool = False) -> None:
        target = f"tmp/planner-20261103/{name}.vvp"
        command = ["iverilog", "-g2012", "-s", "tb_lidar_planner_start", "-o", target]
        if legacy:
            command.append("-DLEGACY_PLANNER")
        command.extend(["tests/tb_lidar_planner_start.v", source])
        run(command, output / f"{name}-compile.log")
        run(["vvp", "-i", target], output / f"{name}-run.log")

    source = ROOT / "src/lidar_path_planner.v"
    print(f"RTL SHA256 {hashlib.sha256(source.read_bytes()).hexdigest()}", flush=True)
    if args.legacy_source:
        baseline = args.legacy_source.read_bytes()
    else:
        result = subprocess.run(
            ["git", "-c", f"safe.directory={ROOT.as_posix()}", "show", "HEAD:src/lidar_path_planner.v"],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        baseline = result.stdout if result.returncode == 0 else b""
    if baseline and b"current_x_mm" not in baseline:
        legacy_path = output / "legacy-planner.v"
        legacy_path.write_bytes(baseline)
        print(f"Legacy control SHA256 {hashlib.sha256(baseline).hexdigest()}", flush=True)
        simulate("tmp/planner-20261103/legacy-planner.v", "legacy", True)
    else:
        print("Legacy negative control skipped: no old-interface baseline supplied.", flush=True)
    simulate("src/lidar_path_planner.v", "planner")
    print("PASS: actual HDL simulation completed; no hardware or Gowin compilation performed.")


if __name__ == "__main__":
    main()
