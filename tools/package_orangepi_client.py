#!/usr/bin/env python3
"""Build the complete portable Orange Pi client without FPGA/vendor files."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
FILES = (
    "pc/udp_probe.py", "pc/udp_video_viewer.py", "pc/capture_dataset.py",
    "pc/video_stream.py", "pc/ui_telemetry.py", "pc/requirements.txt",
)
GUIDE = "docs/香橙派解压配置IP与收图指南.md"
PREFIX = "acg720_vision_client_v2"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT.parent / "deliverables/acg720-orangepi-client-v2-20261003.zip")
    args = parser.parse_args()
    entries = {name: (ROOT / name).read_bytes() for name in FILES}
    entries["香橙派操作指南.md"] = (ROOT / GUIDE).read_bytes()
    interaction_guide = ROOT / "docs/UI分区与LCD缩放R4.md"
    if interaction_guide.exists():
        entries["UI分区与LCD缩放R4.md"] = interaction_guide.read_bytes()
    manifest = {"client_bundle_version": 2, "minimum_python": "3.10",
                "root_folder": PREFIX,
                "sha256": {name: hashlib.sha256(data).hexdigest() for name, data in entries.items()}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in entries.items():
            archive.writestr(f"{PREFIX}/{name}", data)
        archive.writestr(f"{PREFIX}/manifest.json",
                         json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"Created {args.output} ({args.output.stat().st_size} bytes)")
    print("Included all six client files, guide and SHA256 manifest; no photos or virtual environment.")


if __name__ == "__main__":
    main()
