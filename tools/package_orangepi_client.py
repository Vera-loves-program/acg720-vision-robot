#!/usr/bin/env python3
"""Build the complete portable Orange Pi client without FPGA/vendor files."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
FILES = (
    "client.py",
    "pc/udp_probe.py", "pc/udp_video_viewer.py", "pc/capture_dataset.py",
    "pc/video_stream.py", "pc/ui_telemetry.py", "pc/highgui_window.py", "pc/requirements.txt",
    "pc/detect_dog.py", "pc/dog_inference.py", "pc/detection_feedback.py", "pc/requirements-detection.txt",
    "pc/face_inference.py",
    "tools/fix_orangepi_apt.py",
)
GUIDE = "docs/香橙派使用说明.md"
PREFIX = "acg720_vision"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT.parent / "deliverables/acg720-orangepi.zip")
    args = parser.parse_args()
    entries = {name: (ROOT / name).read_bytes() for name in FILES}
    model = ROOT / "models/dog_plush/dog_plush_best.onnx"
    if model.is_file():
        for name in ("models/dog_plush/dog_plush_best.onnx", "models/dog_plush/manifest.json"):
            entries[name] = (ROOT / name).read_bytes()
    face = ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx"
    if face.is_file():
        for name in ("models/face_yunet/face_detection_yunet_2026may.onnx", "models/face_yunet/manifest.json", "models/face_yunet/LICENSE"):
            entries[name] = (ROOT / name).read_bytes()
    entries["README.md"] = (ROOT / GUIDE).read_bytes()
    manifest = {"client_bundle_version": 8, "minimum_python": "3.10",
                "dog_model_included": model.is_file(),
                "face_model_included": face.is_file(),
                "fpga_profile": "R4/R5", "entry_point": "client.py",
                "root_folder": PREFIX,
                "sha256": {name: hashlib.sha256(data).hexdigest() for name, data in entries.items()}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in entries.items():
            archive.writestr(f"{PREFIX}/{name}", data)
        archive.writestr(f"{PREFIX}/manifest.json",
                         json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"Created {args.output} ({args.output.stat().st_size} bytes)")
    print("Fixed directory acg720_vision; one entry point client.py; one guide README.md.")


if __name__ == "__main__":
    main()
