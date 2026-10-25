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
    "pc/usb_camera.py", "pc/usb_gimbal.py", "pc/usb_gimbal_ui.html",
    "pc/gimbal_control.py", "pc/target_tracking.py", "pc/gimbal_image_calibration.py", "pc/calibrate_usb_camera.py",
    "pc/robot_follow.py", "pc/visual_task.py", "tools/install_orangepi_poweroff.sh", "tools/fix_orangepi_follow_network.py", "tools/start_fpga_follow.sh", "tools/install_fpga_follow_service.sh",
    "docs/六题演示与修复验收_20261025.md", "docs/视觉雷达协议_v20.md",
    "docs/上层135度与90度下限_v21.md",
    "docs/香橙派v22人脸与小狗测试.md",
    "docs/香橙派与FPGA跟随联调_20261025.md", "docs/FPGA视觉跟随通信协议_20261025.md",
    "config/gimbal.example.json",
    "tools/fix_orangepi_apt.py",
)
GUIDE = "docs/香橙派v22人脸与小狗测试.md"
PREFIX = "acg720_vision"


def portable_entry(name: str, data: bytes) -> bytes:
    """Shell scripts must run on Linux even when packaged on Windows."""
    if name.endswith(".sh"):
        return data.decode("utf-8-sig").replace("\r\n", "\n").encode("utf-8")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT.parent / "deliverables/acg720-orangepi.zip")
    parser.add_argument("--code-only", action="store_true", help="Preserve the already working field models; package code/docs only")
    args = parser.parse_args()
    entries = {name: portable_entry(name, (ROOT / name).read_bytes()) for name in FILES}
    model = ROOT / "models/dog_plush/dog_plush_best.onnx"
    if model.is_file() and not args.code_only:
        for name in ("models/dog_plush/dog_plush_best.onnx", "models/dog_plush/manifest.json"):
            entries[name] = (ROOT / name).read_bytes()
    face = ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx"
    if face.is_file() and not args.code_only:
        for name in ("models/face_yunet/face_detection_yunet_2026may.onnx", "models/face_yunet/manifest.json", "models/face_yunet/LICENSE"):
            entries[name] = (ROOT / name).read_bytes()
    guide_text=(ROOT / GUIDE).read_text(encoding='utf-8')
    guide_text=guide_text.replace('(六题演示与修复验收_20261025.md)', '(docs/六题演示与修复验收_20261025.md)')
    guide_text=guide_text.replace('(上层135度与90度下限_v21.md)', '(docs/上层135度与90度下限_v21.md)')
    entries["README.md"] = guide_text.encode('utf-8')
    manifest = {"client_bundle_version": 22, "bundle_revision": 2, "shell_line_endings": "LF", "minimum_python": "3.10",
                "dog_model_included": model.is_file() and not args.code_only,
                "face_model_included": face.is_file() and not args.code_only,
                "fpga_profile": "core-r7", "entry_point": "client.py",
                "usb_camera_and_gimbal": True, "default_pwm_backend": "mock",
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
