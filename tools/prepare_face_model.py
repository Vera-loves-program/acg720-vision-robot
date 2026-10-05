#!/usr/bin/env python3
"""Fetch the pinned official YuNet model and its license; no training."""
from pathlib import Path
import hashlib
import json
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REVISION = "47534e27c9851bb1128ccc0102f1145e27f23f98"
NAME = "face_detection_yunet_2026may.onnx"
SHA256 = "ebafce4e3c118d6554634be5c27ab333b4c047a9a8c3faf1d7cf93101c22f0f0"


def main():
    directory = ROOT / "models/face_yunet"
    directory.mkdir(parents=True, exist_ok=True)
    base = f"https://raw.githubusercontent.com/opencv/opencv_zoo/{REVISION}/models/face_detection_yunet"
    pointer = urllib.request.urlopen(f"{base}/{NAME}", timeout=30).read().decode()
    if f"oid sha256:{SHA256}" not in pointer or "size 229738" not in pointer:
        raise RuntimeError("Official Git LFS pointer does not match the pinned model")
    url = f"https://media.githubusercontent.com/media/opencv/opencv_zoo/{REVISION}/models/face_detection_yunet/{NAME}"
    data = urllib.request.urlopen(url, timeout=30).read()
    if len(data) != 229738 or hashlib.sha256(data).hexdigest() != SHA256:
        raise RuntimeError("YuNet model size/checksum mismatch")
    license_data = urllib.request.urlopen(f"{base}/LICENSE", timeout=30).read()
    if not license_data.startswith(b"MIT License"):
        raise RuntimeError("Unexpected official license")
    (directory / NAME).write_bytes(data)
    (directory / "LICENSE").write_bytes(license_data)
    manifest = dict(model_family="YuNet", model_file=NAME, model_sha256=SHA256,
                    model_bytes=len(data), source_repository="https://github.com/opencv/opencv_zoo",
                    source_revision=REVISION, source_url=url, license="MIT",
                    deployment_family="YuNet-CPU-ONNX", display_label="human",
                    meaning="face detection, no identity or full-body detection",
                    preprocessing="native BGR float32 0..255; zero pad right/bottom to multiple of 32; NCHW",
                    output_names=[f"{kind}_{stride}" for kind in ("cls", "obj", "bbox", "kps") for stride in (8, 16, 32)],
                    score_threshold=0.6, nms_threshold=0.3,
                    board_validation="pending user test")
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared {directory / NAME}; {len(data)} bytes; SHA256 verified")


if __name__ == "__main__":
    main()
