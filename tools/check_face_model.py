#!/usr/bin/env python3
"""Local YuNet decode comparison against OpenCV's official implementation."""
from dataclasses import asdict
from pathlib import Path
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))


def main():
    import cv2
    import numpy as np
    from face_inference import FaceDetector
    model = ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx"
    detector = FaceDetector(model)
    reference = cv2.FaceDetectorYN.create(str(model), "", (400, 240), .6, .3, 5000)
    sources = sorted((ROOT / "dataset/captures/20261003_212720_481564/vera").glob("*.png"))
    if not sources:
        raise RuntimeError("Original local face photos not present")
    checks, worst_error = [], 0.0
    for path in sources:
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"Cannot read {path}")
        started = time.perf_counter()
        actual = detector.predict(image)
        milliseconds = (time.perf_counter() - started) * 1000
        reference.setInputSize((image.shape[1], image.shape[0]))
        expected = reference.detect(image)[1]
        expected = [] if expected is None else expected
        if len(actual) != len(expected):
            raise RuntimeError(f"YuNet reference count mismatch {path.name}: {len(actual)} != {len(expected)}")
        for box, face in zip(actual, expected):
            height, width = image.shape[:2]
            expected_box = [np.clip(face[0], 0, width), np.clip(face[1], 0, height),
                            np.clip(face[0] + face[2], 0, width), np.clip(face[1] + face[3], 0, height)]
            error = max(abs(a - float(b)) for a, b in zip((box.x0, box.y0, box.x1, box.y1), expected_box))
            worst_error = max(worst_error, error)
            if error > .05 or abs(box.confidence - float(face[14])) > .0001:
                raise RuntimeError(f"YuNet box/score mismatch {path.name}")
            if np.max(np.abs(np.asarray(box.landmarks) - face[4:14].reshape(5, 2))) > .05:
                raise RuntimeError(f"YuNet landmarks mismatch {path.name}")
        checks.append(dict(source=path.name, faces=[asdict(box) for box in actual], inference_ms=milliseconds))
    report = dict(scope="Local x86 CPU implementation comparison; no accuracy or Orange Pi FPS claim",
                  opencv=cv2.__version__, images=len(checks), frames_with_face=sum(bool(c["faces"]) for c in checks),
                  max_coordinate_error_pixels=worst_error, checks=checks)
    output = ROOT / "dataset/model_validation/yunet_validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"PASS {len(checks)} original images; face_frames={report['frames_with_face']}; worst_error={worst_error:.6f}px; {output}")


if __name__ == "__main__":
    main()
