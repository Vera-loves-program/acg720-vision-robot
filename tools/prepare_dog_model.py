#!/usr/bin/env python3
"""Verify and export the teammate's existing YOLOv8n; never train/download weights."""
import argparse
from dataclasses import asdict
from pathlib import Path
import hashlib
import json
import os
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
EXPECTED_MODEL_SHA256 = "7b463d31c338fc6e983572d4c578f8ea6a812c7f39e6dda54dee7867a8ef328e"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "models/dog_plush")
    args = parser.parse_args()
    source = args.package / "dog_plush_best.pt"
    source_manifest = json.loads((args.package / "manifest.json").read_text(encoding="utf-8"))
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != EXPECTED_MODEL_SHA256 or source_manifest["model_sha256"] != digest:
        parser.error("Unexpected teammate model checksum")
    if source_manifest["model_family"] != "YOLOv8n" or source_manifest["classes"] != {"0": "dog_plush"}:
        parser.error("Unexpected model family/classes")
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / "dog_plush_best.pt"
    shutil.copy2(source, target)
    config_dir = ROOT / "dataset/model_validation/yolo_config"
    config_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(config_dir))
    import torch
    torch.set_num_threads(4)
    from ultralytics import YOLO
    import cv2
    import onnx
    import ultralytics
    model = YOLO(str(target))
    if model.names != {0: "dog_plush"} or model.task != "detect":
        parser.error("Loaded checkpoint task/classes differ from the supplied manifest")
    onnx_path = Path(model.export(format="onnx", imgsz=640, batch=1, dynamic=False,
                                  simplify=False, opset=12, nms=False, device="cpu"))
    onnx.checker.check_model(onnx.load(str(onnx_path)))
    manifest = dict(source_manifest,
                    deployment_family="YOLOv8n-CPU-ONNX", onnx_file=onnx_path.name,
                    onnx_sha256=hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
                    export=dict(ultralytics=ultralytics.__version__, torch=torch.__version__,
                                onnx=onnx.__version__, opset=12, nms=False, dynamic=False,
                                half=False, input_shape=[1, 3, 640, 640], output_shape=[1, 5, 8400]),
                    preprocessing="BGR->RGB; square letterbox 114; float32/255; NCHW",
                    postprocessing="xywh plus one class probability; class NMS; undo letterbox",
                    board_validation="not performed", npu_validation="not performed")
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    from dog_inference import DogDetector, annotate
    detector = DogDetector(onnx_path, threads=4)
    report_dir = ROOT / "dataset/model_validation"
    report_dir.mkdir(parents=True, exist_ok=True)
    checks = []
    for image_path in sorted((args.package / "examples/images").glob("*.png")):
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"Cannot read {image_path}")
        pt = model.predict(image, imgsz=640, rect=False, conf=0.25, iou=0.45,
                           max_det=100, device="cpu", verbose=False)[0]
        started = time.perf_counter()
        boxes = detector.predict(image)
        elapsed_ms = (time.perf_counter() - started) * 1000
        pt_boxes = pt.boxes.xyxy.cpu().numpy()
        pt_scores = pt.boxes.conf.cpu().numpy()
        if len(pt_boxes) != len(boxes):
            raise RuntimeError(f"ONNX/PT count mismatch: {image_path.name}")
        max_error = 0.0
        for actual, expected, score in zip(boxes, pt_boxes, pt_scores):
            error = max(abs(a - float(b)) for a, b in zip(
                (actual.x0, actual.y0, actual.x1, actual.y1), expected))
            max_error = max(max_error, error)
            if error > 0.5 or abs(actual.confidence - float(score)) > 0.001:
                raise RuntimeError(f"ONNX/PT coordinate/confidence mismatch: {image_path.name}")
        if not cv2.imwrite(str(report_dir / f"{image_path.stem}_onnx.jpg"), annotate(image, boxes)):
            raise RuntimeError("Cannot write annotated image")
        checks.append(dict(source=image_path.name, dimensions=list(image.shape[:2][::-1]),
                           detections=[asdict(b) for b in boxes], pt_count=len(pt_boxes),
                           max_coordinate_error_pixels=max_error, inference_ms=elapsed_ms))
        print(f"PASS: {image_path.name}, boxes={len(boxes)}, ONNX={elapsed_ms:.1f}ms, error={max_error:.6f}px", flush=True)
    if len(checks) != 3:
        raise RuntimeError("Expected the package's three sample images")
    report = dict(scope="Local x86_64 CPU: model hash, ONNX checker, three PT/ONNX comparisons; no board test",
                  checks=checks, manifest=manifest)
    (report_dir / "export_validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Prepared: {onnx_path}; verification: {report_dir / 'export_validation.json'}")


if __name__ == "__main__":
    main()
