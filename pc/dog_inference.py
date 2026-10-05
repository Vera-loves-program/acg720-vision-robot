"""CPU ONNX inference for the validated single-class, raw YOLOv8 export."""
from pathlib import Path
import hashlib
import json

from detection_feedback import DogBox


def letterbox(image, size: int):
    import cv2
    import numpy as np
    height, width = image.shape[:2]
    gain = min(size / height, size / width)
    resized_width, resized_height = round(width * gain), round(height * gain)
    left = round((size - resized_width) / 2 - 0.1)
    top = round((size - resized_height) / 2 - 0.1)
    image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    image = cv2.copyMakeBorder(image, top, size - resized_height - top,
                               left, size - resized_width - left,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    tensor = np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32)
    tensor /= 255.0
    return tensor[None], gain, left, top


def postprocess(output, image_shape, gain, left, top, confidence=0.25, iou=0.45, max_det=100):
    import numpy as np
    raw = np.asarray(output)
    if raw.ndim != 3 or raw.shape[0] != 1 or raw.shape[1] != 5:
        raise ValueError(f"Expected raw one-class YOLOv8 [1,5,N], got {raw.shape}; do not use YOLOv5/RKNN decoding.")
    predictions = raw[0].T
    keep = np.isfinite(predictions).all(axis=1) & (predictions[:, 4] >= confidence)
    predictions = predictions[keep]
    if not len(predictions):
        return []
    boxes = np.empty_like(predictions[:, :4])
    boxes[:, :2] = predictions[:, :2] - predictions[:, 2:4] / 2
    boxes[:, 2:4] = predictions[:, :2] + predictions[:, 2:4] / 2
    scores = predictions[:, 4]
    valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    boxes, scores = boxes[valid], scores[valid]
    order = scores.argsort()[::-1]
    selected = []
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    while len(order) and len(selected) < max_det:
        current = order[0]
        selected.append(current)
        rest = order[1:]
        xy0 = np.maximum(boxes[current, :2], boxes[rest, :2])
        xy1 = np.minimum(boxes[current, 2:], boxes[rest, 2:])
        wh = np.maximum(xy1 - xy0, 0)
        intersection = wh[:, 0] * wh[:, 1]
        overlap = intersection / np.maximum(areas[current] + areas[rest] - intersection, 1e-9)
        order = rest[overlap <= iou]
    result = []
    height, width = image_shape[:2]
    for index in selected:
        xyxy = boxes[index].copy()
        xyxy[[0, 2]] = np.clip((xyxy[[0, 2]] - left) / gain, 0, width)
        xyxy[[1, 3]] = np.clip((xyxy[[1, 3]] - top) / gain, 0, height)
        if xyxy[0] < xyxy[2] and xyxy[1] < xyxy[3]:
            result.append(DogBox(*map(float, xyxy), float(scores[index])))
    return result


class DogDetector:
    def __init__(self, model: Path, confidence=0.25, iou=0.45, threads=4):
        import onnxruntime as ort
        manifest = json.loads(model.with_name("manifest.json").read_text(encoding="utf-8"))
        if manifest.get("deployment_family") != "YOLOv8n-CPU-ONNX" or manifest.get("classes") != {"0": "dog_plush"}:
            raise ValueError("Unknown model family/classes; prepare this model with tools/prepare_dog_model.py.")
        if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["onnx_sha256"]:
            raise ValueError("ONNX checksum mismatch")
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        size = manifest["input_size"]
        if self.input.type != "tensor(float)" or self.input.shape != [1, 3, size, size]:
            raise ValueError(f"Unexpected ONNX input {self.input.type}, {self.input.shape}")
        if len(self.session.get_outputs()) != 1 or self.session.get_outputs()[0].shape != [1, 5, 8400]:
            raise ValueError("Unexpected ONNX output; this receiver accepts only the checked 640-square raw export")
        self.size, self.confidence, self.iou = size, confidence, iou

    def predict(self, image):
        tensor, gain, left, top = letterbox(image, self.size)
        output = self.session.run(None, {self.input.name: tensor})[0]
        return postprocess(output, image.shape, gain, left, top, self.confidence, self.iou)


def annotate(image, boxes):
    import cv2
    result = image.copy()
    for box in boxes:
        p0 = (round(box.x0), round(box.y0))
        p1 = (round(box.x1) - 1, round(box.y1) - 1)
        cv2.rectangle(result, p0, p1, (40, 210, 40), 2)
        cv2.putText(result, f"dog {box.confidence:.2f}", (p0[0], max(15, p0[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 210, 40), 1, cv2.LINE_AA)
    return result
