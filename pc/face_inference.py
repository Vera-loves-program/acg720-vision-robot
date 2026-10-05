"""Official dynamic YuNet via ORT CPU; independent of FaceDetectorYN version.

Decode semantics: OpenCV modules/objdetect/src/face_detect.cpp (4.10.0).
Model/source/license are pinned in models/face_yunet/manifest.json.
"""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json


@dataclass(frozen=True)
class FaceBox:
    x0: float
    y0: float
    x1: float
    y1: float
    confidence: float
    landmarks: tuple


def preprocess(image):
    import cv2
    import numpy as np
    height, width = image.shape[:2]
    padded_height, padded_width = (height + 31) // 32 * 32, (width + 31) // 32 * 32
    padded = cv2.copyMakeBorder(image, 0, padded_height - height, 0, padded_width - width,
                               cv2.BORDER_CONSTANT, value=(0, 0, 0))
    # YuNet uses BGR 0..255, unlike the dog's RGB normalized input.
    return np.ascontiguousarray(padded.transpose(2, 0, 1)[None], dtype=np.float32)


def postprocess(outputs, image_shape, confidence=.6, iou=.3, top_k=5000):
    import cv2
    import numpy as np
    height, width = image_shape[:2]
    padded_height, padded_width = (height + 31) // 32 * 32, (width + 31) // 32 * 32
    candidates = []
    for stride in (8, 16, 32):
        rows, cols = padded_height // stride, padded_width // stride
        count = rows * cols
        arrays = {}
        for name, dimensions in (("cls", 1), ("obj", 1), ("bbox", 4), ("kps", 10)):
            array = np.asarray(outputs[f"{name}_{stride}"])
            if array.shape != (1, count, dimensions):
                raise ValueError(f"Unexpected YuNet {name}_{stride} shape {array.shape}")
            arrays[name] = array[0]
        scores = np.sqrt(np.clip(arrays["cls"][:, 0], 0, 1) * np.clip(arrays["obj"][:, 0], 0, 1))
        valid = np.isfinite(scores) & (scores >= confidence)
        valid &= np.isfinite(arrays["bbox"]).all(axis=1) & np.isfinite(arrays["kps"]).all(axis=1)
        for index in np.flatnonzero(valid):
            row, col = divmod(int(index), cols)
            delta = arrays["bbox"][index]
            cx, cy = (col + float(delta[0])) * stride, (row + float(delta[1])) * stride
            bw, bh = np.exp(np.clip(delta[2:4], -30, 30)) * stride
            x0, y0 = cx - float(bw) / 2, cy - float(bh) / 2
            points = arrays["kps"][index].reshape(5, 2).copy()
            points[:, 0] = (points[:, 0] + col) * stride
            points[:, 1] = (points[:, 1] + row) * stride
            candidates.append((x0, y0, float(bw), float(bh), float(scores[index]), points))
    if not candidates:
        return []
    # Match OpenCV's integer Rect2i NMS, then clip display coordinates.
    rectangles = [[int(x), int(y), int(w), int(h)] for x, y, w, h, _, _ in candidates]
    scores = [candidate[4] for candidate in candidates]
    indices = np.asarray(cv2.dnn.NMSBoxes(rectangles, scores, confidence, iou, top_k=top_k)).reshape(-1)
    result = []
    for index in indices:
        x, y, w, h, score, points = candidates[int(index)]
        x0, x1 = max(0, min(width, x)), max(0, min(width, x + w))
        y0, y1 = max(0, min(height, y)), max(0, min(height, y + h))
        if x0 >= x1 or y0 >= y1:
            continue
        result.append(FaceBox(x0, y0, x1, y1, score,
                              tuple((float(px), float(py)) for px, py in points)))
    return result


class FaceDetector:
    def __init__(self, model: Path, confidence=.6, iou=.3, threads=4):
        import onnxruntime as ort
        manifest = json.loads(model.with_name("manifest.json").read_text(encoding="utf-8"))
        if manifest.get("deployment_family") != "YuNet-CPU-ONNX":
            raise ValueError("Unknown face model manifest")
        if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["model_sha256"]:
            raise ValueError("YuNet checksum mismatch")
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].type != "tensor(float)" or inputs[0].shape != [1, 3, "height", "width"]:
            raise ValueError("Expected the pinned dynamic YuNet BGR input")
        self.input_name = inputs[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        if set(self.output_names) != set(manifest["output_names"]):
            raise ValueError("Unexpected YuNet outputs")
        self.confidence, self.iou = confidence, iou

    def predict(self, image):
        tensor = preprocess(image)
        values = self.session.run(None, {self.input_name: tensor})
        return postprocess(dict(zip(self.output_names, values)), image.shape, self.confidence, self.iou)


def annotate_faces(image, faces):
    import cv2
    result = image.copy()
    for face in faces:
        p0, p1 = (round(face.x0), round(face.y0)), (round(face.x1) - 1, round(face.y1) - 1)
        color = (255, 160, 40)
        cv2.rectangle(result, p0, p1, color, 2)
        cv2.putText(result, f"human {face.confidence:.2f}", (p0[0], max(15, p0[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    return result
