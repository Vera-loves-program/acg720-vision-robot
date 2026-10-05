"""YuNet input semantics, decoding, NMS, and local source integration."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from face_inference import preprocess, postprocess


class FaceGeometryTests(unittest.TestCase):
    def outputs(self):
        import numpy as np
        return {f"{kind}_{stride}": np.zeros((1, (256 // stride) * (416 // stride), channels), np.float32)
                for kind, channels in (("cls", 1), ("obj", 1), ("bbox", 4), ("kps", 10)) for stride in (8, 16, 32)}

    def test_native_bgr_and_zero_padding(self):
        import numpy as np
        image = np.zeros((240, 400, 3), np.uint8)
        image[:] = (12, 34, 56)
        tensor = preprocess(image)
        self.assertEqual(tensor.shape, (1, 3, 256, 416))
        self.assertEqual(tuple(tensor[0, :, 0, 0]), (12, 34, 56))
        self.assertEqual(float(tensor[0, :, 240:, :].sum()), 0)
        self.assertEqual(float(tensor[0, :, :, 400:].sum()), 0)

    def test_decode_nms_and_clip(self):
        import numpy as np
        outputs = self.outputs()
        for index, offset, score in ((540, .5, .9), (541, -.5, .8)):
            outputs["cls_8"][0, index, 0] = score
            outputs["obj_8"][0, index, 0] = score
            outputs["bbox_8"][0, index] = (offset, .5, np.log(10), np.log(10))
        faces = postprocess(outputs, (240, 400))
        self.assertEqual(len(faces), 1)
        for actual, expected in zip((faces[0].x0, faces[0].y0, faces[0].x1, faces[0].y1), (124, 44, 204, 124)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertAlmostEqual(faces[0].confidence, .9, places=5)
        self.assertEqual(faces[0].landmarks[0], (160, 80))
        outputs["bbox_8"][0, 540, :2] = (-30, -20)
        clipped = postprocess(outputs, (240, 400))
        self.assertTrue(all(0 <= f.x0 < f.x1 <= 400 and 0 <= f.y0 < f.y1 <= 240 for f in clipped))

    def test_empty_and_wrong_output_shape(self):
        import numpy as np
        outputs = self.outputs()
        self.assertEqual(postprocess(outputs, (240, 400)), [])
        outputs["bbox_8"] = np.zeros((1, 2, 4), np.float32)
        with self.assertRaises(ValueError):
            postprocess(outputs, (240, 400))


class CombinedIntegrationTests(unittest.TestCase):
    def test_face_in_offline_report_and_dog_only_option(self):
        try:
            import onnxruntime
        except ImportError:
            self.skipTest("Inference environment needed")
        source = ROOT / "dataset/captures/20261003_212720_481564/vera/burst001_000001.png"
        if not source.is_file() or not (ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx").is_file():
            self.skipTest("Private source/model not present")
        with tempfile.TemporaryDirectory() as directory:
            command = [sys.executable, str(ROOT / "client.py"), "detect", "--headless",
                       "--source", str(source), "--output", directory]
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report_path = Path(directory) / "burst001_000001_detections.json"
            report = json.loads(report_path.read_text())
            self.assertGreater(report["frames_with_human"], 0)
            self.assertIn("face_ms", report["detections"][0])
            result = subprocess.run(command + ["--no-face"], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(json.loads(report_path.read_text())["frames_with_human"], 0)


if __name__ == "__main__":
    unittest.main()
