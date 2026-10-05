"""Meaningful protocol, YOLOv8 geometry, and live UDP inference checks."""
import binascii
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from detection_feedback import DogBox, encode_feedback, decode_feedback
from dog_inference import letterbox, postprocess


class FeedbackTests(unittest.TestCase):
    def test_roundtrip_edges_and_wrap(self):
        packet = encode_feedback(65536, DogBox(300.2, 100.1, 400, 240, 0.875),
                                 age_ms=100, inference_ms=25, frame_number=123)
        result = decode_feedback(packet)
        self.assertEqual(len(packet), 32)
        self.assertEqual(result["box"], (300, 100, 400, 240))
        self.assertEqual(result["sequence"], 0)
        self.assertEqual(result["frame_number"], 123)
        self.assertEqual(result["confidence"], .875)

    def test_no_dog_is_different_from_stale(self):
        self.assertTrue(decode_feedback(encode_feedback(1))["result_valid"])
        stale = decode_feedback(encode_feedback(2, DogBox(1, 2, 3, 4, .8), age_ms=1501))
        self.assertFalse(stale["result_valid"])
        self.assertFalse(stale["box_valid"])
        self.assertEqual(stale["box"], (0, 0, 0, 0))

    def test_corruption_bad_flags_and_geometry_rejected(self):
        original = encode_feedback(1, DogBox(10, 20, 30, 40, .9))
        for offset in range(32):
            corrupt = bytearray(original)
            corrupt[offset] ^= 1
            self.assertIsNone(decode_feedback(corrupt))
        for offset, data in ((5, b"\x01"), (18, b"\x01\x91"), (10, b"\x01\x8f")):
            corrupt = bytearray(original)
            corrupt[offset:offset + len(data)] = data
            corrupt[30:] = binascii.crc_hqx(corrupt[:30], 0xffff).to_bytes(2, "big")
            self.assertIsNone(decode_feedback(corrupt))


class GeometryTests(unittest.TestCase):
    def test_letterbox_rgb_and_padding(self):
        import numpy as np
        image = np.zeros((240, 400, 3), dtype=np.uint8)
        image[:] = (0, 0, 255)
        tensor, gain, left, top = letterbox(image, 640)
        self.assertEqual(tensor.shape, (1, 3, 640, 640))
        self.assertEqual((gain, left, top), (1.6, 0, 128))
        self.assertAlmostEqual(float(tensor[0, 0, 200, 200]), 1)
        self.assertAlmostEqual(float(tensor[0, 0, 0, 0]), 114 / 255, places=6)
        self.assertEqual(float(tensor[0, 2, 200, 200]), 0)

    def test_nms_and_undo_letterbox(self):
        import numpy as np
        # Target source box (100,50)-(200,150), mapped into 640 square.
        output = np.array([[[240, 241, 500], [288, 288, 300], [160, 160, 30],
                            [160, 160, 30], [.9, .8, .1]]], dtype=np.float32)
        boxes = postprocess(output, (240, 400), 1.6, 0, 128)
        self.assertEqual(len(boxes), 1)
        for actual, expected in zip((boxes[0].x0, boxes[0].y0, boxes[0].x1, boxes[0].y1), (100, 50, 200, 150)):
            self.assertAlmostEqual(actual, expected, places=4)
        with self.assertRaises(ValueError):
            postprocess(np.zeros((1, 100, 6)), (240, 400), 1.6, 0, 128)


class LiveTests(unittest.TestCase):
    def test_cpu_receiver_feedback_and_clear_on_exit(self):
        model = ROOT / "models/dog_plush/dog_plush_best.onnx"
        if not model.is_file():
            self.skipTest("Prepared private model not present")
        try:
            import onnxruntime
        except ImportError:
            self.skipTest("Run live model check in inference environment")
        face_present = (ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx").is_file()
        with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as monitor:
            monitor.bind(("127.0.0.1", 0))
            monitor.settimeout(.2)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(("127.0.0.1", 0))
                port = reserve.getsockname()[1]
            report = Path(directory) / "report.json"
            process = subprocess.Popen([sys.executable, str(ROOT / "pc/detect_dog.py"), "--headless",
                "--bind", "127.0.0.1", "--port", str(port), "--expected-source", "127.0.0.1",
                "--feedback-ip", "127.0.0.1", "--feedback-port", str(monitor.getsockname()[1]),
                "--seconds", "3", "--report", str(report), *([] if face_present else ["--no-face"])], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            packets = []
            try:
                lines = []
                for _ in range(10):
                    line = process.stdout.readline()
                    lines.append(line)
                    if "Listening " in line or not line:
                        break
                self.assertIn("Listening ", "".join(lines))
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for _ in range(5):
                        for row in range(240):
                            sender.sendto(row.to_bytes(2, "little") + b"\x00\xf8" * 400, ("127.0.0.1", port))
                            time.sleep(.0002)
                        time.sleep(.1)
                output, _ = process.communicate(timeout=8)
                self.assertEqual(process.returncode, 0, "".join(lines) + output)
                while True:
                    try:
                        packets.append(decode_feedback(monitor.recv(100)))
                    except socket.timeout:
                        break
                self.assertTrue(all(p is not None for p in packets))
                self.assertTrue(any(p["result_valid"] for p in packets))
                self.assertFalse(packets[-1]["result_valid"])
                data = json.loads(report.read_text())
                self.assertGreater(data["inferred"], 0)
                self.assertEqual(data["receiver"]["bad_packets"], 0)
                if face_present:
                    self.assertTrue(any(row["face_ms"] > 0 for row in data["detections"]))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()
