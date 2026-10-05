"""Exercise the real recorder with synthetic UDP, including playback and gaps."""
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

try:
    import cv2
except ImportError:
    cv2 = None

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(cv2 is None, "OpenCV required for video round-trip")
class VideoRecordingTests(unittest.TestCase):
    def launch(self, output: Path, port: int, *extra: str):
        process = subprocess.Popen(
            [sys.executable, str(ROOT / "pc/record_video.py"),
             "--bind", "127.0.0.1", "--expected-source", "127.0.0.1",
             "--port", str(port), "--output", str(output), *extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(self.stop, process)
        return process

    @staticmethod
    def stop(process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)

    @staticmethod
    def free_port():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("127.0.0.1", 0))
            return sock.getsockname()[1]

    @staticmethod
    def send_frame(sock, port, pixel):
        ui = (b"VUI1" + bytes((1, 0x1D, 1, 2)) + struct.pack(
            ">HHHHH", 4, 7, 3, 799, 479) + bytes((3, 0)) +
            struct.pack(">II", 42, 100) + b"\x00" * 4)
        sock.sendto(ui, ("127.0.0.1", port))
        for row in range(240):
            sock.sendto(row.to_bytes(2, "little") + pixel * 400, ("127.0.0.1", port))

    def test_video_retains_clock_dimensions_color_and_ignores_ui(self):
        with tempfile.TemporaryDirectory() as temp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            output, port = Path(temp) / "clip.avi", self.free_port()
            process = self.launch(output, port, "--seconds", "1.2", "--fps", "10")
            self.assertIn("Listening", process.stdout.readline())
            started = time.monotonic()
            while process.poll() is None and time.monotonic() - started < 2:
                elapsed = time.monotonic() - started
                # A changing scene proves fresh frames reach the encoded file.
                pixel = b"\x00\xf8" if elapsed < 0.5 else b"\x1f\x00"
                self.send_frame(sock, port, pixel)
                time.sleep(0.035)
            log, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, log)
            self.assertIn("bad=0", log)
            self.assertIn("SAVED:", log)
            capture = cv2.VideoCapture(str(output))
            self.addCleanup(capture.release)
            self.assertEqual((capture.get(cv2.CAP_PROP_FRAME_WIDTH), capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), (400, 240))
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 10)
            frames = []
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                frames.append(frame)
            self.assertEqual(len(frames), 12, log)
            self.assertGreater(int(frames[0][100, 100, 2]), 240)  # red, little-endian RGB565
            self.assertLess(int(frames[0][100, 100, 0]), 10)
            self.assertGreater(int(frames[-1][100, 100, 0]), 240)  # blue, newer scene

    def test_no_stream_creates_no_video_and_existing_file_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "clip.avi"
            process = self.launch(output, self.free_port(), "--wait-timeout", "0.15")
            log, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 1, log)
            self.assertFalse(output.exists())
            output.write_bytes(b"keep this existing file")
            process = self.launch(output, self.free_port())
            log, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 2, log)
            self.assertEqual(output.read_bytes(), b"keep this existing file")

    def test_stream_loss_stops_and_finalizes_partial_video(self):
        with tempfile.TemporaryDirectory() as temp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            output, port = Path(temp) / "clip.avi", self.free_port()
            process = self.launch(output, port, "--seconds", "2", "--fps", "10", "--max-age", "0.25")
            self.assertIn("Listening", process.stdout.readline())
            for _ in range(3):
                self.send_frame(sock, port, b"\xe0\x07")
                time.sleep(0.04)
            log, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 1, log)
            self.assertIn("stopped early", log)
            capture = cv2.VideoCapture(str(output))
            self.addCleanup(capture.release)
            self.assertGreater(capture.get(cv2.CAP_PROP_FRAME_COUNT), 0)
            self.assertLess(capture.get(cv2.CAP_PROP_FRAME_COUNT), 20)
            self.assertTrue(capture.read()[0])


if __name__ == "__main__":
    unittest.main()
