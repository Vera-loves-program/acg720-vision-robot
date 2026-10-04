"""Exercise the extracted portable bundle, independently of the source directory."""
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


class PortableClientTests(unittest.TestCase):
    def test_extracted_launcher_receives_and_saves_frame(self):
        with tempfile.TemporaryDirectory(prefix="acg720-bundle-") as directory:
            temporary = Path(directory)
            archive = temporary / "client.zip"
            subprocess.run([sys.executable, str(ROOT / "tools/package_orangepi_client.py"),
                            "--output", str(archive)], check=True, capture_output=True)
            with zipfile.ZipFile(archive) as bundle:
                self.assertIsNone(bundle.testzip())
                bundle.extractall(temporary)
            client = temporary / "acg720_vision"
            manifest = json.loads((client / "manifest.json").read_text(encoding="utf-8"))
            for name, expected in manifest["sha256"].items():
                self.assertEqual(hashlib.sha256((client / name).read_bytes()).hexdigest(), expected)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(("127.0.0.1", 0))
                port = reserve.getsockname()[1]
            process = subprocess.Popen([
                sys.executable, str(client / "client.py"), "burst",
                "--bind", "127.0.0.1", "--port", str(port),
                "--expected-source", "127.0.0.1", "--duration", "1",
                "--class", "dog_plush", "--session", "test-burst",
            ], cwd=temporary, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                startup = []
                for _ in range(10):
                    line = process.stdout.readline()
                    startup.append(line)
                    if "BURST 1:" in line or not line:
                        break
                self.assertIn("BURST 1:", "".join(startup))
                deadline = time.monotonic() + 0.65
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    while time.monotonic() < deadline:
                        for row in range(240):
                            sender.sendto(row.to_bytes(2, "little") + b"\x00\xf8" * 400,
                                          ("127.0.0.1", port))
                            time.sleep(0.0003)
                output, _ = process.communicate(timeout=6)
                self.assertEqual(process.returncode, 0, "".join(startup) + output)
                images = list((client / "dataset/captures/test-burst").rglob("*.png"))
                self.assertGreater(len(images), 0)
                self.assertEqual(struct.unpack(">II", images[0].read_bytes()[16:24]), (400, 240))
                self.assertIn("SAVED dog_plush", output)
                self.assertFalse((temporary / "dataset").exists())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()
