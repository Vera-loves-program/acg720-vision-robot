"""Host protocol and displayed-font regressions; no FPGA compilation."""
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from udp_probe import VideoProbe


def payload(row: int, order: str = "big") -> bytes:
    return row.to_bytes(2, order) + b"\xf8\x00" * 400


class HostToolsTests(unittest.TestCase):
    def test_complete_frame_and_missing_row(self) -> None:
        probe = VideoProbe("192.168.10.2")
        for row in range(240):
            probe.consume(payload(row), "192.168.10.2")
        self.assertEqual(probe.complete, 1)
        for row in range(240):
            if row != 73:
                probe.consume(payload(row), "192.168.10.2")
        probe.consume(payload(0), "192.168.10.2")
        self.assertEqual((probe.complete, probe.incomplete), (1, 1))

    def test_swapped_header_and_invalid_packets(self) -> None:
        probe = VideoProbe("192.168.10.2")
        probe.consume(payload(0), "192.168.10.99")
        probe.consume(b"short", "192.168.10.2")
        for row in range(240):
            probe.consume(payload(row, "little"), "192.168.10.2")
        self.assertEqual((probe.complete, probe.other_source, probe.bad), (1, 1, 1))
        self.assertEqual(probe.orders["little"], 239)

    def test_real_loopback_socket(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        process = subprocess.Popen(
            [sys.executable, str(ROOT / "pc" / "udp_probe.py"),
             "--bind", "127.0.0.1", "--port", str(port),
             "--expected-source", "127.0.0.1", "--seconds", "1.5"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            self.assertIn("Listening", process.stdout.readline())
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                for row in range(240):
                    sender.sendto(payload(row), ("127.0.0.1", port))
                    time.sleep(0.0005)
            output, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, output)
            self.assertIn("Complete frames: 1", output)
            self.assertIn("PASS:", output)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    def test_font_top_middle_bottom_rows(self) -> None:
        source = (ROOT / "src" / "lcd1024_vision_ui.v").read_text(encoding="utf-8")
        bit_index = re.search(r"font\[(\d+)-\(glyph_x\*8\)([+-])glyph_y\]", source)
        self.assertIsNotNone(bit_index)
        base = int(bit_index[1])
        direction = 1 if bit_index[2] == "+" else -1
        glyphs = dict(re.findall(r'"(.)":glyph=40\'h([0-9a-fA-F]{10})', source))

        def row(character: str, y: int) -> str:
            bits = int(glyphs[character], 16)
            return "".join(str((bits >> (base - 8*x + direction*y)) & 1)
                           for x in range(5))

        self.assertEqual(row("A", 0), "01110")
        self.assertEqual(row("A", 4), "11111")
        self.assertEqual(row("A", 6), "10001")
        self.assertEqual(row("V", 0), "10001")
        self.assertEqual(row("V", 6), "00100")
        self.assertEqual(row("E", 0), "11111")
        self.assertEqual(row("E", 6), "11111")


if __name__ == "__main__":
    unittest.main()
