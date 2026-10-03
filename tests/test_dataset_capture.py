"""Complete-frame, capture scheduling and lossless PNG host regressions."""
from pathlib import Path
import json
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from capture_dataset import BurstSchedule, Receiver, SessionWriter, build_parser, encode_png
from ui_telemetry import parse_ui_telemetry
from video_stream import CompleteFrame, FrameAssembler, HEIGHT, WIDTH


def row_payload(row: int, order: str = "big", pixel: bytes = b"\xf8\x00") -> bytes:
    return row.to_bytes(2, order) + pixel * WIDTH


def complete_frame(number: int, at: float, started_at: float | None = None) -> CompleteFrame:
    return CompleteFrame(b"\xf8\x00" * WIDTH * HEIGHT, "192.168.10.2", number,
                         at if started_at is None else started_at, at, "2026-10-03T00:00:00+00:00")


def ui_payload(capture: int = 7, uptime: int = 100, sequence: int = 42) -> bytes:
    return (b"VUI1" + bytes((1, 0x1D, 1, 2)) + struct.pack(">HHHHH", 4, capture, 3, 799, 479) +
            bytes((3, 0)) + struct.pack(">II", sequence, uptime) + b"\x00" * 4)


def decode_png_pixels(png: bytes) -> tuple[tuple[int, int], bytes]:
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    offset, compressed, dimensions = 8, b"", (0, 0)
    while offset < len(png):
        size = struct.unpack(">I", png[offset:offset + 4])[0]
        kind = png[offset + 4:offset + 8]
        data = png[offset + 8:offset + 8 + size]
        expected_crc = struct.unpack(">I", png[offset + 8 + size:offset + 12 + size])[0]
        assert expected_crc == zlib.crc32(kind + data) & 0xFFFFFFFF
        if kind == b"IHDR":
            dimensions = struct.unpack(">II", data[:8])
            assert data[8:] == bytes((8, 2, 0, 0, 0))
        elif kind == b"IDAT":
            compressed += data
        offset += size + 12
    raw = zlib.decompress(compressed)
    width, height = dimensions
    stride = width * 3 + 1
    assert len(raw) == stride * height
    assert all(raw[y * stride] == 0 for y in range(height))
    return dimensions, b"".join(raw[y * stride + 1:(y + 1) * stride] for y in range(height))


class DatasetCaptureTests(unittest.TestCase):
    def test_default_auto_detects_both_transports_without_locking_on_row_zero(self) -> None:
        args = build_parser().parse_args([])
        self.assertEqual((args.row_byte_order, args.byte_order), ("auto", "auto"))
        for order in ("big", "little"):
            assembler = FrameAssembler()
            self.assertIsNone(assembler.consume(row_payload(0, order), "192.168.10.2", 0))
            self.assertIsNone(assembler.detected_row_byte_order)
            frame = None
            for row in range(1, HEIGHT):
                frame = assembler.consume(row_payload(row, order), "192.168.10.2", row / 10000)
            self.assertIsNotNone(frame)
            self.assertEqual(frame.row_byte_order, order)
            self.assertEqual(assembler.detected_row_byte_order, order)
            self.assertEqual(assembler.snapshot_stats()["bad_packets"], 0)

    def test_wrong_forced_order_reproduces_only_row_zero_valid_and_reports_conflicts(self) -> None:
        assembler = FrameAssembler(row_byte_order="big")
        for row in range(HEIGHT):
            self.assertIsNone(assembler.consume(row_payload(row, "little"),
                                               "192.168.10.2", row / 10000))
        stats = assembler.snapshot_stats()
        self.assertEqual((stats["valid_rows"], stats["complete_frames"]), (1, 0))
        self.assertEqual((stats["bad_packets"], stats["invalid_row_headers"],
                          stats["row_order_conflicts"]), (239, 239, 239))
        self.assertEqual(stats["bad_packet_sizes"], 0)

    def test_auto_little_transport_saves_correct_pixels_and_resolved_metadata(self) -> None:
        assembler = FrameAssembler()
        for row in range(HEIGHT):
            frame = assembler.consume(row_payload(row, "little", b"\x00\xf8"),
                                      "192.168.10.2", row / 10000)
        with tempfile.TemporaryDirectory(prefix=".capture_test_", dir=ROOT) as temp:
            config = {"pixel_byte_order": "auto", "fpga_profile_manual": "unconfirmed",
                      "filter_state_manual": "unknown", "pipeline_note": "auto regression"}
            writer = SessionWriter(Path(temp) / "session", config)
            target = writer.save(frame, BurstSchedule(0, 5, 3, "vera", 1), assembler.snapshot_stats())
            writer.close()
            dimensions, rgb = decode_png_pixels(target.read_bytes())
            self.assertEqual(dimensions, (WIDTH, HEIGHT))
            self.assertEqual(rgb, b"\xff\x00\x00" * WIDTH * HEIGHT)
            record = json.loads((target.parent.parent / "metadata.jsonl").read_text())
            self.assertEqual((record["row_byte_order_resolved"],
                              record["pixel_byte_order_resolved"]), ("little", "little"))

    def test_ui_telemetry_validation_and_capture_counter_baseline(self) -> None:
        ui = parse_ui_telemetry(ui_payload())
        self.assertIsNotNone(ui)
        self.assertTrue(ui.gaussian_on)
        self.assertTrue(ui.touch_ready)
        self.assertTrue(ui.touch_identified)
        self.assertEqual((ui.camera_x, ui.camera_y, ui.capture_counter), (799, 479, 7))
        for index, bad_value in ((4, 2), (5, 0x80), (6, 2), (7, 6), (19, 1), (31, 1)):
            bad = bytearray(ui_payload())
            bad[index] = bad_value
            self.assertIsNone(parse_ui_telemetry(bytes(bad)))
        bad = bytearray(ui_payload())
        bad[14:16] = (800).to_bytes(2, "big")
        self.assertIsNone(parse_ui_telemetry(bytes(bad)))
        self.assertIsNone(parse_ui_telemetry(ui_payload()[:-1]))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            receiver = Receiver(sock, FrameAssembler())
            self.assertTrue(receiver.consume_telemetry(ui_payload(7), "192.168.10.2", 1))
            self.assertTrue(receiver.capture_requests.empty())  # Old event is baseline.
            receiver.consume_telemetry(ui_payload(7, sequence=43), "192.168.10.2", 1.1)
            self.assertTrue(receiver.capture_requests.empty())
            receiver.consume_telemetry(ui_payload(8, sequence=44), "192.168.10.2", 1.2)
            requested_ui, requested_at = receiver.capture_requests.get_nowait()
            self.assertEqual((requested_ui.capture_counter, requested_at), (8, 1.2))
            receiver.consume_telemetry(ui_payload(0, uptime=0, sequence=0), "192.168.10.2", 1.3)
            self.assertTrue(receiver.capture_requests.empty())  # Board restart resets baseline.
            receiver.consume_telemetry(b"VUI1short", "192.168.10.2", 1.4)
            receiver.consume_telemetry(ui_payload(9), "192.168.10.99", 1.5)
            self.assertEqual(receiver.stats()["bad_packets"], 0)
            self.assertEqual(receiver.stats()["ui_bad_packets"], 1)
            self.assertEqual(receiver.stats()["ui_other_source_packets"], 1)
            observation = receiver.observed_ui(1.5)
            self.assertEqual(observation["filter_state"], "on")
            self.assertFalse(observation["linked_to_specific_video_frame"])

    def test_capture_ignores_duplicate_and_older_telemetry_without_replaying_counter(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            receiver = Receiver(sock, FrameAssembler())
            receiver.consume_telemetry(ui_payload(7, uptime=100, sequence=11), "192.168.10.2", 1)
            receiver.consume_telemetry(ui_payload(6, uptime=99, sequence=10), "192.168.10.2", 1.1)
            receiver.consume_telemetry(ui_payload(7, uptime=101, sequence=12), "192.168.10.2", 1.2)
            self.assertTrue(receiver.capture_requests.empty())
            receiver.consume_telemetry(ui_payload(8, uptime=102, sequence=12), "192.168.10.2", 1.3)
            self.assertTrue(receiver.capture_requests.empty())
            receiver.consume_telemetry(ui_payload(8, uptime=102, sequence=13), "192.168.10.2", 1.4)
            self.assertEqual(receiver.capture_requests.get_nowait()[0].capture_counter, 8)
            receiver.consume_telemetry(ui_payload(8, uptime=102, sequence=13), "192.168.10.2", 1.5)
            self.assertTrue(receiver.capture_requests.empty())
            receiver.consume_telemetry(ui_payload(0, uptime=0, sequence=0), "192.168.10.2", 1.6)
            self.assertTrue(receiver.capture_requests.empty())
            self.assertEqual(receiver.latest_ui[0].tx_sequence, 0)
            receiver.consume_telemetry(ui_payload(1, uptime=1, sequence=1), "192.168.10.2", 1.7)
            self.assertEqual(receiver.capture_requests.get_nowait()[0].capture_counter, 1)
            self.assertEqual(receiver.stats()["ui_ignored_packets"], 3)

    def test_capture_accepts_forward_uint32_sequence_wrap(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            receiver = Receiver(sock, FrameAssembler())
            receiver.consume_telemetry(ui_payload(65535, uptime=100, sequence=0xFFFFFFFF), "192.168.10.2", 1)
            receiver.consume_telemetry(ui_payload(0, uptime=101, sequence=0), "192.168.10.2", 1.1)
            self.assertEqual(receiver.capture_requests.get_nowait()[0].capture_counter, 0)

    def test_missing_reordered_duplicate_and_wrong_source_rows_never_complete(self) -> None:
        for issue in ("missing", "reordered", "duplicate"):
            assembler = FrameAssembler()
            rows = list(range(HEIGHT))
            if issue == "missing":
                rows.remove(47)
            elif issue == "reordered":
                rows[47], rows[48] = rows[48], rows[47]
            else:
                rows.insert(48, 47)
            for i, row in enumerate(rows):
                self.assertIsNone(assembler.consume(row_payload(row), "192.168.10.2", i / 10000))
            self.assertEqual(assembler.snapshot_stats()["complete_frames"], 0)
            self.assertEqual(assembler.snapshot_stats()["incomplete_frames"], 1)
        assembler = FrameAssembler()
        for row in range(HEIGHT):
            self.assertIsNone(assembler.consume(row_payload(row), "192.168.10.99", row / 10000))
        self.assertEqual(assembler.snapshot_stats()["other_source_packets"], HEIGHT)

    def test_complete_little_endian_immutable_frame_and_timeout_recovery(self) -> None:
        assembler = FrameAssembler(row_byte_order="little", frame_timeout=0.1)
        assembler.consume(row_payload(0, "little"), "192.168.10.2", 0)
        assembler.expire(0.11)
        self.assertEqual(assembler.snapshot_stats()["timed_out_frames"], 1)
        frame = None
        for row in range(HEIGHT):
            frame = assembler.consume(row_payload(row, "little", b"\x1f\x00"),
                                      "192.168.10.2", 1 + row / 10000, "fixed UTC")
        self.assertIsNotNone(frame)
        self.assertEqual(frame.pixels, b"\x1f\x00" * WIDTH * HEIGHT)
        self.assertEqual(frame.received_utc, "fixed UTC")
        assembler.consume(row_payload(0, "little", b"\x00\x00"), "192.168.10.2", 2)
        self.assertEqual(frame.pixels[:2], b"\x1f\x00")

    def test_burst_rate_ceiling_excludes_old_duplicate_stale_and_outside_frames(self) -> None:
        burst = BurstSchedule(10, 5, 10, "vera", 1)
        self.assertFalse(burst.select(complete_frame(1, 10.01, 9.99), 10.02, 0.5))
        self.assertFalse(burst.select(complete_frame(2, 10.01), 11, 0.5))
        self.assertTrue(burst.select(complete_frame(3, 11), 11, 0.5))
        self.assertFalse(burst.select(complete_frame(3, 11), 11.1, 0.5))
        self.assertFalse(burst.select(complete_frame(4, 11.05), 11.05, 0.5))
        self.assertTrue(burst.select(complete_frame(5, 11.1), 11.1, 0.5))
        self.assertFalse(burst.select(complete_frame(6, 15), 15, 0.5))
        self.assertEqual(burst.saved, 2)
        fast = BurstSchedule(0, 5, 10, "dog_plush", 1)
        for i in range(300):
            fast.select(complete_frame(i + 1, i / 60), i / 60, 0.5)
        self.assertEqual(fast.saved, 50)

    def test_png_preserves_native_size_colors_and_byte_order(self) -> None:
        # Known RGB565 primary colors, white and black; no GUI resize or labels.
        words = (0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000)
        expected = b"\xff\x00\x00\x00\xff\x00\x00\x00\xff\xff\xff\xff\x00\x00\x00"
        for order in ("big", "little"):
            pixels = b"".join(word.to_bytes(2, order) for word in words) * (WIDTH * HEIGHT // 5)
            dimensions, rgb = decode_png_pixels(encode_png(pixels, order))
            self.assertEqual(dimensions, (WIDTH, HEIGHT))
            self.assertEqual(rgb, expected * (WIDTH * HEIGHT // 5))

    def test_session_image_metadata_and_no_overwrite(self) -> None:
        with tempfile.TemporaryDirectory(prefix=".capture_test_", dir=ROOT) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(ROOT.resolve()))
            directory = Path(temp) / "session"
            config = {"pixel_byte_order": "big", "fpga_profile_manual": "r2-gauss",
                      "filter_state_manual": "on", "pipeline_note": "manual verified"}
            writer = SessionWriter(directory, config)
            frame = complete_frame(1, 0.02)
            burst = BurstSchedule(0, 5, 10, "earphone_cable", 1)
            self.assertTrue(burst.select(frame, 0.02, 0.5))
            target = writer.save(frame, burst, {"incomplete_frames": 3},
                                 {"filter_state": "off", "linked_to_specific_video_frame": False})
            writer.close()
            self.assertTrue(target.is_file())
            record = json.loads((directory / "metadata.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(record["label"], "earphone_cable")
            self.assertEqual(record["filter_state_manual"], "on")
            self.assertEqual(record["fpga_ui_recent_observation"]["filter_state"], "off")
            self.assertEqual(record["receive_counters"]["incomplete_frames"], 3)
            self.assertEqual(record["path"], "earphone_cable/burst001_000001.png")
            with self.assertRaises(FileExistsError):
                SessionWriter(directory, config)

    def test_headless_burst_receives_real_loopback_udp_and_writes_complete_png(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        with tempfile.TemporaryDirectory(prefix=".capture_test_", dir=ROOT) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(ROOT.resolve()))
            process = subprocess.Popen(
                [sys.executable, str(ROOT / "pc" / "capture_dataset.py"), "--burst",
                 "--bind", "127.0.0.1", "--port", str(port), "--expected-source", "127.0.0.1",
                 "--output", temp, "--session", "loopback", "--duration", "0.5",
                 "--class", "dog_plush", "--filter", "off", "--fpga-profile", "loopback-test"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                while "BURST 1:" not in process.stdout.readline():
                    if process.poll() is not None:
                        self.fail("capture exited before listening")
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for row in range(HEIGHT):
                        sender.sendto(row_payload(row, "little", b"\x00\xf8"), ("127.0.0.1", port))
                output, _ = process.communicate(timeout=5)
                metadata_text = (Path(temp) / "loopback" / "metadata.jsonl").read_text()
                self.assertEqual(process.returncode, 0, output + "\n" + metadata_text)
                images = list((Path(temp) / "loopback" / "dog_plush").glob("*.png"))
                self.assertEqual(len(images), 1, output)
                dimensions, rgb = decode_png_pixels(images[0].read_bytes())
                self.assertEqual(dimensions, (WIDTH, HEIGHT))
                self.assertEqual(rgb, b"\xff\x00\x00" * WIDTH * HEIGHT)
                events = [json.loads(line) for line in
                          (Path(temp) / "loopback" / "metadata.jsonl").read_text().splitlines()]
                self.assertEqual(events[-1]["event"], "session_end")
                self.assertEqual(events[-1]["saved_images"], 1)
                self.assertEqual(events[1]["filter_state_manual"], "off")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()
