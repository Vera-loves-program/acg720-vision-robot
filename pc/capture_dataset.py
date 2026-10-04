#!/usr/bin/env python3
"""Capture clean FPGA UDP images: Space/C starts a five-second PNG burst.

GUI mode needs the existing NumPy/OpenCV dependencies. --burst uses only the
Python standard library and exits after one burst. Images are native 400x240
RGB565 converted to RGB PNG, without the preview's labels, scale or overlays.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import re
import socket
import struct
import threading
import time
import uuid
import zlib

from video_stream import (CompleteFrame, FrameAssembler, HEIGHT, WIDTH,
                          rgb565_bytes_to_bgr, rgb565_bytes_to_rgb)
from ui_telemetry import UITelemetry, parse_ui_telemetry
from highgui_window import WindowCloseMonitor, report_gui


def safe_name(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", value):
        raise argparse.ArgumentTypeError(
            "Use 1-80 letters/numbers/underscores/hyphens, starting with a letter or number.")
    return value


def _png_chunk(kind: bytes, body: bytes) -> bytes:
    return (struct.pack(">I", len(body)) + kind + body +
            struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))


def encode_png(pixels: bytes, byte_order: str) -> bytes:
    rgb = rgb565_bytes_to_rgb(pixels, byte_order)
    stride = WIDTH * 3
    rows = b"".join(b"\x00" + rgb[y * stride:(y + 1) * stride] for y in range(HEIGHT))
    return (b"\x89PNG\r\n\x1a\n" +
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)) +
            _png_chunk(b"IDAT", zlib.compress(rows, level=1)) + _png_chunk(b"IEND", b""))


@dataclass
class BurstSchedule:
    """Select only newly completed frames, never repeat a cached image."""
    started_at: float
    duration: float
    fps: float
    label: str
    number: int
    next_due: float = 0.0
    saved: int = 0
    skipped_cadence: int = 0
    skipped_stale: int = 0
    skipped_before_burst: int = 0
    _last_frame_number: int = 0

    def __post_init__(self) -> None:
        if self.duration <= 0 or self.fps <= 0:
            raise ValueError("duration and fps must be positive")
        self.next_due = self.started_at

    @property
    def ends_at(self) -> float:
        return self.started_at + self.duration

    def select(self, frame: CompleteFrame, now: float, max_age: float) -> bool:
        if frame.frame_number <= self._last_frame_number:
            return False
        self._last_frame_number = frame.frame_number
        if frame.started_at < self.started_at:
            self.skipped_before_burst += 1
            return False
        if frame.received_at >= self.ends_at or frame.received_at < self.started_at:
            return False
        if now - frame.received_at > max_age:
            self.skipped_stale += 1
            return False
        if frame.received_at + 1e-9 < self.next_due:
            self.skipped_cadence += 1
            return False
        # Rate is a ceiling; never fill missing intervals with duplicated frames.
        self.next_due = frame.received_at + 1.0 / self.fps
        self.saved += 1
        return True


class Receiver:
    def __init__(self, sock: socket.socket, assembler: FrameAssembler) -> None:
        self.sock, self.assembler = sock, assembler
        self.frames: queue.Queue[CompleteFrame] = queue.Queue(maxsize=32)
        self.stop = threading.Event()
        self.queue_drops = 0
        self.ui_packets = self.ui_bad_packets = self.ui_other_source_packets = 0
        self.ui_ignored_packets = 0
        self.capture_request_drops = 0
        self.capture_requests: queue.Queue[tuple[UITelemetry, float]] = queue.Queue(maxsize=8)
        self.latest_ui: tuple[UITelemetry, float, str] | None = None
        self.error: OSError | None = None
        self.last_complete_at = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def consume_telemetry(self, payload: bytes, source: str, now: float | None = None) -> bool:
        """Consume the distinct R3 status stream before the video assembler."""
        if not payload.startswith(b"VUI1"):
            return False
        if source != self.assembler.expected_source:
            self.ui_other_source_packets += 1
            return True
        ui = parse_ui_telemetry(payload)
        if ui is None:
            self.ui_bad_packets += 1
            return True
        self.ui_packets += 1
        now = time.monotonic() if now is None else now
        previous = self.latest_ui[0] if self.latest_ui else None
        restarted = False
        if previous is not None:
            delta = (ui.tx_sequence - previous.tx_sequence) & 0xFFFFFFFF
            if delta == 0:
                self.ui_ignored_packets += 1
                return True
            if delta >= 0x80000000:
                # A small backwards step is UDP reordering. Treat a large
                # uptime drop (>2 seconds) together with a backwards sequence
                # as a reboot and establish a new baseline without capture.
                restarted = previous.uptime_ticks - ui.uptime_ticks > 20
                if not restarted:
                    self.ui_ignored_packets += 1
                    return True
        self.latest_ui = (ui, now, datetime.now(timezone.utc).isoformat())
        # First observation and a board restart establish a baseline. Never
        # turn historical counters into fresh camera captures.
        if previous and not restarted and ui.capture_counter != previous.capture_counter:
            try:
                self.capture_requests.put_nowait((ui, now))
            except queue.Full:
                self.capture_request_drops += 1
        return True

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                payload, peer = self.sock.recvfrom(65535)
            except socket.timeout:
                self.assembler.expire()
                continue
            except OSError as exc:
                if not self.stop.is_set():
                    self.error = exc
                return
            if self.consume_telemetry(payload, peer[0]):
                continue
            frame = self.assembler.consume(payload, peer[0])
            if frame is None:
                continue
            self.last_complete_at = frame.received_at
            try:
                self.frames.put_nowait(frame)
            except queue.Full:
                # Keep recent frames while disk/GUI work proceeds on another thread.
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    pass
                self.queue_drops += 1
                try:
                    self.frames.put_nowait(frame)
                except queue.Full:
                    self.queue_drops += 1

    def stats(self) -> dict[str, int]:
        return {**self.assembler.snapshot_stats(), "receiver_queue_drops": self.queue_drops,
                "ui_packets": self.ui_packets, "ui_bad_packets": self.ui_bad_packets,
                "ui_other_source_packets": self.ui_other_source_packets,
                "ui_ignored_packets": self.ui_ignored_packets,
                "capture_request_drops": self.capture_request_drops}

    def observed_ui(self, now: float | None = None) -> dict | None:
        observation = self.latest_ui
        if observation is None:
            return None
        ui, received_at, received_utc = observation
        now = time.monotonic() if now is None else now
        return {"received_utc": received_utc, "age_at_save_seconds": max(0.0, now - received_at),
                "filter_state": "on" if ui.gaussian_on else "off",
                "debug": ui.debug, "source_ip": self.assembler.expected_source,
                "tx_sequence": ui.tx_sequence, "uptime_ticks": ui.uptime_ticks,
                "linked_to_specific_video_frame": False}

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=1)
        self.sock.close()


class SessionWriter:
    def __init__(self, directory: Path, config: dict) -> None:
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.config = config
        self.index = 0
        self.log = (directory / "metadata.jsonl").open("x", encoding="utf-8")
        (directory / "session.json").write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def record(self, event: dict) -> None:
        self.log.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.log.flush()

    def save(self, frame: CompleteFrame, burst: BurstSchedule,
             stats: dict[str, int], observed_ui: dict | None = None) -> Path:
        self.index += 1
        label_dir = self.directory / burst.label
        label_dir.mkdir(exist_ok=True)
        filename = f"burst{burst.number:03d}_{self.index:06d}.png"
        target = label_dir / filename
        temporary = target.with_suffix(".png.tmp")
        pixel_order = (frame.row_byte_order if self.config["pixel_byte_order"] == "auto"
                       else self.config["pixel_byte_order"])
        with temporary.open("xb") as image:
            image.write(encode_png(frame.pixels, pixel_order))
        temporary.replace(target)
        self.record({"event": "image", "path": str(target.relative_to(self.directory)).replace("\\", "/"),
                     "label": burst.label, "burst": burst.number,
                     "received_utc": frame.received_utc,
                     "saved_utc": datetime.now(timezone.utc).isoformat(),
                     "received_monotonic_seconds": frame.received_at,
                     "frame_first_row_monotonic_seconds": frame.started_at,
                     "burst_elapsed_seconds": frame.received_at - burst.started_at,
                     "receiver_frame_number": frame.frame_number,
                     "source_ip": frame.source, "width": WIDTH, "height": HEIGHT,
                     "row_byte_order_resolved": frame.row_byte_order,
                     "pixel_byte_order_resolved": pixel_order,
                     "fpga_profile_manual": self.config["fpga_profile_manual"],
                     "filter_state_manual": self.config["filter_state_manual"],
                     "fpga_ui_recent_observation": observed_ui,
                     "pipeline_note": self.config["pipeline_note"],
                     "receive_counters": stats})
        return target

    def close(self) -> None:
        self.log.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.10.3", help="This machine's Ethernet IPv4 address.")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--expected-source", default="192.168.10.2")
    parser.add_argument("--byte-order", choices=("auto", "big", "little"), default="auto",
                        help="Pixel order; auto follows the detected row transport order.")
    parser.add_argument("--row-byte-order", choices=("auto", "big", "little"), default="auto",
                        help="Auto detects headers from the FPGA (recommended).")
    parser.add_argument("--output", type=Path, default=Path("dataset/captures"))
    parser.add_argument("--session", type=safe_name,
                        default=datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6])
    parser.add_argument("--labels", nargs=3, type=safe_name,
                        default=["vera", "dog_plush", "earphone_cable"], metavar=("PERSON", "DOG", "CABLE"))
    parser.add_argument("--class", dest="class_name", type=safe_name,
                        help="Initial label; defaults to the first --labels entry.")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds in each burst (default: 5).")
    parser.add_argument("--fps", type=float, default=10.0, help="Maximum saved images/sec (default: 10).")
    parser.add_argument("--max-age", type=float, default=0.5,
                        help="Discard frames queued longer than this many seconds.")
    parser.add_argument("--frame-timeout", type=float, default=0.5,
                        help="Discard unfinished row sequences after this many seconds.")
    parser.add_argument("--fpga-profile", default="unconfirmed",
                        help="Manually verified bitstream/pipeline version; copied into every image record.")
    parser.add_argument("--filter", choices=("on", "off", "unknown"), default="unknown",
                        help="Manually verified FPGA filter state; this sends no FPGA command.")
    parser.add_argument("--pipeline-note", default="",
                        help="Optional camera/FPGA preprocessing description; no change to received pixels.")
    parser.add_argument("--burst", action="store_true", help="No GUI: capture one timed burst, then exit.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    for option in ("duration", "fps", "max_age", "frame_timeout"):
        if not (0 < getattr(args, option) < float("inf")):
            parser.error(f"--{option.replace('_', '-')} must be finite and positive")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be 1..65535")
    if len(set(args.labels)) != 3:
        parser.error("--labels must name three different classes")
    cv2 = np = None
    if not args.burst:
        try:
            import cv2
            import numpy as np
            # The Linux OpenCV wheel can set a nonexistent bundled font path.
            # Use already installed fonts, without changing the WSL environment.
            if not Path(os.environ.get("QT_QPA_FONTDIR", "/nonexistent")).is_dir():
                for fonts in ("/usr/share/fonts/truetype/dejavu",
                              "/usr/share/fonts/truetype/liberation2"):
                    if Path(fonts).is_dir():
                        os.environ["QT_QPA_FONTDIR"] = fonts
                        break
        except ImportError as exc:
            print(f"GUI dependencies missing: {exc}. Install pc/requirements.txt, or use --burst.")
            return 1
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.settimeout(0.1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        sock.close()
        print(f"Cannot listen on {args.bind}:{args.port}: {exc}")
        print("Set this machine's Ethernet address to 192.168.10.3; close viewer and udp_probe first.")
        return 1
    session_dir = args.output / args.session
    config = {"format_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
              "bind": args.bind, "port": args.port, "expected_source": args.expected_source,
              "width": WIDTH, "height": HEIGHT, "wire_format": "RGB565 UDP ordered rows",
              "image_format": "lossless RGB PNG; RGB565 quantization retained",
              "row_byte_order": args.row_byte_order, "pixel_byte_order": args.byte_order,
              "labels": args.labels, "initial_label": args.class_name or args.labels[0],
              "max_save_fps": args.fps, "burst_duration_seconds": args.duration,
              "fpga_profile_manual": args.fpga_profile, "filter_state_manual": args.filter,
              "pipeline_note": args.pipeline_note,
              "preprocessing_state_verified_by_receiver": False,
              "note": "R3 VUI1 can report recent UI/filter state; it has no association with a specific video frame."
                      " Older bitstreams have no UI telemetry. The video row protocol has no camera frame ID."}
    try:
        writer = SessionWriter(session_dir, config)
    except OSError as exc:
        sock.close()
        print(f"Cannot create new session {session_dir}: {exc}. Choose a new --session or writable --output.")
        return 1
    receiver = Receiver(sock, FrameAssembler(args.expected_source, args.row_byte_order, args.frame_timeout))
    label = args.class_name or args.labels[0]
    latest = None
    burst: BurstSchedule | None = None
    burst_number = 0
    exit_code = 0
    last_diagnostic_at = 0.0
    announced_order = None
    window = "VERA - Dataset Capture (PNG excludes this UI)"
    print(f"Listening on {args.bind}:{args.port}, source {args.expected_source}")
    print(f"Saving clean {WIDTH}x{HEIGHT} PNG to {session_dir.resolve()}")
    print(f"Manual profile={args.fpga_profile}, filter={args.filter}; R3 UI observations have no per-frame ACK.")
    print("Close other UDP receivers. 1/2/3 select class; Space/C starts burst; Q/Esc exits.")

    def start_burst(trigger: str = "keyboard") -> BurstSchedule | None:
        nonlocal burst_number
        if not args.burst and (receiver.last_complete_at == 0 or
                time.monotonic() - receiver.last_complete_at > args.max_age):
            print("NOT READY: wait for a fresh complete camera image before pressing SPACE/C.", flush=True)
            return None
        burst_number += 1
        started = BurstSchedule(time.monotonic(), args.duration, args.fps, label, burst_number)
        writer.record({"event": "burst_start", "burst": burst_number, "label": label,
                       "started_utc": datetime.now(timezone.utc).isoformat(),
                       "trigger": trigger,
                       "duration_seconds": args.duration, "max_save_fps": args.fps,
                       "receive_counters": receiver.stats()})
        print(f"BURST {burst_number}: {label}, {args.duration:g}s, at most {args.fps:g} images/s", flush=True)
        return started

    receiver.thread.start()
    if args.burst:
        burst = start_burst("command_line")
    try:
        if not args.burst:
            cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE | getattr(cv2, "WINDOW_GUI_NORMAL", 0))
            report_gui(cv2)
            window_monitor = WindowCloseMonitor(cv2, window)
        while True:
            if receiver.error:
                raise receiver.error
            for _ in range(8):
                try:
                    ui, requested_at = receiver.capture_requests.get_nowait()
                except queue.Empty:
                    break
                if time.monotonic() - requested_at > args.max_age:
                    reason = "stale"
                elif burst is not None:
                    reason = "burst_already_running"
                elif args.burst:
                    reason = "headless_single_burst"
                else:
                    burst = start_burst("fpga_touch_capture")
                    print(f"FPGA CAPTURE request counter={ui.capture_counter}")
                    continue
                writer.record({"event": "capture_request_ignored", "reason": reason,
                               "fpga_capture_counter": ui.capture_counter})
            for _ in range(32):
                try:
                    frame = receiver.frames.get_nowait()
                except queue.Empty:
                    break
                now = time.monotonic()
                if not args.burst and now - frame.received_at <= args.max_age:
                    pixel_order = frame.row_byte_order if args.byte_order == "auto" else args.byte_order
                    latest = rgb565_bytes_to_bgr(frame.pixels, pixel_order)
                if burst and burst.select(frame, now, args.max_age):
                    target = writer.save(frame, burst, receiver.stats(), receiver.observed_ui())
                    print(f"SAVED {burst.label} {burst.saved}: {target.name}", flush=True)
            now = time.monotonic()
            detected_order = receiver.assembler.detected_row_byte_order
            if detected_order and detected_order != announced_order:
                announced_order = detected_order
                pixel_order = detected_order if args.byte_order == "auto" else args.byte_order
                print(f"FORMAT: row={detected_order}, pixels={pixel_order}", flush=True)
            stats = receiver.stats()
            if now - last_diagnostic_at >= 3:
                last_diagnostic_at = now
                print(f"RX: packets={stats['packets']} valid_rows={stats['valid_rows']} "
                      f"complete={stats['complete_frames']} incomplete={stats['incomplete_frames']} "
                      f"bad={stats['bad_packets']} row={detected_order or 'detecting'} "
                      f"order_conflicts={stats['row_order_conflicts']}", flush=True)
                if stats['row_order_conflicts']:
                    print("ROW ORDER MISMATCH: retry without explicit byte-order options (auto).", flush=True)
            if burst and now >= burst.ends_at:
                writer.record({"event": "burst_end", "burst": burst.number, "label": burst.label,
                               "saved_images": burst.saved, "skipped_cadence": burst.skipped_cadence,
                               "skipped_stale": burst.skipped_stale,
                               "skipped_before_burst": burst.skipped_before_burst,
                               "receive_counters": receiver.stats()})
                print(f"BURST DONE: {burst.saved} images saved for {burst.label}", flush=True)
                if args.burst:
                    exit_code = 0 if burst.saved else 2
                    if not burst.saved:
                        print("No fresh complete frame saved. Check cable/IP/firewall/byte order; run udp_probe.")
                    burst = None
                    break
                burst = None
            if args.burst:
                time.sleep(0.005)
                continue
            canvas = np.full((660, 800, 3), (244, 244, 240), dtype=np.uint8)
            if latest is not None:
                canvas[:480] = cv2.resize(latest, (800, 480), interpolation=cv2.INTER_NEAREST)
            else:
                cv2.putText(canvas, "WAITING FOR COMPLETE FPGA FRAME", (75, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (23, 23, 23), 2)
            active = "IDLE" if burst is None else f"CAPTURING {burst.saved} / {max(0, burst.ends_at-now):.1f}s left"
            for y, text in ((508, f"CLASS: {label}  |  {active}"),
                            (538, f"1/2/3: class   SPACE/C: {args.duration:g}s burst   Q: quit"),
                            (568, f"RX {stats['packets']}  complete {stats['complete_frames']}  incomplete {stats['incomplete_frames']}"),
                            (598, f"ROWS {detected_order or 'detecting'}  BAD {stats['bad_packets']}  FORMAT CONFLICTS {stats['row_order_conflicts']}"),
                            (628, "PNG: full FPGA frame, no UI. Wait for image, then SPACE/C.")):
                cv2.putText(canvas, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (23, 23, 23), 1)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                print("Capture stopped: Q/Esc pressed.", flush=True)
                break
            if key in (ord("1"), ord("2"), ord("3")):
                if burst is None:
                    label = args.labels[key - ord("1")]
                    print(f"Selected class: {label}")
                else:
                    print("Class stays fixed during a burst; wait until BURST DONE.")
            elif key in (32, ord("c"), ord("C")):
                if burst is None:
                    burst = start_burst()
                else:
                    print("Burst already running.")
            if window_monitor.closed():
                print("Capture stopped: image window closed.", flush=True)
                break
    except KeyboardInterrupt:
        print("Capture stopped by user.")
    except OSError as exc:
        print(f"Capture failed: {exc}")
        exit_code = 1
    except Exception as exc:
        # OpenCV can fail when no desktop/WSLg display is available.
        print(f"Capture failed: {exc}. For no display, retry with --burst.")
        exit_code = 1
    finally:
        receiver.close()
        if burst is not None:
            writer.record({"event": "burst_stopped", "burst": burst.number, "label": burst.label,
                           "saved_images": burst.saved})
        writer.record({"event": "session_end", "saved_images": writer.index,
                       "receive_counters": receiver.stats(),
                       "ended_utc": datetime.now(timezone.utc).isoformat()})
        writer.close()
        if cv2 is not None:
            cv2.destroyAllWindows()
    print(f"Session saved {writer.index} images: {session_dir.resolve()}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
