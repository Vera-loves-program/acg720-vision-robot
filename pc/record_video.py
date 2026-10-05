#!/usr/bin/env python3
"""Record the received FPGA image stream to a headless MJPEG AVI, without UI/audio.

The existing receiver assembles complete frames on a separate thread. Output
uses a wall-clock cadence: excess frames are skipped, brief gaps repeat the
last frame, and a prolonged gap stops recording instead of saving stale video.
No commands are sent to the FPGA. No camera exposure timestamps are available.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import math
from pathlib import Path
import queue
import socket
import time

from capture_dataset import Receiver
from video_stream import FrameAssembler, HEIGHT, WIDTH, rgb565_bytes_to_bgr


def positive_number(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.10.3")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--expected-source", default="192.168.10.2")
    parser.add_argument("--seconds", type=positive_number, default=30.0,
                        help="Recording duration, starting at the first complete frame.")
    parser.add_argument("--fps", type=positive_number, default=30.0,
                        help="Output playback rate (default: 30), not a camera setting.")
    parser.add_argument("--wait-timeout", type=positive_number, default=10.0,
                        help="Maximum initial wait for a complete frame.")
    parser.add_argument("--max-age", type=positive_number, default=0.5,
                        help="Stop if the newest complete frame is older than this.")
    parser.add_argument("--byte-order", choices=("auto", "big", "little"), default="auto")
    parser.add_argument("--row-byte-order", choices=("auto", "big", "little"), default="auto")
    parser.add_argument("--output", type=Path,
                        help="New .avi file; default: dataset/videos/dog_plush_TIMESTAMP.avi")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be 1..65535")
    output = args.output or Path("dataset/videos") / (
        "dog_plush_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".avi")
    if output.suffix.lower() != ".avi":
        parser.error("--output must end in .avi (MJPEG)")
    if output.exists():
        parser.error(f"File already exists; choose another --output: {output}")
    try:
        import cv2
    except ImportError:
        parser.error("OpenCV is missing; use the existing environment with pc/requirements.txt.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.settimeout(0.1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        sock.close()
        parser.error(f"Cannot listen on {args.bind}:{args.port}: {exc}. "
                     "Close capture_dataset.py, viewer and probe first.")
    receiver = Receiver(sock, FrameAssembler(args.expected_source, args.row_byte_order))
    writer = None
    written = repeated = 0
    last_written_number = 0
    cached_number = 0
    cached_image = None
    status = 0
    receiver.thread.start()
    print(f"Listening on {args.bind}:{args.port}; waiting for the first complete frame "
          f"(up to {args.wait_timeout:g}s)...", flush=True)
    try:
        latest = None
        wait_until = time.monotonic() + args.wait_timeout
        while latest is None:
            remaining = wait_until - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("No complete frame arrived; no video was created.")
            if receiver.error:
                raise receiver.error
            try:
                latest = receiver.frames.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue
        if time.monotonic() - latest.received_at > args.max_age:
            raise RuntimeError("First queued frame is stale; no video was created.")
        output.parent.mkdir(parents=True, exist_ok=True)
        # Reserve a new path exclusively before letting OpenCV write its header.
        with output.open("xb"):
            pass
        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"MJPG"),
                                 args.fps, (WIDTH, HEIGHT))
        if not writer.isOpened():
            raise RuntimeError("OpenCV could not open the MJPEG AVI writer.")
        started = time.monotonic()
        target_frames = math.ceil(args.seconds * args.fps)
        print(f"RECORDING: {args.seconds:g}s, {args.fps:g}fps, {WIDTH}x{HEIGHT}, "
              f"no audio -> {output.resolve()}", flush=True)
        last_progress = started
        for index in range(target_frames):
            due = started + index / args.fps
            while time.monotonic() < due:
                if receiver.error:
                    raise receiver.error
                try:
                    latest = receiver.frames.get(timeout=min(0.1, max(0.001, due - time.monotonic())))
                except queue.Empty:
                    pass
            # Bounded drain: writing must proceed even while UDP keeps arriving.
            for _ in range(32):
                try:
                    latest = receiver.frames.get_nowait()
                except queue.Empty:
                    break
            now = time.monotonic()
            if receiver.error:
                raise receiver.error
            if now - due > args.max_age:
                raise RuntimeError("Video encoding fell behind the recording clock; stopped early.")
            if now - latest.received_at > args.max_age:
                raise RuntimeError("Complete frames stopped arriving; stopped early.")
            if latest.frame_number != cached_number:
                order = latest.row_byte_order if args.byte_order == "auto" else args.byte_order
                cached_image = rgb565_bytes_to_bgr(latest.pixels, order)
                cached_number = latest.frame_number
            writer.write(cached_image)
            repeated += int(latest.frame_number == last_written_number)
            last_written_number = latest.frame_number
            written += 1
            if now - last_progress >= 5:
                print(f"REC: {written / args.fps:.1f}/{args.seconds:g}s", flush=True)
                last_progress = now
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C; finalizing the recorded portion.", flush=True)
    except (OSError, RuntimeError, cv2.error) as exc:
        print(f"ERROR: {exc}", flush=True)
        status = 1
    finally:
        receiver.stop.set()
        receiver.thread.join(timeout=1)
        sock.close()
        if writer is not None:
            writer.release()
        stats = receiver.stats()
        print(f"Received complete={stats['complete_frames']}, "
              f"incomplete={stats['incomplete_frames']}, bad={stats['bad_packets']}, "
              f"queue_drops={stats['receiver_queue_drops']}, UI={stats['ui_packets']}", flush=True)
        if written:
            print(f"SAVED: {output.resolve()} | frames={written}, "
                  f"playback={written / args.fps:.2f}s, repeated={repeated}", flush=True)
        else:
            print("No video frames saved.", flush=True)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
