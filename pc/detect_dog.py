#!/usr/bin/env python3
"""Detect dog_plush and human faces on CPU, from a file or FPGA UDP."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from pathlib import Path
import json
import queue
import socket
import threading
import time

from capture_dataset import Receiver
from detection_feedback import encode_feedback, MAX_RESULT_AGE_MS
from dog_inference import DogDetector, annotate
from face_inference import FaceDetector, annotate_faces
from highgui_window import WindowCloseMonitor, report_gui
from video_stream import FrameAssembler, rgb565_bytes_to_bgr

ROOT = Path(__file__).resolve().parents[1]
WINDOW = "ACG720 human + dog (Q/Esc to quit)"


class VisionDetector:
    def __init__(self, args):
        self.dog = DogDetector(args.model, args.confidence, args.iou, args.threads)
        self.face = None if args.no_face else FaceDetector(args.face_model, args.face_confidence, args.face_iou, args.threads)

    def predict(self, image):
        started = time.perf_counter()
        boxes = self.dog.predict(image)
        dog_ms = (time.perf_counter() - started) * 1000
        started = time.perf_counter()
        faces = self.face.predict(image) if self.face else []
        face_ms = (time.perf_counter() - started) * 1000 if self.face else 0.0
        return boxes, faces, dog_ms, face_ms


def render(image, boxes, faces):
    return annotate_faces(annotate(image, boxes), faces)


@dataclass(frozen=True)
class Result:
    frame: object
    image: object
    boxes: list
    inference_ms: float
    faces: list
    dog_ms: float
    face_ms: float


class LatestDetector:
    """UDP receive remains independent; infer the newest complete frame only."""
    def __init__(self, receiver, detector):
        self.receiver, self.detector = receiver, detector
        self.latest = None
        self.error = None
        self.count = self.skipped = self.stale_inputs = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    frame = self.receiver.frames.get(timeout=0.1)
                except queue.Empty:
                    continue
                while True:
                    try:
                        frame = self.receiver.frames.get_nowait()
                        self.skipped += 1
                    except queue.Empty:
                        break
                if time.monotonic() - frame.started_at > 0.5:
                    self.stale_inputs += 1
                    continue
                started = time.perf_counter()
                image = rgb565_bytes_to_bgr(frame.pixels, frame.row_byte_order)
                boxes, faces, dog_ms, face_ms = self.detector.predict(image)
                elapsed = (time.perf_counter() - started) * 1000
                self.latest = Result(frame, image, boxes, elapsed, faces, dog_ms, face_ms)
                self.count += 1
        except Exception as exc:
            self.error = exc


def show(cv2, monitor, image, headless):
    if headless:
        return False
    cv2.imshow(WINDOW, cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    key = cv2.waitKey(1) & 255
    return key in (27, ord("q"), ord("Q")) or monitor.closed()


def offline(args, detector, cv2, monitor):
    source = Path(args.source)
    if not source.is_file():
        raise ValueError(f"Source does not exist: {source}")
    args.output.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(source))
    records = []
    writer = capture = None
    started = time.perf_counter()
    try:
        if image is not None:
            boxes, faces, dog_ms, face_ms = detector.predict(image)
            rendered = render(image, boxes, faces)
            target = args.output / f"{source.stem}_detected.jpg"
            if not cv2.imwrite(str(target), rendered):
                raise RuntimeError(f"Cannot save {target}")
            records.append(dict(frame=1, boxes=[asdict(b) for b in boxes], faces=[asdict(f) for f in faces],
                                dog_ms=dog_ms, face_ms=face_ms))
            show(cv2, monitor, rendered, args.headless)
        else:
            capture = cv2.VideoCapture(str(source))
            if not capture.isOpened():
                raise RuntimeError(f"Cannot open video {source}")
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not 0 < fps < 1000:
                raise RuntimeError("Video FPS unavailable; refusing to invent playback timing")
            target = args.output / f"{source.stem}_detected.avi"
            while True:
                ok, image = capture.read()
                if not ok:
                    break
                if writer is None:
                    height, width = image.shape[:2]
                    writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*"MJPG"), fps, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError(f"Cannot create {target}")
                tick = time.perf_counter()
                boxes, faces, dog_ms, face_ms = detector.predict(image)
                records.append(dict(frame=len(records) + 1, inference_ms=(time.perf_counter() - tick) * 1000,
                                    boxes=[asdict(b) for b in boxes], faces=[asdict(f) for f in faces],
                                    dog_ms=dog_ms, face_ms=face_ms))
                rendered = render(image, boxes, faces)
                writer.write(rendered)
                if show(cv2, monitor, rendered, args.headless):
                    break
                if len(records) % 100 == 0:
                    print(f"Processed {len(records)} frames", flush=True)
            if not records:
                raise RuntimeError("No decodable video frames")
    finally:
        if writer is not None:
            writer.release()
        if capture is not None:
            capture.release()
    report = dict(source=str(source), scope="Offline CPU inference; no ground-truth accuracy or board claim",
                  frames=len(records), frames_with_dog=sum(bool(r["boxes"]) for r in records),
                  frames_with_human=sum(bool(r["faces"]) for r in records),
                  elapsed_seconds=time.perf_counter() - started, detections=records)
    (args.output / f"{source.stem}_detections.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {target}; frames={len(records)}, dog_frames={report['frames_with_dog']}, human_frames={report['frames_with_human']}", flush=True)
    return 0


def live(args, detector, cv2, monitor):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    feedback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    feedback.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.1)
    receiver = Receiver(sock, FrameAssembler(args.expected_source, args.row_byte_order))
    worker = LatestDetector(receiver, detector)
    sequence = 0
    feedback_error = None

    def send(result=None, valid=False):
        nonlocal sequence, feedback_error
        if args.no_feedback:
            return
        age = round((time.monotonic() - result.frame.started_at) * 1000) if result else 0
        packet = encode_feedback(sequence, result.boxes[0] if valid and result.boxes else None,
                                 result_valid=valid, age_ms=age,
                                 inference_ms=round(result.inference_ms) if result else 0,
                                 frame_number=result.frame.frame_number if result else 0)
        sequence = (sequence + 1) & 65535
        try:
            feedback.sendto(packet, (args.feedback_ip, args.feedback_port))
        except OSError as exc:
            feedback_error = str(exc)
            print(f"FPGA feedback unavailable: {exc}", flush=True)

    started = time.monotonic()
    last_sent = last_log = 0.0
    last_result = None
    invalid_sent = False
    records = []
    receiver.thread.start()
    worker.thread.start()
    print(f"Listening {args.bind}:{args.port}; source={args.expected_source}; CPU threads={args.threads}", flush=True)
    print("Feedback: " + ("disabled" if args.no_feedback else f"DET1 display only -> {args.feedback_ip}:{args.feedback_port}"), flush=True)
    try:
        while not args.seconds or time.monotonic() - started < args.seconds:
            now = time.monotonic()
            result = worker.latest
            fresh = result is not None and 0 <= (now - result.frame.started_at) * 1000 <= MAX_RESULT_AGE_MS
            if worker.error:
                raise RuntimeError(f"Inference failed: {worker.error}")
            if receiver.error:
                raise receiver.error
            if fresh and result is not last_result and now - last_sent >= 0.1:
                send(result, True)
                last_result, last_sent, invalid_sent = result, now, False
                records.append(dict(frame=result.frame.frame_number, inference_ms=result.inference_ms,
                                    age_ms=round((now - result.frame.started_at) * 1000),
                                    boxes=[asdict(b) for b in result.boxes], faces=[asdict(f) for f in result.faces],
                                    dog_ms=result.dog_ms, face_ms=result.face_ms))
            elif not fresh and not invalid_sent:
                send()
                invalid_sent = True
            if now - last_log >= 1:
                status = "waiting/stale" if not fresh else f"dog={len(result.boxes)} human={len(result.faces)} infer={result.inference_ms:.1f}ms (dog={result.dog_ms:.1f}, face={result.face_ms:.1f})"
                print(f"complete={receiver.stats()['complete_frames']} inferred={worker.count} skipped={worker.skipped} {status}", flush=True)
                last_log = now
            if not args.headless:
                import numpy as np
                image = render(result.image, result.boxes if fresh else [], result.faces if fresh else []) if result else np.zeros((240, 400, 3), np.uint8)
                status = f"dog={len(result.boxes)} human={len(result.faces)} CPU {result.inference_ms:.0f}ms" if fresh else "WAIT / STALE: detection cleared"
                cv2.putText(image, status, (4, 232), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 240), 1)
                if show(cv2, monitor, image, False):
                    break
            else:
                time.sleep(0.01)
    finally:
        send()  # Explicitly clear the LCD result; no start/stop motor command.
        worker.stop.set()
        receiver.stop.set()
        sock.close()
        receiver.thread.join(timeout=1)
        worker.thread.join(timeout=2)
        feedback.close()
        report = dict(scope="Live UDP CPU inference, display feedback only; no tracking/motor control",
                      seconds=time.monotonic() - started, receiver=receiver.stats(),
                      inferred=worker.count, skipped=worker.skipped, stale_inputs=worker.stale_inputs,
                      feedback_error=feedback_error, detections=records)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Stopped; complete={report['receiver']['complete_frames']}, inferred={worker.count}", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/dog_plush/dog_plush_best.onnx")
    parser.add_argument("--face-model", type=Path, default=ROOT / "models/face_yunet/face_detection_yunet_2026may.onnx")
    parser.add_argument("--face-confidence", type=float, default=.6)
    parser.add_argument("--face-iou", type=float, default=.3)
    parser.add_argument("--no-face", action="store_true", help="Run the original dog-only comparison")
    parser.add_argument("--source", help="Offline image/video; never sends FPGA feedback")
    parser.add_argument("--output", type=Path, default=ROOT / "dataset/model_validation")
    parser.add_argument("--bind", default="192.168.10.3")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--expected-source", default="192.168.10.2")
    parser.add_argument("--row-byte-order", choices=("auto", "big", "little"), default="auto")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=0, help="Live duration; 0=until stopped")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-feedback", action="store_true", help="Test model with existing R4/R5 FPGA firmware")
    parser.add_argument("--feedback-ip", default="192.168.10.255")
    parser.add_argument("--feedback-port", type=int, default=5001)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if not 0 < args.confidence <= 1 or not 0 < args.iou <= 1 or args.threads < 1 or args.seconds < 0 or not 0 < args.face_confidence <= 1 or not 0 < args.face_iou <= 1:
        parser.error("Invalid confidence, IoU, threads or duration")
    try:
        import cv2
        detector = VisionDetector(args)
        print(f"dog: YOLOv8n CPU conf={args.confidence}; human: " +
              ("disabled" if args.no_face else f"YuNet CPU conf={args.face_confidence}"), flush=True)
        print("FPGA DET1 still carries dog only; human labels are on the host preview/report.", flush=True)
        if not args.headless:
            report_gui(cv2)
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        monitor = None if args.headless else WindowCloseMonitor(cv2, WINDOW)
        try:
            return offline(args, detector, cv2, monitor) if args.source else live(args, detector, cv2, monitor)
        finally:
            if not args.headless:
                cv2.destroyAllWindows()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Detection failed: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
