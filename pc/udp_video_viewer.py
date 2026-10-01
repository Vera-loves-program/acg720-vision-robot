#!/usr/bin/env python3
"""Receive chapter-59-style RGB565 rows from the ACG720 over UDP.

Each 802-byte payload is a big-endian row number (0..239) followed by
400 RGB565 pixels. A row-0 packet starts a new frame. The receiver discards
incomplete frames rather than showing rows from different camera frames.
Run on Windows directly or on WSL2 with working LAN inbound UDP networking.
"""

from __future__ import annotations

import argparse
import socket
import time

import cv2
import numpy as np


WIDTH = 400
HEIGHT = 240
PAYLOAD_BYTES = 2 + WIDTH * 2
COMMAND_PORT = 5001
COMMAND_BROADCAST = "192.168.10.255"


def rgb565_to_bgr(pixels: np.ndarray) -> np.ndarray:
    red = ((pixels >> 11) & 31).astype(np.uint16) * 255 // 31
    green = ((pixels >> 5) & 63).astype(np.uint16) * 255 // 63
    blue = (pixels & 31).astype(np.uint16) * 255 // 31
    return np.stack((blue, green, red), axis=-1).astype(np.uint8)


def command_packet(opcode: int, value: int, sequence: int) -> bytes:
    body = b"VRB1" + bytes((opcode, value, sequence & 255))
    checksum = 0
    for byte in body:
        checksum ^= byte
    return body + bytes((checksum,))


def render(frame: np.ndarray | None, stats: dict[str, float | int | str]) -> np.ndarray:
    # Match the owner's neo-brutalist portfolio: cream dots, thick black
    # borders, hard offset shadows, blue #0062AD and yellow #FFD15C.
    cream, ink, blue, yellow = (240, 244, 244), (23, 23, 23), (173, 98, 0), (92, 209, 255)
    white, red = (255, 255, 255), (77, 77, 255)
    canvas = np.full((660, 1060, 3), cream, dtype=np.uint8)
    canvas[4::30, 4::30] = (204, 204, 204)
    cv2.rectangle(canvas, (0, 0), (1059, 60), blue, -1)
    cv2.rectangle(canvas, (0, 59), (1059, 63), ink, -1)
    cv2.putText(canvas, "VERA / VISION LAB", (24, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.86, white, 2, cv2.LINE_AA)
    cv2.putText(canvas, "ACG720  /  LIVE UDP", (730, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.48, white, 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (22, 86), (834, 578), ink, -1)
    cv2.rectangle(canvas, (14, 78), (826, 570), ink, 4)
    if frame is None:
        cv2.putText(canvas, "WAITING FOR FPGA VIDEO", (205, 325), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ink, 2, cv2.LINE_AA)
    else:
        canvas[84:564, 20:820] = cv2.resize(frame, (800, 480), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(canvas, (850, 86), (1054, 578), ink, -1)
    cv2.rectangle(canvas, (842, 78), (1046, 570), yellow, -1)
    cv2.rectangle(canvas, (842, 78), (1046, 570), ink, 4)
    cv2.putText(canvas, "LINK / STATUS", (852, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.52, blue, 2, cv2.LINE_AA)
    for y, label, value in (
        (151, "SOURCE", str(stats["source"])),
        (194, "VIDEO", "400 x 240 RGB565"),
        (237, "FPS", f"{stats['fps']:.1f}"),
        (280, "GOOD FRAMES", str(stats["frames"])),
        (323, "LOST FRAMES", str(stats["lost"])),
        (366, "BAD PACKETS", str(stats["bad"])),
    ):
        cv2.putText(canvas, label, (852, y), cv2.FONT_HERSHEY_SIMPLEX, 0.41, blue, 1, cv2.LINE_AA)
        cv2.putText(canvas, value[:19], (852, y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, ink, 1, cv2.LINE_AA)
    cv2.putText(canvas, "F: GAUSS  D: DEBUG", (852, 442), cv2.FONT_HERSHEY_SIMPLEX, 0.39, ink, 1, cv2.LINE_AA)
    cv2.putText(canvas, "E: STOP   R: CLEAR", (852, 468), cv2.FONT_HERSHEY_SIMPLEX, 0.39, red, 1, cv2.LINE_AA)
    cv2.putText(canvas, "Q: QUIT", (852, 494), cv2.FONT_HERSHEY_SIMPLEX, 0.39, ink, 1, cv2.LINE_AA)
    cv2.putText(canvas, "STOP is a protocol/UI latch; motor cut-off is not wired in this revision.",
                (20, 624), cv2.FONT_HERSHEY_SIMPLEX, 0.48, ink, 1, cv2.LINE_AA)
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local Ethernet address or 0.0.0.0")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--command-address", default=COMMAND_BROADCAST)
    parser.add_argument("--byte-order", choices=("big", "little"), default="big")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((args.bind, args.port))
    sock.setblocking(False)

    frame = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
    seen = np.zeros(HEIGHT, dtype=np.bool_)
    latest: np.ndarray | None = None
    frames = lost = bad = 0
    frame_started = False
    last_frame_time = time.monotonic()
    fps = 0.0
    source = "none"
    filter_on = True
    debug_on = False
    sequence = 0
    dtype = ">u2" if args.byte_order == "big" else "<u2"

    print(f"Listening on {args.bind}:{args.port}; FPGA target 192.168.10.3/24")
    print("Keys: F filter, D debug overlay, E stop latch, R clear, Q quit")
    try:
        while True:
            for _ in range(300):
                try:
                    payload, peer = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                if len(payload) != PAYLOAD_BYTES:
                    bad += 1
                    continue
                row = int.from_bytes(payload[:2], "big")
                if row >= HEIGHT:
                    bad += 1
                    continue
                source = peer[0]
                if row == 0:
                    if frame_started and not seen.all():
                        lost += 1
                    frame_started = True
                    seen.fill(False)
                if not frame_started:
                    continue
                frame[row] = np.frombuffer(payload, dtype=dtype, offset=2, count=WIDTH)
                seen[row] = True
                if row == HEIGHT - 1:
                    if seen.all():
                        latest = rgb565_to_bgr(frame)
                        frames += 1
                        now = time.monotonic()
                        delta = now - last_frame_time
                        if delta > 0:
                            fps = 1.0 / delta if fps == 0 else fps * 0.8 + 0.2 / delta
                        last_frame_time = now
                    else:
                        lost += 1
                    frame_started = False
            stats = {"source": source, "fps": fps, "frames": frames, "lost": lost, "bad": bad}
            cv2.imshow("VERA Vision Lab - FPGA UDP Viewer", render(latest, stats))
            key = cv2.waitKey(1) & 0xFF
            command: tuple[int, int] | None = None
            if key in (ord("q"), 27):
                break
            if key == ord("f"):
                filter_on = not filter_on
                command = (1, int(filter_on))
            elif key == ord("d"):
                debug_on = not debug_on
                command = (2, int(debug_on))
            elif key == ord("e"):
                command = (3, 1)
            elif key == ord("r"):
                command = (4, 0)
            if command:
                packet = command_packet(*command, sequence)
                sequence = (sequence + 1) & 255
                sock.sendto(packet, (args.command_address, COMMAND_PORT))
                print(f"Sent opcode={command[0]} value={command[1]} to {args.command_address}:{COMMAND_PORT}")
            time.sleep(0.005)
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
