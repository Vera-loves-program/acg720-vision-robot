#!/usr/bin/env python3
"""Check the FPGA video UDP stream without OpenCV or any pip packages."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import time
from ui_telemetry import MAGIC as UI_MAGIC, parse_ui_telemetry

HEIGHT = 240
PAYLOAD_BYTES = 802


class VideoProbe:
    def __init__(self, expected_source: str) -> None:
        self.expected_source = expected_source
        self.packets = self.valid = self.other_source = self.bad = 0
        self.ui_packets = self.ui_bad_packets = 0
        self.complete = self.incomplete = 0
        self.sources: Counter[str] = Counter()
        self.lengths: Counter[int] = Counter()
        self.orders: Counter[str] = Counter()
        self.row_counts: Counter[int] = Counter()
        self.steps: Counter[int] = Counter()
        self.bad_headers: Counter[str] = Counter()
        self.examples: list[dict] = []
        self.previous_row: int | None = None
        self.video_packets = 0
        self.rows: set[int] | None = None
        self.first_header: str | None = None

    def consume(self, payload: bytes, source: str) -> None:
        self.packets += 1
        self.sources[source] += 1
        self.lengths[len(payload)] += 1
        if source != self.expected_source:
            self.other_source += 1
            return
        if payload.startswith(UI_MAGIC):
            if parse_ui_telemetry(payload) is None:
                self.ui_bad_packets += 1
            else:
                self.ui_packets += 1
            return
        if len(payload) != PAYLOAD_BYTES:
            self.bad += 1
            return
        self.video_packets += 1
        # Bounded raw examples help distinguish pixel bytes from actual headers.
        # These contain camera pixels; save locally, never commit to source Git.
        if len(self.examples) < 16:
            self.examples.append({"packet": self.packets, "payload_hex": payload.hex()})
        big = int.from_bytes(payload[:2], "big")
        little = int.from_bytes(payload[:2], "little")
        if big < HEIGHT:
            row = big
            if row:
                self.orders["big"] += 1
        elif little < HEIGHT:
            row = little
            self.orders["little"] += 1
        else:
            self.bad += 1
            self.bad_headers[payload[:2].hex(" ")] += 1
            return
        self.valid += 1
        self.row_counts[row] += 1
        if self.previous_row is not None:
            self.steps[(row - self.previous_row) % HEIGHT] += 1
        self.previous_row = row
        if self.first_header is None:
            self.first_header = payload[:10].hex(" ")
        if row == 0:
            if self.rows is not None:
                self.incomplete += 1
            self.rows = set()
        if self.rows is not None:
            self.rows.add(row)
            if len(self.rows) == HEIGHT:
                self.complete += 1
                self.rows = None

    def report(self) -> dict:
        return {
            "format": "acg720-video-probe-v1", "expected_source": self.expected_source,
            "packets": self.packets, "sources": dict(self.sources),
            "lengths": dict(self.lengths), "video_packets": self.video_packets,
            "accepted_header_packets": self.valid, "bad_packets": self.bad,
            "other_sources": self.other_source, "ui_packets": self.ui_packets,
            "invalid_ui_packets": self.ui_bad_packets,
            "complete_frames": self.complete, "incomplete_frames": self.incomplete,
            "unfinished_rows": len(self.rows) if self.rows is not None else 0,
            "byte_order_votes": dict(self.orders), "row_counts": dict(self.row_counts),
            "row_steps_mod_240": dict(self.steps),
            "missing_row_numbers": [r for r in range(HEIGHT) if r not in self.row_counts],
            "bad_header_counts": dict(self.bad_headers), "raw_examples": self.examples,
            "note": "A header in range 0..239 is only plausible; pixel bytes can also pass this check.",
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.10.3")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--expected-source", default="192.168.10.2")
    parser.add_argument("--report", type=Path,
                        help="Save JSON diagnostics including 16 raw video payloads (private camera data).")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    probe = VideoProbe(args.expected_source)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        try:
            sock.bind((args.bind, args.port))
        except OSError as exc:
            print(f"BIND FAILED: {exc}")
            print("Set the Ethernet IPv4 address to 192.168.10.3; close the viewer/other probes.")
            return 1
        print(f"Listening on {args.bind}:{args.port} for {args.seconds:g} seconds ...", flush=True)
        deadline = time.monotonic() + args.seconds
        try:
            while time.monotonic() < deadline:
                sock.settimeout(min(0.5, max(0.001, deadline - time.monotonic())))
                try:
                    payload, peer = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                probe.consume(payload, peer[0])
        except KeyboardInterrupt:
            pass
    print(f"Packets received: {probe.packets}")
    print(f"Sources: {dict(probe.sources)}")
    print(f"UDP payload lengths: {dict(probe.lengths)}")
    print(f"Valid video rows: {probe.valid}; bad packets: {probe.bad}; other sources: {probe.other_source}")
    print(f"R3 UI observations: {probe.ui_packets}; invalid UI packets: {probe.ui_bad_packets}")
    print(f"Complete frames: {probe.complete}; incomplete frames: {probe.incomplete}")
    print(f"Row header byte order votes: {dict(probe.orders)}")
    if probe.first_header:
        print(f"First valid payload, first 10 bytes: {probe.first_header}")
    print(f"Row 0 headers: {probe.row_counts[0]}; distinct row numbers: {len(probe.row_counts)}/240")
    print(f"Most common row numbers: {probe.row_counts.most_common(8)}")
    print(f"Most common row steps modulo 240: {probe.steps.most_common(5)}")
    print(f"Most common invalid 2-byte headers: {probe.bad_headers.most_common(5)}")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(probe.report(), indent=2) + "\n", encoding="utf-8")
        print(f"Diagnostic JSON saved: {args.report.resolve()} (contains camera pixel samples)")
    if probe.complete:
        print("PASS: complete video frames reached this computer.")
        if probe.orders["little"] > probe.orders["big"]:
            print("Little-endian transport detected. Current viewer/capture defaults auto-detect it.")
            print("For older scripts, use: --row-byte-order little --byte-order little")
        return 0
    if probe.video_packets:
        print("PARTIAL: video packets arrive, but no complete frame was observed.")
        if not probe.row_counts[0]:
            print("No row-0 header seen: frame assembly never started. Check FPGA row headers/FIFO boundaries.")
        print("A header in range 0..239 alone does not prove it is a real row number.")
        print("Compare known-working R2 on the same receiver before changing IP or dependencies.")
        return 2
    print("NO VIDEO: check cable, 1 Gbps link, Ethernet IPv4 address, firewall and FPGA network logic.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
