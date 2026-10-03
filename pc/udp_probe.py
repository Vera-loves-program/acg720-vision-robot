#!/usr/bin/env python3
"""Check the FPGA video UDP stream without OpenCV or any pip packages."""
from __future__ import annotations

import argparse
from collections import Counter
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
            return
        self.valid += 1
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.10.3")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--expected-source", default="192.168.10.2")
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
    if probe.complete:
        print("PASS: complete video frames reached this computer.")
        if probe.orders["little"] > probe.orders["big"]:
            print("Use viewer options: --row-byte-order little --byte-order little")
        return 0
    if probe.valid:
        print("PARTIAL: video packets arrive, but no complete frame was observed. Check loss/order.")
        return 2
    print("NO VIDEO: check cable, 1 Gbps link, Ethernet IPv4 address, firewall and FPGA network logic.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
