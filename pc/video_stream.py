#!/usr/bin/env python3
"""Conservative assembly of the FPGA's 400x240 RGB565 UDP row stream.

The wire protocol has a row number, but no frame ID, timestamp or CRC beyond
UDP. Accept only a contiguous 0..239 sequence from the configured source.
This catches loss and reordering; it cannot prove which sensor exposure a
packet belonged to if the sender itself supplied incorrect row numbers.
"""
from __future__ import annotations

from array import array
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import lru_cache
import sys
import time

WIDTH = 400
HEIGHT = 240
PAYLOAD_BYTES = 2 + WIDTH * 2


@dataclass(frozen=True)
class CompleteFrame:
    pixels: bytes
    source: str
    frame_number: int  # Receiver-local counter; not a camera frame ID.
    started_at: float
    received_at: float
    received_utc: str


class FrameAssembler:
    def __init__(self, expected_source: str = "192.168.10.2",
                 row_byte_order: str = "big", frame_timeout: float = 0.5) -> None:
        if row_byte_order not in ("big", "little"):
            raise ValueError("row_byte_order must be big or little")
        if frame_timeout <= 0:
            raise ValueError("frame_timeout must be positive")
        self.expected_source = expected_source
        self.row_byte_order = row_byte_order
        self.frame_timeout = frame_timeout
        self._pixels = bytearray(WIDTH * HEIGHT * 2)
        self._next_row: int | None = None
        self._started_at = 0.0
        self._stats = dict(packets=0, valid_rows=0, complete_frames=0,
                           incomplete_frames=0, bad_packets=0,
                           other_source_packets=0, out_of_order_packets=0,
                           duplicate_rows=0, ignored_rows=0, timed_out_frames=0)

    def _discard(self) -> None:
        if self._next_row is not None:
            self._stats["incomplete_frames"] += 1
        self._next_row = None

    def expire(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if self._next_row is not None and now - self._started_at > self.frame_timeout:
            self._stats["timed_out_frames"] += 1
            self._discard()

    def consume(self, payload: bytes, source: str,
                monotonic_time: float | None = None,
                utc_time: str | None = None) -> CompleteFrame | None:
        now = time.monotonic() if monotonic_time is None else monotonic_time
        self.expire(now)
        self._stats["packets"] += 1
        if source != self.expected_source:
            self._stats["other_source_packets"] += 1
            return None
        if len(payload) != PAYLOAD_BYTES:
            self._stats["bad_packets"] += 1
            return None
        row = int.from_bytes(payload[:2], self.row_byte_order)
        if row >= HEIGHT:
            self._stats["bad_packets"] += 1
            return None
        self._stats["valid_rows"] += 1
        if row == 0:
            self._discard()
            self._next_row = 0
            self._started_at = now
        if self._next_row is None:
            self._stats["ignored_rows"] += 1
            return None
        if row != self._next_row:
            self._stats["out_of_order_packets"] += 1
            if row < self._next_row:
                self._stats["duplicate_rows"] += 1
            self._discard()
            return None
        offset = row * WIDTH * 2
        self._pixels[offset:offset + WIDTH * 2] = payload[2:]
        self._next_row += 1
        if self._next_row != HEIGHT:
            return None
        self._next_row = None
        self._stats["complete_frames"] += 1
        return CompleteFrame(
            pixels=bytes(self._pixels), source=source,
            frame_number=self._stats["complete_frames"],
            started_at=self._started_at, received_at=now,
            received_utc=utc_time or datetime.now(timezone.utc).isoformat(),
        )

    def snapshot_stats(self) -> dict[str, int]:
        return self._stats.copy()


def rgb565_bytes_to_bgr(pixels: bytes, byte_order: str = "big"):
    """Convert a complete frame for OpenCV; no resizing, filter or overlay."""
    import numpy as np
    if byte_order not in ("big", "little"):
        raise ValueError("byte_order must be big or little")
    if len(pixels) != WIDTH * HEIGHT * 2:
        raise ValueError("not a complete RGB565 frame")
    words = np.frombuffer(pixels, dtype=">u2" if byte_order == "big" else "<u2")
    words = words.reshape(HEIGHT, WIDTH)
    red = ((words >> 11) & 31).astype(np.uint16) * 255 // 31
    green = ((words >> 5) & 63).astype(np.uint16) * 255 // 63
    blue = (words & 31).astype(np.uint16) * 255 // 31
    return np.stack((blue, green, red), axis=-1).astype(np.uint8)


@lru_cache(maxsize=1)
def _rgb_lookup() -> tuple[bytes, ...]:
    return tuple(bytes((((word >> 11) & 31) * 255 // 31,
                        ((word >> 5) & 63) * 255 // 63,
                        (word & 31) * 255 // 31)) for word in range(65536))


def rgb565_bytes_to_rgb(pixels: bytes, byte_order: str = "big") -> bytes:
    """Standard-library RGB conversion for lossless headless PNG capture."""
    if byte_order not in ("big", "little"):
        raise ValueError("byte_order must be big or little")
    if len(pixels) != WIDTH * HEIGHT * 2:
        raise ValueError("not a complete RGB565 frame")
    words = array("H")
    words.frombytes(pixels)
    if sys.byteorder != byte_order:
        words.byteswap()
    lookup = _rgb_lookup()
    return b"".join(lookup[word] for word in words)
