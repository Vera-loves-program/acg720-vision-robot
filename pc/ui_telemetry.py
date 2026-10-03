"""Decode R3 FPGA touch/UI observations; this is not a video-frame ACK."""
from __future__ import annotations

from dataclasses import dataclass

MAGIC = b"VUI1"
PAYLOAD_BYTES = 32


@dataclass(frozen=True)
class UITelemetry:
    gaussian_on: bool
    debug: bool
    selected: bool
    touch_ready: bool
    touch_identified: bool
    stop: bool
    zoom_code: int
    contacts: int
    selection_counter: int
    capture_counter: int
    clear_counter: int
    camera_x: int
    camera_y: int
    touch_error: int
    tx_sequence: int
    uptime_ticks: int


def parse_ui_telemetry(payload: bytes) -> UITelemetry | None:
    """Return only version-1 packets with valid fixed fields and reserved bits."""
    if len(payload) != PAYLOAD_BYTES or payload[:4] != MAGIC or payload[4] != 1:
        return None
    flags = payload[5]
    if flags & 0xC0 or payload[6] > 1 or payload[7] > 5:
        return None
    if payload[19] != 0 or payload[28:32] != b"\x00" * 4:
        return None
    camera_x = int.from_bytes(payload[14:16], "big")
    camera_y = int.from_bytes(payload[16:18], "big")
    if camera_x >= 800 or camera_y >= 480:
        return None
    return UITelemetry(
        gaussian_on=bool(flags & 1), debug=bool(flags & 2),
        selected=bool(flags & 4), touch_ready=bool(flags & 8),
        touch_identified=bool(flags & 16), stop=bool(flags & 32),
        zoom_code=payload[6], contacts=payload[7],
        selection_counter=int.from_bytes(payload[8:10], "big"),
        capture_counter=int.from_bytes(payload[10:12], "big"),
        clear_counter=int.from_bytes(payload[12:14], "big"),
        camera_x=camera_x, camera_y=camera_y, touch_error=payload[18],
        tx_sequence=int.from_bytes(payload[20:24], "big"),
        uptime_ticks=int.from_bytes(payload[24:28], "big"),
    )
