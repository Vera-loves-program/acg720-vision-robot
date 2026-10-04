"""Decode FPGA VUI1 version-1/2 observations; this is not a video-frame ACK."""
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
    version: int = 1
    zoom_mode: bool = False
    zoom_step_q8: int = 256
    view_origin_x: int = 0
    view_origin_y: int = 0


def parse_ui_telemetry(payload: bytes) -> UITelemetry | None:
    """Validate both packet versions, including V2's actual LCD crop geometry.

    V2 packs reserved[3], zoom_mode[1], step_q8[9], origin_y[9], origin_x[10]
    into the final big-endian word. The source footprint uses ceil(width*step/256).
    """
    if len(payload) != PAYLOAD_BYTES or payload[:4] != MAGIC or payload[4] not in (1, 2):
        return None
    flags = payload[5]
    if flags & 0xC0 or payload[6] > 1 or payload[7] > 5:
        return None
    if payload[19] != 0:
        return None
    version = payload[4]
    if version == 1:
        if payload[28:32] != b"\x00" * 4:
            return None
        zoom_mode = False
        step = 256 if payload[6] == 0 else 128
        origin_x, origin_y = (0, 0) if payload[6] == 0 else (200, 120)
    else:
        packed = int.from_bytes(payload[28:32], "big")
        if packed >> 29:
            return None
        zoom_mode = bool(packed & (1 << 28))
        step = (packed >> 19) & 0x1FF
        origin_y = (packed >> 10) & 0x1FF
        origin_x = packed & 0x3FF
        if not 64 <= step <= 256 or payload[6] != int(step != 256):
            return None
        crop_width = (800 * step + 255) // 256
        crop_height = (480 * step + 255) // 256
        if origin_x > 800 - crop_width or origin_y > 480 - crop_height:
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
        version=version, zoom_mode=zoom_mode, zoom_step_q8=step,
        view_origin_x=origin_x, view_origin_y=origin_y,
    )
