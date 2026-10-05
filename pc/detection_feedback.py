"""Display-only DET1 feedback, independent of VRB1 controls and VUI1 telemetry."""
from dataclasses import dataclass
import binascii
import math
import struct

from video_stream import WIDTH, HEIGHT

MAGIC = b"DET1"
PACKET_BYTES = 32
MAX_RESULT_AGE_MS = 1500


@dataclass(frozen=True)
class DogBox:
    x0: float
    y0: float
    x1: float
    y1: float
    confidence: float


def encode_feedback(sequence: int, box: DogBox | None = None, *,
                    result_valid: bool = True, age_ms: int = 0,
                    inference_ms: int = 0, frame_number: int = 0) -> bytes:
    if not 0 <= age_ms <= MAX_RESULT_AGE_MS:
        result_valid, box = False, None
    if not result_valid:
        box = None
    coords = (0, 0, 0, 0)
    confidence = 0
    if box is not None:
        if not all(math.isfinite(x) for x in (box.x0, box.y0, box.x1, box.y1, box.confidence)):
            raise ValueError("nonfinite detection")
        coords = (max(0, min(WIDTH - 1, math.floor(box.x0))),
                  max(0, min(HEIGHT - 1, math.floor(box.y0))),
                  max(0, min(WIDTH, math.ceil(box.x1))),
                  max(0, min(HEIGHT, math.ceil(box.y1))))
        if coords[0] >= coords[2] or coords[1] >= coords[3] or not 0 <= box.confidence <= 1:
            raise ValueError("invalid box or confidence")
        confidence = round(box.confidence * 1000)
    payload = bytearray(PACKET_BYTES)
    payload[:4] = MAGIC
    payload[4:6] = bytes((1, int(box is not None) | (int(result_valid) << 1)))
    fields = (confidence, sequence & 65535, WIDTH, HEIGHT, *coords,
              max(0, min(MAX_RESULT_AGE_MS, age_ms)))
    for offset, value in zip(range(6, 24, 2), fields):
        struct.pack_into(">H", payload, offset, value)
    struct.pack_into(">IH", payload, 24, frame_number & 0xFFFFFFFF,
                     max(0, min(65535, inference_ms)))
    struct.pack_into(">H", payload, 30, binascii.crc_hqx(payload[:30], 0xFFFF))
    return bytes(payload)


def decode_feedback(payload: bytes) -> dict | None:
    if len(payload) != PACKET_BYTES or payload[:4] != MAGIC or payload[4] != 1:
        return None
    if binascii.crc_hqx(payload[:30], 0xFFFF) != int.from_bytes(payload[30:], "big"):
        return None
    flags = payload[5]
    confidence, sequence, width, height, x0, y0, x1, y1, age = struct.unpack(">9H", payload[6:24])
    if flags not in (0, 2, 3) or width != WIDTH or height != HEIGHT or confidence > 1000 or age > MAX_RESULT_AGE_MS:
        return None
    if flags & 1:
        if not (0 <= x0 < x1 <= WIDTH and 0 <= y0 < y1 <= HEIGHT):
            return None
    elif confidence or any((x0, y0, x1, y1)):
        return None
    return dict(result_valid=bool(flags & 2), box_valid=bool(flags & 1),
                confidence=confidence / 1000, sequence=sequence,
                box=(x0, y0, x1, y1), age_ms=age,
                frame_number=int.from_bytes(payload[24:28], "big"),
                inference_ms=int.from_bytes(payload[28:30], "big"))
