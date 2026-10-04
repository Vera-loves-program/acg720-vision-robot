#!/usr/bin/env python3
"""Show FPGA UDP video with a local visual-debug interaction preview.

Zoom mode uses double-click to set a source-coordinate focus. Visual Debug uses
double-click to select/cancel a target intent. No detector, tracker, NPU inference
or motor control runs here. Preview zoom never changes received/dataset pixels.
Existing VRB1 control commands have no acknowledgement.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
import socket
import time
from typing import TYPE_CHECKING

from video_stream import FrameAssembler, HEIGHT, WIDTH, rgb565_bytes_to_bgr
from ui_telemetry import MAGIC as UI_MAGIC, UITelemetry, parse_ui_telemetry
from highgui_window import WindowCloseMonitor, report_gui

if TYPE_CHECKING:
    import numpy as np

COMMAND_PORT = 5001
COMMAND_BROADCAST = "192.168.10.255"
WINDOW_NAME = "VERA Vision Lab - FPGA UDP Viewer"
# AUTOSIZE keeps mouse coordinates equal to these displayed canvas coordinates.
VIDEO_RECT = (20, 84, 820, 564)
BUTTON_RECTS = {
    "debug": (852, 320, 1036, 350),
    "zoom_mode": (852, 356, 1036, 386),
    "zoom_out": (852, 392, 940, 422),
    "zoom_in": (948, 392, 1036, 422),
    "filter": (852, 428, 1036, 458),
    "reset_view": (852, 464, 1036, 494),
    "cancel_target": (852, 500, 1036, 530),
    "stop": (852, 536, 1036, 566),
    "clear": (852, 572, 1036, 602),
}


def command_packet(opcode: int, value: int, sequence: int) -> bytes:
    body = b"VRB1" + bytes((opcode, value, sequence & 255))
    checksum = 0
    for byte in body:
        checksum ^= byte
    return body + bytes((checksum,))


def contains(rect: tuple[int, int, int, int], x: int, y: int) -> bool:
    left, top, right, bottom = rect
    return left <= x < right and top <= y < bottom


@dataclass(frozen=True)
class TargetIntent:
    """Future tracker input; all coordinates refer to the received 400x240 frame.

    The box is a provisional selection window, not a detected object boundary.
    Consumers must identify a target and report tracking validity separately.
    """

    point: tuple[float, float]
    roi: tuple[int, int, int, int]
    requested_at: float
    source_size: tuple[int, int] = (WIDTH, HEIGHT)


@dataclass
class ViewerState:
    debug_mode: bool = False
    filter_requested: bool | None = None
    stop_requested: bool | None = None
    zoom: float = 1.0
    center_x: float = WIDTH / 2
    center_y: float = HEIGHT / 2
    zoom_mode: bool = False
    zoom_focus: tuple[float, float] | None = None
    lcd_crop: tuple[int, int, int, int] | None = None
    target: TargetIntent | None = None
    has_frame: bool = False
    ui_telemetry: UITelemetry | None = None
    ui_seen_at: float = 0.0
    notice: str = "Control state is unconfirmed: FPGA sends no acknowledgements."
    pending_commands: list[tuple[int, int]] = field(default_factory=list)

    def crop_bounds(self) -> tuple[int, int, int, int]:
        if self.lcd_crop is not None:
            return self.lcd_crop
        width = max(1, min(WIDTH, round(WIDTH / self.zoom)))
        height = max(1, min(HEIGHT, round(HEIGHT / self.zoom)))
        left = max(0, min(WIDTH - width, round(self.center_x - width / 2)))
        top = max(0, min(HEIGHT - height, round(self.center_y - height / 2)))
        return left, top, left + width, top + height

    def source_point(self, display_x: int, display_y: int) -> tuple[float, float] | None:
        if not contains(VIDEO_RECT, display_x, display_y):
            return None
        left, top, right, bottom = self.crop_bounds()
        x0, y0, x1, y1 = VIDEO_RECT
        return (left + (display_x - x0) / (x1 - x0) * (right - left),
                top + (display_y - y0) / (y1 - y0) * (bottom - top))

    def zoom_at(self, display_x: int, display_y: int, direction: int) -> None:
        point = self.source_point(display_x, display_y)
        if point is None or direction == 0 or not self.has_frame:
            return
        self.zoom_mode = True
        self.zoom_focus = point
        self.change_zoom(direction)

    def _set_preview_zoom(self, next_zoom: float, focus: tuple[float, float]) -> None:
        """Center a local crop on a source point, clamping only at image edges."""
        self.lcd_crop = None
        next_zoom = max(1.0, min(4.0, next_zoom))
        width, height = round(WIDTH / next_zoom), round(HEIGHT / next_zoom)
        self.zoom = next_zoom
        self.center_x = max(width / 2, min(WIDTH - width / 2, focus[0]))
        self.center_y = max(height / 2, min(HEIGHT - height / 2, focus[1]))

    def set_zoom_focus(self, source_x: float, source_y: float) -> None:
        self.zoom_focus = (max(0.0, min(WIDTH - 1.0, source_x)),
                           max(0.0, min(HEIGHT - 1.0, source_y)))
        self._set_preview_zoom(self.zoom, self.zoom_focus)
        self.notice = "Zoom focus set; +/- changes local preview only. Source pixels are unchanged."

    def change_zoom(self, direction: int) -> None:
        if direction == 0 or not self.has_frame:
            return
        if self.zoom_focus is None:
            self.zoom_focus = self.target.point if self.target else (self.center_x, self.center_y)
        self._set_preview_zoom(self.zoom * (1.25 if direction > 0 else 0.8), self.zoom_focus)
        self.notice = f"Local preview {self.zoom:.2f}x around focus; no FPGA zoom command sent."

    def double_click(self, display_x: int, display_y: int) -> TargetIntent | None:
        point = self.source_point(display_x, display_y)
        if point is None or not self.has_frame:
            return None
        if self.zoom_mode:
            self.set_zoom_focus(*point)
            return None
        return self.select_target(display_x, display_y)

    def select_target(self, display_x: int, display_y: int) -> TargetIntent | None:
        if self.zoom_mode or not self.debug_mode or not self.has_frame:
            return None
        point = self.source_point(display_x, display_y)
        if point is None:
            return None
        if self.target is not None and contains(self.target.roi, *point):
            self.action("cancel_target")
            return None
        return self.select_source_target(*point)

    def select_source_target(self, source_x: float, source_y: float) -> TargetIntent:
        point = (max(0.0, min(WIDTH - 1.0, source_x)), max(0.0, min(HEIGHT - 1.0, source_y)))
        # Explicit initial ROI; its size does not imply object recognition.
        left = max(0, min(WIDTH - 80, round(point[0] - 40)))
        top = max(0, min(HEIGHT - 80, round(point[1] - 40)))
        self.target = TargetIntent(point, (left, top, left + 80, top + 80), time.monotonic())
        self.notice = "Target selected; tracker is not connected. No motion command sent."
        return self.target

    def apply_telemetry(self, packet: UITelemetry) -> bool:
        """Apply current UI observations and new event counters, without VRB1 echoes.

        The first packet (or an apparent FPGA reboot) sets a baseline. It cannot
        replay an earlier capture press. Duplicate/out-of-order packets are ignored.
        """
        previous = self.ui_telemetry
        baseline = previous is None
        if previous is not None:
            distance = (packet.tx_sequence - previous.tx_sequence) & 0xFFFFFFFF
            if distance == 0:
                return False
            if distance >= 0x80000000:
                # A large uptime drop alongside a backwards sequence indicates
                # a reboot. Small drops are ordinary UDP reordering.
                if packet.uptime_ticks + 20 < previous.uptime_ticks:
                    baseline = True
                else:
                    return False
        self.ui_telemetry = packet
        self.ui_seen_at = time.monotonic()
        self.debug_mode = packet.debug
        def transform(observation: UITelemetry) -> tuple:
            if observation.version == 2:
                return (2, observation.zoom_step_q8, observation.view_origin_x,
                        observation.view_origin_y, observation.zoom_mode)
            return (1, observation.zoom_code)

        if baseline or transform(previous) != transform(packet):
            # Fresh sequence numbers with an unchanged transform must not
            # continuously undo local mouse/button zoom.
            if packet.version == 2:
                step = packet.zoom_step_q8
                crop_width = (800 * step + 255) // 256
                crop_height = (480 * step + 255) // 256
                left, top = packet.view_origin_x // 2, packet.view_origin_y // 2
                right = (packet.view_origin_x + crop_width + 1) // 2
                bottom = (packet.view_origin_y + crop_height + 1) // 2
                self.lcd_crop = (left, top, right, bottom)
                self.zoom = 256 / step
                self.center_x, self.center_y = (left + right) / 2, (top + bottom) / 2
                self.zoom_mode = packet.zoom_mode
            else:
                self._set_preview_zoom(1.0 if packet.zoom_code == 0 else 2.0,
                                       (WIDTH / 2, HEIGHT / 2))
            self.zoom_focus = (self.center_x, self.center_y)
        if baseline:
            self.target = None
            if packet.selected:
                self.select_source_target(packet.camera_x // 2, packet.camera_y // 2)
            self.notice = "FPGA touch/UI state synchronized. Old capture presses were not replayed."
            return True
        if packet.clear_counter != previous.clear_counter:
            self.target = None
            self.notice = "LCD requested target cancellation; local target cleared."
        if packet.selection_counter != previous.selection_counter:
            if packet.selected:
                self.select_source_target(packet.camera_x // 2, packet.camera_y // 2)
                self.notice = "LCD selected target; tracker is not connected. No motion command sent."
            else:
                self.target = None
                self.notice = "LCD cancelled target by double-click; local target cleared."
        if packet.capture_counter != previous.capture_counter:
            self.notice = "LCD CAPTURE pressed: stop viewer and run capture_dataset.py to save photos."
        return True

    def action(self, name: str) -> None:
        if name == "debug":
            self.debug_mode = not self.debug_mode
            self.pending_commands.append((2, int(self.debug_mode)))
            self.notice = "Visual debug: double-click a target. FPGA debug request has no ACK."
        elif name == "zoom_mode":
            self.zoom_mode = not self.zoom_mode
            self.notice = ("Zoom mode: double-click a focus, then use +/- or wheel. Local preview only."
                           if self.zoom_mode else
                           "Zoom mode OFF. Visual Debug double-click selects/cancels an ROI; no tracker connected.")
        elif name in ("zoom_in", "zoom_out"):
            self.zoom_mode = True
            self.change_zoom(1 if name == "zoom_in" else -1)
        elif name == "filter":
            self.filter_requested = True if self.filter_requested is None else not self.filter_requested
            self.pending_commands.append((1, int(self.filter_requested)))
            self.notice = "Filter request sent without ACK; capture images reflect received pixels."
        elif name == "reset_view":
            self._set_preview_zoom(1.0, (WIDTH / 2, HEIGHT / 2))
            self.zoom_focus = None
            self.zoom_mode = False
            self.notice = "Preview view reset. Source image and target coordinates are unchanged."
            if self.has_recent_ui():
                self.pending_commands.append((6, 0))
                self.notice = "Preview reset; LCD view reset requested without acknowledgement."
        elif name == "cancel_target":
            self.target = None
            self.notice = "Target selection cancelled. Tracker is not connected."
            if self.has_recent_ui():
                self.pending_commands.append((5, 0))
                self.notice = "Local target cancelled; LCD selection-clear requested without acknowledgement."
        elif name == "stop":
            self.stop_requested = True
            self.pending_commands.append((3, 1))
            self.notice = "STOP requested; no acknowledgement or motor cut-off is connected."
        elif name == "clear":
            self.stop_requested = False
            self.pending_commands.append((4, 0))
            self.notice = "STOP clear requested; FPGA latch state is unconfirmed."

    def has_recent_ui(self) -> bool:
        return self.ui_telemetry is not None and 0 <= time.monotonic() - self.ui_seen_at <= 3.0

    def click(self, x: int, y: int) -> None:
        for name, rect in BUTTON_RECTS.items():
            if contains(rect, x, y):
                self.action(name)
                break


def rgb565_to_bgr(pixels: np.ndarray) -> np.ndarray:
    """Compatibility helper; conversion always preserves the source pixel grid."""
    import numpy as np
    red = ((pixels >> 11) & 31).astype(np.uint16) * 255 // 31
    green = ((pixels >> 5) & 63).astype(np.uint16) * 255 // 63
    blue = (pixels & 31).astype(np.uint16) * 255 // 31
    return np.stack((blue, green, red), axis=-1).astype(np.uint8)


def render(frame: np.ndarray | None, stats: dict, state: ViewerState) -> np.ndarray:
    import cv2
    import numpy as np
    cream, ink, blue, yellow = (240, 244, 244), (23, 23, 23), (173, 98, 0), (92, 209, 255)
    white, red = (255, 255, 255), (77, 77, 255)
    canvas = np.full((720, 1060, 3), cream, dtype=np.uint8)
    canvas[4::30, 4::30] = (204, 204, 204)

    def text(label: str, x: int, y: int, scale: float = 0.45, color=ink, weight: int = 1) -> None:
        cv2.putText(canvas, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, weight, cv2.LINE_AA)

    cv2.rectangle(canvas, (0, 0), (1059, 60), blue, -1)
    cv2.rectangle(canvas, (0, 59), (1059, 63), ink, -1)
    text("VERA / VISION LAB", 24, 39, 0.86, white, 2)
    text("ACG720 / ORANGE PI / UDP", 700, 38, 0.45, white)
    cv2.rectangle(canvas, (22, 86), (834, 578), ink, -1)
    cv2.rectangle(canvas, (14, 78), (826, 570), ink, 4)
    if frame is None:
        text("WAITING FOR FPGA VIDEO", 205, 325, 0.8, ink, 2)
    else:
        left, top, right, bottom = state.crop_bounds()
        # Resize only the preview. Never assign this crop back to the source frame.
        canvas[84:564, 20:820] = cv2.resize(frame[top:bottom, left:right], (800, 480),
                                          interpolation=cv2.INTER_NEAREST)
        if state.zoom_mode and state.zoom_focus is not None:
            view = canvas[84:564, 20:820]
            px = round((state.zoom_focus[0] - left) * 800 / (right - left))
            py = round((state.zoom_focus[1] - top) * 480 / (bottom - top))
            if 0 <= px < 800 and 0 <= py < 480:
                cv2.line(view, (px - 12, py), (px + 12, py), blue, 2)
                cv2.line(view, (px, py - 12), (px, py + 12), blue, 2)
        if state.debug_mode and state.target is not None:
            view = canvas[84:564, 20:820]
            rx0, ry0, rx1, ry1 = state.target.roi
            sx, sy = 800 / (right - left), 480 / (bottom - top)
            box = (round((rx0 - left) * sx), round((ry0 - top) * sy),
                   round((rx1 - left) * sx), round((ry1 - top) * sy))
            cv2.rectangle(view, box[:2], box[2:], yellow, 2)
            px = round((state.target.point[0] - left) * sx)
            py = round((state.target.point[1] - top) * sy)
            cv2.line(view, (px - 9, py), (px + 9, py), yellow, 2)
            cv2.line(view, (px, py - 9), (px, py + 9), yellow, 2)
    cv2.rectangle(canvas, (850, 86), (1054, 630), ink, -1)
    cv2.rectangle(canvas, (842, 78), (1046, 622), yellow, -1)
    cv2.rectangle(canvas, (842, 78), (1046, 622), ink, 4)
    text("VISUAL / CONTROL", 852, 109, 0.5, blue, 2)
    text(f"SOURCE {stats['source']}", 852, 138, 0.38)
    text("UDP 400 x 240 RGB565", 852, 160, 0.38)
    text(f"RX FPS {stats['fps']:.1f} VIEW {state.zoom:.2f}x", 852, 188, 0.38)
    text(f"FRAMES {stats['frames']} / LOST {stats['lost']}", 852, 214, 0.37)
    text(f"BAD PACKETS {stats['bad']}", 852, 240, 0.38)
    if frame is not None and stats.get("age", 0.0) > 2.0:
        text("VIDEO STALE", 852, 265, 0.44, red, 2)
    elif state.zoom_mode:
        text("ZOOM FOCUS MODE", 852, 265, 0.40, blue, 2)
    elif state.target is not None:
        text("TARGET SELECTED", 852, 265, 0.42, blue, 2)
    else:
        text("NO TARGET SELECTED", 852, 265, 0.38)
    text("TRACKER NOT CONNECTED", 852, 287, 0.32)
    telemetry = state.ui_telemetry
    if telemetry is not None and time.monotonic() - state.ui_seen_at < 3.0:
        observed_filter = "ON" if telemetry.gaussian_on else "OFF"
        observed_touch = "ID" if telemetry.touch_identified else "WAIT"
        text(f"FPGA GAUSS {observed_filter} TOUCH {observed_touch}", 852, 310, 0.30, blue)
    else:
        text("FPGA STATE UNCONFIRMED", 852, 310, 0.31, red)
    filter_label = "?" if state.filter_requested is None else ("ON" if state.filter_requested else "OFF")
    labels = {
        "debug": f"VISUAL DEBUG {'ON' if state.debug_mode else 'OFF'}",
        "zoom_mode": f"ZOOM MODE {'ON' if state.zoom_mode else 'OFF'}",
        "zoom_out": "- ZOOM",
        "zoom_in": "+ ZOOM",
        "filter": f"GAUSS REQUEST {filter_label}",
        "reset_view": "RESET VIEW / 1x",
        "cancel_target": "CANCEL TARGET",
        "stop": "REQUEST STOP",
        "clear": "REQUEST CLEAR",
    }
    for name, (x0, y0, x1, y1) in BUTTON_RECTS.items():
        active = (name == "debug" and state.debug_mode) or (name == "zoom_mode" and state.zoom_mode)
        color = blue if active else (red if name == "stop" else white)
        cv2.rectangle(canvas, (x0 + 3, y0 + 3), (x1 + 3, y1 + 3), ink, -1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color, -1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), ink, 2)
        text(labels[name], x0 + 7, y0 + 21, 0.36, white if active else ink)
    text("F / D / E / R / Q", 852, 612, 0.35)
    text("PREVIEW ZOOM ONLY / SOURCE PIXELS UNCHANGED", 20, 602, 0.45, blue, 1)
    if telemetry is None:
        text("No FPGA touch telemetry received. Local interactions remain available.", 20, 626, 0.40)
    else:
        age = time.monotonic() - state.ui_seen_at
        observed = "LAST OBSERVED" if age < 3.0 else "STALE OBSERVATION"
        text(f"{observed}: GAUSS {int(telemetry.gaussian_on)} / STOP {int(telemetry.stop)} / "
             f"TOUCH {telemetry.contacts} / ERROR {telemetry.touch_error} / AGE {age:.1f}s",
             20, 626, 0.40, blue if age < 3.0 else red)
    text("Z: zoom mode | +/-: zoom | double-click: focus or ROI | V: reset | C: cancel | Q: quit", 20, 652, 0.43)
    text(state.notice[:113], 20, 680, 0.41,
         red if state.stop_requested or (telemetry is not None and telemetry.stop) else ink)
    text("Selections are local UI intents. No object recognition, tracking or motor motion is active.",
         20, 706, 0.43)
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local Ethernet address or 0.0.0.0")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--command-address", default=COMMAND_BROADCAST)
    parser.add_argument("--expected-source", default="192.168.10.2")
    parser.add_argument("--byte-order", choices=("auto", "big", "little"), default="auto",
                        help="Pixel order; auto follows the detected row transport order.")
    parser.add_argument("--row-byte-order", choices=("auto", "big", "little"), default="auto")
    args = parser.parse_args()
    try:
        import cv2
    except ImportError:
        parser.error("OpenCV is missing. Use the project's Python environment and install pc/requirements.txt.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Only one video receiver may own the port: do not share it with the capture tool.
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        sock.close()
        parser.error(f"Cannot bind {args.bind}:{args.port}: {exc}. "
                     "Check the local Ethernet address and close other receivers/capture tools.")
    sock.setblocking(False)
    assembler = FrameAssembler(expected_source=args.expected_source, row_byte_order=args.row_byte_order)
    latest = None
    state = ViewerState()
    source = "none"
    last_frame_time = 0.0
    fps = 0.0
    fps_window_time = time.monotonic()
    fps_window_frames = 0
    sequence = 0
    bad_telemetry = 0
    announced_order = None

    def mouse_event(event: int, x: int, y: int, flags: int, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            state.click(x, y)
        elif event == cv2.EVENT_LBUTTONDBLCLK:
            selected = state.double_click(x, y)
            if selected:
                print(f"Selected target intent: point={selected.point}, ROI={selected.roi}; tracker not connected.")
            elif contains(VIDEO_RECT, x, y) and state.has_frame:
                print(state.notice)
        elif event == cv2.EVENT_MOUSEWHEEL:
            delta = (flags >> 16) & 0xFFFF
            if delta >= 0x8000:
                delta -= 0x10000
            state.zoom_at(x, y, 1 if delta > 0 else (-1 if delta < 0 else 0))

    print(f"Listening on {args.bind}:{args.port}; expected FPGA source {args.expected_source}")
    print("Zoom mode: double-click focus, +/- or wheel changes preview. Debug: double-click selects/cancels an ROI.")
    print("No detector/tracker is connected. Preview zoom never changes received or saved pixels.")
    print("Keys: Z zoom mode, +/- zoom, F filter request, D debug, E stop request, R clear request, V reset, C cancel, Q quit")
    print("Stop this viewer before running capture_dataset.py; both receive the same UDP port.")
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, mouse_event)
        report_gui(cv2)
        window_monitor = WindowCloseMonitor(cv2, WINDOW_NAME)
        while True:
            for _ in range(1000):
                try:
                    payload, peer = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                if payload.startswith(UI_MAGIC):
                    if peer[0] == args.expected_source:
                        packet = parse_ui_telemetry(payload)
                        if packet is None:
                            bad_telemetry += 1
                        elif state.apply_telemetry(packet):
                            source = peer[0]
                    continue
                complete = assembler.consume(payload, peer[0])
                if complete is not None:
                    pixel_order = complete.row_byte_order if args.byte_order == "auto" else args.byte_order
                    if complete.row_byte_order != announced_order:
                        announced_order = complete.row_byte_order
                        print(f"FORMAT: row={announced_order}, pixels={pixel_order}", flush=True)
                    latest = rgb565_bytes_to_bgr(complete.pixels, byte_order=pixel_order)
                    source = complete.source
                    state.has_frame = True
                    fps_window_frames += 1
                    last_frame_time = time.monotonic()
            assembler.expire()
            elapsed = time.monotonic() - fps_window_time
            if elapsed >= 1.0:
                fps = fps_window_frames / elapsed
                fps_window_time = time.monotonic()
                fps_window_frames = 0
            counters = assembler.snapshot_stats()
            stats = {"source": source, "fps": fps, "frames": counters["complete_frames"],
                     "lost": counters["incomplete_frames"], "bad": counters["bad_packets"] + bad_telemetry,
                     "age": time.monotonic() - last_frame_time if last_frame_time else math.inf}
            state.has_frame = latest is not None and stats["age"] <= 2.0
            cv2.imshow(WINDOW_NAME, render(latest, stats, state))
            key = cv2.waitKey(1) & 0xFF
            if ord("A") <= key <= ord("Z"):
                key += ord("a") - ord("A")
            if key in (ord("q"), 27):
                print("Viewer stopped: Q/Esc pressed.", flush=True)
                break
            if window_monitor.closed():
                print("Viewer stopped: image window closed.", flush=True)
                break
            key_actions = {ord("f"): "filter", ord("d"): "debug", ord("e"): "stop", ord("r"): "clear",
                           ord("v"): "reset_view", ord("c"): "cancel_target", ord("z"): "zoom_mode",
                           ord("+"): "zoom_in", ord("="): "zoom_in", ord("-"): "zoom_out"}
            if key in key_actions:
                state.action(key_actions[key])
            while state.pending_commands:
                command = state.pending_commands.pop(0)
                packet = command_packet(*command, sequence)
                sequence = (sequence + 1) & 255
                try:
                    sock.sendto(packet, (args.command_address, COMMAND_PORT))
                    print(f"Requested opcode={command[0]} value={command[1]}; FPGA state unconfirmed (no ACK).")
                except OSError as exc:
                    state.notice = f"Command send failed: {exc}"
                    print(state.notice)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("Viewer stopped: Ctrl+C pressed.", flush=True)
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
