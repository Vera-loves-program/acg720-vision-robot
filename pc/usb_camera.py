"""Newest-frame USB/V4L2 capture. Host timestamps are not exposure timestamps."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class CameraFrame:
    sequence: int
    arrived_at: float
    image: object
    gimbal: dict


class UsbCamera:
    def __init__(self, device, width=640, height=480, fps=15, pose_snapshot=None):
        self.device = str(device)
        self.width, self.height, self.fps = width, height, fps
        self.pose_snapshot = pose_snapshot or (lambda: {})
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest = None
        self.error = None
        self.actual = {}
        self.thread = threading.Thread(target=self._run, daemon=True, name='usb-capture')

    def start(self):
        self.thread.start()

    def _run(self):
        capture = None
        try:
            import cv2
            capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise RuntimeError(f'Cannot open {self.device}; check video-group permission and v4l2-ctl --list-devices')
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Driver-dependent, not a latency guarantee.
            self.actual = dict(device=self.device, backend=capture.getBackendName(),
                               width=capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                               height=capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
                               fps=capture.get(cv2.CAP_PROP_FPS),
                               timestamp='host_monotonic_after_read; exposure/buffer delay unknown')
            sequence = failures = 0
            while not self.stop.is_set():
                ok, image = capture.read()
                arrived = time.monotonic()
                if not ok or image is None or not image.size:
                    failures += 1
                    if failures >= 10:
                        raise RuntimeError('USB camera stopped providing frames')
                    self.stop.wait(.05)
                    continue
                failures = 0
                sequence += 1
                frame = CameraFrame(sequence, arrived, image, self.pose_snapshot())
                with self.condition:
                    self.latest = frame
                    self.condition.notify_all()
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()
            with self.condition:
                self.condition.notify_all()
        finally:
            if capture is not None:
                capture.release()

    def newest(self, after=0, timeout=.1):
        with self.condition:
            self.condition.wait_for(lambda: self.stop.is_set() or
                                    (self.latest is not None and self.latest.sequence > after), timeout)
            return self.latest if self.latest is not None and self.latest.sequence > after else None

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread.is_alive():
            self.thread.join(timeout=2)


def hardware_inventory():
    """Read device names and PWM device-tree links; never export or enable PWM."""
    cameras = []
    for path in sorted(Path('/sys/class/video4linux').glob('video*')):
        try:
            name = (path / 'name').read_text().strip()
        except OSError:
            name = 'unavailable'
        cameras.append(dict(device='/dev/' + path.name, name=name))
    pwms = []
    for path in sorted(Path('/sys/class/pwm').glob('pwmchip*')):
        try:
            channels = int((path / 'npwm').read_text().strip())
        except (OSError, ValueError):
            channels = None
        pwms.append(dict(chip=str(path), channels=channels,
                         device=str((path / 'device').resolve()),
                         of_node=str((path / 'device/of_node').resolve()),
                         exported=[p.name for p in path.glob('pwm[0-9]*')]))
    return dict(cameras=cameras, pwm_chips=pwms,
                note='pwmchip numbering is kernel-dependent; confirm controller/pinmux before configuring outputs')
