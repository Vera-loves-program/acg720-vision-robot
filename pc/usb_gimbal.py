#!/usr/bin/env python3
"""USB human/dog inspection, raw capture and calibrated two-axis tracking.

Serves loopback HTTP for SSH forwarding. Default PWM backend is a mock;
opening a camera or a browser never enables a physical servo by itself.
"""
from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import hashlib
import json
import math
from pathlib import Path
import secrets
import signal
import threading
import time
from urllib.parse import urlsplit

from gimbal_control import GimbalController, GimbalError, SysfsPWMBackend, default_config, validate_config
from target_tracking import CameraCalibration, TargetTracker, TrackerConfig, gimbal_centering_delta, mount_image_sign
from gimbal_image_calibration import normalize_sample, fit_image_calibration, calibrated_centering_delta, calibrated_centering_offset
from usb_camera import UsbCamera, hardware_inventory

ROOT = Path(__file__).resolve().parents[1]
BROWSER_HEARTBEAT_ERROR = '浏览器心跳已过期，PWM已关闭；连接恢复后请重新点击开启本轴PWM'


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def fpga_initial_tilt(pulse=None,angle=None):
    """Migrate the confirmedv20 launch pose; measured tables remain intact."""
    legacy=pulse==2100
    return (2100 if pulse is None else pulse,
            135.0 if legacy or (pulse is None and angle is None) else angle,legacy)


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


@dataclass
class VisionFrame:
    sequence: int
    arrived_at: float
    image: object  # Unmodified USB pixels for teammate annotation/calibration.
    jpeg: bytes  # Annotated, optionally undistorted preview.
    detections: list
    gimbal_at_arrival: dict
    width: int
    height: int
    inference_ms: float
    coordinates: str


class GimbalApp:
    def __init__(self, config_path, *, backend=None, camera_config=None,
                 dataset=None, max_age=.75, heartbeat_timeout=5.0, follow_link=None, standalone=False):
        self.config_path = Path(config_path)
        self.config = json.loads(self.config_path.read_text(encoding='utf-8')) if self.config_path.is_file() else default_config()
        if camera_config is not None:
            self.config['camera'] = deepcopy(camera_config)
        self.backend = backend
        self.follow_link, self.standalone = follow_link, bool(standalone and follow_link is not None)
        # Runtime operating window, separate from the user's mechanical
        # calibration. All real robot/manual/PWM paths share this lower bound.
        floor={'tilt':90.0} if follow_link is not None or getattr(backend,'physical',False) else {}
        self.gimbal = GimbalController(self.config, backend, operating_angle_min=floor)
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.stop = threading.Event()
        self.token = secrets.token_urlsafe(32)
        self.dataset = Path(dataset or ROOT / 'dataset/usb_captures')
        self.max_age, self.heartbeat_timeout = max_age, heartbeat_timeout
        self.last_heartbeat = time.monotonic()
        self.latest = None
        self.camera = None
        self.tracker = None
        self.camera_key = None
        self.tracking = False
        self.tracking_result = None
        self.target_selected = False
        self.selection_frames = deque(maxlen=12)
        self.error = None
        self.control_error = None
        self.capture_count = 0
        self.capture_thread = None
        self.capture_cancel = threading.Event()
        self.capture_status = dict(active=False, saved=0, total=10, category=None,
                                   folder=None, last_file=None, error=None, cancelled=False)
        self.detection_status = dict(state='waiting', dog=False, face=False, error=None)
        self.browser_recovered = False
        self.tracking_angles = None
        self.tracking_directions = dict(pan=0, tilt=0)
        self.tracking_limits = dict(pan=False, tilt=False)
        self._clear_tracking_memory()
        self._control_time = None
        self._processed_sequence = 0
        self.config_generation = 0
        self.calibration_pending = None
        self.image_calibration_model = fit_image_calibration(self._image_raw().get('samples', []))
        raw_camera = self.config.get('camera', {})
        if not isinstance(raw_camera, dict):
            raise ValueError('camera calibration must be a JSON object')
        self._new_tracker(raw_camera.get('width', 640), raw_camera.get('height', 480))
        self.thread = threading.Thread(target=self._control, daemon=True, name='gimbal-control')

    def _mount_upright(self, config=None):
        mount = (self.config if config is None else config).get('mount', {})
        if not isinstance(mount, dict):
            raise ValueError('相机安装配置必须是对象')
        endpoint = mount.get('upright_tilt_deg', 180)
        mount_image_sign(endpoint)
        return endpoint

    def _image_raw(self, config=None):
        raw = (self.config if config is None else config).get('image_calibration', {})
        if not isinstance(raw, dict) or not isinstance(raw.get('samples', []), list):
            raise ValueError('图像采样配置必须是对象，samples必须是列表')
        if not isinstance(raw.get('enabled', False), bool):
            raise ValueError('图像采样enabled必须为布尔值')
        return raw

    def _new_tracker(self, width, height, config=None):
        config = self.config if config is None else config
        self._mount_upright(config)
        self._image_raw(config)
        raw = config.get('camera', {})
        if not isinstance(raw, dict):
            raise ValueError('camera calibration must be a JSON object')
        fields = ('fx', 'fy', 'cx', 'cy', 'hfov_deg', 'vfov_deg')
        has_calibration = any(raw.get(k) is not None for k in ('fx', 'fy', 'hfov_deg', 'vfov_deg'))
        if has_calibration and (raw.get('width', width) != width or raw.get('height', height) != height):
            raise ValueError('Camera mode differs from calibration; use the calibrated resolution/focus')
        model = CameraCalibration(width=width, height=height, **{k: raw[k] for k in fields if raw.get(k) is not None})
        distortion = raw.get('distortion_coefficients')
        if distortion is not None:
            if model.model != 'intrinsics' or not isinstance(distortion, list) or len(distortion) not in (4, 5, 8, 12, 14):
                raise ValueError('Distortion needs full camera intrinsics and 4/5/8/12/14 coefficients')
            for coefficient in distortion:
                finite(coefficient, 'distortion coefficient')
        options = dict(config.get('tracking', {}))
        options.setdefault('loss_gap_s', max(1.5, 2 * self.max_age))
        options['max_frame_age_s'] = self.max_age
        return TargetTracker(model, TrackerConfig(**options))

    def start(self, camera):
        self.camera = camera
        self.thread.start()

    def publish(self, result, config_generation=None):
        with self.condition:
            if config_generation is not None and config_generation != self.config_generation:
                return False  # Inference started before an intrinsics/config edit.
            key = (result.width, result.height, json.dumps(self.config.get('camera', {}), sort_keys=True))
            if self.tracker is None or self.camera_key != key:
                self.tracking = False
                self._hold_tracking()
                self.tracker = self._new_tracker(result.width, result.height)
                self.camera_key = key
                self.target_selected = False
                self.selection_frames.clear()
            self.latest = result
            if self.follow_link:
                self.follow_link.observe(result)
            self.selection_frames.append((result.sequence, result.arrived_at, deepcopy(result.detections),
                                          deepcopy(result.gimbal_at_arrival), result.width, result.height))
            self.condition.notify_all()
            return True

    def _control_once(self, now):
        with self.lock:
            control_dt = 0 if self._control_time is None else max(0, now - self._control_time)
            self._control_time = max(now, self._control_time or now)
            if self.follow_link:
                self.follow_link.poll(self, now)
            if not self.standalone and now - self.last_heartbeat > self.heartbeat_timeout:
                self.tracking = False
                self._clear_tracking_memory()
                self.gimbal.disable()
                self.control_error = BROWSER_HEARTBEAT_ERROR
                self.browser_recovered = False
                self.tracking_angles = None
                self.tracking_directions = dict(pan=0, tilt=0)
                self.tracking_limits = dict(pan=False, tilt=False)
            result = self.latest
            stale = result is None or now - result.arrived_at > self.max_age
            if self.tracking and (stale or self.error or (self.camera and self.camera.error)):
                self._hold_tracking()
                self.tracking_result = dict(status='stale', yaw_rate_dps=0, pitch_rate_dps=0,
                                            distance_m=None, translation_m=None)
            elif self.tracking and result.sequence != self._processed_sequence:
                self._processed_sequence = result.sequence
                axes = self.gimbal.snapshot()['axes']
                at_capture = result.gimbal_at_arrival.get('axes', {})
                pan = at_capture.get('pan', {}).get('angle_deg')
                tilt = at_capture.get('tilt', {}).get('angle_deg')
                calibrated = all(axes[k]['calibrated'] and at_capture.get(k, {}).get('calibrated') is True
                                 for k in ('pan', 'tilt'))
                self.tracking_result = self.tracker.update(
                    self.follow_link.tracking_boxes(result) if self.follow_link and self.follow_link.registry.selected else result.detections,
                    result.arrived_at, now, frame_seq=result.sequence,
                    pan_deg=pan, tilt_deg=tilt, gimbal_calibrated=calibrated,
                    upright_tilt_deg=self._mount_upright())
                if self.follow_link:
                    self.follow_link.confirm(result, self.tracking_result)
                update = self.tracking_result
                if update.get('status') != 'tracking':
                    if update.get('status') in ('searching', 'reacquiring', 'ambiguous', 'missing'):
                        if self.tracking_memory['phase'] in ('idle', 'live'):
                            # Cancel queued live lead. The observation has
                            # already produced ONE absolute goal, never a
                            # velocity to integrate again on every empty frame.
                            self._hold_tracking(clear_memory=False)
                            self._memory_active = self.tracking_memory['goal'] is not None
                            self._memory_angles = None
                        self.tracking_memory['phase'] = (
                            'centering_last_seen' if self._memory_active else
                            'reacquiring' if update.get('status') == 'reacquiring' else 'waiting')
                    else:
                        self._hold_tracking()
                else:
                    # Image left/up are local camera directions. An over-top
                    # head reverses PAN's image effect after TILT passes90deg;
                    # TILT's own local correction must NOT reverse there.
                    capture_tilt = tilt if calibrated and tilt is not None else axes['tilt']['angle_deg']
                    motion = gimbal_centering_delta(update, capture_tilt,
                                                   max_rate_dps=self.tracker.config.max_rate_dps,
                                                   upright_tilt_deg=self._mount_upright())
                    source = 'mount_geometry'
                    learned = self.config.get('image_calibration', {})
                    if learned.get('enabled') is True and learned.get('angle_signature') == self._angle_signature():
                        model = self.image_calibration_model
                        if model.get('valid'):
                            # Convert the tracker's angular-rate proxy to image
                            # half-width/height per second using measured gain.
                            b = model['matrix_b']
                            scale = .5 * sum(math.hypot(b[0][i], b[1][i]) for i in (0, 1))
                            empirical = calibrated_centering_delta(model,
                                tilt_deg=capture_tilt,
                                desired_x_rate=update.get('yaw_rate_dps', 0) * scale,
                                desired_y_rate=update.get('pitch_rate_dps', 0) * scale,
                                dt=update.get('control_dt_s', 0), width=result.width, height=result.height,
                                max_rate_dps=self.tracker.config.max_rate_dps)
                            if empirical is not None:
                                motion.update(empirical)
                                source = 'sampled_image_jacobian'
                    self._remember_tracking_goal(result, update, pan, capture_tilt, source)
                    update.update(pose_compensation=source,
                                  control_source=source,
                                  tilt_pose_deg=motion['tilt_deg'],
                                  pan_pose_factor=motion['pan_factor'] if source == 'mount_geometry' else None,
                                  near_vertical=motion['near_vertical'],
                                  pan_command_rate_dps=motion['pan_rate_dps'],
                                  tilt_command_rate_dps=motion['tilt_rate_dps'],
                                  delta_pan_command_deg=motion['delta_pan_deg'],
                                  delta_tilt_command_deg=motion['delta_tilt_deg'])
                    if source == 'sampled_image_jacobian':
                        # An empirical pixel-motion model can correct an
                        # uncertain installation, but does not identify an
                        # absolute camera-to-car rotation or target position.
                        update.update(body_direction=None, body_direction_valid=False,
                                      world_direction=None, world_direction_valid=False,
                                      gimbal_angle_source='sampled_centering_no_absolute_pose')
                    if self.tracking_angles is None:
                        self.tracking_angles = {axis: axes[axis]['angle_deg'] for axis in ('pan', 'tilt')}
                    for axis, change in (('pan', motion['delta_pan_deg']),
                                         ('tilt', motion['delta_tilt_deg'])):
                        if axes[axis]['angle_deg'] is None:
                            raise ValueError('Both axes require measured angle calibration')
                        points = self.config['axes'][axis]['calibration_points']
                        actual = axes[axis]['angle_deg']
                        previous = self.tracking_angles[axis]
                        self.tracking_limits[axis] = False
                        if not change:
                            self.gimbal.hold(axis)
                            self.tracking_angles[axis] = actual
                            self.tracking_directions[axis] = 0
                            continue
                        if change * self.tracking_directions[axis] < 0:
                            previous = actual  # A reversing target cancels old-direction lead immediately.
                        self.tracking_directions[axis] = 1 if change > 0 else -1
                        requested = previous + change
                        # Keep fractional angles across frames (PWM rounds to
                        # integer us), but never queue a large invisible motion.
                        cfg = self.tracker.config
                        lead = max(.25, cfg.max_rate_dps * min(.2, 2 * cfg.max_control_dt_s))
                        requested = max(actual - lead, min(actual + lead, requested))
                        low, high = self.gimbal.angle_limits(axis)
                        angle = max(low, min(high, requested))
                        self.tracking_limits[axis] = requested <= low or requested >= high
                        self.tracking_angles[axis] = angle  # No integration beyond a mechanical limit.
                        self.gimbal.set_angle(axis, angle, tracking=True)
            if self.tracking and self._memory_active and not stale and not self.error and not (self.camera and self.camera.error):
                self._advance_tracking_memory(control_dt)
            self.gimbal.tick(now)
            if self.follow_link:
                self.follow_link.send(self, now)

    def _clear_tracking_memory(self):
        self.tracking_memory = dict(phase='idle', last_seen_sequence=None,
                                   goal=None, source=None, limited=False)
        self._memory_seen_at = None
        self._memory_active = False
        self._memory_angles = None

    def _remember_tracking_goal(self, frame, update, pan, tilt, source):
        """One bounded absolute goal from one genuine capture, not old-frame integration."""
        if pan is None or tilt is None:
            self._clear_tracking_memory()
            return
        nx = update.get('yaw_error_normalized', 0)
        ny = update.get('pitch_error_normalized', 0)
        offset = None
        if source == 'sampled_image_jacobian':
            offset = calibrated_centering_offset(self.image_calibration_model,
                tilt_deg=tilt, desired_x=nx, desired_y=ny,
                width=frame.width, height=frame.height, max_step_deg=30)
        if offset is None and update.get('body_direction_valid'):
            # A stationary car and a calibrated ray give two equivalent head
            # branches. Choose the nearest continuous command coordinates;
            # never jump to the opposite180deg branch at a zenith crossing.
            x, y, z = update['body_direction']
            azimuth = math.degrees(math.atan2(y, x))
            elevation = math.degrees(math.atan2(z, math.hypot(x, y)))
            choices = []
            for p, t in ((azimuth, elevation), (azimuth + 180, 180 - elevation)):
                p += 360 * round((pan - p) / 360)
                t += 360 * round((tilt - t) / 360)
                choices.append((p - pan, t - tilt))
            dp, dt = min(choices, key=lambda value: sum(v*v for v in value))
            offset = dict(delta_pan_deg=dp, delta_tilt_deg=dt, source='mount_geometry')
        if offset is None:
            # Without intrinsics or learned image scale, only a bounded local
            # correction can be predicted. This is not an invented bearing or
            # a claim that the unseen target must end up exactly at center.
            local = dict(yaw_rate_dps=30*nx, pitch_rate_dps=30*ny,
                         delta_yaw_deg=30*nx, delta_pitch_deg=30*ny)
            offset = gimbal_centering_delta(local, tilt, max_rate_dps=30,
                                           upright_tilt_deg=self._mount_upright())
            offset['source'] = 'normalized_prediction'
        goal, limited = {}, bool(offset.get('limited'))
        for axis, base in (('pan', pan), ('tilt', tilt)):
            delta = offset['delta_' + axis + '_deg']
            bounded = max(-30, min(30, delta))
            points = self.config['axes'][axis]['calibration_points']
            requested = base + bounded
            low,high=self.gimbal.angle_limits(axis)
            goal[axis] = max(low,min(high,requested))
            limited |= bounded != delta or goal[axis] != requested
        self.tracking_memory = dict(phase='live', last_seen_sequence=frame.sequence,
            goal=goal, source=offset['source'], limited=limited,
            target=deepcopy(update.get('target')))
        self._memory_seen_at = frame.arrived_at
        self._memory_active = False
        self._memory_angles = None

    def _advance_tracking_memory(self, elapsed):
        """Slew toward the saved finite goal; never recompute it from an old image."""
        cfg = self.tracker.config
        dt = min(.1, cfg.max_control_dt_s, elapsed)
        axes = self.gimbal.snapshot()['axes']
        if self._memory_angles is None:
            self._memory_angles = {axis: axes[axis]['angle_deg'] for axis in ('pan', 'tilt')}
        reached = True
        for axis in ('pan', 'tilt'):
            actual = axes[axis]['angle_deg']
            if actual is None or not axes[axis]['armed']:
                self._hold_tracking()
                return
            goal = self.tracking_memory['goal'][axis]
            previous = self._memory_angles[axis]
            step = cfg.max_rate_dps * dt
            command = previous + max(-step, min(step, goal - previous))
            # Bound pending lead when the configured PWM slew is slower than
            # this angular motion. Fractional angles survive PWM quantization.
            lead = max(.25, cfg.max_rate_dps * min(.2, 2*cfg.max_control_dt_s))
            command = max(actual-lead, min(actual+lead, command))
            self._memory_angles[axis] = command
            self.gimbal.set_angle(axis, command, tracking=True)
            pulse = self.gimbal.axes[axis].angle_to_pulse(goal)
            reached &= axes[axis]['output_us'] == pulse
        if reached or cfg.max_rate_dps == 0:
            self._hold_tracking(clear_memory=False)
            self._memory_active = False
            self.tracking_memory['phase'] = ('reacquiring' if
                (self.tracking_result or {}).get('status') == 'reacquiring' else 'waiting')

    def _hold_tracking(self, *, clear_memory=True):
        self.gimbal.hold()
        self.tracking_angles = None
        self.tracking_directions = dict(pan=0, tilt=0)
        self.tracking_limits = dict(pan=False, tilt=False)
        if clear_memory:
            self._clear_tracking_memory()

    def _control(self):
        while not self.stop.wait(.02):
            try:
                self._control_once(time.monotonic())
            except Exception as exc:
                with self.lock:
                    self.control_error = str(exc)
                    self.tracking = False
                    self._clear_tracking_memory()
                    try:
                        self.gimbal.disable()
                    except Exception as disable_error:
                        self.control_error += f'; disable failed: {disable_error}'

    def snapshot(self):
        with self.lock:
            frame = self.latest
            age = time.monotonic() - frame.arrived_at if frame else None
            gimbal = self.gimbal.snapshot()
            tilt = gimbal['axes']['tilt']['angle_deg']
            pose = (dict(tilt_deg=None, pan_factor=None, near_vertical=False, side='unknown')
                    if tilt is None else gimbal_centering_delta({}, tilt, upright_tilt_deg=self._mount_upright()))
            view_config = deepcopy(self.config)
            view_config.setdefault('mount', dict(upright_tilt_deg=self._mount_upright()))
            memory = deepcopy(self.tracking_memory)
            memory['last_seen_age_s'] = (None if self._memory_seen_at is None else
                                          max(0, time.monotonic() - self._memory_seen_at))
            return dict(gimbal=gimbal, config=view_config,
                        fpga_follow=None if self.follow_link is None else dict(
                            software_version='v22',target_kind=self.follow_link.auto_kind,
                            startup_pose=getattr(self.follow_link,'runtime_info',None),
                            standalone=self.standalone, selected_id=self.follow_link.registry.selected,
                            reference=self.follow_link.follower.reference,
                            forward_sign=self.follow_link.follower.forward_sign,
                            fpga=self.follow_link.report, error=self.follow_link.error,
                            log=str(self.follow_link.log_path)),
                        image_calibration=self._calibration_state(),
                        tracking_pose=pose,
                        tracking_memory=memory,
                        tracking=self.tracking, tracking_result=deepcopy(self.tracking_result),
                        target_selected=self.target_selected,
                        tracking_preflight=self._tracking_preflight(),
                        camera=self.camera.actual if self.camera else {},
                        error=self.error or (self.camera.error if self.camera else None),
                        control_error=self.control_error, capture_count=self.capture_count,
                        capture=deepcopy(self.capture_status), detection=deepcopy(self.detection_status),
                        browser_recovered=self.browser_recovered,
                        tracking_limits=deepcopy(self.tracking_limits),
                        tracking_settings=dict(self._new_tracker(
                            frame.width if frame else self.config.get('camera', {}).get('width', 640),
                            frame.height if frame else self.config.get('camera', {}).get('height', 480)).config.__dict__),
                        frame=None if frame is None else dict(sequence=frame.sequence, age_s=age,
                            fresh=age <= self.max_age, width=frame.width, height=frame.height,
                            inference_ms=frame.inference_ms, coordinates=frame.coordinates,
                            detections=deepcopy(frame.detections)),
                        position_note='Direction only; no target range or 3D position. Servo angles are calibrated commands, not encoder feedback.')

    def _angle_signature(self):
        points = {axis: self.config['axes'][axis].get('calibration_points', []) for axis in ('pan', 'tilt')}
        return hashlib.sha256(json.dumps(points, sort_keys=True).encode()).hexdigest()

    def _calibration_state(self):
        raw = self.config.get('image_calibration', {})
        fit = deepcopy(self.image_calibration_model)
        ready = fit.get('valid') is True
        changed = bool(raw.get('samples')) and raw.get('angle_signature') != self._angle_signature()
        reasons = fit.get('diagnostics', {}).get('reasons', [])
        if changed:
            ready = False
            reasons = ['角度表已变化，请清除图像采样后重新采集']
        if ready and self.latest and (fit.get('width'), fit.get('height')) != (self.latest.width, self.latest.height):
            ready = False
            reasons = ['图像分辨率已变化，请恢复采样时分辨率，或清除后重新采样']
        fit.update(ready=ready, reason='；'.join(reasons) if reasons else '两轴画面响应已拟合，可应用采样结果')
        return dict(pending=deepcopy(self.calibration_pending), sample_count=len(raw.get('samples', [])),
                    enabled=ready and raw.get('enabled') is True, fit=fit,
                    samples=deepcopy(raw.get('samples', [])))

    def _calibration_observation(self, data):
        sequence, index = data.get('sequence'), data.get('index')
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            raise ValueError('请从当前画面选择目标')
        frame = next((f for f in reversed(self.selection_frames) if f[0] == sequence), None)
        if frame is None or time.monotonic() - frame[1] > self.max_age:
            raise ValueError('采样画面已过期，请等新画面再点击同一目标')
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(frame[2]):
            raise ValueError('请选择当前同一个人物或玩偶')
        box = frame[2][index]
        point = dict(u=(box['x1'] + box['x2']) / 2, v=(box['y1'] + box['y2']) / 2)
        current = self.gimbal.snapshot()['axes']
        for axis in ('pan', 'tilt'):
            state = frame[3].get('axes', {}).get(axis, {})
            live = current[axis]
            if not state.get('armed') or not state.get('calibrated') or state.get('angle_deg') is None or not live['armed'] or not live['calibrated']:
                raise ValueError('采样需要两轴开启PWM并保留现有角度表')
            if state.get('target_us') != state.get('output_us') or live['target_us'] != live['output_us']:
                raise ValueError('PWM还在渐变，请等输出稳定和人物静止后再记录')
            if state['output_us'] != live['output_us']:
                raise ValueError('这张画面早于当前PWM位置，请等新画面再记录')
            point[axis + '_us'] = state['output_us']
            point[axis + '_deg'] = state['angle_deg']
        return dict(point=point, width=frame[4], height=frame[5], kind=box['kind'],
                    sequence=sequence, timestamp=frame[1])

    def _save_tracking_candidate(self, candidate):
        frame = self.latest
        tracker = self._new_tracker(frame.width if frame else 640, frame.height if frame else 480, candidate)
        model = fit_image_calibration(candidate.get('image_calibration', {}).get('samples', []))
        atomic_json(self.config_path, candidate)
        self._hold_tracking()
        self.tracking = self.target_selected = False
        self.config, self.tracker = candidate, tracker
        self.image_calibration_model = model
        self.config_generation += 1
        self.selection_frames.clear()
        self.tracking_result = None
        return dict(saved=str(self.config_path), image_calibration=self._calibration_state(), gimbal=self.gimbal.snapshot())

    def _tracking_preflight(self):
        """Describe missing prerequisites without moving or enabling outputs."""
        gimbal = self.gimbal.snapshot()
        details, reasons = {}, []
        for axis, title in (('pan', 'PAN底座'), ('tilt', 'TILT上层')):
            current = gimbal['axes'][axis]
            calibrated, enabled = current['calibrated'], current['armed']
            covered = calibrated and current['angle_deg'] is not None
            missing = []
            if not calibrated:
                missing.append(f'{title}角度尚未保存，请在角度标定页保存至少两个实测点')
            if not enabled:
                missing.append(f'{title}PWM关闭，请返回手动调节页开启本轴；已保存角度会保留')
            elif calibrated and not covered:
                points = self.config['axes'][axis]['calibration_points']
                low, high = min(p['pulse_us'] for p in points), max(p['pulse_us'] for p in points)
                missing.append(f'{title}当前输出不在角度表覆盖的{low}～{high}µs内，请先手动移到该范围')
            details[axis] = dict(calibrated=calibrated, pwm_enabled=enabled,
                                 covered=covered, ready=not missing, reason='；'.join(missing))
            reasons.extend(missing)
        if not self.target_selected:
            reasons.append('请先点击画面旁的一个玩偶或人脸目标，再开始跟踪')
        if self.calibration_pending is not None:
            reasons.append('正在记录手动采样，请先记录终点或取消本组，再重新选目标跟踪')
        fresh = self.latest is not None and time.monotonic() - self.latest.arrived_at <= self.max_age
        if not fresh or self.tracker is None:
            reasons.append('相机画面尚未更新或已过期，请等待新画面后重新选择目标')
        camera_error = self.error or (self.camera.error if self.camera else None)
        if camera_error:
            reasons.append('请先处理相机错误：' + str(camera_error))
        if self.control_error or gimbal['fault']:
            reasons.append('请先处理PWM控制错误：' + str(self.control_error or gimbal['fault']))
        return dict(ready=not reasons, reasons=reasons, axes=details,
                    target_selected=self.target_selected, frame_fresh=fresh)

    def command(self, data):
        if not isinstance(data, dict):
            raise ValueError('Expected a JSON object')
        action = data.get('action')
        if action == 'capture':  # Legacy single-photo API; no lock across disk I/O.
            return self.capture(data.get('category', 'positive'))
        with self.lock:
            if action == 'heartbeat':
                self.last_heartbeat = time.monotonic()
                if self.control_error == BROWSER_HEARTBEAT_ERROR:
                    self.control_error = None
                    self.browser_recovered = True  # Never rearm outputs on reconnection.
                return {'ok': True}
            if action == 'save_mount':
                endpoint = data.get('upright_tilt_deg')
                mount_image_sign(endpoint)
                candidate = deepcopy(self.config)
                candidate['mount'] = dict(upright_tilt_deg=endpoint)
                result = self._save_tracking_candidate(candidate)
                self.calibration_pending = None
                return result
            if action == 'calibration_begin':
                if self.calibration_pending is not None:
                    raise ValueError('已有采样起点，请先记录终点或取消本组')
                raw = self.config.get('image_calibration', {})
                if raw.get('samples') and raw.get('angle_signature') != self._angle_signature():
                    raise ValueError('角度表已变化，请先清除旧图像采样')
                observation = self._calibration_observation(data)
                self._hold_tracking()
                self.tracking = False
                self.target_selected = False
                if self.tracker is not None:
                    self.tracker.clear_target()
                self.calibration_pending = dict(start=observation['point'], width=observation['width'],
                    height=observation['height'], kind=observation['kind'], sequence=observation['sequence'],
                    angle_signature=self._angle_signature())
                return self._calibration_state()
            if action == 'calibration_finish':
                pending = self.calibration_pending
                if pending is None:
                    raise ValueError('请先点人物记录本组起点')
                observation = self._calibration_observation(data)
                if observation['sequence'] <= pending['sequence']:
                    raise ValueError('请调节PWM后等新画面，再记录终点')
                if observation['kind'] != pending['kind']:
                    raise ValueError('终点必须选择起点的同一类目标；人物和车身应保持不动')
                if pending['angle_signature'] != self._angle_signature():
                    raise ValueError('采样期间角度表发生变化，请取消本组重新采集')
                if (observation['width'], observation['height']) != (pending['width'], pending['height']):
                    raise ValueError('采样期间图像分辨率变化，请取消本组重新采集')
                sample = normalize_sample(dict(width=pending['width'], height=pending['height'],
                    kind=pending['kind'], start=pending['start'], end=observation['point']))
                candidate = deepcopy(self.config)
                old = candidate.get('image_calibration', {})
                candidate['image_calibration'] = dict(samples=(old.get('samples', []) + [sample])[-64:],
                    angle_signature=self._angle_signature(), enabled=False)
                self._save_tracking_candidate(candidate)
                self.calibration_pending = None
                return self._calibration_state()
            if action == 'calibration_cancel':
                self.calibration_pending = None
                return self._calibration_state()
            if action in ('calibration_clear', 'calibration_apply'):
                candidate = deepcopy(self.config)
                if action == 'calibration_clear':
                    candidate['image_calibration'] = dict(samples=[], enabled=False,
                                                         angle_signature=self._angle_signature())
                else:
                    state = self._calibration_state()
                    if not state['fit']['ready']:
                        raise ValueError(state['fit']['reason'])
                    if self.calibration_pending is not None:
                        raise ValueError('请先完成或取消当前采样组')
                    candidate['image_calibration']['enabled'] = True
                self._save_tracking_candidate(candidate)
                self.calibration_pending = None
                return self._calibration_state()
            if action == 'disable':
                if self.tracking:
                    self._hold_tracking()
                self.tracking = False
                self.tracking_angles = None
                self.tracking_directions = dict(pan=0, tilt=0)
                self.tracking_limits = dict(pan=False, tilt=False)
                self.calibration_pending = None
                return self.gimbal.disable(data.get('axis'))
            if action in ('manual_enable', 'manual_jog', 'manual_pulse', 'manual_steps', 'manual_range',
                          'mark_limit', 'apply_limits', 'clear_limits'):
                axis = data.get('axis')
                if axis not in ('pan', 'tilt'):
                    raise ValueError('请选择pan底座轴或tilt上层轴')
                if action == 'manual_range':
                    candidate = deepcopy(self.config)
                    candidate['axes'][axis].update(explore_min_us=data.get('explore_min_us'),
                                                  explore_max_us=data.get('explore_max_us'))
                    result = self._save_axis_candidate(axis, candidate, clear_pending=False)
                    # A rejected range/storage operation must not alter tracking
                    # targets. Pause only after a successful manual setting edit.
                    if self.tracking:
                        self.tracking = False
                        self._hold_tracking()
                        result['gimbal'] = self.gimbal.snapshot()
                        result['tracking_paused'] = True
                    return result
                # Explicit manual input pauses tracking, but does not cancel
                # an unrelated manual slew on the other axis.
                if self.tracking:
                    self.tracking = False
                    self._hold_tracking()
                if action == 'manual_enable':
                    if time.monotonic() - self.last_heartbeat > self.heartbeat_timeout:
                        raise ValueError('浏览器心跳已过期，请等待连接恢复后再次点击开启本轴PWM')
                    result = self.gimbal.manual_enable(axis, data.get('pulse_us'))
                    self.control_error = None
                    self.browser_recovered = False
                    return result
                if action == 'manual_jog':
                    return self.gimbal.manual_jog(axis, data.get('direction'), coarse=data.get('coarse') is True)
                if action == 'manual_pulse':
                    return self.gimbal.manual_pulse(axis, data.get('pulse_us'))
                if action == 'manual_steps':
                    candidate = deepcopy(self.config)
                    candidate['axes'][axis].update(fine_us=data.get('fine_us'), coarse_us=data.get('coarse_us'))
                    if 'slew_us_per_s' in data:
                        candidate['axes'][axis]['slew_us_per_s'] = data['slew_us_per_s']
                    return self._save_axis_candidate(axis, candidate, clear_pending=False)
                if action == 'clear_limits':
                    candidate = deepcopy(self.config)
                    candidate['axes'][axis].update(safe_min_us=None, safe_max_us=None,
                        calibration_points=[], calibration_confirmed=False)
                    result = self._save_axis_candidate(axis, candidate)
                    result['message'] = '已清除此轴限位和角度标定；保持当前PWM状态，可继续手动探索'
                    return result
                if action == 'mark_limit':
                    self.gimbal.mark_limit(axis, data.get('limit'))
                    pending = self.gimbal.snapshot()['axes'][axis]['manual']
                    if pending['pending_min_us'] is None or pending['pending_max_us'] is None:
                        return {'axis': axis, 'pending': True,
                                'message': '已暂存当前端点；记录另一端后自动保存限位',
                                'gimbal': self.gimbal.snapshot()}
                return self._apply_pending_limits(axis, reset_angles=data.get('reset_angles') is True)
            if action == 'save_angles':
                axis = data.get('axis')
                if axis not in ('pan', 'tilt'):
                    raise ValueError('请选择pan底座轴或tilt上层轴')
                points = data.get('calibration_points')
                if not isinstance(points, list) or len(points) < 2:
                    raise ValueError('至少记录两个不同角度的实测点后，再保存本轴角度')
                candidate = deepcopy(self.config)
                candidate['axes'][axis].update(calibration_points=deepcopy(points), calibration_confirmed=True)
                result = self._save_axis_candidate(axis, candidate, clear_pending=False)
                if self.tracking:
                    self._hold_tracking()
                self.tracking = False
                self.tracking_angles = None
                self.tracking_directions = dict(pan=0, tilt=0)
                self.tracking_limits = dict(pan=False, tilt=False)
                self.target_selected = False
                self.selection_frames.clear()
                result['gimbal'] = self.gimbal.snapshot()
                result['message'] = '本轴角度已保存，PWM开关和当前脉宽保持不变；请重新选择目标后开始跟踪'
                return result
            if action == 'save_tracking':
                options = deepcopy(self.config.get('tracking', {}))
                supplied = data.get('tracking')
                names = ('deadband', 'normalized_gain_dps', 'max_rate_dps', 'filter_time_constant_s',
                         'deadband_hysteresis', 'min_rate_dps', 'max_control_dt_s')
                if not isinstance(supplied, dict) or set(supplied) != set(names):
                    raise ValueError('请填写完整的七项跟踪参数')
                options.update({name: finite(supplied[name], name) for name in names})
                candidate = deepcopy(self.config)
                candidate['tracking'] = options
                self.tracking = False
                self._hold_tracking()
                previous = self.config
                try:
                    self.config = candidate
                    camera = candidate.get('camera', {})
                    tracker = self._new_tracker(self.latest.width if self.latest else camera.get('width', 640),
                                                self.latest.height if self.latest else camera.get('height', 480))
                finally:
                    self.config = previous
                atomic_json(self.config_path, candidate)
                self.config = candidate
                self.tracker = tracker
                self.config_generation += 1
                self.target_selected = False
                self.tracking_result = None
                return {'saved': str(self.config_path), 'tracking_paused': True}
            if action == 'save_config':
                self.tracking = False
                self._clear_tracking_memory()
                stopped = self.gimbal.disable()
                if stopped.get('fault'):
                    raise ValueError('Resolve the PWM stop fault before saving calibration: ' + stopped['fault'])
                config = data['config']
                new_controller = GimbalController(config, self.backend, operating_angle_min=self.gimbal.operating_angle_min)  # No device writes.
                try:
                    old_config = self.config
                    self.config = config
                    try:
                        camera = config.get('camera', {})
                        if not isinstance(camera, dict):
                            raise ValueError('camera calibration must be a JSON object')
                        width = self.latest.width if self.latest else camera.get('width', 640)
                        height = self.latest.height if self.latest else camera.get('height', 480)
                        self._new_tracker(width, height)
                    finally:
                        self.config = old_config
                    atomic_json(self.config_path, config)
                except Exception:
                    new_controller.close()
                    raise
                self.gimbal.close()
                self.gimbal = new_controller
                self.config = deepcopy(config)
                self.calibration_pending = None
                self.image_calibration_model = fit_image_calibration(self._image_raw().get('samples', []))
                self.config_generation += 1
                self.tracker = None
                self.camera_key = None
                self.tracking_result = None
                self.target_selected = False
                self.selection_frames.clear()
                return {'saved': str(self.config_path)}
            if action == 'capture_burst':
                return self.start_burst(data.get('category', 'positive'))
            if action == 'cancel_capture':
                self.capture_cancel.set()
                self.condition.notify_all()
                return {'cancel_requested': True}
            if action == 'save_axis':
                axis = data.get('axis')
                if axis not in ('pan', 'tilt'):
                    raise ValueError('Choose pan or tilt')
                candidate = deepcopy(self.config)
                for name in ('safe_min_us', 'safe_max_us', 'fine_us', 'coarse_us', 'start_us'):
                    candidate['axes'][axis][name] = data.get(name)
                if data.get('reset_angles') is True:
                    candidate['axes'][axis].update(calibration_points=[], calibration_confirmed=False)
                start = finite(data.get('start_us'), 'start_us')
                low, high = finite(data.get('safe_min_us'), 'safe_min_us'), finite(data.get('safe_max_us'), 'safe_max_us')
                if start != int(start) or not low <= start <= high:
                    raise ValueError('起始脉宽必须是安全最小/最大脉宽之间的整数')
                return self.command(dict(action='save_config', config=candidate))
            if action == 'select':
                requested_sequence = data.get('sequence')
                if isinstance(requested_sequence, bool) or not isinstance(requested_sequence, int) or requested_sequence < 0:
                    raise ValueError('sequence must be a nonnegative integer')
                frame = next((f for f in reversed(self.selection_frames) if f[0] == data.get('sequence')), None)
                if frame is None or time.monotonic() - frame[1] > self.max_age:
                    raise ValueError('目标画面已更新或过期，请等待新画面后重新点击目标')
                index = data.get('index')
                if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(frame[2]):
                    raise ValueError('Invalid target index')
                self._hold_tracking()
                self.tracker.select_target(frame[2][index], frame[1], frame_seq=frame[0])
                self.target_selected = True
                self.tracking_result = None
                return {'selected': frame[2][index]}
            if action == 'track':
                enabled = data.get('enabled') is True
                if enabled:
                    prerequisites = self._tracking_preflight()
                    if not prerequisites['ready']:
                        raise ValueError('暂不能跟踪：' + '；'.join(prerequisites['reasons']))
                    axes = self.gimbal.snapshot()['axes']
                    self._clear_tracking_memory()
                    self._processed_sequence = self.latest.sequence
                    self.tracking_angles = {axis: axes[axis]['angle_deg'] for axis in ('pan', 'tilt')}
                    self.tracking_directions = dict(pan=0, tilt=0)
                    self.tracking_limits = dict(pan=False, tilt=False)
                else:
                    self._hold_tracking()
                self.tracking = enabled
                return {'tracking': enabled}
            if action == 'arm':
                self.tracking = False
                self._clear_tracking_memory()
                if time.monotonic() - self.last_heartbeat > self.heartbeat_timeout:
                    raise ValueError('Browser heartbeat required')
                result = self.gimbal.arm(data.get('axis'), data.get('start_us'),
                                         manual_calibration=data.get('manual') is True,
                                         physical=data.get('physical') is True)
                self.control_error = None
                self.browser_recovered = False
                return result
            if action in ('jog', 'pulse', 'angle'):
                if self.tracking:
                    raise ValueError('Pause tracking before manual adjustment')
                if action == 'jog':
                    return self.gimbal.jog(data.get('axis'), data.get('direction'), coarse=data.get('coarse') is True)
                if action == 'pulse':
                    return self.gimbal.set_pulse(data.get('axis'), data.get('pulse_us'), manual_calibration=data.get('manual') is True)
                return self.gimbal.set_angle(data.get('axis'), data.get('angle_deg'))
            raise ValueError('Unknown action')

    def _save_axis_candidate(self, axis, candidate, *, clear_pending=True):
        """Persist and replace one axis; never rebuild/arm/center either axis."""
        try:
            parsed = validate_config(candidate)
        except GimbalError as exc:
            raise ValueError('本轴配置无效：' + str(exc)) from exc
        raw = candidate['axes'][axis]
        cfg = parsed[axis]
        low = cfg.safe_min_us if cfg.configured else cfg.explore_min_us
        high = cfg.safe_max_us if cfg.configured else cfg.explore_max_us
        before = self.gimbal.snapshot()['axes'][axis]
        if before['armed'] and not (
                low <= before['output_us'] <= high and low <= before['target_us'] <= high):
            raise ValueError('当前输出或请求脉宽不在新限位内；请先移动到新范围内或关闭本轴PWM，再重试保存')
        if not clear_pending and any(value is not None and not cfg.explore_min_us <= value <= cfg.explore_max_us
                for value in (before['manual']['pending_min_us'], before['manual']['pending_max_us'])):
            raise ValueError('新探索范围不能排除已暂存的端点；请保留该端点，或先清除本轴记录再重新测量')
        try:
            atomic_json(self.config_path, candidate)
        except OSError as exc:
            raise ValueError(f'配置保存失败：{self.config_path}；请检查目录权限后重试。{exc}') from exc
        self.gimbal.apply_axis_config(axis, raw, clear_pending=clear_pending)
        self.config = candidate
        self.config_generation += 1
        self.tracking_result = None
        return {'saved': str(self.config_path), 'axis': axis, 'gimbal': self.gimbal.snapshot()}

    def _apply_pending_limits(self, axis, *, reset_angles=False):
        pending = self.gimbal.snapshot()['axes'][axis]['manual']
        low, high = pending['pending_min_us'], pending['pending_max_us']
        if low is None or high is None:
            raise ValueError('请先在PWM开启且脉宽稳定时，分别记录下限和上限')
        if low >= high:
            raise ValueError('下限脉宽必须小于上限；请重新记录标错的端点，不需要改变另一轴')
        candidate = deepcopy(self.config)
        raw = candidate['axes'][axis]
        if reset_angles:
            raw.update(calibration_points=[], calibration_confirmed=False)
        elif any(not low <= point['pulse_us'] <= high for point in raw.get('calibration_points', [])):
            raise ValueError('新限位排除了已有角度记录，原数据仍保留。请点“清除旧角度并应用”，或重新记录能包含这些角度点的限位')
        raw.update(safe_min_us=low, safe_max_us=high)
        stored_start = raw.get('start_us', 1500)
        if (isinstance(stored_start, bool) or not isinstance(stored_start, (int, float)) or
                not math.isfinite(stored_start) or stored_start != int(stored_start) or
                not low <= stored_start <= high):
            stored_start = min(high, max(low, 1500))
        raw['start_us'] = int(stored_start)
        result = self._save_axis_candidate(axis, candidate)
        result['message'] = '已保存本轴两端限位；当前PWM脉宽保持不变'
        return result

    def capture(self, category):
        if category not in ('positive', 'negative', 'calibration'):
            raise ValueError('Choose positive, negative or calibration')
        with self.lock:
            frame = self.latest
            if frame is None or time.monotonic() - frame.arrived_at > self.max_age:
                raise ValueError('No fresh camera frame')
        return self._save_frame(frame, category)

    def _save_frame(self, frame, category):
        """The caller pins a frame object; disk I/O never holds the app lock."""
        import cv2
        folder = self.dataset / category
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_{frame.sequence}'
        path = folder / (stamp + '.jpg')
        if not cv2.imwrite(str(path), frame.image):
            raise RuntimeError('Cannot save original USB image')
        with self.lock:
            camera = deepcopy(self.camera.actual if self.camera else {})
        atomic_json(path.with_suffix('.json'), dict(frame=frame.sequence, image='RAW original; no boxes/text',
            camera=camera, captured_gimbal=frame.gimbal_at_arrival,
            observations=frame.detections, observation_coordinates=frame.coordinates,
            annotation='Observations are not ground truth; label the original image manually'))
        with self.lock:
            self.capture_count += 1
        return {'saved': str(path)}

    def start_burst(self, category, *, interval=.5, frame_timeout=3.0):
        if category not in ('positive', 'negative', 'calibration'):
            raise ValueError('Choose positive, negative or calibration')
        with self.lock:
            if self.stop.is_set():
                raise ValueError('Camera service is stopping')
            if self.capture_status['active']:
                raise ValueError('正在连拍，请等待完成或先取消')
            if self.latest is None or time.monotonic() - self.latest.arrived_at > self.max_age:
                raise ValueError('No fresh camera frame')
            self.capture_cancel.clear()
            # A burst begins with the next frame, never ten saves of latest.
            sequence = self.latest.sequence
            self.capture_status = dict(active=True, saved=0, total=10, category=category,
                folder=str(self.dataset / category), last_file=None, error=None, cancelled=False)
            self.capture_thread = threading.Thread(target=self._burst,
                args=(category, sequence, interval, frame_timeout), daemon=True, name='usb-photo-burst')
            self.capture_thread.start()
            return {'started': True, 'total': 10, 'folder': str(self.dataset / category)}

    def _burst(self, category, sequence, interval, frame_timeout):
        next_at = time.monotonic()
        try:
            for _ in range(10):
                deadline = time.monotonic() + frame_timeout + max(0, next_at - time.monotonic())
                with self.condition:
                    while True:
                        if self.stop.is_set() or self.capture_cancel.is_set():
                            self.capture_status['cancelled'] = True
                            return
                        now = time.monotonic()
                        frame = self.latest
                        if (frame is not None and frame.sequence > sequence and now >= next_at
                                and frame.arrived_at >= next_at and now - frame.arrived_at <= self.max_age):
                            break
                        if self.error or (self.camera and self.camera.error):
                            raise RuntimeError(self.error or self.camera.error)
                        if now >= deadline:
                            raise RuntimeError('连拍已停止：未收到新的有效相机帧，已保存照片保留')
                        self.condition.wait(min(.1, max(.001, deadline - now)))
                # Frame objects are immutable to the capture pipeline. This is
                # the raw USB image, not JPEG preview or an annotated copy.
                result = self._save_frame(frame, category)
                with self.lock:
                    self.capture_status['saved'] += 1
                    self.capture_status['last_file'] = result['saved']
                sequence = frame.sequence
                next_at = max(next_at + interval, time.monotonic())
        except Exception as exc:
            with self.lock:
                self.capture_status['error'] = str(exc)
        finally:
            with self.lock:
                self.capture_status['active'] = False

    def close(self):
        self.stop.set()
        with self.condition:
            # Stop physical output before waiting for a slow photo disk write.
            if self.follow_link:
                self.follow_link.close(self)
            self.tracking = False
            self._clear_tracking_memory()
            self.gimbal.close()
            self.condition.notify_all()
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2)
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.gimbal.snapshot().get('fault'):
            print('PWM shutdown fault: ' + self.gimbal.snapshot()['fault'], flush=True)


class IndependentDetectors:
    """A failed dog engine must not disable an otherwise healthy YuNet face.

    The existing per-model inference/preprocessing implementations are reused;
    no model, manifest, calibration or package installation is changed here.
    """
    def __init__(self,args):
        self.engines={};self.errors={}
        for kind in ('dog','human'):
            if kind=='human' and getattr(args,'no_face',False):continue
            try:
                if kind=='dog':
                    from dog_inference import DogDetector
                    engine=DogDetector(args.model,args.confidence,args.iou,args.threads)
                else:
                    from face_inference import FaceDetector
                    engine=FaceDetector(args.face_model,args.face_confidence,args.face_iou,args.threads)
                self.engines[kind]=engine
            except Exception as exc:
                self.errors[kind]=str(exc)
                print('VISION '+kind+' unavailable: '+str(exc),flush=True)

    def predict(self,image):
        result={};elapsed={}
        for kind in ('dog','human'):
            started=time.perf_counter()
            try:
                result[kind]=self.engines[kind].predict(image) if kind in self.engines else []
            except Exception as exc:
                # Retain the healthy class and annotated camera stream. A
                # broken model is retried after an explicit process restart.
                self.errors[kind]=str(exc);self.engines.pop(kind,None);result[kind]=[]
                print('VISION '+kind+' failed: '+str(exc),flush=True)
            elapsed[kind]=(time.perf_counter()-started)*1000
        return result['dog'],result['human'],elapsed['dog'],elapsed['human']

    def status(self):
        return dict(state=('partial' if self.errors else 'enabled') if self.engines else 'error',
                    dog='dog' in self.engines,face='human' in self.engines,
                    errors=dict(self.errors),error='; '.join(k+': '+v for k,v in self.errors.items()) or None)


class VisionWorker:
    def __init__(self, app, camera, args):
        self.app, self.camera, self.args = app, camera, args
        self.thread = threading.Thread(target=self._run, daemon=True, name='usb-inference')
        with app.lock:
            app.detection_status = dict(state='disabled' if args.no_detection else 'loading',
                dog=not args.no_detection,
                face=not args.no_detection and not getattr(args, 'no_face', False), error=None)

    def _run(self):
        try:
            import cv2
            import numpy as np
            detector = None
            if not self.args.no_detection:
                detector = IndependentDetectors(self.args)
                with self.app.lock:
                    self.app.detection_status.update(detector.status())
                print('VISION engines: '+json.dumps(detector.status(),ensure_ascii=False),flush=True)
            sequence = 0
            while not self.app.stop.is_set():
                frame = self.camera.newest(after=sequence)
                if frame is None:
                    if self.camera.error:
                        raise RuntimeError(self.camera.error)
                    continue
                sequence = frame.sequence
                if time.monotonic() - frame.arrived_at > self.app.max_age:
                    continue
                started = time.perf_counter()
                image = frame.image
                with self.app.lock:
                    calibration = deepcopy(self.app.config.get('camera', {}))
                    generation = self.app.config_generation
                coefficients = calibration.get('distortion_coefficients')
                coordinates = 'original USB pixels'
                if coefficients is not None:
                    h, w = image.shape[:2]
                    if (calibration.get('width'), calibration.get('height')) != (w, h):
                        raise ValueError('Undistortion calibration resolution does not match USB camera')
                    matrix = np.array([[calibration['fx'], 0, calibration['cx']],
                                       [0, calibration['fy'], calibration['cy']], [0, 0, 1]], dtype=np.float64)
                    image = cv2.undistort(image, matrix, np.array(coefficients, dtype=np.float64))
                    coordinates = 'undistorted pixels with original K; saved photograph remains RAW'
                boxes, faces, dog_ms, face_ms = detector.predict(image) if detector else ([], [], 0, 0)
                if detector:
                    with self.app.lock:self.app.detection_status.update(detector.status())
                detections = [dict(x1=b.x0, y1=b.y0, x2=b.x1, y2=b.y1, score=b.confidence, kind=kind)
                              for kind, group in (('dog', boxes), ('human', faces)) for b in group]
                preview = image.copy()
                h, w = image.shape[:2]
                for index, box in enumerate(detections):
                    color = (30, 210, 30) if box['kind'] == 'dog' else (240, 140, 30)
                    cv2.rectangle(preview, (round(box['x1']), round(box['y1'])),
                                  (round(box['x2']), round(box['y2'])), color, 2)
                    cv2.putText(preview, f'{index}: {box["kind"]} {box["score"]:.2f}',
                                (round(box['x1']), max(18, round(box['y1']) - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                cv2.drawMarker(preview, (w // 2, h // 2), (0, 220, 220), cv2.MARKER_CROSS, 18, 1)
                cv2.putText(preview, f'USB {sequence} dog {dog_ms:.0f}ms face {face_ms:.0f}ms',
                            (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 220, 220), 1)
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError('Cannot encode browser preview')
                self.app.publish(VisionFrame(sequence, frame.arrived_at, frame.image, encoded.tobytes(),
                    detections, frame.gimbal, w, h, (time.perf_counter() - started) * 1000, coordinates),
                    config_generation=generation)
        except Exception as exc:
            with self.app.lock:
                self.app.detection_status.update(state='error', error=str(exc))
                self.app.error = str(exc)
                self.app.tracking = False
                self.app._clear_tracking_memory()
                self.app.gimbal.disable()


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_bytes(self, code, data, content_type):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def json(self, code, data):
            self.send_bytes(code, json.dumps(data, ensure_ascii=False, allow_nan=False).encode(), 'application/json; charset=utf-8')

        def do_GET(self):
            route = urlsplit(self.path).path
            try:
                if route == '/':
                    html = Path(__file__).with_name('usb_gimbal_ui.html').read_text(encoding='utf-8')
                    self.send_bytes(200, html.replace('__CONTROL_TOKEN__', app.token).encode(), 'text/html; charset=utf-8')
                elif route == '/state':
                    self.json(200, app.snapshot())
                elif route == '/stream.mjpg':
                    self.connection.settimeout(3)
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    sequence = 0
                    while not app.stop.is_set():
                        with app.condition:
                            app.condition.wait_for(lambda: app.stop.is_set() or
                                (app.latest is not None and app.latest.sequence != sequence), .5)
                            result = app.latest
                            if result is None or result.sequence == sequence:
                                continue
                            jpeg, sequence = result.jpeg, result.sequence
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                         str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                else:
                    self.json(404, {'error': 'Not found'})
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_POST(self):
            try:
                self.connection.settimeout(3)
                if urlsplit(self.path).path != '/command':
                    return self.json(404, {'error': 'Not found'})
                if not secrets.compare_digest(self.headers.get('X-Control-Token', ''), app.token):
                    return self.json(403, {'error': 'Reload the local control page'})
                if self.headers.get_content_type() != 'application/json':
                    raise ValueError('JSON required')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 65536:
                    raise ValueError('Invalid request size')
                raw_body = self.rfile.read(length)
                if len(raw_body) != length:
                    raise ValueError('Incomplete request body')
                body = json.loads(raw_body)
                self.json(200, {'ok': True, 'result': app.command(body)})
            except (ValueError, KeyError, TypeError) as exc:
                self.json(400, {'error': str(exc)})
            except OSError as exc:
                self.json(503, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=15)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/gimbal.json')
    parser.add_argument('--camera-config', type=Path)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'dataset/usb_captures')
    parser.add_argument('--pwm-pan', type=Path)
    parser.add_argument('--pwm-tilt', type=Path)
    parser.add_argument('--hardware', action='store_true', help='Select real sysfs backend; browser must still explicitly arm each output')
    parser.add_argument('--fpga', action='store_true', help='Metadata/PWM via FPGA; preserve local USB recognition and calibration')
    parser.add_argument('--standalone', action='store_true', help='With --fpga, LCD owns operation after the PC cable is removed')
    parser.add_argument('--initial-pan', type=int, default=1200)
    tilt_initial=parser.add_mutually_exclusive_group()
    tilt_initial.add_argument('--initial-tilt', type=int, default=None)
    tilt_initial.add_argument('--initial-tilt-angle', type=float, default=None)
    parser.add_argument('--follow-log', type=Path, default=ROOT/'dataset/follow_logs')
    parser.add_argument('--inventory', action='store_true', help='Read device/PWM paths only; no export/enable')
    parser.add_argument('--no-detection', action='store_true', help='Camera/photographs only, without loading ONNX')
    parser.add_argument('--no-face', action='store_true')
    parser.add_argument('--confidence', type=float, default=.5)
    parser.add_argument('--face-confidence', type=float, default=.6)
    parser.add_argument('--iou', type=float, default=.45)
    parser.add_argument('--face-iou', type=float, default=.3)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--max-frame-age', type=float, default=.75)
    parser.add_argument('--model', type=Path, default=ROOT / 'models/dog_plush/dog_plush_best.onnx')
    parser.add_argument('--face-model', type=Path, default=ROOT / 'models/face_yunet/face_detection_yunet_2026may.onnx')
    args = parser.parse_args()
    if args.inventory:
        print(json.dumps(hardware_inventory(), ensure_ascii=False, indent=2))
        return 0
    try:
        if not ipaddress.ip_address(args.host).is_loopback:
            raise ValueError('Use loopback HTTP with ssh -L; public network binding is not enabled')
        if args.width < 160 or args.height < 120 or not 0 < args.fps <= 120 or args.threads < 1 or not 0 < args.max_frame_age <= 2:
            raise ValueError('Invalid camera size, frame rate, threads or frame-age threshold')
        for value in (args.confidence, args.face_confidence, args.iou, args.face_iou):
            if not 0 < value <= 1:
                raise ValueError('Confidence/IoU must be in (0,1]')
        if args.hardware and (args.pwm_pan is None or args.pwm_tilt is None):
            raise ValueError('--hardware requires both explicitly verified --pwm-pan and --pwm-tilt paths')
        if not args.hardware and (args.pwm_pan is not None or args.pwm_tilt is not None):
            raise ValueError('Add --hardware for real PWM; omit PWM paths for the default mock')
        backend = SysfsPWMBackend({'pan': args.pwm_pan, 'tilt': args.pwm_tilt}) if args.hardware else None
        if args.fpga and args.hardware or args.standalone and not args.fpga:
            raise ValueError('--fpga cannot use --hardware; --standalone requires --fpga')
        follow_link = None
        if args.fpga:
            from robot_follow import FollowLink, FpgaPWMBackend
            tilt_pulse,tilt_angle,migrated=fpga_initial_tilt(args.initial_tilt,args.initial_tilt_angle)
            if migrated:
                # The deployedv20 launcher used2100us (the user's old45deg
                # pose). Preserve that command's compatibility while adopting
                # the user's corrected135deg operating pose via THEIR table.
                print('FOLLOW v22: legacy --initial-tilt2100 migrated to measured --initial-tilt-angle135; PAN unchanged.',flush=True)
            if tilt_angle is not None and not math.isfinite(tilt_angle):
                raise ValueError('TILT启动角度必须是有限数值')
            if not all(1 <= p <= 4095 for p in (args.initial_pan, tilt_pulse)):
                raise ValueError('FPGA initial PWM must be1..4095us')
            backend = FpgaPWMBackend()
            follow_link = FollowLink(backend, args.follow_log, args.initial_pan, tilt_pulse, initial_tilt_angle=tilt_angle)
        calibration = json.loads(args.camera_config.read_text(encoding='utf-8')) if args.camera_config else None
        app = GimbalApp(args.config, backend=backend, camera_config=calibration,
                        dataset=args.dataset, max_age=args.max_frame_age, follow_link=follow_link, standalone=args.standalone)
        if follow_link:follow_link.log_runtime(app)
        camera = UsbCamera(args.camera, args.width, args.height, args.fps, app.gimbal.snapshot)
        # The lambda follows a controller replaced after saving calibration.
        camera.pose_snapshot = lambda: app.gimbal.snapshot()
        worker = VisionWorker(app, camera, args)
        server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
        server.daemon_threads = True
        previous_sigterm = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        backend_name = 'FPGA UDP, outputs disabled' if args.fpga else 'HARDWARE, outputs disabled' if args.hardware else 'MOCK, no physical PWM'
        print(f'USB preview: http://{args.host}:{args.port}; backend={backend_name}', flush=True)
        print('FPGA LCD standalone; log='+str(follow_link.log_path) if args.standalone else
              'Use SSH forwarding. Close browser/lose heartbeat for5s -> PWM disabled. Ctrl+C exits.', flush=True)
        try:
            app.start(camera)
            camera.start()
            worker.thread.start()
            server.serve_forever(poll_interval=.2)
        finally:
            app.close()
            camera.close()
            server.server_close()
            if worker.thread.is_alive():
                worker.thread.join(timeout=2)
            signal.signal(signal.SIGTERM, previous_sigterm)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'USB/gimbal failed: {exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
