"""Conservative, geometric single-target tracking; no hardware or network I/O.

Boxes use ``x1,y1,x2,y2,score,kind`` in the ORIGINAL camera image, with kind
``dog`` or ``human``. Select a particular detection explicitly. Association is
only consecutive-image geometry, not identity recognition or re-identification:
similar targets crossing or reappearing cannot be identified reliably. A missing
target keeps its selected class and last box while fresh observations are
searched for a unique, geometrically consistent three-frame reappearance.
Missing, ambiguous and unconfirmed observations issue no motion increments.

All times share one monotonic seconds clock; frame_time is capture time, not
inference completion time. Frame sequence numbers, when supplied, also increase
strictly. Intrinsics refer to this image size and an ideal/rectified pinhole image.
FOV is used ONLY if explicitly supplied by the caller and is marked approximate.
Without either model normalized image P control is possible, but bearing is None.

Directions are unit rays, never target positions. Axes are X forward, Y left,
Z up; positive pan/yaw is left. TILT is a continuous over-top joint angle,
not a world-elevation angle folded back after90. The reference mount is
upright at0; upright_tilt_deg=180 adds an optical half-turn, reversing local
image Y/Z while keeping the measured joint coordinate. Body conversion requires
both calibrated gimbal angles. Those angles are normally estimates from measured
PWM-to-angle mappings, NOT actual-angle feedback; motion can lag the estimate.
The default body ray is therefore explicitly approximate even with intrinsics.
No depth, target-size distance, camera translation or IMU-integrated 3D position
is invented: distance_m and translation_m always remain None.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Optional


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    hfov_deg: Optional[float] = None
    vfov_deg: Optional[float] = None

    def __post_init__(self):
        if any(not isinstance(v, int) or isinstance(v, bool) or v <= 0
               for v in (self.width, self.height)):
            raise ValueError("Camera width and height must be positive integer pixels")
        intrinsics = (self.fx, self.fy, self.cx, self.cy)
        if any(v is not None for v in intrinsics):
            if not all(_finite(v) for v in intrinsics):
                raise ValueError("Intrinsics require finite fx, fy, cx and cy together")
            if self.fx <= 0 or self.fy <= 0:
                raise ValueError("Camera focal lengths must be positive")
            if not 0 <= self.cx <= self.width or not 0 <= self.cy <= self.height:
                raise ValueError("Principal point must be inside this image")
        elif self.hfov_deg is not None or self.vfov_deg is not None:
            if not all(_finite(v) and 0 < v < 179 for v in (self.hfov_deg, self.vfov_deg)):
                raise ValueError("Approximate projection requires explicit horizontal AND vertical FOV")

    @property
    def model(self):
        if self.fx is not None:
            return "intrinsics"
        if self.hfov_deg is not None:
            return "fov_approximate"
        return "unavailable"

    @property
    def approximate(self):
        return self.model != "intrinsics"

    def projection(self):
        """Return fx,fy,cx,cy, or None. Never infer an unprovided FOV."""
        if self.model == "intrinsics":
            return self.fx, self.fy, self.cx, self.cy
        if self.model == "fov_approximate":
            return (self.width / (2 * math.tan(math.radians(self.hfov_deg) / 2)),
                    self.height / (2 * math.tan(math.radians(self.vfov_deg) / 2)),
                    self.width / 2, self.height / 2)
        return None

    def ray(self, u, v):
        """Optical ray in X-forward/Y-left/Z-up camera coordinates."""
        projection = self.projection()
        if projection is None:
            return None
        fx, fy, cx, cy = projection
        ray = (1.0, (cx - u) / fx, (cy - v) / fy)
        if not all(math.isfinite(component) for component in ray):
            return None
        length = math.hypot(*ray)
        return tuple(component / length for component in ray)


@dataclass(frozen=True)
class TrackerConfig:
    deadband: float = .04
    gain: float = 2.0
    normalized_gain_dps: float = 35.0
    max_rate_dps: float = 25.0
    loss_gap_s: float = .75
    max_frame_age_s: float = .40
    max_control_dt_s: float = .10
    min_score: float = .40
    min_iou: float = .10
    max_center_shift: float = .80
    max_size_ratio: float = 1.80
    ambiguity_margin: float = .12
    future_tolerance_s: float = .02
    # Appended fields preserve the positional order of the original options.
    # Smoothing uses capture-time intervals, never variable inference duration.
    filter_time_constant_s: float = .06
    deadband_hysteresis: float = .015
    min_rate_dps: float = 2.0

    def __post_init__(self):
        if not all(_finite(value) for value in self.__dict__.values()):
            raise ValueError("Tracker settings must be finite numbers")
        if not 0 <= self.deadband < 1:
            raise ValueError("Deadband must lie in [0,1) of image half-size")
        if not 0 <= self.min_score <= 1 or not 0 <= self.min_iou <= 1:
            raise ValueError("Detection score and IoU gates must lie in [0,1]")
        if min(self.gain, self.normalized_gain_dps, self.max_rate_dps) < 0:
            raise ValueError("Control gains and rate limit must be nonnegative")
        if min(self.loss_gap_s, self.max_frame_age_s, self.max_control_dt_s,
               self.max_center_shift) <= 0:
            raise ValueError("Time limits and center gate must be positive")
        if self.max_size_ratio < 1 or self.ambiguity_margin < 0 or self.future_tolerance_s < 0:
            raise ValueError("Invalid association size/margin or future tolerance")
        if self.filter_time_constant_s < 0 or self.min_rate_dps < 0:
            raise ValueError("Filter time constant and minimum rate must be nonnegative")
        if not 0 <= self.deadband_hysteresis < 1:
            raise ValueError("Deadband hysteresis must lie in [0,1) of image half-size")


def _canonical_box(value, camera, min_score):
    if not isinstance(value, Mapping) or value.get("kind") not in ("dog", "human"):
        return None
    names = ("x1", "y1", "x2", "y2", "score")
    if not all(_finite(value.get(name)) for name in names):
        return None
    x1, y1, x2, y2, score = (float(value[name]) for name in names)
    if not min_score <= score <= 1:
        return None
    x1, x2 = max(0.0, min(camera.width, x1)), max(0.0, min(camera.width, x2))
    y1, y2 = max(0.0, min(camera.height, y1)), max(0.0, min(camera.height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return dict(x1=x1, y1=y1, x2=x2, y2=y2, score=score, kind=value["kind"])


def _geometry(box):
    return ((box["x1"] + box["x2"]) / 2, (box["y1"] + box["y2"]) / 2,
            box["x2"] - box["x1"], box["y2"] - box["y1"])


def _iou(a, b):
    overlap = (max(0.0, min(a["x2"], b["x2"]) - max(a["x1"], b["x1"])) *
               max(0.0, min(a["y2"], b["y2"]) - max(a["y1"], b["y1"])))
    area_a = (a["x2"] - a["x1"]) * (a["y2"] - a["y1"])
    area_b = (b["x2"] - b["x1"]) * (b["y2"] - b["y1"])
    return overlap / (area_a + area_b - overlap)


def _limited(value, limit):
    return max(-limit, min(limit, value))


def _remove_deadband(value, band):
    return math.copysign(max(0.0, abs(value) - band), value)


def camera_ray_to_body(ray, pan_deg, tilt_deg, *, upright_tilt_deg=0):
    """Rotate a camera ray by tilt-up, then pan-left. No translation or depth."""
    pan, tilt = math.radians(pan_deg), math.radians(tilt_deg)
    cp, sp, ct, st = math.cos(pan), math.sin(pan), math.cos(tilt), math.sin(tilt)
    sign = mount_image_sign(upright_tilt_deg)
    x, y, z = ray[0], sign * ray[1], sign * ray[2]
    tilted_x, tilted_z = ct * x - st * z, st * x + ct * z
    return cp * tilted_x - sp * y, sp * tilted_x + cp * y, tilted_z


def body_ray_to_world(ray, body_yaw_deg):
    """Yaw-only direction transform; this does not provide a world position."""
    yaw = math.radians(body_yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = ray
    return c * x - s * y, s * x + c * y, z


def mount_image_sign(upright_tilt_deg):
    """Camera local Y/Z relative to the head; no inferred PAN zero offset."""
    if isinstance(upright_tilt_deg, bool) or upright_tilt_deg not in (0, 180):
        raise ValueError('Camera upright TILT endpoint must be0 or180 degrees')
    return 1.0 if upright_tilt_deg == 0 else -1.0


def gimbal_centering_delta(result, tilt_deg, *, max_rate_dps=25.0, upright_tilt_deg=0):
    """Map image-local centering rates to an over-the-top pan/tilt head.

    The reference head orientation is Rz(pan) Ry(-tilt). A camera upright
    at180 rather than0 adds a180deg optical roll: BOTH local Y/Z reverse.
    Keep the user's continuous servo angle table; change neither its zero nor
    the meaning of PAN's mechanical direction.
    In the reference mounting, PAN moves the optical axis along camera-left
    *cos(tilt)* and TILT along camera-up. Both directions reverse for an
    upright-at180 camera. The old independent-axis
    controller omitted that cosine and gave positive feedback beyond90deg.

    Use the damped alignment-gradient direction: horizontal*cos(tilt) /
    (cos(tilt)**2 + .1**2), vertical. The positive damping bounds the geometric
    gain at5, reverses PAN over the top and avoids a1/cos singularity. Clamp
    joint rates AFTER this conversion. Do not freeze a whole interval around
    the zenith: that can strand a visible target with residual horizontal error
    while vertical error is already in its deadband. Exactly vertical has no
    instantaneous horizontal authority; report it without an automatic sweep.
    Pixel-only control needs no invented focal length or bearing for this
    kinematic sign/attenuation. It still cannot produce calibrated body rays.
    ``tilt_deg`` must describe the image capture's command-position estimate.
    """
    if not _finite(tilt_deg):
        raise ValueError('A finite measured TILT angle is required for gimbal centering')
    if not _finite(max_rate_dps) or max_rate_dps < 0:
        raise ValueError('Gimbal joint rate limit must be finite and nonnegative')
    sign = mount_image_sign(upright_tilt_deg)
    cosine = math.cos(math.radians(tilt_deg))
    near_vertical = abs(cosine) < .05
    factor = sign * cosine / (cosine * cosine + .01)
    if abs(cosine) < 1e-12:
        factor = 0.0
    raw_pan, raw_tilt = result.get('yaw_rate_dps', 0.0), result.get('pitch_rate_dps', 0.0)
    pan_rate = _limited(raw_pan * factor, max_rate_dps)
    tilt_rate = _limited(sign * raw_tilt, max_rate_dps)
    return dict(tilt_deg=float(tilt_deg), pan_factor=factor,
                near_vertical=near_vertical,
                side='upright' if sign * cosine >= 0 else 'inverted',
                upright_tilt_deg=upright_tilt_deg,
                delta_pan_deg=result.get('delta_yaw_deg', 0.0) * pan_rate / raw_pan if raw_pan else 0.0,
                delta_tilt_deg=result.get('delta_pitch_deg', 0.0) * tilt_rate / raw_tilt if raw_tilt else 0.0,
                pan_rate_dps=pan_rate, tilt_rate_dps=tilt_rate)


class TargetTracker:
    REACQUIRE_REQUIRED = 3

    def __init__(self, camera, config=None):
        self.camera = camera
        self.config = config if config is not None else TrackerConfig()
        self.clear_target()

    def clear_target(self):
        self._target = None
        self._last_match = self._last_frame = self._last_now = None
        self._last_seq = None
        self._searching = False
        self._clear_reacquire()
        self._clear_control()

    def _clear_reacquire(self):
        self._reacquire_target = None
        self._reacquire_frame = None
        self._reacquire_count = 0

    def _begin_search(self):
        self._searching = True
        self._clear_reacquire()
        self._clear_control()

    def _association_rank(self, anchor, box):
        """Normal consecutive-frame gates, also used between confirmation frames."""
        old_x, old_y, old_w, old_h = _geometry(anchor)
        x, y, w, h = _geometry(box)
        overlap = _iou(box, anchor)
        center_shift = math.hypot(x - old_x, y - old_y) / math.hypot(old_w, old_h)
        size_ratio = max(w / old_w, old_w / w, h / old_h, old_h / h)
        if (overlap < self.config.min_iou or center_shift > self.config.max_center_shift
                or size_ratio > self.config.max_size_ratio):
            return None
        return (.65 * overlap + .25 * (1 - center_shift / self.config.max_center_shift)
                + .10 * box["score"])

    def _clear_control(self):
        self._filtered_error = None
        self._filter_frame = None
        self._control_active = [False, False]

    def _filter_error(self, nx, ny, frame_time):
        """Causal low-pass at the image's capture time; first match is fresh.

        A clear sign reversal outside the start band resets that axis's old
        filter tail. It must not keep pushing toward a target now on the other
        side of the image. Missing/ambiguous observations reset both axes.
        """
        raw = nx, ny
        tau = self.config.filter_time_constant_s
        if self._filtered_error is None or not tau:
            filtered = raw
            dt = 0.0 if self._filter_frame is None else frame_time - self._filter_frame
        else:
            dt = frame_time - self._filter_frame
            alpha = -math.expm1(-dt / tau)
            start_band = min(1.0, self.config.deadband + self.config.deadband_hysteresis)
            filtered = tuple(value if value * previous < 0 and abs(value) > start_band
                             else previous + alpha * (value - previous)
                             for value, previous in zip(raw, self._filtered_error))
        self._filtered_error, self._filter_frame = filtered, frame_time
        return filtered, dt

    def _axis_rate(self, axis, observed, filtered, proportional_rate):
        """Schmitt deadband with a rate floor only while an axis is active.

        The raw observation entering the stop band stops immediately, without
        waiting for a stale filter tail. A zero gain still commands zero. The
        final maximum rate always wins over the configurable rate floor.
        """
        band = self.config.deadband
        start_band = min(1.0, band + self.config.deadband_hysteresis)
        if abs(observed) <= band or abs(filtered) <= band:
            self._control_active[axis] = False
        elif not self._control_active[axis] and abs(filtered) > start_band:
            self._control_active[axis] = True
        if not self._control_active[axis] or proportional_rate == 0:
            return 0.0
        return _limited(math.copysign(max(self.config.min_rate_dps,
                                        abs(proportional_rate)), proportional_rate),
                        self.config.max_rate_dps)

    def select_target(self, box, frame_time, frame_seq=None):
        """Explicit lock/relock. Selection itself never issues a motion command."""
        target = _canonical_box(box, self.camera, self.config.min_score)
        if target is None:
            raise ValueError("Select a valid dog/human detection above the confidence gate")
        if not _finite(frame_time) or frame_time < 0:
            raise ValueError("Selection capture time must be finite monotonic seconds")
        if frame_seq is not None and (not isinstance(frame_seq, int) or isinstance(frame_seq, bool)
                                      or frame_seq < 0):
            raise ValueError("Frame sequence must be a nonnegative integer")
        self.clear_target()
        self._target = target
        self._last_match = self._last_frame = float(frame_time)
        self._last_seq = frame_seq
        return self._result("selected", 0.0)

    def _result(self, status, age, target=None):
        return dict(status=status, target=dict(target) if target else None,
                    last_target=dict(self._target) if self._target else None,
                    kind=self._target["kind"] if self._target else None,
                    frame_age_s=age, candidate_count=0, control_dt_s=0.0,
                    reacquire_count=self._reacquire_count,
                    reacquire_required=self.REACQUIRE_REQUIRED,
                    yaw_error_normalized=None, pitch_error_normalized=None,
                    yaw_error_filtered_normalized=None, pitch_error_filtered_normalized=None,
                    yaw_control_active=False, pitch_control_active=False,
                    filter_dt_s=0.0,
                    yaw_error_deg=None, pitch_error_deg=None,
                    yaw_rate_dps=0.0, pitch_rate_dps=0.0,
                    delta_yaw_deg=0.0, delta_pitch_deg=0.0,
                    camera_model=self.camera.model, approximate=self.camera.approximate,
                    control_basis="none", camera_ray=None, bearing_valid=False,
                    body_direction=None, body_direction_valid=False,
                    body_direction_approximate=None, world_direction=None,
                    world_direction_valid=False, gimbal_angle_source=None,
                    distance_m=None, translation_m=None, position_valid=False,
                    association="geometric_no_reidentification")

    def update(self, detections, frame_time, now, frame_seq=None, pan_deg=None,
               tilt_deg=None, gimbal_calibrated=False, body_yaw_deg=None,
               upright_tilt_deg=0):
        """Consume one new captured image; every rejected/missing frame outputs 0.

        ``gimbal_calibrated`` asserts BOTH pan/tilt use measured angle mappings;
        simply knowing PWM microseconds or setting a GUI flag is insufficient.
        ``pan_deg/tilt_deg`` are command-position estimates, not feedback values.
        Both angles and optional body yaw must describe frame_time at capture,
        not the later inference-completion time; caller supplies this alignment.
        An uncalibrated body mount produces no body/world ray, while camera-only
        bearings and centering can still be used. Caller must also enforce its
        empirically calibrated mechanical limits before applying any delta.
        """
        if not _finite(now) or now < 0 or not _finite(frame_time) or frame_time < 0:
            return self._result("invalid_time", None)
        age = now - frame_time
        if self._last_now is not None and now < self._last_now:
            return self._result("clock_reversed", age)
        self._last_now = now
        if self._target is None:
            return self._result("unselected", age)
        if frame_seq is not None and (not isinstance(frame_seq, int) or isinstance(frame_seq, bool)
                                      or frame_seq < 0):
            return self._result("invalid_sequence", age)
        if frame_time < self._last_frame:
            return self._result("out_of_order", age)
        if frame_time == self._last_frame:
            return self._result("awaiting_new_frame", age)
        if frame_seq is not None and self._last_seq is not None and frame_seq <= self._last_seq:
            return self._result("out_of_order", age)
        if age < -self.config.future_tolerance_s:
            return self._result("future_frame", age)
        if age > self.config.max_frame_age_s:
            self._clear_control()
            self._clear_reacquire()
            return self._result("stale_frame", age)
        previous_frame = self._last_frame
        self._last_frame = frame_time
        if frame_seq is not None:
            self._last_seq = frame_seq
        if age < 0:
            # Tiny timestamp tolerance is not permission to produce a negative
            # age or extrapolate position into an unobserved future.
            age = 0.0
        if not self._searching and now - self._last_match > self.config.loss_gap_s:
            self._begin_search()
        boxes = []
        for value in detections:
            box = _canonical_box(value, self.camera, self.config.min_score)
            if box is None or box["kind"] != self._target["kind"]:
                continue
            boxes.append(box)
        candidates = []
        if not self._searching:
            for box in boxes:
                rank = self._association_rank(self._target, box)
                if rank is not None:
                    candidates.append((rank, box))
            candidates.sort(key=lambda item: item[0], reverse=True)
            if (not candidates or (len(candidates) > 1 and
                    candidates[0][0] - candidates[1][0] <= self.config.ambiguity_margin)):
                self._begin_search()
        confirmed_count = 0
        if self._searching:
            # Reentry can occur anywhere in the image. Keep only the selected
            # class and a broad size plausibility gate; unlike consecutive
            # tracking, do not require overlap with a box left seconds ago.
            _, _, old_w, old_h = _geometry(self._target)
            search_candidates = []
            for box in boxes:
                _, _, w, h = _geometry(box)
                if max(w / old_w, old_w / w, h / old_h, old_h / h) <= max(3.0, self.config.max_size_ratio):
                    search_candidates.append(box)
            if len(search_candidates) != 1:
                self._clear_reacquire()
                self._clear_control()
                result = self._result("ambiguous" if search_candidates else "searching", age)
                result["candidate_count"] = len(search_candidates)
                return result
            candidate = search_candidates[0]
            continuous = (self._reacquire_target is not None
                          and frame_time - self._reacquire_frame <= self.config.loss_gap_s
                          and self._association_rank(self._reacquire_target, candidate) is not None)
            self._reacquire_count = self._reacquire_count + 1 if continuous else 1
            self._reacquire_target, self._reacquire_frame = candidate, frame_time
            self._clear_control()
            if self._reacquire_count < self.REACQUIRE_REQUIRED:
                result = self._result("reacquiring", age)
                result["candidate_count"] = 1
                return result
            confirmed_count = self._reacquire_count
            self._searching = False
            self._clear_reacquire()
            candidates = [(0.0, candidate)]
        self._target = candidates[0][1]
        self._last_match = frame_time
        result = self._result("tracking", age, self._target)
        result["reacquire_count"] = confirmed_count
        result["candidate_count"] = len(candidates)
        result["control_dt_s"] = min(self.config.max_control_dt_s, frame_time - previous_frame)
        x, y, _, _ = _geometry(self._target)
        projection = self.camera.projection()
        cx, cy = projection[2:] if projection is not None else (self.camera.width / 2, self.camera.height / 2)
        nx, ny = (cx - x) / (self.camera.width / 2), (cy - y) / (self.camera.height / 2)
        result["yaw_error_normalized"], result["pitch_error_normalized"] = nx, ny
        (filtered_nx, filtered_ny), filter_dt = self._filter_error(nx, ny, frame_time)
        result.update(yaw_error_filtered_normalized=filtered_nx,
                      pitch_error_filtered_normalized=filtered_ny, filter_dt_s=filter_dt)
        ray = self.camera.ray(x, y)
        if ray is None:
            yaw_rate = self.config.normalized_gain_dps * _remove_deadband(filtered_nx, self.config.deadband)
            pitch_rate = self.config.normalized_gain_dps * _remove_deadband(filtered_ny, self.config.deadband)
            result["control_basis"] = "normalized"
        else:
            yaw_error = math.degrees(math.atan2(ray[1], ray[0]))
            pitch_error = math.degrees(math.atan2(ray[2], math.hypot(ray[0], ray[1])))
            fx, fy, _, _ = projection
            yaw_band = math.degrees(math.atan(self.camera.width * .5 * self.config.deadband / fx))
            filtered_x = filtered_nx * self.camera.width * .5 / fx
            horizontal = math.hypot(1.0, filtered_x)
            pitch_band = math.degrees(math.atan(self.camera.height * .5 * self.config.deadband / (fy * horizontal)))
            filtered_yaw = math.degrees(math.atan(filtered_x))
            filtered_pitch = math.degrees(math.atan(filtered_ny * self.camera.height * .5 / (fy * horizontal)))
            yaw_rate = self.config.gain * _remove_deadband(filtered_yaw, yaw_band)
            pitch_rate = self.config.gain * _remove_deadband(filtered_pitch, pitch_band)
            result.update(yaw_error_deg=yaw_error, pitch_error_deg=pitch_error,
                          camera_ray=ray, bearing_valid=True, control_basis="angular")
            if gimbal_calibrated is True and _finite(pan_deg) and _finite(tilt_deg):
                body = camera_ray_to_body(ray, pan_deg, tilt_deg, upright_tilt_deg=upright_tilt_deg)
                result.update(body_direction=body, body_direction_valid=True,
                              body_direction_approximate=True,
                              gimbal_angle_source="calibrated_command_estimate")
                if _finite(body_yaw_deg):
                    result.update(world_direction=body_ray_to_world(body, body_yaw_deg),
                                  world_direction_valid=True)
        result["yaw_rate_dps"] = self._axis_rate(0, nx, filtered_nx, yaw_rate)
        result["pitch_rate_dps"] = self._axis_rate(1, ny, filtered_ny, pitch_rate)
        result["yaw_control_active"], result["pitch_control_active"] = self._control_active
        result["delta_yaw_deg"] = result["yaw_rate_dps"] * result["control_dt_s"]
        result["delta_pitch_deg"] = result["pitch_rate_dps"] * result["control_dt_s"]
        return result
