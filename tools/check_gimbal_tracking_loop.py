"""Closed-loop camera/gimbal regressions with an independent mechanical plant.

No image packages, camera, PWM device, FPGA, socket or wall-clock sleeps are
used. The real GimbalApp consumes detections projected from a fixed 3D point;
its real controller writes to MockPWMBackend. This test reads those emitted
pulses to move the simulated camera before rendering the NEXT image. Both
physical camera installations are rendered independently. The old upright-at-zero
assumption pushes the user's upright-at-180 target outwards, reproducing the
reported hardware failure.

The pulse/angle points reproduce the user's measured table. They define a
continuous command coordinate, not a claim that those numbers equal body yaw
and pitch. The plant independently applies the physical installation direction.
"""

from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
import usb_gimbal
from gimbal_control import MockPWMBackend, default_config
from usb_gimbal import GimbalApp, VisionFrame


# Each pair is (pulse_us, measured command angle_deg). Interpolation below is
# deliberately independent of AxisConfig.angle_to_pulse / pulse_to_angle.
MEASURED = {
    "pan": [(380, 45), (1882, 180), (2550, 240)],
    "tilt": [(2420, 10), (2043, 45), (645, 175)],
}
MOUNTS = ("upright_at_zero", "upright_at_180")


def physical_coordinate(axis, pulse):
    points = sorted(MEASURED[axis])
    for (p0, a0), (p1, a1) in zip(points, points[1:]):
        if p0 <= pulse <= p1:
            return a0 + (pulse - p0) * (a1 - a0) / (p1 - p0)
    raise AssertionError(f"Emitted pulse escaped the measured plant: {axis}={pulse}")


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def pulse_for(axis, angle):
    points = sorted(MEASURED[axis], key=lambda pair: pair[1])
    for (p0, a0), (p1, a1) in zip(points, points[1:]):
        if a0 <= angle <= a1:
            return round(p0 + (angle - a0) * (p1 - p0) / (a1 - a0))
    raise AssertionError("Test requested an angle outside measured travel")


def config_for(*, upright_tilt_deg=0):
    config = default_config()
    config["mount"] = dict(upright_tilt_deg=upright_tilt_deg)
    config["axes"]["pan"].update(explore_min_us=340, explore_max_us=2600,
        safe_min_us=340, safe_max_us=2600)
    config["axes"]["tilt"].update(safe_min_us=645, safe_max_us=2420)
    for axis in ("pan", "tilt"):
        config["axes"][axis].update(calibration_confirmed=True,
            calibration_points=[dict(pulse_us=p, angle_deg=a)
                                for p, a in sorted(MEASURED[axis], key=lambda x: x[1])])
    # Intrinsics are used by this synthetic camera's rendering, but deliberately
    # NOT supplied to the application. Recentring must work without a checkerboard.
    config["camera"] = dict(width=640, height=480)
    return config


def legacy_v15_delta(result, tilt_deg, *, max_rate_dps=25.0, **_):
    """Frozen v15 optical model, deliberately unaware of camera mounting.

    This negative control does not call the current production helper. It
    reproduces its previous theta-as-pitch cosine and positive TILT direction.
    The same renderer, detections, association and PWM actuator are retained.
    """
    cosine = math.cos(math.radians(tilt_deg))
    factor = cosine / (cosine * cosine + .01)
    if abs(cosine) < 1e-12:
        factor = 0.0
    pan_raw, tilt_raw = result.get("yaw_rate_dps", 0), result.get("pitch_rate_dps", 0)
    pan_rate = max(-max_rate_dps, min(max_rate_dps, pan_raw * factor))
    tilt_rate = max(-max_rate_dps, min(max_rate_dps, tilt_raw))
    return dict(tilt_deg=tilt_deg, pan_factor=factor,
        near_vertical=abs(cosine) < .05,
        upright_tilt_deg=0,
        side="over_top" if cosine < 0 else "front",
        delta_pan_deg=result.get("delta_yaw_deg", 0) * pan_rate / pan_raw if pan_raw else 0,
        delta_tilt_deg=result.get("delta_pitch_deg", 0) * tilt_rate / tilt_raw if tilt_raw else 0,
        pan_rate_dps=pan_rate, tilt_rate_dps=tilt_rate)


class CameraPlant:
    """Position servos plus delayed captures; only backend output moves them."""

    def __init__(self, test, *, installation=None,
                 target=(8.0, 0.0), delay=0.0, initial_pulses=None, absolute_pose=True,
                 tracking_options=None, mount="upright_at_zero", configured_upright_tilt_deg=None,
                 camera_options=None):
        if mount not in MOUNTS:
            raise ValueError("Unknown physical camera installation")
        self.mount = mount
        self.installation = installation or dict(pan=1, tilt=1)
        #20deg is below the over-top singularity. Tests explicitly use the
        # user's original1500us and80/100/160deg starts where relevant.
        self.initial = initial_pulses or dict(pan=1500, tilt=2312)
        self.absolute_pose = absolute_pose
        self.delay = delay
        self.now = 1000.0
        self.sequence = 0
        self.width, self.height = 640, 480
        self.next_capture = self.now
        self.pending = deque()
        self.visible = True
        self.camera_enabled = True
        self.target = None
        self.samples = []
        self.motion_samples = []
        self.actuator_samples = []
        self.temporary = tempfile.TemporaryDirectory(
            prefix="tracking-loop-", dir=Path(__file__).resolve().parent)
        assert Path(self.temporary.name).resolve().parent == Path(__file__).resolve().parent
        test.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "config.json"
        # Physical installation and app model remain separately specifiable.
        # The wrong-model regression holds the plant fixed at180 while giving
        # the actual application the old0deg configuration.
        configured_upright = (0 if mount == "upright_at_zero" else 180)
        if configured_upright_tilt_deg is not None:
            configured_upright = configured_upright_tilt_deg
        config = config_for(upright_tilt_deg=configured_upright)
        if camera_options:
            config["camera"].update(camera_options)
        if tracking_options:
            config["tracking"] = dict(tracking_options)
        self.path.write_text(json.dumps(config), encoding="utf8")
        clock = SimpleNamespace(monotonic=lambda: self.now, perf_counter=time.perf_counter)
        self.clock_patch = mock.patch.object(usb_gimbal, "time", clock)
        self.clock_patch.start()
        test.addCleanup(self.clock_patch.stop)
        self.backend = MockPWMBackend()
        self.app = GimbalApp(self.path, backend=self.backend)
        self.app.gimbal._clock = lambda: self.now
        test.addCleanup(self.app.close)
        for axis in ("pan", "tilt"):
            self.app.command(dict(action="manual_enable", axis=axis, pulse_us=self.initial[axis]))
        self.set_image_target_offset(*target)
        # Produce one genuine captured/delivered frame before explicit selection.
        self.advance(self.delay + .12)
        self.select_and_start()

    def orientation(self):
        return {axis: (physical_coordinate(axis, self.initial[axis]) if self.absolute_pose else 0) +
            self.installation[axis] * (physical_coordinate(axis, self.backend.enabled[axis]) -
            physical_coordinate(axis, self.initial[axis])) for axis in ("pan", "tilt")}

    def camera_basis(self):
        pose = self.orientation()
        # This is the INDEPENDENT mechanical renderer, not the production
        # camera_ray_to_body or centering helper. The measured servo table is
        # unchanged by physically mounting the camera the opposite way.
        yaw, pitch = math.radians(pose["pan"]), math.radians(pose["tilt"])
        forward = (math.cos(yaw) * math.cos(pitch),
                   math.sin(yaw) * math.cos(pitch), math.sin(pitch))
        left = (-math.sin(yaw), math.cos(yaw), 0.0)
        up = (-math.cos(yaw) * math.sin(pitch),
              -math.sin(yaw) * math.sin(pitch), math.cos(pitch))
        # Correcting "0deg upright" to "180deg upright" reverses the
        # image's left/up basis (a180deg optical-axis roll), without inventing
        # a180deg change in the camera's pointing direction. Consequently PAN
        # image authority is -cos(theta), and TILT image authority is -1.
        if self.mount == "upright_at_180":
            left = tuple(-value for value in left)
            up = tuple(-value for value in up)
        return forward, left, up

    def set_image_target_offset(self, horizontal_deg, vertical_deg):
        # Build a FIXED world point from the present camera basis. Subsequent
        # camera motion must project this same point, not recreate a screen error.
        forward, left, up = self.camera_basis()
        h, v = math.tan(math.radians(horizontal_deg)), math.tan(math.radians(vertical_deg))
        ray = tuple(f + h * l + v * u for f, l, u in zip(forward, left, up))
        length = math.sqrt(dot(ray, ray))
        self.target = tuple(component / length for component in ray)

    def image_position(self):
        forward, left, up = self.camera_basis()
        depth = dot(self.target, forward)
        if depth <= 0:
            return None
        focal = 400 * self.width / 640
        return (self.width / 2 - focal * dot(self.target, left) / depth,
                self.height / 2 - focal * dot(self.target, up) / depth)

    def normalized_error(self):
        pos = self.image_position()
        if pos is None:
            return dict(pan=float("inf"), tilt=float("inf"))
        return dict(pan=(self.width / 2 - pos[0]) / (self.width / 2),
                    tilt=(self.height / 2 - pos[1]) / (self.height / 2))

    def capture(self):
        self.sequence += 1
        pos = self.image_position()
        boxes = []
        if self.visible and pos is not None and 0 <= pos[0] <= self.width and 0 <= pos[1] <= self.height:
            x, y = pos
            boxes = [dict(x1=x - 80, y1=y - 80, x2=x + 80, y2=y + 80,
                          score=.95, kind="human")]
        frame = VisionFrame(sequence=self.sequence, arrived_at=self.now,
            image=object(), jpeg=b"synthetic", detections=boxes,
            gimbal_at_arrival=deepcopy(self.app.gimbal.snapshot()),
            width=self.width, height=self.height, inference_ms=self.delay * 1000,
            coordinates="original USB pixels")
        self.pending.append((self.now + self.delay, frame))

    def advance(self, seconds):
        # Control ticks at 50Hz, camera captures at10Hz, inference may finish
        # later. Pose is pinned at CAPTURE, not retimestamped when delivered.
        end = self.now + seconds
        while self.now < end - 1e-8:
            self.now = min(end, self.now + .02)
            if self.camera_enabled and self.now >= self.next_capture - 1e-8:
                self.capture()
                self.next_capture += .1
            while self.pending and self.pending[0][0] <= self.now + 1e-8:
                _, frame = self.pending.popleft()
                self.app.publish(frame)
            self.app.command(dict(action="heartbeat"))
            self.app._control_once(self.now)
            self.actuator_samples.append((self.now,
                {axis: physical_coordinate(axis, self.backend.enabled[axis])
                 for axis in ("pan", "tilt")}))
            self.samples.append(self.normalized_error())
            result = self.app.tracking_result
            if result and "pan_command_rate_dps" in result:
                self.motion_samples.append({name: result[name] for name in (
                    "pan_command_rate_dps", "tilt_command_rate_dps", "control_dt_s",
                    "delta_pan_command_deg", "delta_tilt_command_deg", "tilt_pose_deg")})

    def select_and_start(self):
        self.app.command(dict(action="select", sequence=self.app.latest.sequence, index=0))
        self.app.command(dict(action="track", enabled=True))

    def learn_from_stationary_target(self, test):
        """Use the public app workflow and camera images, never fitted rows.

        The renderer keeps the same world point through all four manual moves.
        Every sample is reconstructed by GimbalApp from its own captured
        detection plus the output PWM snapshot, just as the webpage records it.
        """
        self.app.command(dict(action="track", enabled=False))
        base = dict(self.backend.enabled)

        def restore_and_settle():
            for axis in ("pan", "tilt"):
                self.app.command(dict(action="manual_pulse", axis=axis, pulse_us=base[axis]))
            self.advance(1.1 + self.delay)
            for state in self.app.latest.gimbal_at_arrival["axes"].values():
                test.assertEqual(state["target_us"], state["output_us"])

        for axis in ("pan", "tilt"):
            for direction in (1, -1):
                restore_and_settle()
                test.assertEqual(len(self.app.latest.detections), 1)
                self.app.command(dict(action="calibration_begin", sequence=self.app.latest.sequence, index=0))
                requested = physical_coordinate(axis, base[axis]) + direction * 4
                self.app.command(dict(action="manual_pulse", axis=axis, pulse_us=pulse_for(axis, requested)))
                self.advance(1.1 + self.delay)
                for state in self.app.latest.gimbal_at_arrival["axes"].values():
                    test.assertEqual(state["target_us"], state["output_us"])
                self.app.command(dict(action="calibration_finish", sequence=self.app.latest.sequence, index=0))
        state = self.app.snapshot()["image_calibration"]
        test.assertEqual(state["sample_count"], 4)
        test.assertTrue(state["fit"]["ready"], state["fit"]["reason"])
        test.assertFalse(state["enabled"])
        self.app.command(dict(action="calibration_apply"))
        test.assertTrue(self.app.snapshot()["image_calibration"]["enabled"])
        restore_and_settle()
        test.assertEqual(self.backend.enabled, base)
        return deepcopy(self.app.config["image_calibration"])


class ClosedLoopTests(unittest.TestCase):
    def test_independent_physical_180_mount_is_upright_at_180_and_inverted_at_zero(self):
        # Audit the renderer itself against the physical endpoint description.
        # No production ray/centering routine supplies these expected bases.
        plant = object.__new__(CameraPlant)
        plant.mount = "upright_at_180"
        pose = dict(pan=0, tilt=0)
        plant.orientation = lambda: pose
        forward, _, up = plant.camera_basis()
        self.assertAlmostEqual(forward[0], 1)
        self.assertAlmostEqual(up[2], -1)
        pose["tilt"] = 180
        forward, _, up = plant.camera_basis()
        self.assertAlmostEqual(forward[0], -1)
        self.assertAlmostEqual(up[2], 1)
        pose["tilt"] = 90
        forward, _, _ = plant.camera_basis()
        self.assertAlmostEqual(forward[2], 1)

        # Numerical motion of its optical axis relative to its own image axes:
        # both PAN and TILT change sign under the opposite camera installation.
        epsilon = 1e-5
        for mount in MOUNTS:
            plant.mount = mount
            sign = 1 if mount == "upright_at_zero" else -1
            for theta in (0, 45, 89.9, 90, 90.1, 135, 180):
                pose.update(pan=23, tilt=theta)
                forward, left, up = plant.camera_basis()
                pose["pan"] += math.degrees(epsilon)
                moved, _, _ = plant.camera_basis()
                derivative = tuple((a - b) / epsilon for a, b in zip(moved, forward))
                self.assertAlmostEqual(dot(derivative, left), sign * math.cos(math.radians(theta)), places=8)
                pose.update(pan=23, tilt=theta + math.degrees(epsilon))
                moved, _, _ = plant.camera_basis()
                derivative = tuple((a - b) / epsilon for a, b in zip(moved, forward))
                self.assertAlmostEqual(dot(derivative, up), sign, places=8)

    def test_full_3d_projection_both_sides_and_over_top_angles_converge(self):
        #80/100deg straddle the singularity;1500us is the user's actual start,
        # approximately95.49deg. At high tilt yaw about the BODY Z axis is not
        # the same motion as camera-horizontal yaw.
        for mount in MOUNTS:
            for theta in (20, 80, 100, 160, physical_coordinate("tilt", 1500)):
                for offsets in ((8, 0), (-8, 0), (0, 8), (0, -8), (6, -6), (-6, 6)):
                    with self.subTest(mount=mount, theta=theta, offsets=offsets):
                        plant = CameraPlant(self, target=offsets, mount=mount,
                            initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)))
                        initial = math.hypot(*plant.normalized_error().values())
                        # Gradient control intentionally slows body yaw near zenith;
                        # the simulation advances virtual time, never sleeps.
                        plant.advance(360 if 75 < theta < 105 else 20)
                        final = math.hypot(*plant.normalized_error().values())
                        # Each Schmitt controller stops inside its deadband or the
                        # retained hysteresis band (.04+.015), not a circular band.
                        self.assertLess(max(abs(v) for v in plant.normalized_error().values()), .056)
                        self.assertLess(final, initial * .5)
                        self.assertTrue(plant.app.tracking)

    def test_centered_target_does_not_drift_with_user_measured_tables(self):
        for mount in MOUNTS:
            with self.subTest(mount=mount):
                plant = CameraPlant(self, target=(0, 0), mount=mount,
                    initial_pulses=dict(pan=1500, tilt=1500))
                emitted = list(plant.backend.events)
                plant.advance(20)
                self.assertEqual(plant.backend.enabled, plant.initial)
                self.assertEqual(plant.backend.events, emitted)
                self.assertLess(abs(plant.normalized_error()["pan"]), 1e-12)
                self.assertLess(abs(plant.normalized_error()["tilt"]), 1e-12)

    def test_v15_zero_upright_model_reproduces_users_180_upright_runaway(self):
        # Fixed physical installation is180deg-upright in BOTH arms of this
        # comparison. Only the controller geometry changes; target selection,
        # PWM interpolation and the renderer do not borrow production transforms.
        for theta in (45, 135, 160):
            for offsets in ((8, 0), (0, 8), (0, -8), (-6, 6)):
                with self.subTest(theta=theta, offsets=offsets):
                    plant = CameraPlant(self, mount="upright_at_180", target=offsets,
                        initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)))
                    initial = math.hypot(*plant.normalized_error().values())
                    with mock.patch.object(usb_gimbal, "gimbal_centering_delta", legacy_v15_delta):
                        plant.advance(.8)
                    old_error = math.hypot(*plant.normalized_error().values())
                    self.assertGreater(old_error, initial * 1.08)
                    plant.advance(25)
                    self.assertLess(max(abs(v) for v in plant.normalized_error().values()), .056)
                    self.assertTrue(plant.app.tracking)

    def test_physical_180_mount_and_application_model_are_independent(self):
        for configured_upright in (0, 180):
            with self.subTest(configured_upright=configured_upright):
                plant = CameraPlant(self, mount="upright_at_180", target=(0, 8),
                    configured_upright_tilt_deg=configured_upright,
                    initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 135)))
                initial = abs(plant.normalized_error()["tilt"])
                plant.advance(.8)
                error = abs(plant.normalized_error()["tilt"])
                if configured_upright == 0:
                    self.assertGreater(error, initial * 1.08)
                else:
                    self.assertLess(error, initial * .7)

    def test_real_sample_workflow_overrides_wrong_mount_and_closes_loop_without_drift(self):
        # This intentionally retains the WRONG old geometric model. The only
        # way to center this physical180deg installation is to learn from its
        # actual image response; a correct-mount fallback cannot hide a bug.
        for delay in (0, .17):
            with self.subTest(delay=delay):
                plant = CameraPlant(self, mount="upright_at_180", configured_upright_tilt_deg=0,
                    target=(0, 0), delay=delay,
                    initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 135)),
                    tracking_options=dict(max_rate_dps=5))
                learned = plant.learn_from_stationary_target(self)
                self.assertEqual(plant.app.config["mount"]["upright_tilt_deg"], 0)
                persisted = json.loads(plant.path.read_text(encoding="utf8"))
                self.assertEqual(persisted["image_calibration"], learned)
                self.assertEqual(persisted["axes"], config_for()["axes"])
                plant.select_and_start()
                stable = dict(plant.backend.enabled)
                plant.advance(2)
                self.assertEqual(plant.backend.enabled, stable)
                self.assertEqual(plant.app.tracking_result["control_source"], "sampled_image_jacobian")

                for offset in ((8, -6), (-8, 6)):
                    plant.set_image_target_offset(*offset)
                    initial = math.hypot(*plant.normalized_error().values())
                    plant.advance(20)
                    self.assertEqual(plant.app.tracking_result["control_source"], "sampled_image_jacobian")
                    self.assertLess(math.hypot(*plant.normalized_error().values()), initial * .5)
                    self.assertLess(max(abs(v) for v in plant.normalized_error().values()), .056)
                    self.assertTrue(plant.app.tracking)
                    settled = dict(plant.backend.enabled)
                    plant.advance(2)
                    self.assertEqual(plant.backend.enabled, settled)
                for sample in plant.motion_samples:
                    for axis in ("pan", "tilt"):
                        self.assertLessEqual(abs(sample[f"{axis}_command_rate_dps"]), 5 + 1e-12)
                        self.assertLessEqual(abs(sample[f"delta_{axis}_command_deg"]),
                                             5 * sample["control_dt_s"] + 1e-12)

    def test_learned_application_falls_back_on_resolution_change(self):
        plant = CameraPlant(self, mount="upright_at_180", configured_upright_tilt_deg=0,
            target=(0, 0), initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 135)))
        plant.learn_from_stationary_target(self)
        plant.width, plant.height = 800, 600
        plant.set_image_target_offset(0, 5)
        plant.advance(.2)
        self.assertFalse(plant.app.tracking)
        plant.select_and_start()
        plant.advance(.2)
        self.assertEqual(plant.app.tracking_result["control_source"], "mount_geometry")
        self.assertEqual(plant.app.latest.width, 800)
        self.assertEqual(plant.app.image_calibration_model["width"], 640)

    def test_learned_application_disables_changed_angle_table_signature(self):
        plant = CameraPlant(self, mount="upright_at_180", configured_upright_tilt_deg=0,
            target=(0, 0), initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 135)))
        plant.learn_from_stationary_target(self)
        old_model = deepcopy(plant.app.image_calibration_model)
        points = deepcopy(plant.app.config["axes"]["pan"]["calibration_points"])
        points[1]["angle_deg"] += 1
        plant.app.command(dict(action="save_angles", axis="pan", calibration_points=points))
        state = plant.app.snapshot()["image_calibration"]
        self.assertFalse(state["enabled"])
        self.assertFalse(state["fit"]["ready"])
        self.assertIn("角度表已变化", state["fit"]["reason"])
        self.assertEqual(plant.app.image_calibration_model, old_model)
        plant.set_image_target_offset(0, 5)
        plant.advance(.2)
        plant.select_and_start()
        plant.advance(.2)
        self.assertEqual(plant.app.tracking_result["control_source"], "mount_geometry")

    def test_v14_raw_pan_delta_reproduces_over_top_positive_feedback(self):
        # This ONE injected legacy mapping is the oldv14 boundary: raw image
        # delta was applied directly as body pan. The target renderer, tracker,
        # actuator limits and PWM slew remain identical for the comparison.
        def legacy_delta(result, tilt_deg, **_):
            return dict(delta_pan_deg=result.get("delta_yaw_deg", 0),
                        delta_tilt_deg=result.get("delta_pitch_deg", 0),
                        pan_factor=1, near_vertical=False, tilt_deg=tilt_deg,
                        side="legacy_independent_axes",
                        pan_rate_dps=result.get("yaw_rate_dps", 0),
                        tilt_rate_dps=result.get("pitch_rate_dps", 0))
        for theta in (100, 160):
            with self.subTest(theta=theta):
                plant = CameraPlant(self, target=(8, 0),
                    initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)))
                initial = math.hypot(*plant.normalized_error().values())
                with mock.patch.object(usb_gimbal, "gimbal_centering_delta", legacy_delta):
                    plant.advance(1)
                old_error = math.hypot(*plant.normalized_error().values())
                self.assertGreater(old_error, initial * 1.1)
                # The same still-visible target can be recovered immediately by
                # the corrected camera geometry, without a new calibration.
                plant.advance(360 if theta == 100 else 25)
                self.assertLess(max(abs(v) for v in plant.normalized_error().values()), .056)

    def test_target_loss_finishes_one_last_seen_goal_then_holds_without_reintegrating(self):
        plant = CameraPlant(self, target=(8, -8), mount="upright_at_180",
            initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 135)),
            camera_options=dict(fx=400, fy=400, cx=320, cy=240),
            tracking_options=dict(max_rate_dps=5))
        plant.advance(.4)
        plant.visible = False
        plant.advance(.2)
        self.assertIn(plant.app.tracking_result["status"], ("missing", "lost", "searching"))
        memory = plant.app.snapshot()["tracking_memory"]
        self.assertIn(memory["phase"], ("centering_last_seen", "waiting"))
        goal = deepcopy(memory["goal"])
        error = math.hypot(*plant.normalized_error().values())
        plant.advance(10)
        self.assertTrue(plant.app.tracking)
        self.assertLess(math.hypot(*plant.normalized_error().values()), error * .5)
        self.assertLess(max(abs(value) for value in plant.normalized_error().values()), .056)
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["goal"], goal)
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "waiting")
        outputs = dict(plant.backend.enabled)
        plant.advance(30)
        self.assertEqual(plant.backend.enabled, outputs)
        for state in plant.app.gimbal.snapshot()["axes"].values():
            self.assertEqual(state["target_us"], state["output_us"])

    def test_delayed_inference_keeps_negative_feedback_before_and_after_over_top(self):
        for mount in MOUNTS:
            for theta in (20, 100, 160):
                for offsets in ((8, -6), (-8, 6)):
                    with self.subTest(mount=mount, theta=theta, offsets=offsets):
                        plant = CameraPlant(self, target=offsets, delay=.17, mount=mount,
                            initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)))
                        initial = math.hypot(*plant.normalized_error().values())
                        plant.advance(360 if theta == 100 else 25)
                        errors = [math.hypot(*s.values()) for s in plant.samples]
                        self.assertLess(max(errors), initial + .003)
                        self.assertLess(max(abs(v) for v in plant.normalized_error().values()), .056)
                        self.assertTrue(plant.app.tracking)
                        self.assertAlmostEqual(plant.app.tracking_result["frame_age_s"], .18, places=6)

    def test_stale_frames_stop_and_do_not_continue_integrating(self):
        plant = CameraPlant(self, target=(8, 0))
        plant.advance(.8)
        plant.camera_enabled = False
        plant.advance(1)
        self.assertEqual(plant.app.tracking_result["status"], "stale")
        outputs = dict(plant.backend.enabled)
        plant.advance(1)
        self.assertEqual(plant.backend.enabled, outputs)

    def test_measured_limit_does_not_wind_up_and_target_reversal_leaves_limit(self):
        for mount in MOUNTS:
            image_sign = 1 if mount == "upright_at_zero" else -1
            for axis in ("pan", "tilt"):
                with self.subTest(mount=mount, axis=axis):
                    initial = dict(pan=1500, tilt=2312)
                    initial[axis] = 2540 if axis == "pan" else 650
                    plant = CameraPlant(self, initial_pulses=initial, mount=mount,
                        target=(image_sign * 8, 0) if axis == "pan" else (0, image_sign * 8))
                    plant.advance(5)
                    self.assertTrue(plant.app.tracking_limits[axis])
                    limit_pulse = 2550 if axis == "pan" else 645
                    limit_angle = 240 if axis == "pan" else 175
                    self.assertEqual(plant.backend.enabled[axis], limit_pulse)
                    self.assertAlmostEqual(plant.app.tracking_angles[axis], limit_angle)
                    # Same human shifts back inside travel. The detector boxes still
                    # overlap, so this exercises reversal without resetting the lock.
                    plant.set_image_target_offset(image_sign * -5 if axis == "pan" else 0,
                                                  image_sign * -5 if axis == "tilt" else 0)
                    plant.advance(.3)
                    self.assertFalse(plant.app.tracking_limits[axis])
                    self.assertNotEqual(plant.backend.enabled[axis], limit_pulse)
                    self.assertLess(plant.app.tracking_angles[axis], limit_angle)
                    plant.advance(6)
                    self.assertLess(abs(plant.normalized_error()[axis]), .042)

    def test_exact_and_quantized_zenith_have_no_rate_explosion(self):
        for mount in MOUNTS:
            with self.subTest(mount=mount):
                upright = 0 if mount == "upright_at_zero" else 180
                exact = usb_gimbal.gimbal_centering_delta(dict(
                    delta_yaw_deg=2, delta_pitch_deg=1, yaw_rate_dps=20, pitch_rate_dps=10),
                    90, upright_tilt_deg=upright)
                self.assertEqual(exact["delta_pan_deg"], 0)
                self.assertEqual(exact["pan_rate_dps"], 0)
                plant = CameraPlant(self, target=(8, 0), mount=mount,
                    initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", 90)))
                start_pan = plant.backend.enabled["pan"]
                plant.advance(1)
                #1559us actually means90.007deg in the measured table; a tiny
                # correction is valid. It must not become a1/cos(theta) rate explosion.
                self.assertLessEqual(abs(plant.backend.enabled["pan"] - start_pan), 2)
                self.assertLess(max(abs(m["pan_command_rate_dps"]) for m in plant.motion_samples), .15)
                # Opposite installations reverse TILT's image effect, including
                # the near-vertical pose where PAN has little authority.
                plant.set_image_target_offset(0, -8 if upright == 0 else 8)
                plant.advance(.3)
                self.assertLess(plant.orientation()["tilt"], 90)
                self.assertLess(abs(plant.normalized_error()["tilt"]), .24)

    def test_tilt_crosses_90_continuously_without_flipping_its_own_correction(self):
        for mount in MOUNTS:
            image_sign = 1 if mount == "upright_at_zero" else -1
            for theta, joint_direction, expected_side in ((80, 15, 1), (100, -15, -1)):
                vertical = joint_direction * image_sign
                with self.subTest(mount=mount, theta=theta, vertical=vertical):
                    plant = CameraPlant(self, target=(0, vertical), mount=mount,
                        initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)))
                    plant.advance(10)
                    self.assertGreater((plant.orientation()["tilt"] - 90) * expected_side, 0)
                    self.assertLess(abs(plant.normalized_error()["tilt"]), .056)
                    self.assertTrue(all(m["tilt_command_rate_dps"] * joint_direction >= 0
                                        for m in plant.motion_samples))
                    self.assertEqual(plant.backend.enabled["pan"], 1500)

    def test_pose_compensation_enforces_joint_rate_after_damped_inverse(self):
        for mount in MOUNTS:
            for theta in (80, 89, 91, 100):
                with self.subTest(mount=mount, theta=theta):
                    plant = CameraPlant(self, target=(20, 10), mount=mount,
                        initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)),
                        tracking_options=dict(max_rate_dps=5))
                    plant.advance(2)
                    self.assertTrue(plant.motion_samples)
                    for sample in plant.motion_samples:
                        for name in ("pan", "tilt"):
                            self.assertLessEqual(abs(sample[f"{name}_command_rate_dps"]), 5 + 1e-12)
                            self.assertLessEqual(abs(sample[f"delta_{name}_command_deg"]),
                                                 5 * sample["control_dt_s"] + 1e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
