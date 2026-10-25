"""Actual-app last-observation/reacquisition tests with an independent camera.

Uses the measured PWM tables and fixed-world renderer from the closed-loop
regression. Hiding a detection does not move the physical target; only emitted
PWM can improve its image error. No camera, PWM hardware, HDL or network I/O.
"""

from copy import deepcopy
import math
import unittest

from check_gimbal_tracking_loop import CameraPlant, physical_coordinate, pulse_for


INTRINSICS = dict(fx=400, fy=400, cx=320, cy=240)


class TrackingMemoryTests(unittest.TestCase):
    def make_plant(self, *, theta=135, target=(8, -6), learned=False, normalized=False):
        plant = CameraPlant(self, mount="upright_at_180",
            configured_upright_tilt_deg=0 if learned else 180,
            initial_pulses=dict(pan=1500, tilt=pulse_for("tilt", theta)),
            target=(0, 0) if learned else target,
            camera_options=None if normalized or learned else INTRINSICS,
            tracking_options=dict(max_rate_dps=5))
        if learned:
            plant.learn_from_stationary_target(self)
            plant.set_image_target_offset(*target)
            plant.advance(.2)
            plant.select_and_start()
        return plant

    def assert_motion_rate_bounded(self, samples, limit=5):
        # A rounded PWM quantum is about.09deg for the measured tables. Use
        # real emitted outputs over.2s windows, not reported controller rates.
        for index in range(10, len(samples)):
            then, before = samples[index - 10]
            now, after = samples[index]
            for axis in ("pan", "tilt"):
                self.assertLessEqual(abs(after[axis] - before[axis]),
                                     limit * (now - then) + .2)

    def test_learned_last_seen_goal_overrides_wrong_mount_and_finishes_only_once(self):
        plant = self.make_plant(learned=True)
        plant.advance(.4)
        self.assertEqual(plant.app.tracking_result["control_source"], "sampled_image_jacobian")
        plant.visible = False
        start = len(plant.actuator_samples)
        error = math.hypot(*plant.normalized_error().values())
        plant.advance(.2)
        memory = plant.app.snapshot()["tracking_memory"]
        self.assertIn(memory["phase"], ("centering_last_seen", "waiting"))
        self.assertEqual(memory["source"], "sampled_image_jacobian")
        goal = deepcopy(memory["goal"])
        plant.advance(12)
        self.assertTrue(plant.app.tracking)
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "waiting")
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["goal"], goal)
        self.assertLess(math.hypot(*plant.normalized_error().values()), error * .5)
        self.assertLess(max(abs(value) for value in plant.normalized_error().values()), .056)
        self.assert_motion_rate_bounded(plant.actuator_samples[start:])
        held = dict(plant.backend.enabled)
        plant.advance(30)
        self.assertEqual(plant.backend.enabled, held)
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["goal"], goal)

    def test_reappearance_at_another_image_position_automatically_resumes(self):
        plant = self.make_plant()
        plant.advance(.4)
        plant.visible = False
        plant.advance(12)
        self.assertTrue(plant.app.tracking)
        self.assertTrue(plant.app.target_selected)
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "waiting")
        plant.set_image_target_offset(-8, 6)
        error = math.hypot(*plant.normalized_error().values())
        plant.visible = True
        plant.advance(.2)
        self.assertIn(plant.app.tracking_result["status"], ("reacquiring", "tracking"))
        plant.advance(20)
        self.assertEqual(plant.app.tracking_result["status"], "tracking")
        self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "live")
        self.assertTrue(plant.app.tracking)
        self.assertLess(math.hypot(*plant.normalized_error().values()), error * .5)
        self.assertLess(max(abs(value) for value in plant.normalized_error().values()), .056)

    def test_explicit_pause_and_manual_adjustment_cancel_memory_and_auto_restart(self):
        for action in ("pause", "manual"):
            with self.subTest(action=action):
                plant = self.make_plant()
                plant.advance(.4)
                plant.visible = False
                plant.advance(.2)
                if action == "pause":
                    plant.app.command(dict(action="track", enabled=False))
                else:
                    plant.app.command(dict(action="manual_pulse", axis="pan",
                                           pulse_us=plant.backend.enabled["pan"]))
                held = dict(plant.backend.enabled)
                self.assertFalse(plant.app.tracking)
                self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "idle")
                plant.visible = True
                plant.advance(5)
                self.assertEqual(plant.backend.enabled, held)
                self.assertFalse(plant.app.tracking)

    def test_stale_camera_stops_an_in_progress_last_seen_goal(self):
        plant = self.make_plant()
        plant.advance(.4)
        plant.visible = False
        plant.advance(.2)
        plant.camera_enabled = False
        plant.advance(1)
        self.assertEqual(plant.app.tracking_result["status"], "stale")
        held = dict(plant.backend.enabled)
        plant.advance(12)
        self.assertEqual(plant.backend.enabled, held)
        self.assertTrue(plant.app.tracking)

    def test_normalized_last_seen_goal_stays_bounded_at_zenith_and_limit(self):
        for theta, target in ((90, (8, -6)), (174, (0, -8)), (135, (8, -6))):
            with self.subTest(theta=theta):
                plant = self.make_plant(theta=theta, target=target, normalized=True)
                plant.advance(.4)
                visible_pose = {axis: physical_coordinate(axis, plant.backend.enabled[axis])
                                for axis in ("pan", "tilt")}
                plant.visible = False
                start = len(plant.actuator_samples)
                plant.advance(15)
                self.assertTrue(plant.app.tracking)
                self.assertEqual(plant.app.snapshot()["tracking_memory"]["phase"], "waiting")
                for axis in ("pan", "tilt"):
                    position = physical_coordinate(axis, plant.backend.enabled[axis])
                    self.assertLessEqual(abs(position - visible_pose[axis]), 30.2)
                self.assert_motion_rate_bounded(plant.actuator_samples[start:])
                held = dict(plant.backend.enabled)
                plant.advance(15)
                self.assertEqual(plant.backend.enabled, held)


if __name__ == "__main__":
    unittest.main(verbosity=2)
