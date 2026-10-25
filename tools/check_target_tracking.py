"""Pure unit tests: association, stale-frame freeze and calibrated ray geometry.

No camera, FPGA, socket, model runtime, GUI, HDL compiler or simulator is used.
Run with the workspace Python: python tools/check_target_tracking.py
"""

from dataclasses import replace
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
from target_tracking import CameraCalibration, TargetTracker, TrackerConfig


def box(cx=320, cy=240, width=80, height=80, score=.8, kind="dog"):
    return dict(x1=cx - width / 2, y1=cy - height / 2,
                x2=cx + width / 2, y2=cy + height / 2, score=score, kind=kind)


def closed_loop_demo(config, moving=False, latency=.16, jitter_px=3.0):
    """Independent, causal camera/servo plant; physical parameters are fixtures.

    The controller is given NO intrinsics, servo span or motion feedback. This
    test plant alone projects target bearings through a400px-focal camera,
    delays detections, and moves each servo with80ms lag and0.18degree deadzone.
    Only one observed image can generate one command. These are synthetic test
    parameters, not claimed measurements of either installed servo or camera.
    """
    rng = random.Random(20261025)
    tracker = TargetTracker(CameraCalibration(640, 480), config)
    actual = [0.0, 0.0]
    command = [0.0, 0.0]
    queue, errors, rates = [], [], []
    capture_next, sequence, command_count, tracked = 0.0, 0, 0, 0
    dt = .005
    for step in range(2401):
        now = step * dt
        world = [14.0 + (4.0 * math.sin(now * 1.0) if moving else 0.0),
                 8.0 + (3.0 * math.sin(now * .8) if moving else 0.0)]
        for axis in range(2):
            difference = command[axis] - actual[axis]
            if abs(difference) > .18:
                actual[axis] += max(-90.0, min(90.0, difference / .08)) * dt
        if now >= capture_next:
            ex, ey = world[0] - actual[0], world[1] - actual[1]
            x = 320 - 400 * math.tan(math.radians(ex)) + rng.uniform(-jitter_px, jitter_px)
            y = 240 - 400 * math.tan(math.radians(ey)) + rng.uniform(-jitter_px, jitter_px)
            detection = box(x, y, 140, 130)
            if sequence == 0:
                tracker.select_target(detection, now, sequence)
            queue.append((now + latency + rng.uniform(-.025, .025), now, sequence, detection))
            queue.sort(key=lambda item: item[0])
            capture_next = now + .14 + rng.uniform(-.025, .025)
            sequence += 1
        while queue and queue[0][0] <= now:
            _, capture, seq, detection = queue.pop(0)
            result = tracker.update([detection], capture, now, seq)
            if result['status'] == 'tracking':
                command[0] += result['delta_yaw_deg']
                command[1] += result['delta_pitch_deg']
                rates.append((result['yaw_rate_dps'], result['pitch_rate_dps']))
                command_count += 1
                tracked += 1
                # A UI refresh of this exact image is not another control tick.
                duplicate = tracker.update([detection], capture, now, seq)
                assert duplicate['delta_yaw_deg'] == duplicate['delta_pitch_deg'] == 0
            else:
                assert result['delta_yaw_deg'] == result['delta_pitch_deg'] == 0
        if now > 8:
            errors.append((world[0] - actual[0], world[1] - actual[1]))
    rms = tuple(math.sqrt(sum(e[axis] ** 2 for e in errors) / len(errors)) for axis in (0, 1))
    return dict(rms_deg=rms, final_command=command, captures=sequence,
                command_count=command_count, tracked=tracked, rates=rates)


class TargetTrackingTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration(640, 480, fx=400, fy=400, cx=320, cy=240)

    def tracker(self, selected=None, camera=None, config=None):
        tracker = TargetTracker(camera or self.camera, config)
        tracker.select_target(selected or box(), 1.0, frame_seq=10)
        return tracker

    def assertFrozen(self, result):
        for name in ("yaw_rate_dps", "pitch_rate_dps", "delta_yaw_deg", "delta_pitch_deg"):
            self.assertEqual(result[name], 0, name)
        self.assertIsNone(result["target"])
        self.assertFalse(result["bearing_valid"])
        self.assertFalse(result["body_direction_valid"])
        self.assertFalse(result["world_direction_valid"])

    def assertNoPosition(self, result):
        self.assertIsNone(result["distance_m"])
        self.assertIsNone(result["translation_m"])
        self.assertFalse(result["position_valid"])

    def test_explicit_selection_is_required_and_itself_never_moves(self):
        tracker = TargetTracker(self.camera)
        result = tracker.update([box()], 1, 1, frame_seq=10)
        self.assertEqual(result["status"], "unselected")
        self.assertFrozen(result)
        self.assertEqual(tracker.select_target(box(), 1, 10)["status"], "selected")
        self.assertFrozen(tracker.update([box()], 1, 1, 10))
        tracker.clear_target()
        self.assertEqual(tracker.update([box()], 1.1, 1.1, 11)["status"], "unselected")

    def test_selected_target_kept_when_another_has_higher_confidence(self):
        tracker = self.tracker(box(180, 200))
        other = box(480, 240, 140, 140, .99)
        target = box(185, 202, score=.60)
        result = tracker.update([other, box(185, 202, kind="human"), target], 1.1, 1.1, 11)
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(result["target"], target)
        self.assertEqual(result["candidate_count"], 1)

    def test_human_and_dog_locks_do_not_cross_class(self):
        tracker = self.tracker(box(kind="human"))
        result = tracker.update([box(kind="dog")], 1.1, 1.1, 11)
        self.assertEqual(result["status"], "searching")
        self.assertEqual(result["kind"], "human")
        self.assertFrozen(result)

    def test_even_zero_ambiguity_margin_does_not_choose_an_exact_tie_by_list_order(self):
        config = replace(TrackerConfig(), ambiguity_margin=0)
        for candidates in ([box(315), box(325)], [box(325), box(315)]):
            result = self.tracker(config=config).update(candidates, 1.1, 1.1, 11)
            self.assertEqual(result["status"], "ambiguous")
            self.assertFrozen(result)

    def test_crossing_targets_with_ambiguous_geometry_freeze(self):
        tracker = self.tracker()
        result = tracker.update([box(315), box(325)], 1.1, 1.1, 11)
        self.assertEqual(result["status"], "ambiguous")
        self.assertEqual(result["candidate_count"], 2)
        self.assertFrozen(result)
        self.assertEqual(result["last_target"], box())
        # Ambiguous observations did not move the association anchor.
        for index in (1, 2):
            result = tracker.update([box(320)], 1.1 + index * .1, 1.1 + index * .1, 11 + index)
            self.assertEqual(result["status"], "reacquiring")
            self.assertFrozen(result)
        result = tracker.update([box(320)], 1.4, 1.4, 14)
        self.assertEqual(result["status"], "tracking")

    def test_separate_overlap_center_and_size_gates(self):
        cases = [
            (TrackerConfig(), box(410)),  # no overlap, despite reasonable size
            (replace(TrackerConfig(), min_iou=0, max_center_shift=.1), box(340)),
            (TrackerConfig(), box(width=200, height=200)),  # abrupt scale jump
            (TrackerConfig(), box(width=80, height=20)),  # abrupt aspect change
        ]
        for config, candidate in cases:
            with self.subTest(config=config, candidate=candidate):
                result = self.tracker(config=config).update([candidate], 1.1, 1.1, 11)
                self.assertIn(result["status"], ("searching", "reacquiring"))
                self.assertFrozen(result)

    def test_missing_preserves_lock_and_reappearance_automatically_confirms(self):
        tracker = self.tracker(box(180, 200))
        result = tracker.update([], 1.1, 1.1, 11)
        self.assertEqual(result["status"], "searching")
        self.assertFrozen(result)
        for timestamp, sequence, count in ((5, 12, 1), (5.1, 13, 2)):
            result = tracker.update([box(180, 200)], timestamp, timestamp, sequence)
            self.assertEqual(result["status"], "reacquiring")
            self.assertEqual(result["last_target"], box(180, 200))
            self.assertEqual(result["reacquire_count"], count)
            self.assertFrozen(result)
        result = tracker.update([box(181, 200)], 5.2, 5.2, 14)
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(result["reacquire_count"], 3)
        self.assertLessEqual(result["control_dt_s"], .1)

    def test_old_capture_timestamp_not_inference_finish_time_controls_age(self):
        tracker = self.tracker(box(180, 200))
        result = tracker.update([box(180, 200)], 1.1, 1.6, 11)
        self.assertEqual(result["status"], "stale_frame")
        self.assertAlmostEqual(result["frame_age_s"], .5)
        self.assertFrozen(result)
        # A rejected stale observation never updates the lock's last-match time.
        recovered = tracker.update([box(180, 200)], 1.8, 1.8, 12)
        self.assertEqual(recovered["status"], "reacquiring")
        self.assertEqual(recovered["reacquire_count"], 1)

    def test_long_gap_reentry_elsewhere_uses_three_fresh_frames_and_no_gap_integral(self):
        config = replace(TrackerConfig(), max_control_dt_s=.2, filter_time_constant_s=.5)
        original, returning = box(160, 120), box(490, 340)
        tracker = self.tracker(original, config=config)
        moving = tracker.update([original], 1.1, 1.1, 11)
        self.assertGreater(moving['yaw_rate_dps'], 0)
        for index in (1, 2):
            timestamp = 60 + index * .05
            result = tracker.update([returning], timestamp, timestamp, 20 + index)
            self.assertEqual(result['status'], 'reacquiring')
            self.assertEqual(result['last_target'], original)
            self.assertEqual(result['reacquire_count'], index)
            self.assertEqual(result['reacquire_required'], 3)
            self.assertFrozen(result)
        result = tracker.update([returning], 60.15, 60.15, 23)
        self.assertEqual(result['status'], 'tracking')
        self.assertEqual(result['target'], returning)
        self.assertAlmostEqual(result['control_dt_s'], .05)
        self.assertEqual(result['filter_dt_s'], 0)
        self.assertLess(result['yaw_rate_dps'], 0)
        self.assertLess(result['pitch_rate_dps'], 0)
        self.assertLessEqual(abs(result['delta_yaw_deg']), config.max_rate_dps * .05 + 1e-8)
        normal = tracker.update([box(488, 338)], 60.20, 60.20, 24)
        self.assertEqual(normal['status'], 'tracking')
        self.assertEqual(normal['reacquire_count'], 0)

    def test_long_absence_retains_selected_class_and_anchor_without_expiration(self):
        original = box(180, 200, kind='human')
        tracker = self.tracker(original)
        for sequence, timestamp in enumerate((1.1, 2, 10, 60, 600), 11):
            result = tracker.update([], timestamp, timestamp, sequence)
            self.assertEqual(result['status'], 'searching')
            self.assertEqual(result['last_target'], original)
            self.assertEqual(result['kind'], 'human')
            self.assertEqual(result['reacquire_count'], 0)
            self.assertFrozen(result)

    def test_reacquisition_does_not_cross_class_or_accept_implausible_size(self):
        tracker = self.tracker(box(180, 200, kind='human'))
        for sequence, timestamp in enumerate((2, 2.1, 2.2, 2.3), 11):
            result = tracker.update([box(480, 240, kind='dog'),
                                     box(480, 240, width=8, height=8, kind='human')],
                                    timestamp, timestamp, sequence)
            self.assertEqual(result['status'], 'searching')
            self.assertEqual(result['reacquire_count'], 0)
            self.assertFrozen(result)

    def test_search_multiple_same_class_candidates_wait_regardless_of_score_or_order(self):
        for reverse in (False, True):
            tracker = self.tracker(box(180, 200))
            one = box(185, 202, score=.99)
            two = box(480, 340, score=.45)
            candidates = [two, one] if reverse else [one, two]
            for sequence in range(11, 15):
                timestamp = 2 + sequence * .1
                result = tracker.update(candidates, timestamp, timestamp, sequence)
                self.assertEqual(result['status'], 'ambiguous')
                self.assertEqual(result['candidate_count'], 2)
                self.assertEqual(result['reacquire_count'], 0)
                self.assertEqual(result['last_target'], box(180, 200))
                self.assertFrozen(result)

    def test_reacquisition_missing_ambiguity_jump_and_long_gap_reset_confirmation(self):
        for interruption in ('missing', 'ambiguity', 'jump', 'gap'):
            with self.subTest(interruption=interruption):
                tracker = self.tracker(box(180, 200))
                returning = box(480, 340)
                self.assertEqual(tracker.update([returning], 2, 2, 11)['reacquire_count'], 1)
                self.assertEqual(tracker.update([returning], 2.1, 2.1, 12)['reacquire_count'], 2)
                candidates = ({'missing': [], 'ambiguity': [returning, box(180, 200)],
                               'jump': [box(160, 120)], 'gap': [returning]})[interruption]
                timestamp = 5 if interruption == 'gap' else 2.2
                interrupted = tracker.update(candidates, timestamp, timestamp, 13)
                self.assertFrozen(interrupted)
                expected = 1 if interruption in ('jump', 'gap') else 0
                self.assertEqual(interrupted['reacquire_count'], expected)
                candidate = candidates[0] if interruption in ('jump', 'gap') else returning
                required = 3 - expected
                for index in range(1, required + 1):
                    at = timestamp + index * .1
                    result = tracker.update([candidate], at, at, 13 + index)
                    self.assertEqual(result['status'], 'tracking' if index == required else 'reacquiring')
                    if index < required:
                        self.assertFrozen(result)

    def test_rejected_frames_do_not_advance_reacquisition_or_move_anchor(self):
        original, returning = box(180, 200), box(480, 340)
        tracker = self.tracker(original)
        self.assertEqual(tracker.update([returning], 2, 2, 11)['reacquire_count'], 1)
        rejected = ((2, 2, 11, 'awaiting_new_frame'),
                    (1.9, 2, 12, 'out_of_order'),
                    (2.01, 2.01, 10, 'out_of_order'),
                    (2.01, 2.01, True, 'invalid_sequence'),
                    (3, 2.01, 12, 'future_frame'),
                    (float('nan'), 2.01, 12, 'invalid_time'),
                    (2.02, 1.5, 12, 'clock_reversed'))
        for frame_time, now, sequence, status in rejected:
            result = tracker.update([returning], frame_time, now, sequence)
            self.assertEqual(result['status'], status)
            self.assertEqual(result['reacquire_count'], 1)
            self.assertEqual(result['last_target'], original)
            self.assertFrozen(result)
        self.assertEqual(tracker.update([returning], 2.1, 2.1, 12)['reacquire_count'], 2)
        stale = tracker.update([returning], 2.2, 2.7, 13)
        self.assertEqual(stale['status'], 'stale_frame')
        self.assertEqual(stale['reacquire_count'], 0)
        self.assertFrozen(stale)
        for index in (1, 2, 3):
            at = 2.7 + index * .1
            result = tracker.update([returning], at, at, 13 + index)
            self.assertEqual(result['status'], 'tracking' if index == 3 else 'reacquiring')
        self.assertEqual(result['target'], returning)

    def test_clear_and_explicit_reselection_discard_pending_reacquisition(self):
        tracker = self.tracker(box(180, 200))
        self.assertEqual(tracker.update([box(480, 340)], 2, 2, 11)['reacquire_count'], 1)
        tracker.clear_target()
        cleared = tracker.update([box(480, 340)], 2.1, 2.1, 12)
        self.assertEqual(cleared['status'], 'unselected')
        self.assertIsNone(cleared['last_target'])
        self.assertEqual(cleared['reacquire_count'], 0)
        self.assertFrozen(cleared)
        selected = box(200, 180, kind='human')
        tracker.select_target(selected, 2.1, 12)
        result = tracker.update([selected, box(480, 340)], 2.2, 2.2, 13)
        self.assertEqual(result['status'], 'tracking')
        self.assertEqual(result['target'], selected)
        self.assertEqual(result['reacquire_count'], 0)

    def test_duplicate_and_out_of_order_frames_never_repeat_commands(self):
        tracker = self.tracker(box(180, 200))
        self.assertGreater(tracker.update([box(180, 200)], 1.1, 1.1, 11)["yaw_rate_dps"], 0)
        for timestamp, sequence, status in ((1.1, 11, "awaiting_new_frame"),
                                            (1.05, 12, "out_of_order"),
                                            (1.2, 10, "out_of_order")):
            result = tracker.update([box(180, 200)], timestamp, 1.2, sequence)
            self.assertEqual(result["status"], status)
            self.assertFrozen(result)
        # An invalid sequence did not poison the accepted-time watermark.
        self.assertEqual(tracker.update([box(180, 200)], 1.15, 1.21, 12)["status"], "tracking")

    def test_future_or_invalid_time_and_backwards_clock_freeze(self):
        tracker = self.tracker()
        result = tracker.update([box()], 1.3, 1.1, 11)
        self.assertEqual(result["status"], "future_frame")
        self.assertFrozen(result)
        for timestamp, now in ((float("nan"), 1.2), (1.2, float("inf")), (-1, 1.2)):
            result = tracker.update([box()], timestamp, now, 11)
            self.assertEqual(result["status"], "invalid_time")
            self.assertFrozen(result)
        result = tracker.update([box()], 1.05, 1.05, 11)
        self.assertEqual(result["status"], "clock_reversed")
        self.assertFrozen(result)

    def test_sequence_numbers_optional_but_timestamp_still_monotonic(self):
        tracker = TargetTracker(self.camera)
        tracker.select_target(box(), 0)
        self.assertEqual(tracker.update([box()], .1, .1)["status"], "tracking")
        result = tracker.update([box()], .1, .2)
        self.assertEqual(result["status"], "awaiting_new_frame")
        self.assertFrozen(result)

    def test_tiny_timestamp_tolerance_does_not_create_negative_reported_age(self):
        result = self.tracker().update([box()], 1.11, 1.10, 11)
        self.assertEqual(result["status"], "tracking")
        self.assertEqual(result["frame_age_s"], 0)

    def test_deadband_centers_and_left_up_right_down_signs(self):
        for cx, cy, yaw_sign, pitch_sign in ((320, 240, 0, 0), (310, 235, 0, 0),
                                             (200, 150, 1, 1), (440, 330, -1, -1)):
            with self.subTest(cx=cx, cy=cy):
                selected = box(cx, cy)
                result = self.tracker(selected).update([selected], 1.1, 1.1, 11)
                for rate, sign in ((result["yaw_rate_dps"], yaw_sign),
                                   (result["pitch_rate_dps"], pitch_sign)):
                    self.assertEqual(0 if abs(rate) < 1e-9 else int(math.copysign(1, rate)), sign)

    def test_rate_and_single_update_displacement_limits(self):
        config = replace(TrackerConfig(), gain=100, max_rate_dps=2, max_control_dt_s=.05)
        selected = box(100, 100)
        result = self.tracker(selected, config=config).update([selected], 1.6, 1.6, 11)
        self.assertEqual(result["yaw_rate_dps"], 2)
        self.assertEqual(result["pitch_rate_dps"], 2)
        self.assertEqual(result["control_dt_s"], .05)
        self.assertAlmostEqual(result["delta_yaw_deg"], .1)
        self.assertAlmostEqual(result["delta_pitch_deg"], .1)

    def test_calibrated_principal_point_not_assumed_image_center(self):
        camera = CameraCalibration(640, 480, fx=400, fy=500, cx=300, cy=220)
        selected = box(300, 220)
        result = self.tracker(selected, camera).update([selected], 1.1, 1.1, 11)
        self.assertAlmostEqual(result["yaw_error_deg"], 0)
        self.assertAlmostEqual(result["pitch_error_deg"], 0)
        self.assertEqual(result["camera_ray"], (1, 0, 0))

    def test_intrinsics_override_fov_and_pinhole_angles_are_geometric(self):
        camera = CameraCalibration(640, 480, fx=400, fy=400, cx=320, cy=240,
                                   hfov_deg=20, vfov_deg=20)
        selected = box(120, 40, 20, 20)
        result = self.tracker(selected, camera).update([selected], 1.1, 1.1, 11)
        self.assertEqual(result["camera_model"], "intrinsics")
        self.assertFalse(result["approximate"])
        self.assertAlmostEqual(result["yaw_error_deg"], math.degrees(math.atan(.5)))
        self.assertAlmostEqual(result["pitch_error_deg"], math.degrees(math.atan(.5 / math.sqrt(1.25))))
        self.assertAlmostEqual(sum(v * v for v in result["camera_ray"]), 1)
        self.assertNoPosition(result)

    def test_explicit_fov_is_approximate_and_unknown_fov_never_invents_bearing(self):
        selected = box(160, 120)
        camera = CameraCalibration(640, 480, hfov_deg=90, vfov_deg=60)
        result = self.tracker(selected, camera).update([selected], 1.1, 1.1, 11)
        self.assertTrue(result["bearing_valid"])
        self.assertTrue(result["approximate"])
        self.assertAlmostEqual(result["yaw_error_deg"], math.degrees(math.atan(.5)))
        self.assertNoPosition(result)
        result = self.tracker(selected, CameraCalibration(640, 480)).update(
            [selected], 1.1, 1.1, 11, pan_deg=30, tilt_deg=20, gimbal_calibrated=True)
        self.assertEqual(result["control_basis"], "normalized")
        self.assertGreater(result["yaw_rate_dps"], 0)
        self.assertGreater(result["pitch_rate_dps"], 0)
        self.assertIsNone(result["yaw_error_deg"])
        self.assertIsNone(result["pitch_error_deg"])
        self.assertIsNone(result["camera_ray"])
        self.assertIsNone(result["body_direction"])
        self.assertNoPosition(result)

    def test_body_extrinsics_require_both_measured_axis_mappings(self):
        for pan, tilt, calibrated in ((None, 10, True), (10, None, True),
                                      (10, 10, False), (float("nan"), 10, True), (10, 10, 1)):
            result = self.tracker().update([box()], 1.1, 1.1, 11, pan, tilt, calibrated, 90)
            self.assertFalse(result["body_direction_valid"])
            self.assertFalse(result["world_direction_valid"])
            self.assertIsNone(result["body_direction"])
            self.assertNoPosition(result)

    def test_center_ray_rotates_pan_left_and_tilt_up_then_optional_world_yaw(self):
        result = self.tracker().update([box()], 1.1, 1.1, 11, 90, 30, True, 90)
        expected_body = (0, math.sqrt(3) / 2, .5)
        expected_world = (-math.sqrt(3) / 2, 0, .5)
        for actual, expected in zip(result["body_direction"], expected_body):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(result["world_direction"], expected_world):
            self.assertAlmostEqual(actual, expected)
        self.assertTrue(result["body_direction_approximate"])
        self.assertEqual(result["gimbal_angle_source"], "calibrated_command_estimate")
        self.assertNoPosition(result)

    def test_off_axis_ray_includes_pitch_before_pan(self):
        selected = box(120, 40, 20, 20)
        result = self.tracker(selected).update([selected], 1.1, 1.1, 11, 90, 30, True)
        norm = math.sqrt(1.5)
        expected = (-.5 / norm, (math.sqrt(3) / 2 - .25) / norm,
                    (.5 + math.sqrt(3) / 4) / norm)
        for actual, reference in zip(result["body_direction"], expected):
            self.assertAlmostEqual(actual, reference)
        self.assertIsNone(result["world_direction"])
        self.assertNoPosition(result)

    def test_seeded_geometry_directions_and_limits_are_finite_and_unit_length(self):
        rng = random.Random(20261024)
        for _ in range(500):
            selected = box(rng.uniform(20, 620), rng.uniform(20, 460), 20, 20,
                           kind=rng.choice(("dog", "human")))
            result = self.tracker(selected).update([selected], 1.1, 1.1, 11,
                rng.uniform(-180, 180), rng.uniform(-90, 90), True, rng.uniform(-180, 180))
            for name in ("camera_ray", "body_direction", "world_direction"):
                ray = result[name]
                self.assertTrue(all(math.isfinite(v) for v in ray))
                self.assertAlmostEqual(sum(v * v for v in ray), 1, places=12)
            self.assertLessEqual(abs(result["yaw_rate_dps"]), 25)
            self.assertLessEqual(abs(result["pitch_rate_dps"]), 25)
            self.assertNoPosition(result)

    def test_invalid_detections_do_not_refresh_or_switch_lock(self):
        malformed = [{}, box(score=float("nan")), box(score=.1), box(score=2),
                     box(kind="cat"), dict(box(), x2=float("inf")),
                     dict(box(), x2=100), box(2000, 2000)]
        result = self.tracker().update(malformed, 1.1, 1.1, 11)
        self.assertEqual(result["status"], "searching")
        self.assertFrozen(result)
        for candidate in malformed:
            with self.assertRaises(ValueError):
                self.tracker().select_target(candidate, 1)

    def test_capture_time_filter_is_independent_of_inference_completion_jitter(self):
        config = replace(TrackerConfig(), filter_time_constant_s=.15,
                         deadband=.01, deadband_hysteresis=0, min_rate_dps=0)
        cameras = CameraCalibration(640, 480)
        trackers = [self.tracker(box(240, width=140), cameras, config) for _ in range(2)]
        for i, nx in enumerate((.24, .31, .23, .29, .25, .30), 1):
            capture = 1 + i * .1
            frame = [box(320 - nx * 320, width=140)]
            fixed = trackers[0].update(frame, capture, capture + .10, 10 + i)
            delayed = trackers[1].update(frame, capture, capture + (.10, .13, .17, .12)[i % 4], 10 + i)
            self.assertEqual(fixed['status'], 'tracking')
            self.assertEqual(fixed['yaw_error_filtered_normalized'], delayed['yaw_error_filtered_normalized'])
            self.assertEqual(fixed['delta_yaw_deg'], delayed['delta_yaw_deg'])
            self.assertAlmostEqual(fixed['yaw_error_normalized'], nx)

    def test_filter_reduces_box_jitter_without_changing_raw_camera_bearing(self):
        config = replace(TrackerConfig(), filter_time_constant_s=.15,
                         deadband=.01, deadband_hysteresis=0, min_rate_dps=0,
                         max_rate_dps=100)
        tracker = self.tracker(box(240, width=140), config=config)
        raw, filtered = [], []
        for i in range(1, 41):
            nx = .25 + (.035 if i % 2 else -.035)
            result = tracker.update([box(320 - nx * 320, width=140)], 1 + i * .1, 1 + i * .1, 10 + i)
            self.assertEqual(result['status'], 'tracking')
            self.assertAlmostEqual(result['yaw_error_deg'], math.degrees(math.atan(nx * .8)))
            if i > 10:
                raw.append(nx)
                filtered.append(result['yaw_error_filtered_normalized'])
        raw_rms = math.sqrt(sum((v - .25) ** 2 for v in raw) / len(raw))
        filtered_rms = math.sqrt(sum((v - .25) ** 2 for v in filtered) / len(filtered))
        self.assertLess(filtered_rms, raw_rms * .5)

    def test_deadband_hysteresis_and_minimum_rate_do_not_hunt_at_center(self):
        config = replace(TrackerConfig(), filter_time_constant_s=0,
                         normalized_gain_dps=10, deadband=.04,
                         deadband_hysteresis=.02, min_rate_dps=3, max_rate_dps=20)
        tracker = self.tracker(box(304, width=140), CameraCalibration(640, 480), config)
        observations = (.05, .059, .061, .055, .045, .039, .055, .065)
        for i, (nx, active) in enumerate(zip(observations, (False, False, True, True, True, False, False, True)), 1):
            result = tracker.update([box(320 - nx * 320, width=140)], 1 + i * .1, 1 + i * .1, 10 + i)
            self.assertEqual(result['yaw_control_active'], active)
            self.assertEqual(result['yaw_rate_dps'], 3.0 if active else 0.0)
        centered = tracker.update([box(width=140)], 1.9, 1.9, 19)
        self.assertEqual(centered['yaw_rate_dps'], 0)
        self.assertFalse(centered['yaw_control_active'])

    def test_minimum_rate_never_overrides_zero_gain_or_maximum_rate(self):
        selected = box(220, 160)
        for options in (dict(gain=0), dict(max_rate_dps=0), dict(min_rate_dps=50, max_rate_dps=2)):
            config = replace(TrackerConfig(), **options)
            result = self.tracker(selected, config=config).update([selected], 1.1, 1.1, 11)
            expected = 2 if options.get('max_rate_dps') == 2 else 0
            self.assertEqual(result['yaw_rate_dps'], expected)
            self.assertEqual(result['pitch_rate_dps'], expected)

    def test_missing_or_ambiguous_frame_clears_filter_and_active_state(self):
        config = replace(TrackerConfig(), filter_time_constant_s=.5)
        for missing in ([], [box(235, width=140), box(245, width=140)]):
            tracker = self.tracker(box(240, width=140), CameraCalibration(640, 480), config)
            moving = tracker.update([box(240, width=140)], 1.1, 1.1, 11)
            self.assertGreater(moving['yaw_rate_dps'], 0)
            self.assertFrozen(tracker.update(missing, 1.2, 1.2, 12))
            for index in (1, 2):
                pending = tracker.update([box(256, width=140)], 1.2 + index * .1, 1.2 + index * .1, 12 + index)
                self.assertEqual(pending['status'], 'reacquiring')
                self.assertFrozen(pending)
            recovered = tracker.update([box(256, width=140)], 1.5, 1.5, 15)
            self.assertEqual(recovered['status'], 'tracking')
            self.assertAlmostEqual(recovered['yaw_error_filtered_normalized'], .20)
            self.assertEqual(recovered['filter_dt_s'], 0)

    def test_filter_tail_cannot_push_opposite_a_clear_new_error(self):
        config = replace(TrackerConfig(), filter_time_constant_s=.5)
        tracker = self.tracker(box(288, width=180), CameraCalibration(640, 480), config)
        first = tracker.update([box(288, width=180)], 1.1, 1.1, 11)
        self.assertGreater(first['yaw_rate_dps'], 0)
        other_side = tracker.update([box(352, width=180)], 1.2, 1.2, 12)
        self.assertLess(other_side['yaw_rate_dps'], 0)

    def test_causal_static_and_moving_demo_with_delayed_jittery_detections(self):
        # Candidate demo settings, not a measured tuning of the user's servos.
        config = replace(TrackerConfig(), normalized_gain_dps=65,
                         max_rate_dps=30, max_control_dt_s=.20,
                         filter_time_constant_s=.06, min_rate_dps=3)
        for latency in (.12, .16, .20):
            static = closed_loop_demo(config, latency=latency)
            moving = closed_loop_demo(config, moving=True, latency=latency)
            self.assertLess(static['rms_deg'][0], 3.0)
            self.assertLess(static['rms_deg'][1], 2.5)
            self.assertLess(moving['rms_deg'][0], 5.0)
            self.assertLess(moving['rms_deg'][1], 4.0)
            self.assertGreater(static['tracked'], 70)
            self.assertLess(static['command_count'], static['captures'])
            self.assertTrue(all(abs(rate) <= 30 for pair in static['rates'] + moving['rates'] for rate in pair))
            print('Synthetic camera/servo fixture:', latency, 's latency, static/moving RMS degrees',
                  static['rms_deg'], moving['rms_deg'])

    def test_invalid_calibration_and_settings_fail_early(self):
        for values in (dict(width=0, height=480), dict(width=640, height=480, fx=400),
                       dict(width=640, height=480, hfov_deg=90),
                       dict(width=640, height=480, hfov_deg=180, vfov_deg=60),
                       dict(width=640, height=480, fx=-1, fy=400, cx=320, cy=240)):
            with self.assertRaises(ValueError):
                CameraCalibration(**values)
        for values in (dict(deadband=-1), dict(gain=-1), dict(loss_gap_s=0),
                       dict(max_center_shift=0), dict(max_size_ratio=.5), dict(gain=float("inf")),
                       dict(filter_time_constant_s=-.01), dict(min_rate_dps=-1),
                       dict(deadband_hysteresis=-.01), dict(deadband_hysteresis=1)):
            with self.assertRaises(ValueError):
                TrackerConfig(**values)


if __name__ == "__main__":
    unittest.main(verbosity=2)
