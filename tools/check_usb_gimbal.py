"""USB/gimbal integration tests using standard-library mocks and loopback HTTP.

No real camera, cv2/numpy package, PWM/sysfs path, FPGA or external network is
used. Fake VisionFrames test the existing capture-time/control boundary. By
default HTTP bytes drive the real handler through an in-memory connection.
Optional --loopback instead binds an ephemeral 127.0.0.1 port and closes every
worker/socket; managed sandboxes may prohibit even this local connection.
"""

from copy import deepcopy
import http.client
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc"))
import usb_gimbal
from gimbal_control import MockPWMBackend, default_config
from usb_camera import CameraFrame, UsbCamera
from usb_gimbal import GimbalApp, VisionFrame, VisionWorker, make_handler

LOOPBACK = "--loopback" in sys.argv
if LOOPBACK:
    sys.argv.remove("--loopback")


def detection(cx=180, cy=180, kind="dog"):
    return dict(x1=cx - 40, y1=cy - 40, x2=cx + 40, y2=cy + 40, score=.9, kind=kind)


def measured_test_config():
    """Synthetic calibration fixture, never a recommendation for real mechanics."""
    result = default_config()
    result['mount'] = dict(upright_tilt_deg=0)  # Explicit reference mounting for legacy numeric fixtures.
    for name in ("pan", "tilt"):
        result["axes"][name].update(safe_min_us=1000, safe_max_us=2000,
            calibration_points=[dict(angle_deg=-90, pulse_us=1000),
                                dict(angle_deg=0, pulse_us=1500),
                                dict(angle_deg=90, pulse_us=2000)],
            calibration_confirmed=True)
    result["camera"] = dict(width=640, height=480, fx=400, fy=400, cx=320, cy=240)
    return result


SCREENSHOT_ANGLE_POINTS = {
    'pan': [dict(pulse_us=380, angle_deg=45), dict(pulse_us=1882, angle_deg=180),
            dict(pulse_us=2550, angle_deg=240)],
    'tilt': [dict(pulse_us=2420, angle_deg=10), dict(pulse_us=2043, angle_deg=45),
             dict(pulse_us=645, angle_deg=175)],
}


def screenshot_test_config(*, calibrated=False):
    """The user's measured pulse/angle table, not an assumed servo travel."""
    result = default_config()
    result['mount'] = dict(upright_tilt_deg=0)
    result['axes']['pan'].update(explore_min_us=340, explore_max_us=2600,
                                 safe_min_us=340, safe_max_us=2600)
    result['axes']['tilt'].update(safe_min_us=645, safe_max_us=2420)
    if calibrated:
        for name in ('pan', 'tilt'):
            result['axes'][name].update(calibration_points=deepcopy(SCREENSHOT_ANGLE_POINTS[name]),
                                        calibration_confirmed=True)
    # A camera intrinsic calibration is optional for normalized image control.
    result['camera'] = dict(width=640, height=480)
    return result


class AppFixture(unittest.TestCase):
    def setUp(self):
        # The managed Windows sandbox may redirect its default temp location
        # outside this workspace's writable ACL. Keep test fixtures underneath
        # tools and verify the cleanup target before recursive temp cleanup.
        test_root = Path(__file__).resolve().parent
        self.temporary = tempfile.TemporaryDirectory(prefix="usb-gimbal-test-", dir=test_root)
        assert Path(self.temporary.name).resolve().parent == test_root
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "config.json"
        self.config = measured_test_config()
        self.path.write_text(json.dumps(self.config), encoding="utf-8")
        self.now = 1000.0
        # Replace this module's time namespace, not the process-wide threading
        # clock. Real Event.wait/server shutdown therefore continue to work.
        clock = SimpleNamespace(monotonic=lambda: self.now, perf_counter=time.perf_counter)
        self.clock_patch = mock.patch.object(usb_gimbal, "time", clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.backend = MockPWMBackend()
        self.app = GimbalApp(self.path, backend=self.backend,
                             dataset=Path(self.temporary.name) / "photos")
        self.app.gimbal._clock = lambda: self.now
        self.addCleanup(self.app.close)

    def frame(self, sequence=10, boxes=None, pose=None, arrived=None):
        return VisionFrame(sequence=sequence, arrived_at=self.now if arrived is None else arrived,
            image=object(), jpeg=b"fake-jpeg", detections=[detection()] if boxes is None else boxes,
            gimbal_at_arrival=deepcopy(self.app.gimbal.snapshot() if pose is None else pose),
            width=640, height=480, inference_ms=12.5, coordinates="original USB pixels")

    def publish(self, sequence=10, **kwargs):
        frame = self.frame(sequence, **kwargs)
        self.app.publish(frame)
        return frame

    def arm(self, manual=False):
        for name in ("pan", "tilt"):
            self.app.command(dict(action="arm", axis=name, start_us=1500, manual=manual))

    def begin_tracking(self, boxes=None):
        self.arm()
        self.publish(boxes=boxes)
        self.app.command(dict(action="select", sequence=10, index=0))
        self.app.command(dict(action="track", enabled=True))

    def use_screenshot_axes(self, *, calibrated=False, enabled=True):
        self.app.command(dict(action='save_config', config=screenshot_test_config(calibrated=calibrated)))
        self.app.gimbal._clock = lambda: self.now
        if enabled:
            for name in ('pan', 'tilt'):
                self.app.command(dict(action='manual_enable', axis=name, pulse_us=1500))

    def track_screenshot_axes(self):
        self.use_screenshot_axes(calibrated=True)
        self.publish(10)
        self.app.command(dict(action='select', sequence=10, index=0))
        self.app.command(dict(action='track', enabled=True))
        # Queue different targets without ticking PWM. These requests must
        # survive rejected edits, even though actual output is still1500us.
        self.app.gimbal.set_angle('pan', 190, tracking=True)
        self.app.gimbal.set_angle('tilt', 50, tracking=True)

    def assertHeld(self):
        for axis in self.app.gimbal.snapshot()["axes"].values():
            self.assertEqual(axis["target_us"], axis["output_us"])


class GimbalIntegrationTests(AppFixture):
    def test_save_user_angle_tables_one_axis_at_a_time_preserves_enabled_pwm_and_targets(self):
        self.use_screenshot_axes()
        events = list(self.backend.events)
        for axis, other in (('pan', 'tilt'), ('tilt', 'pan')):
            other_before = deepcopy(self.app.config['axes'][other])
            state_before = self.app.gimbal.snapshot()
            response = self.app.command(dict(action='save_angles', axis=axis,
                calibration_points=deepcopy(SCREENSHOT_ANGLE_POINTS[axis])))
            self.assertEqual(response['axis'], axis)
            self.assertEqual(self.app.config['axes'][other], other_before)
            self.assertEqual(self.backend.events, events)
            self.assertEqual(self.backend.enabled, {'pan': 1500, 'tilt': 1500})
            for name, state in self.app.gimbal.snapshot()['axes'].items():
                self.assertTrue(state['armed'])
                self.assertEqual(state['output_us'], state_before['axes'][name]['output_us'])
                self.assertEqual(state['target_us'], state_before['axes'][name]['target_us'])
            saved = json.loads(self.path.read_text(encoding='utf8'))
            self.assertEqual(saved, self.app.config)
            self.assertEqual(saved['axes'][axis]['calibration_points'], SCREENSHOT_ANGLE_POINTS[axis])
            self.assertTrue(saved['axes'][axis]['calibration_confirmed'])
        self.publish(11)
        self.app.command(dict(action='select', sequence=11, index=0))
        self.assertTrue(self.app.snapshot()['tracking_preflight']['ready'])
        self.assertEqual(self.app.command(dict(action='track', enabled=True)), {'tracking': True})
        self.assertEqual(self.app.tracker.camera.model, 'unavailable')
        self.assertEqual(self.backend.events, events)

    def test_invalid_angle_save_keeps_disk_tracking_selection_and_pending_targets_unchanged(self):
        self.track_screenshot_axes()
        disk, config = self.path.read_bytes(), deepcopy(self.app.config)
        state, events = self.app.gimbal.snapshot(), list(self.backend.events)
        selected, generation = list(self.app.selection_frames), self.app.config_generation
        invalid_tables = (None, [], SCREENSHOT_ANGLE_POINTS['pan'][:1],
            [dict(pulse_us=339, angle_deg=45), dict(pulse_us=2550, angle_deg=240)],
            [dict(pulse_us=380, angle_deg=45), dict(pulse_us=2550, angle_deg=45)],
            [dict(pulse_us=380, angle_deg=45), dict(pulse_us=1882, angle_deg=180), dict(pulse_us=1000, angle_deg=240)],
            [dict(pulse_us=380, angle_deg=float('nan')), dict(pulse_us=2550, angle_deg=240)],
            [dict(pulse_us=True, angle_deg=45), dict(pulse_us=2550, angle_deg=240)])
        for points in invalid_tables:
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.app.command(dict(action='save_angles', axis='pan', calibration_points=points))
            self.assertEqual(self.path.read_bytes(), disk)
            self.assertEqual(self.app.config, config)
            self.assertEqual(self.app.gimbal.snapshot(), state)
            self.assertEqual(self.backend.events, events)
            self.assertEqual(list(self.app.selection_frames), selected)
            self.assertEqual(self.app.config_generation, generation)
            self.assertTrue(self.app.tracking)
            self.assertTrue(self.app.target_selected)

    def test_angle_storage_failure_keeps_active_tracking_and_both_pending_targets(self):
        self.track_screenshot_axes()
        disk, config = self.path.read_bytes(), deepcopy(self.app.config)
        state, events = self.app.gimbal.snapshot(), list(self.backend.events)
        selected, generation = list(self.app.selection_frames), self.app.config_generation
        changed = deepcopy(SCREENSHOT_ANGLE_POINTS['pan'])
        changed[1]['angle_deg'] = 181
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('angle storage denied')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='save_angles', axis='pan', calibration_points=changed))
        self.assertEqual(self.path.read_bytes(), disk)
        self.assertEqual(self.app.config, config)
        self.assertEqual(self.app.gimbal.snapshot(), state)
        self.assertEqual(self.backend.events, events)
        self.assertEqual(list(self.app.selection_frames), selected)
        self.assertEqual(self.app.config_generation, generation)
        self.assertTrue(self.app.tracking)
        self.assertTrue(self.app.target_selected)

    def test_successful_angle_save_pauses_tracking_without_pwm_off_and_requires_fresh_reselection(self):
        self.track_screenshot_axes()
        tilt_before = deepcopy(self.app.config['axes']['tilt'])
        state, events = self.app.gimbal.snapshot(), list(self.backend.events)
        changed = deepcopy(SCREENSHOT_ANGLE_POINTS['pan'])
        changed[1]['angle_deg'] = 181
        self.app.command(dict(action='save_angles', axis='pan', calibration_points=changed))
        self.assertFalse(self.app.tracking)
        self.assertFalse(self.app.target_selected)
        self.assertEqual(list(self.app.selection_frames), [])
        self.assertIsNone(self.app.tracking_angles)
        self.assertHeld()  # Successful edit cancels old tracking motion only.
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.backend.enabled, {'pan': 1500, 'tilt': 1500})
        self.assertEqual(self.app.config['axes']['tilt'], tilt_before)
        for name, current in self.app.gimbal.snapshot()['axes'].items():
            self.assertTrue(current['armed'])
            self.assertEqual(current['output_us'], state['axes'][name]['output_us'])
        self.assertEqual(json.loads(self.path.read_text(encoding='utf8'))['axes']['pan']['calibration_points'], changed)
        with self.assertRaisesRegex(ValueError, '选择|点击'):
            self.app.command(dict(action='track', enabled=True))
        with self.assertRaisesRegex(ValueError, '画面已更新或过期'):
            self.app.command(dict(action='select', sequence=10, index=0))
        self.publish(11)
        self.app.command(dict(action='select', sequence=11, index=0))
        self.assertTrue(self.app.snapshot()['tracking_preflight']['ready'])
        self.assertTrue(self.app.command(dict(action='track', enabled=True))['tracking'])
        self.assertEqual(self.backend.events, events)

    def test_calibrated_pwm_off_preflight_explains_switches_and_enabling_then_reselection_is_ready(self):
        self.use_screenshot_axes(calibrated=True, enabled=False)
        self.publish(10)
        self.app.command(dict(action='select', sequence=10, index=0))
        prerequisite = self.app.snapshot()['tracking_preflight']
        self.assertFalse(prerequisite['ready'])
        for axis in ('pan', 'tilt'):
            detail = prerequisite['axes'][axis]
            self.assertTrue(detail['calibrated'])
            self.assertFalse(detail['pwm_enabled'])
            self.assertIn('PWM关闭', detail['reason'])
            self.assertNotIn('未保存', detail['reason'])
        with self.assertRaisesRegex(ValueError, '暂不能跟踪.*PWM关闭'):
            self.app.command(dict(action='track', enabled=True))
        self.assertEqual(self.backend.enabled, {})
        self.assertEqual(self.backend.events, [])
        saved = deepcopy(self.app.config)
        for axis in ('pan', 'tilt'):
            self.app.command(dict(action='manual_enable', axis=axis, pulse_us=1500))
        self.publish(11)
        self.app.command(dict(action='select', sequence=11, index=0))
        self.assertTrue(self.app.snapshot()['tracking_preflight']['ready'])
        self.app.command(dict(action='track', enabled=True))
        self.assertEqual(self.app.config, saved)
        self.assertEqual(self.backend.enabled, {'pan': 1500, 'tilt': 1500})

    def test_preflight_distinguishes_current_output_outside_angle_table_from_unsaved_calibration(self):
        self.use_screenshot_axes(calibrated=True, enabled=False)
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=340))
        self.app.command(dict(action='manual_enable', axis='tilt', pulse_us=1500))
        self.publish(10)
        self.app.command(dict(action='select', sequence=10, index=0))
        events = list(self.backend.events)
        prerequisites = self.app.snapshot()['tracking_preflight']
        pan, tilt = prerequisites['axes']['pan'], prerequisites['axes']['tilt']
        self.assertTrue(pan['calibrated'])
        self.assertTrue(pan['pwm_enabled'])
        self.assertFalse(pan['covered'])
        self.assertIn('角度表覆盖的380～2550', pan['reason'])
        self.assertNotIn('未保存', pan['reason'])
        self.assertNotIn('PWM关闭', pan['reason'])
        self.assertTrue(tilt['ready'])
        with self.assertRaisesRegex(ValueError, '角度表覆盖'):
            self.app.command(dict(action='track', enabled=True))
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.backend.enabled, {'pan': 340, 'tilt': 1500})

    def test_construction_and_publishing_never_enable_pwm(self):
        self.assertEqual(self.backend.events, [])
        self.publish()
        self.assertEqual(self.backend.events, [])
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.app.snapshot()["tracking"])

    def test_safe_window_required_even_for_explicit_manual_arm(self):
        self.app.command(dict(action="save_config", config=default_config()))
        with self.assertRaises(ValueError):
            self.app.command(dict(action="arm", axis="pan", start_us=1500, manual=True))
        self.assertEqual(self.backend.enabled, {})

    def test_simple_manual_draft_enable_and_axis_disable_do_not_need_limits(self):
        self.app.command(dict(action='save_config', config=default_config()))
        self.app.gimbal._clock = lambda: self.now
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=600))
        self.app.command(dict(action='manual_jog', axis='pan', direction=1, coarse=True))
        state = self.app.gimbal.snapshot()['axes']['pan']
        self.assertEqual(state['manual']['draft_us'], 650)
        self.assertFalse(state['configured'])
        self.assertEqual(self.backend.enabled, {})
        self.app.command(dict(action='manual_enable', axis='pan'))
        self.app.command(dict(action='manual_enable', axis='tilt'))
        self.assertEqual(self.backend.enabled, {'pan': 650, 'tilt': 1500})
        self.app.command(dict(action='disable', axis='pan'))
        self.assertEqual(self.backend.enabled, {'tilt': 1500})
        self.app.command(dict(action='manual_jog', axis='pan', direction=1))
        self.assertEqual(self.backend.enabled, {'tilt': 1500})
        self.app.command(dict(action='heartbeat'))
        self.assertEqual(self.backend.enabled, {'tilt': 1500})
        self.app.command(dict(action='save_config', config=deepcopy(self.app.config)))
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.app.gimbal.snapshot()['axes']['pan']['manual']['enabled'])

    def test_mark_limits_pair_persists_only_after_second_actual_settled_endpoint(self):
        self.app.command(dict(action='save_config', config=default_config()))
        self.app.gimbal._clock = lambda: self.now
        self.app.command(dict(action='manual_enable', axis='pan'))
        self.app.command(dict(action='manual_enable', axis='tilt'))
        before = self.path.read_bytes()
        self.app.command(dict(action='mark_limit', axis='pan', limit='min'))
        self.assertEqual(self.path.read_bytes(), before)
        self.app.command(dict(action='manual_jog', axis='pan', direction=1, coarse=True))
        with self.assertRaisesRegex(ValueError, '脉宽仍在渐变'):
            self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        for _ in range(4):
            self.now += .1
            self.app.gimbal.tick(self.now)
        self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        saved = json.loads(self.path.read_text(encoding='utf8'))
        self.assertEqual((saved['axes']['pan']['safe_min_us'], saved['axes']['pan']['safe_max_us']), (1500,1550))
        self.assertIsNone(saved['axes']['tilt']['safe_min_us'])
        self.assertEqual(self.backend.enabled, {'pan':1550,'tilt':1500})
        self.assertIsNone(self.app.gimbal.snapshot()['axes']['pan']['manual']['pending_min_us'])

    def test_limit_conflict_preserves_old_angles_and_has_explicit_retry(self):
        self.arm()
        self.app.command(dict(action='mark_limit', axis='pan', limit='min'))
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=1600))
        for _ in range(8):
            self.now += .1
            self.app.gimbal.tick(self.now)
        before = self.path.read_bytes()
        with self.assertRaisesRegex(ValueError, '清除旧角度并应用'):
            self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(len(self.app.config['axes']['pan']['calibration_points']), 3)
        self.app.command(dict(action='apply_limits', axis='pan', reset_angles=True))
        self.assertEqual(self.app.config['axes']['pan']['calibration_points'], [])
        self.assertEqual(len(self.app.config['axes']['tilt']['calibration_points']), 3)
        self.assertEqual(self.backend.enabled, {'pan':1600,'tilt':1500})

    def test_axis_steps_and_clear_limits_preserve_both_outputs_without_rearming(self):
        self.arm()
        events = list(self.backend.events)
        self.app.command(dict(action='manual_steps', axis='pan', fine_us=2, coarse_us=25))
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.app.config['axes']['pan']['fine_us'], 2)
        self.assertEqual(self.app.config['axes']['tilt']['fine_us'], 5)
        self.app.command(dict(action='clear_limits', axis='pan'))
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.backend.enabled, {'pan':1500,'tilt':1500})
        self.assertIsNone(self.app.config['axes']['pan']['safe_min_us'])
        self.assertEqual(self.app.config['axes']['pan']['calibration_points'], [])
        self.assertEqual(len(self.app.config['axes']['tilt']['calibration_points']), 3)
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=600))
        self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['target_us'], 600)
        self.assertEqual(self.backend.enabled['pan'], 1500)  # Still slew-limited.

    def test_manual_range_extends_only_pan_and_preserves_measured_tilt_and_output(self):
        config = default_config()
        config['axes']['tilt'].update(safe_min_us=645, safe_max_us=2420)
        self.app.command(dict(action='save_config', config=config))
        self.app.gimbal._clock = lambda: self.now
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=2500))
        self.app.command(dict(action='manual_enable', axis='tilt', pulse_us=2420))
        self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        events = list(self.backend.events)
        tilt = deepcopy(self.app.config['axes']['tilt'])
        self.app.command(dict(action='manual_range', axis='pan', explore_min_us=400, explore_max_us=2700))
        self.assertEqual(self.backend.events, events)  # No extra PWM write or enable.
        saved = json.loads(self.path.read_text(encoding='utf8'))
        self.assertEqual(saved['axes']['tilt'], tilt)
        self.assertEqual(saved['axes']['pan']['explore_min_us'], 400)
        self.assertEqual(saved['axes']['pan']['explore_max_us'], 2700)
        pan = self.app.gimbal.snapshot()['axes']['pan']
        self.assertEqual(pan['manual']['pending_max_us'], 2500)
        self.app.command(dict(action='manual_jog', axis='pan', direction=1))
        self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['target_us'], 2505)
        for _ in range(2):
            self.now += .1
            self.app.gimbal.tick(self.now)
        self.assertEqual(self.backend.enabled, {'pan':2505, 'tilt':2420})
        self.app.command(dict(action='manual_pulse', axis='tilt', pulse_us=2600))
        self.assertEqual(self.app.gimbal.snapshot()['axes']['tilt']['target_us'], 2420)
        # Rebuilding the controller from disk must retain the extended range and
        # measured upper-axis limits, without enabling either channel on load.
        self.app.command(dict(action='save_config', config=deepcopy(saved)))
        self.assertEqual(self.backend.enabled, {})
        self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['manual']['numeric_max_us'], 2700)
        self.assertEqual(self.app.config['axes']['tilt'], tilt)

    def test_manual_range_validation_and_storage_failures_preserve_disk_and_motion(self):
        self.app.command(dict(action='save_config', config=default_config()))
        self.app.gimbal._clock = lambda: self.now
        self.app.command(dict(action='manual_range', axis='pan', explore_min_us=400, explore_max_us=3000))
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=2600))
        before, events = self.path.read_bytes(), list(self.backend.events)
        for low, high in ((500,2500), (0,3000), (400,20000), (400,400), (None,3000), (True,3000)):
            with self.subTest(low=low,high=high), self.assertRaises(ValueError):
                self.app.command(dict(action='manual_range', axis='pan', explore_min_us=low, explore_max_us=high))
            self.assertEqual(self.path.read_bytes(), before)
            self.assertEqual(self.backend.events, events)
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('range storage denied')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='manual_range', axis='pan', explore_min_us=300, explore_max_us=3100))
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.app.config['axes']['pan']['explore_max_us'], 3000)
        self.assertEqual(self.backend.enabled, {'pan':2600})

    def test_record_extended_endpoints_does_not_lose_upper_axis_calibration(self):
        self.app.command(dict(action='clear_limits', axis='pan'))
        tilt = deepcopy(self.app.config['axes']['tilt'])
        self.app.command(dict(action='manual_range', axis='pan', explore_min_us=350, explore_max_us=3200))
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=450))
        self.app.command(dict(action='mark_limit', axis='pan', limit='min'))
        # Use a new explicit OFF/ON command to avoid simulating a long slew.
        self.app.command(dict(action='disable', axis='pan'))
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=2800))
        self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        saved = json.loads(self.path.read_text(encoding='utf8'))
        self.assertEqual((saved['axes']['pan']['safe_min_us'], saved['axes']['pan']['safe_max_us']), (450,2800))
        self.assertEqual(saved['axes']['tilt'], tilt)
        self.assertEqual(self.backend.enabled, {'pan':2800})

    def test_shrinking_exploration_cannot_silently_exclude_a_pending_endpoint(self):
        self.app.command(dict(action='save_config', config=default_config()))
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=2500))
        self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        self.app.command(dict(action='disable', axis='pan'))
        before, events = self.path.read_bytes(), list(self.backend.events)
        with self.assertRaisesRegex(ValueError, '已暂存的端点'):
            self.app.command(dict(action='manual_range', axis='pan', explore_min_us=500, explore_max_us=2400))
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.backend.events, events)
        self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['manual']['pending_max_us'], 2500)

    def test_range_storage_failure_preserves_tracking_and_success_pauses_without_pwm_writes(self):
        self.begin_tracking()
        self.app.gimbal.set_angle('pan', 60, tracking=True)
        before, events = self.path.read_bytes(), list(self.backend.events)
        targets = {axis: data['target_us'] for axis,data in self.app.gimbal.snapshot()['axes'].items()}
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('range storage denied')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='manual_range', axis='pan', explore_min_us=500, explore_max_us=2600))
        self.assertTrue(self.app.tracking)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.backend.events, events)
        self.assertEqual({axis: data['target_us'] for axis,data in self.app.gimbal.snapshot()['axes'].items()}, targets)
        self.app.command(dict(action='manual_range', axis='pan', explore_min_us=500, explore_max_us=2600))
        self.assertFalse(self.app.tracking)
        self.assertEqual(self.backend.events, events)
        self.assertHeld()

    def test_limit_storage_failure_retains_pending_pair_and_outputs_for_retry(self):
        self.app.command(dict(action='save_config', config=default_config()))
        self.app.gimbal._clock = lambda: self.now
        self.app.command(dict(action='manual_enable', axis='pan'))
        self.app.command(dict(action='mark_limit', axis='pan', limit='min'))
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=1550))
        for _ in range(4):
            self.now += .1
            self.app.gimbal.tick(self.now)
        before = self.path.read_bytes()
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('fake storage permission')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='mark_limit', axis='pan', limit='max'))
        self.assertEqual(self.path.read_bytes(), before)
        self.assertFalse(self.app.gimbal.snapshot()['axes']['pan']['configured'])
        self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['manual']['pending_max_us'], 1550)
        self.assertEqual(self.backend.enabled, {'pan':1550})
        self.app.command(dict(action='apply_limits', axis='pan'))
        self.assertTrue(self.app.gimbal.snapshot()['axes']['pan']['configured'])

    def test_uncalibrated_axes_cannot_track(self):
        config = measured_test_config()
        for axis in config["axes"].values():
            axis.update(calibration_points=[], calibration_confirmed=False)
        self.app.command(dict(action="save_config", config=config))
        self.app.gimbal._clock = lambda: self.now
        self.arm(manual=True)
        self.publish()
        self.app.command(dict(action="select", sequence=10, index=0))
        with self.assertRaises(ValueError):
            self.app.command(dict(action="track", enabled=True))
        self.assertFalse(self.app.tracking)

    def test_explicit_target_selection_required(self):
        self.arm()
        self.publish()
        with self.assertRaises(ValueError):
            self.app.command(dict(action="track", enabled=True))
        self.assertFalse(self.app.tracking)

    def test_selection_cache_uses_selected_frame_and_rejects_expired_or_unknown_sequence(self):
        first = detection(180, 180)
        self.publish(10, boxes=[first])
        self.now += .1
        self.publish(11, boxes=[detection(480, 280, "human")])
        selected = self.app.command(dict(action="select", sequence=10, index=0))
        self.assertEqual(selected["selected"], first)
        for sequence, index in ((999, 0), (10, True), (10, -1), (10, 1)):
            with self.assertRaises(ValueError):
                self.app.command(dict(action="select", sequence=sequence, index=index))
        self.now += .8
        with self.assertRaises(ValueError):
            self.app.command(dict(action="select", sequence=10, index=0))

    def test_select_does_not_accept_float_or_boolean_frame_identifiers(self):
        self.publish(1)
        for sequence in (True, 1.0):
            with self.assertRaises(ValueError):
                self.app.command(dict(action="select", sequence=sequence, index=0))

    def test_reselect_holds_old_pending_targets_immediately(self):
        self.begin_tracking()
        self.app.gimbal.set_angle("pan", 60, tracking=True)
        self.app.gimbal.set_angle("tilt", -30, tracking=True)
        self.assertNotEqual(self.app.gimbal.snapshot()["axes"]["pan"]["target_us"], 1500)
        self.app.command(dict(action="select", sequence=10, index=0))
        self.assertHeld()

    def test_save_configuration_disables_pwm_and_persists_no_runtime_enable(self):
        self.arm()
        old = self.app.gimbal
        result = self.app.command(dict(action="save_config", config=deepcopy(self.config)))
        self.assertEqual(result["saved"], str(self.path))
        self.assertEqual(self.backend.enabled, {})
        self.assertTrue(old.snapshot()["closed"])
        self.assertFalse(self.app.tracking)
        self.assertFalse(self.app.target_selected)
        self.assertIsNone(self.app.tracker)
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")), self.config)
        self.assertTrue(all(not axis["armed"] for axis in self.app.gimbal.snapshot()["axes"].values()))
        self.assertEqual([event for event in self.backend.events if event[0] == "start"],
                         [("start", "pan", 1500), ("start", "tilt", 1500)])

    def test_failed_configuration_write_leaves_old_file_but_outputs_disabled(self):
        self.arm()
        before = self.path.read_bytes()
        with mock.patch.object(usb_gimbal, "atomic_json", side_effect=OSError("mock disk failure")):
            with self.assertRaises(OSError):
                self.app.command(dict(action="save_config", config=deepcopy(self.config)))
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.app.tracking)

    def test_inference_started_before_calibration_edit_cannot_publish_into_new_tracker(self):
        original = self.publish()
        generation = self.app.config_generation
        changed = deepcopy(self.config)
        changed["camera"]["fx"] = 450
        self.app.command(dict(action="save_config", config=changed))
        self.assertGreater(self.app.config_generation, generation)
        self.assertFalse(self.app.publish(self.frame(11), config_generation=generation))
        self.assertIs(self.app.latest, original)
        self.assertIsNone(self.app.tracker)
        self.assertFalse(self.app.target_selected)
        self.assertTrue(self.app.publish(self.frame(12), config_generation=self.app.config_generation))
        self.assertEqual(self.app.latest.sequence, 12)
        self.assertEqual(self.app.tracker.camera.fx, 450)
        self.assertEqual(self.backend.enabled, {})

    def test_invalid_camera_or_tracking_configuration_is_rejected_without_a_frame(self):
        before = self.path.read_bytes()
        for part in (dict(camera={"fx": 400}), dict(tracking={"loss_gap_s": 0})):
            with self.subTest(part=part):
                candidate = deepcopy(self.config)
                candidate.update(part)
                with self.assertRaises(ValueError):
                    self.app.command(dict(action="save_config", config=candidate))
                self.assertEqual(self.path.read_bytes(), before)
                self.assertEqual(self.backend.enabled, {})

    def test_five_second_browser_watchdog_disables_and_requires_explicit_rearm(self):
        self.arm()
        self.now += 5.01
        self.app._control_once(self.now)
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.app.tracking)
        self.assertIn("心跳", self.app.control_error)
        with self.assertRaises(ValueError):
            self.app.command(dict(action="arm", axis="pan", start_us=1500))
        self.app.command(dict(action="heartbeat"))
        self.assertIsNone(self.app.control_error)
        self.assertTrue(self.app.browser_recovered)
        self.app._control_once(self.now)
        self.assertEqual(self.backend.enabled, {})  # heartbeat alone never re-enables
        self.app.command(dict(action="arm", axis="pan", start_us=1500))
        self.assertEqual(self.backend.enabled, {"pan": 1500})

    def test_missing_without_a_live_observation_and_stale_frames_hold_pending_motion(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                self.app.close()
                self.backend = MockPWMBackend()
                self.app = GimbalApp(self.path, backend=self.backend)
                self.app.gimbal._clock = lambda: self.now
                self.addCleanup(self.app.close)
                self.begin_tracking()
                self.app.gimbal.set_angle("pan", 60, tracking=True)
                self.app.gimbal.set_angle("tilt", 30, tracking=True)
                self.now += .1 if not stale else .8
                if not stale:
                    self.publish(11, boxes=[])
                self.app._control_once(self.now)
                self.assertHeld()
                self.assertEqual(self.app.tracking_result["status"], "searching" if not stale else "stale")
                self.assertTrue(self.app.tracking)
                self.assertTrue(self.app.target_selected)

    def test_new_frames_drive_only_once_and_out_of_order_frame_freezes(self):
        self.begin_tracking()
        self.now += .1
        self.publish(11)
        self.app._control_once(self.now)
        first_targets = {axis: state["target_us"] for axis, state in self.app.gimbal.snapshot()["axes"].items()}
        self.now += .02
        self.app._control_once(self.now)
        self.assertEqual(first_targets,
            {axis: state["target_us"] for axis, state in self.app.gimbal.snapshot()["axes"].items()})
        self.now += .02
        self.publish(9)
        self.app._control_once(self.now)
        self.assertEqual(self.app.tracking_result["status"], "out_of_order")
        self.assertHeld()

    def test_capture_time_pose_not_current_gimbal_pose_drives_direction_transform(self):
        self.begin_tracking(boxes=[detection(320, 240)])
        captured = self.app.gimbal.snapshot()
        captured["axes"]["pan"]["angle_deg"] = 30
        captured["axes"]["tilt"]["angle_deg"] = 20
        self.now += .1
        self.publish(11, boxes=[detection(320, 240)], pose=captured)
        self.app._control_once(self.now)
        result = self.app.tracking_result
        expected = (math.cos(math.radians(20)) * math.cos(math.radians(30)),
                    math.cos(math.radians(20)) * math.sin(math.radians(30)), math.sin(math.radians(20)))
        for actual, wanted in zip(result["body_direction"], expected):
            self.assertAlmostEqual(actual, wanted)
        self.assertEqual(self.app.gimbal.snapshot()["axes"]["pan"]["angle_deg"], 0)
        self.assertIsNone(result["distance_m"])
        self.assertIsNone(result["translation_m"])

    def test_calibration_flag_must_also_have_been_valid_at_frame_arrival(self):
        self.begin_tracking(boxes=[detection(320, 240)])
        captured = self.app.gimbal.snapshot()
        captured["axes"]["pan"].update(angle_deg=30, calibrated=False)
        self.now += .1
        self.publish(11, boxes=[detection(320, 240)], pose=captured)
        self.app._control_once(self.now)
        self.assertFalse(self.app.tracking_result["body_direction_valid"])

    def test_no_fpga_wheel_command_or_outbound_socket_is_produced(self):
        with mock.patch("socket.socket") as socket_constructor:
            self.begin_tracking()
            self.now += .1
            self.publish(11)
            self.app._control_once(self.now)
            for action in ("motor", "wheel", "navigate", "fpga"):
                with self.assertRaises(ValueError):
                    self.app.command(dict(action=action, left=100, right=100))
            socket_constructor.assert_not_called()
        self.assertTrue(all(event[1] in ("pan", "tilt") for event in self.backend.events))

    def test_control_thread_exception_disables_and_close_joins(self):
        self.arm()
        disabled = threading.Event()
        original_disable = self.app.gimbal.disable

        def disable(*args, **kwargs):
            result = original_disable(*args, **kwargs)
            disabled.set()
            return result

        with mock.patch.object(self.app, "_control_once", side_effect=RuntimeError("mock control fault")), \
                mock.patch.object(self.app.gimbal, "disable", side_effect=disable):
            self.app.start(SimpleNamespace(actual={}, error=None))
            self.assertTrue(disabled.wait(1), "control thread did not handle its exception")
            self.app.close()
        self.assertFalse(self.app.thread.is_alive())
        self.assertEqual(self.backend.enabled, {})
        self.assertIn("mock control fault", self.app.control_error)
        self.app.close()  # idempotent

    def test_inference_worker_failure_disables_outputs_without_importing_real_cv2(self):
        self.arm()
        self.app.tracking = True
        frame = CameraFrame(10, self.now, object(), self.app.gimbal.snapshot())
        camera = SimpleNamespace(newest=mock.Mock(return_value=frame))
        worker = VisionWorker(self.app, camera, SimpleNamespace(no_detection=True))
        # object() has no copy()/shape: this intentionally exercises the worker
        # exception path. These module stubs perform no image processing.
        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(), numpy=SimpleNamespace()):
            worker._run()
        self.assertIsNotNone(self.app.error)
        self.assertFalse(self.app.tracking)
        self.assertEqual(self.backend.enabled, {})

    def test_camera_failure_exits_inference_worker_instead_of_spinning_on_none(self):
        self.arm()
        self.app.tracking = True
        calls = []

        def newest(**kwargs):
            calls.append(None)
            if len(calls) >= 5:
                self.app.stop.set()  # bounded guard against the pre-fix spin
            return None

        camera = SimpleNamespace(newest=newest, error="mock camera unplugged")
        worker = VisionWorker(self.app, camera, SimpleNamespace(no_detection=True))
        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(), numpy=SimpleNamespace()):
            worker._run()
        self.assertLessEqual(len(calls), 1)
        self.assertIn("mock camera unplugged", self.app.error or "")
        self.assertFalse(self.app.tracking)
        self.assertEqual(self.backend.enabled, {})

    def test_capture_passes_original_pixels_to_encoder_and_marks_detections_not_ground_truth(self):
        frame = self.publish()
        encoder = mock.Mock(return_value=True)
        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(imwrite=encoder)):
            result = self.app.command(dict(action="capture", category="positive"))
        encoder.assert_called_once()
        self.assertIs(encoder.call_args.args[1], frame.image)
        metadata = json.loads(Path(result["saved"]).with_suffix(".json").read_text(encoding="utf-8"))
        self.assertIn("RAW original", metadata["image"])
        self.assertIn("not ground truth", metadata["annotation"])
        self.assertEqual(metadata["observations"], frame.detections)
        self.assertEqual(self.backend.enabled, {})

    def test_axis_form_saves_limits_steps_and_start_without_camera_calibration(self):
        self.app.config.pop('camera', None)
        self.arm()
        self.app.config['axes']['pan'].update(calibration_points=[], calibration_confirmed=False)
        result = self.app.command(dict(action='save_axis', axis='pan', safe_min_us=1100,
            safe_max_us=1900, start_us=1500, fine_us=2, coarse_us=25))
        self.assertEqual(result['saved'], str(self.path))
        saved = json.loads(self.path.read_text(encoding='utf-8'))
        self.assertEqual(saved['axes']['pan']['start_us'], 1500)
        self.assertEqual(saved['axes']['pan']['fine_us'], 2)
        self.assertNotIn('camera', saved)
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.app.gimbal.snapshot()['axes']['pan']['armed'])

    def test_axis_limits_never_silently_drop_old_angle_measurements(self):
        with self.assertRaises(ValueError):
            self.app.command(dict(action='save_axis', axis='pan', safe_min_us=1100,
                safe_max_us=1900, start_us=1500, fine_us=2, coarse_us=25))
        self.assertEqual(len(self.app.config['axes']['pan']['calibration_points']), 3)

    def test_axis_form_explicit_reset_only_clears_selected_axis_measurements(self):
        self.app.command(dict(action='save_axis', axis='pan', safe_min_us=1100,
            safe_max_us=1900, start_us=1500, fine_us=2, coarse_us=25, reset_angles=True))
        self.assertEqual(self.app.config['axes']['pan']['calibration_points'], [])
        self.assertFalse(self.app.config['axes']['pan']['calibration_confirmed'])
        self.assertEqual(len(self.app.config['axes']['tilt']['calibration_points']), 3)
        self.assertEqual(self.backend.enabled, {})

    def test_axis_form_rejects_reversed_or_incomplete_limits_and_outside_start(self):
        for values in ((1600, 1400, 1500), (None, 1700, 1500), (1000, 2000, 999), (1000, 2000, 1500.5)):
            before = self.path.read_bytes()
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.app.command(dict(action='save_axis', axis='pan', safe_min_us=values[0],
                    safe_max_us=values[1], start_us=values[2], fine_us=5, coarse_us=50))
            self.assertEqual(self.path.read_bytes(), before)

    def test_burst_ten_distinct_raw_frames_does_not_block_control_or_heartbeat(self):
        self.publish(10)
        self.arm()
        written, io_started, io_release = [], threading.Event(), threading.Event()

        def encode(path, image):
            written.append((path, image))
            if len(written) == 1:
                io_started.set()
                self.assertTrue(io_release.wait(1), 'test did not release image writer')
            return True

        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(imwrite=encode)):
            self.app.start_burst('positive', interval=.01)
            with self.assertRaises(ValueError):
                self.app.start_burst('negative')
            self.now += .02
            first = self.publish(11)
            self.assertTrue(io_started.wait(1))
            # I/O is deliberately blocked in the other thread. These would
            # deadlock/time out if capture held app.lock across imwrite.
            self.app.command(dict(action='heartbeat'))
            self.app.command(dict(action='disable', axis='pan'))
            self.assertNotIn('pan', self.backend.enabled)
            io_release.set()
            for sequence in range(12, 21):
                deadline = time.monotonic() + 1
                while self.app.snapshot()['capture']['saved'] < sequence - 11:
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.001)
                self.now += .02
                self.publish(sequence)
            self.app.capture_thread.join(1)
        self.assertFalse(self.app.capture_thread.is_alive())
        status = self.app.snapshot()['capture']
        self.assertEqual(status['saved'], 10)
        self.assertFalse(status['active'])
        self.assertIsNone(status['error'])
        self.assertIs(written[0][1], first.image)
        self.assertEqual(len({id(image) for _, image in written}), 10)
        records = [json.loads(Path(path).with_suffix('.json').read_text(encoding='utf-8')) for path, _ in written]
        self.assertEqual([record['frame'] for record in records], list(range(11, 21)))
        self.assertTrue(all('not ground truth' in record['annotation'] for record in records))

    def test_burst_cancel_shutdown_and_stale_timeout_never_reuse_a_frame(self):
        self.publish(10)
        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(imwrite=mock.Mock(return_value=True))):
            waiting = threading.Event()
            original_wait = self.app.condition.wait

            def wait(timeout=None):
                waiting.set()
                return original_wait(timeout)

            with mock.patch.object(self.app.condition, 'wait', side_effect=wait):
                self.app.start_burst('negative', interval=.01, frame_timeout=.1)
                self.assertTrue(waiting.wait(1))
                with self.app.condition:
                    self.now += .11
                    self.app.condition.notify_all()
                self.app.capture_thread.join(1)
            self.assertIn('未收到新的', self.app.snapshot()['capture']['error'])
            self.assertEqual(self.app.capture_count, 0)
            self.publish(11)
            self.app.start_burst('negative', interval=.01)
            self.app.command(dict(action='cancel_capture'))
            self.app.capture_thread.join(1)
            self.assertTrue(self.app.snapshot()['capture']['cancelled'])
            self.app.start_burst('negative', interval=.01)
            self.app.close()
            self.assertFalse(self.app.capture_thread.is_alive())
            self.assertTrue(self.app.snapshot()['capture']['cancelled'])

    def test_burst_save_failure_remains_in_capture_state_separate_from_control_error(self):
        self.publish(10)
        with mock.patch.dict(sys.modules, cv2=SimpleNamespace(imwrite=mock.Mock(side_effect=OSError('disk full')))):
            self.app.start_burst('positive', interval=.01)
            self.now += .02
            self.publish(11)
            self.app.capture_thread.join(1)
        self.assertEqual(self.app.snapshot()['capture']['saved'], 0)
        self.assertIn('disk full', self.app.snapshot()['capture']['error'])
        self.assertIsNone(self.app.control_error)

    def test_fractional_tracking_requests_accumulate_past_pwm_rounding(self):
        self.begin_tracking()
        result = dict(status='tracking', delta_yaw_deg=.03, delta_pitch_deg=0,
                      yaw_rate_dps=.3, pitch_rate_dps=0, control_dt_s=.1)
        with mock.patch.object(self.app.tracker, 'update', return_value=result):
            for sequence in range(11, 21):
                self.now += .1
                self.publish(sequence)
                self.app._control_once(self.now)
        self.assertGreater(self.app.gimbal.snapshot()['axes']['pan']['output_us'], 1500)
        # The pose conversion slightly damps the requested angle. It must
        # still preserve fractional requests across integer-PWM rounding.
        self.assertGreater(self.app.tracking_angles['pan'], .25)
        self.assertLessEqual(self.app.tracking_angles['pan'], .3)
        self.app.command(dict(action='track', enabled=False))
        self.assertIsNone(self.app.tracking_angles)
        self.assertHeld()

    def test_measured_tracking_limit_does_not_wind_up_and_reversal_returns(self):
        self.begin_tracking()
        self.app.gimbal.set_angle('pan', 90, tracking=True)
        self.now += 4
        for _ in range(40):
            self.now += .1
            self.app.gimbal.tick(self.now)
        # Move to the high endpoint within the measured fixture, then feed
        # independent tracking requests without relying on image association.
        self.app.tracking_angles = dict(pan=90., tilt=0.)
        result = dict(status='tracking', delta_yaw_deg=2., delta_pitch_deg=0.,
                      yaw_rate_dps=20., pitch_rate_dps=0., control_dt_s=.1)
        with mock.patch.object(self.app.tracker, 'update', return_value=result):
            for sequence in range(11, 31):
                self.now += .1
                self.app.last_heartbeat = self.now
                self.publish(sequence)
                self.app._control_once(self.now)
            self.assertTrue(self.app.tracking_limits['pan'])
            self.assertEqual(self.app.tracking_angles['pan'], 90.)
            self.assertEqual(self.app.gimbal.snapshot()['axes']['pan']['target_us'], 2000)
            result['delta_yaw_deg'] = -2.
            result['yaw_rate_dps'] = -20.
            self.now += .1
            self.publish(31)
            self.app._control_once(self.now)
            self.assertFalse(self.app.tracking_limits['pan'])
            self.assertLess(self.app.gimbal.snapshot()['axes']['pan']['target_us'], 2000)
        self.now += .8
        self.app._control_once(self.now)
        self.assertIsNone(self.app.tracking_angles)
        self.assertHeld()

    def test_tracking_requested_lead_is_bounded_and_zero_rate_holds(self):
        self.begin_tracking()
        result = dict(status='tracking', delta_yaw_deg=100., delta_pitch_deg=0.,
                      yaw_rate_dps=1000., pitch_rate_dps=0., control_dt_s=.1)
        with mock.patch.object(self.app.tracker, 'update', return_value=result):
            self.now += .1
            self.publish(11)
            before = self.app.gimbal.snapshot()['axes']['pan']['angle_deg']
            self.app._control_once(self.now)
            self.assertLessEqual(self.app.tracking_angles['pan'] - before, 5.)
            result['delta_yaw_deg'] = 0.
            result['yaw_rate_dps'] = 0.
            self.now += .1
            self.publish(12)
            self.app._control_once(self.now)
            self.assertHeld()

    def test_tracking_form_applies_without_intrinsics_and_pauses_without_disabling(self):
        self.begin_tracking()
        self.app.config.pop('camera', None)
        options = dict(deadband=.04, normalized_gain_dps=50, max_rate_dps=20,
            filter_time_constant_s=.06, deadband_hysteresis=.015, min_rate_dps=2, max_control_dt_s=.2)
        self.app.gimbal.set_angle('pan', 30, tracking=True)
        response = self.app.command(dict(action='save_tracking', tracking=options))
        self.assertTrue(response['tracking_paused'])
        self.assertFalse(self.app.tracking)
        self.assertFalse(self.app.target_selected)
        self.assertHeld()
        self.assertIsNone(self.app.tracking_angles)
        self.assertEqual(set(self.backend.enabled), {'pan', 'tilt'})
        self.assertEqual(self.app.tracker.camera.model, 'unavailable')
        self.assertEqual(self.app.tracker.config.normalized_gain_dps, 50)
        self.assertEqual(json.loads(self.path.read_text(encoding='utf-8'))['tracking'], options)
        with self.assertRaises(ValueError):
            self.app.command(dict(action='track', enabled=True))

    def test_tracking_form_rejects_incomplete_nonfinite_and_invalid_values(self):
        settings = self.app.snapshot()['tracking_settings']
        names = ('deadband', 'normalized_gain_dps', 'max_rate_dps', 'filter_time_constant_s',
                 'deadband_hysteresis', 'min_rate_dps', 'max_control_dt_s')
        valid = {name: settings[name] for name in names}
        for candidate in ({}, dict(valid, deadband=1), dict(valid, min_rate_dps=-1),
                          dict(valid, max_control_dt_s=0), dict(valid, normalized_gain_dps=float('nan'))):
            before = self.path.read_bytes()
            with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                self.app.command(dict(action='save_tracking', tracking=candidate))
            self.assertEqual(self.path.read_bytes(), before)


class HttpIntegrationTests(AppFixture):
    def setUp(self):
        super().setUp()
        self.publish()
        if not LOOPBACK:
            return
        self.server = usb_gimbal.ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.app))
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(target=self.server.serve_forever,
                                              kwargs=dict(poll_interval=.02), daemon=True)
        self.server_thread.start()
        self.addCleanup(self.close_server)

    def close_server(self):
        self.app.close()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(2)
        self.assertFalse(self.server_thread.is_alive())

    def request(self, method="POST", body=b'{"action":"heartbeat"}', headers=None, route="/command"):
        if not LOOPBACK:
            code, _, payload = self.memory_request(method, body, headers, route)
            return code, json.loads(payload)
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        try:
            defaults = {"Content-Type": "application/json", "X-Control-Token": self.app.token}
            if headers is not None:
                defaults.update(headers)
            connection.request(method, route, body, defaults)
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def memory_request(self, method, body, headers, route, stop_after_jpeg=False):
        body = b"" if body is None else body
        values = {"Content-Type": "application/json", "X-Control-Token": self.app.token}
        values.update(headers or {})
        values.setdefault("Content-Length", str(len(body)))
        raw = (f"{method} {route} HTTP/1.0\r\nHost: 127.0.0.1\r\n" +
               "".join(f"{key}: {value}\r\n" for key, value in values.items()) + "\r\n").encode("ascii") + body
        app = self.app

        class MemoryConnection:
            def __init__(self):
                self.incoming, self.outgoing = io.BytesIO(raw), io.BytesIO()

            def makefile(self, *args):
                return self.incoming

            def sendall(self, data):
                self.outgoing.write(data)
                if stop_after_jpeg and b"fake-jpeg" in data:
                    app.stop.set()

            def settimeout(self, value):
                pass

        connection = MemoryConnection()
        make_handler(self.app)(connection, ("127.0.0.1", 0), SimpleNamespace())
        response_headers, payload = connection.outgoing.getvalue().split(b"\r\n\r\n", 1)
        lines = response_headers.decode("ascii").split("\r\n")
        status = int(lines[0].split()[1])
        response_headers = dict(line.split(": ", 1) for line in lines[1:])
        return status, response_headers, payload

    def test_command_requires_session_token_and_valid_json_object(self):
        for token in ("", "wrong-session-token"):
            code, _ = self.request(headers={"X-Control-Token": token})
            self.assertEqual(code, 403)
        for body in (b"not-json", b"[]", b"null", b"\xff", b'"heartbeat"', b'{"action":"unknown"}'):
            with self.subTest(body=body):
                self.assertEqual(self.request(body=body)[0], 400)
        self.assertEqual(self.request(headers={"Content-Type": "text/plain"})[0], 400)
        code, response = self.request()
        self.assertEqual(code, 200)
        self.assertTrue(response["ok"])
        self.assertEqual(self.backend.enabled, {})

    def test_request_size_and_route_validation(self):
        for value in ("0", "-1", "65537", "NaN"):
            self.assertEqual(self.request(body=b"", headers={"Content-Length": value})[0], 400)
        self.assertEqual(self.request(route="/wrong-route")[0], 404)
        self.assertEqual(self.request(method="GET", body=None, route="/wrong-route")[0], 404)

    @unittest.skipIf(LOOPBACK, "truncated body uses deterministic EOF in the in-memory connection")
    def test_truncated_json_body_is_not_accepted_as_a_complete_command(self):
        self.assertEqual(self.request(headers={"Content-Length": "64"})[0], 400)

    def test_http_state_reports_mock_frame_and_direction_only(self):
        code, state = self.request(method="GET", body=None, route="/state")
        self.assertEqual(code, 200)
        self.assertFalse(state["gimbal"]["backend"]["physical"])
        self.assertEqual(state["frame"]["sequence"], 10)
        self.assertTrue(state["frame"]["fresh"])
        self.assertEqual(state["frame"]["detections"], [detection()])
        self.assertIn("no target range or 3D position", state["position_note"])
        self.assertEqual(self.backend.enabled, {})

    def test_http_storage_failure_returns_503_without_enabling_outputs(self):
        with mock.patch.object(self.app, "command", side_effect=OSError("mock storage failure")):
            code, response = self.request()
        self.assertEqual(code, 503)
        self.assertIn("mock storage failure", response["error"])
        self.assertEqual(self.backend.enabled, {})

    def test_mjpeg_uses_published_fake_jpeg_without_opening_a_camera(self):
        if not LOOPBACK:
            code, headers, payload = self.memory_request("GET", None, None, "/stream.mjpg", True)
            self.assertEqual(code, 200)
            self.assertIn("multipart/x-mixed-replace", headers["Content-Type"])
            self.assertEqual(payload, b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 9\r\n\r\nfake-jpeg\r\n")
            self.assertEqual(self.backend.enabled, {})
            return
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        try:
            connection.request("GET", "/stream.mjpg")
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn("multipart/x-mixed-replace", response.getheader("Content-Type"))
            self.assertEqual(response.fp.readline(), b"--frame\r\n")
            self.assertEqual(response.fp.readline(), b"Content-Type: image/jpeg\r\n")
            self.assertEqual(response.fp.readline(), b"Content-Length: 9\r\n")
            self.assertEqual(response.fp.readline(), b"\r\n")
            self.assertEqual(response.fp.read(9), b"fake-jpeg")
            response.close()
        finally:
            connection.close()
        self.assertEqual(self.backend.enabled, {})


class CameraLifecycleTests(unittest.TestCase):
    def test_close_before_start_and_waiting_newest_are_safe(self):
        camera = UsbCamera("/dev/NOT_A_REAL_CAMERA")
        result = []
        waiting = threading.Thread(target=lambda: result.append(camera.newest(timeout=30)))
        waiting.start()
        camera.close()
        waiting.join(1)
        self.assertFalse(waiting.is_alive())
        self.assertEqual(result, [None])
        camera.close()

    def test_mocked_capture_read_failure_releases_capture_and_wakes_waiter(self):
        capture = mock.Mock()
        capture.isOpened.return_value = True
        capture.getBackendName.return_value = "PURE_TEST_DOUBLE"
        capture.get.return_value = 640
        capture.read.side_effect = RuntimeError("mock read failed")
        # This object replaces the module import entirely; no cv2/numpy package
        # is imported and VideoCapture is never invoked on a physical device.
        fake_cv2 = SimpleNamespace(VideoCapture=mock.Mock(return_value=capture),
            CAP_V4L2=1, CAP_PROP_FOURCC=2, CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5, CAP_PROP_BUFFERSIZE=6,
            VideoWriter_fourcc=mock.Mock(return_value=0))
        camera = UsbCamera("/dev/NOT_A_REAL_CAMERA")
        with mock.patch.dict(sys.modules, cv2=fake_cv2):
            camera.start()
            camera.newest(timeout=.5)
            camera.close()
        self.assertFalse(camera.thread.is_alive())
        self.assertTrue(camera.stop.is_set())
        self.assertIn("mock read failed", camera.error)
        capture.release.assert_called_once()


if __name__ == "__main__":
    unittest.main(verbosity=2)
