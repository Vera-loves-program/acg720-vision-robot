"""Real GimbalApp save commands against isolated files and MockPWM only.

No camera, network, FPGA or physical PWM. Failures must preserve persisted
bytes AND live settings, pending motion, tracking selection and backend state.
"""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
import usb_gimbal
from gimbal_control import MockPWMBackend
from usb_gimbal import GimbalApp, VisionFrame


class RobotPolicyMock(MockPWMBackend):
    # Exercise the same runtime angle policy as FPGA output, but every output
    # remains an in-memory MockPWMBackend operation.
    physical = True


def measured_fixture():
    axis = dict(safe_min_us=500, safe_max_us=2500, explore_min_us=400, explore_max_us=2600,
                fine_us=5, coarse_us=50, slew_us_per_s=150, calibration_confirmed=True)
    return dict(axes=dict(
        pan=dict(axis, calibration_points=[dict(angle_deg=0, pulse_us=500), dict(angle_deg=180, pulse_us=2500)]),
        tilt=dict(axis, calibration_points=[dict(angle_deg=0, pulse_us=2500), dict(angle_deg=180, pulse_us=500)])),
        camera=dict(width=640, height=480), mount=dict(upright_tilt_deg=180))


class SaveTransactionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='save-transaction-', dir=ROOT / 'tools')
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'gimbal.json'
        self.path.write_text(json.dumps(measured_fixture()), encoding='utf-8')
        self.now = 1000.
        clock = SimpleNamespace(monotonic=lambda: self.now, perf_counter=time.perf_counter)
        self.clock_patch = mock.patch.object(usb_gimbal, 'time', clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.backend = RobotPolicyMock()
        self.app = GimbalApp(self.path, backend=self.backend, dataset=Path(self.temporary.name) / 'photos')
        self.app.gimbal._clock = lambda: self.now
        self.addCleanup(self.app.close)
        self.app.command(dict(action='manual_enable', axis='pan', pulse_us=1500))
        self.app.command(dict(action='manual_enable', axis='tilt', pulse_us=1000))
        box = dict(kind='dog', score=.9, x1=140, x2=220, y1=140, y2=220)
        self.app.publish(VisionFrame(10, self.now, object(), b'fixture', [box],
                                    self.app.gimbal.snapshot(), 640, 480, 12., 'original USB pixels'))
        self.app.command(dict(action='select', sequence=10, index=0))
        self.app.command(dict(action='track', enabled=True))
        self.app.gimbal.set_angle('pan', 100, tracking=True)
        self.app.gimbal.set_angle('tilt', 120, tracking=True)

    def state(self):
        return dict(disk=self.path.read_bytes(), config=deepcopy(self.app.config),
                    generation=self.app.config_generation, gimbal=deepcopy(self.app.gimbal.snapshot()),
                    tracking=self.app.tracking, selected=self.app.target_selected,
                    frames=deepcopy(list(self.app.selection_frames)), angles=deepcopy(self.app.tracking_angles),
                    memory=deepcopy(self.app.tracking_memory), servo=deepcopy(self.app._live_servo.snapshot()),
                    events=deepcopy(self.backend.events), enabled=deepcopy(self.backend.enabled))

    def rejected_before_storage(self, command):
        before = self.state()
        with mock.patch.object(usb_gimbal, 'atomic_json') as persist:
            with self.assertRaises(ValueError):
                self.app.command(command)
            persist.assert_not_called()
        self.assertEqual(self.state(), before)

    def test_new_tilt_table_not_covering90_rejected_before_any_disk_or_live_mutation(self):
        self.rejected_before_storage(dict(action='save_angles', axis='tilt', calibration_points=[
            dict(angle_deg=0, pulse_us=2500), dict(angle_deg=45, pulse_us=500)]))

    def test_clear_tilt_table_rejected_without_pausing_or_destroying_pending_commands(self):
        self.rejected_before_storage(dict(action='clear_limits', axis='tilt'))

    def test_invalid_manual_steps_rejected_without_pausing_tracking(self):
        self.rejected_before_storage(dict(action='manual_steps', axis='pan', fine_us=0, coarse_us=50))

    def test_invalid_manual_range_rejected_without_pausing_tracking(self):
        self.rejected_before_storage(dict(action='manual_range', axis='pan', explore_min_us=600, explore_max_us=2600))

    def test_storage_failure_preserves_valid_angle_candidate_live_state_and_original_file(self):
        before = self.state()
        points = [dict(angle_deg=0, pulse_us=500), dict(angle_deg=90, pulse_us=1500), dict(angle_deg=180, pulse_us=2500)]
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('fixture disk denied')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='save_angles', axis='pan', calibration_points=points))
        self.assertEqual(self.state(), before)

    def test_storage_failure_for_steps_preserves_tracking_and_pending_commands(self):
        before = self.state()
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('fixture disk denied')):
            with self.assertRaisesRegex(ValueError, '配置保存失败'):
                self.app.command(dict(action='manual_steps', axis='pan', fine_us=10, coarse_us=60))
        self.assertEqual(self.state(), before)


if __name__ == '__main__':
    unittest.main()
