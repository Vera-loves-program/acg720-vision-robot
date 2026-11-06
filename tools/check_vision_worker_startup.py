"""Run the real vision loop through its first published frame, without devices.

The camera, image arrays, detector and OpenCV codec are in-memory stand-ins;
GimbalApp, configuration commands, VisionWorker._run and publish are actual
product code. The negative control changes ONLY the center_band assignment
back to the failing pre-fix expression, not the surrounding vision pipeline.
"""
import ast
from copy import deepcopy
import hashlib
import inspect
import json
from pathlib import Path
import sys
import tempfile
import textwrap
import time
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
import usb_gimbal
from gimbal_control import MockPWMBackend, default_config
from target_tracking import TrackerConfig
from usb_camera import CameraFrame
from usb_gimbal import GimbalApp, VisionWorker


class ImageMock:
    shape = (480, 640, 3)

    def copy(self):
        return ImageMock()


def legacy_center_band_run():
    """Counterfactual actual worker; restore exactly the old unsafe read."""
    tree = ast.parse(textwrap.dedent(inspect.getsource(VisionWorker._run)))
    replaced = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == 'center_band'
                for target in node.targets):
            node.value = ast.parse('self.app.tracker.config.deadband', mode='eval').body
            replaced += 1
    if replaced != 1:
        raise AssertionError(f'Expected one actual center_band assignment, found {replaced}')
    ast.fix_missing_locations(tree)
    namespace = dict(vars(usb_gimbal))
    exec(compile(tree, '<actual VisionWorker with legacy center_band read>', 'exec'), namespace)
    return namespace['_run']


class VisionStartupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='vision-startup-', dir=ROOT / 'tools')
        self.addCleanup(self.temporary.cleanup)
        self.config_path = Path(self.temporary.name) / 'gimbal.json'
        self.config_path.write_text(json.dumps(default_config()), encoding='utf-8')
        self.now = 1000.
        clock = SimpleNamespace(monotonic=lambda: self.now, perf_counter=time.perf_counter)
        patcher = mock.patch.object(usb_gimbal, 'time', clock)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.backend = MockPWMBackend()
        self.app = GimbalApp(self.config_path, backend=self.backend,
                             dataset=Path(self.temporary.name) / 'photos')
        self.addCleanup(self.app.close)
        self.sequence = 0

    def run_first_frame(self, *, legacy=False, detection=False):
        self.app.stop.clear()
        self.sequence += 1
        raw = ImageMock()
        frame = CameraFrame(self.sequence, self.now, raw, self.app.gimbal.snapshot())
        visits = []

        def newest(*, after):
            visits.append(after)
            if len(visits) == 1:
                return frame
            # Stop the synchronous actual loop once it requests the next frame.
            # This is a camera fixture boundary, not a stubbed publish method.
            self.app.stop.set()
            return None

        camera = SimpleNamespace(newest=newest, error=None)
        cv = SimpleNamespace(
            rectangle=mock.Mock(), drawMarker=mock.Mock(), putText=mock.Mock(),
            imencode=mock.Mock(return_value=(True, SimpleNamespace(tobytes=lambda: b'encoded-preview'))),
            FONT_HERSHEY_SIMPLEX=0, MARKER_CROSS=0, IMWRITE_JPEG_QUALITY=1)
        np = SimpleNamespace(array=mock.Mock(), float64='mock-float64')
        detector = mock.Mock()
        detector.status.return_value = dict(state='enabled', dog=True, face=True, error=None)
        detector.predict.return_value = ([SimpleNamespace(x0=40, y0=50, x1=80, y1=90, confidence=.9)], [], 2., 0.)
        worker = VisionWorker(self.app, camera, SimpleNamespace(no_detection=not detection))
        events_before = deepcopy(self.backend.events)
        with mock.patch.dict(sys.modules, cv2=cv, numpy=np), \
                mock.patch.object(usb_gimbal, 'IndependentDetectors', return_value=detector) as engines:
            if legacy:
                legacy_center_band_run()(worker)
            else:
                worker._run()
        self.assertEqual(self.backend.enabled, {}, 'Preview must never enable a PWM output')
        self.assertFalse(any(axis['armed'] for axis in self.app.gimbal.snapshot()['axes'].values()))
        # On success no driver operations at all are necessary for a preview.
        if self.app.error is None:
            self.assertEqual(self.backend.events, events_before)
            self.assertEqual(len(visits), 2, 'The actual loop must finish encoding and request another frame')
            self.assertEqual(engines.call_count, int(detection))
        return frame, cv, detector

    def assert_published(self, frame, cv, deadband):
        self.assertIsNone(self.app.error)
        self.assertIsNotNone(self.app.latest, 'Actual publish must establish the first preview')
        self.assertEqual(self.app.latest.sequence, frame.sequence)
        self.assertIs(self.app.latest.image, frame.image, 'Raw camera pixels must remain the source image')
        self.assertEqual(self.app.latest.jpeg, b'encoded-preview')
        self.assertEqual(self.app.latest.coordinates, 'original USB pixels')
        self.assertEqual((self.app.latest.width, self.app.latest.height), (640, 480))
        self.assertIsNotNone(self.app.tracker, 'The real publish path must initialize the tracker')
        self.assertAlmostEqual(self.app.tracker.config.deadband, deadband)
        band = [call for call in cv.rectangle.call_args_list if call.args[3] == (0, 180, 180)]
        self.assertEqual(len(band), 1)
        bx, by = round(320 * deadband), round(240 * deadband)
        self.assertEqual(band[0].args[1:3], ((320 - bx, 240 - by), (320 + bx, 240 + by)))
        cv.imencode.assert_called_once()

    def test_cold_start_without_detection_publishes_default_preview(self):
        self.assertIsNone(self.app.tracker)
        frame, cv, _ = self.run_first_frame()
        self.assert_published(frame, cv, TrackerConfig().deadband)

    def test_cold_start_with_detection_publishes_box_and_preview(self):
        self.assertIsNone(self.app.tracker)
        frame, cv, detector = self.run_first_frame(detection=True)
        self.assert_published(frame, cv, TrackerConfig().deadband)
        self.assertEqual(self.app.latest.detections[0]['kind'], 'dog')
        detector.predict.assert_called_once_with(frame.image, kind=None)
        self.assertEqual(self.app.detection_status['state'], 'enabled')

    def test_real_config_reset_then_first_frame_reinitializes_tracker(self):
        frame, cv, _ = self.run_first_frame()
        self.assert_published(frame, cv, TrackerConfig().deadband)
        candidate = deepcopy(self.app.config)
        candidate['camera'] = dict(width=640, height=480, fx=400, fy=400, cx=320, cy=240)
        self.app.command(dict(action='save_config', config=candidate))
        self.assertIsNone(self.app.tracker, 'The real configuration API must exercise its reset path')
        generation = self.app.config_generation
        frame, cv, _ = self.run_first_frame()
        self.assert_published(frame, cv, TrackerConfig().deadband)
        self.assertEqual(self.app.config_generation, generation)
        self.assertEqual(self.app.tracker.camera.model, 'intrinsics')

    def test_custom_deadband_saved_before_first_frame_is_used_by_preview(self):
        self.app.command(dict(action='save_tracking', tracking=dict(
            deadband=.08, normalized_gain_dps=50, gain=2, max_rate_dps=20,
            max_accel_dps2=180, filter_time_constant_s=.06, deadband_hysteresis=.015,
            min_rate_dps=0, max_control_dt_s=.1)))
        self.assertEqual(json.loads(self.config_path.read_text(encoding='utf-8'))['tracking']['deadband'], .08)
        # A real config reset removes the cached tracker but preserves the
        # newly saved setting; preview must not silently revert to its default.
        self.app.command(dict(action='save_config', config=deepcopy(self.app.config)))
        self.assertIsNone(self.app.tracker)
        frame, cv, _ = self.run_first_frame()
        self.assert_published(frame, cv, .08)

    def test_legacy_negative_control_cold_start_reproduces_none_config_black_screen(self):
        self.assertIsNone(self.app.tracker)
        _, cv, _ = self.run_first_frame(legacy=True)
        self.assertIsNone(self.app.latest)
        self.assertIn("'NoneType' object has no attribute 'config'", self.app.error or '')
        cv.imencode.assert_not_called()

    def test_legacy_negative_control_after_config_reset_reproduces_black_screen(self):
        # Publish directly only to establish the prerequisite live tracker;
        # the frame under test still runs the entire actual vision pipeline.
        self.app.publish(usb_gimbal.VisionFrame(0, self.now, ImageMock(), b'old-preview', [],
            self.app.gimbal.snapshot(), 640, 480, 0., 'original USB pixels'))
        self.app.command(dict(action='save_config', config=deepcopy(self.app.config)))
        self.assertIsNone(self.app.tracker)
        before = self.app.latest
        _, cv, _ = self.run_first_frame(legacy=True)
        self.assertIs(self.app.latest, before, 'No new image may publish under the old failing read')
        self.assertIn("'NoneType' object has no attribute 'config'", self.app.error or '')
        cv.imencode.assert_not_called()


if __name__ == '__main__':
    if '--legacy-negative-only' in sys.argv:
        sys.argv.remove('--legacy-negative-only')
        sys.argv.extend([
            'VisionStartupTests.test_legacy_negative_control_cold_start_reproduces_none_config_black_screen',
            'VisionStartupTests.test_legacy_negative_control_after_config_reset_reproduces_black_screen'])
    print('Actual worker source SHA256: ' + hashlib.sha256(Path(usb_gimbal.__file__).read_bytes()).hexdigest(), flush=True)
    unittest.main(verbosity=2)
