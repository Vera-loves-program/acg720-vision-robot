"""App/API regressions for manual image-motion sampling and persistence.

Uses published frames with capture-time PWM snapshots and genuine manual PWM
commands against MockPWMBackend. Synthetic target motion is produced by an
independent, small image-motion plant. HTTP checks use the real handler with
an in-memory connection; no device, socket, image library or HDL tool is used.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_usb_gimbal as fixtures
import usb_gimbal
from gimbal_control import MockPWMBackend


class CalibrationFlowTests(fixtures.AppFixture):
    matrix = [[-.020, -.002], [.001, -.026]]

    def setUp(self):
        super().setUp()
        config = deepcopy(self.app.config)
        config['camera'] = dict(width=640, height=480)  # no fake intrinsics
        config['mount'] = dict(upright_tilt_deg=180)
        self.app.command(dict(action='save_config', config=config))
        self.app.gimbal._clock = lambda: self.now
        self.arm()
        self.sequence = 100

    def api(self, action, **options):
        body = json.dumps(dict(action=action, **options)).encode('utf8')
        status, _, payload = fixtures.HttpIntegrationTests.memory_request(
            self, 'POST', body, None, '/command')
        response = json.loads(payload)
        if status == 200:
            self.assertTrue(response['ok'])
            return status, response['result']
        return status, response

    def show(self, u=320., v=240., *, kind='human', width=640, height=480, arrived=None):
        self.sequence += 1
        frame = self.frame(sequence=self.sequence,
            boxes=[fixtures.detection(u, v, kind=kind)], arrived=arrived)
        frame.width, frame.height = width, height
        self.app.publish(frame)
        return self.sequence

    def move(self, pan_us=None, tilt_us=None):
        self.app.command(dict(action='heartbeat'))
        for axis, value in (('pan', pan_us), ('tilt', tilt_us)):
            if value is not None:
                self.app.command(dict(action='manual_pulse', axis=axis, pulse_us=value))
        for _ in range(500):
            states = self.app.gimbal.snapshot()['axes']
            if all(state['output_us'] == state['target_us'] for state in states.values()):
                return states
            self.now += .02
            self.app.gimbal.tick(self.now)
        self.fail('Synthetic PWM controller did not reach requested pulses')

    def begin(self, **kwargs):
        sequence = self.show(**kwargs)
        return self.app.command(dict(action='calibration_begin', sequence=sequence, index=0))

    def prepare_pair(self, pan_step=28, tilt_step=0):
        self.move(1500, 1500)
        self.begin()
        start = deepcopy(self.app.calibration_pending['start'])
        end = self.move(1500 + pan_step, 1500 + tilt_step)
        dpan = end['pan']['angle_deg'] - start['pan_deg']
        dtilt = end['tilt']['angle_deg'] - start['tilt_deg']
        pan_feature = math.cos(math.radians((end['tilt']['angle_deg'] + start['tilt_deg']) / 2)) * dpan
        du, dv = [row[0] * pan_feature + row[1] * dtilt for row in self.matrix]
        sequence = self.show(start['u'] + du * 320, start['v'] + dv * 240)
        return sequence

    def pair(self, pan_step=28, tilt_step=0):
        sequence = self.prepare_pair(pan_step, tilt_step)
        return self.app.command(dict(action='calibration_finish', sequence=sequence, index=0))

    def learn(self, *, apply=True):
        for pan, tilt in ((28, 0), (-28, 0), (0, 28), (0, -28)):
            result = self.pair(pan, tilt)
        self.assertTrue(result['fit']['ready'], result)
        self.assertEqual(result['fit']['diagnostics']['rank'], 2)
        if apply:
            result = self.app.command(dict(action='calibration_apply'))
            self.assertTrue(result['enabled'])
        return result

    def assert_preserved(self, *, disk=None, config=None, pending=None, outputs=None,
                         model=None, events=None):
        if disk is not None:
            self.assertEqual(self.path.read_bytes(), disk)
        if config is not None:
            self.assertEqual(self.app.config, config)
        if pending is not None:
            self.assertEqual(self.app.calibration_pending, pending)
        if outputs is not None:
            self.assertEqual(self.backend.enabled, outputs)
        if model is not None:
            self.assertEqual(self.app.image_calibration_model, model)
        if events is not None:
            self.assertEqual(self.backend.events, events)

    def test_real_http_records_visible_target_and_capture_pwm_without_device_restart(self):
        sequence = self.show(260, 210)
        before, outputs = list(self.backend.events), deepcopy(self.backend.enabled)
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 200, response)
        start = response['pending']['start']
        self.assertEqual((start['u'], start['v']), (260, 210))
        self.assertEqual((start['pan_us'], start['tilt_us']), (1500, 1500))
        self.assertEqual((start['pan_deg'], start['tilt_deg']), (0, 0))
        self.assert_preserved(events=before, outputs=outputs)
        self.move(1528, 1500)
        sequence = self.show(230, 211)
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 200, response)
        self.assertIsNone(response['pending'])
        self.assertEqual(response['sample_count'], 1)
        saved = json.loads(self.path.read_text(encoding='utf8'))
        self.assertEqual(saved, self.app.config)
        sample = saved['image_calibration']['samples'][0]
        self.assertEqual(sample['end']['u'], 230)
        self.assertEqual(sample['end']['pan_us'], 1528)
        self.assertEqual(sample['start'], start)

    def test_four_independent_samples_fit_apply_and_retain_both_axes(self):
        result = self.learn()
        self.assertEqual(result['sample_count'], 4)
        self.assertEqual(result['fit']['diagnostics']['effective_samples'], 4)
        for row, expected in zip(result['fit']['matrix_b'], self.matrix):
            for value, reference in zip(row, expected):
                self.assertAlmostEqual(value, reference, places=12)
        self.assertEqual(set(self.backend.enabled), {'pan', 'tilt'})
        self.assertEqual(self.app.config['axes'], self.config['axes'])
        self.assertEqual(json.loads(self.path.read_text(encoding='utf8')), self.app.config)

    def test_finish_without_start_and_same_capture_are_rejected(self):
        sequence = self.show()
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.app.command(dict(action='calibration_begin', sequence=sequence, index=0))
        pending = deepcopy(self.app.calibration_pending)
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIn('新画面', response['error'])
        self.assert_preserved(pending=pending)

    def test_a_second_start_cannot_overwrite_pending_group(self):
        self.begin()
        pending = deepcopy(self.app.calibration_pending)
        sequence = self.show(280, 200)
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assert_preserved(pending=pending)

    def test_wrong_target_kind_stale_frame_and_unknown_sequence_keep_pending(self):
        self.begin()
        self.move(1528, 1500)
        pending = deepcopy(self.app.calibration_pending)
        sequence = self.show(280, 240, kind='dog')
        for options in (dict(sequence=sequence, index=0), dict(sequence=999999, index=0),
                        dict(sequence=True, index=0), dict(sequence=sequence, index=True)):
            status, response = self.api('calibration_finish', **options)
            self.assertEqual(status, 400, response)
            self.assert_preserved(pending=pending)
        sequence = self.show(280, 240, arrived=self.now - 2)
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIn('过期', response['error'])
        self.assert_preserved(pending=pending)

    def test_disabled_at_capture_and_disabled_after_capture_cannot_start_sampling(self):
        self.app.command(dict(action='disable', axis='tilt'))
        sequence = self.show()
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.app.command(dict(action='manual_enable', axis='tilt', pulse_us=1500))
        sequence = self.show()
        self.app.command(dict(action='disable', axis='tilt'))
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIsNone(self.app.calibration_pending)

    def test_unsettled_at_capture_and_unsettled_after_capture_are_rejected(self):
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=1528))
        sequence = self.show()
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.move(1500, 1500)
        sequence = self.show()
        self.app.command(dict(action='manual_pulse', axis='pan', pulse_us=1528))
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIsNone(self.app.calibration_pending)

    def test_cached_frame_cannot_be_used_with_a_different_current_settled_output(self):
        sequence = self.show()
        self.move(1510, 1500)
        status, response = self.api('calibration_begin', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIsNone(self.app.calibration_pending)

    def test_resolution_change_mid_group_rejects_end_without_losing_start(self):
        self.begin()
        pending = deepcopy(self.app.calibration_pending)
        self.move(1528, 1500)
        sequence = self.show(600, 340, width=1280, height=720)
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIn('分辨率', response['error'])
        self.assert_preserved(pending=pending)

    def test_angle_table_change_mid_group_rejects_end_and_invalidates_old_model(self):
        self.learn()
        self.begin()
        pending = deepcopy(self.app.calibration_pending)
        points = deepcopy(self.app.config['axes']['pan']['calibration_points'])
        points[0]['angle_deg'] = -89
        self.app.command(dict(action='save_angles', axis='pan', calibration_points=points))
        self.move(1528, 1500)
        sequence = self.show(280, 240)
        status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 400, response)
        self.assertIn('角度表', response['error'])
        self.assert_preserved(pending=pending)
        state = self.app.snapshot()['image_calibration']
        self.assertFalse(state['fit']['ready'])
        self.assertFalse(state['enabled'])

    def test_resolution_change_invalidates_old_model_before_apply(self):
        self.learn(apply=False)
        self.show(600, 340, width=1280, height=720)
        state = self.app.snapshot()['image_calibration']
        self.assertFalse(state['fit']['ready'])
        status, response = self.api('calibration_apply')
        self.assertEqual(status, 400, response)
        self.assertIn('分辨率', response['error'])

    def test_insufficient_samples_and_single_axis_do_not_apply(self):
        self.pair()
        status, response = self.api('calibration_apply')
        self.assertEqual(status, 400, response)
        for step in (-28, 35, -35):
            self.pair(step, 0)
        self.assertEqual(self.app.snapshot()['image_calibration']['fit']['diagnostics']['rank'], 1)
        status, response = self.api('calibration_apply')
        self.assertEqual(status, 400, response)
        self.assertFalse(self.app.config['image_calibration']['enabled'])

    def test_failed_finish_save_preserves_pending_file_model_and_pwm(self):
        self.learn()
        sequence = self.prepare_pair(20, 0)
        before = dict(disk=self.path.read_bytes(), config=deepcopy(self.app.config),
                      pending=deepcopy(self.app.calibration_pending),
                      outputs=deepcopy(self.backend.enabled), model=deepcopy(self.app.image_calibration_model),
                      events=list(self.backend.events))
        with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('mock disk full')):
            status, response = self.api('calibration_finish', sequence=sequence, index=0)
        self.assertEqual(status, 503, response)
        self.assert_preserved(**before)
        result = self.app.command(dict(action='calibration_finish', sequence=sequence, index=0))
        self.assertEqual(result['sample_count'], 5)
        self.assertIsNone(result['pending'])

    def test_clear_cancel_and_mount_save_keep_outputs_and_measured_angles(self):
        self.learn()
        for action in ('calibration_cancel', 'save_mount', 'calibration_clear'):
            self.begin()
            axes, outputs = deepcopy(self.app.config['axes']), deepcopy(self.backend.enabled)
            events = list(self.backend.events)
            options = dict(upright_tilt_deg=0) if action == 'save_mount' else {}
            result = self.app.command(dict(action=action, **options))
            self.assertIsNone(self.app.calibration_pending)
            self.assertEqual(self.app.config['axes'], axes)
            self.assert_preserved(outputs=outputs, events=events)
            if action == 'calibration_clear':
                self.assertEqual(result['sample_count'], 0)
                self.assertFalse(result['enabled'])
            else:
                self.assertEqual(self.app.snapshot()['image_calibration']['sample_count'], 4)

    def test_failed_clear_or_mount_save_does_not_destroy_pending_or_model(self):
        self.learn()
        self.begin()
        before = dict(disk=self.path.read_bytes(), config=deepcopy(self.app.config),
                      pending=deepcopy(self.app.calibration_pending),
                      outputs=deepcopy(self.backend.enabled), model=deepcopy(self.app.image_calibration_model),
                      events=list(self.backend.events))
        for action, options in (('calibration_clear', {}), ('save_mount', dict(upright_tilt_deg=0))):
            with mock.patch.object(usb_gimbal, 'atomic_json', side_effect=OSError('mock read-only filesystem')):
                status, response = self.api(action, **options)
            self.assertEqual(status, 503, response)
            self.assert_preserved(**before)

    def test_restart_reconstructs_enabled_model_without_enabling_hardware(self):
        learned = self.learn()
        original_model = deepcopy(self.app.image_calibration_model)
        new_backend = MockPWMBackend()
        reopened = usb_gimbal.GimbalApp(self.path, backend=new_backend)
        self.addCleanup(reopened.close)
        state = reopened.snapshot()['image_calibration']
        self.assertTrue(state['enabled'])
        self.assertEqual(state['sample_count'], learned['sample_count'])
        self.assertEqual(reopened.image_calibration_model, original_model)
        self.assertEqual(new_backend.events, [])
        self.assertEqual(new_backend.enabled, {})
        self.assertEqual(reopened.config['axes'], self.app.config['axes'])

    def test_applied_image_model_drives_control_even_with_opposite_mount_metadata(self):
        self.learn()
        # The synthetic plant B learned negative responses in both directions.
        # Intentionally select the opposite analytic installation afterward:
        # learned joint/image motion must take precedence over that fallback.
        self.app.command(dict(action='save_mount', upright_tilt_deg=0))
        self.move(1500, 1500)
        sequence = self.show(250, 190)
        self.app.command(dict(action='select', sequence=sequence, index=0))
        self.app.command(dict(action='track', enabled=True))
        self.now += .1
        self.show(250, 190)
        self.app._control_once(self.now)
        result = self.app.tracking_result
        self.assertEqual(result['control_source'], 'sampled_image_jacobian')
        self.assertLess(result['pan_command_rate_dps'], 0)
        self.assertLess(result['tilt_command_rate_dps'], 0)
        self.assertLess(result['delta_pan_command_deg'], 0)
        self.assertLess(result['delta_tilt_command_deg'], 0)

    def test_incompatible_image_model_falls_back_to_mount_geometry_during_control(self):
        self.learn()
        self.move(1500, 1500)
        sequence = self.show(500, 280, width=1280, height=720)
        self.assertFalse(self.app.snapshot()['image_calibration']['enabled'])
        self.app.command(dict(action='select', sequence=sequence, index=0))
        self.app.command(dict(action='track', enabled=True))
        self.now += .1
        self.show(500, 280, width=1280, height=720)
        self.app._control_once(self.now)
        result = self.app.tracking_result
        self.assertEqual(result['control_source'], 'mount_geometry')
        self.assertLess(result['pan_command_rate_dps'], 0)
        self.assertLess(result['tilt_command_rate_dps'], 0)

    def test_apply_refuses_an_unfinished_new_group_without_losing_model(self):
        self.learn()
        self.begin()
        pending = deepcopy(self.app.calibration_pending)
        model = deepcopy(self.app.image_calibration_model)
        outputs = deepcopy(self.backend.enabled)
        status, response = self.api('calibration_apply')
        self.assertEqual(status, 400, response)
        self.assertIn('采样', response['error'])
        self.assert_preserved(pending=pending, model=model, outputs=outputs)

    def test_sampling_pauses_tracking_clears_old_selection_and_blocks_restart_until_done(self):
        sequence = self.show()
        self.app.command(dict(action='select', sequence=sequence, index=0))
        self.app.command(dict(action='track', enabled=True))
        before = deepcopy(self.backend.enabled)
        self.app.command(dict(action='calibration_begin', sequence=sequence, index=0))
        self.assertFalse(self.app.tracking)
        self.assertFalse(self.app.target_selected)
        self.assert_preserved(outputs=before)
        sequence = self.show()
        self.app.command(dict(action='select', sequence=sequence, index=0))
        self.assertFalse(self.app.snapshot()['tracking_preflight']['ready'])
        status, response = self.api('track', enabled=True)
        self.assertEqual(status, 400, response)
        self.assertIn('采样', response['error'])
        self.assertFalse(self.app.tracking)
        self.app.command(dict(action='calibration_cancel'))
        self.assertTrue(self.app.snapshot()['tracking_preflight']['ready'])
        self.assertTrue(self.app.command(dict(action='track', enabled=True))['tracking'])

    def test_disabling_an_axis_cancels_pending_without_rewriting_learned_samples(self):
        self.learn()
        self.begin()
        saved = deepcopy(self.app.config['image_calibration'])
        self.app.command(dict(action='disable', axis='tilt'))
        self.assertIsNone(self.app.calibration_pending)
        self.assertEqual(self.app.config['image_calibration'], saved)
        self.assertIn('pan', self.backend.enabled)
        self.assertNotIn('tilt', self.backend.enabled)


if __name__ == '__main__':
    unittest.main(verbosity=2)
