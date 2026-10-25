"""Independent synthetic checks for local, sampled gimbal image calibration.

No camera, PWM, network, image package, or HDL tool is used. A synthetic plant
generates image observations from an independently selected B, then applies
the learned controller to those observations. It is not a pose reconstruction.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pc'))
from gimbal_image_calibration import (calibrated_centering_delta, calibrated_centering_offset,
                                      fit_image_calibration, normalize_sample)


def sample(matrix, pan_delta, tilt_delta, *, tilt=45, noise=(0, 0), kind='human',
           width=640, height=480):
    # Synthetic image displacement is generated outside the implementation.
    pan_feature = math.cos(math.radians(tilt + tilt_delta / 2)) * pan_delta
    image_delta = [row[0] * pan_feature + row[1] * tilt_delta for row in matrix]
    start = dict(u=width / 2, v=height / 2, pan_deg=130., tilt_deg=tilt,
                 pan_us=1400., tilt_us=1800.)
    end = dict(u=start['u'] + image_delta[0] * width / 2 + noise[0],
               v=start['v'] + image_delta[1] * height / 2 + noise[1],
               pan_deg=start['pan_deg'] + pan_delta, tilt_deg=tilt + tilt_delta,
               pan_us=1400. + 10 * pan_delta, tilt_us=1800. - 10 * tilt_delta)
    return dict(width=width, height=height, start=start, end=end, kind=kind)


def observations(matrix, *, tilt=45, noise=0, seed=7):
    randomizer = random.Random(seed)
    return [sample(matrix, dp, dt, tilt=tilt,
                   noise=(randomizer.uniform(-noise, noise), randomizer.uniform(-noise, noise)))
            for dp, dt in ((6, 0), (-6, 0), (0, 6), (0, -6),
                           (4, 3), (-3, 4), (3, -4), (-4, -3))]


class ImageCalibrationChecks(unittest.TestCase):
    matrix = [[.020, .003], [-.002, .026]]

    def fit(self, matrix=None, **kwargs):
        result = fit_image_calibration(observations(matrix or self.matrix, **kwargs))
        self.assertTrue(result['valid'], result)
        return result

    def command(self, model, **kwargs):
        options = dict(tilt_deg=45, desired_x_rate=.2, desired_y_rate=.1,
                       dt=.1, width=640, height=480, max_rate_dps=25)
        options.update(kwargs)
        return calibrated_centering_delta(model, **options)

    def offset(self, model, **kwargs):
        options = dict(tilt_deg=45, desired_x=.2, desired_y=.1,
                       width=640, height=480, max_step_deg=30)
        options.update(kwargs)
        return calibrated_centering_offset(model, **options)

    def test_recovers_coupled_matrix_without_numerical_packages(self):
        model = self.fit()
        for actual, expected in zip(model['matrix_b'], self.matrix):
            for value, reference in zip(actual, expected):
                self.assertAlmostEqual(value, reference, places=12)
        self.assertEqual(model['diagnostics']['rank'], 2)
        self.assertEqual(model['diagnostics']['effective_samples'], 8)
        self.assertLess(model['diagnostics']['rms'], 1e-12)
        json.dumps(model, allow_nan=False)

    def test_four_samples_allow_two_axes_bidirectionally(self):
        model = fit_image_calibration(observations(self.matrix)[:4])
        self.assertTrue(model['valid'], model)
        self.assertEqual(model['diagnostics']['direction_coverage'],
                         dict(pan=dict(positive=1, negative=1),
                              tilt=dict(positive=1, negative=1)))

    def test_two_samples_are_not_enough_to_check_residual_consistency(self):
        model = fit_image_calibration([sample(self.matrix, 5, 0), sample(self.matrix, 0, 5)])
        self.assertFalse(model['valid'])
        self.assertEqual(model['diagnostics']['rank'], 2)
        self.assertIn('4', model['diagnostics']['reasons'][0])

    def test_different_tilt_poses_and_small_motion_across_zenith(self):
        values = []
        for pose in (15, 45, 80, 100, 135, 170):
            values.extend(observations(self.matrix, tilt=pose))
        values += [sample(self.matrix, 2, 8, tilt=86),
                   sample(self.matrix, -2, -8, tilt=94)]
        result = fit_image_calibration(values)
        self.assertTrue(result['valid'], result)
        for row, expected in zip(result['matrix_b'], self.matrix):
            for a, b in zip(row, expected):
                self.assertAlmostEqual(a, b, places=12)

    def test_upright_at_zero_and_upright_at_180_learn_opposite_signs(self):
        for direction in (1, -1):
            matrix = [[.02 * direction, 0], [0, .025 * direction]]
            model = self.fit(matrix)
            result = self.command(model, tilt_deg=45)
            self.assertEqual(math.copysign(1, result['pan_rate_dps']), direction)
            self.assertEqual(math.copysign(1, result['tilt_rate_dps']), direction)
            # The same learned installation reverses only the PAN effect
            # after crossing90deg; TILT remains a continuous local correction.
            result = self.command(model, tilt_deg=135)
            self.assertEqual(math.copysign(1, result['pan_rate_dps']), -direction)
            self.assertEqual(math.copysign(1, result['tilt_rate_dps']), direction)

    def test_stationary_target_converges_under_both_installations(self):
        for direction in (1, -1):
            matrix = [[.02 * direction, .002 * direction],
                      [-.001 * direction, .026 * direction]]
            model = self.fit(matrix, noise=.3)
            for initial_tilt in (35., 140.):
                tilt, x, y = initial_tilt, -.20, .15
                initial_error = math.hypot(x, y)
                for _ in range(120):
                    # Target image coordinates x/y are actual CURRENT positions
                    # relative to desired center. Controller consumes NEGATIVE
                    # error so a target on the left moves RIGHT toward center.
                    output = self.command(model, tilt_deg=tilt,
                        desired_x_rate=-1.5 * x, desired_y_rate=-1.5 * y)
                    dp, dt = output['delta_pan_deg'], output['delta_tilt_deg']
                    pan_feature = math.cos(math.radians(tilt + dt / 2)) * dp
                    x += matrix[0][0] * pan_feature + matrix[0][1] * dt
                    y += matrix[1][0] * pan_feature + matrix[1][1] * dt
                    tilt += dt
                self.assertLess(math.hypot(x, y), initial_error / 1000)

    def test_small_realistic_pixel_noise_preserves_fit_and_direction(self):
        model = self.fit(noise=1.0)
        self.assertLess(model['diagnostics']['rms'], .01)
        self.assertLess(model['diagnostics']['relative_rms'], .1)
        for row, expected in zip(model['matrix_b'], self.matrix):
            for a, b in zip(row, expected):
                self.assertLess(abs(a - b), .001)

    def test_single_axis_and_collinear_comotion_never_report_success(self):
        single = [sample(self.matrix, delta, 0) for delta in (3, -3, 5, -5)]
        same_direction = [sample(self.matrix, 5 * scale, 2 * scale,
                          tilt=45 - scale) for scale in (1, 2, 3, 4)]
        # Set mean tilt to45 for each row, making the two design columns exactly
        # proportional. Many samples of one diagonal do not learn two axes.
        for supplied in (single, same_direction):
            result = fit_image_calibration(supplied)
            self.assertFalse(result['valid'])
            self.assertEqual(result['diagnostics']['rank'], 1)

    def test_inadequate_axis_excitation_reports_condition(self):
        supplied = [sample(self.matrix, d, 0) for d in (15, -15)]
        supplied += [sample(self.matrix, 0, d) for d in (.4, -.4)]
        # The small TILT sample still produces enough pixels but is too small
        # relative to the PAN samples for a well-conditioned fit.
        result = fit_image_calibration(supplied)
        self.assertFalse(result['valid'])
        self.assertGreater(result['diagnostics']['condition'], 20)

    def test_inconsistent_target_motion_rejects_model(self):
        supplied = observations(self.matrix)
        supplied[0]['end']['u'] += 160
        result = fit_image_calibration(supplied)
        self.assertFalse(result['valid'])
        self.assertGreater(result['diagnostics']['relative_rms'], .35)

    def test_image_motion_rank_collapse_is_invalid_even_with_two_joint_axes(self):
        matrix = [[.02, .03], [.02, .03]]
        result = fit_image_calibration(observations(matrix))
        self.assertFalse(result['valid'])
        self.assertEqual(result['diagnostics']['rank'], 2)
        self.assertIsNone(result['diagnostics']['jacobian_condition'])

    def test_geometry_then_joint_rate_limits_and_bounded_integration(self):
        model = self.fit()
        output = self.command(model, desired_x_rate=100, desired_y_rate=-100,
                              max_rate_dps=3, dt=5)
        for axis in ('pan', 'tilt'):
            self.assertLessEqual(abs(output[axis + '_rate_dps']), 3)
            self.assertLessEqual(abs(output['delta_' + axis + '_deg']), .6 + 1e-12)
        self.assertEqual(output['control_dt_s'], .2)

    def test_centered_target_and_zero_dt_or_rate_limit_never_moves(self):
        model = self.fit()
        output = self.command(model, desired_x_rate=0, desired_y_rate=0)
        self.assertEqual(output['delta_pan_deg'], 0)
        self.assertEqual(output['delta_tilt_deg'], 0)
        output = self.command(model, dt=0)
        self.assertEqual(output['delta_pan_deg'], 0)
        self.assertEqual(output['delta_tilt_deg'], 0)
        output = self.command(model, max_rate_dps=0)
        self.assertEqual(output['delta_pan_deg'], 0)
        self.assertEqual(output['delta_tilt_deg'], 0)

    def test_zenith_has_no_pan_authority_and_crossing_is_finite(self):
        model = self.fit()
        center = self.command(model, tilt_deg=90)
        self.assertTrue(center['near_vertical'])
        self.assertEqual(center['pan_rate_dps'], 0)
        before = self.command(model, tilt_deg=89.9)
        after = self.command(model, tilt_deg=90.1)
        self.assertLess(before['pan_rate_dps'] * after['pan_rate_dps'], 0)
        self.assertLess(abs(before['pan_rate_dps']), 2)
        self.assertLess(abs(after['pan_rate_dps']), 2)
        for value in (*before['matrix_j'][0], *before['matrix_j'][1]):
            self.assertTrue(math.isfinite(value))

    def test_static_offset_uses_full_displacement_without_assumed_frame_duration(self):
        model = self.fit()
        output = self.offset(model)
        self.assertEqual(output['source'], 'sampled_image_jacobian')
        self.assertFalse(output['limited'])
        self.assertNotIn('control_dt_s', output)
        self.assertNotIn('pan_rate_dps', output)
        self.assertAlmostEqual(output['delta_pan_deg'], 12.743263802068574, places=12)
        self.assertAlmostEqual(output['delta_tilt_deg'], 4.521085804739272, places=12)
        # The local plant predicts the whole requested image displacement,
        # rather than one .1/.2second fragment of a velocity command.
        predicted = [row[0] * output['delta_pan_deg'] + row[1] * output['delta_tilt_deg']
                     for row in output['matrix_j']]
        self.assertLess(math.hypot(predicted[0] - .2, predicted[1] - .1), .01)
        velocity = self.command(model, dt=.1)
        self.assertAlmostEqual(output['delta_pan_deg'], 10 * velocity['delta_pan_deg'])
        self.assertAlmostEqual(output['delta_tilt_deg'], 10 * velocity['delta_tilt_deg'])
        json.dumps(output, allow_nan=False)

    def test_static_offset_preserves_learned_mount_signs_and_zenith_damping(self):
        for direction in (1, -1):
            model = self.fit([[.02 * direction, 0], [0, .025 * direction]])
            before = self.offset(model, tilt_deg=45)
            after = self.offset(model, tilt_deg=135)
            self.assertEqual(math.copysign(1, before['delta_pan_deg']), direction)
            self.assertEqual(math.copysign(1, after['delta_pan_deg']), -direction)
            self.assertEqual(math.copysign(1, after['delta_tilt_deg']), direction)
            self.assertAlmostEqual(before['delta_tilt_deg'], after['delta_tilt_deg'])
            zenith = self.offset(model, tilt_deg=90)
            self.assertEqual(zenith['delta_pan_deg'], 0)
            self.assertTrue(zenith['near_vertical'])
            near_before = self.offset(model, tilt_deg=89.9)
            near_after = self.offset(model, tilt_deg=90.1)
            self.assertLess(near_before['delta_pan_deg'] * near_after['delta_pan_deg'], 0)
            self.assertLess(abs(near_before['delta_pan_deg']), 2)
            for result in (before, after, zenith, near_before, near_after):
                json.dumps(result, allow_nan=False)

    def test_static_offset_is_clamped_per_axis_even_for_a_larger_requested_span(self):
        model = self.fit()
        for requested, actual in ((5, 5), (30, 30), (500, 30), (0, 0)):
            result = self.offset(model, desired_x=100, desired_y=-100, max_step_deg=requested)
            self.assertTrue(result['limited'])
            self.assertEqual(result['limited_axes'], ['pan', 'tilt'])
            self.assertEqual(result['max_step_deg'], actual)
            self.assertEqual(result['delta_pan_deg'], actual)
            self.assertEqual(result['delta_tilt_deg'], -actual)
        centered = self.offset(model, desired_x=0, desired_y=0)
        self.assertEqual(centered['delta_pan_deg'], 0)
        self.assertEqual(centered['delta_tilt_deg'], 0)
        self.assertFalse(centered['limited'])

    def test_static_offset_reuses_all_model_quality_and_resolution_checks(self):
        model = self.fit()
        invalid = [None, {}, dict(model, valid=False), dict(model, schema_version=100),
                   dict(model, matrix_b=[[0, 0], [0, 0]]),
                   dict(model, matrix_b=[[1], [2, 3]]),
                   dict(model, matrix_b=[[math.nan, 0], [0, 1]]),
                   dict(model, matrix_b=[[1, 0], [0, 1]], diagnostics={})]
        for field, value in (('rms', .2), ('rms', math.nan), ('relative_rms', .8),
                             ('condition', 100), ('effective_samples', 2)):
            changed = deepcopy(model)
            changed['diagnostics'][field] = value
            invalid.append(changed)
        for candidate in invalid:
            self.assertIsNone(self.offset(candidate))
            self.assertIsNone(self.command(candidate))
        self.assertIsNone(self.offset(model, width=1280, height=720))

    def test_static_offset_rejects_invalid_arguments_without_changing_model(self):
        model = self.fit()
        saved = deepcopy(model)
        for field, value in (('tilt_deg', math.nan), ('tilt_deg', True),
                             ('desired_x', math.inf), ('desired_y', '1'),
                             ('max_step_deg', -1), ('max_step_deg', math.nan)):
            with self.assertRaises(ValueError):
                self.offset(model, **{field: value})
        self.assertEqual(model, saved)

    def test_shared_solver_preserves_existing_rate_controller_numeric_results(self):
        # Recorded before extracting the shared quality/DLS helpers. These
        # cover the coupled matrix, inverted pose, zenith, duration and rate caps.
        cases = ((45, .2, .1, .1, 25, 12.743263802068574, 4.521085804739272, .1),
                 (135, -.3, .4, .2, 7, 7., 7., .2),
                 (90, .2, .1, .1, 25, 0., 4.625280046252797, .1),
                 (89.9, .2, .1, 5, 25, .9586112821675196, 4.625260699961416, .2),
                 (0, 100, -100, .1, 3, 3., -3., .1))
        model = self.fit()
        for tilt, x, y, dt, limit, pan, pitch, duration in cases:
            result = self.command(model, tilt_deg=tilt, desired_x_rate=x,
                                  desired_y_rate=y, dt=dt, max_rate_dps=limit)
            self.assertEqual(result['pan_rate_dps'], pan)
            self.assertEqual(result['tilt_rate_dps'], pitch)
            self.assertEqual(result['delta_pan_deg'], pan * duration)
            self.assertEqual(result['delta_tilt_deg'], pitch * duration)
            self.assertEqual(result['control_dt_s'], duration)

    def test_resolution_change_invalidates_sample_set_and_active_model(self):
        supplied = observations(self.matrix)
        supplied += [sample(self.matrix, 5, 0, width=1280, height=720)]
        result = fit_image_calibration(supplied)
        self.assertFalse(result['valid'])
        self.assertIsNone(self.command(self.fit(), width=1280, height=720))

    def test_invalid_stored_models_fall_back_to_geometry(self):
        model = self.fit()
        bad = [None, {}, dict(model, valid=False), dict(model, schema_version=100),
               dict(model, matrix_b=[[0, 0], [0, 0]]),
               dict(model, matrix_b=[[1], [2, 3]]),
               dict(model, matrix_b=[[math.nan, 0], [0, 1]]),
               dict(model, matrix_b=[[1, 0], [0, 1]], diagnostics={})]
        for candidate in bad:
            self.assertIsNone(self.command(candidate), candidate)

    def test_inconsistent_stored_quality_is_not_reenabled_by_valid_flag(self):
        model = self.fit()
        for field, value in (('rms', .2), ('rms', math.nan),
                             ('relative_rms', .8), ('condition', 100),
                             ('effective_samples', 2)):
            changed = deepcopy(model)
            changed['diagnostics'][field] = value
            self.assertIsNone(self.command(changed))

    def test_invalid_and_ineffective_samples_are_explained(self):
        baseline = sample(self.matrix, 5, 0)
        variants = []
        for name, value in (('pan_deg', math.nan), ('tilt_deg', True), ('u', -1),
                            ('v', 999), ('pan_us', 0), ('tilt_us', 20000)):
            candidate = deepcopy(baseline)
            candidate['end'][name] = value
            variants.append(candidate)
        candidate = deepcopy(baseline)
        candidate['width'] = True
        variants.append(candidate)
        candidate = deepcopy(baseline)
        candidate['kind'] = 'unknown'
        variants.append(candidate)
        variants += [sample(self.matrix, .001, 0), sample(self.matrix, 31, 0),
                     sample(self.matrix, 5, 0, tilt=90)]
        candidate = sample(self.matrix, 5, 0)
        candidate['end']['u'] = candidate['start']['u'] + .1
        candidate['end']['v'] = candidate['start']['v']
        variants.append(candidate)
        for candidate in variants:
            with self.assertRaises(ValueError):
                normalize_sample(candidate)
        result = fit_image_calibration(observations(self.matrix) + variants)
        self.assertTrue(result['valid'])
        self.assertEqual(len(result['diagnostics']['rejected_samples']), len(variants))

    def test_samples_are_copied_and_only_latest64_are_retained(self):
        baseline = sample(self.matrix, 5, 0)
        normalized = normalize_sample(baseline)
        normalized['end']['u'] = -100
        self.assertNotEqual(normalized['end']['u'], baseline['end']['u'])
        supplied = observations(self.matrix) * 12
        model = fit_image_calibration(supplied)
        self.assertTrue(model['valid'])
        self.assertEqual(model['diagnostics']['effective_samples'], 64)
        self.assertEqual(model['diagnostics']['truncated_samples'], 32)


if __name__ == '__main__':
    unittest.main(verbosity=2)
