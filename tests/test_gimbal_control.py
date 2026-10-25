"""Calibration/state tests and fake sysfs only; no GPIO/hardware is touched."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from pathlib import Path
import errno
import json
import tempfile
import unittest
from unittest.mock import patch

from pc.gimbal_control import (AxisConfig, GimbalController, GimbalError,
                               MockPWMBackend, PERIOD_NS, SysfsPWMBackend,
                               default_config, validate_config)


def configured(calibrated=False):
    config = default_config()
    for name in ("pan", "tilt"):
        axis = config["axes"][name]
        axis.update(safe_min_us=1400, safe_max_us=1600,
                    slew_us_per_s=100, fine_us=5, coarse_us=50)
        if calibrated:
            axis.update(calibration_confirmed=True, calibration_points=[
                {"angle_deg": -20, "pulse_us": 1400},
                {"angle_deg": 0, "pulse_us": 1500},
                {"angle_deg": 30, "pulse_us": 1600}])
    return config


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class ConfigTests(unittest.TestCase):
    def test_defaults_have_no_guessed_safe_range_or_calibration(self):
        config = default_config()
        self.assertFalse(validate_config(config)["pan"].configured)
        config["axes"]["pan"]["fine_us"] = 12
        self.assertEqual(config["axes"]["tilt"]["fine_us"], 5)
        self.assertEqual(default_config()["axes"]["pan"]["fine_us"], 5)

    def test_legacy_configs_keep_exploration_defaults_and_new_bounds_round_trip(self):
        old = default_config()['axes']['pan']
        old.pop('explore_min_us')
        old.pop('explore_max_us')
        parsed = AxisConfig.parse(old)
        self.assertEqual((parsed.explore_min_us, parsed.explore_max_us), (500, 2500))
        self.assertFalse(parsed.configured)
        extended = dict(old, explore_min_us=400, explore_max_us=4000,
            safe_min_us=500, safe_max_us=3500, calibration_confirmed=True,
            calibration_points=[dict(angle_deg=0, pulse_us=500), dict(angle_deg=300, pulse_us=3500)])
        # JSON persistence uses explicit measured samples, not a guessed servo
        # angle for the newly widened software waveform range.
        restored = AxisConfig.parse(json.loads(json.dumps(extended)))
        self.assertEqual(restored.angle_to_pulse(300), 3500)
        self.assertEqual(restored.angle_to_pulse(150), 2000)
        self.assertEqual(restored.pulse_to_angle(3500), 300)

    def test_exploration_bounds_require_an_ordered_integer_window_inside_period(self):
        original = default_config()['axes']['pan']
        for low, high in ((0, 2500), (-1, 2500), (500, 20_000), (500, 20_001),
                          (2500, 2500), (2501, 2500), (True, 2500), (500.5, 2500), (None, 2500)):
            with self.subTest(low=low, high=high), self.assertRaises(GimbalError):
                AxisConfig.parse(dict(original, explore_min_us=low, explore_max_us=high))
        largest = AxisConfig.parse(dict(original, explore_min_us=1, explore_max_us=19_999))
        self.assertEqual((largest.manual_clamp(-1), largest.manual_clamp(30_000)), (1, 19_999))
        with self.assertRaises(GimbalError):
            AxisConfig.parse(dict(original, safe_min_us=499, safe_max_us=2000))
        with self.assertRaises(GimbalError):
            AxisConfig.parse(dict(original, safe_min_us=1000, safe_max_us=3500))

    def test_interpolation_is_measured_piecewise_and_invertible(self):
        axis = validate_config(configured(True))["pan"]
        self.assertEqual(axis.angle_to_pulse(-10), 1450)
        self.assertEqual(axis.angle_to_pulse(15), 1550)
        self.assertAlmostEqual(axis.pulse_to_angle(1550), 15)
        self.assertIsNone(axis.pulse_to_angle(1700))
        with self.assertRaises(GimbalError):
            axis.angle_to_pulse(360)
        data = deepcopy(configured(True)["axes"]["pan"])
        for point in data["calibration_points"]:
            point["pulse_us"] = 3000 - point["pulse_us"]
        reverse = AxisConfig.parse(data)
        self.assertEqual(reverse.angle_to_pulse(-10), 1550)
        self.assertAlmostEqual(reverse.pulse_to_angle(1550), -10)

    def test_rejects_nonmonotonic_or_unmeasured_or_unsafe_config(self):
        original = configured(True)["axes"]["pan"]
        mutations = [
            {"safe_min_us": 499}, {"safe_max_us": 2501},
            {"safe_min_us": 1600}, {"safe_min_us": None},
            {"slew_us_per_s": float("nan")}, {"fine_us": 0},
            {"coarse_us": 0}, {"calibration_confirmed": "yes"},
            {"calibration_points": []},
            {"calibration_points": [{"angle_deg": 0, "pulse_us": 1300}]},
            {"calibration_points": [{"angle_deg": 0, "pulse_us": 1400},
                                    {"angle_deg": 0, "pulse_us": 1500}]},
            {"calibration_points": [{"angle_deg": 0, "pulse_us": 1400},
                                    {"angle_deg": 1, "pulse_us": 1600},
                                    {"angle_deg": 2, "pulse_us": 1500}]},
            {"calibration_points": [{"angle_deg": -1e308, "pulse_us": 1400},
                                    {"angle_deg": 1e308, "pulse_us": 1600}]},
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation), self.assertRaises(GimbalError):
                AxisConfig.parse({**original, **mutation})
        with self.assertRaises(GimbalError):
            validate_config({"axes": {}})
        self.assertIn("pan", validate_config({**configured(), "camera": {"index": 3}}))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.backend = MockPWMBackend()
        self.controller = GimbalController(configured(), self.backend, clock=self.clock)
        self.addCleanup(self.controller.close)

    def test_init_and_default_mock_never_enable(self):
        self.assertEqual(self.backend.events, [])
        self.assertFalse(self.controller.snapshot()["axes"]["pan"]["armed"])
        with GimbalController(default_config()) as other:
            self.assertIsInstance(other.backend, MockPWMBackend)
            with self.assertRaises(GimbalError):
                other.arm("pan", 1500, manual_calibration=True)
            self.assertEqual(other.backend.events, [])

    def test_uncalibrated_requires_explicit_arm_and_every_absolute_request(self):
        for args in ({}, {"manual_calibration": False}):
            with self.assertRaises(GimbalError):
                self.controller.arm("pan", 1500, **args)
        self.controller.arm("pan", 1500, manual_calibration=True)
        with self.assertRaises(GimbalError):
            self.controller.set_pulse("pan", 1550)
        self.controller.set_pulse("pan", 1700, manual_calibration=True)
        self.assertEqual(self.controller.snapshot()["axes"]["pan"]["target_us"], 1600)
        with self.assertRaises(GimbalError):
            self.controller.set_angle("pan", 0, tracking=True)
        with self.assertRaises(GimbalError):
            self.controller.arm("pan", 1500, manual_calibration=True)

    def test_jog_coarse_fine_clamps_and_tick_slews_with_no_time_replay(self):
        self.controller.arm("pan", 1500, manual_calibration=True)
        self.controller.jog("pan", 1)
        self.assertEqual(self.controller.snapshot()["axes"]["pan"]["target_us"], 1505)
        self.controller.jog("pan", 1, coarse=True)
        self.assertEqual(self.controller.snapshot()["axes"]["pan"]["target_us"], 1555)
        self.clock.advance(0.02)
        self.controller.tick()
        self.assertEqual(self.backend.enabled["pan"], 1502)
        for _ in range(10):
            self.controller.tick()
        self.assertEqual(self.backend.enabled["pan"], 1502)
        self.clock.advance(10)
        self.controller.tick()
        self.assertEqual(self.backend.enabled["pan"], 1512)  #100ms catch-up cap.
        for _ in range(20):
            self.controller.jog("pan", -1, coarse=True)
        self.assertEqual(self.controller.snapshot()["axes"]["pan"]["target_us"], 1400)
        with self.assertRaises(GimbalError):
            self.controller.jog("pan", 0)

    def test_hold_cancels_pending_tracking_or_jog_target(self):
        self.controller.arm("pan", 1500, manual_calibration=True)
        self.controller.jog("pan", 1, coarse=True)
        self.clock.advance(0.02)
        self.controller.tick()
        self.controller.hold()
        self.clock.advance(0.1)
        self.controller.tick()
        self.assertEqual(self.backend.enabled["pan"], 1502)
        self.assertEqual(self.controller.snapshot()["axes"]["pan"]["target_us"], 1502)

    def test_confirmed_calibration_required_on_both_tracking_axes(self):
        config = configured(True)
        config["axes"]["tilt"]["calibration_confirmed"] = False
        with GimbalController(config, clock=self.clock) as control:
            control.arm("pan", 1500)
            with self.assertRaises(GimbalError):
                control.set_angle("pan", 15, tracking=True)
            control.set_angle("pan", 15)
            self.assertEqual(control.snapshot()["axes"]["pan"]["target_us"], 1550)
        with GimbalController(configured(True), clock=self.clock) as control:
            control.arm("pan", 1500)
            control.arm("tilt", 1500)
            control.set_angle("pan", 15, tracking=True)
            self.clock.advance(0.02)
            control.tick()
            self.assertAlmostEqual(control.snapshot()["axes"]["pan"]["angle_deg"], 0.6)

    def test_parallel_commands_and_disable_close_are_serialized(self):
        self.controller.arm("pan", 1500, manual_calibration=True)
        def jog(_):
            self.controller.jog("pan", 1 if _ % 2 else -1)
            return self.controller.snapshot()
        with ThreadPoolExecutor(max_workers=8) as workers:
            results = list(workers.map(jog, range(400)))
        self.assertTrue(all(1400 <= state["axes"]["pan"]["target_us"] <= 1600 for state in results))
        self.controller.disable()
        self.assertEqual(self.backend.enabled, {})
        with self.assertRaises(GimbalError):
            self.controller.jog("pan", 1)
        self.controller.close()
        self.controller.close()
        with self.assertRaises(GimbalError):
            self.controller.arm("pan", 1500, manual_calibration=True)

    def test_write_failure_disables_both_axes(self):
        self.controller.arm("pan", 1500, manual_calibration=True)
        self.controller.arm("tilt", 1500, manual_calibration=True)
        self.controller.jog("pan", 1, coarse=True)
        self.clock.advance(0.02)
        with patch.object(self.backend, "set_pulse", side_effect=OSError("lost PWM")):
            with self.assertRaises(GimbalError):
                self.controller.tick()
        self.assertEqual(self.backend.enabled, {})
        self.assertFalse(self.controller.snapshot()["axes"]["tilt"]["armed"])
        self.assertIn("lost PWM", self.controller.snapshot()["fault"])

    def test_manual_off_draft_and_explicit_enable_without_recorded_limits(self):
        with GimbalController(default_config(), self.backend, clock=self.clock) as controller:
            manual = controller.snapshot()['axes']['pan']['manual']
            self.assertEqual((manual['draft_us'], manual['fine_us'], manual['coarse_us']), (1500, 5, 50))
            controller.manual_jog('pan', 1, coarse=True)
            controller.manual_jog('pan', -1)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 1545)
            controller.manual_pulse('pan', 600)
            self.assertEqual(self.backend.events, [])
            self.assertFalse(controller.snapshot()['axes']['pan']['configured'])
            controller.manual_enable('pan')
            self.assertEqual(self.backend.enabled['pan'], 600)
            self.assertFalse(controller.snapshot()['axes']['pan']['configured'])
            controller.manual_jog('pan', 1, coarse=True)
            self.assertEqual(self.backend.enabled['pan'], 600)
            self.clock.advance(.02)
            controller.tick()
            self.assertEqual(self.backend.enabled['pan'], 603)
            controller.disable('pan')
            count = len(self.backend.events)
            controller.manual_pulse('pan', 800)
            controller.tick()
            self.assertEqual(len(self.backend.events), count)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 800)

    def test_manual_draft_uses_valid_stored_start_and_enforces_existing_limits(self):
        config = configured()
        config['axes']['pan']['start_us'] = 1420
        config['axes']['tilt']['start_us'] = 700
        with GimbalController(config, self.backend, clock=self.clock) as controller:
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 1420)
            self.assertEqual(controller.snapshot()['axes']['tilt']['manual']['draft_us'], 1500)
            controller.manual_pulse('pan', 600)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 1400)
            controller.manual_enable('pan')
            self.assertEqual(self.backend.enabled['pan'], 1400)
            controller.manual_enable('tilt')
            events = list(self.backend.events)
            controller.apply_axis_config('pan', {**config['axes']['pan'], 'safe_max_us': 1550})
            self.assertEqual(self.backend.events, events)
            self.assertEqual(self.backend.enabled, {'pan': 1400, 'tilt': 1500})
            controller.disable('pan')
            self.assertEqual(self.backend.enabled, {'tilt': 1500})

    def test_default_draft_already_respects_a_saved_window_without_output(self):
        config = default_config()
        config['axes']['pan'].update(safe_min_us=500, safe_max_us=800)
        with GimbalController(config, self.backend, clock=self.clock) as controller:
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 800)
            self.assertEqual(controller.snapshot()['axes']['tilt']['manual']['draft_us'], 1500)
            self.assertEqual(self.backend.events, [])
            controller.apply_axis_config('pan', {**config['axes']['pan'], 'safe_max_us': 700})
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 700)
            self.assertEqual(self.backend.events, [])
            controller.manual_enable('pan')
            self.assertEqual(self.backend.enabled['pan'], 700)

    def test_manual_mark_captures_settled_output_not_unsent_or_slewing_target(self):
        with self.assertRaises(GimbalError):
            self.controller.mark_limit('pan', 'min')
        self.controller.manual_enable('pan')
        self.controller.mark_limit('pan', 'min')
        self.controller.manual_jog('pan', 1, coarse=True)
        with self.assertRaises(GimbalError):
            self.controller.mark_limit('pan', 'max')
        self.assertIsNone(self.controller.snapshot()['axes']['pan']['manual']['pending_max_us'])
        for _ in range(6):
            self.clock.advance(.1)
            self.controller.tick()
        self.controller.mark_limit('pan', 'max')
        manual = self.controller.snapshot()['axes']['pan']['manual']
        self.assertEqual((manual['pending_min_us'], manual['pending_max_us']), (1500, 1550))
        self.assertEqual((manual['recorded_min_us'], manual['recorded_max_us']), (1400, 1600))

    def test_manual_enable_button_is_physical_authorization_and_failure_is_retryable(self):
        backend = MockPWMBackend()
        backend.physical = True  # Fake backend; this test never writes a device.
        with GimbalController(default_config(), backend, clock=self.clock) as controller:
            with patch.object(backend, 'start', side_effect=PermissionError('fake/pwmchip7/pwm0/enable')):
                with self.assertRaisesRegex(GimbalError, 'PWM启动失败.*fake/pwmchip7/pwm0/enable'):
                    controller.manual_enable('pan')
            self.assertEqual(backend.enabled, {})
            controller.manual_enable('pan')
            self.assertEqual(backend.enabled, {'pan': 1500})
            self.assertIsNone(controller.snapshot()['fault'])

    def test_pan_can_expand_above_2500_and_below_500_without_arming_on_edit(self):
        with GimbalController(default_config(), self.backend, clock=self.clock) as controller:
            controller.manual_pulse('pan', 2500)
            controller.manual_jog('pan', 1)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 2500)
            raw = {**controller.config['axes']['pan'], 'explore_max_us': 2600}
            controller.apply_axis_config('pan', raw)
            self.assertEqual(self.backend.events, [])
            controller.manual_jog('pan', 1)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 2505)
            controller.manual_pulse('pan', 2600)
            controller.manual_enable('pan')
            self.assertEqual(self.backend.enabled['pan'], 2600)
            controller.disable('pan')
            events = list(self.backend.events)
            raw.update(explore_min_us=400)
            controller.apply_axis_config('pan', raw)
            controller.manual_pulse('pan', 500)
            controller.manual_jog('pan', -1)
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 495)
            self.assertEqual(self.backend.events, events)
            bounds = controller.snapshot()['axes']
            self.assertEqual((bounds['pan']['manual']['numeric_min_us'], bounds['pan']['manual']['numeric_max_us']), (400, 2600))
            self.assertEqual((bounds['pan']['manual']['absolute_min_us'], bounds['pan']['manual']['absolute_max_us']), (1, 19_999))
            self.assertEqual((bounds['tilt']['manual']['numeric_min_us'], bounds['tilt']['manual']['numeric_max_us']), (500, 2500))

    def test_pan_expansion_does_not_change_tilt_645_to_2420_mechanical_limits(self):
        config = default_config()
        config['axes']['tilt'].update(safe_min_us=645, safe_max_us=2420,
            calibration_confirmed=True, calibration_points=[dict(angle_deg=-80, pulse_us=645),
                                                           dict(angle_deg=80, pulse_us=2420)])
        with GimbalController(config, self.backend, clock=self.clock) as controller:
            controller.manual_enable('tilt', 1500)
            before = deepcopy(controller.config['axes']['tilt'])
            events = list(self.backend.events)
            controller.apply_axis_config('pan', {**config['axes']['pan'], 'explore_min_us': 400, 'explore_max_us': 3500})
            self.assertEqual(self.backend.events, events)
            self.assertEqual(controller.config['axes']['tilt'], before)
            controller.manual_pulse('tilt', 3500)
            self.assertEqual(controller.snapshot()['axes']['tilt']['target_us'], 2420)
            controller.manual_pulse('tilt', 400)
            self.assertEqual(controller.snapshot()['axes']['tilt']['target_us'], 645)
            self.assertEqual((controller.axes['tilt'].safe_min_us, controller.axes['tilt'].safe_max_us), (645, 2420))

    def test_active_exploration_shrink_checks_output_and_target_but_widening_never_moves(self):
        config = default_config()
        config['axes']['pan'].update(explore_min_us=400, explore_max_us=3500)
        with GimbalController(config, self.backend, clock=self.clock) as controller:
            controller.manual_enable('pan', 2600)
            controller.manual_enable('tilt', 1500)
            events = list(self.backend.events)
            with self.assertRaises(GimbalError):
                controller.apply_axis_config('pan', {**config['axes']['pan'], 'explore_max_us': 2500})
            controller.manual_pulse('pan', 3300)
            with self.assertRaises(GimbalError):
                controller.apply_axis_config('pan', {**config['axes']['pan'], 'explore_max_us': 3000})
            self.assertEqual(controller.axes['pan'].explore_max_us, 3500)
            self.assertEqual(self.backend.events, events)
            controller.apply_axis_config('pan', {**config['axes']['pan'], 'explore_max_us': 3600})
            self.assertEqual(self.backend.events, events)
            state = controller.snapshot()['axes']
            self.assertEqual((state['pan']['output_us'], state['pan']['target_us']), (2600, 3300))
            self.assertTrue(state['tilt']['armed'])
            controller.disable('pan')
            count = len(self.backend.events)
            controller.apply_axis_config('pan', {**config['axes']['pan'], 'explore_max_us': 2500})
            self.assertEqual(controller.snapshot()['axes']['pan']['manual']['draft_us'], 2500)
            self.assertEqual(len(self.backend.events), count)
            self.assertEqual(self.backend.enabled, {'tilt': 1500})

    def test_absolute_pulse_validation_rejects_zero_or_a_full_period(self):
        for pulse in (0, -1, 20_000, 20_001, True, 1.5):
            with self.subTest(pulse=pulse), self.assertRaises(GimbalError):
                self.backend.start('pan', pulse)
        self.assertEqual(self.backend.events, [])
        self.backend.start('pan', 1)
        self.backend.set_pulse('pan', 19_999)
        self.assertEqual(self.backend.enabled['pan'], 19_999)
        self.backend.disable('pan')


class KernelCheckingSysfs:
    """Temporary files with the Linux5.10 sysfs full-state write constraint.

    This patches the filesystem boundary, not SysfsPWMBackend._write, so its
    production error translation is exercised too. All paths are test-created.
    pwm_apply_state validates period>0 and duty<=period before treating a
    redundant write as a no-op; enable=0 therefore also fails at period=0.
    """
    def __init__(self, paths, failure=None):
        self.paths = {path.resolve() for path in paths.values()}
        self.failure = failure
        self.attempts = []
        self.accepted = []
        self.original_write = Path.write_text
        self.original_read = Path.read_text

    def __enter__(self):
        fake = self

        def write(path, value, *args, **kwargs):
            if path.parent.resolve() not in fake.paths or path.name not in ('period', 'duty_cycle', 'enable', 'polarity'):
                return fake.original_write(path, value, *args, **kwargs)
            converted = str(value).strip() if path.name == 'polarity' else int(value)
            event = (path.name, converted)
            fake.attempts.append(event)
            if fake.failure:
                error = fake.failure(path, converted)
                if error:
                    raise error
            state = {name: fake.original_read(path.parent / name, encoding='ascii').strip()
                     for name in ('period', 'duty_cycle', 'enable', 'polarity')}
            state[path.name] = str(converted)
            period, duty = int(state['period']), int(state['duty_cycle'])
            if (period <= 0 or duty < 0 or duty > period or
                    state['enable'] not in ('0', '1') or state['polarity'] not in ('normal', 'inversed')):
                raise OSError(errno.EINVAL, 'Invalid argument', str(path))
            result = fake.original_write(path, value, *args, **kwargs)
            fake.accepted.append(event)
            return result

        self.patch = patch.object(Path, 'write_text', new=write)
        self.patch.start()
        return self

    def __exit__(self, *args):
        self.patch.stop()


class SysfsBackendTests(unittest.TestCase):
    def setUp(self):
        # Keep fake sysfs and its recursive TemporaryDirectory cleanup inside
        # this known workspace, including on locked-down Windows runners.
        workspace = Path(__file__).resolve().parent
        self.temp = tempfile.TemporaryDirectory(prefix=".gimbal-test-", dir=workspace)
        self.assertTrue(Path(self.temp.name).resolve().is_relative_to(workspace))
        self.addCleanup(self.temp.cleanup)
        self.paths = {}
        for index, axis in enumerate(("pan", "tilt")):
            path = Path(self.temp.name) / f"pwmchip{index + 7}" / "pwm0"
            path.mkdir(parents=True)
            for name, value in {"period": "1000000", "duty_cycle": "500000",
                                "enable": "0", "polarity": "normal"}.items():
                (path / name).write_text(value, encoding="ascii")
            self.paths[axis] = path
        self.backend = SysfsPWMBackend(self.paths)

    def test_mapping_validation_and_readonly_diagnostics(self):
        before = {axis: (path / "enable").read_text() for axis, path in self.paths.items()}
        info = self.backend.diagnostics()
        self.assertEqual(info["type"], "sysfs")
        self.assertTrue(info["channels"]["pan"]["exported"])
        self.assertEqual(before, {axis: (path / "enable").read_text() for axis, path in self.paths.items()})
        with self.assertRaises(GimbalError):
            SysfsPWMBackend({"pan": "pwmchip0/pwm0", "tilt": self.paths["tilt"]})
        with self.assertRaises(GimbalError):
            SysfsPWMBackend({"pan": self.paths["pan"], "tilt": self.paths["pan"]})

    def test_physical_opt_in_and_20ms_ns_write_order_cleanup(self):
        with KernelCheckingSysfs(self.paths) as fake:
            with GimbalController(configured(), self.backend) as control:
                self.assertEqual(fake.attempts, [])
                with self.assertRaises(GimbalError):
                    control.arm("pan", 1500, manual_calibration=True)
                self.assertEqual(fake.attempts, [])
                control.arm("pan", 1500, manual_calibration=True, physical=True)
                self.assertEqual(fake.accepted, [("period", PERIOD_NS),
                    ("duty_cycle", 1_500_000), ("enable", 1)])
            self.assertEqual(fake.accepted[-1], ("enable", 0))
            self.assertEqual((self.paths["pan"] / "enable").read_text(), "0")

    def set_state(self, axis='pan', *, period=0, duty=0, enabled=0, polarity='normal'):
        for name, value in dict(period=period, duty_cycle=duty, enable=enabled, polarity=polarity).items():
            (self.paths[axis] / name).write_text(str(value), encoding='ascii')

    def test_fresh_export_period_zero_is_initialized_before_any_other_write(self):
        self.set_state()
        with KernelCheckingSysfs(self.paths) as fake:
            with GimbalController(default_config(), self.backend) as control:
                control.manual_enable('pan', 1500)
                self.assertEqual(fake.accepted, [('period', PERIOD_NS), ('duty_cycle', 1_500_000), ('enable', 1)])
                self.assertTrue(control.snapshot()['axes']['pan']['armed'])
            self.assertEqual(fake.accepted[-1], ('enable', 0))

    def test_fake_rejects_even_redundant_disable_and_duty_zero_before_period(self):
        self.set_state()
        with KernelCheckingSysfs(self.paths) as fake:
            for name in ('enable', 'duty_cycle'):
                with self.subTest(name=name), self.assertRaises(GimbalError) as error:
                    self.backend._write(self.paths['pan'] / name, 0)
                self.assertIn(name, str(error.exception))
                self.assertIn('22', str(error.exception))
            self.assertEqual(fake.accepted, [])

    def test_large_existing_duty_is_cleared_before_period_is_shortened(self):
        self.set_state(period=50_000_000, duty=40_000_000)
        with KernelCheckingSysfs(self.paths) as fake:
            self.backend.start('pan', 1500)
            self.assertEqual(fake.accepted, [('duty_cycle', 0), ('period', PERIOD_NS),
                                            ('duty_cycle', 1_500_000), ('enable', 1)])
            self.backend.disable('pan')

    def test_inversed_polarity_is_changed_only_after_period_becomes_valid(self):
        self.set_state(polarity='inversed')
        with KernelCheckingSysfs(self.paths) as fake:
            self.backend.start('pan', 1500)
            self.assertEqual(fake.accepted, [('period', PERIOD_NS), ('polarity', 'normal'),
                                            ('duty_cycle', 1_500_000), ('enable', 1)])
            self.backend.disable('pan')

    def test_refuses_foreign_enabled_channel_without_disabling_it(self):
        (self.paths["pan"] / "enable").write_text("1")
        with KernelCheckingSysfs(self.paths) as fake:
            with GimbalController(configured(), self.backend) as control:
                with self.assertRaises(GimbalError):
                    control.arm("pan", 1500, manual_calibration=True, physical=True)
            self.assertEqual(fake.attempts, [])
        self.assertEqual((self.paths["pan"] / "enable").read_text(), "1")

    def test_start_failure_rolls_back_before_later_enable(self):
        def fail_duty(path, value):
            if path.name == 'duty_cycle' and value == 1_500_000:
                return OSError(errno.EIO, 'injected duty failure', str(path))
        with KernelCheckingSysfs(self.paths, fail_duty) as fake:
            with self.assertRaises(GimbalError) as error:
                self.backend.start('pan', 1500)
            self.assertIn(str(self.paths['pan'] / 'duty_cycle'), str(error.exception))
            self.assertIn('1500000', str(error.exception))
            self.assertIn('5', str(error.exception))
            self.assertNotIn(('enable', 1), fake.attempts)
            self.assertEqual(fake.accepted[-1], ('enable', 0))
        self.assertEqual((self.paths["pan"] / "enable").read_text(), "0")

    def test_initial_period_failure_preserves_primary_error_without_zero_period_cleanup(self):
        self.set_state()
        def fail_period(path, value):
            if path.name == 'period':
                return OSError(errno.EIO, 'initial period rejected', str(path))
        with KernelCheckingSysfs(self.paths, fail_period) as fake:
            with self.assertRaises(GimbalError) as error:
                self.backend.start('pan', 1500)
            self.assertIn(str(self.paths['pan'] / 'period'), str(error.exception))
            self.assertIn(str(PERIOD_NS), str(error.exception))
            self.assertIn('5', str(error.exception))
            self.assertEqual(fake.attempts, [('period', PERIOD_NS)])
            self.assertEqual(fake.accepted, [])
        # A later retry sees the same unconfigured channel and can succeed.
        with KernelCheckingSysfs(self.paths):
            self.backend.start('pan', 1500)
            self.backend.disable('pan')

    def test_primary_write_and_cleanup_errors_are_both_reported(self):
        def fail_duty_and_cleanup(path, value):
            if path.name == 'duty_cycle':
                return OSError(errno.EIO, 'primary requested-duty failure', str(path))
            if path.name == 'enable' and value == 0:
                return OSError(errno.EBUSY, 'cleanup disable failure', str(path))
        with KernelCheckingSysfs(self.paths, fail_duty_and_cleanup):
            with self.assertRaises(GimbalError) as error:
                self.backend.start('pan', 1500)
            detail = str(error.exception)
            self.assertIn('duty_cycle', detail)
            self.assertIn('1500000', detail)
            self.assertIn('enable', detail)
            self.assertIn('cleanup disable failure', detail)
        with KernelCheckingSysfs(self.paths):
            self.backend.disable('pan')
            self.backend.start('pan', 1500)
            self.backend.disable('pan')

    def test_read_failure_includes_the_node_path_and_never_enables(self):
        original = Path.read_text
        target = self.paths['pan'] / 'period'
        def read(path, *args, **kwargs):
            if path == target:
                raise OSError(errno.EIO, 'injected period read failure', str(path))
            return original(path, *args, **kwargs)
        with KernelCheckingSysfs(self.paths) as fake, patch.object(Path, 'read_text', new=read):
            with self.assertRaises(GimbalError) as error:
                self.backend.start('pan', 1500)
            self.assertIn(str(target), str(error.exception))
            self.assertEqual(fake.attempts, [])

    def test_failed_disable_is_reported_and_can_be_retried(self):
        with GimbalController(configured(), self.backend) as control:
            control.arm("pan", 1500, manual_calibration=True, physical=True)
            with patch.object(self.backend, "_write", side_effect=OSError("disable denied")):
                result = control.disable()
            self.assertIn("disable denied", result["fault"])
            self.assertEqual((self.paths["pan"] / "enable").read_text(), "1")
            control.disable()
            self.assertEqual((self.paths["pan"] / "enable").read_text(), "0")


if __name__ == "__main__":
    unittest.main()
