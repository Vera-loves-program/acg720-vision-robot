"""Actual controller and actual page limit callbacks, with MockPWM only.

No camera, network, configuration file, or physical PWM is opened. The page
callbacks run in Node's VM with a tiny DOM/command sink, not a real browser.
"""
from copy import deepcopy
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
from gimbal_control import GimbalController, GimbalError, MockPWMBackend


def config():
    axis = dict(safe_min_us=500, safe_max_us=2500, explore_min_us=400,
                explore_max_us=2600, fine_us=5, coarse_us=50, slew_us_per_s=150,
                calibration_confirmed=True)
    return {'axes': {
        'pan': dict(axis, calibration_points=[dict(angle_deg=0, pulse_us=500),
                                             dict(angle_deg=180, pulse_us=2500)]),
        'tilt': dict(axis, calibration_points=[dict(angle_deg=0, pulse_us=2500),
                                              dict(angle_deg=180, pulse_us=500)])}}


class LimitsTests(unittest.TestCase):
    def controller(self, raw=None):
        backend = MockPWMBackend()
        g = GimbalController(raw or config(), backend, clock=lambda: 0,
                             operating_angle_min={'tilt': 90})
        self.addCleanup(g.close)
        return g, backend

    def test_four_distinct_ranges_and_both_pulse_directions(self):
        for increasing in (False, True):
            raw = config()
            if increasing:
                for point in raw['axes']['tilt']['calibration_points']:
                    point['pulse_us'] = 3000 - point['pulse_us']
            original = deepcopy(raw)
            g, backend = self.controller(raw)
            limits = g.snapshot()['axes']['tilt']['limits']
            self.assertEqual((limits['explore_min_us'], limits['explore_max_us']), (400, 2600))
            self.assertEqual((limits['mechanical_min_us'], limits['mechanical_max_us']), (500, 2500))
            self.assertEqual((limits['table_min_angle_deg'], limits['table_max_angle_deg']), (0, 180))
            self.assertEqual((limits['table_min_us'], limits['table_max_us']), (500, 2500))
            self.assertEqual(limits['operating_floor_deg'], 90)
            self.assertEqual(limits['operating_floor_us'], 1500)
            self.assertEqual((limits['operating_min_us'], limits['operating_max_us']),
                             (1500, 2500) if increasing else (500, 1500))
            self.assertEqual(raw, original)
            self.assertEqual(backend.events, [])

    def test_preflight_is_pure_and_successful_apply_matches_it(self):
        g, backend = self.controller()
        g.manual_enable('tilt', 1000)
        before, events = deepcopy(g.snapshot()), deepcopy(backend.events)
        candidate = deepcopy(config()['axes']['tilt'])
        candidate['slew_us_per_s'] = 600
        parsed = g.validate_axis_config_change('tilt', candidate, clear_pending=False)
        self.assertEqual(parsed.slew_us_per_s, 600)
        self.assertEqual(g.snapshot(), before)
        self.assertEqual(backend.events, events)
        g.apply_axis_config('tilt', candidate, clear_pending=False)
        self.assertEqual(g.axes['tilt'].slew_us_per_s, 600)
        self.assertEqual(backend.events, events)

    def test_cannot_erase_tilt_table_on_or_off_and_cannot_install_below_floor_table(self):
        g, backend = self.controller()
        for enabled in (False, True):
            if enabled:
                g.manual_enable('tilt', 1000)
            before, events = deepcopy(g.snapshot()), deepcopy(backend.events)
            cleared = deepcopy(config()['axes']['tilt'])
            cleared.update(safe_min_us=None, safe_max_us=None,
                           calibration_points=[], calibration_confirmed=False)
            with self.assertRaisesRegex(GimbalError, '保留'):
                g.validate_axis_config_change('tilt', cleared)
            with self.assertRaises(GimbalError):
                g.apply_axis_config('tilt', cleared)
            self.assertEqual(g.snapshot(), before)
            self.assertEqual(backend.events, events)
        g.disable('tilt')
        below_floor = deepcopy(config()['axes']['tilt'])
        below_floor['calibration_points'] = [dict(angle_deg=0, pulse_us=2500), dict(angle_deg=45, pulse_us=500)]
        before = deepcopy(g.snapshot())
        with self.assertRaises(GimbalError):
            g.validate_axis_config_change('tilt', below_floor)
        self.assertEqual(g.snapshot(), before)

    def test_pending_bounds_and_active_limits_rejected_without_side_effects(self):
        g, backend = self.controller()
        g.manual_enable('pan', 2400)
        g.mark_limit('pan', 'max')
        g.disable('pan')
        candidate = deepcopy(config()['axes']['pan'])
        candidate.update(safe_max_us=2200, explore_max_us=2300,
                         calibration_points=[], calibration_confirmed=False)
        before, events = deepcopy(g.snapshot()), deepcopy(backend.events)
        with self.assertRaisesRegex(GimbalError, '暂存'):
            g.validate_axis_config_change('pan', candidate, clear_pending=False)
        self.assertEqual(g.snapshot(), before)
        self.assertEqual(backend.events, events)
        # PAN has no angular operating floor; ordinary deliberate recalibration
        # remains possible, without changing or erasing TILT's calibration.
        g.validate_axis_config_change('pan', candidate, clear_pending=True)
        g.manual_enable('tilt', 1000)
        invalid = deepcopy(config()['axes']['tilt'])
        invalid['safe_min_us'] = 1200
        invalid['calibration_points'] = [dict(angle_deg=90, pulse_us=1500), dict(angle_deg=100, pulse_us=1400)]
        with self.assertRaisesRegex(GimbalError, '当前输出'):
            g.validate_axis_config_change('tilt', invalid)

    def test_actual_page_range_labels_and_clear_guards(self):
        node = shutil.which('node')
        if not node:
            cached = Path('C:/Users/Vera/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe')
            node = str(cached) if cached.is_file() else None
        self.assertIsNotNone(node, 'Node is needed to execute actual page callbacks')
        g, _ = self.controller()
        g.manual_enable('tilt', 1000)
        snapshot = g.snapshot()
        snapshot['backend'] = dict(type='fpga_udp', physical=True)
        snapshot['axes']['tilt']['manual']['draft_us'] = 1500
        snapshot['axes']['tilt']['target_us'] = 1500
        fixture = dict(gimbal=snapshot, config=config(), tracking_limits=dict(tilt=True),
                       tracking_result=dict(error_px=dict(x=4.25,y=-3),observation_age_s=.12,control_basis='angular'),
                       tracking_servo=dict(status='holding',effective_gain=3),
                       configuration=dict(path='/robot/gimbal.json',sha256='fixture-sha',mtime_ns=123),
                       fpga_follow=dict(fpga=dict(pan_us=0, tilt_us=1200)))
        html = (ROOT / 'pc/usb_gimbal_ui.html').read_text(encoding='utf-8')
        names = ('manual', 'axisLimitData', 'operatingFloor', 'explorationBounds', 'protectTiltTable',
                 'limitNotice', 'renderRange', 'rangeText', 'renderAxisLimitDetails', 'trackingLimitNotice',
                 'expandRange', 'applyLimits', 'clearLimits', 'trackingPresentation', 'renderTrackingDiagnostics',
                 'number', 'saveTracking')
        functions = []
        for name in names:
            match = re.search(r'^(?:async )?function ' + name + r'\(.*$', html, re.M)
            self.assertIsNotNone(match, name)
            functions.append(match[0])
        fields = re.search(r'^const fields=.*$', html, re.M)[0]
        js = """const assert=require('node:assert/strict');
const nodes=new Map(),$=id=>{if(!nodes.has(id))nodes.set(id,{});return nodes.get(id)};
const axes=['pan','tilt'],titles={pan:'PAN 底座',tilt:'TILT 上层'},rangeDirty={pan:false,tilt:false};
const notices=[],commands=[];function notice(id,text,error=false){notices.push(text);$(id).textContent=text}
async function axisAction(axis,data){commands.push({axis,...data});return {ok:true}}
async function command(data){commands.push(data);return {ok:true}}
async function fetchState(){return true}
let trackingLoaded=true;
const state=FIXTURE;
FIELDS
FUNCTIONS
(async()=>{
 const m=manual('tilt'),input={};renderRange('tilt',m,input);
 assert.equal(input.max,'1500');assert.equal($('tiltExploreMax').value,2600);
 assert.equal($('tiltRangeSummary').textContent,'400～2600µs');
 renderAxisLimitDetails('tilt');
 const info=$('tiltLimitDetails').textContent;
 for(const text of ['探索边界：400～2600','机械脉宽：500～2500','角度表：0～180','当前可请求：500～1500','软件运行下限90'])assert.ok(info.includes(text),info);
 assert.equal($('tiltClearLimits').disabled,true);assert.equal($('tiltClearAnglesApply').disabled,true);
 assert.ok($('tiltReported').textContent.includes('1000µs'));assert.ok($('tiltReported').textContent.includes('1200µs'));
 limitNotice('tilt',2000);assert.ok(notices.at(-1).includes('90°'));assert.ok(!notices.at(-1).includes('清除'));
 await clearLimits('tilt');await applyLimits('tilt',true);assert.equal(commands.length,0);
 await expandRange('tilt',1);assert.deepEqual(commands.at(-1),{axis:'tilt',action:'manual_range',explore_min_us:400,explore_max_us:2700});
 assert.ok(trackingLimitNotice('tilt').includes('90°软件运行下限'));
 renderTrackingDiagnostics();assert.ok($('trackingObservation').textContent.includes('4.3px'));assert.ok($('trackingObservation').textContent.includes('120ms'));assert.ok($('trackingObservation').textContent.includes('angular'));
 assert.ok($('trackingActuation').textContent.includes('目标1500µs / Pi命令1000µs'));assert.ok($('configurationState').textContent.includes('fixture-sha'));
 state.tracking=true;state.tracking_memory={phase:'waiting'};state.tracking_result={status:'missing'};
 const hold=trackingPresentation();assert.ok(hold.summary.includes('保持'));assert.ok(hold.detail.includes('不执行旧画面预测移动'));
 const expectedFields=['deadband','normalized_gain_dps','gain','max_rate_dps','max_accel_dps2','filter_time_constant_s','deadband_hysteresis','min_rate_dps','max_control_dt_s'];assert.deepEqual(fields.map(f=>f[0]),expectedFields);
 for(const [name]of fields)$('track_'+name).value=String(name==='max_accel_dps2'?180:1);
 await saveTracking();assert.deepEqual(Object.keys(commands.at(-1).tracking),expectedFields);assert.equal(commands.at(-1).tracking.max_accel_dps2,180);
 console.log('PASS actual page callbacks: four limit labels, real exploration expansion, runtime clamp cause, no TILT erase request, Pi/FPGA commands, nine-field save, live error/age/basis and hold text');
})().catch(e=>{console.error(e);process.exitCode=1});
""".replace('FIXTURE', json.dumps(fixture, ensure_ascii=True)).replace('FIELDS', fields).replace('FUNCTIONS', '\n'.join(functions))
        completed = subprocess.run([node, '-e', js], capture_output=True, text=True,
                                   encoding='utf-8', cwd=ROOT, timeout=15)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        print(completed.stdout.strip())


if __name__ == '__main__':
    unittest.main()
