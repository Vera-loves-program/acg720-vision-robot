"""Actual app closed-loop tests with independent delayed mechanical motion.

No camera, OpenCV/model, socket, servo or FPGA is opened. Unlike the original
CameraPlant, emitted PWM is a REQUEST: a rate-limited first-order actuator
follows after transport delay. Camera capture/inference are separate; inference
is serial and consumes the newest capture rather than a growing frame queue.
The fixture parameters are engineering scenarios, not measured servo constants.
--compare/--legacy-probe additionally require Git commit84a3ced; normal tests
and the deployed application do not depend on Git or historical sources.
"""
from collections import deque
from copy import deepcopy
from contextlib import contextmanager
import importlib.util
import hashlib
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import sys
import tempfile
import subprocess
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
import usb_gimbal
from usb_gimbal import GimbalApp,VisionFrame
from gimbal_control import MockPWMBackend
from check_gimbal_tracking_loop import CameraPlant,physical_coordinate,pulse_for,config_for,dot
import check_gimbal_tracking_loop as original_fixture


@contextmanager
def legacy_application(ref='84a3ced'):
    """Execute preserved actual R4 modules, rather than a reimplemented loop."""
    names=('gimbal_control','target_tracking','usb_gimbal')
    previous={name:sys.modules.get(name) for name in names}
    with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
        try:
            for name in names:
                result=subprocess.run(['git','-c',f'safe.directory={ROOT.as_posix()}',
                    'show',f'{ref}:pc/{name}.py'],cwd=ROOT,capture_output=True,check=True)
                path=Path(tmp)/(name+'.py');path.write_bytes(result.stdout)
                spec=importlib.util.spec_from_file_location(name,path)
                module=importlib.util.module_from_spec(spec);sys.modules[name]=module
                spec.loader.exec_module(module)
            legacy=sys.modules['usb_gimbal']
            with mock.patch.object(original_fixture,'GimbalApp',legacy.GimbalApp), \
                 mock.patch.object(original_fixture,'usb_gimbal',legacy), \
                 mock.patch.object(sys.modules[__name__],'GimbalApp',legacy.GimbalApp), \
                 mock.patch.object(sys.modules[__name__],'VisionFrame',legacy.VisionFrame):
                yield
        finally:
            for name,module in previous.items():
                if module is None:sys.modules.pop(name,None)
                else:sys.modules[name]=module


class MechanicalCameraPlant(CameraPlant):
    def __init__(self,*args,mechanical_tau=.16,command_delay=.04,
                 mechanical_max_rate=120.,camera_fps=15.,**kwargs):
        self.mechanical_tau=mechanical_tau;self.command_delay=command_delay
        self.mechanical_max_rate=mechanical_max_rate;self.camera_fps=camera_fps
        self.mechanical=None;self.command_queue=deque();self.applied_command=None
        self.latest_capture=None;self.inference=None;self.inference_sequence=0
        self.target_motion=None;self.motion_started=None;self.trace=[]
        self.command_trace=[];self.delivered=0
        self.pixel_jitter=None;self.dropout_every=0
        super().__init__(*args,**kwargs)

    def orientation(self):
        if self.mechanical is None:self.mechanical=super().orientation()
        return dict(self.mechanical)

    def set_image_target_offset(self,horizontal_deg,vertical_deg):
        super().set_image_target_offset(horizontal_deg,vertical_deg)
        self.motion_basis=self.camera_basis()

    def set_motion(self,callback):
        self.target_motion=callback;self.motion_started=self.now

    def capture(self):
        self.sequence+=1;position=self.image_position();boxes=[]
        if self.visible and position is not None and 0<=position[0]<=self.width and 0<=position[1]<=self.height:
            x,y=position
            boxes=[dict(x1=x-80,y1=y-80,x2=x+80,y2=y+80,score=.95,kind='human')]
            if self.pixel_jitter:
                jitter=self.pixel_jitter[(self.sequence-1)%len(self.pixel_jitter)]
                boxes[0]['x1']+=jitter;boxes[0]['x2']+=jitter
            if self.dropout_every and self.sequence%self.dropout_every==0:boxes=[]
        self.latest_capture=VisionFrame(self.sequence,self.now,object(),b'synthetic',boxes,
            deepcopy(self.app.gimbal.snapshot()),self.width,self.height,self.delay*1000,'raw synthetic projection')

    def advance(self,seconds):
        end=self.now+seconds
        while self.now<end-1e-8:
            dt=min(.01,end-self.now);self.now+=dt
            command={axis:physical_coordinate(axis,self.backend.enabled[axis]) for axis in ('pan','tilt')}
            self.command_queue.append((self.now+self.command_delay,command))
            while self.command_queue and self.command_queue[0][0]<=self.now+1e-8:
                self.applied_command=self.command_queue.popleft()[1]
            if self.applied_command:
                for axis in ('pan','tilt'):
                    step=(self.applied_command[axis]-self.mechanical[axis])*(-math.expm1(-dt/self.mechanical_tau))
                    limit=self.mechanical_max_rate*dt
                    self.mechanical[axis]+=max(-limit,min(limit,step))
            if self.target_motion is not None:
                h,v=self.target_motion(self.now-self.motion_started)
                forward,left,up=self.motion_basis
                h,v=math.tan(math.radians(h)),math.tan(math.radians(v))
                ray=tuple(f+h*l+v*u for f,l,u in zip(forward,left,up));norm=math.sqrt(dot(ray,ray))
                self.target=tuple(x/norm for x in ray)
            if self.camera_enabled and self.now>=self.next_capture-1e-8:
                self.capture();self.next_capture+=1/self.camera_fps
            if self.inference is not None and self.inference[0]<=self.now+1e-8:
                self.app.publish(self.inference[1]);self.delivered+=1;self.inference=None
            if self.inference is None and self.latest_capture is not None and self.latest_capture.sequence>self.inference_sequence:
                self.inference_sequence=self.latest_capture.sequence
                self.inference=(self.now+self.delay,self.latest_capture)
            self.app.command(dict(action='heartbeat'))
            # The real control thread is50Hz; mechanics/capture run finer.
            if not self.command_trace or self.now-self.command_trace[-1]['time']>=.02-1e-8:
                self.app._control_once(self.now)
                entry=dict(time=self.now,mechanical=dict(self.mechanical),command={
                    axis:physical_coordinate(axis,self.backend.enabled[axis]) for axis in ('pan','tilt')},
                    pwm=dict(self.backend.enabled),error=self.normalized_error(),
                    status=(self.app.tracking_result or {}).get('status'))
                self.command_trace.append(entry)
                self.trace.append(entry)
                self.samples.append(entry['error']);self.actuator_samples.append((self.now,entry['command']))

    def tail_metrics(self,seconds=3.):
        tail=[x for x in self.trace if x['time']>=self.now-seconds]
        return dict(rms={axis:math.sqrt(sum(x['error'][axis]**2 for x in tail)/len(tail)) for axis in ('pan','tilt')},
            error_peak={axis:max(abs(x['error'][axis]) for x in tail) for axis in ('pan','tilt')},
            physical_span={axis:max(x['mechanical'][axis] for x in tail)-min(x['mechanical'][axis] for x in tail) for axis in ('pan','tilt')},
            max_command_lead=max(abs(x['command'][axis]-x['mechanical'][axis]) for x in tail for axis in ('pan','tilt')))


def centered_then_missing(ambiguous=False):
    """A measured6.4px offset is inside the existing12.8px PAN stop band."""
    with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
        path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)),encoding='utf-8')
        backend=MockPWMBackend();app=GimbalApp(path,backend=backend);app.gimbal._clock=lambda:100.
        try:
            for axis,pulse in (('pan',1500),('tilt',pulse_for('tilt',135))):
                app.command(dict(action='manual_enable',axis=axis,pulse_us=pulse))
            def publish(sequence,now,visible=True):
                x=320-6.4;target=dict(x1=x-40,y1=200,x2=x+40,y2=280,kind='human',score=.95)
                detections=([target] if visible else [target,dict(target)] if ambiguous else [])
                app.publish(VisionFrame(sequence,now,None,b'jpeg',detections,app.gimbal.snapshot(),640,480,0,'raw'))
            publish(1,100)
            with mock.patch('time.monotonic',return_value=100):
                app.command(dict(action='select',sequence=1,index=0));app.command(dict(action='track',enabled=True))
            publish(2,100.1)
            with mock.patch('time.monotonic',return_value=100.1):app._control_once(100.1)
            before=dict(backend.enabled);result=dict(app.tracking_result)
            publish(3,100.2,False)
            for index in range(6):
                now=100.2+index*.02
                with mock.patch('time.monotonic',return_value=now):app._control_once(now)
            return dict(before=before,after=dict(backend.enabled),last_live=result,
                        status=(app.tracking_result or {}).get('status'))
        finally:app.close()


class MechanicalResponseTests(unittest.TestCase):
    def plant(self,**kwargs):
        initial=kwargs.pop('initial_pulses',dict(pan=1500,tilt=pulse_for('tilt',135)))
        p=MechanicalCameraPlant(self,target=(12,6),mount='upright_at_180',
            initial_pulses=initial,**kwargs)
        p.app.command(dict(action='set_speed_profile',profile='normal'));p.advance(.4);p.select_and_start()
        return p

    def test_near_center_then_missing_never_moves_pwm(self):
        result=centered_then_missing()
        self.assertAlmostEqual(result['last_live']['yaw_error_normalized'],.02)
        self.assertEqual(result['before'],result['after'],result)

    def test_ambiguous_center_frame_does_not_replay_a_saved_offset(self):
        result=centered_then_missing(ambiguous=True)
        self.assertEqual(result['status'],'ambiguous')
        self.assertEqual(result['before'],result['after'],result)

    def test_center_pixel_jitter_and_intermittent_dropout_stay_still(self):
        p=MechanicalCameraPlant(self,target=(0,0),mount='upright_at_180',delay=.12,
            initial_pulses=dict(pan=1500,tilt=pulse_for('tilt',135)))
        p.pixel_jitter=(-3.,2.,-4.,3.,0.);p.dropout_every=6
        p.app.command(dict(action='set_speed_profile',profile='fast'));p.advance(.4);p.select_and_start()
        before=dict(p.backend.enabled);p.advance(5)
        self.assertEqual(p.backend.enabled,before)
        self.assertTrue(all(row['pwm']==before for row in p.trace[-100:]))

    def test_delayed_servo_static_settles_without_late_motion(self):
        for delay,tau in ((.08,.12),(.17,.20)):
            with self.subTest(inference_delay=delay,mechanical_tau=tau):
                p=self.plant(delay=delay,mechanical_tau=tau)
                p.advance(12);metrics=p.tail_metrics();print('STATIC',delay,tau,metrics)
                self.assertLess(max(metrics['rms'].values()),.07)
                self.assertLess(max(metrics['physical_span'].values()),1.5)

    def test_moving_target_with_serial_inference_has_bounded_error(self):
        p=self.plant(delay=.12,mechanical_tau=.16)
        p.set_motion(lambda t:(8*math.sin(.7*t),3*math.sin(.5*t)))
        p.advance(14);metrics=p.tail_metrics(10);print('MOVING',metrics)
        self.assertLess(max(metrics['rms'].values()),.16)
        self.assertLess(max(metrics['error_peak'].values()),.32)
        self.assertLess(p.delivered,p.sequence)  # Frames are dropped, never queued for old inference.

    def test_250ms_serial_inference_cannot_accumulate_unbounded_motion(self):
        p=self.plant(delay=.25,mechanical_tau=.2);p.advance(8)
        p.camera_enabled=False;p.advance(.55);held=dict(p.backend.enabled)
        p.advance(.5)
        self.assertEqual(p.backend.enabled,held)
        for row in p.trace:
            self.assertLessEqual(max(abs(row['command'][axis]-row['mechanical'][axis])
                                     for axis in ('pan','tilt')),8.)

    def test_stop_holds_commands_even_while_mechanics_finish_responding(self):
        p=self.plant(delay=.12,mechanical_tau=.2);p.advance(.4)
        p.app.command(dict(action='track',enabled=False));before=dict(p.backend.enabled)
        p.advance(1)
        self.assertEqual(p.backend.enabled,before)

    def test_lost_camera_expires_pending_command_before_old_frame_age_limit(self):
        p=self.plant(delay=.08,mechanical_tau=.16);p.advance(.25)
        p.camera_enabled=False;p.advance(.4);before=dict(p.backend.enabled)
        p.advance(.25)
        self.assertEqual(p.backend.enabled,before)

    def test_limits_remain_bounded_and_reversal_can_return(self):
        p=self.plant(delay=.08,mechanical_tau=.16,
            initial_pulses=dict(pan=pulse_for('pan',238),tilt=pulse_for('tilt',135)))
        p.set_image_target_offset(12,0);p.advance(3)
        self.assertGreaterEqual(p.backend.enabled['pan'],2545,'fixture must actually reach the calibrated endpoint')
        for row in p.trace:
            self.assertTrue(380<=row['pwm']['pan']<=2550)
            self.assertTrue(645<=row['pwm']['tilt']<=1559)
        p.set_image_target_offset(-8,0);p.advance(.5);p.select_and_start()
        before=p.backend.enabled['pan'];p.advance(4)
        self.assertNotEqual(p.backend.enabled['pan'],before)


def probe():
        result=centered_then_missing()
        print('CENTERED_MISSING',dict(before=result['before'],after=result['after'],
            live_delta=result['last_live'].get('delta_pan_command_deg'),status=result['status']))
        case=unittest.TestCase()
        try:
            p=MechanicalCameraPlant(case,target=(12,6),mount='upright_at_180',delay=.12,
                initial_pulses=dict(pan=1500,tilt=pulse_for('tilt',135)))
            p.app.command(dict(action='set_speed_profile',profile='normal'));p.advance(.4);p.select_and_start()
            p.set_motion(lambda t:(8*math.sin(.7*t),3*math.sin(.5*t)));p.advance(14)
            print('MOVING_PROBE',p.tail_metrics(10),'delivered/captured',p.delivered,p.sequence)
        finally:case.doCleanups()


def compare_matrix(label):
    rows=[]
    for profile in ('normal','fast'):
        for delay in (.04,.12,.25):
            case=unittest.TestCase()
            try:
                p=MechanicalCameraPlant(case,target=(12,6),mount='upright_at_180',delay=delay,
                    initial_pulses=dict(pan=1500,tilt=pulse_for('tilt',135)))
                p.app.command(dict(action='set_speed_profile',profile=profile));p.advance(.4);p.select_and_start()
                p.set_motion(lambda t:(8*math.sin(.7*t),3*math.sin(.5*t)));p.advance(14)
                row=dict(controller=label,profile=profile,inference_delay_s=delay,
                         mechanical_tau_s=.16,command_delay_s=.04,**p.tail_metrics(10))
                rows.append(row);print(json.dumps(row,ensure_ascii=False))
            finally:case.doCleanups()
    return rows
if __name__=='__main__':
    if '--legacy-probe' in sys.argv:
        with legacy_application():probe()
    elif '--compare' in sys.argv:
        with legacy_application():old=compare_matrix('R4 actual3modules84a3ced')
        new=compare_matrix('current')
        output=ROOT/'tmp/pi-tracking-20261104';output.mkdir(parents=True,exist_ok=True)
        paths=('pc/usb_gimbal.py','pc/target_tracking.py','pc/gimbal_control.py','pc/tracking_servo.py')
        evidence=dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),baseline_commit='84a3ced',
            baseline_modules=('pc/usb_gimbal.py','pc/target_tracking.py','pc/gimbal_control.py'),
            current_source_sha256={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in paths},
            shared_renderer='check_gimbal_tracking_loop.CameraPlant camera basis; actual PWM drives separate mechanical lag',
            hardware_tested=False,old=old,new=new)
        (output/'mechanical-comparison.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    elif '--probe' in sys.argv:probe()
    else:unittest.main(verbosity=2)
