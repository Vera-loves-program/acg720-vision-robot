"""Real controller tests for TILT>=90 and135 start; no hardware/network I/O."""
from copy import deepcopy
import json
from pathlib import Path
import random
import struct
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from gimbal_control import GimbalController,MockPWMBackend,GimbalError
from robot_follow import FollowLink,FpgaPWMBackend
from check_robot_follow import Socket
from check_gimbal_tracking_loop import config_for
from usb_gimbal import GimbalApp


class Tests(unittest.TestCase):
    def test_both_pwm_directions_round_inward(self):
        for increasing in (False,True):
            config=config_for(upright_tilt_deg=180)
            if increasing:
                points=config['axes']['tilt']['calibration_points']
                for p in points:p['pulse_us']=3065-p['pulse_us']
            before=deepcopy(config)
            backend=MockPWMBackend()
            g=GimbalController(config,backend,clock=lambda:0,operating_angle_min={'tilt':90})
            try:
                lo,hi=g.pulse_limits('tilt')
                for pulse in range(lo,hi+1):
                    self.assertGreaterEqual(g.axes['tilt'].pulse_to_angle(pulse),90)
                outside=lo-1 if increasing else hi+1
                self.assertLess(g.axes['tilt'].pulse_to_angle(outside),90)
                g.manual_enable('tilt',outside)
                self.assertGreaterEqual(g.snapshot()['axes']['tilt']['angle_deg'],90)
                now=0
                rng=random.Random(15)
                for i in range(600):
                    if i%4==0:g.manual_pulse('tilt',rng.randint(1,4095))
                    elif i%4==1:g.manual_jog('tilt',rng.choice((-1,1)),True)
                    elif i%4==2:g.set_angle('tilt',rng.uniform(0,175))
                    else:g.set_pulse('tilt',rng.randint(1,4095),manual_calibration=True)
                    now+=.02;g.tick(now)
                    pose=g.snapshot()['axes']['tilt']
                    self.assertGreaterEqual(pose['angle_deg'],90)
                    self.assertGreaterEqual(g.axes['tilt'].pulse_to_angle(pose['target_us']),90)
                self.assertEqual(config,before)
                self.assertEqual(g.config,before)
            finally:g.close()

    def test_actual_robot_start_and_fpga_limits(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
            path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)))
            original=path.read_bytes();backend=FpgaPWMBackend();sock=Socket()
            link=FollowLink(backend,tmp,sock=sock)
            app=GimbalApp(path,backend=backend,follow_link=link,standalone=True)
            try:
                signature=app._angle_signature()
                link._arm(app)
                self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
                self.assertAlmostEqual(app.gimbal.snapshot()['axes']['tilt']['angle_deg'],135,delta=.1)
                app.gimbal.manual_pulse('tilt',2100)
                self.assertGreaterEqual(app.gimbal.axes['tilt'].pulse_to_angle(app.gimbal.snapshot()['axes']['tilt']['target_us']),90)
                link.send(app,100)
                raw=next(raw for raw,_ in sock.sent if raw[:4]==b'FOL1')
                low,high=struct.unpack('>HH',raw[18:22])
                self.assertEqual((low,high),app.gimbal.pulse_limits('tilt'))
                self.assertGreaterEqual(app.gimbal.axes['tilt'].pulse_to_angle(high),90)
                self.assertLess(high,2100)
                self.assertEqual(app._angle_signature(),signature)
                self.assertEqual(path.read_bytes(),original)
                self.assertEqual(app.gimbal.pulse_limits('pan'),(340,2600))
            finally:app.close()

    def test_cannot_start_at_old45_degree_pose(self):
        g=GimbalController(config_for(upright_tilt_deg=180),MockPWMBackend(),operating_angle_min={'tilt':90})
        try:
            with self.assertRaises(GimbalError):g.arm('tilt',2100)
            self.assertFalse(g.snapshot()['axes']['tilt']['armed'])
        finally:g.close()


if __name__=='__main__':unittest.main()
