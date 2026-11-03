"""Real application/profile and confidence-hysteresis regressions; mock PWM only.

Imported from teammate's2026-10-30 response candidate, original SHA256
99399a75b468004cab35bb9fa520b582308cbe1cfc4790ab453d8d295b81b441.
"""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from gimbal_control import MockPWMBackend
from target_tracking import TargetTracker,CameraCalibration
from usb_gimbal import GimbalApp
from robot_follow import FollowLink
from check_gimbal_tracking_loop import config_for,CameraPlant,pulse_for


def face(score):
    return dict(x1=300,y1=180,x2=380,y2=300,score=score,kind='human')


class ResponseTests(unittest.TestCase):
    def test_normal_and_fast_profiles_center_with_delayed_camera(self):
        for profile in ('normal','fast'):
            for delay in (0.,.16):
                with self.subTest(profile=profile,delay=delay):
                    plant=CameraPlant(self,target=(12,6),mount='upright_at_180',delay=delay,
                        initial_pulses=dict(pan=1500,tilt=pulse_for('tilt',135)))
                    plant.app.command(dict(action='set_speed_profile',profile=profile))
                    plant.advance(.4)
                    plant.select_and_start()
                    plant.advance(8)
                    self.assertLess(max(abs(v) for v in plant.normalized_error().values()),.06)

    def test_fast_profile_changes_real_pwm_slew_and_preserves_calibration(self):
        distances={}
        for profile in ('slow','normal','fast'):
            with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as directory:
                path=Path(directory)/'config.json'
                original=config_for(upright_tilt_deg=180)
                path.write_text(json.dumps(original),encoding='utf-8')
                backend=MockPWMBackend()
                app=GimbalApp(path,backend=backend)
                self.addCleanup(app.close)
                now=[100.]
                app.gimbal._clock=lambda:now[0]
                app.gimbal.manual_enable('pan',1500)
                app.gimbal.manual_pulse('pan',1700)
                app.tracking=True
                before=list(backend.events)
                app.command(dict(action='set_speed_profile',profile=profile))
                self.assertEqual(backend.events,before)  # Applying never moves/enables.
                self.assertFalse(app.tracking)
                app.gimbal.manual_pulse('pan',2200)
                for _ in range(20):
                    now[0]+=.02
                    app.gimbal.tick(now[0])
                distances[profile]=backend.enabled['pan']-1500
                saved=json.loads(path.read_text(encoding='utf-8'))
                for axis in ('pan','tilt'):
                    for key in ('safe_min_us','safe_max_us','calibration_points','calibration_confirmed'):
                        self.assertEqual(saved['axes'][axis][key],original['axes'][axis][key])
                self.assertFalse(app.gimbal.snapshot()['axes']['tilt']['armed'])
                app.close()
        self.assertGreater(distances['normal'],distances['slow']*3)
        self.assertGreater(distances['fast'],distances['normal']*1.8)

    def test_invalid_profile_does_not_write_or_pause(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as directory:
            path=Path(directory)/'config.json'
            app=GimbalApp(path,backend=MockPWMBackend())
            self.addCleanup(app.close)
            with self.assertRaises(ValueError):app.command(dict(action='set_speed_profile',profile='typo'))
            self.assertFalse(path.exists())

    def test_face_score_dip_keeps_lock_but_cannot_acquire_or_reacquire(self):
        tracker=TargetTracker(CameraCalibration(640,480))
        with self.assertRaises(ValueError):tracker.select_target(face(.45),100,1)
        tracker.select_target(face(.65),100,1)
        for seq in range(2,6):
            result=tracker.update([face(.45)],100+seq*.05,100+seq*.05,seq)
            self.assertEqual(result['status'],'tracking')
            self.assertEqual(result['target']['score'],.45)
        tracker.update([],100.35,100.35,7)
        for seq in range(8,12):
            result=tracker.update([face(.45)],100+seq*.05,100+seq*.05,seq)
            self.assertEqual(result['status'],'searching')
            self.assertEqual(result['yaw_rate_dps'],0)
        for seq in range(12,15):
            result=tracker.update([face(.65)],100+seq*.05,100+seq*.05,seq)
        self.assertEqual(result['status'],'tracking')

    def test_fpga_link_passes_low_score_to_tracker_without_catalog_readmission(self):
        link=FollowLink.__new__(FollowLink)
        link.registry=SimpleNamespace(selected=1,tracks={1:dict(box=face(.65))})
        frame=SimpleNamespace(detections=[face(.45),dict(face(.9),kind='dog')])
        self.assertEqual(link.tracking_boxes(frame),[face(.45)])

    def test_fpga_dog_gate_is_not_relaxed_by_face_hysteresis(self):
        link=FollowLink.__new__(FollowLink)
        link.registry=SimpleNamespace(selected=1,tracks={1:dict(box=dict(face(.65),kind='dog'))})
        low=dict(face(.45),kind='dog');admitted=dict(face(.50),kind='dog')
        frame=SimpleNamespace(detections=[low,admitted,face(.9)])
        self.assertEqual(link.tracking_boxes(frame),[admitted])


if __name__=='__main__':unittest.main(verbosity=2)
