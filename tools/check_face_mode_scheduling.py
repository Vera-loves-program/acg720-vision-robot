"""Actual app/worker/controller tests; mocked models/PWM, no camera or sockets."""
import json
import struct
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from usb_gimbal import GimbalApp,IndependentDetectors,VisionFrame,VisionWorker
from usb_camera import UsbCamera,CameraFrame
from check_independent_face_tracking import app_link,box,feed
from check_gimbal_tracking_loop import config_for
from gimbal_control import MockPWMBackend
from check_robot_follow import report


def fake_detectors():
    detector=IndependentDetectors.__new__(IndependentDetectors)
    detector.errors={};detector.active_kind=None
    detector.engines={kind:SimpleNamespace(predict=mock.Mock(return_value=[
        SimpleNamespace(x0=290,y0=170,x1=350,y1=290,confidence=.9)])) for kind in ('dog','human')}
    return detector


class Image:
    shape=(480,640,3)
    def copy(self):return Image()


class Tests(unittest.TestCase):
    def test_face_only_skips_expensive_dog_and_switch_back_is_supported(self):
        detector=fake_detectors()
        dogs,faces,dog_ms,_=detector.predict(None,kind='human')
        self.assertEqual(dogs,[]);self.assertEqual(len(faces),1);self.assertEqual(dog_ms,0)
        detector.engines['dog'].predict.assert_not_called()
        self.assertEqual(detector.status()['scheduled'],dict(dog=False,face=True))
        detector.predict(None,kind='dog')
        self.assertEqual(detector.engines['human'].predict.call_count,1)
        self.assertEqual(detector.engines['dog'].predict.call_count,1)
        detector.predict(None)
        self.assertEqual(detector.engines['human'].predict.call_count,2)
        self.assertEqual(detector.engines['dog'].predict.call_count,2)

    def test_lcd_face_prepare_three_fresh_frames_then_dog_switch(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100)
            sock.incoming.append((report(0,1,9),('192.168.10.2',5000)))
            with mock.patch('time.monotonic',return_value=100.1):link.poll(app,100.1)
            self.assertEqual(app._detection_kind(),'human')
            self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
            for seq in (1,2):feed(app,sock,seq,100.5+seq*.1,[box(290,'human')],1,9)
            self.assertFalse(app.tracking)
            feed(app,sock,3,100.8,[box(290,'human')],1,9)
            self.assertTrue(app.tracking,link.error)
            face_id=link.registry.selected
            self.assertEqual(app._detection_kind(),'human')
            sock.incoming.append((report(0,2,5,tilt=1075),('192.168.10.2',5000)))
            with mock.patch('time.monotonic',return_value=101):link.poll(app,101)
            self.assertEqual(app._detection_kind(),'dog');self.assertFalse(app.tracking)
            for seq in (4,5):feed(app,sock,seq,101+(seq-3)*.1,[box(290)],2,5)
            self.assertFalse(app.tracking)
            feed(app,sock,6,101.3,[box(290)],2,5)
            self.assertTrue(app.tracking,link.error)
            self.assertNotEqual(link.registry.selected,face_id)
            self.assertEqual(link.registry.tracks[link.registry.selected]['box']['kind'],'dog')
            self.assertEqual(app._detection_kind(),'dog')

    def test_web_kind_change_cancels_old_lcd_filter_and_wheel_reference(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100);link._arm(app,100)
            link.registry.selected=1;link.registry.tracks[1]=dict(box=box(280),time=101,sequence=1,hits=3)
            link.follower.reference=120.;link.follower.forward_sign=1;link.follower.active=True
            link.error='old DOG command failed';link.error_code=2
            app.publish(VisionFrame(1,101,None,b'jpeg',[box(290,'human'),box(280)],app.gimbal.snapshot(),640,480,170,'raw'))
            with mock.patch('time.monotonic',return_value=101):
                app.command(dict(action='select',sequence=1,index=0))
                app.command(dict(action='track',enabled=True))
            self.assertEqual(link.registry.selected,0);self.assertIsNone(link.follower.reference)
            self.assertFalse(link.auto_select);self.assertFalse(link.geometry_active)
            self.assertEqual(app._detection_kind(),'human')
            sock.incoming.append((report(0,0,0,tilt=1075),('192.168.10.2',5000)))
            app.publish(VisionFrame(2,101.1,None,b'jpeg',[box(290,'human')],app.gimbal.snapshot(),640,480,32,'raw'))
            with mock.patch('time.monotonic',return_value=101.1):app._control_once(101.1)
            self.assertEqual(app.tracking_result['status'],'tracking')
            self.assertIsNone(link.error)
            fol=next(raw for raw,_ in reversed(sock.sent) if raw[:4]==b'FOL1')
            self.assertTrue(fol[5]&1)
            self.assertEqual(fol[5]&6,0)  # Old side-follow reference/sign withdrawn.
            self.assertEqual(struct.unpack('>h',fol[8:10])[0],0)
            app.command(dict(action='track',enabled=False))
            self.assertIsNone(app._detection_kind())  # Both classes become selectable again.
            with mock.patch('time.monotonic',return_value=101):
                app.command(dict(action='select',sequence=1,index=1))
                app.command(dict(action='track',enabled=True))
            self.assertEqual(app._detection_kind(),'dog')

    def test_mode_change_discards_inflight_old_class_result(self):
        with app_link() as (app,link,backend,sock):
            link.auto_select=True;link.auto_kind='human'
            result=VisionFrame(10,101,None,b'jpeg',[box(290)],app.gimbal.snapshot(),640,480,135,'raw')
            self.assertFalse(app.publish(result,inference_kind='dog'))
            self.assertFalse(app.publish(result,inference_kind=None))
            self.assertIsNone(app.latest)
            human=VisionFrame(11,101,None,b'jpeg',[box(290,'human')],app.gimbal.snapshot(),640,480,32,'raw')
            self.assertTrue(app.publish(human,inference_kind='human'))
            self.assertEqual(app.latest.sequence,11)

    def test_camera_newest_returns_latest_not_a_backlog(self):
        camera=UsbCamera('/not-opened')
        try:
            first=CameraFrame(1,100,None,{})
            newest=CameraFrame(30,101,None,{})
            camera.latest=first;camera.latest=newest
            self.assertIs(camera.newest(after=1,timeout=0),newest)
            self.assertIsNone(camera.newest(after=30,timeout=0))
        finally:camera.close()

    def test_actual_worker_uses_single_model_and_drops_outdated_mode(self):
        for switch_to_preview in (False,True):
            with self.subTest(switch=switch_to_preview), tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
                path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)),encoding='utf-8')
                app=GimbalApp(path,backend=MockPWMBackend())
                self.addCleanup(app.close)
                detector=fake_detectors();now=[100.];calls=[]
                app.tracker=app._new_tracker(640,480)
                app.tracker.select_target(box(290,'human'),100,0);app.tracking=True;app.target_selected=True
                app.camera_key=(640,480,json.dumps(app.config.get('camera',{}),sort_keys=True))
                def newest(after):
                    calls.append(after)
                    if len(calls)>2:app.stop.set();return None
                    now[0]+=.1
                    return CameraFrame(len(calls)*10,now[0],Image(),app.gimbal.snapshot())
                camera=SimpleNamespace(newest=newest,error=None)
                app.camera=camera
                if switch_to_preview:
                    def stop_during_first_face(image):
                        if detector.engines['human'].predict.call_count==1:app.tracking=False
                        return [SimpleNamespace(x0=290,y0=170,x1=350,y1=290,confidence=.9)]
                    detector.engines['human'].predict.side_effect=stop_during_first_face
                cv2=SimpleNamespace(rectangle=lambda *a:None,putText=lambda *a:None,drawMarker=lambda *a:None,
                    FONT_HERSHEY_SIMPLEX=0,MARKER_CROSS=0,IMWRITE_JPEG_QUALITY=1,
                    imencode=lambda *a:(True,SimpleNamespace(tobytes=lambda:b'jpeg')))
                with mock.patch('usb_gimbal.IndependentDetectors',return_value=detector), \
                     mock.patch.dict(sys.modules,cv2=cv2,numpy=SimpleNamespace()), \
                     mock.patch('time.monotonic',side_effect=lambda:now[0]):
                    VisionWorker(app,camera,SimpleNamespace(no_detection=False,no_face=False))._run()
                self.assertIsNone(app.error)
                self.assertEqual(calls,[0,10,20])
                self.assertEqual(app.latest.sequence,20)
                self.assertEqual(detector.engines['human'].predict.call_count,2)
                self.assertEqual(detector.engines['dog'].predict.call_count,int(switch_to_preview))

    def test_runtime_revision_and_real_operating_window_survive(self):
        with app_link() as (app,link,backend,sock):
            info=app.snapshot()['software']
            self.assertEqual((info['version'],info['revision'],info['label']),(22,4,'v22修订4'))
            self.assertEqual(link.pose_info(app)['bundle_revision'],4)
            link._arm(app,100)
            app.gimbal.set_angle('tilt',45)
            for now in (100.1,100.2,100.3,100.4):app.gimbal.tick(now)
            self.assertGreaterEqual(app.gimbal.snapshot()['axes']['tilt']['angle_deg'],90)


if __name__=='__main__':unittest.main(verbosity=2)
