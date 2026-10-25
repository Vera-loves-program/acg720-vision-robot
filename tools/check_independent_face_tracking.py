"""Actual app/link mocked-PWM regressions. No model/network/device operations."""
from contextlib import contextmanager
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from robot_follow import CandidateRegistry,FollowLink,FpgaPWMBackend
from usb_gimbal import GimbalApp,VisionFrame,IndependentDetectors,fpga_initial_tilt
from check_gimbal_tracking_loop import config_for
from check_robot_follow import Socket,report

def box(x,kind='dog',score=.9,width=60):
    return dict(x1=x,y1=170,x2=x+width,y2=290,kind=kind,score=score)

@contextmanager
def app_link(first=None):
    with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
        path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)),encoding='utf-8')
        original=path.read_bytes();sock=Socket()
        if first is not None:sock.incoming[:]=[(first,('192.168.10.2',5000))]
        backend=FpgaPWMBackend();link=FollowLink(backend,tmp,sock=sock)
        app=GimbalApp(path,backend=backend,follow_link=link,standalone=True)
        app.gimbal._clock=lambda:100
        try:yield app,link,backend,sock
        finally:
            app.close()
            assert path.read_bytes()==original,'field calibration was modified'

def feed(app,sock,sequence,now,boxes,event,op,pan=1200,tilt=1075):
    sock.incoming.append((report(0,event,op,pan=pan,tilt=tilt),('192.168.10.2',5000)))
    app.publish(VisionFrame(sequence,now,None,b'x',boxes,app.gimbal.snapshot(),640,480,170,'raw'))
    with mock.patch('time.monotonic',return_value=now):app._control_once(now)

class Tests(unittest.TestCase):
    def test_legacy_service_pose_migrates_without_relabeling(self):
        self.assertEqual(fpga_initial_tilt(2100),(2100,135.,True))
        self.assertEqual(fpga_initial_tilt(),(2100,135.,False))
        self.assertEqual(fpga_initial_tilt(None,135),(2100,135,False))
        self.assertEqual(fpga_initial_tilt(1075),(1075,None,False))
        with app_link() as (app,link,backend,sock):
            link.poll(app,100)
            self.assertFalse(backend.enabled)
            sock.incoming.append((report(0,1,11),('192.168.10.2',5000)))
            with mock.patch('time.monotonic',return_value=100.1):link.poll(app,100.1)
            self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
            self.assertAlmostEqual(app.gimbal.snapshot()['axes']['tilt']['angle_deg'],135,delta=.1)
            self.assertFalse(app.tracking)
            link.log_runtime(app)
            info=link.runtime_info
            self.assertEqual(info['software_version'],'v22')
            self.assertEqual(info['tilt_operating_min_deg'],90)
            self.assertFalse(info['physical_angle_feedback'])

    def test_kind_filter_precedes_top_four_catalog_limit(self):
        registry=CandidateRegistry()
        boxes=[box(x,'human',.99,width=30) for x in (20,160,390,530)]+[box(280,'dog',.8)]
        for seq in range(1,4):registry.update(boxes,100+seq*.1,seq,640,480)
        self.assertEqual(len(registry.catalog(100.3)),4)
        self.assertTrue(all(t['box']['kind']=='human' for _,t in registry.catalog(100.3)))
        self.assertEqual(len(registry.catalog(100.3,kind='dog')),1)

    def test_dog_prepares_before_detection_and_waits_fresh_pose_ack(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100)
            sock.incoming.append((report(0,1,5),('192.168.10.2',5000)))
            with mock.patch('time.monotonic',return_value=100.1):link.poll(app,100.1)
            self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
            self.assertTrue(link.auto_select);self.assertFalse(app.tracking)
            for seq in range(1,5):
                feed(app,sock,seq,100.5+seq*.1,[box(290)],1,5,tilt=2100)
            self.assertFalse(app.tracking)  # desired output alone is not FPGA application ACK
            for seq in range(5,8):feed(app,sock,seq,100.5+seq*.1,[box(290)],1,5)
            self.assertTrue(app.tracking,link.error)
            self.assertEqual(link.registry.tracks[link.registry.selected]['box']['kind'],'dog')

    def test_face_tracks_independently_and_warm_pan_is_preserved(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100)
            link._arm(app,100)
            app.gimbal.set_angle('pan',app.gimbal.axes['pan'].pulse_to_angle(1250))
            app.gimbal.tick(101)
            pan=backend.enabled['pan']
            self.assertNotEqual(pan,1200)
            app.detection_status=dict(state='partial',dog=False,face=True,errors={'dog':'mock dog model failed'})
            sock.incoming.append((report(0,1,9,pan=pan,tilt=1075),('192.168.10.2',5000)))
            with mock.patch('time.monotonic',return_value=101.1):link.poll(app,101.1)
            self.assertEqual(backend.enabled['pan'],pan)
            for seq in range(1,7):feed(app,sock,seq,101.2+seq*.1,[box(290,'human')],1,9,pan=pan)
            self.assertTrue(app.tracking,link.error)
            self.assertEqual(link.registry.tracks[link.registry.selected]['box']['kind'],'human')
            self.assertFalse(link.auto_reference)
            self.assertIsNone(link.follower.reference)
            link.last_sent=-1e9;link.send(app,101.9)
            fol=next(raw for raw,_ in reversed(sock.sent) if raw[:4]==b'FOL1')
            self.assertEqual(struct.unpack('>h',fol[8:10])[0],0)  # face gimbal tracking never authorizes wheels
            self.assertEqual(link.diagnostics(app,101.9)[0],43)

    def test_face_search_and_old_dog_search_remain_distinct(self):
        for op,kind in ((7,'dog'),(10,'human')):
            with app_link() as (app,link,backend,sock):
                link.poll(app,100)
                sock.incoming.append((report(0,1,op),('192.168.10.2',5000)))
                with mock.patch('time.monotonic',return_value=100.1):link.poll(app,100.1)
                self.assertTrue(link.visual.searching,link.error)
                self.assertEqual(link.visual.kind,kind)
                self.assertEqual(link.auto_kind,kind)
                self.assertGreaterEqual(app.gimbal.snapshot()['axes']['tilt']['angle_deg'],90)

    def test_first_report_never_replays_face_or_prepare_event(self):
        for op in (9,10,11):
            with app_link(first=report(128,99,op)) as (app,link,backend,sock):
                link.poll(app,100)
                self.assertFalse(backend.enabled)
                self.assertFalse(link.auto_select)
                self.assertEqual(link.event_ack,99)

    def test_model_load_failure_does_not_kill_other_class(self):
        args=SimpleNamespace(model='untouched-dog',confidence=.5,iou=.45,threads=4,
                             face_model='untouched-face',face_confidence=.6,face_iou=.3,no_face=False)
        for failed in ('dog','human'):
            good=SimpleNamespace(predict=lambda image:['healthy'])
            def fail(*args,**kwargs):raise RuntimeError('mock model failed')
            modules={'dog_inference':SimpleNamespace(DogDetector=fail if failed=='dog' else lambda *a:good),
                     'face_inference':SimpleNamespace(FaceDetector=fail if failed=='human' else lambda *a:good)}
            with mock.patch.dict(sys.modules,modules):detector=IndependentDetectors(args)
            dogs,faces,_,_=detector.predict(None)
            self.assertEqual((dogs,faces),([],['healthy']) if failed=='dog' else (['healthy'],[]))
            self.assertEqual(detector.status()['state'],'partial')

    def test_runtime_dog_failure_preserves_face_engine(self):
        args=SimpleNamespace(model='dog',confidence=.5,iou=.45,threads=4,
                             face_model='face',face_confidence=.6,face_iou=.3,no_face=False)
        bad=SimpleNamespace(predict=lambda image:(_ for _ in ()).throw(RuntimeError('mock inference failed')))
        good=SimpleNamespace(predict=lambda image:['face'])
        with mock.patch.dict(sys.modules,{'dog_inference':SimpleNamespace(DogDetector=lambda *a:bad),
                                          'face_inference':SimpleNamespace(FaceDetector=lambda *a:good)}):
            detector=IndependentDetectors(args)
        for _ in range(2):
            dogs,faces,_,_=detector.predict(None)
            self.assertEqual((dogs,faces),([],['face']))
        self.assertFalse(detector.status()['dog']);self.assertTrue(detector.status()['face'])

    def test_independent_face_does_not_load_near_vertical_laser_geometry(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100);link._arm(app,100)
            target=box(290,'human')
            snapshot=app.gimbal.snapshot();snapshot['axes']['tilt']['angle_deg']=100
            app.latest=VisionFrame(4,101,None,b'x',[target],snapshot,640,480,170,'raw')
            app.tracking=True;app.tracking_result=dict(status='tracking',target=target)
            link.registry.selected=1;link.registry.tracks[1]=dict(box=target,time=101,sequence=4,hits=3)
            link.report=dict(flags=128,gain=20,cap=5);link.report_at=101
            link.visual.config={};link.visual.reference=dict(offset=0,angle_signature=app._angle_signature())
            with self.assertRaisesRegex(ValueError,'朝天'):
                link.visual.query(app,101,1,True)  # real geometry is inapplicable at100deg
            with mock.patch.object(link.visual,'query',side_effect=AssertionError('gimbal-only queried geometry')):
                link.send(app,101)
            self.assertIsNone(link.error)
            fol=next(raw for raw,_ in sock.sent if raw[:4]==b'FOL1')
            self.assertTrue(fol[5]&1)
            self.assertFalse(fol[5]&64)
            vec=next(raw for raw,_ in sock.sent if raw[:4]==b'VEC1')
            self.assertEqual(vec[5],0);self.assertEqual(vec[8:10],b'\0\0')
            self.assertFalse(any(raw[:4]==b'GOL1' for raw,_ in sock.sent))

    def test_find_still_uses_explicit_geometry(self):
        with app_link() as (app,link,backend,sock):
            link.poll(app,100)
            link._begin_auto(app,'human',100.1,searching=True)
            link.report=dict(flags=128,gain=20,cap=5);link.report_at=100.1
            with mock.patch.object(link.visual,'query',wraps=link.visual.query) as query:
                link.send(app,100.1)
            self.assertTrue(link.geometry_active)
            query.assert_called_once()

if __name__=='__main__':unittest.main()
