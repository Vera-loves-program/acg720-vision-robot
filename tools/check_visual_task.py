"""Real Pi math/controller + independent scan/rotation references. Offline only.

The scan reference does not execute RTL. A separate HDL testbench is supplied
for the actual matcher; semantic elaboration is reported separately.
"""
from pathlib import Path
import json
import math
import random
import struct
import sys
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from visual_task import VisualTask,read_laser,world_point,approach_goal
from robot_follow import FollowLink,FpgaPWMBackend,packet
from check_robot_follow import Socket,report,box
from check_gimbal_tracking_loop import config_for
from usb_gimbal import GimbalApp,VisionFrame


def laser(seq=1, *, x=0,y=0,yaw=0,bx=1000,by=0,target=1,flags=1,status=4):
    raw=b'LDR1\x01'+struct.pack('>BHiiihhHBBHB',flags,seq,x,y,yaw,bx,by,target,1,9,0,status)
    check=0
    for byte in raw:check^=byte
    return raw+bytes([check])


def scan_reference(points,bearing=0,camera=(0,0),uncertainty=261):
    """Full Cartesian runs before cone clipping, independently in floats."""
    a=math.radians(bearing);dx,dy=math.cos(a),math.sin(a)
    runs=[]
    for p in points:
        if not runs or max(abs(p[0]-runs[-1][-1][0]),abs(p[1]-runs[-1][-1][1]))>100:
            runs.append([])
        runs[-1].append(p)
    candidates=[]
    for run in runs:
        if len(run)<3:continue
        xs,ys=zip(*run)
        if max(xs)-min(xs)>400 or max(ys)-min(ys)>400:continue
        def near(p):
            x,y=p[0]-camera[0],p[1]-camera[1]
            longitudinal=x*dx+y*dy
            return longitudinal>50 and abs(x*dy-y*dx)<longitudinal/8+uncertainty
        if any(map(near,run)):candidates.append(((min(xs)+max(xs))/2,(min(ys)+max(ys))/2))
    return candidates


class Tests(unittest.TestCase):
    def test_pose_and_laser_world_transform(self):
        for yaw,expected in [(0,(1100,200)),(90000,(100,1200)),(-90000,(100,-800)),(180000,(-900,200))]:
            p=world_point(read_laser(laser(x=100,y=200,yaw=yaw)))
            self.assertLess(math.dist(p,expected),1e-8)
        self.assertIsNone(read_laser(laser()[:-1]+b'\0'))

    def test_standoff_all_quadrants(self):
        rng=random.Random(67)
        for _ in range(3000):
            a=rng.uniform(-math.pi,math.pi);d=rng.uniform(600,3500)
            target=(d*math.cos(a),d*math.sin(a));goal=approach_goal(target,(0,0))
            self.assertAlmostEqual(math.dist(goal,target),512,delta=.71)
            self.assertLess(math.dist(goal,(0,0)),d)
        self.assertEqual(approach_goal((300,0),(0,0)),(0,0))

    def test_compact_unique_cluster_and_foreground_ambiguity(self):
        dog=[(1000,y) for y in range(-60,61,20)]
        self.assertEqual(scan_reference(dog),[(1000,0)])
        obstacle=[(600,y) for y in range(-40,41,20)]
        self.assertEqual(len(scan_reference(obstacle+dog)),2)
        self.assertEqual(scan_reference([(1000,y) for y in range(-1200,1201,20)]),[])
        self.assertEqual(scan_reference([(1000,0),(1000,20)]),[])
        left=[(-y,x) for x,y in dog]
        self.assertEqual(scan_reference(left,90),[(0,1000)])

    def test_locked_target_never_switches_to_occluding_obstacle(self):
        task=VisualTask();task.config={}
        task.accept(read_laser(laser()),10,1)
        self.assertEqual(task.lock,(1000,0))
        task.accept(read_laser(laser(seq=2,bx=600)),10.1,1)
        self.assertEqual(task.lock,(1000,0));self.assertEqual(task.state,27)
        self.assertIsNone(task.goal_body(10.1,1))
        self.assertFalse(task.query(mock.Mock(),10.1,1,True)[0]&1)
        task.cancel();self.assertIsNone(task.lock)

    def test_laser_metadata_packet_lengths_and_age(self):
        task=VisualTask();task.config={}
        task.accept(read_laser(laser()),10,1)
        body=task.goal_body(10.2,1)
        self.assertEqual(len(body),22)
        self.assertEqual(struct.unpack('>HHiiiiH',body),(1,0,488,0,1000,0,512))
        self.assertEqual(len(packet(b'GOL1',0,1,body)),32)
        self.assertIsNone(task.goal_body(10.6,1))
        self.assertIsNone(task.goal_body(10.2,2))

    def test_real_app_front_reference_does_not_modify_calibration(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
            path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)))
            before=path.read_bytes();app=GimbalApp(path,backend=FpgaPWMBackend())
            try:
                for axis,pulse in [('pan',1200),('tilt',1075)]:app.gimbal.arm(axis,pulse,physical=True)
                task=VisualTask();axes=app.gimbal.snapshot()['axes']
                app.latest=VisionFrame(1,10,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw')
                app.tracking_result=dict(target=box(280))
                task.teach_front(app,axes['pan']['angle_deg'])
                frame=VisionFrame(1,10,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw')
                app.latest=frame
                flags,body=task.query(app,10,1,True)
                self.assertEqual(len(body),22);self.assertTrue(flags&1)
                fields=struct.unpack('>HHhhhhHhhI',body)
                self.assertEqual(fields[2:4],(16384,0));self.assertEqual(fields[7],0)
                self.assertEqual(path.read_bytes(),before)
                self.assertTrue(path.with_name('visual_task.json').exists())
                # A frame with no capture-pose can never use a newer PWM pose.
                app.latest=VisionFrame(2,10,None,b'x',[],{},640,480,170,'raw')
                self.assertFalse(task.query(app,10,1,True)[0]&1)
            finally:app.close()

    def test_real_gimbal_search_finishes_and_uses_measured_limits(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
            path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)))
            app=GimbalApp(path,backend=FpgaPWMBackend())
            try:
                app.gimbal._clock=lambda:0
                for axis,pulse in [('pan',1200),('tilt',1075)]:app.gimbal.arm(axis,pulse,physical=True)
                task=VisualTask();task.start(app)
                now=0;turns=0;done=False
                while task.searching and now<200:
                    flags=64 if done else 0
                    task.search_tick(app,now,dict(flags=flags))
                    app.gimbal.tick(now)
                    if done and not task.turn_request:done=False
                    elif task.turn_request:done=True;turns+=1
                    for axis,state in app.gimbal.snapshot()['axes'].items():
                        cfg=app.gimbal.axes[axis]
                        if state['output_us'] is not None:self.assertTrue(cfg.safe_min_us<=state['output_us']<=cfg.safe_max_us)
                    now+=.1
                self.assertFalse(task.searching);self.assertEqual(turns,3)
                self.assertLess(now,180)
                self.assertEqual(task.state,22)
            finally:app.close()

    def test_fpga_integration_uses_mission_and_actual_turn_completion(self):
        src=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8')
        self.assertIn('.turn_done(nav_turn_complete)',src)
        self.assertIn('if(armed && task_motion==2 && turn_done)',src)
        self.assertIn('task_motion==3 && turn_was_running && nav_status==6',src)
        self.assertIn('.user_goal_x(page==5 ? follow_goal_x : goal_x)',src)
        self.assertIn('reg cluster_in_cone;',src)
        self.assertIn('if(!consecutive)begin',src)
        self.assertNotIn('if(!in_cone || !consecutive)',src)
        top=(ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
        self.assertIn('ui_sequence[0] ? follow_laser_telemetry : follow_telemetry',top)
        self.assertIn("ROBOT_ACTIVE ? 23'd2499999 : 23'd4999999",top)
        self.assertIn('vec_delta!=0 && !vec_delta[15]',src)

    def test_actual_pi_new_session_clears_shutdown_with_outputs_off(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
            path=Path(tmp)/'gimbal.json';path.write_text(json.dumps(config_for(upright_tilt_deg=180)))
            sock=Socket();sock.incoming[:]=[(report(128|16,42,6),('192.168.10.2',5000))]
            backend=FpgaPWMBackend();link=FollowLink(backend,tmp,sock=sock)
            app=GimbalApp(path,backend=backend,follow_link=link,standalone=True)
            try:
                with mock.patch('robot_follow.subprocess.run') as power:
                    app._control_once(100)
                    self.assertTrue(link.session_reset_pending)
                    fol=next(raw for raw,_ in sock.sent if raw[:4]==b'FOL1')
                    self.assertEqual(fol[5],128);self.assertEqual(fol[8:10],b'\0\0')
                    self.assertFalse(backend.enabled)
                    sock.incoming.append((report(128,42,0),('192.168.10.2',5000)))
                    app._control_once(100.2)
                    self.assertFalse(link.session_reset_pending)
                    self.assertFalse(backend.enabled);power.assert_not_called()
            finally:app.close()

    def test_fpga_laser_bearing_reference_all_quadrants(self):
        steps=(45000,26565,14036,7125,3576,1790,895,448,224,112,56,28,14,7,3,2)
        rng=random.Random(12)
        for _ in range(10000):
            bx,by=rng.randint(-12000,12000),rng.randint(-12000,12000)
            if math.hypot(bx,by)<50:continue
            x,y=bx*256,by*256
            angle=0
            if x<0:
                x,y=-x,-y;angle=-180000 if by<0 else 180000
            for i,step in enumerate(steps):
                x,y,angle=(x-(y>>i),y+(x>>i),angle-step) if y<0 else (x+(y>>i),y-(x>>i),angle+step)
            expected=math.degrees(math.atan2(by,bx))*1000
            error=(angle-expected+180000)%360000-180000
            self.assertLess(abs(error),15)

    def test_invalid_external_config_does_not_leave_partial_state(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as tmp:
            path=Path(tmp)/'gimbal.json'
            app=mock.Mock(config_path=path)
            for config in [[],dict(standoff_mm=512.5),dict(pan_sign=True),dict(camera_origin_mm=[0]),dict(front_reference={})]:
                path.with_name('visual_task.json').write_text(json.dumps(config),encoding='utf-8')
                task=VisualTask()
                with self.assertRaises(ValueError):task.load(app)
                self.assertIsNone(task.config)


if __name__=='__main__':unittest.main()
