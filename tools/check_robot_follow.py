"""Offline packet/target/outer-loop regressions; no sockets, PWM or poweroff."""
from pathlib import Path
import binascii
import json
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pc'))
from robot_follow import CandidateRegistry, PanFollower, FpgaPWMBackend, FollowLink, packet, read_report


def box(x, kind='dog', score=.9):
    return dict(x1=x, y1=150, x2=x+80, y2=250, kind=kind, score=score)


class Socket:
    def __init__(self):
        self.sent = []
        self.incoming = [(report(),('192.168.10.2',5000))]
    def setsockopt(self, *args): pass
    def bind(self, *args): pass
    def setblocking(self, *args): pass
    def close(self): pass
    def sendto(self, raw, peer): self.sent.append((raw, peer))
    def recvfrom(self, size):
        if not self.incoming: raise BlockingIOError()
        return self.incoming.pop(0)


def report(flags=0, event=0, op=0, target=0, *, pan=1200,tilt=2100):
    raw = b'RBT1\x01'+struct.pack('>BHBHBHiHHHhhHB', flags,event,op,target,20,5,-90000,pan,tilt,8,-10,11,4,6)
    check = 0
    for byte in raw: check ^= byte
    return raw+bytes([check])


class Checks(unittest.TestCase):
    def test_real_app_offline_selection_and_fresh_frame(self):
        from check_gimbal_tracking_loop import config_for
        from usb_gimbal import GimbalApp, VisionFrame
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temporary:
            config_path=Path(temporary)/'gimbal.json'
            config_path.write_text(json.dumps(config_for(upright_tilt_deg=180)),encoding='utf-8')
            sock=Socket();backend=FpgaPWMBackend()
            link=FollowLink(backend,temporary,sock=sock)
            with mock.patch('time.monotonic',return_value=100):
                app=GimbalApp(config_path,backend=backend,follow_link=link,standalone=True)
                app.last_heartbeat=0  #PC absent; explicit standalone must work
                for seq in range(1,4):
                    app.publish(VisionFrame(seq,100,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw'))
                key=link.registry.catalog(100)[0][0]
                sock.incoming.append((report(0,1,1,key),('192.168.10.2',5000)))
                try:
                    app._control_once(100)
                    self.assertTrue(app.tracking)
                    self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
                    app.publish(VisionFrame(4,100.02,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw'))
                    app._control_once(100.02)
                    self.assertEqual(app.tracking_result['status'],'tracking')
                    self.assertEqual(link.registry.selected,key)
                    for seq in range(5,14):
                        now=100.1+(seq-5)*.1
                        sock.incoming.append((report(0,1,1,key),('192.168.10.2',5000)))
                        detections=[] if seq<9 else [box(280)]
                        app.publish(VisionFrame(seq,now,None,b'x',detections,app.gimbal.snapshot(),640,480,170,'raw'))
                        app._control_once(now)
                        if seq==8:
                            self.assertTrue(app.tracking)  #operator request stays active
                            self.assertNotEqual(app.tracking_result['status'],'tracking')
                    self.assertEqual(app.tracking_result['status'],'tracking')
                    self.assertEqual(link.registry.selected,key)
                    app._control_once(102)  #unplugged FPGA: explicitly disabled
                    self.assertFalse(app.tracking)
                    self.assertFalse(backend.enabled)
                finally:
                    app.close()

    def test_report_and_damage(self):
        raw = report(7, 23, 1, 320)
        got = read_report(raw)
        self.assertEqual((got['target'], got['yaw_mdeg'], got['left']), (320,-90000,-10))
        for index in range(32):
            changed = bytearray(raw); changed[index] ^= 1
            self.assertIsNone(read_report(changed))

    def test_candidate_reorder_and_admission(self):
        registry = CandidateRegistry()
        for seq in range(1, 4): registry.update([box(80), box(400, 'human')], seq*.1, seq, 640,480)
        self.assertEqual(len(registry.catalog(.3)), 2)
        dog_id = next(key for key,t in registry.catalog(.3) if t['box']['kind']=='dog')
        registry.selected = dog_id
        registry.update([box(402,'human'), box(83)],.4,4,640,480)
        self.assertEqual(registry.selected_boxes(4)[0]['kind'], 'dog')
        registry.update([box(84),box(86)],.5,5,640,480)
        self.assertEqual(registry.selected_boxes(5), [])  #competing same-class distractor
        registry.update([], .6,6,640,480)
        self.assertEqual(registry.selected_boxes(6), [])
        for seq in range(7,10): registry.update([box(83)],seq*.1,seq,640,480)
        self.assertEqual(len(registry.selected_boxes(9)),1)
        self.assertEqual(registry.selected, dog_id)
        self.assertEqual(registry.catalog(4), [])

    def test_live_catalog_stable_id_contract(self):
        # A page opened with no detections must accept the first populated
        # list. Slot reorder and disappearance must never change the ID the
        # operator is about to select. Explicit NEXT/REFRESH chooses again.
        registry = CandidateRegistry()
        self.assertEqual(registry.catalog(0), [])
        for seq in range(1,4):
            registry.update([box(80),box(400,'human',.8)],seq*.1,seq,640,480)
        initial = registry.catalog(.3)
        browse_id = initial[0][0]
        for seq in range(4,7):
            registry.update([box(400,'human',.99),box(80,'dog',.7)],seq*.1,seq,640,480)
        reordered = registry.catalog(.6)
        self.assertNotEqual(reordered[0][0],browse_id)
        self.assertEqual(dict(reordered)[browse_id]['box']['kind'],'dog')
        for seq in range(7,14):
            registry.update([box(400,'human',.99)],seq*.1,seq,640,480)
        missing = registry.catalog(1.3)
        self.assertNotIn(browse_id,dict(missing))
        self.assertTrue(missing)
        sub=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8-sig')
        self.assertIn('candidate[31:16]==browse_id',sub)
        self.assertIn('if(browse_id==0 && packet_hold[210:208]!=0)',sub)
        self.assertNotIn('if(page!=5 || subpage!=1 || !have_catalog)',sub)
        self.assertIn('catalog<=packet_hold;catalog_count<=packet_hold[210:208];',sub)
        self.assertIn('!motion_ready || !link_live || rx_flags[6]',sub)

    def test_pan_direction_loss_and_resume(self):
        for sign in (-1,1):
            follow = PanFollower(); follow.capture(100,8000)
            with self.assertRaises(ValueError): follow.teach(101)
            follow.teach(100+10*sign)
            for tick in range(100): result=follow.update(100+30*sign,tick*.02,True,.2,5)
            self.assertEqual(result,5)
            self.assertEqual(follow.update(130,2,False,.2,5),0)
            for tick in range(100): result=follow.update(100-30*sign,3+tick*.02,True,.2,5)
            self.assertEqual(result,-5)
            for tick in range(100): result=follow.update(101,6+tick*.02,True,.2,5)
            self.assertEqual(result,0)

    def test_wire_layout_and_disabled_start(self):
        sock = Socket()
        cfg = SimpleNamespace(safe_min_us=340,safe_max_us=2600)
        gimbal = SimpleNamespace(axes={'pan':cfg,'tilt':cfg}, snapshot=lambda: {'axes':{'pan':{'angle_deg':100},'tilt':{'angle_deg':20}}})
        app = SimpleNamespace(gimbal=gimbal, latest=None, tracking_result={}, tracking=False,error=None)
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temporary:
            link=FollowLink(FpgaPWMBackend(),temporary,sock=sock)
            link.send(app,1)
            raw=sock.sent[0][0]
            self.assertEqual(len(raw),32)
            self.assertEqual(raw[:6],b'FOL1\x01\0')
            self.assertEqual(struct.unpack('>HH',raw[10:14]),(1200,2100))
            self.assertEqual(raw[30:],struct.pack('>H',binascii.crc_hqx(raw[:30],0xffff)))
            self.assertEqual(sock.sent[1][0][:6],b'TGT1\x01\0')
            link.log.close()

    def test_shutdown_never_invoked_by_test(self):
        sock=Socket();sock.incoming.append((report(16,2,6),('192.168.10.2',5000)))
        backend=FpgaPWMBackend()
        app=SimpleNamespace(tracking=True, _hold_tracking=lambda:None)
        app.gimbal=SimpleNamespace(disable=backend.enabled.clear)
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temporary:
            link=FollowLink(backend,temporary,sock=sock)
            with mock.patch('robot_follow.subprocess.run',return_value=SimpleNamespace(returncode=1,stderr='permission missing')) as run:
                link.poll(app,1)
                link.poll(app,1.1)
            self.assertEqual(run.call_count,1)  #permission probe only; no repeated request
            self.assertFalse(link.shutdown_accepted)
            self.assertFalse(app.tracking)
            self.assertIn('关机权限',link.error)
            link.log.close()
            lines=[json.loads(x) for x in link.log_path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(lines[-1]['event'],'shutdown_failed')

    def test_poweroff_accepted_mock_only(self):
        cfg=SimpleNamespace(safe_min_us=340,safe_max_us=2600)
        backend=FpgaPWMBackend()
        app=SimpleNamespace(tracking=True,_hold_tracking=lambda:None,latest=None,
                            tracking_result={},error=None)
        app.gimbal=SimpleNamespace(disable=backend.enabled.clear,axes={'pan':cfg,'tilt':cfg},
                                  snapshot=lambda:{'axes':{'pan':{'angle_deg':100},'tilt':{'angle_deg':20}}})
        sock=Socket();sock.incoming.append((report(16,2,6),('192.168.10.2',5000)))
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temporary:
            link=FollowLink(backend,temporary,sock=sock)
            try:
                with mock.patch('robot_follow.subprocess.run',return_value=SimpleNamespace(returncode=0,stderr='')) as run:
                    link.poll(app,1)
                self.assertEqual(run.call_count,2)
                self.assertEqual(run.call_args.args[0],['sudo','-n','/usr/bin/systemctl','poweroff'])
                self.assertTrue(link.shutdown_accepted)
                self.assertTrue(sock.sent[0][0][5] & 32)
                self.assertFalse(sock.sent[0][0][5] & 24)  #no enabled PWM in ACK
            finally: link.log.close()

    def test_rtl_contracts(self):
        top=(ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8-sig')
        sub=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8-sig')
        nav=(ROOT/'src/robot_navigation.v').read_text(encoding='utf-8-sig')
        self.assertIn('parameter integer SEND_CAMERA_UDP=0',top)
        self.assertIn('u_framebuffer',top)
        self.assertIn('.follow_pan_pwm(follow_pan_pwm)',top)
        self.assertIn('ui_sequence[0] ? follow_laser_telemetry : follow_telemetry',top)
        self.assertIn('event_ack==event_seq',sub)
        # Contact STOP suppresses the later tap event in robot_ui_control.
        # It must revoke the follow owner through the immediate PID STOP path.
        self.assertIn('.cancel(remote_takeover || explicit_local_stop || ui_motor_fault',sub)
        self.assertIn('rx_target==selected_id',sub)
        self.assertNotIn('shutdown_ticks==2999',sub)
        self.assertNotIn('wire motor_safety_stop=pi_shutdown_requested',top)
        self.assertIn('software_stopped<=0',sub)
        live=nav.split('if(command==5 && motion_mode==MODE_MANUAL)begin',1)[1].split('end else begin',1)[0]
        self.assertNotIn('heading_target<=',live)
        self.assertNotIn('command_generation<=',live)


if __name__=='__main__': unittest.main()
