"""Task UI, coordinate and network-recovery regressions; no device access.

Python tests execute the real Pi controller with a fake UDP socket. RTL
contracts and independent physical references do not constitute HDL simulation.
"""
from pathlib import Path
from fractions import Fraction
import json
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pc'))
from robot_follow import FollowLink,FpgaPWMBackend
from check_robot_follow import Socket,report,box
from robot_ui_spec import VIEWS,button_rects,hit_test,route_for,generate_panel


class FlakySocket(Socket):
    fail=False
    def sendto(self,raw,peer):
        if self.fail:raise OSError(101,'Network is unreachable')
        super().sendto(raw,peer)


class Tests(unittest.TestCase):
    def test_six_direct_task_routes(self):
        expected=[(0,1),(4,0),(2,3),(4,1),(5,0),(5,3)]
        self.assertEqual([route_for((0,0),i) for i in range(1,7)],expected)
        self.assertTrue(all(not any('ARM' in c for c in v.captions) for v in VIEWS.values()))
        self.assertEqual(route_for((4,1),3),None)  # Origin is an action, never an editor.
        self.assertEqual(route_for((4,1),5),(3,3))
        self.assertEqual(route_for((3,3),6),(4,1))
        self.assertEqual(route_for((3,0),1),None)

    def test_every_active_button_and_blank_card(self):
        for key,v in VIEWS.items():
            for button,(x0,y0,x1,y1) in button_rects(*key).items():
                self.assertEqual(hit_test((x0+x1)//2,(y0+y1)//2,*key),button,(key,button))
        for key,slots in [((5,0),(4,5,6)),((5,4),(6,))]:
            for button in slots:
                x=408 if button%2 else 608
                y=128+((button-1)//2)*88
                self.assertEqual(hit_test(x+40,y+20,*key),0)
        self.assertEqual((ROOT/'src/robot_control_panel.v').read_text(encoding='utf-8'),generate_panel())

    def test_axle_and_lidar_coordinates_are_equivalent(self):
        f,b,l,r=map(Fraction,('231.27','117.39','119','119'))
        ox,oy=Fraction('32.56'),Fraction('-77.35')
        self.assertEqual((f-ox,b+ox,l-oy,r+oy),tuple(map(Fraction,('198.71','149.95','196.35','41.65'))))
        # A raw ray reaching each edge must transform to THAT edge in O frame.
        self.assertEqual(ox+(f-ox),f)
        self.assertEqual(ox-(b+ox),-b)
        self.assertEqual(oy+(l-oy),l)
        self.assertEqual(oy-(r+oy),-r)

    def test_turn_and_goto_source_contracts(self):
        nav=(ROOT/'src/robot_navigation.v').read_text(encoding='utf-8')
        mission=(ROOT/'src/robot_mission_control.v').read_text(encoding='utf-8')
        ui=(ROOT/'src/robot_ui_control.v').read_text(encoding='utf-8')
        self.assertIn('avoid_enable && !gyro_only_turn && !gyro_only_turn_result',nav)
        self.assertIn('motion_mode!=MODE_TURN',nav)
        self.assertNotIn('recovery_state<=1',mission)
        self.assertIn("assign recovery_speed=16'sd0",mission)
        self.assertIn('rebase_pending<=3',ui)
        self.assertIn('if(rebase_pending==1)',ui)
        self.assertIn("assign pose_x_mm=32'sd0;assign pose_y_mm=32'sd0;assign pose_heading_mdeg=32'sd0;",ui)
        self.assertIn("assign geometry_valid=1'b1;assign wheel_mm_per_rev=16'd204;",ui)

    def test_position_seed_is_not_used_for_local_navigation(self):
        tuning=(ROOT/'src/motor_tuning_control.v').read_text(encoding='utf-8')
        top=(ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
        self.assertIn('position_kp_left<=32; position_ki_left<=0; position_kd_left<=0;',tuning)
        self.assertIn("wire selected_mode_now=robot_local_drive ? 1'b0 : motor_mode_position;",top)
        self.assertIn("selected_mode_q<=selected_mode_now;",top)
        self.assertIn(".mode_position(ROBOT_ACTIVE ? selected_mode_q : selected_mode_now)",top)
        self.assertNotIn('wire motor_safety_stop=pi_shutdown_requested',top)

    def test_network_error_is_recoverable_in_real_app(self):
        from check_gimbal_tracking_loop import config_for
        from usb_gimbal import GimbalApp,VisionFrame
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temp:
            cfg=Path(temp)/'gimbal.json'
            cfg.write_text(json.dumps(config_for(upright_tilt_deg=180)),encoding='utf-8')
            sock=FlakySocket();sock.fail=True
            backend=FpgaPWMBackend();link=FollowLink(backend,temp,sock=sock)
            app=GimbalApp(cfg,backend=backend,follow_link=link,standalone=True)
            try:
                for seq in range(1,4):
                    app.publish(VisionFrame(seq,100,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw'))
                app._control_once(100)
                self.assertIsNone(app.control_error)
                self.assertIn('unreachable',link.transport_error)
                self.assertFalse(backend.enabled)
                sock.fail=False
                app._control_once(100.1)
                self.assertIsNone(link.transport_error)
                self.assertFalse(backend.enabled)  # Reconnection grants no output.
                sock.incoming.append((report(0,2,5),('192.168.10.2',5000)))
                with mock.patch('time.monotonic',return_value=100.2):
                    app._control_once(100.2)
                self.assertFalse(app.tracking)  # Upright preparation precedes target admission.
                self.assertEqual(backend.enabled,dict(pan=1200,tilt=1075))
                for seq,now in ((4,100.6),(5,100.8),(6,101.0)):
                    sock.incoming.append((report(0,2,5,pan=1200,tilt=1075),('192.168.10.2',5000)))
                    app.publish(VisionFrame(seq,now,None,b'x',[box(280)],app.gimbal.snapshot(),640,480,170,'raw'))
                    with mock.patch('time.monotonic',return_value=now):app._control_once(now)
                self.assertTrue(app.tracking,link.error)
                self.assertIsNone(app.control_error)
            finally:app.close()

    def test_first_report_never_replays_old_selection(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temp:
            sock=Socket();sock.incoming[:]=[(report(0,99,1,45),('192.168.10.2',5000))]
            link=FollowLink(FpgaPWMBackend(),temp,sock=sock)
            try:
                link.poll(SimpleNamespace(),100)
                self.assertIsNone(link.error)
                self.assertEqual(link.registry.selected,0)
                self.assertEqual(link.event_ack,99)
            finally:link.log.close()

    def test_poweroff_failure_can_retry_but_not_on_every_report(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tmp') as temp:
            sock=Socket();backend=FpgaPWMBackend()
            link=FollowLink(backend,temp,sock=sock)
            app=SimpleNamespace(tracking=False,_hold_tracking=lambda:None,
                                gimbal=SimpleNamespace(disable=backend.enabled.clear))
            try:
                with mock.patch('robot_follow.subprocess.run',return_value=SimpleNamespace(returncode=1,stderr='denied')) as run:
                    link.poll(app,100)
                    for event in (1,1,2):
                        sock.incoming.append((report(16,event,6),('192.168.10.2',5000)))
                        link.poll(app,100.1)
                self.assertEqual(run.call_count,2)
                self.assertEqual(link.error_code,12)
                self.assertFalse(link.shutdown_started)
            finally:link.log.close()

    def test_unknown_footprint_has_distinct_diagnostics(self):
        source=(ROOT/'src/lidar_path_planner.v').read_text(encoding='utf-8')
        self.assertIn("build_unknown<=plan_rd_cell==0",source)
        self.assertIn("node_read_data[2] ? 8'd8 : 8'd2",source)
        self.assertIn("node_read_data[2] ? 8'd9 : 8'd3",source)


if __name__=='__main__':unittest.main()
