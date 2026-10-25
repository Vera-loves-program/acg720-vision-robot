"""Final XY/waypoint numerical and source contracts; no hardware or HDL run."""
from pathlib import Path
import random
import unittest

ROOT=Path(__file__).resolve().parents[1]
NAV=(ROOT/'src/robot_navigation.v').read_text(encoding='utf-8')
MISSION=(ROOT/'src/robot_mission_control.v').read_text(encoding='utf-8')
SUBSYSTEM=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8')
UI=(ROOT/'src/robot_ui_control.v').read_text(encoding='utf-8')


def arrived(pose,goal,tolerance=5):
    return all(abs(p-g)<=tolerance for p,g in zip(pose,goal))


def coarse(pose):
    return tuple((coordinate+4096+64)//128 for coordinate in pose)


def choose_segment(next_fine,goal):
    goal_fine=tuple((v+4096)//64 for v in goal)
    if tuple((v+1)//2 for v in next_fine)==tuple((v+1)//2 for v in goal_fine):
        return goal,5
    return tuple(-4096+(v//2)*128 for v in next_fine),60


def done_action(pose,goal,detouring):
    if arrived(pose,goal):return 'COMPLETE'
    if not detouring or coarse(pose)==coarse(goal):return 'EXACT_FINAL'
    return 'PLAN_NEXT'


class Tests(unittest.TestCase):
    def test_300mm_not_done_at240_280_or288(self):
        goal=(300,0)
        for x in (240,280,288,289,294,306,311,360):
            self.assertFalse(arrived((x,0),goal))
            self.assertNotEqual(done_action((x,0),goal,False),'COMPLETE')
        for x in range(295,306):self.assertTrue(arrived((x,0),goal))
        self.assertFalse(arrived((300,6),goal))
        self.assertFalse(arrived((300,-6),goal))

    def test_signed_and_both_axis_boundaries(self):
        rng=random.Random(100)
        for _ in range(10000):
            gx,gy=rng.randint(-3900,3900),rng.randint(-3900,3900)
            dx,dy=rng.randint(-100,100),rng.randint(-100,100)
            self.assertEqual(arrived((gx+dx,gy+dy),(gx,gy)),max(abs(dx),abs(dy))<=5)
        for goal in ((300,0),(-300,0),(0,300),(0,-300),(300,-300)):
            for dx,dy in ((-5,-5),(5,5),(0,0)):
                self.assertTrue(arrived((goal[0]+dx,goal[1]+dy),goal))
            for dx,dy in ((-6,0),(6,0),(0,-6),(0,6)):
                self.assertFalse(arrived((goal[0]+dx,goal[1]+dy),goal))

    def test_planner_goal_cell_uses_exact_xy_not_cell_center(self):
        self.assertEqual(choose_segment((68,64),(300,0)),((300,0),5))
        self.assertEqual(choose_segment((67,64),(300,0)),((300,0),5))
        self.assertEqual(choose_segment((66,64),(300,0)),((128,0),60))
        for goal in ((-300,0),(300,-300),(-3900,3900),(3900,-3900)):
            fine=tuple((v+4096)//64 for v in goal)
            self.assertEqual(choose_segment(fine,goal),(goal,5))
        # The old cell centre288 is12mm short, so a relaxed waypoint DONE
        # requires the real final segment instead of another same-cell BFS.
        self.assertEqual(done_action((288,0),(300,0),True),'EXACT_FINAL')
        self.assertEqual(done_action((180,0),(300,0),True),'PLAN_NEXT')

    def test_approach_and_slowdown_cannot_round_both_wheels_to_zero(self):
        for remaining in range(6,1610):
            distance_speed=200 if remaining>=1600 else 1 if remaining<8 else remaining//8
            for cruise in (1,2,5,10,30,100,200):
                speed=min(distance_speed,cruise)
                slow=1 if speed<=1 else speed//2
                for selected in (speed,slow):
                    self.assertGreaterEqual(selected,1)
                    for steer in (-selected,0,selected):
                        left,right=selected-steer,selected+steer
                        self.assertGreater(left+right,0)
        self.assertIn('else if(remaining_mm<8)approach_sample<=1;',NAV)
        self.assertIn('else approach_sample<=remaining_mm[10:3];',NAV)
        self.assertIn("speed_work<=speed_work<=1 ? 11'sd1 : speed_work>>>1;",NAV)
        self.assertIn('if(turn_work==0)turn_work<=1;',NAV)
        self.assertAlmostEqual(204/1469/.01,13.887,places=3)
        self.assertGreater(5/(204/1469),35)
        self.assertEqual(min(240//8,30),30)
        self.assertLessEqual(7//8,1)

    def test_base30_units_and_separate_tapered_turn_cap12(self):
        self.assertIn("assign cruise_counts=16'd30;assign turn_counts=16'd12;",UI)
        self.assertIn("assign manual_speed_left=16'd30;assign manual_speed_right=16'd30;",UI)
        self.assertIn('.cruise_counts(cruise),.turn_counts(turn_speed)',SUBSYSTEM)
        self.assertAlmostEqual(30*204/1469/.01,416.610,places=3)
        self.assertLess(204/1469,5)

    def test_ideal_straight_encoder_quantization_can_finish_within5mm(self):
        # Independent ideal kinematics, NOT an HDL run or a claim about motor
        # inertia/friction. It checks the integer request profile has progress
        # through the5mm boundary with the configured encoder resolution.
        for goal in (6,10,30,300,1000,3900):
            position=0.0
            peak_speed=0
            for step in range(10000):
                remaining=goal-position
                if remaining<=5:break
                speed=min(30,max(1,int(remaining)//8))
                peak_speed=max(peak_speed,speed)
                position+=speed*204/1469
            else:self.fail(f'no progress at goal{goal}')
            self.assertLessEqual(abs(goal-position),5)
            self.assertLessEqual(position,goal)
            self.assertGreater(step,0)
            if goal>=300:self.assertEqual(peak_speed,30)

    def test_tolerance_is_latched_with_goal_and_wired_from_mission(self):
        self.assertIn('reg [15:0] goto_arrival_tolerance_mm;',NAV)
        self.assertIn('wire [15:0] arrival_limit=goto_arrival_tolerance_mm;',NAV)
        self.assertIn("goto_arrival_tolerance_mm<=arrival_tolerance_mm<5 ? 16'd5 : arrival_tolerance_mm;",NAV)
        self.assertIn('final_dx>=-5 && final_dx<=5 && final_dy>=-5 && final_dy<=5',MISSION)
        self.assertIn('nav_arrival_tolerance_mm<=60;detouring<=1;',MISSION)
        self.assertIn('nav_goal_x<=final_x;nav_goal_y<=final_y;',MISSION)
        self.assertIn('.arrival_tolerance_mm(mission_arrival_tolerance)',SUBSYSTEM)
        self.assertNotIn('.arrival_tolerance_mm(16\'d60)',SUBSYSTEM)

    def test_waypoint_done_is_hidden_from_lcd_pi_and_follow_completion(self):
        self.assertIn("assign mission_status=mission_active && nav_status==6 ? 4'd5 : nav_status;",SUBSYSTEM)
        self.assertIn('.nav_status(mission_status)',SUBSYSTEM)
        self.assertIn(".nav_status(planning ? 4'd10 : no_path ? 4'd11 : mission_status)",SUBSYSTEM)
        self.assertIn('pose_x-goal_x<=5 && goal_x-pose_x<=5 && pose_y-goal_y<=5 && goal_y-pose_y<=5',SUBSYSTEM)
        self.assertIn('if(nav_status==7 && same_goal_cell && !detouring)',MISSION)
        self.assertIn('no_path<=1;waiting_path<=1;',MISSION)

    def test_stop_and_existing_gyro_obstacle_contracts_preserved(self):
        self.assertIn('if(!permit)begin',MISSION)
        self.assertIn('if(safety_stop)begin',MISSION)
        self.assertIn('planning<=0;mission_active<=0;detouring<=0;wait_depart<=0;pending<=0;',MISSION)
        self.assertIn("assign recovery_speed=16'sd0",MISSION)
        self.assertNotIn('recovery_state<=1',MISSION)
        self.assertIn('avoid_enable && !gyro_only_turn && !gyro_only_turn_result',NAV)
        self.assertIn('decision_blocked_live',NAV)

    def test_coarse_waypoint_tolerance_stays_inside_validated_box(self):
        for fine in range(128):
            centre=-4096+(fine//2)*128
            origin=centre-64
            self.assertGreaterEqual(centre-60,origin)
            self.assertLess(centre+60,origin+128)
        self.assertIn("planner_next_x[6:1],7'd0",MISSION)

    def test_nav_lookahead_excludes_self_outside_corridor_and_beyond_final(self):
        def hit(point,pose,goal):
            x,y=point
            remaining=abs(goal[0]-pose[0])+abs(goal[1]-pose[1])
            return 232<=x<=632 and -139<=y<=139 and x<=remaining+252
        self.assertFalse(hit((100,0),(0,0),(1000,0)))
        self.assertFalse(hit((500,140),(0,0),(1000,0)))
        self.assertTrue(hit((600,0),(0,0),(1000,0)))
        for position in range(0,301):
            self.assertFalse(hit((800-position,0),(position,0),(300,0)))
        self.assertTrue(hit((400,0),(0,0),(300,0)))
        # Same far person must not repeatedly block a map-validated192mm
        # first leg before the path turns aside at its next coarse cell.
        self.assertTrue(hit((600,0),(0,0),(1000,0)))
        self.assertFalse(hit((600,0),(0,0),(192,64)))
        self.assertTrue(hit((350,0),(0,0),(192,64)))
        self.assertIn('point_driving && run_request && body_nav_ahead',NAV)
        self.assertIn('motion_mode==MODE_GOTO',NAV)
        self.assertIn('nav_ahead_work<=0;nav_ahead_pending<=0;nav_ahead_published<=0;',SUBSYSTEM)
        self.assertIn('if(scan_complete)nav_ahead_published<=nav_ahead_pending;',SUBSYSTEM)
        self.assertIn('map_point_valid && nav_point_x>=nav_front',SUBSYSTEM)
        self.assertIn('mission_dx=mission_goal_x-robot_x;',SUBSYSTEM)
        self.assertIn('mission_dy=mission_goal_y-robot_y;',SUBSYSTEM)
        self.assertIn('(mission_command_valid && mission_command==3)',SUBSYSTEM)

    def test_blockage_parks_before_fresh_map_then_plans_or_times_out(self):
        self.assertIn('refreshing_map<=1;refresh_phase<=0;refresh_ticks<=0;',MISSION)
        self.assertIn('else if(refreshing_map)begin',MISSION)
        self.assertIn('0:if(capture_boundary)begin',MISSION)
        self.assertIn('1:if(required_scan_integrated)begin',MISSION)
        self.assertIn('map_integrated_pulse && !integrated_after_required[15]',MISSION)
        self.assertIn('refresh_ticks==300',MISSION)
        self.assertIn('map_capture_ready(world_map_pose_valid',SUBSYSTEM)
        self.assertIn('.map_scan_count(map_scans)',SUBSYSTEM)
        self.assertIn('permit && !planning && !planner_busy && !no_path && !pending',MISSION)

    def test_mission_failure_diagnostics_do_not_reuse_old_planner_error(self):
        # A first GOTO may fail its live all-sector turn visibility check
        # before BFS ever runs. The old renderer saw stale planner_error0
        # and called this NO ROUTE. Keep those independent fault owners.
        def displayed_error(mission_error,planner_error):
            return mission_error if mission_error else planner_error
        for planner_error in (0,1,2,3,4,8,9):
            self.assertEqual(displayed_error(18,planner_error),18)
            self.assertEqual(displayed_error(13,planner_error),13)
            self.assertEqual(displayed_error(0,planner_error),planner_error)
        self.assertIn('output reg [7:0] mission_error,',MISSION)
        self.assertIn('assign waiting_map=refreshing_map;',MISSION)
        self.assertIn("mission_error<=!pose_valid ? 8'd14 : !map_ready ? 8'd15 : 8'd13;",MISSION)
        self.assertIn("mission_error<=nav_status==8 ? 8'd18 : 8'd19;",MISSION)
        self.assertIn('mission_error<=16;',MISSION)
        self.assertIn('mission_error<=17;',MISSION)
        self.assertIn('planning<=0;no_path<=0;mission_error<=0;',MISSION)

    def test_filtered_sector_ranges_never_fall_back_to_chassis_returns(self):
        for name,bit in [('front',0),('right',1),('left',2),('rear',3)]:
            self.assertIn(f"assign {name}=body_guard_ready && body_coverage[{bit}] ? filtered_{name} : 16'hffff;",SUBSYSTEM)
        self.assertIn('avoid_lidar_ready && body_guard_ready && body_coverage[0]',SUBSYSTEM)
        self.assertIn('else if(scan_complete)complete_scan_age<=0;',SUBSYSTEM)


if __name__=='__main__':unittest.main()
