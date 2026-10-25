"""Independent differential-drive plant checks; NOT HDL or real-car simulation.

Primary formula references (read2026-10-25):
https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/odometry.cpp
https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpimath/src/main/java/edu/wpi/first/math/kinematics/DifferentialDriveOdometry.java
https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/configuring_robot_localization.rst
https://github.com/STMicroelectronics/lsm6dsv16x-pid/blob/master/lsm6dsv16x_reg.c

The plant uses analytic wheel/body kinematics, first-order wheel lag, Hall
edge quantization and60Hz quantized rate samples. It deliberately has no
map corrections, slip, sensor mounting error or electrical/timing failures.
Passing these assumptions does not establish5mm physical positioning.
"""
from pathlib import Path
import math
import re
import unittest

ROOT=Path(__file__).resolve().parents[1]
NAV=(ROOT/'src/robot_navigation.v').read_text(encoding='utf-8')
MM_PER_COUNT=204/1469
DT=.01


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


def symmetric_steering(error_units,shift=13):
    return (abs(error_units)//(1<<shift))*(-1 if error_units<0 else 1)


def turn_effort(error_units,rate_dps,cap):
    predicted=error_units-rate_dps*.12*10000
    braking=(predicted<0)!=(error_units<0) and rate_dps!=0
    if braking:return 0
    return min(cap,max(1,int(abs(predicted))//16384))


def run_plant(goal=(300,0),lag=.08,left_scale=1,right_scale=1,turn_goal=None,old_turn=False):
    # World pose is independent of sensor-derived pose. Hall edges measure
    # wheel rotation; wheel circumference mismatch affects the physical plant.
    x=y=h=ex=ey=eh=vl=vr=cl=cr=0.0
    previous_l=previous_r=0
    raw_rate=0
    next_gyro=0.0
    tangent=math.atan2(goal[1],goal[0])
    mode='turn' if turn_goal is not None else 'drive'
    driving=False
    reacquiring=False
    done=None
    done_pose=None
    max_cross=0.0
    for tick in range(6000):
        dx,dy=goal[0]-ex,goal[1]-ey
        inside=max(abs(dx),abs(dy))<=5
        if done is not None:
            left_command=right_command=0
        else:
            if mode=='align' and not inside:
                mode='drive';reacquiring=True;driving=False
            if mode=='drive' and inside:mode='align'
            if mode=='turn':target=math.radians(turn_goal)
            elif mode=='align':target=tangent
            else:
                cross=dy*math.cos(tangent)-dx*math.sin(tangent)
                along=dx*math.cos(tangent)+dy*math.sin(tangent)
                max_cross=max(max_cross,abs(cross))
                if along< -5:reacquiring=True;driving=False
                target=math.atan2(dy,dx) if reacquiring else tangent+math.radians(max(-5,min(5,.25*cross)))
            error=wrap(target-eh)
            units=round(math.degrees(error)*10000)
            threshold=20000 if mode=='turn' else 10000 if mode=='align' else 5000
            if mode in ('turn','align') or not driving or abs(units)>150000:
                if abs(units)<=threshold:
                    if mode in ('turn','align'):done=tick;done_pose=(ex,ey,eh)
                    else:driving=True
                    left_command=right_command=0
                else:
                    if old_turn:effort=min(6,max(1,abs(units)//32768))
                    else:effort=turn_effort(units,raw_rate*.07,12)
                    left_command,right_command=(-effort,effort) if units>0 else (effort,-effort)
            else:
                speed=min(30,max(1,int(max(abs(dx),abs(dy)))//8))
                steering=max(-speed,min(speed,symmetric_steering(units)))
                left_command,right_command=speed-steering,speed+steering
        vl+=(left_command-vl)*min(1,DT/lag)
        vr+=(right_command-vr)*min(1,DT/lag)
        dl=vl*MM_PER_COUNT*left_scale
        dr=vr*MM_PER_COUNT*right_scale
        distance=(dl+dr)/2
        yaw_delta=(dr-dl)/213
        x+=distance*math.cos(h+yaw_delta/2)
        y+=distance*math.sin(h+yaw_delta/2)
        h+=yaw_delta
        cl+=vl;cr+=vr
        il,ir=round(cl),round(cr)
        delta_sum=il-previous_l+ir-previous_r
        previous_l,previous_r=il,ir
        if tick*DT>=next_gyro:
            raw_rate=round(math.degrees(yaw_delta/DT)/.07)
            next_gyro+=1/60
        estimated_yaw_delta=math.radians(raw_rate*.07*DT)
        estimated_distance=math.copysign(abs(delta_sum*204*128)//1469,delta_sum)/256
        ex+=estimated_distance*math.cos(eh+estimated_yaw_delta/2)
        ey+=estimated_distance*math.sin(eh+estimated_yaw_delta/2)
        eh+=estimated_yaw_delta
        if done is not None and tick-done>=75:
            return dict(time=done*DT,physical=(x,y,h),estimated=(ex,ey,eh),arrival_estimated=done_pose,
                        max_cross=max_cross,reacquiring=reacquiring)
    raise AssertionError(('did not converge',goal,lag,left_scale,right_scale,turn_goal))


class TrackingTests(unittest.TestCase):
    def test_signed_steering_is_odd_and_rejects_old_rounding_bias(self):
        self.assertEqual((-1)>>15,-1)
        self.assertEqual(1>>15,0)
        for units in range(0,1800001,113):
            self.assertEqual(symmetric_steering(-units),-symmetric_steering(units))
        self.assertEqual(symmetric_steering(-8191),0)
        self.assertEqual(symmetric_steering(8191),0)
        self.assertEqual(symmetric_steering(8192),1)
        self.assertEqual(symmetric_steering(-8192),-1)
        self.assertNotIn('steering_work<=heading_error_sample>>>15;',NAV)

    def test_odometry_units_and_midpoint_match_independent_arc(self):
        # ds=(dl+dr)/2; gyro gives angular increment, not a second translation.
        self.assertAlmostEqual(30*MM_PER_COUNT/DT,416.6099387,places=6)
        self.assertAlmostEqual(7*.1,0.7)  # .07deg/s * .01s = .7millidegree/count
        x=y=heading=0.0
        for _ in range(200):
            distance=1.0
            angular=math.pi/400
            x+=distance*math.cos(heading+angular/2)
            y+=distance*math.sin(heading+angular/2)
            heading+=angular
        radius=1/(math.pi/400)
        self.assertLess(abs(x-radius),.001)
        self.assertLess(abs(y-radius),.001)
        # Chassis nose and axle origin are different measured points.
        self.assertAlmostEqual(232*math.sin(math.radians(5)),20.220,places=3)

    def test_faster_damped_turns_settle_both_directions_under_lag(self):
        for lag in (.03,.08,.15):
            for direction in (-1,1):
                old=run_plant(lag=lag,turn_goal=direction*90,old_turn=True)
                new=run_plant(lag=lag,turn_goal=direction*90)
                self.assertLess(new['time'],old['time']*.75)
                self.assertLess(abs(math.degrees(new['physical'][2])-direction*90),2.5)
        self.assertEqual(turn_effort(100000,100,12),0)
        self.assertEqual(turn_effort(25000,0,12),1)
        self.assertEqual(turn_effort(-25000,0,12),1)

    def test_axis_segments_with_hall_quantization_gyro60hz_and_wheel_lag(self):
        for goal in ((300,0),(-300,0),(0,300),(0,-300),(300,300)):
            for lag in (.03,.08,.15):
                for scale in ((1,1),(1.03,1),(1,1.03)):
                    result=run_plant(goal,lag,*scale)
                    ax,ay,ah=result['arrival_estimated']
                    self.assertLessEqual(max(abs(goal[0]-ax),abs(goal[1]-ay)),5)
                    self.assertLessEqual(abs(math.degrees(wrap(ah-math.atan2(goal[1],goal[0])))),1)
                    ex,ey,eh=result['estimated']
                    self.assertLessEqual(max(abs(goal[0]-ex),abs(goal[1]-ey)),5.7)
                    self.assertLess(abs(math.degrees(wrap(eh-math.atan2(goal[1],goal[0])))),1.5)
                    self.assertLess(result['max_cross'],4.5)
                    self.assertFalse(result['reacquiring'])
        left=run_plant(left_scale=1.03)
        right=run_plant(right_scale=1.03)
        self.assertAlmostEqual(left['physical'][1],-right['physical'][1],places=6)

    def test_frozen_path_bounded_cross_track_and_final_only_alignment(self):
        for lateral in range(-100,101):
            correction=max(-5,min(5,.25*lateral))
            self.assertLessEqual(abs(correction),5)
            self.assertEqual(correction,-max(-5,min(5,-.25*lateral)))
        self.assertIn('goal_path_heading_valid && !goal_reacquiring && motion_mode!=MODE_ALIGN',NAV)
        self.assertIn('if(goto_arrival_tolerance_mm<=5 && goal_path_heading_valid)',NAV)
        self.assertIn('heading_target<=goal_path_heading;motion_mode<=MODE_ALIGN;',NAV)
        self.assertIn('motion_mode<=MODE_GOTO;point_driving<=0;goal_reacquiring<=1;',NAV)
        self.assertIn('if(motion_mode==MODE_TURN)turn_complete_pulse<=1;',NAV)
        self.assertEqual(len(re.findall(r'<=multiply_[xy]_input\*multiply_[xy]_factor;',NAV)),2)
        self.assertIn('if(frame_generation==command_generation && path_along_product<',NAV)

    def test_rebase_and_replacement_cannot_turn_path_pass_into_double_odometry(self):
        # A replacementGOTO invalidates the old generation but must retain the
        # old trig pass's return selector until it skips displacement state5.
        new_goal=NAV.split('goal_latched_x<=goal_x_mm;',1)[1].split('4:begin',1)[0]
        self.assertNotIn('path_trig_pending<=0;',new_goal)
        self.assertIn('path_trig_pending ? 24 : 5',NAV)
        rebase=NAV.split('if (pose_set && !run_request)',1)[1].split('if (command_valid)',1)[0]
        self.assertEqual(rebase.count('path_trig_pending<=0;goal_path_heading_valid<=0;goal_reacquiring<=0;'),2)


if __name__=='__main__':
    unittest.main()
