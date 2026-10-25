"""Robot source contracts and independent unit/layout arithmetic only.

Does not compile, elaborate, clock or simulate HDL. Run the individual C1,
navigation, grid and path checkers for their independent algorithm models.
"""
from pathlib import Path
from fractions import Fraction
from itertools import product
import math
import re
from check_motor_integration import uncomment, strings, check_blocks, closing
from robot_ui_spec import VIEWS, button_rects, hit_test

ROOT = Path(__file__).resolve().parents[1]

def read(name):
    return uncomment((ROOT / 'src' / name).read_text(encoding='utf-8-sig'))

top, sub, ui, status, mission, panel, uart, lcd = map(read, (
    'vision_robot_ui_top.v', 'robot_subsystem.v', 'robot_ui_control.v',
    'robot_status_text.v', 'robot_mission_control.v', 'robot_control_panel.v',
    'sensor_demo_uart.v', 'lcd1024_interaction_ui.v'))
for name in ('robot_ui_control', 'robot_status_text', 'robot_control_panel',
             'robot_mission_control', 'robot_subsystem'):
    check_blocks(read(name + '.v'), name)

def compact(code):
    return re.sub(r'\s+', '', code)

t,s,u,m = map(compact, (top,sub,ui,mission))


def instance_connections(source, name):
    """Named-port expressions only; this is not Verilog elaboration."""
    found=re.search(r'\b'+re.escape(name)+r'\s*\(',source)
    assert found,name
    start=source.index('(',found.start())
    body=source[start+1:closing(source,start)]
    result={}
    for match in re.finditer(r'\.\s*(\w+)\s*\(',body):
        opening=body.index('(',match.start())
        assert match[1] not in result,(name,match[1])
        result[match[1]]=compact(body[opening+1:closing(body,opening)])
    return result


def check_core_profile():
    """Current six-task build: check live wiring, not removed legacy menus."""
    nav=compact(read('robot_navigation.v'))
    adapter=compact(read('motor_tuning_control.v'))
    codec=compact(read('motor_uart_commands.v'))
    st,p,l=map(compact,(status,panel,lcd))
    def tokens(code, required, owner):
        for token in required:assert token in code,(owner,token)
    def ports(source,name,expected):
        actual=instance_connections(source,name)
        for key,value in expected.items():assert actual.get(key)==value,(name,key,actual.get(key),value)
        return actual

    # Real pruning is behind constant generate conditions. Camera/DDR, IMU,
    # motor bridge, receive UDP and the Pi control link remain instantiated.
    tokens(t,('parameterintegerACTUATOR_TEST=0','parameterintegerROBOT_MODE=1',
        'parameterintegerSEND_CAMERA_UDP=0','generateif(!ROBOT_ACTIVE)begin:legacy_motor_report',
        'endelsebegin:core_no_motor_dashboard',"assignmotor_byte_valid=1'b0;",
        'generateif(!ROBOT_ACTIVE)begin:legacy_clock_display',
        'generateif(SEND_CAMERA_UDP!=0)begin:camera_udp_fifo',
        'bq34z100_gauge#(.ENABLE_SETUP(!ROBOT_ACTIVE))',
        'bq34z100_status_text#(.MINIMAL(ROBOT_ACTIVE))'), 'core pruning')
    for name in ('u_camera_init','u_dvp','u_framebuffer','u_imu_demo','u_motors','u_udp_rx','u_robot','u_touch'):
        assert instance_connections(top,name),name
    ports(top,'u_framebuffer',{'wr_clk':'camera_pclk','wfifo_wren':'gaussian_valid',
        'wfifo_din':'gaussian_pixel','rd_clk':'pixel_clk','rfifo_rden':'camera_pop','rfifo_dout':'camera_pixel_read'})
    ports(top,'u_lcd',{'camera_pixel':'camera_pixel_read','camera_ready':'camera_pixel_sync[1]',
        'robot_control':'ROBOT_ACTIVE','camera_pop':'camera_pop'})
    ports(top,'u_robot',{'gyro_z':'imu_gz','gyro_valid':'imu_demo_valid','gyro_samples':'imu_demo_samples',
        'position_left':'motor_position_left','position_right':'motor_position_right',
        'speed_left':'motor_speed_left','speed_right':'motor_speed_right','tick_10ms':'motor_tick',
        'lidar_rx':'lidar_rx','lidar_tx':'lidar_tx','follow_request':'follow_mailbox_request',
        'follow_data':'follow_mailbox_data','follow_ack':'follow_mailbox_ack',
        'shutdown_requested':'pi_shutdown_requested'})
    tokens(t,('wirecamera_capture_reset_n=ready&&camera_init_done&&init_calib_complete;',
        'assignTFT_pwm=ready;'), 'camera remains independent of Pi shutdown')

    # One UART transaction -> adapter -> same LCD action owner -> response.
    # Check all seven handshake/data wires at BOTH top-level endpoints and
    # through the subsystem; no parallel raw-wheel task controller exists.
    task_ports={name:'vofa_'+name for name in ('task_valid','task_action','task_value',
        'task_ready','task_rsp_valid','task_rsp_status','task_rsp_value')}
    ports(top,'u_vofa_settings',task_ports);ports(top,'u_robot',task_ports)
    ports(sub,'u_controls',{**{name:name for name in task_ports},'task_visual_ready':'task_visual_ready'})
    ports(sub,'u_follow',{'task_ready_flags':'task_visual_ready','tap_valid':'ui_tap_valid','tap_button':'ui_tap',
        'cancel':'remote_takeover||explicit_local_stop||ui_motor_fault||!touch_ready||(snapshot_valid&&snapshot_cancel)'})
    uart_ports=ports(top,'u_vofa',{'byte_valid':'demo_rx_byte_valid','byte_data':'demo_rx_byte',
        'emergency_stop_pulse':'vofa_emergency','cmd_valid':'vofa_cmd_valid','cmd_ready':'vofa_cmd_ready'})
    backend=instance_connections(top,'u_vofa_settings')
    for name in ('cmd_seq','cmd_op','cmd_wheel','cmd_field','cmd_value','rsp_valid','rsp_ready',
                 'rsp_seq','rsp_op','rsp_wheel','rsp_field','rsp_status','rsp_value'):
        assert uart_ports[name]==backend[name],name
    tokens(codec,('1:if(magnitude<=9&&!negative)parsed_op<=magnitude[3:0];',
        'regoutstanding;','wiregain_packet=(parsed_op==2||parsed_op==3)&&parsed_field<=2;'), 'VOFA codec')
    tokens(adapter,('assigntask_valid=state==TASK_SEND;','assigntask_action=held_field;',
        'assigntask_value=held_value;','9:if(cmd_wheel!=0||cmd_field>8||',
        'TASK_SEND:begin','TASK_WAIT:begin','if(task_rsp_valid)begin',
        'transaction_error<=TASK_TIMEOUT;state<=WAIT_ONE;'), 'VOFA serialized task backend')
    tokens(u,('if(task_valid&&task_ready)begin','task_held_action<=task_action;',
        '0:beginaction<=3;action_valid<=1;end','1:goal_x_mm<=task_held_value;',
        '2:goal_y_mm<=task_held_value;',
        '3:if(gains_ready&&gyro_ready&&lidar_ready)beginaction<=4;action_valid<=1;',
        '4:if(task_visual_ready[0])beginaction<=1;action_valid<=1;',
        '5:if(task_visual_ready[1])beginaction<=2;action_valid<=1;',
        '6:if(task_visual_ready[2])beginaction<=3;action_valid<=1;',
        '8:if(task_visual_ready[0])beginaction<=2;action_valid<=1;',
        '7:beginaction<=6;action_valid<=1;armed<=0;command<=0;command_valid<=1;pid_action<=10;pid_action_valid<=1;end',
        'if(task_action!=5&&task_action!=6)begin',
        'rebase_pending==0&&!pose_set','task_rsp_valid<=1;'), 'same UI task actions / STOP / zero ACK')

    # The motor bridge has one target mux; a running remote motor owner wins
    # over local navigation. STOP propagates to Pi/search as well as wheels.
    tokens(t,('assignrobot_local_drive=robot_drive_enable&&!(motor_remote_owner&&motor_armed&&motor_running);',
        '.core_stop_override(settings_ui_stop)',
        '||vofa_emergency||test_emergency_stop||'), 'motor ownership / emergency')
    ports(top,'u_motors',{'run_enable':'motor_request_now&&(!ROBOT_ACTIVE||selected_snapshot_ready)',
        'target_speed_left':"ROBOT_ACTIVE?selected_speed_left_q:(robot_local_drive?robot_target_left:motor_target_speed_left)",
        'target_speed_right':"ROBOT_ACTIVE?selected_speed_right_q:(robot_local_drive?robot_target_right:motor_target_speed_right)",
        'safety_stop':'motor_safety_stop||motor_link_fault'})
    tokens(s,('wireexplicit_local_stop=pid_action_valid&&(pid_action==10||pid_action==11);',
        'assignarmed=ui_armed||follow_armed;','software_stopped<=0;',
        'robot_follow_control#(.ENABLE_CHOOSER(0))u_follow('), 'core owners')
    ports(sub,'u_navigation',{'avoid_enable':'!manual_view&&!remote_turn_active',
        'cruise_counts':'cruise','turn_counts':'turn_speed','arrival_tolerance_mm':'mission_arrival_tolerance',
        'body_nav_ahead':'nav_obstacle_ahead','lidar_valid':'avoid_lidar_ready'})
    tokens(u,('assignmanual_speed_left=16\'d30;assignmanual_speed_right=16\'d30;',
        'assigncruise_counts=16\'d30;assignturn_counts=16\'d12;',
        'assigngeometry_valid=1\'b1;','assignwheel_mm_per_rev=16\'d204;',
        'assignencoder_counts_per_rev=16\'d1469;assigntrack_mm=16\'d213;',
        'rebase_pending<=3;','pose_set<=1;map_clear<=1;'), 'fixed geometry / direct movement / zero')
    assert 'start_delay_ticks' not in u and 'local_arm_ok' not in u

    # Final XY and waypoint DONE are distinct, and dynamic replanning waits
    # for new stationary map integrations. Raw node traffic is not scan age.
    tokens(nav,('goto_arrival_tolerance_mm<=arrival_tolerance_mm<5?16\'d5:arrival_tolerance_mm;',
        'approach_sample<=remaining_mm[10:3];','point_driving&&run_request&&body_nav_ahead',
        'avoid_enable&&!gyro_only_turn&&!gyro_only_turn_result'), 'precise NAV')
    tokens(m,('final_dx>=-5&&final_dx<=5&&final_dy>=-5&&final_dy<=5',
        'nav_arrival_tolerance_mm<=60;detouring<=1;',"planner_next_x[6:1],7'd0",
        'else if(refreshing_map)begin'.replace(' ',''),'0:if(capture_boundary)begin',
        '1:if(required_scan_integrated)begin','map_integrated_pulse&&!integrated_after_required[15]',
        'mission_active<=user_command==3;','reusable_map&&map_integrated_pulse',
        'if(!permit)begin','if(safety_stop)begin','waiting_path<=1;',
        'refresh_ticks==300',"assignrecovery_speed=16'sd0;"), 'mission precision / fresh map')
    tokens(s,('assignmission_status=mission_active&&nav_status==6?4\'d5:nav_status;',
        'assignavoid_lidar_ready=lidar_ready&&complete_scan_age<30;',
        'elseif(scan_complete)complete_scan_age<=0;','mission_dx=mission_goal_x-robot_x;',
        'mission_dy=mission_goal_y-robot_y;'), 'navigation integration')
    for name,bit in (('front',0),('right',1),('left',2),('rear',3)):
        assert f"assign{name}=body_guard_ready&&body_coverage[{bit}]?filtered_{name}:16'hffff;" in s

    # The active UI has six tasks plus four auxiliary views, no PID/calibration
    # pages. The byte renderer still freezes flags and values as one frame.
    assert set(VIEWS)=={(0,0),(0,1),(2,1),(2,3),(3,3),(4,0),(4,1),(5,0),(5,3),(5,4)}
    assert len(VIEWS[(0,0)].captions)==6
    assert not any('ARM' in caption for view in VIEWS.values() for caption in view.captions)
    tokens(st,('regsigned[19:0]values[0:5];','held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,lidar_ready,calibrating,gyro_calibrated,geometry_valid,armed,page};',
        'S_COMMIT:beginflags<=held_flags;text_valid<=1;state<=S_IDLE;end'), 'compact atomic status')
    assert 'robot_status_text#(.PARALLEL_COMPAT(0))u_status(' in s
    ports(top,'u_robot_lcd_text',{'byte_valid':'ROBOT_ACTIVE&&robot_byte_valid&&robot_byte_ready'})
    ports(top,'u_robot',{'status_byte_ready':"ROBOT_ACTIVE?(robot_byte_ready&&robot_lcd_byte_ready):1'b1"})
    tokens(p,("map_x_wide=(x-11'd31)>>1;","map_y_wide=(10'd367-y)>>1;"), '2-pixel map prefetch')
    map_cases=0
    for x,y in product(range(32,288),range(112,368)):
        assert ((x-1)-31)//2==(x-32)//2
        assert 0<=(367-y)//2<128
        map_cases+=1
    owner_cases=0
    for test,remote_owner,armed,running,local,stop in product((False,True),repeat=6):
        selected=local and not(remote_owner and armed and running)
        enabled=not test and not stop and ((armed and running) or selected)
        if stop or test:assert not enabled
        if remote_owner and armed and running:assert not selected
        owner_cases+=1
    print('PASS: current six-task core only; pruned dashboards/RTC/camera-TX FIFO, retained FPGA camera/DDR/IMU/drive/network')
    print('PASS: named-port VOFA codec/backend/UI task and response chain; shared motion owner, Pi/search STOP cancellation')
    print('PASS: final5mm, cruise30/turncap12, exactXY/coarse waypoint, fresh-map barrier and filtered-sector source contracts')
    print(f'PASS: compact atomic text and {map_cases} map-prefetch coordinates; {owner_cases} motor-owner arithmetic cases')
    print('Legacy21-view ARM/delayed-start/PID-page contracts were NOT run for this core profile.')
    print('Static source and independent arithmetic checks only; no HDL execution, timing/resource-fit or hardware validation.')


if 'core_no_motor_dashboard' in top:
    check_core_profile()
    raise SystemExit(0)

# Historical expanded UI profile only. These assertions intentionally remain
# separate: removing obsolete features is not evidence that their old tests pass.
assert 'parameterintegerACTUATOR_TEST=0' in t
assert 'parameterintegerROBOT_MODE=1' in t
assert 'parameterintegerSEND_CAMERA_UDP=0' in t
assert '.pixel_valid(SEND_CAMERA_UDP?gaussian_valid:1\'b0)' in t
assert 'cs100a_demou_' not in t
assert "assignultrasonic_trig=1'b0;" in t
assert '.lidar_rx(lidar_rx),.lidar_tx(lidar_tx)' in t
assert '.motor_byte_valid(!ACTUATOR_TEST&&motor_byte_valid&&(!ROBOT_ACTIVE||motor_lcd_byte_ready))' in t
assert '.text_byte_ready(ACTUATOR_TEST?1\'b1:(motor_byte_ready&&(!ROBOT_ACTIVE||motor_lcd_byte_ready)))' in t
assert '.robot_byte_valid(ROBOT_ACTIVE&&robot_byte_valid&&robot_lcd_byte_ready)' in t
assert '.status_byte_ready(ROBOT_ACTIVE?(robot_byte_ready&&robot_lcd_byte_ready):1\'b1)' in t
assert '.motor_text_data(' not in t and '.robot_text_data(' not in t
assert '.uart_action_valid(vofa_action_valid||(vofa_cmd_ready&&!vofa_cmd_valid&&motor_uart_action_valid&&(!ROBOT_ACTIVE||(robot_page==1&&!robot_calibration_enable))))' in t
assert '.run_enable(!ACTUATOR_TEST&&((motor_armed&&motor_running)||(ROBOT_ACTIVE&&robot_local_drive)))' in t
assert 'assignrobot_local_drive=robot_drive_enable&&!(motor_remote_owner&&motor_armed&&motor_running);' in t
assert '.ui_action_valid(settings_ui_valid&&(vofa_cmd_ready||settings_ui_stop))' in t
assert 'wiresettings_ui_stop=settings_ui_valid&&(settings_ui_action==10||settings_ui_action==11);' in t
assert '.core_stop_override(settings_ui_stop)' in t
assert '.uart_live_update(vofa_action_valid&&vofa_live_update)' in t
assert '.remote_takeover(vofa_takeover)' in t
assert '.remote_takeover(remote_takeover)' in s
assert '.remote_active((motor_remote_owner&&motor_armed&&motor_running)||robot_turn_active)' in t
assert '.remote_active(remote_active)' in s
assert '.calibration_enable(ROBOT_ACTIVE&&robot_calibration_enable)' in t
assert '.safety_stop(motor_safety_stop||motor_link_fault)' in t
assert '.snapshot_cancel(touch_snapshot_cancel||pid_page||ACTUATOR_TEST||ROBOT_ACTIVE)' in t
assert 'vofa_emergency' in t.split('wiremotor_safety_stop=',1)[1].split(';',1)[0]
assert '||vofa_emergency||test_emergency_stop||' in t
assert 'assignbluetooth_tx=ROBOT_ACTIVE?vofa_tx:legacy_bluetooth_tx;' in t
assert 'robot_vofa_uart#(.CHANNELS(4))u_vofa(' in t
assert 'motor_status_text#(.COMPACT_ROBOT(ROBOT_ACTIVE))u_motor_text(' in t
vofa_only=t.split('generateif(ROBOT_ACTIVE)begin:vofa_only_commands',1)[1].split('endelsebegin:legacy_motor_commands',1)[0]
for assignment in ("assignmotor_uart_action_valid=1'b0;", "assignmotor_uart_action=8'd0;",
                   "assignmotor_uart_argument=32'sd0;", "assignmotor_uart_emergency=1'b0;",
                   "assignmotor_uart_heartbeat=1'b0;", "assignmotor_parse_errors=32'd0;"):
    assert assignment in vofa_only,assignment
assert 'motor_uart_commandsu_motor_commands(' not in vofa_only
assert 'sensor_demo_commandsu_demo_commands(' in t  # shared UART RX is retained
assert '.cmd_value(vofa_cmd_value)' in t and '.rsp_value(vofa_rsp_value)' in t
assert '.core_selected_value(motor_selected_value)' in t

# Legacy wide CDC remains held until ACK. The active robot build instead
# transfers indexed bytes into independently owned RAM banks; its receiving
# page and text still commit together in vertical blanking.
assert 'if(!ROBOT_ACTIVE&&robot_text_valid&&robot_status_request==robot_status_ack_sync[2])' in t
assert 'if(!ROBOT_ACTIVE&&motor_text_valid&&pid_text_request==pid_text_ack_sync[2])' in t
assert 'lcd1024_interaction_ui#(.BYTE_TEXT_MODE(ROBOT_ACTIVE))u_lcd' in t
assert 'robot_status_ack_sync<={robot_status_ack_sync[1:0],robot_status_ack};' in t
assert 'robot_status_request==robot_status_ack_sync[2]' in t
assert 'robot_status_mailbox<={robot_flags,robot_text};' in t
l=compact(lcd)
assert 'if(!BYTE_TEXT_MODE&&frame_restart&&robot_request_sync[2]!=robot_status_ack)' in l
assert 'robot_status<=robot_status_data;robot_status_ack<=robot_request_sync[2];' in l

# Manual raw-count commissioning and obstacle-aware navigation have separate
# owners. Raw driving requires ARM, live touch and no actual fault; zero PID
# gains cause zero output rather than an additional permission failure.
# Navigation still requires ready gains and its independent run_request.
assert 'wiremanual_view=page==4&&subpage==0;' in s
assert 'wiremanual_permit=armed&&manual_view&&touch_ready&&!safety_stop&&!ui_motor_fault;' in s
assert 'wirelocal_permit=armed&&page==4&&subpage==1&&touch_ready&&gains_ready&&!safety_stop&&!ui_motor_fault;' in s
assert '.motion_enable(remote_turn_permit||(page==5?follow_permit:manual_view?manual_permit:nav_permit))' in s
assert 'assigndrive_enable=(remote_turn_permit&&nav_run)||(manual_permit&&(manual_active||(manual_turn_active&&nav_run)))||(local_permit&&nav_permit&&nav_run)||(follow_permit&&nav_run);' in s
assert '.avoid_enable(!manual_view&&!remote_turn_active)' in s
assert 'if(!manual_permit||map_reset)begin' in s
assert '.start(planner_start&&!planner_cancel&&!map_reset)' in s
assert '.cancel(planner_cancel||map_reset)' in s
assert '.safety_stop(safety_stop||map_reset)' in s
assert '.lidar_correction_valid(correction_valid&&!manual_active&&!manual_turn_active&&!remote_turn_active&&!clear_map&&!pose_set&&!encoder_zeroed)' in s
assert '.correction_enable(!map_local&&world_map_pose_valid&&!manual_active&&!manual_turn_active&&!remote_turn_active&&!map_reset)' in s
assert '.pose_valid(world_map_pose_valid),.local_scan(map_local),.pose_x_mm(map_pose_x)' in s
assert '.pose_y_mm(map_pose_y),.pose_heading_q6(map_pose_heading)' in s
assert '.lidar_valid(avoid_lidar_ready)' in s
assert 'assignavoid_lidar_ready=lidar_ready&&complete_scan_age<30;' in s
assert '.correction_ack(correction_ack)' in s
assert 'gyro_valid&&static_gyro_age<20' in s
assert 'geometry_valid<=0;wheel_mm_per_rev<=204;encoder_counts_per_rev<=1469;track_mm<=213;' in u
assert 'low=-180000;high=179999;' in u
assert 'sum=heading+delta;' in u
assert 'if(sum>=180000)turn_relative=sum-360000;' in u
assert 'elseif(sum<-180000)turn_relative=sum+360000;' in u
assert "delayed_turn_delta<=180000;delayed_command<=2;" in u
assert "turn_relative(live_heading_mdeg,delayed_turn_delta)" in u
assert "start_delay_ticks<=300;ui_notice<=8;" in u
assert 'age<=600' in u and 'dx>16||dy>16' in u
assert 'if(safety_stop||motor_fault||stop_contact||!touch_ready||(snapshot_valid&&snapshot_cancel))begin' in u
assert 'if(action==105)begin' in u
assert '6:beginnext_page=5;next_subpage=0;end' in u
assert 'page<=next_page;subpage<=next_subpage;if(!remote_active)beginpid_action<=10;pid_action_valid<=1;end' in u
assert 'if(!remote_active||safety_stop||motor_fault||stop_contact)beginpid_action_valid<=1;' in u
assert 'wirelocal_arm_ok=!safety_stop&&!motor_fault&&touch_ready&&!remote_active;' in u
assert 'if(subpage==1&&!remote_active)case(action)' in u
assert 'armed<=0;calibration_enable<=0;jog_ms<=0;' in u
assert 'view_cooldown_ms<=150;wait_release<=1;invalid_tap<=1;previous_count<=0;' in u
assert 'if(!touch_ready||(snapshot_valid&&snapshot_cancel)||view_cooldown_ms!=0||wait_release)begin' in u
assert 'if(snapshot_valid&&contact_count==0&&view_cooldown_ms==0)wait_release<=0;' in u
for contact in ('current_button','second_button'):
    assert 'stop_button('+contact+',page,subpage)' in u
assert u.count('robot_ui_hit_test')==2
assert '5,6:if(armed&&local_arm_ok)begincalibration_enable<=1;jog_ms<=250;' in u
assert 'if(jog_ms==1)begincalibration_enable<=0;calibration_duty_left<=0;calibration_duty_right<=0;end' in u
assert 'if(armed&&local_arm_ok)begin' in u
assert 'if(local_arm_ok&&gains_ready&&geometry_valid&&gyro_ready&&lidar_ready)armed<=1;' in u
assert 'elseif(page==4)selected_field<=selected_field==8?9:8;' in u
assert 'elseif(page==4&&subpage==3)next_subpage=1;' in u
assert '8:cruise_counts<=bounded[15:0];9:turn_counts<=bounded[15:0];' in u
assert '8,9:beginlow=1;high=200;end' in u
assert '.cruise_counts(cruise),.turn_counts(turn_speed)' in s
# LCD is a display/drive surface; tapping PID values cannot write gains.
assert len(VIEWS)==21
assert not {(1,4),(1,6),(1,7)}&set(VIEWS)
assert 'pid_sequence' not in u and 'pid_action<=24;' not in u and 'pid_action<=25;' not in u
for pid_view in ((1,2),(1,3)):
    assert VIEWS[pid_view].captions==('DRIVE TEST','STOP BOTH')
    assert set(button_rects(*pid_view))=={1,2,100,101,105,106}
    for x in (48,160,320,448,560,768):
        for y in (138,192,250,322,394):
            assert hit_test(x,y,*pid_view)==0
assert 'elseif((subpage==2||subpage==3)&&action==2)beginpid_action<=10;pid_argument<=0;pid_action_valid<=1;end' in u
assert 'if(remote_takeover)beginstart_delay_ticks<=0;armed<=0;calibration_enable<=0;calibration_duty_left<=0;calibration_duty_right<=0;jog_ms<=0;command<=0;command_valid<=1;end' in u
assert '2:beginscan_enable<=0;lidar_restart<=1;end' in u
assert '3:beginscan_enable<=0;lidar_restart<=1;end' in u
assert 'if((stop_contact&&(current_button==106||(contact_count==2&&second_button==106)))||safety_stop)beginscan_enable<=0;if(scan_enable||stop_contact)lidar_restart<=1;' in u

# Superseding a search must not lose the replacement command. Permit stays
# low while a replacement is pending, then rises with its registered command.
assert 'assignplanner_cancel=!permit||safety_stop||user_command_valid||recovery_state!=0;' in m
assert 'pending&&!planner_busy' in m
assert 'assignmotion_permit=permit&&!planning&&!planner_busy&&!no_path&&!pending;' in m
assert m.count('settle_ticks<=3;') == 3
assert 'wait_depart&&nav_status!=6&&nav_status!=7' not in m
assert 'if(!permit||safety_stop)begin' in m and 'pending<=0;settle_ticks<=0;' in m

# Every standalone UI label is exactly one 16-byte row. Concatenations in the
# formatter are inspected separately using their explicit source widths.
# A memory implementation attribute is not rendered text. Constant labels now
# live in the synchronous character ROM; its complete content is compared in
# check_robot_layout_math.py, including all blank and unused slots.
panel_labels = re.sub(r'\(\*.*?\*\)', '', panel, flags=re.S)
for literal in strings(panel_labels):
    assert len(literal)==16, (literal, len(literal))
st=compact(status)
# Ten raw slots share a single converter. The paired PID page uses all ten;
# existing pages retain their six body rows plus selected value/step trailer.
# Static functions return narrow IDs, not128-bit rows.
# The complete byte semantics are checked by check_formatter_stream.py and
# the independent synchronous-ROM event model; do not retain old concat
# assertions after removing the wide-row circuit they were intended to guard.
for function in ('field_name','navigation_label','view_title','notice_label','unit_hint','menu_line'):
    assert 'function[7:0]'+function+';' in st,function
assert 'function[127:0]' not in st
assert 'reg[127:0]rows' not in st and 'formatted_row' not in st and 'body_row' not in st
assert 'reg[7:0]row_ids[0:11];' in st
assert {int(index) for index in re.findall(r'\brow_ids\[(\d+)\]\s*<=',status)}=={0,1,8,9,10,11}
assert 'reg[31:0]values[0:9];' in st and 'reg[2:0]state;' in st and 'reg[3:0]field;' in st
assert 'reg[9:0]signs,fractional;' in st
assert 'wire[87:0]number=fractional[field]?fixed4_text(bcd):decimal_text(bcd,signs[field]);' in st
assert "wire[31:0]selected_magnitude=signs[field]?~selected_raw+32'd1:selected_raw;" in st
assert 'shift<=selected_conversion;bcd<=0;bits<=0;state<=CONVERT;' in st
assert 'mag(' not in st
assert 'robot_status_text#(.PARALLEL_COMPAT(0))u_status(' in s
assert "assigntext_rows=2048'd0;" in st
rom=st.split('modulerobot_status_label_rom(',1)[1].split('endmodule',1)[0]
assert '(*syn_romstyle="block_rom"*)reg[7:0]memory[0:4095];' in rom
assert "for(init_index=0;init_index<1024;init_index=init_index+1)beginmemory[init_index]=8'h20;memory[init_index+1024]=8'h20;memory[init_index+2048]=8'h20;memory[init_index+3072]=8'h20;end" in rom
assert 'if(read_enable)data<=memory[address];' in rom
rom_bytes=[32]*4096
for address,value in re.findall(r"memory\[(\d+)\]=8'h([0-9a-fA-F]+);",rom):
    address=int(address)
    assert 0<=address<4096,address
    rom_bytes[address]=int(value,16)
assert bytes(rom_bytes[102*16:103*16])==b'NO LIDAR GUARD  '
assert '32:if(rowfield==7)beginbody_id=TEXT102;body_numeric=0;end' in st
assert 'wireheld_pid_view=held_view==10||held_view==11;' in st
assert 'if(held_pid_view?((rowfield>=2&&rowfield<=6)||rowfield>=11):((rowfield>=2&&rowfield<=7)||rowfield==12||rowfield==13))begin' in st
assert "if(held_pid_view?((rowfield>=2&&rowfield<=6)||(rowfield>=11&&rowfield<=14)):((rowfield>=2&&rowfield<=7)||rowfield==12))field<=field+1'b1;" in st
for capture in ('values[0]<={{16{motor_target_left[15]}},motor_target_left};',
                'values[5]<={{16{motor_target_right[15]}},motor_target_right};',
                'values[0]<=motor_position_target_left;values[1]<=position_left;',
                'values[5]<=motor_position_target_right;values[6]<=position_right;',
                "fractional<=10'b1110011100;"):
    assert capture in st,capture
assert '.debug_status(lidar_debug_status),.last_rx_word(lidar_last_rx_word)' in s
assert '.lidar_debug_status(lidar_debug_status),.lidar_last_rx_word(lidar_last_rx_word)' in s
assert '.reply_capture(lidar_reply_capture),.last_tx_word(lidar_last_tx_word)' in s
assert '.lidar_reply_capture(lidar_reply_capture),.lidar_last_tx_word(lidar_last_tx_word)' in s
assert 'held_reply_h0<=lidar_reply_capture[127:96];held_reply_h1<=lidar_reply_capture[95:64];' in st
assert 'held_reply_s0<=lidar_reply_capture[63:32];held_reply_s1<=lidar_reply_capture[31:0];held_last_tx_word<=lidar_last_tx_word;' in st
assert 'if(!held_pid_view&&held_view!=16&&rowfield==14)' in st
assert 'if(!held_pid_view&&held_view!=16&&rowfield==15)' in st
assert bytes(rom_bytes[195*16:197*16])==b'VOFA PID EDIT   READ ONLY LCD   '
c1_sidebar=st.split('row_ids[9]<=',1)[1].split('row_ids[10]<=',1)[0]
assert c1_sidebar.index("lidar_error==8'h15?TEXT202")<c1_sidebar.index('!scan_enable?TEXT119')
assert 'held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,' in st
assert sum((1,3,5,8,4,1,1,1,1,1,1,1,1,3))==32
assert 'wire[5:0]held_view={held_flags[2:0],held_flags[30:28]};' in st
assert 'ROBOT_PREFIX' in uart and 'ROBOT_BODY' in uart
uart_code=compact(uart)
assert 'generateif(VOFA_MODE!=0)begin:vofa_lcd_onlyassignmotor_byte_ready=1\'b1;assignrobot_byte_ready=1\'b1;assigntx_line_count=32\'d0;assigntx=1\'b1;endelsebegin:legacy_sender' in uart_code
assert '.VOFA_MODE(ROBOT_ACTIVE)' in t
for guard in ('sensor_demo_text_snapshot_store',
              'held_robot_bank<=latest_robot_bank;',
              'held_motor_bank<=latest_motor_bank;',
              'wiregrant_motor=!read_request&&',
              'wiregrant_robot=!read_request&&',
              'TEXT_RAM_WAIT:if(text_read_valid)begin'):
    assert guard in uart_code, guard
assert not re.search(r'reg\s*\[2047:0\]\s*held_(?:motor|robot)\b', uart)
assert 'wire[31:0]robot_display_flags=BYTE_TEXT_MODE?robot_byte_meta:robot_status[2079:2048];' in l
assert "wirepid_page_display=robot_control?(robot_display_flags[2:0]==3'd1):pid_page_legacy;" in l
assert '.flags(robot_display_flags)' in l
assert 'read_meta<=snapshot_meta;' in l and 'active_bank<=snapshot_bank;' in l

# Map RAM is synchronous: the request at screen x-1 must select the same
# fine cell used by the following pixel. Check all 384x384 display positions.
for x in range(28,412):
    for y in range(104,488):
        request_x=x-1
        assert 27<=request_x<411
        assert (request_x-27)//3==(x-28)//3
        assert 0<=(487-y)//3<128
assert 128*128*2//8==4096
assert Fraction(5000*5*10,460800)<1  # C1 standard nodes, 8N1
assert Fraction(800*10,115200//10)<1  # illustrative BT report budget
assert Fraction(56*20,115200//10)<1  # four worst-case signed32 FireWater fields

# Heading buttons are relative to measured yaw, not the editable pose field.
for yaw in range(-180000,180000,100):
    for delta in (-90000,90000,180000):
        turned=yaw+delta
        if turned>=180000:turned-=360000
        elif turned<-180000:turned+=360000
        assert -180000<=turned<180000
        assert (turned-yaw)%360000==delta%360000

# Independent permission truth cases: a stopped navigator may start when its
# permit rises; a raw-count manual drive does not accidentally require C1 or
# geometry, whereas every non-driving menu/settings subpage remains idle.
motion_cases=0
for page,subpage in product(range(5),range(8)):
    for armed,touch,gains,stop,fault,active,turn,run,permit in product((False,True),repeat=9):
        safe=armed and touch and not stop and not fault
        manual=safe and (page,subpage)==(4,0)
        navigate=safe and gains and (page,subpage)==(4,1)
        drive=(manual and (active or (turn and run))) or (navigate and permit and run)
        expected=safe and (((page,subpage)==(4,0) and (active or (turn and run))) or
                           ((page,subpage)==(4,1) and gains and permit and run))
        assert drive==expected
        if (page,subpage) not in ((4,0),(4,1)):assert not drive
        if not safe:assert not drive
        if manual and active:assert drive  # even without ready gains/geometry/C1
        if not gains and (page,subpage)==(4,1):assert not drive
        motion_cases+=1

# A remote pair command is not gated by which LCD view displays the running
# state. The explicit UI STOP/ALL STOP paths above still revoke that command.
# A remote owner also suppresses a pre-existing local target until takeover
# clears local permission; it never mixes left/right target ownership.
remote_cases=0
for page,subpage in product(range(5),range(8)):
    for test,armed,running,remote_owner,local,safety in product((False,True),repeat=6):
        local_selected=local and not(remote_owner and armed and running)
        requested=not test and ((armed and running) or local_selected)
        bridge_enabled=requested and not safety
        assert bridge_enabled==(not test and not safety and ((armed and running) or local_selected))
        if remote_owner and armed and running:assert not local_selected
        if safety or test:assert not bridge_enabled
        remote_cases+=1

# Encoder quantization is a geometry fact, not a claim about the user's
# currently unknown motor model. Four-edge counts include the gearbox ratio.
assert Fraction(314,20)==Fraction(157,10)
assert Fraction(314,600)<1
assert Fraction(100*314,600)==Fraction(157,3)  # mm/s per count/10ms
assert round(65*math.pi)==204
assert abs(204/(65*math.pi)-1)<0.001
assert (65*math.pi)/(13*28*4)<0.141  # conditional example, not configured CPR

print('PASS: robot source contracts, narrow text IDs, shared numeric conversion, byte-ROM and CDC/stop/mission guards')
print('PASS: 147456 map-prefetch coordinates, yaw buttons, serial capacity and encoder-unit arithmetic')
print(f'PASS: {motion_cases} manual/navigation permission cases; 150ms+fresh-release view guard; timed calibration jog; frozen 32-bit page/subpage metadata')
print(f'PASS:21 LCD views, VOFA setting/readback/STOP route and {remote_cases} remote/local motor-owner permission cases')
print('NO HDL compilation/simulation, timing closure, resource-fit claim or hardware validation')
