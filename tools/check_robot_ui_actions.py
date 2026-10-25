"""Independent user-intent checks for18views, live VOFA ownership and local drive.

Models touch/permission/timer events and checks source port/action contracts.
No HDL compilation, elaboration, simulation or device access is performed.
"""
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path
import re
from check_motor_integration import uncomment, check_blocks, closing
from robot_ui_spec import VIEWS, button_rects

ROOT = Path(__file__).resolve().parents[1]

def compact(text):
    return re.sub(r'\s+', '', text)

def read(name):
    text = uncomment((ROOT / 'src' / f'{name}.v').read_text(encoding='utf-8-sig'))
    check_blocks(text, name)
    return text

control, subsystem, top, tuning = map(read, ('robot_ui_control', 'robot_subsystem', 'vision_robot_ui_top', 'motor_tuning_control'))
c, s, t, m = map(compact, (control, subsystem, top, tuning))

def connected(text, module, instance):
    for found in re.finditer(r'\b' + module + r'\s*', text):
        pos = found.end()
        if text[pos:pos + 1] == '#':
            pos = closing(text, text.index('(', pos)) + 1
        match = re.match(r'\s*' + instance + r'\s*\(', text[pos:])
        if match:
            start = pos + match.end() - 1
            body = text[start + 1:closing(text, start)]
            result = {}
            for port in re.finditer(r'\.(\w+)\s*\(', body):
                opening = port.end() - 1
                assert port[1] not in result
                result[port[1]] = compact(body[opening + 1:closing(body, opening)])
            return result
    raise AssertionError((module, instance, 'not found'))

views = ((0, 0), (0, 1), (1, 0), (1, 1), (1, 2), (1, 3),
         (2, 0), (2, 1), (2, 2), (3, 0), (3, 1), (3, 2), (3, 3), (3, 4),
         (4, 0), (4, 1), (4, 2), (4, 3), (5,0), (5,1), (5,2))
assert set(VIEWS) == set(views) and len(views) == 21
for view in ((1, 2), (1, 3)):
    assert VIEWS[view].kind == 'dashboard'
    assert VIEWS[view].captions == ('DRIVE TEST', 'STOP BOTH')
    assert set(button_rects(*view)) == {1, 2, 100, 101, 105, 106}
# No hidden on-screen gain/target editor or START can exist in a read-only view.
for removed in ((1, 4), (1, 6), (1, 7)):
    assert removed not in VIEWS
for removed in ('pid_sequence', 'pending_pid_field', 'speed_edit', 'editor_parent'):
    assert removed not in c
assert 'pid_action<=24;' not in c and 'pid_action<=19;' not in c
assert 'pid_action<=25;' not in c

@dataclass(frozen=True)
class Local:
    armed: bool = True
    active: bool = True
    jog_ms: int = 0
    scan: bool = True
    remote_running: bool = True

def local_event(old, event):
    # Remote takeover revokes only local ownership. It cannot generate a
    # competing PID STOP which would immediately cancel the newly accepted RUN.
    if event == 'remote_takeover':
        return replace(old, armed=False, active=False, jog_ms=0), None, False
    if event in ('home', 'back', 'browse', 'touch_lost', 'cancel'):
        return replace(old, armed=False, active=False, jog_ms=0), None if old.remote_running else 10, False
    if event in ('stop', 'fault', 'all_stop'):
        return replace(old, armed=False, active=False, jog_ms=0,
                       scan=False if event == 'all_stop' else old.scan,
                       remote_running=False), 10, event == 'all_stop'
    raise AssertionError(event)

for view, event, remote in product(views, ('stop', 'home', 'back', 'browse', 'touch_lost', 'cancel', 'fault', 'all_stop', 'remote_takeover'), (False,True)):
    state, pid, stop_scan = local_event(Local(jog_ms=250,remote_running=remote), event)
    assert not state.armed and not state.active and state.jog_ms == 0
    if event == 'remote_takeover':
        assert pid is None and state.remote_running==remote and state.scan
    elif event in ('home','back','browse','touch_lost','cancel'):
        assert state.remote_running==remote
        assert pid==(None if remote else 10)
    else:
        assert pid == 10 and not state.remote_running
    assert stop_scan == (event == 'all_stop')

# Browsing/cancelled touches cannot stop an active remote run or grant local
# calibration ownership. Explicit STOP must be observed before JOG may arm.
state=Local(jog_ms=250)
for event in ('browse','home','touch_lost','cancel','back','browse'):
    state,pid,_=local_event(state,event)
    assert state.remote_running and pid is None and not state.armed and state.jog_ms==0
state,pid,_=local_event(state,'stop')
assert not state.remote_running and pid==10
remote_touch_cases=0
for remote,armed,touch,stop,fault in product((False,True),repeat=5):
    local_allowed=touch and not stop and not fault and not remote
    for button in range(1,7):
        encoder_action=button if not remote else None
        jog=armed and local_allowed and encoder_action in (5,6)
        if remote:
            assert encoder_action is None and not local_allowed and not jog
        if not touch or stop or fault:
            assert not local_allowed and not jog
        remote_touch_cases+=1

# HOME/BACK cannot reach the removed PID edit/test views.
def back(view):
    if view == (4, 2):
        return 4, 0
    if view == (4, 3):
        return 4, 1
    p, sub = view
    return (p, 0) if sub else (0, 0)
for view in views:
    assert back(view) in views

# Unchanged manual counts control and distinct navigation prerequisites.
def raw_motion(button, left, right):
    return (-left if button in (2, 3) else left,
            -right if button in (2, 4) else right)
permission_cases = 0
for armed, touch, stop, fault, gains, gyro, geometry, lidar in product((False, True), repeat=8):
    local = touch and not stop and not fault
    raw_allowed = armed and local
    turn_allowed = raw_allowed and gyro
    nav_allowed = turn_allowed and gains and geometry and lidar
    for button in range(1, 5):
        targets = raw_motion(button, 23, 17) if raw_allowed else (0, 0)
        assert (targets != (0, 0)) == raw_allowed
        if not touch or stop or fault:
            assert targets == (0, 0)
        permission_cases += 1
    if armed and local and not gains:
        assert raw_allowed and not nav_allowed
    if armed and local and not gyro:
        assert raw_allowed and not turn_allowed
    assert not nav_allowed or turn_allowed
assert [raw_motion(b, 23, 17) for b in range(1, 5)] == [(23, 17), (-23, -17), (-23, 17), (23, -17)]

speed_cases = 0

def adjust_speed(value, step, down):
    return max(1, value - step) if down else min(200, value + step)
for left, right, step, linked, wheel, down in product(range(1, 201), range(1, 201), (1, 5, 10), (False, True), (0, 1), (False, True)):
    changed = adjust_speed(right if wheel else left, step, down)
    new_left = changed if not wheel or linked else left
    new_right = changed if wheel or linked else right
    assert 1 <= new_left <= 200 and 1 <= new_right <= 200
    if linked:
        assert new_left == new_right
    elif wheel:
        assert new_left == left
    else:
        assert new_right == right
    speed_cases += 1
nav_speed_cases = 0
for field, value, step, down in product((8, 9), range(1, 201), (1, 10, 100), (False, True)):
    assert 1 <= adjust_speed(value, step, down) <= 200
    nav_speed_cases += 1
jog_cases = 0
for wheel, forward, elapsed in product((0, 1), (False, True), range(252)):
    amount = 100 if forward else -100
    targets = (0, amount) if wheel else (amount, 0)
    visible = targets if elapsed < 250 else (0, 0)
    assert sum(value != 0 for value in visible) == int(elapsed < 250)
    assert all(abs(value) <= 100 for value in visible)
    jog_cases += 1
for heading in (*range(-180000, 180000, 1000), -179999, -1, 1, 179999):
    target = (heading + 360000) % 360000 - 180000
    assert -180000 <= target < 180000 and (target - heading) % 360000 == 180000

# STOP remains immediate even during raster settling or a two-finger gesture.
def is_stop(view, button):
    return (button == 106 or (view in ((1, 2), (1, 3)) and button == 2) or
            (view in ((4, 0), (4, 1)) and button == 6) or (view == (5,0) and button == 5))
assert not is_stop((5,1),6)  # REFRESH dispatches a tap; it cannot be swallowed by STOP contact logic.
assert '(s==1&&b==6)' not in c
stop_cases = 0
for view, count, first, second, settling in product(views, range(3), (0, 2, 3, 5, 6, 106), (0, 2, 3, 5, 6, 106), (False, True)):
    immediate = (count > 0 and is_stop(view, first)) or (count == 2 and is_stop(view, second))
    if count == 0:
        assert not immediate
    if count == 2 and (is_stop(view, first) or is_stop(view, second)):
        assert immediate
    if immediate:
        state, action, _ = local_event(Local(), 'all_stop' if first == 106 or (count == 2 and second == 106) else 'stop')
        assert not state.armed and action == 10
    stop_cases += 1
for dx, dy, age, same_id, same_button, cancel, two in product((0, 16, 17), (0, 16, 17), (0, 600, 601), (False, True), (False, True), (False, True), (False, True)):
    accepted = dx <= 16 and dy <= 16 and age <= 600 and same_id and same_button and not cancel and not two
    if cancel or two or dx > 16 or dy > 16 or age > 600:
        assert not accepted
for elapsed, held, released in product(range(151), (False, True), (False, True)):
    permitted = elapsed >= 150 and not held and released
    if elapsed < 150 or held or not released:
        assert not permitted

for contract in (
    '2,3:if(action==1)beginnext_page=4;next_subpage=0;end',
    'elseif((subpage==2||subpage==3)&&action==2)beginpid_action<=10;',
    '2:beginpid_action<=21;pid_action_valid<=1;end',
    '3:beginpid_action<=22;pid_action_valid<=1;end',
    '4:if(local_arm_ok)armed<=1;', '5,6:if(armed&&local_arm_ok)begin',
    'calibration_enable<=1;jog_ms<=250;',
    "calibration_duty_left<=motor_selected_wheel?16'sd0:(action==5?16'sd100:-16'sd100);",
    "calibration_duty_right<=motor_selected_wheel?(action==5?16'sd100:-16'sd100):16'sd0;",
    'if(jog_ms==1)begincalibration_enable<=0;calibration_duty_left<=0;calibration_duty_right<=0;end',
    'manual_left<=(action==2||action==3)?-$signed(manual_speed_left):$signed(manual_speed_left);',
    'manual_right<=(action==2||action==4)?-$signed(manual_speed_right):$signed(manual_speed_right);',
    'if(armed&&local_arm_ok&&gyro_ready)begin',
    'if(armed&&local_arm_ok&&gains_ready&&geometry_valid&&gyro_ready&&lidar_ready)begin',
    '2:beginscan_enable<=0;lidar_restart<=1;end',
    '3:beginscan_enable<=0;lidar_restart<=1;end',
    'wirelocal_arm_ok=!safety_stop&&!motor_fault&&touch_ready&&!remote_active;',
    'if(subpage==1&&!remote_active)case(action)',
    'page<=next_page;subpage<=next_subpage;if(!remote_active)beginpid_action<=10;pid_action_valid<=1;end',
    'view_cooldown_ms<=150;wait_release<=1;invalid_tap<=1;previous_count<=0;',
    'if(!touch_ready||(snapshot_valid&&snapshot_cancel)||view_cooldown_ms!=0||wait_release)begin',
    'if(snapshot_valid&&contact_count==0&&view_cooldown_ms==0)wait_release<=0;',
    'wirestop_contact=snapshot_valid&&touch_ready&&',
    'stop_button=b==106||(p==1&&(s==2||s==3)&&b==2)||',
    '(contact_count==2&&stop_button(second_button,page,subpage))',
    'elseif(contact_count==2)invalid_tap<=1;',
    'id0!=down_id||dx>16||dy>16||current_button!=down_button',
    'contact_count==0&&previous_count==1&&!invalid_tap&&age<=600&&down_button!=0',
    'if(page==4&&subpage==2)next_subpage=0;', 'elseif(page==4&&subpage==3)next_subpage=1;',
    'elseif(action==8)next_subpage=3;', '3:if(action==6)next_subpage=1;',
    'if(next_page==4&&next_subpage==3)selected_field<=8;',
    'elseif(page==4)selected_field<=selected_field==8?9:8;',
    '8,9:beginlow=1;high=200;end',
    '8:cruise_counts<=bounded[15:0];9:turn_counts<=bounded[15:0];',
):
    assert contract in c, contract
assert c.index('if(action_valid)begin') < c.index('if(view_change)begin') < c.index('if(safety_stop||motor_fault||stop_contact||!touch_ready||')
final_guard = c.split('if(safety_stop||motor_fault||stop_contact||!touch_ready||', 1)[1]
for token in ('armed<=0;calibration_enable<=0;calibration_duty_left<=0;calibration_duty_right<=0;jog_ms<=0;',
              'command<=0;command_valid<=1;', 'action_valid<=0;invalid_tap<=1;',
              'if(!remote_active||safety_stop||motor_fault||stop_contact)beginpid_action_valid<=1;',
              'scan_enable<=0;', 'if(scan_enable||stop_contact)lidar_restart<=1;'):
    assert token in final_guard, token
remote_body = c.split('if(remote_takeover)begin', 1)[1]
assert 'armed<=0;calibration_enable<=0;' in remote_body
assert 'pid_action_valid<=' not in remote_body and 'pid_action<=' not in remote_body
assert 'command<=0;command_valid<=1;' in remote_body
controls=connected(subsystem,'robot_ui_control','u_controls')
robot_link=connected(top,'robot_subsystem','u_robot')
assert controls['remote_active']=='remote_active'
assert robot_link['remote_active']=='(motor_remote_owner&&motor_armed&&motor_running)||robot_turn_active'
for token in ('wiremanual_permit=armed&&manual_view&&touch_ready&&!safety_stop&&!ui_motor_fault;',
              'wirelocal_permit=armed&&page==4&&subpage==1&&touch_ready&&gains_ready&&!safety_stop&&!ui_motor_fault;',
              'manual_active<=ui_command==1&&manual_permit;', 'manual_turn_active<=ui_command==2&&manual_permit;',
              'if(!manual_permit||map_reset)beginmanual_active<=0;manual_turn_active<=0;manual_target_left<=0;manual_target_right<=0;',
              'assigntarget_left=!remote_turn_active&&manual_view&&manual_active?manual_target_left:nav_target_left;',
              'assigntarget_right=!remote_turn_active&&manual_view&&manual_active?manual_target_right:nav_target_right;',
              'assigndrive_enable=(remote_turn_permit&&nav_run)||(manual_permit&&(manual_active||(manual_turn_active&&nav_run)))||(local_permit&&nav_permit&&nav_run)||(follow_permit&&nav_run);'):
    assert token in s, token
nav = connected(subsystem, 'robot_navigation', 'u_navigation')
assert nav['avoid_enable'] == '!manual_view&&!remote_turn_active'
assert nav['motion_enable'] == 'remote_turn_permit||(page==5?follow_permit:manual_view?manual_permit:nav_permit)'
settings = connected(top, 'motor_tuning_control', 'u_motor_settings')
assert settings['ui_argument'] == "ROBOT_ACTIVE?robot_pid_argument:32'sd0"
assert 'vofa_action_valid' in settings['uart_action_valid']
assert settings['uart_live_update'] == 'vofa_action_valid&&vofa_live_update'
assert settings['remote_owner'] == 'motor_remote_owner'
assert settings['drive_invert_left'] == 'motor_drive_invert_left' and settings['drive_invert_right'] == 'motor_drive_invert_right'
motors = connected(top, 'motor_drive_control', 'u_motors')
assert '(motor_armed&&motor_running)' in motors['run_enable']
assert 'robot_page' not in motors['run_enable'] and 'robot_subpage' not in motors['run_enable']
assert motors['calibration_enable'] == 'ROBOT_ACTIVE&&robot_calibration_enable'
assert 'wirelive_parameter_write=from_remote&&uart_live_update&&selected_field<=7;' in m
assert 'heartbeat_age' not in m and 'link_fault<=1' not in m
assert 'encoder_invert_left<=1;encoder_invert_right<=0;' in m
assert settings['ui_action_valid'] == 'settings_ui_valid&&(vofa_cmd_ready||settings_ui_stop)'
assert 'vofa_cmd_ready&&!vofa_cmd_valid&&motor_uart_action_valid' in settings['uart_action_valid']
assert 'wiresettings_ui_stop=settings_ui_valid&&(settings_ui_action==10||settings_ui_action==11);' in t
print(f'PASS:21views with read-only speed/position PID and metadata FOLLOW; HOME/BACK and remote takeover preserve separate ownership.')
print(f'PASS:{permission_cases} permission cases,{speed_cases} manual speed/link edits,{nav_speed_cases} NAV edits,{jog_cases} finite JOG intervals.')
print(f'PASS:{stop_cases} immediate STOP/contact/cooldown cases; C1 STOP pulses and ALL STOP disables scan; remote run has no page gate.')
print(f'PASS:{remote_touch_cases} local/remote encoder-action cases; active remote RUN survives HOME/BACK/browse/touch cancellation, explicit STOP/fault still wins.')
print('No HDL compilation/simulation, resource/timing closure, network or hardware validation was performed.')
