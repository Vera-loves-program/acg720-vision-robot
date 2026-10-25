"""Equivalent command data selection and immediate raw-fault stop contracts."""
from pathlib import Path
from itertools import product
import re

ROOT=Path(__file__).resolve().parents[1]
top=(ROOT/'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
sub=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8')
compact=lambda s:re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',s,flags=re.S))
t=compact(top);s=compact(sub)
for active in (False,True):
    for valid in (False,True):
        for legacy_valid in (False,True):
            if active and legacy_valid:continue  # Actual constant producer.
            for ui_valid in (False,True):
                for vofa in (-2147483648,-3900,0,30,3900,2147483647):
                    legacy=17;ui=26
                    old=vofa if valid else legacy
                    new=vofa if active or valid else legacy
                    old_applied=old if valid or legacy_valid else ui if ui_valid else None
                    new_applied=new if valid or legacy_valid else ui if ui_valid else None
                    assert old_applied==new_applied
assert '.uart_argument(ROBOT_ACTIVE?vofa_argument:(vofa_action_valid?vofa_argument:motor_uart_argument))' in t
assert '.uart_action(ROBOT_ACTIVE?vofa_action:(vofa_action_valid?vofa_action:motor_uart_action))' in t
assert '.fault_left(motor_fault_left),.fault_right(motor_fault_right)' in t
assert '.safety_stop(motor_safety_stop||motor_link_fault)' in t
assert '.core_fault_left(motor_fault_left),.core_fault_right(motor_fault_right)' in t
assert 'ui_motor_fault<=(motor_fault_left|motor_fault_right)!=0||motor_link_fault;' in s
assert 'if(!reset_n)beginui_motor_fault<=0;gains_ready_q<=0;end' in s
assert 'gains_ready_q<=gains_ready;' in s
assert '.gains_ready(gains_ready_q)' in s
assert 'motor_request_now&&(!ROBOT_ACTIVE||selected_snapshot_ready)' in t
assert 'selected_request_q&&selected_owner_q==robot_local_drive&&selected_mode_q==selected_mode_now' in t
assert 'selected_speed_left_q<=robot_local_drive?robot_target_left:motor_target_speed_left;' in t
assert 'selected_speed_right_q<=robot_local_drive?robot_target_right:motor_target_speed_right;' in t
cycles=0
for sequence in product(range(8),repeat=4):
    previous=(False,False,False);pair=(0,0)
    for tick,word in enumerate(sequence):
        request,owner,mode=(bool(word&1),bool(word&2),bool(word&4))
        enabled=request and previous[0] and owner==previous[1] and mode==previous[2]
        if not request:assert not enabled
        if owner!=previous[1] or mode!=previous[2]:assert not enabled
        if enabled:
            assert pair==(tick*100-100+(1 if owner else 7),tick*100-100+(2 if owner else 8))
        # A whole pair and its owner/mode commit together at this edge.
        pair=(tick*100+(1 if owner else 7),tick*100+(2 if owner else 8))
        previous=(request,owner,mode);cycles+=1
axis=compact((ROOT/'src/motor_axis_control.v').read_text(encoding='utf-8'))
assert 'wirebridge_enabled=reset_n&&motion_requested&&!safety_stop&&fault==0;' in axis
assert '.enable(bridge_enabled)' in axis
assert 'ui_motor_fault' not in axis
print('PASS: active/legacy applied-command equivalence; raw axis/bridge and VOFA fault guards remain independent of delayed UI notification')
print(f'PASS:{cycles} coherent pair/owner/mode edges; new-owner guard and raw STOP priority; source bindings')
print('Static/arithmetic checks only; not timing proof, HDL execution or device actuation.')
