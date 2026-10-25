"""Paired speed-dashboard transaction contracts and permission truth tables.

Independent Python policy examples plus static source checks only. No HDL
compilation, elaboration, clocked RTL simulation, synthesis or hardware I/O.
"""
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'src/motor_tuning_control.v').read_text(encoding='utf-8-sig')
code = re.sub(r'\s+', '', re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S))
start = code.split('24:begin', 1)[1].split('25:if(', 1)[0]
select = code.split('25:if(', 1)[1].split('5,6,19:begin', 1)[0]
stop = code.split('10:begin', 1)[1].split('end', 1)[0]
owner_code = code.split('endmodule', 1)[0] + 'endmodule'
priority = owner_code.split('if(safety_stop||fault_left!=0', 1)[1]

assert 'target_speed_left<=10;target_speed_right<=10;' in code
assert 'speed_kp_left<=0;speed_ki_left<=0;speed_kd_left<=0;' in code
assert 'speed_kp_right<=0;speed_ki_right<=0;speed_kd_right<=0;' in code
assert 'max_duty_permille<=200;min_duty_permille<=0;' in code
assert 'armed<=0;running<=0;mode_position<=0;edit_position<=0;' in start
assert 'position_unlocked<=0;' in start
assert 'if(!safety_stop&&!link_fault&&fault_left==0&&fault_right==0)begin' in start
assert 'armed<=1;running<=1;remote_owner<=from_remote;' in start
assert not any(name in start for name in ('config_ok', '_kp_', '_ki_', '_kd_', 'target_speed_'))
assert 'effective_argument>=0&&effective_argument<=19&&' in select
assert '(effective_argument[3:0]<=3||effective_argument==5)' in select
assert 'armed<=0;running<=0;edit_position<=0;' in select
assert 'selected_wheel<=effective_argument[4];' in select
assert 'selected_field<=effective_argument[3:0];' in select
assert not any(name in select for name in ('target_speed_', 'target_position_', '_kp_', '_ki_', '_kd_'))
assert stop == 'armed<=0;running<=0;'
assert '||link_fault||' in priority
assert '(ui_action_valid&&ui_action==10)||(uart_action_valid&&uart_action==10)' in priority
assert 'heartbeat_age' not in code and 'watchdog' not in owner_code
assert 'uart_heartbeat' not in priority, 'quiet Bluetooth cannot be a running-stop condition'
assert priority.endswith('beginarmed<=0;running<=0;endendendendmodule')


@dataclass(frozen=True)
class Wheel:
    target: int
    kp: int
    ki: int
    kd: int
    position_target: int = 12345
    encoder_inverted: bool = False
    motor_inverted: bool = False


@dataclass(frozen=True)
class Settings:
    left: Wheel
    right: Wheel
    maximum: int = 200
    minimum: int = 0


@dataclass(frozen=True)
class State:
    settings: Settings
    armed: bool = False
    running: bool = False
    position_mode: bool = False
    position_edit: bool = False
    position_unlocked: bool = False
    selected_wheel: int = 0
    selected_field: int = 0
    remote_owner: bool = False


def start_pair(state, *, stop=False, left_fault=False, right_fault=False,
               link_fault=False, remote=False):
    """User-level START transaction: parameters are data, faults are inhibits."""
    permitted = not any((stop, left_fault, right_fault, link_fault))
    return replace(state, armed=permitted, running=permitted, position_mode=False,
                   position_edit=False, position_unlocked=False,
                   remote_owner=remote if permitted else state.remote_owner)


def stop_pair(state):
    return replace(state, armed=False, running=False)


def select_editor(state, argument):
    allowed = {0, 1, 2, 3, 5, 16, 17, 18, 19}
    if argument not in allowed:
        return state
    return replace(state, armed=False, running=False, position_edit=False,
                   selected_wheel=argument >> 4, selected_field=argument & 15)


permissions = 0
preservation = 0
for lt, rt in product((-200, -1, 0, 1, 10, 200), repeat=2):
    for lg, rg in product(((0, 0, 0), (2560, 0, 0), (0, 16, 0), (0, 0, 256)), repeat=2):
        settings = Settings(Wheel(lt, *lg), Wheel(rt, *rg, position_target=-23456,
                                             encoder_inverted=True, motor_inverted=True))
        old = State(settings, True, True, True, True, True, 1, 7, True)
        for stop, lf, rf, link, remote in product((False, True), repeat=5):
            new = start_pair(old, stop=stop, left_fault=lf, right_fault=rf,
                             link_fault=link, remote=remote)
            # Neither zero gains, signed targets, nor position-test history
            # impose extra START permissions in the speed dashboard.
            assert new.armed == new.running == (not any((stop, lf, rf, link)))
            assert not new.position_mode and not new.position_edit and not new.position_unlocked
            assert new.settings is old.settings
            assert (new.selected_wheel, new.selected_field) == (1, 7)
            assert stop_pair(new).settings is settings
            assert not stop_pair(new).running
            permissions += 1
        for selector in range(-32, 64):
            edited = select_editor(old, selector)
            assert edited.settings is settings
            if selector in {0, 1, 2, 3, 5, 16, 17, 18, 19}:
                assert not edited.armed and not edited.running and not edited.position_edit
                assert edited.selected_wheel == (selector >> 4)
                assert edited.selected_field == (selector & 15)
            else:
                assert edited is old
            preservation += 1

# Regression: both wheels keep their independent setpoints through selecting
# and changing to the other editor, STOP and START. Legacy action23 isolation
# must not be part of this dashboard's actions (UI is checked separately).
pair = State(Settings(Wheel(10, 2560, 0, 0), Wheel(17, 3072, 0, 0)))
for editor in (0, 16, 1, 17, 2, 18, 3, 19, 5):
    pair = start_pair(select_editor(pair, editor))
    assert pair.running and pair.settings.left.target == 10 and pair.settings.right.target == 17
    pair = stop_pair(pair)
    assert not pair.running
for remote in (False, True):
    started = start_pair(pair, remote=remote)
    assert started.running and started.armed and started.remote_owner == remote
    assert not start_pair(started, stop=True, remote=remote).running
    assert not start_pair(started, left_fault=True, remote=remote).running
    assert not start_pair(started, right_fault=True, remote=remote).running
    assert not start_pair(started, link_fault=True, remote=remote).running

print(f'PASS: paired START/STOP source contract; {permissions} stop/fault/gain/target permission cases')
print(f'PASS: {preservation} direct-editor preservation/selector cases; paired setpoint workflow')
print('PASS: local/remote paired RUN has no heartbeat watchdog; STOP and real faults retain priority')
print('NO HDL compilation/simulation, synthesis/resource measurements, timing or hardware validation')
