"""Source contracts plus independent motor-owner/event arithmetic checks.

These are Python reference models, not HDL compilation or simulation. They
exercise RUN without traffic, the remote TURN pipeline, asynchronous operator
stops/faults, and report ownership under UART backpressure.
"""
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path
import re

from check_motor_integration import check_blocks, uncomment

ROOT = Path(__file__).resolve().parents[1]


def read(name):
    text = uncomment((ROOT / 'src' / (name + '.v')).read_text(encoding='utf-8-sig'))
    check_blocks(text, name)
    return re.sub(r'\s+', '', text)


motor = read('motor_tuning_control')
adapter = motor.split('modulemotor_vofa_settings_adapter(', 1)[1]
core = motor.split('endmodule', 1)[0]
sub = read('robot_subsystem')
nav = read('robot_navigation')
codec = read('motor_uart_commands').split('modulerobot_vofa_uart#', 1)[1]
top = read('vision_robot_ui_top')

# RUN has no time/traffic-dependent stop path. The only highest-priority stop
# causes are the physical stop, real motor faults and explicit STOP actions.
assert 'heartbeat_age' not in motor and 'link_fault<=1' not in core
final_stop = core.rsplit('if(safety_stop||fault_left!=0', 1)[1].split('begin', 1)[0]
assert final_stop == '||fault_right!=0||link_fault||(ui_action_valid&&ui_action==10)||(uart_action_valid&&uart_action==10))'
assert '6:state<=WAIT_ONE;' in adapter and 'heartbeat<=1' not in adapter
assert 'armed<=1;running<=1;remote_owner<=from_remote;' in core
assert 'TURN_STOP=18,TURN_START=19' in adapter
assert 'POSITION_STOP,STOP,CLEAR_STOP,TURN_STOP:core_action=10;' in adapter
assert 'TURN_STOP:state<=TURN_START;' in adapter
assert 'assignturn_request=state==TURN_START&&!cancelled&&!blocked_now&&turn_ready;' in adapter
assert 'cmd_value==0||cmd_value<-180||cmd_value>180||cmd_wheel||cmd_field!=0' in adapter
assert 'takeover<=cmd_op==0||cmd_op==1||cmd_op==5||cmd_op==8||' in adapter
assert 'blocked_now&&(held_op==1||held_op==8||is_set)' in adapter
assert 'if(held_op==1||held_op==8||state==WRITE_VALUE||state==REPLY)state<=WAIT_ONE;' in adapter
assert "rsp_value<={31'd0,turn_active};" in adapter
assert 'if(transaction_error==OK&&(cancelled||blocked_now||!turn_active))rsp_status<=BLOCKED;' in adapter

# A pending completion at *any* pipeline stage must drain before another
# accepted TURN. Otherwise its old untagged line could follow a new RUN ACK.
ready = sub.split('assignremote_turn_ready=', 1)[1].split(';', 1)[0]
for term in ('gyro_ready','!calibrating','nav_fault==0','!safety_stop','!ui_motor_fault',
             '!explicit_local_stop','!map_reset','!remote_takeover',
             '!nav_turn_complete','!turn_complete_valid','!turn_report_busy'):
    assert term in ready, term
assert 'geometry' not in ready and 'lidar' not in ready and 'touch' not in ready and 'page' not in ready
assert 'assignturn_complete_busy=event_pending||frame_event;' in codec
assert '.turn_report_busy(robot_turn_report_busy)' in top
assert '.turn_complete_busy(robot_turn_report_busy)' in top
assert 'if(turn_complete_valid&&turn_complete_ready)turn_complete_valid<=0;' in sub
success = 'if(nav_turn_complete&&!safety_stop&&!ui_motor_fault&&!explicit_local_stop&&!remote_takeover&&!map_reset)begin'
assert success in sub
request_index = sub.index('if(remote_turn_request&&remote_turn_ready')
abort_index = sub.index('if(remote_takeover||explicit_local_stop||safety_stop||ui_motor_fault||map_reset||')
assert sub.index(success) < request_index < abort_index
assert 'remote_turn_active<=0;remote_turn_start<=0;remote_turn_abort<=1;' in sub[abort_index:]
assert '.command_valid(remote_turn_abort||remote_turn_start||(!remote_turn_active&&' in sub
assert ".command(remote_turn_abort?3'd0:remote_turn_start?3'd2:" in sub
assert '.motion_enable(remote_turn_permit||(manual_view?manual_permit:nav_permit))' in sub
assert 'wireexplicit_local_stop=pid_action_valid&&(pid_action==10||pid_action==11);' in sub
assert '.avoid_enable(!manual_view&&!remote_turn_active)' in sub

# Only MODE_TURN->MODE_DONE creates this pulse, and every cancellation below
# the arithmetic controller explicitly overrides it on that same clock.
assert nav.count('turn_complete_pulse<=1;') == 1
assert 'if(motion_mode==MODE_TURN)begindecision_turn_done<=1;decision_status<=6;' in nav
assert 'if(decision_turn_done)beginturn_complete_pulse<=1;motion_mode<=MODE_DONE;' in nav
assert 'if(decision_valid&&decision_mode==motion_mode&&frame_generation==command_generation' in nav
for guard in ('if(command_valid)begin', 'if(safety_stop||!motion_enable)begin',
              'elseif((pose_set||encoder_zeroed)&&!run_request)begin',
              'elseif(!motion_pose_ok||calibrating||(avoid_enable&&!lidar_valid))begin'):
    assert nav.split(guard, 1)[1].startswith('turn_complete_pulse<=0;'), guard
assert 'fault[1]<=1;turn_complete_pulse<=0;' in nav
assert '.turn_complete_pulse(nav_turn_complete)' in sub
assert 'assignconsume_event=state==IDLE&&!ack_pending&&!notice_pending&&event_pending;' in codec
assert 'if(frame_event&&prefix_index==12)state<=NEWLINE;' in codec
assert 'vofa_sample_values[0*32+:32]=(robot_turn_active||robot_local_drive)?' in top
assert 'vofa_sample_values[2*32+:32]=(robot_turn_active||robot_local_drive)?' in top
assert top.count(",motor_target_speed_left}:32'sd0;") == 1
assert top.count(",motor_target_speed_right}:32'sd0;") == 1


def motor_step(running, *, stop=False, fault=False, reset=False, run=False, heartbeat=False):
    # Traffic/heartbeat is intentionally irrelevant to the state transition.
    if reset or stop or fault:
        return False
    return True if run else running


traffic_cases = 0
for heartbeat, stop, fault, run in product((False, True), repeat=4):
    a = motor_step(True, heartbeat=heartbeat, stop=stop, fault=fault, run=run)
    b = motor_step(True, heartbeat=not heartbeat, stop=stop, fault=fault, run=run)
    assert a == b == (not stop and not fault)
    traffic_cases += 1
running = motor_step(False, run=True)
for milliseconds in range(360000):  # six minutes, including every old timeout
    running = motor_step(running)
    assert running
assert not motor_step(running, stop=True)
assert not motor_step(running, fault=True, run=True)


@dataclass(frozen=True)
class Owner:
    active: bool = False
    start: bool = False
    abort: bool = False
    nav_turn: bool = False
    pulse: bool = False
    producer: bool = False
    mailbox: bool = False
    sending: int = 0
    reports: int = 0
    launches: int = 0


def owner_step(old, *, request=False, takeover=False, stop=False, fault=False,
               gyro=True, finish=False, tx_ready=True):
    """Independent edge model; all consumers read the previous edge's state."""
    blocked = takeover or stop or fault or not gyro
    report_busy = old.mailbox or old.sending != 0
    ready = not blocked and not old.pulse and not old.producer and not report_busy
    accepted = request and ready
    active, start, abort, producer = old.active, False, False, old.producer
    if old.producer and not old.mailbox:
        producer = False
    if old.pulse and not (takeover or stop or fault):
        producer, active = True, False
    if accepted:
        active, start = True, True
    if blocked:
        active, start, abort = False, False, True
    # A valid old arithmetic frame alone may complete. A command or loss of
    # permit has later priority; a queued old calculation cannot report success.
    permit = old.active and not blocked
    pulse = old.nav_turn and finish and permit and not old.start and not old.abort
    nav_turn = old.nav_turn and not pulse
    if old.start:
        nav_turn, pulse = True, False
    if old.abort or not permit:
        nav_turn, pulse = False, False
    mailbox, sending, reports = old.mailbox, old.sending, old.reports
    if old.sending:
        if tx_ready:
            sending -= 1
            if not sending:
                reports += 1
    elif old.mailbox:
        mailbox, sending = False, 14  # literal TURN_COMPLETE plus LF
    if old.producer and not old.mailbox:
        mailbox = True
    return Owner(active, start, abort, nav_turn, pulse, producer, mailbox,
                 sending, reports, old.launches + accepted), accepted


def launch(old):
    # Adapter accepted op8 -> TURN_STOP -> TURN_START. The one-clock takeover
    # is already over by the request; even a forced coincidence cannot launch.
    state, accepted = owner_step(old, takeover=True)
    assert not accepted and not state.active
    state, accepted = owner_step(state, request=True)
    assert accepted and state.active and state.start
    state, _ = owner_step(state)
    assert state.nav_turn and state.active
    return state


owner_cases = 0
for takeover, stop, fault, gyro, finish, request in product((False, True), repeat=6):
    state = Owner(active=True, nav_turn=True)
    state, accepted = owner_step(state, takeover=takeover, stop=stop, fault=fault,
                                 gyro=gyro, finish=finish, request=request)
    if takeover or stop or fault or not gyro:
        assert not state.active and not state.pulse and not accepted
    if accepted:
        assert state.active and state.start
    if state.pulse:
        assert finish and not (takeover or stop or fault) and gyro
    owner_cases += 1

# A successful turn reports once; a completed old mode never reports again.
state = launch(Owner())
state, _ = owner_step(state, finish=True)
assert state.pulse
for _ in range(40):
    state, _ = owner_step(state)
assert state.reports == 1 and not state.active and not state.nav_turn

# STOP/RUN takeover on the success edge, or on the producer's following edge,
# cancels the old completion rather than falsely declaring the replacement done.
for cancellation in ('takeover', 'stop', 'fault'):
    for delay in (0, 1):
        state = launch(Owner())
        if delay:
            state, _ = owner_step(state, finish=True)
            assert state.pulse
        state, accepted = owner_step(state, finish=True, **{cancellation: True})
        assert not accepted and not state.pulse and not state.producer
        for _ in range(40):
            state, _ = owner_step(state)
        assert state.reports == 0
        owner_cases += 1

# An overlapping second TURN first aborts the unfinished one; it inherits no
# old completion pulse. Only the replacement's eventual success produces a line.
state = launch(launch(Owner()))
assert state.launches == 2 and state.reports == 0
state, _ = owner_step(state, finish=True)
for _ in range(40):
    state, _ = owner_step(state)
assert state.reports == 1

# Test every stage of the previous result, including arbitrarily stalled UART.
# A replacement must not ACK-active until the old untagged line fully drains.
for stage in ('pulse', 'producer', 'mailbox', 'sending'):
    state = Owner(**{stage: 14 if stage == 'sending' else True})
    for _ in range(40):
        state, accepted = owner_step(state, request=True, tx_ready=False)
        assert not accepted and not state.active
    for _ in range(40):
        state, accepted = owner_step(state, tx_ready=True)
        assert not accepted
    assert state.reports == 1
    state = launch(state)
    assert state.active and state.reports == 1
    owner_cases += 1

# +180/-180 must retain the requested direction at the absolute wrap boundary.
angle_cases = 0
for heading in range(-180000, 180000, 113):
    for degrees in (-180, -179, -90, -1, 1, 90, 179, 180):
        value = heading + (degrees << 10) - (degrees << 4) - (degrees << 3)
        target = (value + 180000) % 360000 - 180000
        assert target == (heading + degrees * 1000 + 180000) % 360000 - 180000
        for drift in (-100, 0, 100):
            error = (target - heading - drift + 180000) % 360000 - 180000
            if degrees == 180 and error < -178000:
                error += 360000
            elif degrees == -180 and error > 178000:
                error -= 360000
            if abs(degrees) == 180:
                assert (error > 0) == (degrees > 0)
            angle_cases += 1

print(f'PASS motor RUN: no watchdog registers or link-timeout assertion; {traffic_cases} traffic-independent cases, six minutes without traffic')
print(f'PASS TURN: source ordering/STOP/RUN takeover/report handshakes, {owner_cases} owner/cancel/stalled-report cases, {angle_cases} wrapped relative-angle cases')
print('PASS four effective speed targets/feedback only; ACK readbacks preserved; successful TURN event has one producer and one serializer')
print('No HDL compilation/simulation, resource/timing closure, network, PWM or board validation was performed.')
