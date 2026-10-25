"""Independent clock-edge transaction model; no HDL tool or device access.

The settings owner and transaction sequencer both sample PRE-edge inputs.
Readback is observed only after their state updates, modelling NBA ordering
without compiling, elaborating or simulating Verilog.
"""
from dataclasses import dataclass, field
from pathlib import Path
from copy import deepcopy
from itertools import product
import argparse
import re

ROOT = Path(__file__).resolve().parents[1]
OK, RANGE, BLOCKED, INTERRUPTED = 0, 2, 4, 5
IDLE, SW, LOOP, FIELD, CHECK, WRITE, RUN, PSTOP, UNLOCK, MODE, ARM, PRUN, STOP, CSTOP, CLEAR, WAIT1, WAIT2, REPLY, TURN_STOP, TURN_START, TASK_SEND, TASK_WAIT = range(22)


@dataclass
class Owner:
    wheel: int = 0
    position_edit: int = 0
    selected: int = 0
    armed: bool = False
    running: bool = False
    position_mode: bool = False
    unlocked: bool = False
    remote: bool = False
    link_fault: bool = False
    turn_ready: bool = True
    turn_active: bool = False
    turn_accept: bool = True  # Negative control: peripheral can reject request.
    turn_requests: list = field(default_factory=list)
    minimum: int = 0
    maximum: int = 200
    deadbands: list = field(default_factory=lambda: [1, 4])
    invert: list = field(default_factory=lambda: [[1, 0], [0, 0]])
    gains: list = field(default_factory=lambda: [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]])
    targets: list = field(default_factory=lambda: [[10, 10], [0, 0]])

    def value(self):
        f, w, p = self.selected, self.wheel, self.position_edit
        if f < 3:
            return self.gains[p][w][f]
        if f == 3:
            return self.targets[p][w]
        if f == 4:
            return self.minimum
        if f == 5:
            return self.maximum
        if f < 8:
            return self.deadbands[f - 6]
        return self.invert[f - 8][w]

    def limits(self):
        return limits(self.selected, self.position_edit, self.minimum, self.maximum)

    def edge(self, action=None, live=False, ui=None, heartbeat=False, safety=False, fault=False,
             takeover=False, turn_request=False, turn_degrees=0):
        old = deepcopy(self)
        # The current user-requested protocol has latched motion and no link
        # watchdog. The retained heartbeat input is an ignored compatibility
        # pin, not a hidden requirement to keep wheels moving.
        chosen = ui if ui is not None else action
        remote = ui is None and action is not None
        if chosen is not None:
            a, argument = chosen
            if a in (16, 17, 18):
                if a == 16:
                    self.wheel = argument
                elif a == 17:
                    self.position_edit = argument
                else:
                    self.selected = argument
            elif a == 10:
                self.armed = self.running = False
            elif a == 11 and not old.running:
                self.armed = False
                self.link_fault = False
            elif a == 15 and not old.armed and not old.running:
                self.unlocked = True
            elif a == 13 and not old.running and old.unlocked:
                self.armed = False
                self.position_mode = True
            elif a == 8 and not old.running and not (safety or fault or old.link_fault):
                self.armed, self.remote = True, remote
            elif a == 9 and old.armed and not old.running and not (safety or fault or old.link_fault):
                configured = old.unlocked and any(old.targets[0][w] and any(old.gains[0][w][:2]) and old.gains[1][w][0] for w in (0, 1))
                if configured:
                    self.running, self.remote = True, remote
            elif a == 24:
                self.armed = self.running = False
                self.position_mode = self.position_edit = False
                self.unlocked = False
                if not (safety or fault or old.link_fault):
                    self.armed = self.running = True
                    self.remote = remote
            elif a == 19:
                f, w, p = old.selected, old.wheel, old.position_edit
                preserve = remote and live and f <= 7
                if not preserve:
                    self.armed = self.running = False
                lo, hi = old.limits()
                value = min(hi, max(lo, argument))
                if f < 3:
                    self.gains[p][w][f] = value
                    if not p and not preserve:
                        self.position_mode = self.unlocked = False
                elif f == 3:
                    self.targets[p][w] = value
                elif f == 4:
                    self.minimum = value
                elif f == 5:
                    self.maximum = value
                    self.minimum = min(old.minimum, value)
                elif f < 8:
                    self.deadbands[f - 6] = value
                else:
                    self.invert[f - 8][w] = value
                    self.position_mode = self.unlocked = False
                    self.targets[1] = [0, 0]
        # Same-edge STOP and actual faults have final priority over ARM/RUN.
        if safety or fault or old.link_fault or (ui is not None and ui[0] == 10) or (action is not None and action[0] == 10):
            self.armed = self.running = False
        if turn_request and old.turn_ready and not (safety or fault or old.link_fault):
            self.turn_requests.append(turn_degrees)
            if old.turn_accept:
                self.turn_active = True
        if takeover or safety or fault or old.link_fault or (ui is not None and ui[0] in (10, 11)):
            self.turn_active = False


def limits(f, position, minimum, maximum):
    if f < 3:
        return 0, 32767
    if f == 3:
        return (-2_000_000, 2_000_000) if position else (-200, 200)
    return {4: (0, maximum), 5: (50, 900), 6: (0, 32), 7: (0, 1000), 8: (0, 1), 9: (0, 1)}[f]


@dataclass
class Transaction:
    state: int = IDLE
    command: tuple = (0, 0, 0, 0, 0)
    cancelled: bool = False
    error: int = OK
    unknown: bool = False
    response: tuple | None = None
    heartbeat: bool = False
    takeover: bool = False
    active_wheel: int = 0
    task_age: int = 0
    task_result: int = 0

    def outputs(self, blocked):
        seq, op, wheel, f, value = self.command
        fixed = {SW: (16, self.active_wheel), LOOP: (17, int(op in (3, 7))), FIELD: (18, f), RUN: (24, 0),
                 PSTOP: (10, 0), UNLOCK: (15, 0), MODE: (13, 0), ARM: (8, 0), PRUN: (9, 0),
                 STOP: (10, 0), CSTOP: (10, 0), CLEAR: (11, 0), TURN_STOP: (10, 0)}
        action = fixed.get(self.state)
        if self.state == WRITE and not self.cancelled and not blocked:
            action = 19, value
        if op == 1 and (self.cancelled or blocked):
            action = None
        return action, self.state == WRITE and f <= 7 and action is not None

    def edge(self, owner, incoming=None, blocked=False, rsp_ready=False, turn_ready=True,
             task_ready=False, task_rsp=None, task_timeout=8):
        seq, op, wheel, f, value = self.command
        active = self.state != IDLE and self.response is None and not self.cancelled
        cancel_new = active and blocked and op in (1, 2, 3, 8)
        state = self.state
        ready_before = state == IDLE and self.response is None
        self.heartbeat = False
        self.takeover = False
        if self.response is not None and rsp_ready:
            self.response = None
        if ready_before and incoming is not None:
            self.command = incoming
            seq, op, wheel, f, value = incoming
            self.active_wheel = wheel & 1
            self.cancelled = blocked and op in (1, 2, 3, 8)
            self.error = BLOCKED if blocked and op in (1, 8) else INTERRUPTED if blocked and op in (2, 3) else OK
            self.unknown = False
            self.task_age = self.task_result = 0
            self.takeover = op in (0, 1, 5, 8) or (op in (2, 3) and f in (8, 9))
            if not (wheel in (0, 1) or (wheel == 2 and op in (2, 3) and f <= 7)):
                self.error, self.unknown, self.state = RANGE, True, WAIT1
                self.takeover = False
            elif op == 0:
                self.state = STOP
            elif op == 1:
                self.state = WAIT1 if blocked else PSTOP if wheel else RUN
            elif op in (2, 3, 4, 7):
                if f <= 9:
                    self.state = SW
                else:
                    self.error, self.unknown, self.state = RANGE, True, WAIT1
            elif op == 5:
                self.state = CSTOP
            elif op == 8:
                if not -180 <= value <= 180 or value == 0 or wheel or f:
                    self.error, self.state = RANGE, WAIT1
                else:
                    self.state = WAIT1 if blocked else TURN_STOP
            elif op == 9:
                valid=(wheel==0 and 0<=f<=8 and
                       (-3900<=value<=3900 if f in (1,2) else value==0))
                if not valid:self.error,self.unknown,self.state=RANGE,True,WAIT1
                else:self.state=TASK_SEND
            else:
                self.state = WAIT1
        elif state in (SW, LOOP):
            self.state += 1
        elif state == FIELD:
            self.state = CHECK
        elif state == CHECK:
            lo, hi = limits(f, op == 3, owner.minimum, owner.maximum)
            valid = lo <= value <= hi and not (f == 5 and value < owner.minimum)
            if op in (2, 3) and not valid:
                if not self.cancelled:
                    self.error = RANGE
                self.state = WAIT1
            else:
                self.state = WRITE if op in (2, 3) and not self.cancelled and not cancel_new else WAIT1
        elif state in (PSTOP, UNLOCK, MODE, ARM):
            self.state += 1
        elif state in (WRITE, RUN, PRUN, STOP, CLEAR):
            self.state = WAIT1
        elif state == CSTOP:
            self.state = CLEAR
        elif state == TURN_STOP:
            self.state = TURN_START
        elif state == TURN_START:
            if not turn_ready or self.cancelled or blocked:
                self.error = BLOCKED
            self.state = WAIT1
        elif state in (TASK_SEND,TASK_WAIT):
            if state==TASK_SEND and task_ready:
                self.task_age=0
                if task_rsp is None:self.state=TASK_WAIT
                else:self.error,self.task_result=task_rsp;self.state=WAIT1
            elif state==TASK_WAIT and task_rsp is not None:
                self.error,self.task_result=task_rsp;self.state=WAIT1
            elif self.task_age==task_timeout-1:self.error,self.state=6,WAIT1
            else:self.task_age+=1
        elif state == WAIT1:
            self.state = WAIT2
        elif state == WAIT2:
            self.state = REPLY
        elif state == REPLY:
            if (op in (2, 3) and wheel == 2 and not self.active_wheel and self.error == OK
                    and not self.cancelled and not cancel_new and owner.value() == value):
                self.active_wheel, self.state = 1, SW
                return
            status, actual = self.error, 0
            if self.unknown:
                actual = 0
            elif op in (2, 3, 4, 7):
                actual = 0 if self.unknown else owner.value()
                if op in (2, 3) and status == OK and actual != value:
                    status = INTERRUPTED
            elif op == 1:
                actual = int(owner.running)
                if self.cancelled or blocked or not owner.armed or not owner.running or owner.position_mode != wheel:
                    status = BLOCKED
            elif op == 0:
                actual = int(owner.running)
                if owner.armed or owner.running:
                    status = BLOCKED
            elif op == 5 and blocked:
                status = BLOCKED
            elif op == 6:
                actual = int(owner.running)
            elif op == 8:
                actual = int(owner.turn_active)
                if status == OK and (self.cancelled or blocked or not actual):
                    status = BLOCKED
            elif op == 9:
                actual=self.task_result
            self.response = seq, op, wheel, f, status, actual
            self.state = IDLE
        if cancel_new:
            self.cancelled = True
            self.error = BLOCKED if op in (1, 8) else INTERRUPTED
            if op in (1, 8) or state in (WRITE, REPLY):
                self.state = WAIT1
            if state == REPLY:
                self.response = None


def edge(owner, transaction, incoming=None, ui=None, safety=False, fault=False, rsp_ready=False, drop_write=False):
    before = deepcopy(owner)
    blocked = safety or fault or owner.link_fault or (ui is not None and ui[0] in (10, 11))
    action, live = transaction.outputs(blocked)
    old_heartbeat = transaction.heartbeat
    ready = owner.turn_ready and not transaction.takeover and not blocked
    turn_request = transaction.state == TURN_START and not transaction.cancelled and ready
    degrees = transaction.command[4]
    if drop_write and action is not None and action[0] == 19:
        action = None
    owner.edge(action, live, ui, old_heartbeat, safety, fault,
               transaction.takeover, turn_request, degrees)
    transaction.edge(before, incoming, blocked, rsp_ready, ready)
    return action


def run(owner, command, inject=None):
    tx, trace = Transaction(), []
    for cycle in range(40):
        effects = {} if inject is None else inject(cycle, tx.state)
        trace.append((tx.state, owner.running))
        edge(owner, tx, command if cycle == 0 else None, **effects)
        if tx.response is not None:
            return tx, trace
    raise AssertionError(('no response', command, trace))


def compact(text):
    return re.sub(r'\s+', '', re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S))


def source_contracts(adapter):
    a = compact(adapter)
    core = compact((ROOT / 'src/motor_tuning_control.v').read_text(encoding='utf-8-sig'))
    top = compact((ROOT / 'src/vision_robot_ui_top.v').read_text(encoding='utf-8-sig'))
    subsystem = compact((ROOT / 'src/robot_subsystem.v').read_text(encoding='utf-8-sig'))
    required = (
        'SELECT_WHEEL:begincore_action=16;', 'SELECT_LOOP:begincore_action=17;', 'SELECT_FIELD:begincore_action=18;',
        'core_action=19;core_argument=held_value;', 'core_live_update=held_field<=7;',
        'RUN_SPEED:core_action=24;', 'POSITION_STOP,STOP,CLEAR_STOP,TURN_STOP:core_action=10;',
        'POSITION_UNLOCK:core_action=15;', 'POSITION_MODE:core_action=13;', 'POSITION_ARM:core_action=8;', 'POSITION_RUN:core_action=9;',
        'WAIT_ONE:state<=WAIT_TWO;', 'WAIT_TWO:state<=REPLY;',
        'rsp_value<=unknown_field?32\'sd0:core_selected_value;', 'core_selected_value!=held_value)rsp_status<=INTERRUPTED;',
        'if(cancel_new)begin', 'if(state==REPLY)rsp_valid<=0;',
        'cancelled<=blocked_now&&(cmd_op==1||cmd_op==8||cmd_op==2||cmd_op==3);',
        'transaction_error<=blocked_now&&(cmd_op==1||cmd_op==8)?BLOCKED:blocked_now&&(cmd_op==2||cmd_op==3)?INTERRUPTED:OK;',
        '1:state<=blocked_now?WAIT_ONE:cmd_wheel?POSITION_STOP:RUN_SPEED;',
        'held_value>=$signed({22\'d0,core_min_duty});',
        'inputwire[3:0]cmd_op,', 'outputreg[3:0]rsp_op,', 'reg[3:0]held_op;',
        'inputwire[1:0]cmd_wheel,', 'outputreg[1:0]rsp_wheel,', 'reg[1:0]held_wheel;',
        'core_argument={31\'d0,active_wheel};', 'active_wheel<=cmd_wheel[0];',
        'wirecommand_wheel_valid=cmd_wheel<=1||(cmd_wheel==2&&(cmd_op==2||cmd_op==3)&&cmd_field<=7);',
        'if(!command_wheel_valid)begintransaction_error<=RANGE_ERROR;unknown_field<=1;takeover<=0;state<=WAIT_ONE;',
        'if(is_set&&held_wheel==2&&!active_wheel&&transaction_error==OK&&!cancelled&&!cancel_new&&core_selected_value==held_value)beginactive_wheel<=1;state<=SELECT_WHEEL;',
        'assignturn_request=state==TURN_START&&!cancelled&&!blocked_now&&turn_ready;',
        'assignturn_degrees=held_value[15:0];',
        '6:state<=WAIT_ONE;',
        '8:if(cmd_value==0||cmd_value<-180||cmd_value>180||cmd_wheel||cmd_field!=0)begin',
        'endelsestate<=blocked_now?WAIT_ONE:TURN_STOP;',
        'TURN_STOP:state<=TURN_START;',
        'if(!turn_ready||cancelled||blocked_now)transaction_error<=BLOCKED;',
        "rsp_value<={31'd0,turn_active};",
        'if(transaction_error==OK&&(cancelled||blocked_now||!turn_active))rsp_status<=BLOCKED;',
    )
    for token in required:
        assert token in a, token
    for token in (
        'wirelive_parameter_write=from_remote&&uart_live_update&&selected_field<=7;',
        'if(!live_parameter_write)beginarmed<=0;running<=0;end',
        'remote_owner<=from_remote;',
        'encoder_invert_left<=1;encoder_invert_right<=0;',
    ):
        assert token in core, token
    assert 'heartbeat_age' not in core
    assert 'heartbeat<=1' not in a
    assert '.uart_live_update(vofa_action_valid&&vofa_live_update)' in top
    assert '.remote_owner(motor_remote_owner)' in top
    assert 'motor_armed&&motor_running' in top and 'robot_subpage==7' not in top.split('motor_drive_controlu_motors', 1)[1].split(');', 1)[0]
    # Top-level arbitration must hold all selectors stable through a VOFA
    # transaction; STOP/CLEAR may preempt and are reported to the adapter.
    assert 'wiresettings_ui_stop=settings_ui_valid&&(settings_ui_action==10||settings_ui_action==11);' in top
    assert '.ui_action_valid(settings_ui_valid&&(vofa_cmd_ready||settings_ui_stop))' in top
    assert 'vofa_cmd_ready&&!vofa_cmd_valid&&motor_uart_action_valid' in top
    assert '.core_stop_override(settings_ui_stop)' in top
    assert '.turn_request(vofa_turn_request),.turn_degrees(vofa_turn_degrees),.turn_ready(robot_turn_ready),.turn_active(robot_turn_active)' in top
    assert 'wireexplicit_local_stop=pid_action_valid&&(pid_action==10||pid_action==11);' in subsystem


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path)
    args = parser.parse_args()
    source = (args.source or ROOT / 'src/motor_tuning_control.v').read_text(encoding='utf-8-sig')
    assert 'module motor_vofa_settings_adapter' in source
    source_contracts(source.split('module motor_vofa_settings_adapter', 1)[1])
    cases = 0
    for position, wheel, f in product((0, 1), (0, 1), range(10)):
        lo, hi = limits(f, position, 75, 450)
        for value in sorted({lo - 1, lo, (lo + hi) // 2, hi, hi + 1, -2**31, 2**31 - 1}):
            owner = Owner(minimum=75, maximum=450)
            original = deepcopy(owner)
            tx, _ = run(owner, (123, 3 if position else 2, wheel, f, value))
            valid = lo <= value <= hi and not (f == 5 and value < original.minimum)
            assert tx.response[:4] == (123, 3 if position else 2, wheel, f)
            assert tx.response[4] == (OK if valid else RANGE), (position, wheel, f, value, tx.response)
            assert tx.response[5] == owner.value()
            if valid:
                assert owner.value() == value
            else:
                original.wheel, original.position_edit, original.selected = wheel, position, f
                assert owner.value() == original.value()
            assert not owner.running
            # An unacknowledged reply is immutable and prevents a second write.
            held = tx.response
            for _ in range(5):
                edge(owner, tx, (999, 2, wheel, f, lo))
                assert tx.response == held
            edge(owner, tx, rsp_ready=True)
            assert tx.response is None
            cases += 1
    # ACK observes actual settled storage. A suppressed write cannot be echoed
    # as success simply because the parser accepted its syntax.
    owner = Owner()
    failed, _ = run(owner, (22, 2, 1, 0, 2304), lambda _, state: {'drop_write': state == WRITE})
    assert failed.response[4:] == (INTERRUPTED, 0)
    # BOTH keeps one request/ACK while visiting the unique settings owner twice.
    # Every bank must be read back successfully before this can ACK OK.
    def read_bank(owner, position, wheel, f):
        view = deepcopy(owner)
        view.position_edit, view.wheel, view.selected = position, wheel, f
        return view.value()

    broadcast_cases = broadcast_aborts = 0
    for position, f in product((0, 1), range(8)):
        baseline = Owner()
        lo, hi = limits(f, position, baseline.minimum, baseline.maximum)
        for value in (lo - 1, lo, (lo + hi) // 2, hi, hi + 1):
            owner = deepcopy(baseline)
            before = [read_bank(owner, position, w, f) for w in (0, 1)]
            tx, trace = run(owner, (61, 3 if position else 2, 2, f, value))
            valid = lo <= value <= hi
            assert tx.response[:4] == (61, 3 if position else 2, 2, f)
            assert tx.response[4] == (OK if valid else RANGE), (position, f, value, tx.response)
            actual = [read_bank(owner, position, w, f) for w in (0, 1)]
            assert actual == ([value, value] if valid else before)
            assert sum(state == WRITE for state, _ in trace) == (2 if valid else 0)
            assert tx.response[5] == actual[1 if valid else 0]
            assert not owner.running
            held = tx.response
            for _ in range(5):
                edge(owner, tx, (62, 2, 2, f, lo))
                assert tx.response == held
                assert [read_bank(owner, position, w, f) for w in (0, 1)] == actual
            edge(owner, tx, rsp_ready=True)
            assert tx.response is None
            broadcast_cases += 1
        # Broadcast live edits preserve the active loop and current motion;
        # editing POSITION gains is not a request to switch running modes.
        for mode in (False, True):
            owner = Owner(armed=True, running=True, remote=True, position_mode=mode,
                          unlocked=mode, turn_active=True)
            tx, trace = run(owner, (63, 3 if position else 2, 2, f, (lo + hi) // 2))
            assert tx.response[4] == OK and all(running for _, running in trace)
            assert owner.running and owner.armed and owner.position_mode == mode and owner.turn_active

    # Wrong wheel2 operations/directions/queries are rejected before selecting
    # or touching the owner. In particular invalid RUN2 must not mean RUN1.
    invalid_broadcast = [(op, 2, f) for op, f in product((0, 1, 4, 5, 6, 7, 8), (0, 3, 8, 9))]
    invalid_broadcast += [(op, 2, f) for op, f in product((2, 3), (8, 9, 10))]
    invalid_broadcast += [(op, 3, 0) for op in range(9)]
    for op, wheel, f in invalid_broadcast:
        owner = Owner(armed=True, running=True, remote=True, turn_active=True)
        original = deepcopy(owner)
        tx, trace = run(owner, (64, op, wheel, f, 0))
        assert tx.response[4:] == (RANGE, 0), (op, wheel, f, tx.response)
        assert owner == original and all(running for _, running in trace)
        assert not any(state in (SW, WRITE, RUN, TURN_START) for state, _ in trace)

    for missing_wheel in (0, 1):
        owner = Owner()
        owner.gains[0][0][0], owner.gains[0][1][0] = 256, 512
        tx, trace = run(owner, (65, 2, 2, 0, 2560),
                        lambda _, state: {'drop_write': state == WRITE and owner.wheel == missing_wheel})
        assert tx.response == (65, 2, 2, 0, INTERRUPTED, (256, 512)[missing_wheel])
        assert owner.gains[0][0][0] == (256 if missing_wheel == 0 else 2560)
        assert owner.gains[0][1][0] == 512
        assert sum(state == WRITE for state, _ in trace) == missing_wheel + 1
        assert not owner.running

    for position in (0, 1):
        command = (66, 3 if position else 2, 2, 0, 2560)
        _, baseline_trace = run(Owner(armed=True, running=True), command)
        assert sum(state == REPLY for state, _ in baseline_trace) == 2
        for injection in ('ui', 'safety', 'fault'):
            for phase in range(len(baseline_trace)):
                owner = Owner(armed=True, running=True)
                event = {'ui': (10, 0)} if injection == 'ui' else {injection: True}
                tx, _ = run(owner, command, lambda cycle, _: event if cycle == phase else {})
                assert tx.response[4] == INTERRUPTED, (position, injection, phase, tx.response)
                assert not owner.running and not owner.armed
                storage = deepcopy(owner.gains)
                for _ in range(20):
                    edge(owner, tx)
                assert owner.gains == storage and not owner.running
                broadcast_aborts += 1
    for position, wheel, f in product((0, 1), (0, 1), range(8)):
        owner = Owner(armed=True, running=True, remote=True, position_mode=bool(position), unlocked=bool(position))
        lo, hi = limits(f, position, owner.minimum, owner.maximum)
        value = (lo + hi) // 2
        tx, trace = run(owner, (5, 3 if position else 2, wheel, f, value))
        assert tx.response[4] == OK and owner.running and all(running for _, running in trace)
        assert owner.position_mode == bool(position)
        owner.selected, owner.position_edit, owner.wheel = f, position, wheel
        owner.edge((19, value), live=False)
        assert not owner.running  # legacy edits retain their stop behaviour.
    aborts = 0
    for position in (0, 1):
        owner = Owner()
        owner.gains[0][0][0] = owner.gains[1][0][0] = 256
        _, baseline = run(deepcopy(owner), (7, 1, position, 0, 0))
        for injection in ('ui', 'safety', 'fault'):
            for phase in range(len(baseline)):
                event = {'ui': (10, 0)} if injection == 'ui' else {injection: True}
                tx, trace = run(deepcopy(owner), (7, 1, position, 0, 0), lambda cycle, _: event if cycle == phase else {})
                assert tx.response[4] == BLOCKED and tx.response[5] == 0, (position, injection, phase, trace, tx.response)
                aborts += 1
    # A UI STOP on acceptance was a concrete missing cancellation case. SET
    # continues selecting its requested readback, but never commits a write.
    _, set_trace = run(Owner(armed=True, running=True), (18, 2, 1, 0, 2560))
    for phase in range(len(set_trace)):
        tx, _ = run(Owner(armed=True, running=True), (18, 2, 1, 0, 2560), lambda cycle, _: {'ui': (10, 0)} if cycle == phase else {})
        assert tx.response[4] == INTERRUPTED and tx.response[5] in (0, 2560)
    same_value, _ = run(Owner(), (18, 2, 1, 0, 0), lambda cycle, _: {'ui': (10, 0)} if cycle == 0 else {})
    assert same_value.response[4:] == (INTERRUPTED, 0)
    # Query alone never starts/stops a correctly running owner.
    for op, wheel, f in product((4, 7), (0, 1), range(10)):
        tx, trace = run(Owner(armed=True, running=True), (9, op, wheel, f, 0))
        assert tx.response[4] == OK and all(running for _, running in trace)
    # Both ownership paths now latch RUN indefinitely without heartbeats.
    # This model checks the former2s boundary and much longer quiet periods;
    # actual STOP/fault remains dominant, and the old op6 is a genuine no-op.
    for remote in (False, True):
        owner = Owner()
        owner.edge(action=(24, 0) if remote else None, ui=None if remote else (24, 0))
        for _ in range(1000):
            owner.edge()
        assert owner.running and not owner.link_fault
        no_op, trace = run(owner, (29, 6, 0, 0, 0))
        assert no_op.response == (29, 6, 0, 0, OK, 1)
        assert not no_op.heartbeat and all(running for _, running in trace)
        owner.edge(ui=(10, 0))
        assert not owner.running
    turns = 0
    for degrees in range(-180, 181):
        owner = Owner(armed=True, running=True)
        tx, trace = run(owner, (51, 8, 0, 0, degrees))
        if degrees:
            assert tx.response == (51, 8, 0, 0, OK, 1)
            assert owner.turn_requests == [degrees] and owner.turn_active
            assert not owner.running and any(state == TURN_STOP for state, _ in trace)
        else:
            assert tx.response == (51, 8, 0, 0, RANGE, 0)
            assert not owner.turn_requests
        turns += 1
    for degrees, wheel, f in ((-181,0,0), (181,0,0), (-2**31,0,0), (2**31-1,0,0), (90,1,0), (90,0,1), (-90,0,15)):
        owner = Owner()
        tx, _ = run(owner, (52, 8, wheel, f, degrees))
        assert tx.response[4:] == (RANGE, 0) and not owner.turn_requests
    for ready, accept in ((False,True), (True,False)):
        owner = Owner(turn_ready=ready, turn_accept=accept)
        tx, _ = run(owner, (53, 8, 0, 0, 90))
        assert tx.response[4:] == (BLOCKED, 0) and not owner.turn_active
        assert owner.turn_requests == ([90] if ready else [])
    _, turn_trace = run(Owner(), (54, 8, 0, 0, -90))
    turn_aborts = 0
    for injection in ('ui', 'clear', 'safety', 'fault'):
        for phase in range(len(turn_trace)):
            owner = Owner()
            event = {'ui': (10 if injection == 'ui' else 11, 0)} if injection in ('ui', 'clear') else {injection: True}
            tx, _ = run(owner, (54, 8, 0, 0, -90), lambda cycle, _: event if cycle == phase else {})
            assert tx.response[4:] == (BLOCKED, 0), (injection, phase, tx.response)
            assert not owner.turn_active and len(owner.turn_requests) <= 1
            for _ in range(20):
                edge(owner, tx)
            assert not owner.turn_active  # No delayed request after cancellation.
            turn_aborts += 1
    # An unrelated new motion owner aborts an already active TURN. Parameter
    # gains leave it alone; direction inversion is itself an explicit takeover.
    for op, f, value, takeover in ((0,0,0,True), (1,0,0,True), (5,0,0,True),
                                  (2,8,1,True), (3,9,0,True), (2,0,256,False)):
        owner = Owner(turn_active=True)
        tx, _ = run(owner, (55, op, 0, f, value))
        assert owner.turn_active is not takeover
    # The ACK returns actual peripheral state, and backpressure freezes it.
    owner = Owner()
    tx, _ = run(owner, (56,8,0,0,180))
    held = tx.response
    owner.turn_active = False
    for _ in range(5):
        edge(owner, tx, incoming=(57,8,0,0,-180))
        assert tx.response == held and owner.turn_requests == [180]
    edge(owner, tx, rsp_ready=True)
    assert tx.response is None
    print(f'PASS: {cases} bounded SET/readback/backpressure transactions; suppressed writes cannot ACK success.')
    print(f'PASS: {broadcast_cases} BOTH SET bounds/readbacks, {broadcast_aborts} each-phase cancellations, '
          f'{len(invalid_broadcast)} invalid BOTH selectors, independent left/right dropped-write detection and frozen wheel2 ACKs.')
    print(f'PASS: {aborts} same-edge/each-phase RUN cancellation events, including multi-step position RUN.')
    print('PASS: live gain/target writes preserve run; legacy edits stop; selectors read both banks; latchedlocal/remote RUN needs no heartbeat; op6 is an ACKedno-op.')
    print(f'PASS: {turns} TURN angle/range cases, {turn_aborts} same-edge/each-phase cancellations, ready/rejected-peripheral readback and frozenACKs.')
    print('No HDL compilation/simulation, synthesis, programming or hardware access was used.')


if __name__ == '__main__':
    main()
