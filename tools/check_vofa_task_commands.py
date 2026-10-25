"""Independent task/UART transaction model and RTL interface contracts.

No UART/COM/device operations or HDL dynamic simulation. Legacy PID/turn
transactions remain covered by check_motor_vofa_adapter.py.
"""
from pathlib import Path
from copy import deepcopy
import re
from check_motor_vofa_adapter import Owner,Transaction,IDLE,TASK_SEND,TASK_WAIT

ROOT=Path(__file__).resolve().parents[1]

def ui_owner(action,value=0,*,initial_goal=(300,0),ready=True,visual=(True,True,True),
             cancel_phase=None,cancel_status=4):
    """Independent staged UI-owner contract; not a Verilog simulator.

    Cancellation is sticky through acceptance/staging/application; an ACK
    reports that owner's result, and ZERO waits for its actual pose pulse.
    """
    goal=list(initial_goal);page=(0,0);error=0;applied=[];pid_stops=[]
    response=None;pending=None;rebase=0;pose_pulses=[];moving=False;pose_pulse=False
    for phase in range(9):
        old_rebase=rebase;old_pose_pulse=pose_pulse;pose_pulse=False
        cancelled=phase==cancel_phase
        if cancelled and action!=7:error=cancel_status;moving=False;pending=None
        if rebase:
            rebase-=1
            if rebase==0:pose_pulses.append(phase);goal=[300,0];pose_pulse=True
        if phase==0:
            if action not in (5,6):pid_stops.append(phase);moving=False
        elif phase==1:page=(4,1) if action<=3 else (5,0) if action==8 else (5,3)
        elif phase==2:
            metadata=(0<=action<=8 and (-3900<=value<=3900 if action in (1,2) else value==0))
            if error:pass
            elif not metadata:error=2
            elif action!=7 and not ready:error=4
            elif action in (4,5,6,8) and not visual[0 if action==8 else action-4]:error=4
            elif action in (1,2):goal[action-1]=value
            else:pending=action
        elif phase==3 and pending is not None:
            applied.append(pending)
            if pending==0:rebase=3
            elif pending==7:moving=False;pid_stops.append(phase)
            else:moving=True
            pending=None
        if phase>=4 and (action!=0 or error or (old_rebase==0 and not old_pose_pulse)):
            response=(error,goal[action-1] if action in (1,2) else 0,phase)
            break
    return dict(goal=tuple(goal),page=page,response=response,applied=applied,
                pid_stops=pid_stops,pose_pulses=pose_pulses,moving=moving)

def check_ui_owner():
    zero=ui_owner(0)
    assert zero['page']==(4,1) and zero['goal']==(300,0)
    assert zero['applied']==[0] and zero['pose_pulses']==[6] and zero['response']==(0,0,8)
    x=ui_owner(1,-1200);y=ui_owner(2,800,initial_goal=x['goal'])
    assert x['response'][:2]==(0,-1200) and y['response'][:2]==(0,800)
    go=ui_owner(3,initial_goal=y['goal'])
    assert go['goal']==(-1200,800) and go['applied']==[3] and go['moving']
    for action in (4,5,6):
        visual=ui_owner(action)
        assert visual['page']==(5,3) and visual['applied']==[action] and visual['moving']
        assert bool(visual['pid_stops'])==(action==4)
        denied=ui_owner(action,visual=(False,False,False))
        assert denied['response'][:2]==(4,0) and not denied['applied']
    face=ui_owner(8)
    assert face['page']==(5,0) and face['applied']==[8] and face['moving']
    assert face['pid_stops']==[0]
    denied=ui_owner(8,visual=(False,True,True))
    assert denied['response'][:2]==(4,0) and not denied['applied']
    stop=ui_owner(7,ready=False)
    assert stop['applied']==[7] and not stop['moving'] and stop['response'][:2]==(0,0)
    for action in (1,2):
        for value in (-3901,-3900,3900,3901):
            result=ui_owner(action,value)
            assert result['response'][0]==(0 if abs(value)<=3900 else 2)
    for action in (*range(7),8):
        for status in (4,5):
            for phase in range(5):
                result=ui_owner(action,0,cancel_phase=phase,cancel_status=status)
                assert result['response'][0]==status
                assert not result['moving'],(action,status,phase)
                if phase<=2:assert not result['applied']
    print('PASS: UI-owner ZERO pose-before-ACK, SETX/Y exact readback, GO goal retention, TRACK FACE page/readiness, FACE/GO NEAR target retention, STOP, and80 each-phase sticky cancellations.')

def run(command,ready_cycle=4,response_cycle=6,response=(0,123),blocked=False):
    owner=Owner(armed=True,running=True,remote=True)
    initial=deepcopy(owner)
    tx=Transaction()
    requests=[]
    for cycle in range(30):
        action,live=tx.outputs(blocked)
        assert action is None and not live,('raw motor bypass',cycle,action)
        assert not tx.takeover,('raw motor owner takeover',cycle)
        ready=cycle==ready_cycle
        if tx.state==TASK_SEND and ready:requests.append((tx.command[3],tx.command[4]))
        tx.edge(owner,incoming=command if cycle==0 else None,blocked=blocked,
                task_ready=ready,task_rsp=response if cycle==response_cycle else None)
        assert owner==initial
        if tx.response is not None:return tx,requests
    raise AssertionError('task failed to reply')

def main():
    check_ui_owner()
    checked=0
    for field in range(9):
        for value in (-3901,-3900,-1,0,1,3900,3901):
            for wheel in range(4):
                command=(700,9,wheel,field,value)
                valid=wheel==0 and (-3900<=value<=3900 if field in (1,2) else value==0)
                tx,requests=run(command)
                assert tx.response==(700,9,wheel,field,0 if valid else 2,123 if valid else 0)
                assert requests==([(field,value)] if valid else [])
                checked+=1
    # The response belongs to the handler, not the parser or requested value.
    for status in (0,2,4,5,6):
        tx,requests=run((701,9,0,3,0),response=(status,-456))
        assert tx.response==(701,9,0,3,status,-456) and requests==[(3,0)]
    for field in (9,):
        tx,requests=run((702,9,0,field,0));assert tx.response[4:]==(2,0) and not requests
    # Missing owner-ready or missing response recovers to a bounded ERR6 ACK.
    for ready_cycle,response_cycle in ((99,99),(4,99)):
        tx,requests=run((703,9,0,3,0),ready_cycle,response_cycle)
        assert tx.response[4:]==(6,0)
    # Task STOP is still delivered to the UI owner under a motor fault; only
    # that owner decides whether other task actions are currently permitted.
    tx,requests=run((704,9,0,7,0),blocked=True,response=(0,0))
    assert tx.response[4:]==(0,0) and requests==[(7,0)]
    # Accepted immediate response and delayed response both work once only.
    tx,requests=run((705,9,0,1,-3900),4,4,response=(0,-3900))
    assert tx.response[4:]==(0,-3900) and len(requests)==1
    tx=Transaction();owner=Owner();cmd=(706,9,0,3,0)
    tx.edge(owner,incoming=cmd)
    for _ in range(5):
        assert tx.state==TASK_SEND and tx.command==cmd
        tx.edge(owner)
    tx.edge(owner,task_ready=True)
    assert tx.state==TASK_WAIT
    tx.edge(owner,task_rsp=(4,0))
    for _ in range(3):tx.edge(owner)
    frozen=tx.response
    for _ in range(5):tx.edge(owner,incoming=(999,9,0,4,0))
    assert tx.response==frozen

    source=(ROOT/'src/motor_tuning_control.v').read_text(encoding='utf-8-sig')
    adapter=source.split('module motor_vofa_settings_adapter',1)[1]
    compact=re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',adapter,flags=re.S))
    for token in ('assigntask_valid=state==TASK_SEND;', 'assigntask_action=held_field;',
                  'assigntask_value=held_value;', 'TASK_TIMEOUT_CYCLES=CLK_HZ/10<1?1:CLK_HZ/10;',
                  'transaction_error<=task_rsp_status;task_result<=task_rsp_value;',
                  'transaction_error<=TASK_TIMEOUT;state<=WAIT_ONE;',
                  'elseif(held_op==9)rsp_value<=task_result;',
                  '9:if(cmd_wheel!=0||cmd_field>8||'):
        assert token in compact,token
    # No TASK_* branch exposes core motor/PID actions or takeover.
    outputs=adapter.split('always @* begin',2)[2].split('always @(posedge',1)[0]
    assert not re.search(r'\bTASK_(SEND|WAIT)\s*:',outputs)
    takeover=compact.split('takeover<=cmd_op==0',1)[1].split(';',1)[0]
    assert 'cmd_op==9' not in takeover
    ui=(ROOT/'src/robot_ui_control.v').read_text(encoding='utf-8-sig')
    ui=re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',ui,flags=re.S))
    for token in ('task_error<=task_action==7?8\'d0:',
                  'if(task_error!=0)beginendelseif(task_held_action>8',
                  'assigntap_valid=action_valid&&(!task_cancel_live||stop_button(action,page,subpage)||action==105);',
                  'task_phase!=0&&task_error!=0',
                  'if(task_action!=5&&task_action!=6)begin',
                  '4:if(task_visual_ready[0])begin',
                  '5:if(task_visual_ready[1])begin',
                  '6:if(task_visual_ready[2])begin',
                  '8:if(task_visual_ready[0])beginaction<=2;action_valid<=1;',
                  'elseif(task_held_action==8)beginpage<=5;subpage<=0;end',
                  'rebase_pending==0&&!pose_set',
                  'if(task_held_action==1)task_rsp_value<=goal_x_mm;',
                  'if(task_held_action==2)task_rsp_value<=goal_y_mm;'):
        assert token in ui,token
    print(f'PASS: {checked} task metadata/range cases; exact handler ACK; request/backpressure; immediate/delayed reply; timeout recovery; faulted STOP; no raw motor/PID bypass.')
    print('Independent model/source checks, not HDL simulation or hardware validation.')

if __name__=='__main__':main()
