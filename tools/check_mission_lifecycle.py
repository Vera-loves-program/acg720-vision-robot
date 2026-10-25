"""Mission ownership/lifecycle event-model regressions; no hardware/HDL run."""
from pathlib import Path
import re

HERE=Path(__file__).resolve().parents[1]/'src'
source=(HERE/'robot_mission_control.v').read_text(encoding='utf-8')
code=re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S))

class GoalOwner:
    """Independent user-visible contract, with generation-tagged plan events."""
    def __init__(self):
        self.active=False;self.goal=None;self.paused=False;self.invalid=False
        self.fault=False;self.permit=True;self.count=0
        self.generation=0;self.planning=False;self.running=False
    def command(self,command,goal=None):
        self.generation+=1;self.running=False;self.planning=False
        self.active=command=='GO';self.goal=goal if self.active else self.goal
        self.invalid=self.active and not all(-4096<=v<4032 for v in goal)
        self.fault=False
        self.paused=self.active and (self.invalid or not self.permit)
        self.planning=self.active and not self.paused
    def fail(self,kind):
        assert self.active
        self.paused=True;self.running=False;self.planning=False
        self.fault|=kind=='FAULT'
    def plan_result(self,generation,reachable):
        if generation!=self.generation or not self.active or not self.planning:return
        if reachable:self.planning=False;self.running=True
        else:self.fail('NO_ROUTE')
    def integrated(self,count,eligible=True):
        pulse=count!=self.count
        self.count=count
        if not self.permit or not pulse:return
        if self.active and self.paused and not self.invalid and not self.fault and eligible:
            self.paused=False;self.planning=True
    def permit_change(self,enabled):
        self.permit=enabled
        if not enabled and self.active:self.fail('PERMIT')
    def arrived(self,pose,nav_done):
        if self.active and self.running and nav_done and all(abs(p-g)<=5 for p,g in zip(pose,self.goal)):
            self.active=False;self.running=False;self.paused=False

# Person appears, remains through failed plans, then leaves. The goal survives
# every failure and is completed only by the actual final arrival.
m=GoalOwner();m.command('GO',(1000,0));generation=m.generation
m.plan_result(generation,True);assert m.running
m.count=10;m.fail('BLOCKED');m.integrated(10);assert m.paused and not m.planning
for count in (11,12,13):
    m.integrated(count);assert m.active and m.planning and m.goal==(1000,0)
    m.plan_result(generation,False);assert m.active and m.paused and not m.running
m.integrated(14);m.plan_result(generation,True);assert m.running and m.active
m.arrived((994,0),True);assert m.active
m.arrived((997,3),False);assert m.active
m.arrived((997,3),True);assert not m.active

# No absolute refresh/no-route/stale timeout terminates mission ownership.
for reason in ('REFRESH_TIMEOUT','NO_ROUTE','START_UNKNOWN','GOAL_UNKNOWN','STALE','CONTACT','FINAL_BLOCKED'):
    m=GoalOwner();m.command('GO',(300,0));m.fail(reason)
    for _ in range(10000):m.integrated(0,eligible=False)
    assert m.active and m.goal==(300,0) and m.paused
    m.integrated(1,eligible=False);assert m.paused
    m.integrated(2,eligible=True);assert m.planning and m.active

# STOP, manual takeover, and ALL STOP must win over any queued new map or old
# planner result, including while permission is absent.
for command in ('STOP','MANUAL','TURN','ALL_STOP'):
    m=GoalOwner();m.command('GO',(1000,0));old=m.generation;m.fail('NO_ROUTE')
    m.permit_change(False);m.command(command);m.integrated(1)
    m.permit_change(True);m.integrated(2);m.plan_result(old,True)
    assert not m.active and not m.running and not m.planning
m=GoalOwner();m.command('GO',(300,0));old=m.generation
m.command('GO',(1000,200));m.plan_result(old,True)
assert m.active and m.goal==(1000,200) and not m.running
m.plan_result(m.generation,True);assert m.running

# Temporary permit/sensor loss parks; explicit physical fault stays latched.
m=GoalOwner();m.command('GO',(1000,0));m.permit_change(False)
m.integrated(1);m.permit_change(True);m.integrated(1);assert m.paused
m.integrated(2);assert m.planning and m.active
m.fail('FAULT');m.integrated(3);assert m.paused and not m.planning
m.command('GO',(1000,0));assert m.planning and not m.fault
for goal in ((4032,0),(0,5000),(-4097,0)):
    m=GoalOwner();m.command('GO',goal)
    for count in range(1,30):m.integrated(count)
    assert m.active and m.invalid and m.paused and not m.running
    m.command('GO',(300,0));assert m.active and m.planning and not m.invalid

# Source ownership checks: terminal assignments occur only at reset, ARRIVED,
# and explicit safety stop. User commands select the new owner's active flag.
assert code.count('mission_active<=0;')==3
assert 'mission_active<=user_command==3;' in code
assert 'mission_requested=(mission_active&&!user_command_valid)||(user_command_valid&&user_command==3)' in code
assert 'if(mission_requested&&!mission_completed_now)' in code
assert 'outputregwaiting_path,' in code
assert 'reusable_map&&map_integrated_pulse' in code
assert all(name not in code for name in ('refresh_map_count','last_map_count','retry_map_count'))
assert 'if(!invalid_goal&&!fault_paused&&permit&&coordinates_inside&&pose_valid&&map_ready&&' in code
assert 'if(nav_status==9)fault_paused<=1;' in code
assert 'if(!permit)begin' in code and 'if(safety_stop)begin' in code
timeout=code.split('if(!pose_valid||!map_ready||refresh_ticks==300)begin',1)[1].split('endelse',1)[0]
assert 'waiting_path<=1;' in timeout and 'mission_active<=0;' not in timeout
assert 'scan_enable' not in code # mission cannot shut down radar acquisition
print('PASS: persistent obstacle -> repeated no-route -> removal -> automatic resume -> exact arrival')
print('PASS: refresh/unknown/stale/contact waits retain ownership indefinitely; retry requires a NEW eligible integration')
print('PASS: STOP/manual/newGO generation races, permit pauses, invalid-goal lock and physical-fault manual acknowledgement')
print('Independent contract event models/source checks only, not HDL simulation or actual driving.')
