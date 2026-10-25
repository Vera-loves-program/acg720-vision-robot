"""Independent scan-bank/RAM arbitration and measured-body seed regressions.

No device or HDL simulator. Includes a frozen OLD policy as a negative control
for the periodic missing scan wedge and one-port RAM stale-Q bug.
"""
from pathlib import Path
from fractions import Fraction
import re

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'src/lidar_grid_map.v').read_text(encoding='utf-8')
code=re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S)
compact=re.sub(r'\s+','',code)
assert 'S_MPOINT:if(!ray_we)state<=S_MPOINT_LATCH;' in compact
assert 'S_IPOINT:if(!ray_we)state<=S_IPOINT_LATCH;' in compact
assert 'front_state!=F_IDLE||busy||frame_ready' not in compact
assert 'fill_bank_available=!busy||state==S_BODY_SEED||fill_bank!=work_bank;' in compact
assert 'fill_bank<=!work_bank;' in compact
assert 'map_we=seed_origin_valid&&seed_cell_inside;map_wdata=2\'b01;' in compact
assert 'pose_x_mm==0&&pose_y_mm==0&&pose_heading_q6==0' in compact
assert '(freeze_granted||body_seed_done||!seed_origin_valid)' in compact
assert 'if(enable&&!freeze_granted&&!body_seed_done&&seed_origin_valid)' in compact
assert 'elseif(clear_addr==MAP_CELLS-1)beginbody_seed_done<=1;' in compact
assert 'body_seed_done<=1;clear_addr<=0;' not in compact


def read_with_writes(mask,stall):
    memory=list(range(64));q=-999;phase=0;index=0;out=[]
    for clock in range(100):
        write=clock<12 and bool(mask&(1<<clock))
        old_q=q
        if write:memory[32+clock%32]=1000+clock
        else:q=memory[index]
        if phase==0:
            if not stall or not write:phase=1
        else:
            out.append(old_q);index+=1;phase=0
            if index==16:return out
    raise AssertionError('bounded write stream must drain')


old_failures=0
for mask in range(1<<12):
    assert read_with_writes(mask,True)==list(range(16))
    old_failures+=read_with_writes(mask,False)!=list(range(16))
assert old_failures>0


class Banks:
    def __init__(self,old=False):
        self.slots=[[None]*1024 for _ in range(2)]
        self.fill=0;self.count=0;self.frame=None;self.work=None
        self.wait=True;self.epoch=0;self.old=old;self.drops=0;self.accepted=[]

    def available(self):return self.work is None or self.fill!=self.work[0]

    def boundary(self):
        if self.wait:
            if self.available() and self.frame is None:self.wait=False;self.count=0
        elif self.available() and self.frame is None and self.count:
            self.frame=(self.fill,self.count);self.accepted.append(tuple(self.slots[self.fill][:self.count]))
            self.fill^=1;self.count=0
        else:self.wait=True;self.count=0

    def point(self,scan,k):
        locked=not self.available()
        if self.wait or self.frame is not None or locked or (self.old and self.work is not None):
            self.drops+=1
            if not self.old and (locked or self.frame is not None):self.wait=True;self.count=0
            return
        assert self.work is None or self.fill!=self.work[0]
        self.slots[self.fill][self.count]=(self.epoch,scan,k);self.count+=1

    def take(self):
        assert self.work is None and self.frame is not None
        bank,count=self.frame
        self.work=(bank,tuple(self.slots[bank][:count]));self.frame=None
        assert self.fill!=bank

    def finish(self):
        bank,snapshot=self.work
        assert tuple(self.slots[bank][:len(snapshot)])==snapshot
        self.work=None

    def correction(self):
        assert self.work is not None
        self.epoch+=1;self.frame=None;self.count=0;self.wait=True;self.fill=1-self.work[0]


def normal_scan_run(old):
    b=Banks(old)
    for scan in range(8):
        b.boundary()
        if b.frame is not None:b.take()
        for k in range(500):
            if b.work is not None and k==15:b.finish()
            b.point(scan,k)
    b.boundary()
    if b.work is not None:b.finish()
    return b


old,new=normal_scan_run(True),normal_scan_run(False)
assert old.drops==7*15 and new.drops==0
assert all(len(scan)==500 for scan in new.accepted)
assert all([p[2] for p in scan]==list(range(500)) for scan in new.accepted)
assert all(scan[0][2]==15 for scan in old.accepted[1:])

# Deliberately exceed one revolution of service. A queued/sealed bank and
# the working bank both remain immutable; no partial new scan is resumed.
b=Banks();b.boundary()
for k in range(10):b.point(0,k)
b.boundary();b.take()
for k in range(10):b.point(1,k)
b.boundary();queued=b.frame;held=tuple(b.slots[queued[0]][:queued[1]])
for k in range(5):b.point(2,k)
assert tuple(b.slots[queued[0]][:queued[1]])==held and b.wait
b.finish();b.take();snapshot=b.work
b.correction()
assert b.work==snapshot and b.fill!=b.work[0] and b.wait and b.frame is None
b.point(2,9);assert b.count==0
b.boundary()
for k in range(10):b.point(3,k)
assert all(p[0]==b.epoch for p in b.slots[b.fill][:b.count])
b.finish()

# Only complete fine cells inside the measured body at HERE IS ZERO may be
# seeded. Compare integer implementation with exact physical millimetres.
seed=[]
f,rear,left,right=232,118,119,119
exact=tuple(map(Fraction,('231.27','117.39','119','119')))
for gy in range(128):
    for gx in range(128):
        x,y=-4096+gx*64,-4096+gy*64
        inside=x>=-(rear-1) and x+64<=f-1 and y>=-(right-1) and y+64<=left-1
        if inside:
            seed.append((gx,gy))
            for xx in (x,x+64):
                for yy in (y,y+64):assert -exact[1]<=xx<=exact[0] and -exact[3]<=yy<=exact[2]
assert len(seed)==8 and (64,64) in seed and (70,64) not in seed
for pose in [(0,0,0,False),(1,0,0,True),(0,1,0,True),(0,0,1,True)]:
    x,y,yaw,fresh=pose
    assert not(fresh and x==0 and y==0 and yaw==0)

# A planner request can arrive while the IMU pose is stale. Once acknowledged,
# a fresh zero pose cannot begin a write pass until ownership is released.
for refresh_clock in range(1,16):
    granted=False;seed_done=False;state='idle';writes=0
    for clock in range(20):
        request=clock<18;valid=clock>=refresh_clock
        frozen=request and state=='idle' and (granted or seed_done or not valid)
        next_grant=False if not request else granted or frozen
        start_seed=state=='idle' and not granted and not seed_done and valid
        if frozen:assert not start_seed and writes==0
        if start_seed:state='seed'
        elif state=='seed':writes+=1
        if clock<18:assert state=='idle' and writes==0
        granted=next_grant
    assert state=='seed'

# A stale-pose abort does not mark an incomplete pass complete; the next fresh
# zero pose may retry when the planner is not holding ownership.
seed_done=False;state='seed'
for valid,last_cell in [(False,False),(True,False),(True,True)]:
    if state=='seed':
        if not valid:state='idle'
        elif last_cell:seed_done=True;state='idle'
    elif valid and not seed_done:state='seed'
assert seed_done and state=='idle'

print(f'PASS: 4096 adversarial one-port RAM schedules; old unconditional issue fails{old_failures}, stalled issue always correct')
print('PASS: eight500-point revolutions; old repeats15-point missing prefix, opposite-bank capture retains all500')
print('PASS: prolonged backend backpressure, sealed/working bank isolation and correction-epoch discard/restart')
print('PASS: only8 WHOLE64mm cells inside exact measured body at fresh zero pose; no exterior halo seeded')
print('PASS: planner freeze survives pose refresh; interrupted body seed retries only after ownership release')
print('Independent arithmetic/ownership models and source contracts only; no HDL execution or hardware validation.')
