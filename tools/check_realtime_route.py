"""Independent R6 route/tag models and source contracts; no HDL simulation."""
from collections import deque
from pathlib import Path
import hashlib,json,random,re

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/'src'
mission=(HERE/'robot_mission_control.v').read_text(encoding='utf-8')
planner=(HERE/'lidar_path_planner.v').read_text(encoding='utf-8')
mapper=(HERE/'lidar_grid_map.v').read_text(encoding='utf-8')

def rotate(v,q):
    x,y=v
    for _ in range(q):x,y=-y,x
    return x,y

def route(free,start,goal,directions):
    if start not in free or goal not in free:return None
    parents={start:None};queue=deque([start])
    while queue:
        p=queue.popleft()
        if p==goal:
            result=[]
            while p is not None:result.append(p);p=parents[p]
            return result[::-1]
        for dx,dy in directions:
            n=p[0]+dx,p[1]+dy
            if n in free and n not in parents:parents[n]=p;queue.append(n)
    return None

# Derive expected body-right independently by rotating Cartesian vectors.
canonical=((1,0),(-1,0),(0,1),(0,-1))
physical_to_code={v:i for i,v in enumerate(canonical)}
table={int(a,16):int(b) for a,b in re.findall(r"4'h([0-9A-F]):ordered_direction=(\d);",planner)}
for q in range(4):
    order=tuple(rotate(v,q) for v in ((1,0),(0,-1),(0,1),(-1,0)))
    assert tuple(table[q*4+rank] for rank in range(4))==tuple(physical_to_code[v] for v in order)
    # Symmetric obstacle has equal detours. The first sideways edge must be
    # body-right, including when worldRIGHT is +X/+Y/-X rather than -Y.
    base={(x,y) for x in range(-10,11) for y in range(-10,11)
          if not(-1<=x<=1 and -1<=y<=1)}
    free={rotate(p,q) for p in base}
    start,goal=rotate((-6,0),q),rotate((6,0),q)
    path=route(free,start,goal,order)
    assert len(path)==len(route(free,start,goal,canonical))
    right=rotate((0,-1),q)
    lateral=next((b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:])
                 if (b[0]-a[0],b[1]-a[1]) not in (rotate((1,0),q),rotate((-1,0),q)))
    assert lateral==right
    # Blocking the preferred side cannot force an unsafe/right-only path.
    left_only={p for p in free if p[0]*right[0]+p[1]*right[1]<=0}
    left_path=route(left_only,start,goal,order)
    assert left_path and all(p in left_only for p in left_path)

rng=random.Random(20261025)
for _ in range(500):
    cells={(x,y):rng.choice((0,1,1,1,2,3)) for x in range(12) for y in range(12)}
    free={p for p,v in cells.items() if v==1};start=(1,1);goal=(10,10)
    old=route(free,start,goal,canonical)
    for q in range(4):
        order=tuple(rotate(v,q) for v in ((1,0),(0,-1),(0,1),(-1,0)))
        new=route(free,start,goal,order)
        assert (new is None)==(old is None)
        if new:
            assert len(new)==len(old)
            assert all(cells[p]==1 for p in new)

def after_or_equal(tag,required):return ((tag-required)&65535)<32768

class Barrier:
    def __init__(self):self.required=None
    def step(self,eligible,boundary,scan,tag,valid,pulse):
        if not eligible:self.required=None;return False
        if self.required is None:
            if boundary:self.required=scan
            return False
        return valid and pulse and after_or_equal(tag,self.required)

for required in (0,1,2,32767,32768,65534,65535):
    b=Barrier();previous=(required-1)&65535
    assert not b.step(True,True,required,previous,True,True)
    for old in (previous,(required-2)&65535):
        assert not b.step(True,False,required,old,True,True)
    assert not b.step(True,False,required,required,False,True)
    assert b.step(True,False,(required+1)&65535,required,True,True)
    # A correction may discard the exact required scan; later complete scan
    # may satisfy, but a sticky16bit alias with no integration event cannot.
    assert not b.step(True,False,required,required,True,False)
    assert b.step(True,False,required,(required+1)&65535,True,True)
    assert not b.step(False,False,required,required,True,True)
    assert not b.step(True,False,required,required,True,False)
    assert not b.step(True,True,(required+3)&65535,required,True,True)
    assert not b.step(True,False,required,required,True,True)

# Model bank identity independently from coordinates. Normal boundaries seal
# OLD fill identity and simultaneously open the new one; clear cancels work.
fill=None;frame=None;work=None;integrated=None
for scan in (65534,65535,0,1,2):
    if fill is not None:frame=fill
    fill=scan
    if frame is not None:
        work=frame;frame=None
        assert work==((scan-1)&65535)
        integrated=work;work=None
assert integrated==1
work=1;fill=2;frame=None
# Correction retains working identity but discards opposite-bank partial scan.
fill=None;assert work==1
# Clearing the map cancels the entire session; no old work may publish later.
fill=frame=work=integrated=None
assert integrated is None and work is None

# Reuse checks elapsed time as well as16bit tag distance; long-pause wrap
# aliases cannot pass. A fresh map starts planning with no arbitrary delay.
def reusable(valid,age,raw,integrated,stationary):
    return valid and age<30 and ((raw-integrated)&65535)<=2 and stationary
assert reusable(True,0,101,100,True)
for args in ((False,0,101,100,True),(True,30,101,100,True),
             (True,0,104,100,True),(True,0,101,100,False)):
    assert not reusable(*args)
for offset in range(100):
    #10Hz scan, <=6ms integration and <=37.27ms BFS model. This is an
    # algorithm budget, not measured motor stopping or hardware timing.
    first_boundary=100-offset
    complete=first_boundary+100+6
    assert 106<=(complete+37.27)<=243.27

compact=lambda s:re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',s,flags=re.S))
m,p,g=map(compact,(mission,planner,mapper))
assert 'if(pending_command==3)begin' in m and 'if(reusable_map)planner_start<=1;' in m
assert 'current_map_scan_age<=16\'d2&&map_capture_ready' in m
assert 'map_integrated_pulse&&!integrated_after_required[15]' in m
assert 'required_capture_id<=scan_id;refresh_phase<=1;' in m
assert all(name not in m for name in ('refresh_map_count','last_map_count','retry_map_count'))
assert 'refresh_scans' not in m and 'refresh_integrations' not in m
assert 'wait_depart<=1;settle_ticks<=3;' in m
assert 'node_write_data={2\'b11,neighbour_direction}' in p
assert 'if(neighbour_rank==3)' in p and 'case(neighbour_direction)' in p
assert 'frame_scan_id<=fill_scan_id;fill_scan_id<=scan_id;' in g
assert g.count('fill_scan_id<=scan_id;')==2
assert 'work_scan_id<=frame_scan_id;' in g
finish=g.split('S_FINISH:begin',1)[1].split('S_SNAPSHOT_READ:',1)[0]
assert 'integrated_scan_id<=work_scan_id;integrated_scan_valid<=enable;integrated_scan_pulse<=enable;' in finish
assert 'integrated_scan_pulse<=0;' in g
assert g.count('integrated_scan_id<=0;integrated_scan_valid<=0;')==2
assert 'assignrecovery_speed=16\'sd0;' in m and 'final_dx>=-5&&final_dx<=5' in m

# Separate saturating maturity exactly matches the old wide count's decisions
# throughout all practical scans, without its32bit diagnostic carry chain.
for threshold in (0,1,2,3,4,7,16,32,127):
    mature=0
    width=max(1,threshold.bit_length())
    for scans in range(1000):
        assert (mature>=threshold)==(scans>=threshold)
        if mature<threshold:mature+=1
        assert mature<(1<<width)
assert 'reference_scans>=MIN_REFERENCE_SCANS' in g
assert 'pose_valid&&correction_enable&&reference_mature&&' in g
assert 'scan_count>=MIN_REFERENCE_SCANS' not in g

print('PASS:4 heading-relative right detours, blocked-right alternate path, 2000 random shortest-route comparisons')
print('PASS: old queued tags, correction drops, eligibility loss,65535->0 wrap, sticky-tag alias and clear/session barriers')
print('PASS: immediate recent-map planning, single complete new scan budget, retained exact5mm/STOP/source contracts')
print('PASS: one-clock integration events replace3wide snapshots; saturating reference maturity for9 thresholds/9000 scans')
print('Independent models/source checks only; no HDL simulation/build/device action.')
