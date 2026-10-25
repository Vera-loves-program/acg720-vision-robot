"""Independent physical geometry and fixed-point/new motion contracts.

This does not claim HDL simulation or timing closure. Physical Cartesian
examples are separate from the CORDIC arithmetic reference; delay and recovery
event models audit cancel/restart semantics without touching hardware.
"""
from pathlib import Path
from math import sin,cos,radians,hypot,ceil
from dataclasses import dataclass,replace
import re

ROOT=Path(__file__).resolve().parents[1]
ATAN=(2880,1700,898,456,229,115,57,29,14,7,4,2)

def project(angle,distance,forward=0,tx=33,ty=-77):
    z=forward-angle
    if z>=11520:z-=23040
    elif z< -11520:z+=23040
    if z>5760:x,z=-(distance<<10),z-11520
    elif z< -5760:x,z=-(distance<<10),z+11520
    else:x=distance<<10
    y=0
    for i,a in enumerate(ATAN):
        if z>=0:x,y,z=x-(y>>i),y+(x>>i),z-a
        else:x,y,z=x+(y>>i),y-(x>>i),z+a
        assert -(1<<25)<=x<(1<<25) and -(1<<25)<=y<(1<<25)
    def normal(v):return (-1 if v<0 else 1)*((abs(v)+843)//1686)
    return tx+normal(x),ty+normal(y)

def flags(x,y,drive=20,turn=30,contact=10):
    f,b,l,r=232,118,119,119
    inside=-b<=x<=f and -r<=y<=l
    if inside:return dict(self=True,front=False,rear=False,slow=False,turn=False,contact=False)
    width=-r-drive<=y<=l+drive
    return dict(self=False,front=width and -b<=x<=f+drive,
        rear=width and -b-drive<=x<=f,
        slow=width and -b<=x<=f+drive+100,
        turn=x*x+y*y<=(ceil(hypot(max(f,b),max(l,r)))+turn)**2,
        contact=-b-contact<=x<=f+contact and -r-contact<=y<=l+contact)

def root_model(n):
    root=0
    bit=1024
    while bit:
        trial=root|bit
        if trial*trial<=n:root=trial
        bit>>=1
    return root+(root*root<n)

# Physical cases: parallel side pedestrian does not block a straight corridor;
# close forward obstacle, rear retreat obstruction, diagonal swept corner.
assert flags(350,0)['slow'] and not flags(350,0)['front']
assert flags(250,0)['front'] and not flags(250,0)['contact']
assert flags(242,0)['contact'] and not flags(243,0)['contact']
assert flags(0,180)['turn'] and not flags(0,180)['front']
assert flags(-130,0)['rear'] and not flags(-130,0)['front']
assert flags(100,100)['self'] and not flags(100,100)['turn']
assert flags(0,-139)['front']  # side clearance at the configured 20mm margin
assert not flags(0,-140)['front']
assert ceil(hypot(232,119))==261 and root_model(232**2+119**2)==261
assert flags(0,-200)['turn']  # 81mm from right SIDE is not a rotation proof

max_near=max_far=0.0
cases=0
for d in (50,100,200,400,1000,4000,12000):
    for a in range(23040):
        x,y=project(a,d)
        rad=radians(-a/64)
        error=hypot(x-33-d*cos(rad),y+77-d*sin(rad))
        if d<=1000:max_near=max(max_near,error)
        max_far=max(max_far,error);cases+=1
assert max_near<3 and max_far<20,(max_near,max_far)
for forward in (0,90*64,180*64,270*64):
    x,y=project(forward,500,forward)
    assert abs(x-533)<=1 and abs(y+77)<=1
for f in range(1,1001,17):
    for l in range(1,1001,19):
        assert root_model(f*f+l*l)==ceil(hypot(f,l))

@dataclass
class Scan:
    age:int=63
    valid:bool=False
    seen:bool=False
    bad:bool=False
    working:bool=False
    published:bool=False
    coverage:int=0
    work_coverage:int=0
    def boundary(self,busy=False):
        self.valid=self.seen and not self.bad and not busy
        if self.valid:
            self.published=self.working;self.coverage=self.work_coverage;self.age=0
        self.seen=True;self.bad=False;self.working=False;self.work_coverage=0
    def point(self,blocked=False,sector=0,inside=False,overrun=False):
        if overrun:self.bad=True
        elif self.seen and not inside:
            self.working|=blocked;self.work_coverage|=1<<sector
    def clear(self):self.__dict__.update(Scan().__dict__)
    @property
    def ready(self):return self.valid and not self.bad and self.age<30

scan=Scan();scan.boundary();assert not scan.ready
for sector in range(4):scan.point(blocked=sector==1,sector=sector)
scan.boundary();assert scan.ready and scan.published and scan.coverage==15
scan.point(inside=True,sector=1);scan.boundary()
assert scan.ready and not scan.published and scan.coverage==0
for sector in range(4):scan.point(sector=sector)
scan.point(overrun=True);assert not scan.ready
scan.boundary();assert not scan.ready
for sector in range(4):scan.point(sector=sector)
scan.boundary();assert scan.ready
scan.age=30;assert not scan.ready
scan.clear();assert not scan.ready and scan.coverage==0

source=(ROOT/'src/rplidar_c1.v').read_text(encoding='utf-8-sig').split('module lidar_body_guard(',1)[1]
clean=re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S)
assert '/' not in clean and '%' not in clean
assert clean.count('square_operand*square_operand')==1
assert not re.search(r'>>>\s*iteration',clean)
for token in ('state==IDLE','scan_age<30','work_bad<=1','if(inside_body)',
              'map_point_valid<=1','coverage<=work_coverage','point_x_squared'):
    assert token in source,token
ui=(ROOT/'src/robot_ui_control.v').read_text(encoding='utf-8-sig')
for token in ('rebase_pending<=3',):
    assert token in ui,token
for token in ("assign drive_margin_mm=16'd20","assign turn_margin_mm=16'd30","assign contact_margin_mm=16'd10"):
    assert token in ui,token
mission=(ROOT/'src/robot_mission_control.v').read_text(encoding='utf-8-sig')
assert 'recovery_state<=1' not in mission
assert "assign recovery_speed=16'sd0" in mission
assert 'if(!permit)begin' in mission
assert 'if(safety_stop)begin' in mission
assert 'mission_requested' in mission
print(f'PASS: {cases} CORDIC/Cartesian rays; max error <=1m {max_near:.3f}mm, <=12m {max_far:.3f}mm; rectangle/contact/swept-corner/root checks.')
print('PASS: live scan expiry, self filtering, no automatic reverse; serial resource/source contracts.')
# Hardware regression: a100mm LEFT polar return is inside this offset chassis,
# whereas the800mm wall is external. It must not win the displayed minimum.
for relative in range(45*64+1,135*64):
    x,y=project((90*64-relative)%(360*64),100,90*64)
    assert -118<=x<=232 and -119<=y<=119,(relative,x,y)
external=[]
for distance in (800,100,850,100):
    x,y=project(0,distance,90*64)
    if not (-118<=x<=232 and -119<=y<=119):external.append(distance)
assert min(external)==800
for token in ('filtered_left_mm<=work_range_left','if(held_distance<work_range_left)',
              'filtered_front_mm<=work_range_front'):
    assert token in source,token
print('PASS:5760 left-angle SELF rays and mixed100/800mm range-display regression.')
print('Independent model/source checks only; not HDL simulation, timing closure or actual driving.')
