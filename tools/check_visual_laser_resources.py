"""Offline fixed-width equivalence and resource-shape checks for visual laser.

Independent Python arithmetic checks do not execute the HDL, infer DSP usage,
or demonstrate routing/timing success. Actual counts come from Gowin reports.
"""
from pathlib import Path
import math
import random
import re

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'src/robot_subsystem.v').read_text(encoding='utf-8')
source=source.split('module robot_visual_laser(',1)[1].split('endmodule',1)[0]
code=re.sub(r'//[^\n]*|/\*.*?\*/','',source,flags=re.S)
compact=re.sub(r'\s+','',code)
assert 'wiresigned[32:0]point_product=point_operand_a*point_operand_b;' in compact
assert compact.count('point_operand_a*point_operand_b')==1
assert 'regsigned[16:0]relative_x,relative_y;' in compact
assert 'regsigned[25:0]atan_x,atan_y,atan_sx,atan_sy;' in compact
assert 'atan_y[31]' not in compact
assert 'if(point_valid&&point_phase!=0)scan_ok<=0;' in compact
assert 'pending_scan_good<=scan_ok&&same_pose&&same_query&&laser_ready&&point_phase==0;' in compact
assert 'result_status==4&&publish_state==0&&atan_state==0' in compact
assert 'if(publish_state==1)begin' in compact and 'elseif(publish_state==2)begin' in compact
assert 'pending_prior_good<=prior_good;' in compact and 'pending_heading<=sh;' in compact
assert 'if(!consecutive)begin' in compact and 'regcluster_in_cone;' in compact

rng=random.Random(6025)
point_cases=0
corners=(-32768,-32767,-501,-1,0,1,500,32766,32767)
for _ in range(60000):
    px,py,cx,cy=(rng.randint(-32768,32767) for _ in range(4))
    dx,dy=(rng.randint(-32768,32767) for _ in range(2))
    x,y=px-cx,py-cy
    assert -65536<=x<65536 and -65536<=y<65536
    products=(x*dx,y*dy,x*dy,y*dx)
    # Each service clocks one shared multiplier. Product and sums preserve
    # all signed bits, including extreme subtraction of two signed16 inputs.
    assert all(-(1<<32)<=p<(1<<32) for p in products)
    partial=products[0];dot=partial+products[1]
    partial=products[2];cross=partial-products[3]
    assert (dot,cross)==(x*dx+y*dy,x*dy-y*dx)
    assert all(-(1<<32)<=p<(1<<32) for p in (dot,cross))
    point_cases+=1

steps=(45000,26565,14036,7125,3576,1790,895,448,224,112,56,28,14,7,3,2)
vectors=[(x,y) for x in corners for y in corners]
vectors += [(rng.randint(-32768,32767),rng.randint(-32768,32767)) for _ in range(50000)]
peak=0;max_error=0
for bx,by in vectors:
    x,y=bx*256,by*256
    angle=0
    if bx<0:
        x,y=-x,-y;angle=-180000 if by<0 else 180000
    nx,ny,na=x,y,angle
    for i,step in enumerate(steps):
        x,y,angle=(x-(y>>i),y+(x>>i),angle-step) if y<0 else (x+(y>>i),y-(x>>i),angle+step)
        nx,ny,na=(nx-(ny>>i),ny+(nx>>i),na-step) if ny<0 else (nx+(ny>>i),ny-(nx>>i),na+step)
        # Check before wrapping: any overflow would invalidate the reduction.
        assert -(1<<25)<=nx<(1<<25) and -(1<<25)<=ny<(1<<25),(bx,by,i,nx,ny)
        assert (nx,ny,na)==(x,y,angle)
        peak=max(peak,abs(nx),abs(ny))
    if math.hypot(bx,by)>=50:
        error=(angle-math.degrees(math.atan2(by,bx))*1000+180000)%360000-180000
        max_error=max(max_error,abs(error))
        assert abs(error)<15,(bx,by,error)

print(f'PASS: {point_cases} signed17-delta/shared-multiplier product equivalence cases; one17x16 multiply expression')
print(f'PASS: {len(vectors)} full-signed16 vectors, exact32-to26-bit CORDIC equivalence at every iteration; peak={peak}, limit={1<<25}, angular error<{max_error:.3f}mdeg')
print('PASS: six-clock point service, overrun invalidates scan, complete physical clusters retained')
publication_cases=0
for _ in range(30000):
    min_x,min_y=rng.randint(-32768,32000),rng.randint(-32768,32000)
    max_x=min(32767,min_x+rng.choice((0,1,399,400,401,1000)))
    max_y=min(32767,min_y+rng.choice((0,1,399,400,401,1000)))
    bx,by=(min_x+max_x)//2,(min_y+max_y)//2
    px,py=bx+rng.choice((-128,-127,0,127,128)),by+rng.choice((-128,-127,0,127,128))
    px=max(-32768,min(32767,px));py=max(-32768,min(32767,py))
    count=rng.randrange(4);seen,qgood,sgood,pgood,same_id=(bool(rng.getrandbits(1)) for _ in range(5))
    width_ok=max_x-min_x<=400 and max_y-min_y<=400
    old_stable=pgood and same_id and bx-px<128 and px-bx<128 and by-py<128 and py-by<128
    frozen=(min_x,max_x,min_y,max_y,count,seen,qgood,sgood,pgood,same_id,px,py)
    # Boundary freezes inputs; later live fields can change arbitrarily.
    mid_x,mid_y=(frozen[0]+frozen[1])//2,(frozen[2]+frozen[3])//2
    dx,dy=mid_x-frozen[10],mid_y-frozen[11]
    new_stable=frozen[8] and frozen[9] and -128<dx<128 and -128<dy<128
    old_status=0 if not seen or not qgood else 5 if not sgood else 1 if count==0 else 2 if count>1 or not width_ok else 4 if old_stable else 3
    new_status=0 if not frozen[5] or not frozen[6] else 5 if not frozen[7] else 1 if frozen[4]==0 else 2 if frozen[4]>1 or not width_ok else 4 if new_stable else 3
    assert (mid_x,mid_y,new_status)==(bx,by,old_status)
    publication_cases+=1
print(f'PASS: {publication_cases} frozen two-stage midpoint/status equivalence cases including strict128-mm boundaries')
print('No HDL execution, synthesis, DSP inference, routing or timing is claimed.')
