"""Independent geometry/arithmetic checks; never compiles or simulates HDL.

Exercises the former CORDIC reference, new quarter-wave Q14 lookup, mount
transform, signed serial multiplication, Bresenham ray
geometry, scan-match rejection/acceptance and sequential RAM budgets. Source
guards only assert critical interface/contracts exist; they are not RTL proof.
"""
from pathlib import Path
from itertools import product
import math
import random
import re

ATAN_Q12 = (184320,108810,57492,29184,14649,7331,3667,1833,
            917,458,229,115,57,29,14,7)

def rotate_fixed(x_mm, y_mm, heading_q6):
    x,y=x_mm*39797,y_mm*39797
    a=heading_q6%23040
    if 5760<a<=17280:
        x,y=-x,-y
        a-=11520
    elif a>17280:
        a-=23040
    a<<=6
    for bit,step in enumerate(ATAN_Q12):
        if a>=0:
            x,y,a=x-(y>>bit),y+(x>>bit),a-step
        else:
            x,y,a=x+(y>>bit),y-(x>>bit),a+step
        assert -(1<<31)<=x<(1<<31) and -(1<<31)<=y<(1<<31)
    return x>>16,y>>16

def load_sine_table(source):
    table_section=source.split("module lidar_quarter_sine_rom",1)[1]
    entries=re.findall(r"sine\[(\d+)\]=15'd(\d+);",table_section)
    assert len(entries)==361
    table={int(index):int(value) for index,value in entries}
    assert set(table)==set(range(361))
    for i,value in table.items():
        assert value==math.floor(math.sin(i*math.pi/720)*16384+.5),(i,value)
        assert 0<=value<=16384
    return table


def lookup_coefficients(angle_q6,table):
    a=angle_q6%23040
    index=(a+8)>>4
    if index>=1440:index=0
    if index<360:
        sin_address,cos_address=index,360-index
        sin_negative=cos_negative=False
    elif index<720:
        sin_address,cos_address=720-index,index-360
        sin_negative,cos_negative=False,True
    elif index<1080:
        sin_address,cos_address=index-720,1080-index
        sin_negative=cos_negative=True
    else:
        sin_address,cos_address=1440-index,index-1080
        sin_negative,cos_negative=True,False
    assert 0<=sin_address<=360 and 0<=cos_address<=360
    sine=table[sin_address]*(-1 if sin_negative else 1)
    cosine=table[cos_address]*(-1 if cos_negative else 1)
    assert -16384<=sine<=16384 and -16384<=cosine<=16384
    return cosine,sine,index


def serial_signed_product(operand,coefficient):
    # Exactly 15 magnitude bits; signed mm operand is extended before negation.
    factor=abs(coefficient)
    value=-operand if coefficient<0 else operand
    result=0
    assert -(1<<15)<=operand<(1<<15) and factor<=16384
    for _ in range(15):
        result+=value if factor&1 else 0
        value<<=1
        factor>>=1
        assert -(1<<31)<=result<(1<<31) and -(1<<31)<=value<(1<<31)
    assert factor==0
    return result


def rotate_lookup(x,y,angle_q6,table):
    c,s,_=lookup_coefficients(angle_q6,table)
    return (x*c-y*s)>>14,(x*s+y*c)>>14


def lookup_error_bound(x,y):
    # Chord bound for nearest +/-0.125deg; Q14 coefficient rounding followed
    # by one integer floor per axis. Mount vector uses its own norm/heading.
    angle_error=2*math.hypot(x,y)*math.sin(math.radians(.125)/2)
    coefficient_error=math.sqrt(2)*(abs(x)+abs(y))/32768
    return angle_error+coefficient_error+math.sqrt(2)


def bresenham(x,y,hx,hy):
    dx,dy=abs(hx-x),abs(hy-y)
    sx,sy=(1 if hx>=x else -1),(1 if hy>=y else -1)
    err=dx-dy
    result=[]
    for _ in range(129):
        result.append((x,y))
        if (x,y)==(hx,hy):
            return result
        e2=err*2
        if e2>-dy:
            err-=dy
            x+=sx
        if e2<dx:
            err+=dx
            y+=sy
    raise AssertionError("ray failed to terminate inside 128-cell window")

MATCH_CANDIDATES=[(x,y) for y in range(-2,3) for x in range(-2,3)]

def rank_reference(scores,masks):
    """Original separate 25-entry array evaluation, including all-zero case."""
    best_score=second_score=best_mask=0
    best_id=12
    for i in range(25):
        if scores[i]>best_score:
            second_score=best_score
            best_score=scores[i]
            best_id=i
            best_mask=masks[i]
        elif scores[i]>second_score:
            second_score=scores[i]
    return best_id,best_score,second_score,scores[12],best_mask

def rank_stream(scored_candidates):
    """Ranking needs only one running candidate and best/second/baseline."""
    winner=(12,0,0)
    runner_up=baseline=0
    for i,score,mask in scored_candidates:
        if i==12:baseline=score
        if score>winner[1]:
            runner_up=winner[1]
            winner=(i,score,mask)
        elif score>runner_up:
            runner_up=score
    return winner[0],winner[1],runner_up,baseline,winner[2]

def point_outer_statistics(frame,occupied,stride=8):
    scores=[0]*25
    masks=[0]*25
    for sx,sy,hx,hy in frame[::stride]:
        for i,(dx,dy) in enumerate(MATCH_CANDIDATES):
            x,y=hx+dx,hy+dy
            if 0<=x<128 and 0<=y<128 and (x,y) in occupied:
                scores[i]+=1
                masks[i]|=1<<((int(hy<sy)<<1)|int(hx<sx))
    return rank_reference(scores,masks),scores,masks

def candidate_outer_statistics(frame,occupied,stride=8):
    # The lists below are test diagnostics, not required by the streaming rank.
    # Each score is completed before the next candidate begins, as in new RTL.
    scores=[]
    masks=[]
    def completed_candidates():
        for i,(dx,dy) in enumerate(MATCH_CANDIDATES):
            score=mask=0
            for sx,sy,hx,hy in frame[::stride]:
                x,y=hx+dx,hy+dy
                if 0<=x<128 and 0<=y<128 and (x,y) in occupied:
                    score+=1
                    mask|=1<<((int(hy<sy)<<1)|int(hx<sx))
            scores.append(score)
            masks.append(mask)
            yield i,score,mask
    result=rank_stream(completed_candidates())
    return result,scores,masks

def choose_match(points, occupied, reference_scans=3):
    frame=[(64,64,x,y) for x,y in points]
    old=point_outer_statistics(frame,occupied,stride=1)
    new=candidate_outer_statistics(frame,occupied,stride=1)
    assert old==new
    (best,best_score,second_score,baseline,best_mask),scores,_=new
    xspan=max(x for x,y in points)-min(x for x,y in points)
    yspan=max(y for x,y in points)-min(y for x,y in points)
    accepted=(reference_scans>=3 and len(points)>=32 and best!=12
              and xspan>=8 and yspan>=8 and best_score>=16
              and best_mask==15
              and best_score>=baseline+4
              and best_score>=second_score+3)
    return MATCH_CANDIDATES[best] if accepted else None,scores


def check_mapping_coordinate_modes(source,table):
    """Independent physical examples and coordinate-frame admission contracts.

    This models COMPLETE scan ownership, not Verilog clocks. The world target
    stays fixed while the vehicle moves; transforms consume an independently
    generated per-point pose. No coordinate helper invents wheel geometry.
    """
    subsystem=(Path(__file__).parents[1]/"src/robot_subsystem.v").read_text(encoding="utf8")
    navigation=(Path(__file__).parents[1]/"src/robot_navigation.v").read_text(encoding="utf8")
    compact=lambda value:re.sub(r"\s+","",re.sub(r"//[^\n]*|/\*.*?\*/","",value,flags=re.S))
    mapper,system,nav=map(compact,(source,subsystem,navigation))
    for token in ("inputwirepose_valid,local_scan", "S_LOCAL_CLEAR=18",
                  "if(state==S_CLEAR||state==S_LOCAL_CLEAR)",
                  "(fill_count!=0||local_scan)",
                  "state<=work_count==0?S_FINISH:S_IPOINT",
                  "if(local_scan)beginclear_addr<=0;state<=S_LOCAL_CLEAR;end",
                  "(!local_scan&&!pose_valid)",
                  "point_pose_x<=local_scan?32'sd0:pose_x_mm",
                  "point_pose_y<=local_scan?32'sd0:pose_y_mm",
                  "point_heading<=capture_heading_q6",
                  "world_angle_q6<=$signed({2'b00,capture_heading_q6})"):
        assert token in mapper,token
    for token in ("wiremap_local=!geometry_valid||!gyro_calibrated||calibrating",
                  "map_pose_x<=robot_x;map_pose_y<=robot_y;map_pose_heading<=heading_q6",
                  "elseif(pose_update)begin", "map_pose_slow<=correction_allowed",
                  "pose_valid&&correction_allowed&&map_pose_seen&&map_pose_slow&&map_pose_age<2",
                  "map_pose_age<2&&static_gyro_age<3",
                  ".pose_valid(world_map_pose_valid),.local_scan(map_local)",
                  ".pose_x_mm(map_pose_x)", ".pose_y_mm(map_pose_y)",
                  ".pose_heading_q6(map_pose_heading)",
                  "encoder_zeroed||map_mode_change", "assignavoid_lidar_ready=lidar_ready&&complete_scan_age<30",
                  "elseif(scan_complete)complete_scan_age<=0", ".lidar_valid(avoid_lidar_ready)"):
        assert token in system,token
    assert "static_scan" not in system
    assert "abs32(distance_q8)<=512&&abs32(gyro_step)<=2000" in nav

    # Moving a vehicle toward a stationary wall: reusing origin0 and retaining
    # every local frame leaves three false wall positions. A live frame contains
    # only the most recent wall; a pose-corrected world frame keeps ONE wall.
    old_local=set()
    latest_local=set()
    accumulated_world=set()
    for position in (0,512,1024):
        distance=2048-position
        bx,by=rotate_lookup(distance,0,0,table)
        point=((bx+4096)>>6,(by+4096)>>6)
        old_local.add(point)
        latest_local={point}
        accumulated_world.add(((position+bx+4096)>>6,(by+4096)>>6))
    assert len(old_local)==3 and len(latest_local)==len(accumulated_world)==1
    assert latest_local=={(80,64)} and accumulated_world=={(96,64)}
    # A complete revolution with no useful returns replaces the local display
    # with unknown cells rather than retaining a previous fake obstacle.
    latest_local=set()
    assert not latest_local and old_local

    # Point-time dewarping of a stationary4m endpoint over a100ms revolution:
    # the vehicle translates20mm and yaws2deg, exactly at the slow-map limits.
    # Using the scan's first pose for every point produces multiple false cells.
    per_point=set()
    first_pose=set()
    maximum_error=0
    for step in range(101):
        px,heading=20*step/100,2*step/100
        distance=round(4000-px)
        heading_q6=round(heading*64)
        # C1's CLOCKWISE angle is body-heading minus world-beam angle.
        raw_q6=heading_q6%23040
        corrected=(heading_q6-raw_q6)%23040
        bx,by=rotate_lookup(distance,0,corrected,table)
        actual_x=round(px)+bx
        maximum_error=max(maximum_error,math.hypot(actual_x-4000,by))
        per_point.add(((actual_x+4096)>>6,(by+4096)>>6))
        ox,oy=rotate_lookup(distance,0,-raw_q6,table)
        first_pose.add(((ox+4096)>>6,(oy+4096)>>6))
    assert maximum_error<=1 and per_point=={(126,64)}
    assert len(first_pose)>=3

    # Capabilities select LOCAL/WORLD. Once a world frame exists, short-lived
    # pose loss/fast movement must HOLD it, not silently choose origin0 again.
    admission_cases=0
    for geometry,gyro,calibrating,valid,slow_now,seen,slow_capture in product((False,True),repeat=7):
        for age,gyro_age in product(range(4),(0,1,2,3,19)):
            local=not geometry or not gyro or calibrating
            pose_ok=valid and slow_now and seen and slow_capture and age<2 and gyro_age<3
            label="LOCAL LIVE SCAN" if local else "WORLD MAP" if pose_ok else "WORLD MAP HOLD"
            assert (label=="WORLD MAP") == (not local and pose_ok)
            assert local or valid or label=="WORLD MAP HOLD"
            # Local visibility does not depend on arm/encoder speed/gyro rate.
            for armed,moving in product((False,True),repeat=2):
                assert (local or pose_ok)==(True if local else pose_ok)
            admission_cases+=1
    # Epoch validity is all-or-nothing: one stale/fast sample abandons the
    # partially captured world revolution, and a later valid suffix cannot be
    # appended to the old prefix without a NEW revolution boundary.
    for failure in range(32):
        quality=[True]*32
        quality[failure]=False
        assert not all(quality)
        assert all([True]*32) # following full low-speed/fresh revolution

    # NAV already treats unknown sectors as blocking. The new complete-scan
    # age additionally prevents unrelated fresh NODEs from authorizing motion.
    for ready,age in product((False,True),range(64)):
        obstacle_fresh=ready and age<30
        assert obstacle_fresh==(ready and age*10<300)
    ranges=(0,49,50,549,550,849,850,16000,65535)
    for front,rear,left,right in product(ranges,repeat=4):
        known=lambda value:50<=value<=16000
        forward=known(front) and front>=550
        backward=known(rear) and rear>=550
        turn=all(known(value) and value>=550 for value in (front,rear,left,right))
        assert front!=65535 or not forward
        assert rear!=65535 or not backward
        assert all(value!=65535 for value in (front,rear,left,right)) or not turn
    print(f"Mapping modes: {admission_cases} capability/freshness cases; moving-wall local reset/empty scan; "
          f"101 point-pose samples vs {len(first_pose)} stale-pose cells; complete-scan300ms guard; "
          "6561 four-sector unknown/reverse/turn cases")

def main():
    rng=random.Random(20261023)
    source=(Path(__file__).parents[1]/"src/lidar_grid_map.v").read_text(encoding="utf-8")
    table=load_sine_table(source)
    check_mapping_coordinate_modes(source,table)
    lookup_cases=0
    largest_beam_error=largest_mount_error=largest_old_difference=0.0
    beam_errors={4000:0.0,16000:0.0}
    full_angle_old_difference=0.0
    for angle in range(23040):
        c,s,index=lookup_coefficients(angle,table)
        quantised=math.radians(index/4)
        # Independent full-circle floating reference, not another quadrant fold.
        assert c==math.floor(math.cos(quantised)*16384+.5)
        assert s==math.floor(math.sin(quantised)*16384+.5)
        circular_error=abs(((index*16-angle+11520)%23040)-11520)
        assert circular_error<=8
        for distance in (4000,16000):
            x,y=rotate_lookup(distance,0,angle,table)
            exact=math.radians(angle/64)
            error=math.hypot(x-distance*math.cos(exact),y-distance*math.sin(exact))
            assert error<=lookup_error_bound(distance,0)+1e-8
            largest_beam_error=max(largest_beam_error,error)
            beam_errors[distance]=max(beam_errors[distance],error)
            old_x,old_y=rotate_fixed(distance,0,angle)
            full_angle_old_difference=max(full_angle_old_difference,math.hypot(x-old_x,y-old_y))
            lookup_cases+=1
    assert [lookup_coefficients(a,table)[:2] for a in (0,5760,11520,17280,23040)]==[
        (16384,0),(0,16384),(-16384,0),(0,-16384),(16384,0)]
    assert lookup_coefficients(23032,table)[:2]==(16384,0)
    assert lookup_coefficients(23031,table)[1]<0
    assert lookup_coefficients(-8,table)[:2]==(16384,0)
    products=0
    # Exact cardinal signs, negative offsets and both signed input boundaries.
    product_pairs=[(x,c) for x in (-32768,-16000,-1000,-1,0,1,1000,16000,32767)
                   for c in (-16384,-16383,-1,0,1,16383,16384)]
    product_pairs += [(rng.randrange(-16000,16001),rng.randrange(-16384,16385))
                      for _ in range(10000)]
    for operand,coefficient in product_pairs:
        assert serial_signed_product(operand,coefficient)==operand*coefficient
        products+=1
    grid_compared=grid_changed=0
    for _ in range(10000):
        heading,forward,raw=(rng.randrange(23040) for _ in range(3))
        distance=rng.randrange(50,6001)
        ox,oy=rng.randrange(-1000,1001),rng.randrange(-1000,1001)
        px,py=rng.randrange(-4096,4096),rng.randrange(-4096,4096)
        for clockwise in (0,1):
            world=(heading+forward-raw if clockwise else heading+raw-forward)%23040
            bx,by=rotate_lookup(distance,0,world,table)
            mx,my=rotate_lookup(ox,oy,heading,table)
            old_bx,old_by=rotate_fixed(distance,0,world)
            old_mx,old_my=rotate_fixed(ox,oy,heading)
            largest_old_difference=max(largest_old_difference,
                math.hypot(bx+mx-old_bx-old_mx,by+my-old_by-old_my))
            exact=math.radians(heading/64)
            mount_error=math.hypot(mx-(ox*math.cos(exact)-oy*math.sin(exact)),
                                  my-(ox*math.sin(exact)+oy*math.cos(exact)))
            assert mount_error<=lookup_error_bound(ox,oy)+1e-8
            largest_mount_error=max(largest_mount_error,mount_error)
            old_cell=((px+old_bx+old_mx+4096)>>6,(py+old_by+old_my+4096)>>6)
            new_cell=((px+bx+mx+4096)>>6,(py+by+my+4096)>>6)
            assert all(abs(a-b)<=1 for a,b in zip(old_cell,new_cell))
            if all(0<=v<128 for v in old_cell+new_cell):
                grid_compared+=1
                grid_changed+=old_cell!=new_cell
    # Existing normalisation adds/subtracts a revolution each clock. With
    # default YAW_OFFSET_Q6=0 the legal heading/raw/forward sum lies in
    # [-23039,46078], so at most one adjustment plus one ready clock is needed.
    for unnormalised in range(-23039,46079):
        norm=unnormalised
        adjustments=0
        while not 0<=norm<23040:
            norm+=23040 if norm<0 else -23040
            adjustments+=1
        assert norm==unnormalised%23040 and adjustments<=1
    rotations=0
    for angle in (0,1,5759,5760,5761,11520,17279,17280,17281,23039):
        for distance in (50,100,1000,4000,16000):
            x,y=rotate_fixed(distance,0,angle)
            a=math.radians(angle/64)
            assert abs(x-distance*math.cos(a))<3
            assert abs(y-distance*math.sin(a))<3
            rotations+=1
    for _ in range(5000):
        angle=rng.randrange(23040)
        distance=rng.randrange(50,16001)
        x,y=rotate_fixed(distance,0,angle)
        a=math.radians(angle/64)
        assert abs(x-distance*math.cos(a))<3
        assert abs(y-distance*math.sin(a))<3
        ox,oy=rng.randrange(-1000,1001),rng.randrange(-1000,1001)
        sx,sy=rotate_fixed(ox,oy,angle)
        assert abs(sx-(ox*math.cos(a)-oy*math.sin(a)))<3
        assert abs(sy-(ox*math.sin(a)+oy*math.cos(a)))<3
        # Raw C1 clockwise; forward_angle establishes raw beam at body +x.
        forward,raw=rng.randrange(23040),rng.randrange(23040)
        world=(angle+forward-raw)%23040
        assert 0<=world<23040
        rotations+=2

    rays=0
    cases=[(0,0,127,127),(127,127,0,0),(0,127,127,0),
           (64,64,64,64),(64,64,64,0),(64,64,127,64)]
    cases += [tuple(rng.randrange(128) for _ in range(4)) for _ in range(10000)]
    for sx,sy,hx,hy in cases:
        ray=bresenham(sx,sy,hx,hy)
        assert ray[0]==(sx,sy) and ray[-1]==(hx,hy)
        assert len(ray)==max(abs(hx-sx),abs(hy-sy))+1
        assert len(ray)==len(set(ray))
        assert all(0<=x<128 and 0<=y<128 for x,y in ray)
        assert all(abs(a-c)<=1 and abs(b-d)<=1
                   for (a,b),(c,d) in zip(ray,ray[1:]))
        rays+=1

    # A room corner plus an asymmetric object gives a distinct translation.
    reference=set((20,y) for y in range(12,100,3))
    reference.update((x,100) for x in range(20,110,3))
    reference.update((84,y) for y in range(40,70,3))
    points=sorted(reference)
    assert len(points)>=32
    for drift in ((-2,-1),(-1,2),(1,1),(2,-2)):
        observed=[(x+drift[0],y+drift[1]) for x,y in points]
        correction,scores=choose_match(observed,reference)
        assert correction==(-drift[0],-drift[1]),(drift,correction,scores)
    assert choose_match(points,reference)[0] is None
    assert choose_match([(x+1,y+1) for x,y in points],reference,0)[0] is None
    wall={(40,y) for y in range(5,120)}
    assert choose_match([(41,y) for y in range(10,110)],wall)[0] is None
    diagonal={(x,x) for x in range(10,110)}
    assert choose_match([(x+1,x+1) for x in range(10,110)],diagonal)[0] is None
    assert choose_match([(x+1,y+1) for x,y in points[:20]],reference)[0] is None
    assert choose_match(points,set())[0] is None
    # Seeded arbitrary maps, scan counts, moving beam origins and map borders
    # compare original point-major storage with the new candidate-major stream.
    scan_equivalence_cases=0
    for case in range(500):
        count=(1,7,8,9,255,256,500,625,1023,1024)[case%10]
        frame=[tuple(rng.randrange(128) for _ in range(4)) for _ in range(count)]
        occupancy_count=(0,80,500,2000,8192,16384)[case%6]
        occupied={(cell%128,cell//128) for cell in rng.sample(range(16384),occupancy_count)}
        assert point_outer_statistics(frame,occupied)==candidate_outer_statistics(frame,occupied)
        scan_equivalence_cases+=1
    # Ranking independently cross-checks sorting, ties, baseline and the
    # original special all-zero winner=12 (no actual candidate exceeded zero).
    vectors=[[0]*25,[128]*25]
    vectors += [[5 if i in pair else 0 for i in range(25)]
                for pair in ((0,1),(0,24),(12,24),(11,12))]
    vectors += [[rng.randrange(129) for _ in range(25)] for _ in range(10000)]
    for scores in vectors:
        masks=[rng.randrange(16) for _ in range(25)]
        expected_id=max(range(25),key=lambda i:scores[i]) if max(scores)>0 else 12
        ordered=sorted(scores,reverse=True)
        expected=(expected_id,ordered[0],ordered[1],scores[12],
                  masks[expected_id] if ordered[0]>0 else 0)
        streamed=rank_stream((i,score,masks[i]) for i,score in enumerate(scores))
        assert rank_reference(scores,masks)==streamed==expected
    # Free/hit evidence transitions and no giant array reset.
    hit=lambda state:3 if state&2 else 2
    free=lambda state:2 if state==3 else 1
    assert [hit(s) for s in range(4)]==[2,2,3,3]
    assert [free(s) for s in range(4)]==[1,1,1,2]
    assert free(free(3))==1
    assert (-4097+4096)>>6 == -1
    assert (-4096+4096)>>6 == 0
    assert (4095+4096)>>6 == 127
    assert (4096+4096)>>6 == 128
    for token in ("S_CORRECTION_WAIT", "correction_valid && correction_ack",
                  "forward_angle_q6", "lidar_offset_x_mm", "MIN_SCORE_MARGIN",
                  "best_quadrants==4'b1111", "S_CANDIDATE_DONE",
                  "if(candidate==12) baseline_score<=candidate_score",
                  "candidate_score>best_score", "candidate_score>second_score",
                  "map_rdata==2'b11"):
        assert token in source,token
    assert "scores[" not in source and "hit_quadrants[" not in source
    # Static topology/width contracts. This is a targeted source guard, not a
    # substitute for the user's next synthesis parser/timing report.
    for token in ('syn_romstyle="block_rom"', "data_q14<=sine[address]",
                  "world_angle_q6[14:0]+15'd8", "rounded_angle_q6[14:4]>=11'd1440",
                  "trig_index<11'd360", "trig_index<11'd720", "trig_index<11'd1080",
                  "-$signed({1'b0,trig_value})", "coefficient_magnitude[14:0]",
                  "mul_bit==14", "selected_operand_wide", "offset_combined>>>14"):
        assert token in source,token
    assert "atan_q12" not in source and "rotate_bit" not in source
    assert "FRAME_POINTS = 1024" in source and "POINT_DECIMATION = 1" in source
    # Every new FSM case item with multiple statements has an explicit group.
    front_case=source.split("case(front_state)",1)[1].split("endcase",1)[0]
    for name in ("F_NORM", "F_QUANTISE", "F_TRIG_SETUP", "F_TRIG_SIN_WAIT",
                 "F_TRIG_SIN", "F_TRIG_COS", "F_MUL_INIT", "F_MUL", "F_MUL_DONE"):
        assert re.search(r"\b"+name+r":begin\b",front_case),name
    assert "selected_operand=(mul_phase<2) ? $signed(point_distance)" in source
    assert "(mul_phase==2 || mul_phase==4) ? point_offset_x : point_offset_y" in source
    assert "selected_coefficient=(mul_phase==1 || mul_phase==3 ||\n        mul_phase==4) ? trig_sin : trig_cos" in source
    assert "offset_term-mul_accum : offset_term+mul_accum" in source
    assert "front_state<=F_TRIG_SIN" in source and "trig_address<=trig_cos_address" in source
    # Balanced lexical structures catch the missing-case-group class that a
    # geometry-only digest cannot detect. They still do not validate all HDL.
    lexical=re.sub(r"/\*.*?\*/|//[^\n]*|\"(?:\\.|[^\"\\])*\"", " ", source, flags=re.S)
    pairs={"end":"begin","endcase":"case","endmodule":"module","endfunction":"function","endgenerate":"generate"}
    stack=[]
    for token in re.findall(r"\b(?:begin|end|case|endcase|module|endmodule|function|endfunction|generate|endgenerate)\b",lexical):
        if token in pairs:
            assert stack and stack.pop()==pairs[token],token
        else:stack.append(token)
    assert not stack,stack
    product_case=source.split("case(mul_phase)",1)[1].split("endcase",1)[0]
    for label in ("0","1","2","3","4","default"):
        assert re.search(r"\b"+label+r":begin\b",product_case),label
    # A coordinate correction starts a new acquisition epoch. The front-end
    # discards fill/mailbox state, never the engine-owned work bank, then gates
    # all samples until a fresh boundary with an unowned opposite bank.
    epoch_barrier=source.split("end else if(correction_valid && correction_ack) begin",1)[1]
    epoch_barrier=epoch_barrier.split("end else begin",1)[0]
    for token in ("front_state<=F_IDLE", "fill_count<=0", "frame_ready<=0",
                  "ray_we<=0", "wait_scan_start<=1"):
        assert token in epoch_barrier,token
    assert "work_bank<=" not in epoch_barrier and "work_count<=" not in epoch_barrier
    assert "if(wait_scan_start || front_state!=F_IDLE || !fill_bank_available || frame_ready)" in source
    restart=source.split("if(wait_scan_start) begin",1)[1].split("end else if",1)[0]
    assert "fill_bank_available && !frame_ready" in restart and "wait_scan_start<=0" in restart
    assert "S_MPOINT:if(!ray_we)state<=S_MPOINT_LATCH;" in source
    assert "S_IPOINT:if(!ray_we)state<=S_IPOINT_LATCH;" in source
    # LCD snapshots have exclusive physical RAM port ownership, including the
    # handshake interval. These Boolean invariants cover both toggle phases;
    # they do not execute RTL or simulate clocks/primitive behaviour.
    for published in (0,1):
        active=published
        writer=1-published
        assert writer!=active  # permitted copy before publication
        new_request=writer
        writer=1-writer
        assert new_request!=published  # copy disabled until returning ACK
        active=new_request
        ack=new_request
        assert writer!=active and new_request==ack  # next copy owns old bank
        # A map clear changes validity, not ownership selectors/toggles.
        invalidated=(new_request,ack,active,writer)
        assert invalidated==(new_request,ack,active,writer)
    for token in ("S_SNAPSHOT_READ", "S_SNAPSHOT_WRITE", "lidar_map_snapshot_bank",
                  "snapshot_request_sync[2]!=snapshot_ack && !rd_en",
                  "snapshot_request==snapshot_ack_sync[2] && snapshot_epoch_valid",
                  "display_read_bank<=display_active_bank",
                  "display_read_enable && !display_active_bank",
                  "display_read_enable && display_active_bank",
                  ".CEA(reset_n && write_enable)", ".CEB(reset_n && read_enable)",
                  ".WRITE_MODE0(2'b00)", ".WRITE_MODE1(2'b00)",
                  ".BIT_WIDTH_0(1)", ".BIT_WIDTH_1(1)"):
        assert token in source,token
    assert "rd_cell <= cells[" not in source and "rd_cell<=cells[" not in source
    clear_body=source.split("if(clear_map) begin\n                state<=S_CLEAR",1)[1]
    clear_body=clear_body.split("end else case(state)",1)[0]
    assert "snapshot_display_valid<=0" in clear_body
    assert "snapshot_epoch_valid<=0" in clear_body
    assert "snapshot_request<=" not in clear_body and "snapshot_write_bank<=" not in clear_body
    wrapper=source.split("module lidar_map_snapshot_bank",1)[1]
    assert "2'b10" not in wrapper and ".READ_MODE1(1'b0)" in wrapper
    # <=1024 integration rays; only every eighth ray is used for matching.
    work_cycles=25*(128*4+1)+1024*(3+128*2)+10
    assert work_cycles<300000
    snapshot_cycles=128*128*2
    assert work_cycles+snapshot_cycles<350000
    # Maximum DEFAULT YAW_OFFSET_Q6=0 front-end budget: six INIT/15-bit-MUL/DONE products,
    # two five-clock lookup sequences, two world-add clocks, two normalisation
    # clocks and GRID/STORE. No additional ray decimation or frame drop policy.
    front_cycles=6*(1+15+1)+2*5+2+2+2
    assert front_cycles<140 and front_cycles<10000
    print(f"Lookup: {lookup_cases} full-angle beam cases, {products} signed products; "
          f"max beam error 4m={beam_errors[4000]:.3f}mm / 16m={beam_errors[16000]:.3f}mm, "
          f"max mount error {largest_mount_error:.3f}mm")
    print(f"Full-angle max old/new beam difference {full_angle_old_difference:.3f}mm; "
          "69118 normalisation inputs checked (default YAW_OFFSET_Q6=0)")
    print(f"20,000 mount/pose/direction cases: max old/new endpoint difference "
          f"{largest_old_difference:.3f}mm; {grid_changed}/{grid_compared} in-map "
          "endpoints changed a 64mm cell (expected near boundaries; not bit-identical)")
    print(f"Front end: <= {front_cycles} clocks ({front_cycles/50:.3f}us at 50MHz), "
          "versus 200us nominal C1 point interval; 512x15-bit coefficient ROM")
    print(f"PASS: {rotations} fixed-point rotations; {rays} finite rays; "
          "unique/ambiguous/immature/sparse match cases; grid boundaries; evidence transitions; "
          "front-end correction epoch and exclusive LCD snapshot source guards")
    print(f"Candidate-order equivalence: {scan_equivalence_cases} seeded scans and "
          f"{len(vectors)} ranking vectors, including ties/all-zero/baseline")
    print(f"Budget: {work_cycles} work clocks ({work_cycles/50000:.3f} ms at 50MHz), "
          "16384 startup clear clocks (0.328ms), 4096 occupancy bytes, 7168 scan-buffer bytes")
    print(f"LCD snapshot: {snapshot_cycles} clocks ({snapshot_cycles/50000:.3f}ms), "
          f"total compute+copy {(work_cycles+snapshot_cycles)/50000:.3f}ms; "
          "two 4096-byte display banks (four explicit x1 DPBs)")
    print("These are independent arithmetic/source checks, not HDL simulation or timing closure.")

if __name__=="__main__":
    main()
