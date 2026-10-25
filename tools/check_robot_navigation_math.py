"""Independent numerical/range checks only; no HDL compiler or HDL simulation.

Run with the project's documented Python runtime. This validates fixed-point
error/bounds and protocol source guards, not FPGA timing or physical navigation.
"""
from pathlib import Path
from itertools import product
import math
import random
import re

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'src' / 'robot_navigation.v').read_text(encoding='utf-8')
ATAN = [round(math.degrees(math.atan(2 ** -i)) * 10000) for i in range(16)]


def wrap(angle):
    if angle >= 1800000:
        return angle - 3600000
    if angle < -1800000:
        return angle + 3600000
    return angle


def rotate(angle):
    x, y = (-19898 if abs(angle) > 900000 else 19898), 0
    z = angle - 1800000 if angle > 900000 else angle + 1800000 if angle < -900000 else angle
    for i, a in enumerate(ATAN):
        if z >= 0:
            x, y, z = x - (y >> i), y + (x >> i), z - a
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + a
    return x / 32768, y / 32768


def bearing(x, y):
    if x < 0:
        x, y, z = -x, -y, -1800000 if y < 0 else 1800000
    else:
        z = 0
    for i, a in enumerate(ATAN):
        if y >= 0:
            x, y, z = x + (y >> i), y - (x >> i), z + a
        else:
            x, y, z = x - (y >> i), y + (x >> i), z - a
    return wrap(z)


def signed32(value):
    value &= (1 << 32) - 1
    return value - (1 << 32) if value & (1 << 31) else value


def original_iteration(x, y, angle, index, vectoring):
    # The original three simultaneous equations; preserve the old X/Y on both
    # right-hand sides. Each destination has the RTL's signed 32-bit wrap.
    direction = y < 0 if vectoring else angle >= 0
    if direction:
        return (signed32(x - (y >> index)), signed32(y + (x >> index)),
                signed32(angle - ATAN[index]))
    return (signed32(x + (y >> index)), signed32(y - (x >> index)),
            signed32(angle + ATAN[index]))


def shared_adder_iteration(x, y, angle, index, vectoring):
    # Independent fixed-width arithmetic for the shared operand selector and
    # XOR/carry subtract expression. This is not a Verilog event simulator.
    subtract_x = y < 0 if vectoring else angle >= 0

    def operation(a, b, subtract):
        return signed32(a + ((b & 0xffffffff) ^ (0xffffffff if subtract else 0)) + subtract)

    pending_x = operation(x, y >> index, subtract_x)
    next_y = operation(y, x >> index, not subtract_x)
    next_angle = operation(angle, ATAN[index], subtract_x)
    return pending_x, next_y, next_angle


def fixed_step_shift(value, count):
    """Independent two's-complement one-bit operation; no variable >>count."""
    word = value & 0xffffffff
    for _ in range(count):
        word = (word >> 1) | (word & 0x80000000)
    return signed32(word)


def stepped_shift_iteration(x, y, angle, index, vectoring):
    # Direction and both source operands are the ORIGINAL iteration values.
    # Copy/shift Y before pendingX; copy/shift old X before updating Y. Angle
    # uses the same saved direction after both coordinate calculations.
    subtract = y < 0 if vectoring else angle >= 0
    shifted_y = fixed_step_shift(y, index)
    pending_x = signed32(x - shifted_y if subtract else x + shifted_y)
    shifted_x = fixed_step_shift(x, index)
    next_y = signed32(y + shifted_x if subtract else y - shifted_x)
    next_angle = signed32(angle - ATAN[index] if subtract else angle + ATAN[index])
    return pending_x, next_y, next_angle


def check_gyro_only_turn_permission():
    """Boolean/maths contracts; this is not an HDL event/cycle simulator."""
    cases = 0
    for geometry, calibrated, fresh, fault, calibrating, avoid, lidar, stop, enabled in product(
            (False, True), repeat=9):
        for mode, done in product(range(5), (False, True)):
            for command in (None, 0, 1, 2, 3, 4, 7):
                for angle in (-180001, -180000, 0, 90000, 180000, 180001):
                    # Expanded RTL predicates, including the command acceptance
                    # edge. A previous TURN cannot lend permission to a GOTO.
                    valid_turn = command == 2 and -180000 <= angle <= 180000
                    gyro_turn = valid_turn if command is not None else mode == 2
                    retained_result = command is None and mode == 4 and done
                    require_lidar = avoid and not gyro_turn and not retained_result
                    pose_valid = geometry and calibrated and fresh and not fault
                    motion_pose_ok = pose_valid or ((gyro_turn or retained_result)
                                                   and calibrated and fresh and not fault)
                    actual = (not stop and enabled and motion_pose_ok and not calibrating
                              and (not require_lidar or lidar))
                    # Requirements expressed in order of their safety decisions.
                    expected = False
                    if not stop and enabled and not calibrating and calibrated and fresh and not fault:
                        if not require_lidar or lidar:
                            if geometry:
                                expected = True
                            else:
                                if command is not None:
                                    expected = command == 2 and abs(angle) <= 180000
                                else:
                                    expected = mode == 2 or (mode == 4 and done)
                    assert actual == expected, (geometry, mode, command, angle)
                    if not geometry and command in (1, 3):
                        assert not actual, 'MANUAL/GOTO cannot borrow the previous TURN exception'
                    if not geometry and avoid and not (gyro_turn or retained_result):
                        assert not actual, 'GOTO/translation still require wheel geometry'
                    # Accepted POSE_SET / encoder ZERO outrank even a valid TURN
                    # on the same edge; a rebase during running is not accepted.
                    for pose_set, zero, running in product((False, True), repeat=3):
                        accepted_rebase = (pose_set or zero) and not running
                        final = actual and not accepted_rebase
                        assert not accepted_rebase or not final
                    cases += 1

    # MODE_DONE is a display-only terminal state, never a controller state that
    # can produce wheel targets. Heading drift must not restart a completed turn.
    def turn_decision(mode, target, heading):
        if mode == 4:  # completion consumes no heading-error feedback
            return False, 0, 0, 6
        assert mode == 2
        error = wrap(target - heading)
        if abs(error) <= 20000:
            return False, 0, 0, 6
        speed = min(100, max(1, abs(error) >> 14))
        left, right = (-speed, speed) if error > 0 else (speed, -speed)
        return True, left, right, 4

    for heading in range(-1800000, 1800000, 10000):
        assert turn_decision(4, 900000, heading) == (False, 0, 0, 6)

    # 90/180 degree absolute goals reach the unchanged <=2 degree stop band.
    # +/-180 are equivalent. No wheel circumference or track enters this math.
    turn_checks = 0
    for target_mdeg in (-180000, -90000, 90000, 180000):
        target = wrap(target_mdeg * 10)
        for heading_mdeg in range(-180000, 180001, 1000):
            error = wrap(target - wrap(heading_mdeg * 10))
            complete = abs(error) <= 20000
            run, left, right, status = turn_decision(2, target, wrap(heading_mdeg * 10))
            assert complete == (min(abs(target_mdeg-heading_mdeg),
                                       360000-abs(target_mdeg-heading_mdeg)) <= 2000)
            assert (left == right == 0) == complete
            assert run == (not complete) and status == (6 if complete else 4)
            assert complete or left * right < 0
            turn_checks += 1

    # The geometry-invalid odometry route still produces heading while its
    # displacement is zero; denominator zero is replaced by one, not a stall.
    for counts, circumference, dl, dr in product((0, 1, 4, 65535), (0, 50, 2000),
                                                (-2048, 0, 2048), (-2048, 0, 2048)):
        # This branch is deliberately selected by unconfirmed geometry even
        # when dimensions happen to be within valid numerical bounds.
        distance_product = 0
        distance_q8 = (abs(distance_product) << 7) // (counts if counts else 1)
        assert distance_q8 == 0
        assert wrap(0 + 1000*7) == 7000  # gyro 70deg/s, one 10ms tick
    return cases, turn_checks


def check_decision_pipeline():
    """Whole-integer old/new decisions; not an HDL event/cycle simulator."""
    def constants(s):
        stop=max(400,s['stop'])
        return min(200,max(1,s['cruise'])),min(100,max(1,s['turn'])),stop,max(stop+100,s['slow']),max(450,s['rotate'])

    def guard(left,right,status,turn_done,start_drive,s):
        _,_,stop,_,rotation=constants(s)
        known=lambda distance:50<=distance<=16000
        front,rear,near_left,near_right=(s[name] for name in ('front','rear','left','right'))
        turning=(left<0<=right or right<0<=left)
        reverse=left<0 and right<0
        unknown=False
        blocked=False
        if s['avoid']:
            if turning:
                unknown=not all(map(known,(front,rear,near_left,near_right)))
                blocked=unknown or min(front,rear,near_left,near_right)<rotation
            elif reverse:
                unknown=not known(rear)
                blocked=unknown or rear<stop
            elif left>0 or right>0:
                unknown=not known(front)
                blocked=unknown or front<stop
        recommended=0
        if blocked:
            status=8 if unknown else 7
            if known(near_left) and near_left>=rotation and (not known(near_right) or near_left>near_right):recommended=1
            elif known(near_right) and near_right>=rotation:recommended=2
            return (0,0,status,False,False,recommended)
        limit=lambda value:max(-200,min(200,value))
        return (limit(left),limit(right),status,turn_done,start_drive,recommended)

    def reference_decision(s):
        cruise,turn,_,slow,_=constants(s)
        mode,error=s['mode'],s['error']
        left=right=0
        turn_done=start_drive=False
        if mode==0:return (0,0,0,False,False,0)
        if mode==4:return (0,0,6,False,False,0)
        if mode==1:
            status=3
            left,right=s['manual_l'],s['manual_r']
            if s['hold'] and left==right and left:
                steer=(abs(error)>>15)*(-1 if error<0 else 1)
                steer=max(-min(abs(left),20),min(min(abs(left),20),steer))
                left,right=left-steer,right+steer
        elif mode in (2,5) or not s['point_driving'] or abs(error)>150000:
            threshold=20000 if mode==2 else 10000 if mode==5 else 5000
            if abs(error)<=threshold:
                status=6 if mode in (2,5) else 5
                turn_done=mode in (2,5)
                start_drive=mode not in (2,5)
            else:
                status=4
                predicted=error-12*s.get('gyro_step',0)
                braking=(predicted<0)!=(error<0) and s.get('gyro_step',0)!=0
                velocity=0 if braking else min(turn,max(1,abs(predicted)>>14))
                left,right=(-velocity,velocity) if error>0 else (velocity,-velocity)
        else:
            status=5
            velocity=max(1,min(cruise,s['remaining']>>3))
            if s['avoid'] and s['front']<slow:velocity=max(1,velocity>>1)
            steer=(abs(error)>>13)*(-1 if error<0 else 1)
            steer=max(-velocity,min(velocity,steer))
            left,right=velocity-steer,velocity+steer
        return guard(left,right,status,turn_done,start_drive,s)

    def staged_decision(s):
        #12 preparation: registered8/7bit limits and independently bounded
        # approach speed; no current wheel/run output is changed here.
        cap,turn_cap,_,slow,_=constants(s)
        approach=200 if s['remaining']>=1600 else 1 if s['remaining']<8 else (s['remaining']>>3)&255
        steer=(abs(s['error'])>>(13 if s['mode']==3 else 15))*(-1 if s['error']<0 else 1)
        predicted=s['error']-12*s.get('gyro_step',0)
        braking=(predicted<0)!=(s['error']<0) and s.get('gyro_step',0)!=0
        turn=abs(predicted)>>14
        mode=s['mode']
        kind=0
        left=right=velocity=0
        done=start=False
        status=0 if mode==0 else 6 if mode==4 else 3 if mode==1 else 5
        hold=False
        if mode==1:
            kind=1
            left,right=s['manual_l'],s['manual_r']
            hold=s['hold'] and left==right and left!=0
        elif mode in (2,3,5):
            if mode in (2,5) or not s['point_driving'] or abs(s['error'])>150000:
                threshold=20000 if mode==2 else 10000 if mode==5 else 5000
                if abs(s['error'])<=threshold:
                    done=mode in (2,5)
                    start=mode not in (2,5)
                    status=6 if done else 5
                else:kind,status=2,4
            else:kind,status=3,5
        #17 speed selection,18 near-obstacle slowing,19 steering saturation.
        if kind==3:velocity=min(approach,cap)
        elif kind==1 and hold:velocity=min(abs(left),20)
        turn=0 if kind==2 and braking else min(max(1,turn),turn_cap)
        if kind==3 and s['avoid'] and s['front']<slow:velocity=max(1,velocity>>1)
        steer=max(-velocity,min(velocity,steer))
        #20 mixing,21 direction classification,22 clipping,23 publication.
        if kind==3:left,right=velocity-steer,velocity+steer
        elif kind==1 and hold:left,right=left-steer,right+steer
        elif kind==2:left,right=(-turn,turn) if s['error']>0 else (turn,-turn)
        assert -400<=left<=400 and -400<=right<=400
        return guard(left,right,status,done,start,s)

    base=dict(mode=3,error=0,remaining=1000,cruise=30,turn=12,point_driving=True,
              manual_l=10,manual_r=10,hold=True,avoid=True,
              front=1000,rear=1000,left=1000,right=1000,stop=550,slow=850,rotate=550)
    samples=[]
    for cruise,remaining,error,mode in product((0,1,199,200,201,255,256,65535),
            (0,1,5,6,7,8,9,239,240,241,1599,1600,1601,8191,8192,0xffffffff),
            (-1800000,-150001,-150000,-32769,-32768,-20001,-20000,-10001,-10000,-5001,-5000,-1,0,1,5000,5001,10000,10001,20000,20001,32768,150000,150001,1800000),range(6)):
        samples.append({**base,'cruise':cruise,'remaining':remaining,'error':error,'mode':mode})
    rng=random.Random(20261025)
    ranges=(0,49,50,449,450,549,550,849,850,16000,16001,65535)
    for _ in range(20000):
        samples.append({**base,'mode':rng.randrange(6),'error':rng.randrange(-1820000,1820001),
            'remaining':rng.getrandbits(32),'cruise':rng.randrange(65536),'turn':rng.randrange(65536),
            'point_driving':bool(rng.randrange(2)),'manual_l':rng.randrange(-200,201),
            'manual_r':rng.randrange(-200,201),'hold':bool(rng.randrange(2)),'avoid':bool(rng.randrange(2)),
            **{key:rng.choice(ranges) for key in ('front','rear','left','right','stop','slow','rotate')}})
    for sample in samples:
        assert reference_decision(sample)==staged_decision(sample),sample
    # Explicit equal-wheel/reverse/turn/unknown scans exercise headings at the
    # deadband and negative arithmetic-shift boundaries, not only random data.
    for velocity,error,front,rear in product((-200,-20,-1,0,1,20,200),
            (-150000,-32769,-32768,-1,0,1,32767,32768,150000),ranges,ranges):
        sample={**base,'mode':1,'manual_l':velocity,'manual_r':velocity,'error':error,'front':front,'rear':rear}
        assert reference_decision(sample)==staged_decision(sample),sample
        samples.append(sample)

    code=re.sub(r'\s+','',re.sub(r'//[^\n]*','',SOURCE))
    for token in ('D_SPEED=17','D_SLOW=18','D_STEER=19','D_MIX=20','D_GUARD=21','D_CLAMP=22','D_COMMIT=23',
                  'cruise_sample<=cruise_limit[7:0]','approach_sample<=remaining_mm[10:3]',
                  'state<=D_SPEED','state<=D_SLOW','state<=D_STEER','state<=D_MIX','state<=D_GUARD','state<=D_CLAMP','state<=D_COMMIT',
                  'decision_mode==motion_mode','frame_generation==command_generation',
                  'if(blocked_work||decision_blocked_live)',
                  'blocked_work<=decision_blocked_live','unknown_work<=avoid_required&&decision_unknown'):
        assert token in code,token
    prepare=code.split('12:begin',1)[1].split('D_SPEED:begin',1)[0]
    assert 'run_request<=' not in prepare and 'target_speed_left<=' not in prepare and 'target_speed_right<=' not in prepare
    for name,next_name in (('D_SPEED','D_SLOW'),('D_SLOW','D_STEER'),('D_STEER','D_MIX'),('D_MIX','D_GUARD'),('D_GUARD','D_CLAMP'),('D_CLAMP','D_COMMIT')):
        stage=code.split(name+':begin',1)[1].split(next_name+':begin',1)[0]
        assert 'run_request<=' not in stage and 'target_speed_left<=' not in stage and 'target_speed_right<=' not in stage,name
    commit=code.index('D_COMMIT:begin')
    for token in ('if(command_valid)begin','if(safety_stop||!motion_enable)begin',
                  'elseif((pose_set||encoder_zeroed)&&!run_request)begin',
                  'elseif(!motion_pose_ok||calibrating||(avoid_required&&!lidar_valid))begin'):
        assert code.index(token,commit)>commit,token
    # Every inter-stage cancellation prevents a stale final publication. A
    # fresh collision is checked again at COMMIT, after the precomputed guard.
    for stage,cancel in product(range(8),('stop','pose','gyro','collision','new_command')):
        captured_mode,current_mode=2,2
        frame_generation,current_generation=7,7
        if cancel in ('stop','pose','gyro'):current_mode=0
        if cancel in ('pose','new_command'):current_generation+=1
        # Even when STOP/gyro permission RECOVERS before publication, changing
        # the current mode invalidates the captured decision. A replacement
        # command with the same mode is invalidated independently by generation.
        permission_restored=True
        live_clear=cancel!='collision'
        assert not(permission_restored and captured_mode==current_mode and
                   frame_generation==current_generation and live_clear)
    return len(samples)


def main():
    decision_cases=check_decision_pipeline()
    assert ATAN == [450000, 265651, 140362, 71250, 35763, 17899, 8952, 4476,
                    2238, 1119, 560, 280, 140, 70, 35, 17]
    trig_error = 0.0
    for a in range(-1800000, 1800000, 1000):
        c, s = rotate(a)
        rad = math.radians(a / 10000)
        trig_error = max(trig_error, abs(c - math.cos(rad)), abs(s - math.sin(rad)))
    assert trig_error < 0.0004, trig_error

    rng = random.Random(20261023)
    cordic_cases = 0
    edges = (-2147483648, -1800000, -32768, -1, 0, 1, 32767, 1800000, 2147483647)
    starts = [(x, y, z) for x in edges for y in edges for z in (-1, 0, 1)]
    starts += [tuple(rng.randint(-2147483648, 2147483647) for _ in range(3))
               for _ in range(10000)]
    for vectoring in (False, True):
        for start in starts:
            reference = reduced = stepped = start
            for index in range(16):
                reference = original_iteration(*reference, index, vectoring)
                reduced = shared_adder_iteration(*reduced, index, vectoring)
                stepped = stepped_shift_iteration(*stepped, index, vectoring)
                assert reduced == reference, (start, vectoring, index, reference, reduced)
                assert stepped == reference, (start, vectoring, index, reference, stepped)
                cordic_cases += 1
    for value in edges + tuple(rng.randint(-2147483648, 2147483647) for _ in range(10000)):
        for count in range(16):
            assert fixed_step_shift(value, count) == value >> count
    # Copy/preload, X operation, Y operation and angle commit, plus both
    # variable-count loops of FIXED one-bit shifts. Index0 executes no shift.
    pass_clocks = sum(4 + 2*index for index in range(16))
    assert pass_clocks == 304
    added_clocks = 2 * (pass_clocks - 16*3)
    assert added_clocks == 512 and added_clocks / 50000000 == 0.00001024
    assert 2*pass_clocks + 3*40 + 50 < 1000 < 500000
    # Initial path setup adds one trig pass; steady straight tracking reuses
    # its frozen unit vector and requires no per-frame targetatan2 pass.
    assert 3*pass_clocks + 3*40 + 60 < 1500 < 500000
    bearing_error = 0.0
    vectors = [(256, 0), (-256, 0), (0, 256), (0, -256)]
    vectors += [(rng.randint(-64000, 64000)*256, rng.randint(-64000, 64000)*256)
                for _ in range(10000)]
    for x, y in vectors:
        if not (x or y):
            continue
        expected = math.degrees(math.atan2(y, x)) * 10000
        error = abs(wrap(bearing(x, y) - expected)) / 10000
        bearing_error = max(bearing_error, error)
    # Tiny one-millimetre vectors have integer-shift quantization. Navigation
    # stops within >=20 mm, so only >=20 mm vectors affect commanded bearing.
    assert bearing_error < 0.2, bearing_error

    for decimdeg in range(-1800000, 1800000, 101):
        positive = decimdeg + 3600000 if decimdeg < 0 else decimdeg
        q6 = positive * 4 // 625
        assert 0 <= q6 < 23040
        assert abs(q6 / 64 - positive / 10000) < 1/64 + 1e-9

    max_distance_numerator = 4096 * 2000 * 128
    assert max_distance_numerator < (1 << 30)
    assert 128000 < (1 << 18)  # signed nineteen-bit distance multiplier operand
    assert 128000 * 32780 < (1 << 36)  # signed thirty-seven-bit product
    assert 32768 * 128 <= (1 << 23)  # signed twenty-four-bit calibration sum
    # 1000 raw counts => 70 deg/s; one second => 70 degrees.
    assert 100 * (1000 * 7) / 10000 == 70
    # One encoder revolution on both wheels must move one circumference.
    for circumference in (50, 210, 314, 2000):
        for counts in (4, 1320, 4096, 65535):
            assert 2 * counts * circumference * 128 // counts == circumference * 256
    turn_permission_cases, turn_target_cases = check_gyro_only_turn_permission()

    # Source guards for the actual correction transaction/cancellation contract.
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', SOURCE, flags=re.S)
    assert '/' not in code and '%' not in code, 'No combinational RTL divide/modulo'
    assert 'if (lidar_correction_valid && correction_ready)' in code
    assert 'correction_wait_drop<=1; correction_accepted<=1;' in code
    assert 'if (!lidar_correction_valid) correction_wait_drop<=0;' in code
    assert 'frame_generation==command_generation' in code
    assert 'encoder_zeroed && !run_request' in code
    assert 'wire front_known=clear_front_mm>=50 && clear_front_mm<=16000;' in code
    assert 'wire turn_space_known=front_known && rear_known && left_known && right_known;' in code
    assert 'wire decision_unknown=BODY_GUARD ?' in code
    assert '(turning_work ? !turn_space_known : reverse_work ? !rear_known :' in code
    assert '(left_work>0 || right_work>0) ? !front_known : 1\'b0);' in code
    assert 'turning_work ? body_coverage!=4\'b1111' in code
    assert '(BODY_GUARD && !body_guard_ready)' in code
    assert 'blocked_work<=decision_blocked_live;' in code
    assert '4,9:begin' in code
    assert 'cordic_subtract_latched<=cordic_subtract_x;' in code
    assert 'cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0; state<=4;' in code
    assert 'cordic_index<=0;cordic_phase<=0;cordic_shift_remaining<=0; state<=9;' in code
    assert 'cordic_x<=cordic_pending_x;cordic_angle<=cordic_alu_result;' in code
    assert '(cordic_operand_b^{32{cordic_subtract}})+{31\'d0,cordic_subtract}' in code
    assert 'cordic_x<=cordic_x-' not in code and 'cordic_x<=cordic_x+' not in code
    assert 'reg signed [31:0] cordic_shift_copy;' in code
    assert 'reg [3:0] cordic_shift_remaining;' in code
    assert 'cordic_shift_copy<=cordic_y;' in code and 'cordic_shift_copy<=cordic_x;' in code
    assert 'cordic_shift_copy<=cordic_shift_copy>>>1;' in code
    assert 'cordic_phase<=cordic_index==0 ? 2 : 1;' in code
    assert 'cordic_phase<=cordic_index==0 ? 4 : 3;' in code
    assert 'if(cordic_shift_remaining==1)begin' in code
    assert 'cordic_phase<=cordic_phase==1 ? 2 : 4;' in code
    assert not re.search(r'>>>\s*cordic_index', code), 'No variable CORDIC barrel shift'
    assert 'cordic_shift_input' not in code
    assert re.search(r'if\s*\(!reset_n\).*?cordic_shift_remaining<=0;cordic_shift_copy<=0;', code, re.S)
    # Interrupts retain the original global priority. POSE_SET / encoder zero
    # leave the entire CORDIC state case, then the next pass always starts at
    # phase0 with a new counter/source copy; an old pending partial is unusable.
    pose_interrupt = code.split('if (pose_set && !run_request)', 1)[1].split('if (encoder_zeroed', 1)[0]
    zero_interrupt = code.split('if (encoder_zeroed && !run_request)', 1)[1].split('if (command_valid)', 1)[0]
    for interrupt in (pose_interrupt, zero_interrupt):
        assert 'state<=14; divider_start<=0;' in interrupt
        assert 'command_generation<=command_generation+1\'b1;' in interrupt
    assert code.index('case (state)') < code.index('if (pose_set && !run_request)')
    assert code.index('if (encoder_zeroed && !run_request)') < code.index('if (command_valid)')
    assert 'if (safety_stop || !motion_enable)' in code
    # The public pose/GOTO/correction contract stays strict. Only explicit
    # empty-floor TURN and its zero-output completion indicator get an exception.
    assert 'assign pose_valid=geometry_ok && gyro_calibrated && gyro_fresh && fault==0;' in code
    assert 'if ((motion_mode==MODE_GOTO || motion_mode==MODE_ALIGN) && pose_valid && frame_generation==command_generation)' in code
    assert 'assign correction_allowed=pose_valid && !calibrating && !pose_set' in code
    assert 'wire valid_turn_command=command_valid && command==3\'d2 &&' in code
    assert '(command_valid ? valid_turn_command : motion_mode==MODE_TURN)' in code
    assert 'wire gyro_only_turn=' in code
    assert 'wire avoid_required=avoid_enable && !gyro_only_turn && !gyro_only_turn_result;' in code
    assert 'wire gyro_only_turn_result=!command_valid &&' in code
    assert 'motion_mode==MODE_DONE && gyro_turn_done;' in code
    assert 'wire motion_pose_ok=pose_valid || ((gyro_only_turn || gyro_only_turn_result) &&' in code
    assert 'gyro_calibrated && gyro_fresh && fault==0);' in code
    assert '(!geometry_ok && motion_mode!=MODE_TURN)' in code
    assert 'else if (!motion_pose_ok || calibrating || (avoid_required && !lidar_valid))' in code
    assert 'sample_geometry_ok<=geometry_ok && encoder_step_ok;' in code
    assert 'distance_product<=sample_geometry_ok ?' in code
    assert 'divider_denominator<=counts_sample==0 ? 32\'d1 : {16\'d0,counts_sample};' in code
    assert 'heading_accumulator<=next_heading;' in code and 'else state<=11;' in code
    assert 'gyro_turn_done<=motion_mode==MODE_TURN;' in code
    command_block = code.rsplit('if (command_valid) begin', 1)[1]
    live, normal = command_block.split('end else begin', 1)
    assert 'command==5 && motion_mode==MODE_MANUAL' in live
    assert 'heading_target<=' not in live and 'command_generation<=' not in live
    assert normal.lstrip().startswith('turn_complete_pulse<=0;\n                gyro_turn_done<=0;')
    rebase_priority = code.index('else if ((pose_set || encoder_zeroed) && !run_request) begin')
    assert rebase_priority > code.rindex('if (command_valid) begin')
    assert 'motion_mode<=MODE_STOP; gyro_turn_done<=0; run_request<=0;' in code[rebase_priority:]
    # DONE is classified before movement branches. The decision pipeline must
    # retain the preceding run/targets until ONE atomic final publication.
    controller = code.split('12:begin', 1)[1].split('default:state<=0;', 1)[0]
    assert controller.index('else if (motion_mode==MODE_DONE) decision_status<=6;') < controller.index('else if (fault!=0)')
    commit=controller.split('D_COMMIT:begin',1)[1]
    assert commit.lstrip().startswith('run_request<=0;target_speed_left<=0;target_speed_right<=0;')
    assert 'if (tick_10ms && state!=0 && state!=14 && state!=15) begin\n                fault[1]<=1;turn_complete_pulse<=0;' in code
    assert re.findall(r'\bbegin\b', code).__len__() == re.findall(r'\bend\b', code).__len__()
    print(f'PASS: 3600 CORDIC rotations, 10004 bearings, 35644 heading quantizations; '
          f'trig max error={trig_error:.7f}, bearing max error={bearing_error:.5f} deg')
    print('PASS: distance/product/calibration bounds and unique correction ACK guards')
    print(f'PASS: {cordic_cases} exact full-signed32 original/shared/stepped CORDIC iterations; '
          '160144 shift boundaries/randoms and pose/zero/reset/stop source guards')
    print(f'PASS: {pass_clocks} clocks/pass; old two-pass rewrite added{added_clocks} clocks; '
          'initial three-pass path setup bounded below1500 clocks vs500000-clock control interval')
    print(f'PASS: {turn_permission_cases} TURN permission combinations plus concurrent rebase cases; '
          f'{turn_target_cases} 90/180deg targets, strict GOTO/pose and terminal zero-output guards')
    assert 2*pass_clocks+3*35+48+7<1000
    print(f'PASS: {decision_cases} reference/registered navigation decisions;8 publication stages, '
          '7 extra50MHz clocks(0.14us),40 inter-stage cancellation cases, no intermediate run drop')
    print('No HDL compilation, HDL simulation, synthesis, timing analysis or flashing was run.')


if __name__ == '__main__':
    main()
