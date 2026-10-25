"""Exact arithmetic/range proofs for the resource-reduced wheel controller.

This runs independent Python integer calculations and source guards only.
It does NOT compile, elaborate, simulate or synthesize any HDL, generate a
waveform, run timing analysis, or interact with the FPGA/motors.
"""
from pathlib import Path
import random
import re

ROOT = Path(__file__).resolve().parents[1]
RNG = random.Random(20261023)
I32_MIN, I32_MAX = -(1 << 31), (1 << 31)-1


def fits(value, bits):
    assert -(1 << (bits-1)) <= value < (1 << (bits-1)), (value, bits)
    return value


def sat32(value):
    return max(I32_MIN, min(I32_MAX, value))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def round_original(value):
    return sat32(-((-value + 128) >> 8) if value < 0 else (value + 128) >> 8)


def round_reduced(value):
    fits(value, 42)
    return sat32((value + (127 if value < 0 else 128)) >> 8)


def product_shift_add(value, gain):
    """Mathematical signed product by right-shifting a partial product.

    A signed33 upper partial sum plus the remaining unsigned16 multiplier
    represents the same 16-term expansion. This integer algorithm is not a
    clocked RTL simulator; every partial addition is range checked.
    """
    assert I32_MIN <= value <= I32_MAX and 0 <= gain <= 65535
    upper,lower=0,gain
    for _ in range(16):
        upper_sum=fits(upper+(value if lower&1 else 0),33)
        lower=((upper_sum&1)<<15)|(lower>>1)
        upper=upper_sum>>1
    result=(upper<<16)|lower
    assert abs(value*gain)<(1<<47)
    fits(result, 48)
    return result


def shared_add_sub(a,b,subtract=False):
    """One 50-bit binary addition with its low bit supplying carry-in."""
    fits(a,49);fits(b,49)
    mask=(1<<49)-1
    operand_b=(b&mask)^(mask if subtract else 0)
    full=(((a&mask)<<1)|1)+(operand_b<<1)+int(subtract)
    encoded=(full>>1)&mask
    result=encoded-(1<<49) if encoded&(1<<48) else encoded
    assert result==fits(a-b if subtract else a+b,49)
    return result


def clamp_via_25(value,limit):
    """Sign-extension fit test plus a signed25 clamp, full signed49 input."""
    fits(value,49)
    assert 0<=limit<=32768*256
    encoded=value&((1<<49)-1)
    low=encoded&((1<<25)-1)
    signed_low=low-(1<<25) if low&(1<<24) else low
    fits25=(encoded>>24)==(((1<<25)-1) if low&(1<<24) else 0)
    above=(not fits25 and value>=0) or (fits25 and signed_low>limit)
    below=(not fits25 and value<0) or (fits25 and signed_low < -limit)
    bounded=limit if above else -limit if below else signed_low
    assert bounded==clamp(value,limit)
    return bounded,above,below


def pid_original(error, measurement_delta, kp, ki, kd, previous_i, limit):
    # Independent behavioral oracle: Ki0 discards any formerly weighted I;
    # antiwindup rejects the new I state, not the candidate output for this tick.
    previous_i = previous_i if ki else 0
    p, step, d = error*kp, error*ki, -measurement_delta*kd
    candidate = clamp(previous_i + step, limit << 8)
    raw = p + candidate + d
    if (raw > limit*256 and error > 0) or (raw < -limit*256 and error < 0):
        selected_i = clamp(previous_i, limit << 8)
    else:
        selected_i = candidate
    return selected_i, round_original(clamp(raw, limit << 8))


def pid_reduced(error, measurement_delta, kp, ki, kd, previous_i, limit):
    fits(previous_i, 25)
    previous_i = previous_i if ki else 0
    pd_sum = product_shift_add(error, kp)
    step = product_shift_add(error, ki)
    integral_sum=shared_add_sub(previous_i,step)
    candidate=fits(clamp_via_25(integral_sum,limit*256)[0],25)
    derivative=product_shift_add(measurement_delta,kd)
    pd_sum=shared_add_sub(pd_sum,derivative,True)
    command=shared_add_sub(pd_sum,candidate)
    _,above,below=clamp_via_25(command,limit*256)
    reject_i=(above and error>0) or (below and error<0)
    if reject_i:
        candidate=clamp_via_25(previous_i,limit*256)[0]
    chosen_i=candidate
    bounded=fits(clamp_via_25(command,limit*256)[0],25)
    rounded = fits((bounded + (127 if bounded < 0 else 128)) >> 8, 17)
    return chosen_i, rounded


def check_pid_tuning_behavior():
    """Expected tuning behavior, expressed as independent discrete equations.

    These are controller-level algebraic checks, not execution/elaboration of
    the RTL. Gains have the real Q8 units and PWM limits are permille.
    """
    cases = 0
    for direction in (-1, 1):
        # Concrete saturation regressions: rolling back I must not roll back
        # this tick's already-saturated PWM request below the user's cap.
        integral, commands = 0, []
        for _ in range(8):
            integral, output = pid_reduced(direction*3, 0, 50*256, 5*256, 0, integral, 200)
            commands.append(output)
            cases += 1
        assert commands == [direction*v for v in (165, 180, 195, 200, 200, 200, 200, 200)]
        assert integral == direction*45*256
        frozen = integral
        for _ in range(100):
            integral, output = pid_reduced(direction*3, 0, 50*256, 5*256, 0, integral, 200)
            assert integral == frozen and output == direction*200
            cases += 1
        for _ in range(8):
            integral2, output = pid_reduced(direction*2, 0, 80*256, 30*256, 0, 0, 200)
            assert integral2 == 0 and output == direction*200
            cases += 1
        # Coming off the rail is immediate; opposite error can unwind rather
        # than accumulating an arbitrary amount of hidden integral while capped.
        integral, output = pid_reduced(direction, 0, 50*256, 5*256, 0, frozen, 200)
        assert integral == direction*50*256 and output == direction*100
        integral, output = pid_reduced(-direction, 0, 50*256, 5*256, 0, integral, 200)
        assert integral == direction*45*256 and output == -direction*5
        cases += 2
        # A live reduction must clamp the stored old I too. Rejected updates
        # cannot preserve a former900permille bias behind a50permille limit.
        integral, output = pid_reduced(direction*3, 0, 50*256, 5*256, 0,
                                       direction*500*256, 50)
        assert integral == direction*50*256 and output == direction*50
        integral, output = pid_reduced(0, 0, 50*256, 5*256, 0, integral, 200)
        assert integral == direction*50*256 and output == direction*50
        cases += 2
        # Switching Ki off clears the formerly WEIGHTED I term in both loops;
        # merely multiplying the next error by0 would leave an old bias.
        for previous in (-32768*256, -900*256, 0, 900*256, 32768*256):
            integral, output = pid_reduced(direction*3, direction*2,
                                           50*256, 0, 10*256, previous, 200)
            assert integral == 0 and output == direction*130
            cases += 1
        # The PWM slew may approach the rail gradually, but it must reach the
        # configured cap, obey a lower live cap immediately, and pass through0
        # on polarity reversal. The min-duty setting remains independently0.
        duty = 0
        for _ in range(20):
            duty = duty_reduced(direction*200, duty, 0, 200)
            cases += 1
        assert duty == direction*200
        duty = duty_reduced(direction*50, duty, 0, 50)
        assert duty == direction*50
        duty = duty_reduced(-direction*50, duty, 0, 50)
        assert duty == 0
        duty = duty_reduced(-direction*50, duty, 0, 50)
        assert duty == -direction*15
        cases += 3

    # A default speed deadband of0 keeps a real one-count error visible during
    # low-speed tuning. This is separate from the user's adjustable PWM floor.
    for direction in (-1, 1):
        measured_error = direction if abs(direction) > 0 else 0
        integral, output = pid_reduced(measured_error, 0, 256, 0, 0, 900*256, 200)
        assert integral == 0 and output == direction
        assert duty_reduced(output, 0, 0, 200) == direction
        cases += 1

    for _ in range(3000):
        new_cap = RNG.randint(0, 900)
        previous = RNG.randint(-900*256, 900*256)
        args = (RNG.randint(-200, 200), RNG.randint(-200, 200),
                RNG.randrange(32768), RNG.randrange(32768), RNG.randrange(32768),
                previous, new_cap)
        integral, output = pid_reduced(*args)
        assert (integral, output) == pid_original(*args)
        assert abs(integral) <= new_cap*256 and abs(output) <= new_cap
        cases += 1
    print(f'PASS: {cases} P/PI tuning cases; both saturation rails use the full cap, '
          'Ki0 clears I, live cap changes clamp old I, recovery and PWM slew preserve limits')


def duty_original(command, previous, minimum, limit, slew=15):
    floor = min(minimum, limit)
    if 0 < command < floor:
        command = floor
    elif -floor < command < 0:
        command = -floor
    if previous*command < 0 or command == 0:
        return 0
    return clamp(previous + clamp(command-previous, slew), limit)


def duty_reduced(command, previous, minimum, limit, slew=15):
    value = fits(command, 11)
    floor = fits(min(minimum, limit), 11)
    if 0 < value < floor:
        value = floor
    elif -floor < value < 0:
        value = -floor
    if previous*value < 0 or value == 0:
        return 0
    delta = fits(value-previous, 12)
    delta = fits(clamp(delta, slew), 12)
    intermediate = fits(previous+delta, 13)
    return clamp(intermediate, limit)


def check_arithmetic():
    product_cases = 0
    for value in (I32_MIN, -1, 0, 1, I32_MAX):
        for gain in range(65536):
            assert product_shift_add(value, gain) == value*gain
            product_cases += 1
    for _ in range(20000):
        value, gain = RNG.randint(I32_MIN, I32_MAX), RNG.randrange(65536)
        assert product_shift_add(value, gain) == value*gain
        product_cases += 1

    clamp_cases=0
    for limit in (0,1,255,256,900*256,32768*256):
        for value in (-(1<<48),(1<<48)-1,-(1<<24)-1,-(1<<24),
                      (1<<24)-1,1<<24,-limit-1,-limit,-limit+1,limit-1,limit,limit+1):
            clamp_via_25(value,limit)
            clamp_cases+=1
    for _ in range(20000):
        clamp_via_25(RNG.randint(-(1<<48),(1<<48)-1),RNG.randint(0,32768*256))
        clamp_cases+=1

    rounding_values = {0, -(1 << 39), (1 << 39)-256, -(1 << 40)+256, (1 << 40)-256}
    for integer in (-4294967295, I32_MIN, -32768, -1000, -1, 0, 1, 900, 32768, I32_MAX, 4294967295):
        for fraction in range(-257, 258):
            rounding_values.add(integer*256 + fraction)
    for _ in range(20000):
        rounding_values.add(RNG.randint(-(1 << 40)+256, (1 << 40)-256))
    for value in rounding_values:
        assert round_original(value) == round_reduced(value)

    filter_state, derivative_state, previous_speed = 0, 0, 0
    position_state = 0
    raw_values = [I32_MIN, I32_MAX]*1000
    raw_values += [RNG.randint(I32_MIN, I32_MAX) for _ in range(30000)]
    for raw in raw_values:
        original_next = filter_state + ((raw*256-filter_state) >> 2)
        difference = fits(raw*256-filter_state, 41)
        reduced_next = fits(filter_state + (difference >> 2), 40)
        assert original_next == reduced_next
        filter_state = reduced_next
        current_speed = round_reduced(filter_state)
        measurement = fits(current_speed-previous_speed, 33)
        original_derivative = derivative_state + ((measurement*256-derivative_state) >> 2)
        difference = fits(measurement*256-derivative_state, 42)
        derivative_state = fits(derivative_state + (difference >> 2), 41)
        assert derivative_state == original_derivative
        assert round_reduced(derivative_state) == round_original(derivative_state)
        previous_speed = current_speed
        original_position = position_state + ((raw*256-position_state) >> 2)
        position_state = fits(position_state + (fits(raw*256-position_state, 41) >> 2), 40)
        assert position_state == original_position

    pid_cases = 0
    edge_errors = (I32_MIN, I32_MIN+1, -2000000, -1, 0, 1, 2000000, I32_MAX)
    edge_gains = (0, 1, 255, 256, 32767, 32768, 65535)
    for error in edge_errors:
        for delta in edge_errors:
            for gain in edge_gains:
                for limit in (0, 1, 900, 1000, 32768):
                    for old_i in (-32768*256, 0, 32768*256):
                        args = error, delta, gain, 65535-gain, gain, old_i, limit
                        assert pid_original(*args) == pid_reduced(*args)
                        pid_cases += 1
    for _ in range(20000):
        limit_max = 32768 if RNG.getrandbits(1) else 900
        args = (RNG.randint(I32_MIN, I32_MAX), RNG.randint(I32_MIN, I32_MAX),
                RNG.randrange(65536), RNG.randrange(65536), RNG.randrange(65536),
                RNG.randint(-limit_max*256, limit_max*256), RNG.randint(0, limit_max))
        result = pid_reduced(*args)
        assert result == pid_original(*args)
        if limit_max == 900:
            fits(result[0], 19)
        pid_cases += 1

    duty_cases = 0
    for _ in range(50000):
        limit = RNG.randint(0, 900)
        args = (RNG.randint(-limit, limit), RNG.randint(-900, 900),
                RNG.randint(0, 1023), limit, RNG.choice((0, 1, 15, 900, 1000)))
        assert duty_original(*args) == duty_reduced(*args)
        duty_cases += 1
    # Each complete signed16 duty maps to the same exact default PWM width.
    for duty in range(-32768, 32768):
        magnitude = min(abs(duty), 1000)
        assert magnitude*2500//1000 == (5*magnitude)//2
    print(f'PASS: {product_cases} full-range gain products, {len(rounding_values)} rounding '
          f'values, {len(raw_values)} filter updates, {pid_cases} PID, {duty_cases} duty cases; '
          f'{clamp_cases} full-signed49 narrow-clamp equivalence cases')


def check_source_contracts():
    source = (ROOT/'src/motor_axis_control.v').read_text(encoding='utf-8')
    bridge = (ROOT/'src/tb6612_axis_pwm.v').read_text(encoding='utf-8')
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    assert not re.search(r'(?<![(/])\*(?![)/])|(?<!/)/(?!/)|%', code), 'No inferred multiply/divide/modulo'
    assert '[63:0]' not in code
    for guard in (
        'mul_index==15', 'wire signed [47:0] mul_product={mul_partial[31:0],mul_gain_bits};',
        'mul_partial<={mul_sum[32],mul_sum[32:1]};',
        'mul_gain_bits<={mul_sum[0],mul_gain_bits[15:1]};', 'else if(!enabled)begin',
        'mul_busy<=0;mul_done<=0', 'mul_start<=0;',
        'reg signed [39:0] speed_filter_q8,position_derivative_q8;',
        'reg signed [40:0] derivative_filter_q8;',
        'reg signed [18:0] speed_integral_q8;',
        'pid_outer ? position_kp_sample : speed_kp_sample',
        'pid_outer ? position_ki_sample : speed_ki_sample',
        'pid_outer ? position_kd_sample : speed_kd_sample',
        'if(clear_fault && !motion_requested)fault<=0;',
        'if(encoder_overflow)fault[3]<=1;',
        'if(abs32(speed_raw)>MAX_SPEED_COUNTS)fault[4]<=1;',
        'if(stall_samples>=STALL_SAMPLES-1)fault[0]<=1;',
        'if(reverse_samples>=REVERSE_SAMPLES-1)fault[1]<=1;',
        'wire enabled = run_enable && !calibration_enable && !safety_stop && fault == 0;',
        'reg signed [48:0] pd_sum,pid_value;',
        'wire pid_fits25=pid_value[48:24]=={25{pid_value[24]}};',
        'PID_REJECT_COMMAND:begin',
        'control_state<=PID_CLAMP;',
        'pid_value<={{24{integral_old[24]}},integral_old};',
        'pid_value<={{24{command_clamped_q8[24]}},command_clamped_q8};',
        "position_ki_sample==0 ? 25'sd0 : position_integral_q8",
        "speed_ki_sample==0 ? 25'sd0 : {{6{speed_integral_q8[18]}},speed_integral_q8}",
        'command_clamped_q8<=pid_clamped_q8;control_state<=PID_ROUND;',
    ):
        assert guard in source, guard
    assert code.count('mul_start<=1;') == 3, 'P/I/D share exactly one job sequencer'
    reject_stage = re.search(r'PID_REJECT_COMMAND:begin(.*?)\bend\b', code, re.S).group(1)
    assert 'pid_value<=pid_alu_result' not in reject_stage, 'rejecting I must not recompute output with old I'
    assert 'integral_candidate<=pid_clamped_q8;' in reject_stage, 'rejected old I must obey the new cap'
    assert 'pid_value<={{24{command_clamped_q8[24]}},command_clamped_q8};' in reject_stage
    assert 'control_state<=PID_CLAMP;' in reject_stage
    select_stage = re.search(r'PID_SELECT:begin(.*?)\bend\b', code, re.S).group(1)
    assert 'command_clamped_q8<=pid_clamped_q8;' in select_stage
    assert 'pid_value<={{24{integral_old[24]}},integral_old};' in select_stage
    assert 'PID_COMMAND,PID_REJECT_COMMAND:' not in code, 'the rejection state needs no ALU recomputation'
    engine_abort = re.search(r'else if\(!enabled\)begin(.*?)end else begin', code, re.S).group(1)
    for register in ('mul_busy','mul_done','mul_gain_bits','mul_partial','mul_index'):
        assert f'{register}<=0;' in engine_abort, f'stop must cancel {register}'
    engine_start = re.search(r'if\(mul_start\)begin(.*?)\bend\b', code, re.S).group(1)
    assert 'mul_partial<=0;' in engine_start and 'mul_index<=0;' in engine_start
    for removed in ('mul_addend','mul_accumulator','p_term','i_step','d_term',
                    'integral_sum','raw_command','fallback_command','selected_command',
                    'selected_integral','clamp_pid'):
        assert not re.search(r'\b'+removed+r'\b',code),removed
    assert code.count('pid_clamped_q8;')==4, 'one clamp is reused for candidate/old I and saved/final output'
    assert 'wire [49:0] pid_alu_full={pid_operand_a,1\'b1}+' in source
    # Sampled gains and the current mul_value are written only when starting
    # a new product; the right-shift engine therefore sees a held operand.
    assert code.count('mul_value<=')==4  # reset plus P/I/D job launches
    assert code.count('mul_gain<=')==4
    assert 'mul_value<=error;mul_gain<=pid_outer ? position_kp_sample : speed_kp_sample;' in source
    # A cancelled prefix must make no contribution to a restarted job. Check
    # every possible prefix length numerically, together with the reset/start
    # source guards above; this is not execution of the HDL state machine.
    for prefix_length in range(17):
        # Arithmetic prefixes are discarded, and a fresh job starts with zero
        # upper partial bits and the full new gain, independent of prefix size.
        prefix = I32_MIN * ((65535) & ((1 << prefix_length)-1))
        fits(prefix,48)
        assert product_shift_add(I32_MAX,65535)==I32_MAX*65535
    assert len(re.findall(r'\bbegin\b', code)) == len(re.findall(r'\bend\b', code))
    assert 'if(PERIOD_CYCLES==2500)' in bridge
    assert 'assign on_cycles=five_magnitude[12:1];' in bridge
    assert 'requested_direction == active_direction' in bridge
    assert 'assign pwm = !drive_allowed || pwm_counter < on_cycles;' in bridge
    # Static conservative budget: one complete outer + inner update uses <=150
    # control clocks, plus eight sampling clocks, independent of values/gains.
    assert 158*20 < 10000*1000
    # Default and boundary parameter widths preserve all reachable values;
    # invalid/oversized historical configurations retain their old 16 bits.
    def startup_bits(threshold):
        return max(1,threshold.bit_length()) if 0<=threshold<65535 else 16
    def fault_bits(threshold):
        return max(1,(threshold-1).bit_length()) if 0<threshold<=65535 else 16
    for threshold in (-1,0,1,2,19,20,99,100,65534,65535,65536):
        if 0<=threshold<=65535:
            assert threshold < 1<<startup_bits(threshold)
        if 0<threshold<=65535:
            assert threshold-1 < 1<<fault_bits(threshold)
    assert (startup_bits(100),fault_bits(100),fault_bits(20))==(7,7,5)
    drive=(ROOT/'src/motor_drive_control.v').read_text(encoding='utf-8')
    settings=(ROOT/'src/motor_tuning_control.v').read_text(encoding='utf-8')
    assert 'speed_deadband<=0; position_deadband<=4;' in settings
    assert 'max_duty_permille<=200; min_duty_permille<=0;' in settings
    assert '5:begin low_limit=50; high_limit=900; end' in settings
    assert 'reg [SAMPLE_COUNTER_BITS-1:0] sample_counter;' in drive
    assert 'tick_10ms <= 1;' in drive and 'if (outer_counter == 4)' in drive
    print('PASS: serial-only arithmetic, cancellation, safety/threshold guards; '
          'conservative service budget <3.2 us at 50 MHz, same 10/50 ms periods')
    print('No HDL compilation, elaboration, HDL simulation, synthesis, timing analysis or flashing.')


if __name__ == '__main__':
    check_arithmetic()
    check_pid_tuning_behavior()
    check_source_contracts()
