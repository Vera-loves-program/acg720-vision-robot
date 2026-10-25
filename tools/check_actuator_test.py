#!/usr/bin/env python3
"""Static commissioning audit and independent pulse arithmetic, NOT HDL validation."""
from pathlib import Path
import re
from check_motor_integration import uncomment, strings

ROOT = Path(__file__).resolve().parents[1]
control = uncomment((ROOT / 'src/actuator_test_control.v').read_text(encoding='utf-8'))
panel = uncomment((ROOT / 'src/actuator_test_panel.v').read_text(encoding='utf-8'))
servo = uncomment((ROOT / 'src/servo_pulse_output.v').read_text(encoding='utf-8'))
top = uncomment((ROOT / 'src/vision_robot_ui_top.v').read_text(encoding='utf-8'))

# Check ALL standalone panel display strings, including ternary alternatives.
for literal in strings(panel):
    if literal in (b'IMU ',b'    '):  # 4 + existing 8-byte status + 4
        continue
    if literal==b'COARSE':  # Compare the first six bytes of the mode label.
        continue
    assert len(literal) == 16, (literal, len(literal))
for statement in re.findall(r'status_text\[[^;]+;', control):
    if '{' not in statement:
        for literal in strings(statement):
            assert len(literal) == 16, (literal, len(literal))
prefix = (ROOT / 'src/sensor_demo_uart.v').read_text(encoding='utf-8')
assert 'wheel_speed_left' in prefix and 'wheel_speed_right' in prefix
assert 'baud_9600' not in prefix and 'TEST_BEACON' not in prefix

# Inspect the actual concatenations, including function return widths, so an
# accidental extra/missing ASCII byte fails before opening the FPGA tools.
def fragments(expr):
    start=depth=0;quoted=False
    for index,char in enumerate(expr):
        if char=='"':quoted=not quoted
        if not quoted:
            if char in '([':depth+=1
            elif char in ')]':depth-=1
            elif char==',' and depth==0:
                yield expr[start:index].strip();start=index+1
    yield expr[start:].strip()

checked_rows=0
for line in control.splitlines():
    if 'status_text[' not in line:continue
    for concat in re.findall(r'\{([^{}]+)\}',line):
        width=0
        for part in fragments(concat):
            if part=='report_ascii':width+=4
            elif part=='report_ascii[23:8]':width+=2
            elif re.fullmatch(r"report_negative\[[01]\]\s*\?\s*8'h2d\s*:\s*8'h2b",part):width+=1
            else:
                literals=list(strings(part))
                assert literals and len({len(literal) for literal in literals})==1,part
                width+=len(literals[0])
        assert width==16,(line,width)
        checked_rows+=1
assert checked_rows==4
assert 'parameter integer ACTUATOR_TEST=0' in top
assert 'parameter integer ROBOT_MODE=1' in top
assert '.run(pixel_run_pipe[1])' in top
assert '.reset_n(ready && ACTUATOR_TEST!=0)' in top
assert '.run_enable(!ACTUATOR_TEST && ((motor_armed && motor_running && (!ROBOT_ACTIVE || robot_page==1)) ||' in top
assert '(ROBOT_ACTIVE && robot_drive_enable)))' in top
assert '.snapshot_cancel(touch_snapshot_cancel || pid_page || ACTUATOR_TEST || ROBOT_ACTIVE)' in top
assert 'target_us[i]<=1500' in control and 'servo_enabled<=0' in control
assert 'min_saved<=0;max_saved<=0' in control
assert 'target_us[axis]==applied[axis]' in control
assert 'applied[axis]>=lower_us[axis] && applied[axis]<=upper_us[axis]' in control
assert 'if (safety_stop || emergency_stop_pulse || stop_contact || !touch_ready || bq_setup_busy)' in control
assert 'action==25' in control and '!run_left && !run_right && servo_enabled==0' in control
assert 'touch_ready && !bq_setup_busy' in control
assert 'CLK_HZ/1000000' in servo and 'US_CYCLES*20000' in servo
assert 'if (!enable) frame_enabled<=0' in servo
assert 'output wire pwm' in servo
assert 'applied_us<=applied_us+1' in servo
assert 'reg coarse[0:1]' in control and 'step_select' not in control
assert 'coarse[axis] ? 100 : 10' in control and 'coarse[axis] ? 10 : 1' in control
assert re.search(r'if\s*\(servo_enabled\[axis\]\)\s*begin\s*target_us\[axis\]<=applied\[axis\]',control)
assert 'target_us[0]<=applied0;target_us[1]<=applied1' in control
assert 'if (cancel_pending[i]) target_us[i]<=applied[i]' in control
assert 'cancel_pending[axis]<=1' in control
assert control.count('unsigned_divmod ')==2
assert 'MATH_ANGLE:if (servo_div_done)' in control
assert 'MATH_PULSE:if (servo_div_done)' in control
assert 'servo_math_state<=MATH_IDLE;servo_math_pending<=0' in control
assert '!calibration[servo_math_axis] && !cancel_pending[servo_math_axis]' in control
assert 'REPORT_SHIFT:begin' in control and 'report_bit==31' in control
assert 'report_index==13' in control and 'status_valid<=1' in control
assert 'report_values[12]<=rx_byte_count;report_values[13]<=tx_line_count' in control
# Clock constants may divide at elaboration; runtime operands must use the
# bounded sequential divider/BCD converter, including display-only fields.
arithmetic_only=re.sub(r'"[^"\n]*"','',control)
arithmetic_only=arithmetic_only.replace('CLK_HZ/1000','').replace('CLK_HZ/20','')
assert '/' not in arithmetic_only and '%' not in arithmetic_only
assert 'digits4(' not in control and 'percent(' not in control
assert 'tx_line_count<=tx_line_count+1' in prefix
assert 'display_text_valid' in prefix
assert 'COUNT/10MS' in prefix
assert '.wheel_speed_left(motor_speed_left),.wheel_speed_right(motor_speed_right)' in top
assert 'speed_text[2047-6*128 -:128]' in control and 'speed_text[2047-8*128 -:128]' in control

# Independent exact-rational reference: round a 0..360 electrical command to
# the nearest microsecond, then inspect monotonicity and round-trip error.
def pulse(degrees):
    return 500 + (degrees * 2000 + 180) // 360

def angle(us):
    return ((us - 500) * 360 + 1000) // 2000

assert [pulse(d) for d in (0,90,180,270,360)] == [500,1000,1500,2000,2500]
assert all(pulse(d+1) > pulse(d) for d in range(360))
assert all(angle(pulse(d)) == d for d in range(361))
for us in range(500,2501):
    reduced_angle=((us-500)*9+25)//50
    assert reduced_angle==angle(us)
    for step in (-10,-1,1,10):
        limited_angle=max(0,min(360,reduced_angle+step))
        reduced_pulse=500+(limited_angle*50+4)//9
        # The old signed expression truncates toward zero; electrical bounds
        # make its result equal after clamping even outside 0..360 degrees.
        old_numerator=(angle(us)+step)*2000+180
        signed_quotient=(abs(old_numerator)//360)*(1 if old_numerator>=0 else -1)
        original_pulse=max(500,min(2500,500+signed_quotient))
        assert reduced_pulse==original_pulse
for low, high in ((501,517),(1000,1900),(1487,1511),(500,2500)):
    for us in range(low,high+1):
        for step in (-10,-5,-1,1,5,10):
            commanded = max(low,min(high,pulse(angle(us)+step)))
            assert low <= commanded <= high
            # A 1 us/frame slew beginning inside the same interval cannot
            # leave that interval. This is arithmetic, not a waveform test.
            first = us + (commanded>us) - (commanded<us)
            assert low <= first <= high
assert 50000000//1000000*20000 == 1000000
assert 50000000//20000 == 2500
print('PASS: test-source integration guards, display widths and nominal pulse/limit arithmetic')
print('NO HDL compilation/simulation, timing analysis, measured angles or hardware validation')
