#!/usr/bin/env python3
"""Independent UI/sensor arithmetic and source checks; never HDL simulation."""

from decimal import Decimal, ROUND_HALF_UP
from fractions import Fraction
from pathlib import Path
import random
import re

from check_motor_integration import uncomment


ROOT = Path(__file__).resolve().parents[1]
UINT32 = (1 << 32) - 1


def double_dabble(value, input_bits, decimal_digits):
    """Evaluate shift/add-3 conversion as integer arithmetic, not RTL timing."""
    assert 0 <= value < (1 << input_bits)
    packed = 0
    packed_mask = (1 << (4 * decimal_digits)) - 1
    for bit in range(input_bits - 1, -1, -1):
        adjusted = packed
        for digit in range(decimal_digits):
            shift = 4 * digit
            nibble = (packed >> shift) & 15
            if nibble >= 5:
                adjusted += 3 << shift
        packed = ((adjusted << 1) | ((value >> bit) & 1)) & packed_mask
    digits = tuple((packed >> (4 * i)) & 15 for i in reversed(range(decimal_digits)))
    assert all(digit <= 9 for digit in digits)
    return ''.join(str(digit) for digit in digits)


def restoring_divmod(numerator, denominator):
    """Independent finite-width arithmetic compared with Python's divmod."""
    assert 0 <= numerator <= UINT32 and 0 <= denominator <= UINT32
    if denominator == 0:
        return UINT32, numerator
    quotient = remainder = 0
    for bit in range(31, -1, -1):
        remainder = 2 * remainder + ((numerator >> bit) & 1)
        take = remainder >= denominator
        trial = remainder - denominator
        assert 0 <= remainder < (1 << 33)
        assert -(1 << 32) <= trial < (1 << 32)
        assert take == (((trial & ((1 << 33) - 1)) >> 32) == 0)
        if take:
            remainder -= denominator
        quotient = ((quotient << 1) | take) & UINT32
        assert 0 <= remainder < denominator and remainder <= UINT32
    return quotient, remainder


def trunc_div(numerator, denominator):
    """Verilog signed division truncates toward zero, unlike Python //."""
    assert denominator > 0
    return numerator // denominator if numerator >= 0 else -((-numerator) // denominator)


def runtime_operators(source):
    # Display text may contain '%' or '/', and CLK_HZ is an elaboration-time
    # parameter. Neither should be mistaken for a run-time arithmetic divider.
    source = re.sub(r'"(?:\\.|[^"\\])*"', '""', uncomment(source))
    source = re.sub(r'\bCLK_HZ\s*/\s*\d+', '1', source)
    return re.findall(r'[/%]', source)


# Exhaustive temperature equivalence: exact Decimal rounding is independent of
# both the old integer expression and the new subtraction-only algebra.
temperature_count = 0
for raw in range(1 << 16):
    exact = Decimal(raw) / 10 - Decimal('273.15')
    expected_tenths = int(exact.quantize(Decimal('0.1'), rounding=ROUND_HALF_UP) * 10)
    centi = raw * 10 - 27315
    original_tenths = trunc_div(centi + 5, 10) if centi >= 0 else -trunc_div(-centi + 5, 10)
    negative = raw < 2732
    magnitude = 2732 - raw if negative else raw - 2731
    reduced_tenths = -magnitude if negative else magnitude
    assert reduced_tenths == original_tenths == expected_tenths
    assert 0 <= magnitude <= 62804 < (1 << 16)
    assert (magnitude <= 999) == (abs(original_tenths) <= 999)
    assert double_dabble(raw, 16, 5) == f'{raw:05d}'
    temperature_count += 1

# All legal pulse inputs and every implemented +/- degree command must retain
# rounding and electrical clamping, including steps beyond either endpoint.
angle_count = 0
for pulse in range(500, 2501):
    old_angle = ((pulse - 500) * 360 + 1000) // 2000
    new_angle = ((pulse - 500) * 9 + 25) // 50
    assert old_angle == new_angle
    assert (pulse - 500) * 9 + 25 < (1 << 15)
    for step in (-10, -1, 1, 10):
        old_pulse = 500 + trunc_div((old_angle + step) * 2000 + 180, 360)
        old_pulse = min(2500, max(500, old_pulse))
        adjusted_angle = min(360, max(0, new_angle + step))
        new_pulse = 500 + (adjusted_angle * 50 + 4) // 9
        assert new_pulse == old_pulse
        assert adjusted_angle * 50 + 4 < (1 << 15)
        assert 500 <= new_pulse <= 2500 < (1 << 12)
        angle_count += 1

rng = random.Random(0xAC6720)
edges = (0, 1, 2, 3, 7, 9, 10, 50, 360, 10000, 0xffff,
         0x7fffffff, 0x80000000, 0xfffffffe, UINT32)
division_cases = {(n, d) for n in edges for d in edges}
division_cases.update((rng.getrandbits(32), rng.getrandbits(32)) for _ in range(8192))
division_cases.update((rng.getrandbits(32), d) for d in (1, 9, 10, 50, 100, 1000, 10000) for _ in range(64))
for numerator, denominator in division_cases:
    quotient, remainder = restoring_divmod(numerator, denominator)
    assert 0 <= quotient <= UINT32 and 0 <= remainder <= UINT32
    if denominator:
        assert (quotient, remainder) == divmod(numerator, denominator)
        assert numerator == quotient * denominator + remainder and remainder < denominator
    else:
        assert quotient == UINT32 and remainder == numerator

# Every accepted ultrasonic pulse: compare the bounded shift/add numerator
# and serial division with independent decimal 0.17 mm/us nearest-mm rounding.
ultrasonic_count = 0
for width_us in range(118, 26471):
    numerator = (width_us << 4) + width_us + 50
    assert width_us < (1 << 15) and numerator < (1 << 19)
    quotient, remainder = restoring_divmod(numerator, 100)
    expected_mm = int((Decimal(width_us) * Decimal('0.17')).quantize(
        Decimal('1'), rounding=ROUND_HALF_UP))
    assert quotient == expected_mm == (width_us * 17 + 50) // 100
    assert 20 <= quotient <= 4500 < (1 << 16)
    ultrasonic_count += 1

# Four displayed decimal digits deliberately retain the low four decimal
# digits of the full counter, including values above the 16-bit range.
counter_cases = set(edges)
counter_cases.update(rng.getrandbits(32) for _ in range(2048))
for counter in counter_cases:
    assert double_dabble(counter, 32, 4) == f'{counter % 10000:04d}'

divider = (ROOT / 'src/unsigned_divmod.v').read_text(encoding='utf-8')
actuator = (ROOT / 'src/actuator_test_control.v').read_text(encoding='utf-8')
bq_text = (ROOT / 'src/bq34z100_status_text.v').read_text(encoding='utf-8')
ultrasonic = (ROOT / 'src/cs100a_demo.v').read_text(encoding='utf-8')
lcd = (ROOT / 'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8')
setup = (ROOT / 'src/bq34z100_board_setup.v').read_text(encoding='utf-8')
top = (ROOT / 'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
tx_mux = (ROOT / 'src/ui_video_tx_mux.v').read_text(encoding='utf-8')
sdc = (ROOT / 'src/lcd_interaction.sdc').read_text(encoding='utf-8')
divider_code = uncomment(divider)
actuator_code = uncomment(actuator)
bq_code = uncomment(bq_text)
ultrasonic_code = uncomment(ultrasonic)
assert not runtime_operators(divider)
assert not runtime_operators(actuator)
assert not runtime_operators(bq_text)
assert not runtime_operators(ultrasonic)
assert 'unsigned_divmod u_distance_math' in ultrasonic_code
assert '.numerator({13\'d0,distance_numerator})' in ultrasonic_code
assert '.denominator(32\'d100)' in ultrasonic_code
assert 'state == CONVERT && distance_done' in ultrasonic_code
assert 'distance_mm <= distance_quotient[15:0]' in ultrasonic_code
assert 'distance_start <= 0' in ultrasonic_code and 'distance_start <= 1' in ultrasonic_code
assert "echo_width_us >= 32'd118 && echo_width_us <= 32'd26470" in ultrasonic_code
assert "echo_width_us >= 32'd60000" in ultrasonic_code
assert "active_echo_us >= 32'd70000" in ultrasonic_code
assert "trigger_us == 6'd49" in ultrasonic_code
assert "interval_us == 17'd100000" in ultrasonic_code
for name in ('numerator', 'denominator', 'quotient', 'remainder'):
    assert re.search(rf'\[31:0\]\s+[^;\n]*\b{name}\b', divider_code)
assert 'reg [32:0] remainder_work' in divider_code
assert 'bit_count==31' in divider_code
assert "quotient<=32'hffffffff;remainder<=numerator" in divider_code
assert actuator_code.count('unsigned_divmod ') == 2
assert 'report_values[0:13]' in actuator_code
assert 'report_bit==31' in actuator_code and 'report_index==13' in actuator_code
assert "servo_div_denominator<=32'd50" in actuator_code
assert "servo_div_denominator<=32'd9" in actuator_code
assert 'reg [15:0] values[0:11]' in bq_code
assert 'reg [19:0] digits[0:11]' in bq_code
assert 'bit_count==15' in bq_code and 'field==11' in bq_code
assert "raw>=16'd2732 ? raw-16'd2731 : 16'd2732-raw" in bq_code
assert 'valid_flags<=held_fresh & held_configured' in bq_code

# Structural pixel-bundle alignment: layout selection now occupies a separate
# register boundary. Trace the explicit source-to-L-to-A-to-B edges; this does
# not execute clocks, video/FIFO models or any HDL.
lcd_code = re.sub(r'\s+', '', uncomment(lcd))
pixel_edges = {
    'label_text_l': 'label_text', 'label_dx_l': 'dx', 'label_dy_l': 'dy[3:0]',
    'robot_control_l': 'robot_control', 'robot_label_source_l': 'robot_label_source',
    'robot_label_row_l': 'robot_label_row', 'robot_label_literal_l': 'robot_label_literal',
    'in_label_l': 'active&&in_label', 'label_color_l': 'label_color',
    'base_color_l': 'base_color', 'de_l': 'active',
    'hs_l': '(h>=H_SYNC)', 'vs_l': '(v>=V_SYNC)',
    'in_label_a': 'in_label_l', 'label_color_a': 'label_color_l',
    'base_color_a': 'base_color_l', 'de_a': 'de_l', 'hs_a': 'hs_l', 'vs_a': 'vs_l',
    'in_label_b': 'in_label_a', 'label_color_b': 'label_color_a',
    'base_color_b': 'base_color_a', 'de_b': 'de_a', 'hs_b': 'hs_a', 'vs_b': 'vs_a',
    'de': 'de_b', 'hs': 'hs_b', 'vs': 'vs_b',
    'character_a': 'robot_control_l?robot_dynamic_character:character',
    'robot_constant_a': "robot_control_l&&robot_label_source_l==3'd0",
    'robot_byte_source_a': "BYTE_TEXT_MODE&&robot_control_l?(robot_label_source_l==3'd1?2'd1:robot_label_source_l==3'd2?2'd2:2'd0):2'd0",
    'font_b': 'glyph(glyph_character_a)',
    'glyph_x_a': 'glyph_x_now', 'glyph_y_a': 'glyph_y_now',
    'glyph_x_b': 'glyph_x_a', 'glyph_y_b': 'glyph_y_a',
}
for destination, source in pixel_edges.items():
    assert f'{destination}<={source};' in lcd_code
assert "char_ix=label_dx_l/11'd12;" in lcd_code
assert "glyph_x_now=(label_dx_l%11'd12)>>1;" in lcd_code
assert 'glyph_y_now=label_dy_l[3:1];' in lcd_code
assert 'character=label_text_l[127-(char_ix*8)-:8];' in lcd_code
assert 'glyph_character_a=robot_constant_a?robot_constant_character_a:glyph_dynamic_character_a;' in lcd_code
assert "glyph_dynamic_character_a=robot_byte_source_a==2'd1?robot_byte_character:robot_byte_source_a==2'd2?pid_byte_character:character_a;" in lcd_code
assert 'robot_byte_source_a<=0;' in lcd_code
assert 'assignbyte_text_read_index={robot_label_row_l,char_ix};' in lcd_code
assert "assignrobot_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3'd1;" in lcd_code
assert "assignpid_byte_read_en=BYTE_TEXT_MODE&&run&&in_label_l&&robot_control_l&&robot_label_source_l==3'd2;" in lcd_code
assert '.label_id(robot_label_literal_l),.character_index(char_ix)' in lcd_code
assert 'rgb<=text_pixel?label_color_b:base_color_b;' in lcd_code
assert 'base_color_a<=base_color;' not in lcd_code
assert 'in_label_a<=active&&in_label;' not in lcd_code
assert "H_TOTAL=1344" in lcd_code and "V_TOTAL=635" in lcd_code
assert "frame_restart=run&&h==H_TOTAL-1'b1&&v==V_TOTAL-1'b1;" in lcd_code

# BQ waits retain all minimum protocol durations. Counting N down to zero,
# then returning, adds exactly one conservative clock instead of using a
# 32-bit variable compare in the counter-enable feedback path.
setup_code = re.sub(r'\s+', '', uncomment(setup))
assert 'delay_count<=MS_CYCLES*wait_ms;' in setup_code
assert 'reg[15:0]response;' in setup_code
assert 'response<=master_read_data[15:0];delay_count<=bus_delay_cycles;' in setup_code
assert "if(request_read&&request_register==8'h40&&request_read_count==32)block_data<=master_read_data;" in setup_code
assert 'BUS_DELAY:if(delay_count==0)state<=bus_return;' in setup_code
assert "elsedelay_count<=delay_count-1'b1;" in setup_code
assert 'delay_count>=' not in setup_code
for frequency in (1_000_000, 12_000_000, 50_000_000, 100_000_000):
    for milliseconds in (40, 100, 110, 300, 1000, 2000):
        cycles = ((frequency + 999) // 1000) * milliseconds
        assert 0 < cycles < (1 << 32)
        assert Fraction(cycles + 1, frequency) >= Fraction(milliseconds, 1000)
        assert Fraction(cycles + 1, frequency) - Fraction(cycles, frequency) == Fraction(1, frequency)

# Scope the hold exception to one reviewed held-data mailbox. This is not a
# blanket asynchronous-clock cut: the same bundle keeps a setup/max-delay
# bound below its three-cycle request latency, and all other timing remains.
scope = '-from [get_regs {ui_mailbox_data*}] -to [get_regs {u_ui_tx_mux/latest_snapshot*}]'
sdc_code = '\n'.join(line.strip() for line in sdc.splitlines()
                     if line.strip() and not line.lstrip().startswith('#'))
assert f'set_max_delay {scope} 16.000' in sdc_code
assert f'set_false_path {scope} -hold' in sdc_code
assert sdc_code.count('set_false_path') == 1
assert 'set_clock_groups' not in sdc_code
assert 'create_clock -name clk50M -period 20.000' in sdc_code
top_code = re.sub(r'\s+', '', uncomment(top))
mux_code = re.sub(r'\s+', '', uncomment(tx_mux))
assert 'ui_ack_sync<={ui_ack_sync[1:0],ui_mailbox_ack};' in top_code
assert 'if(ui_mailbox_request==ui_ack_sync[2])begin' in top_code
assert 'request_sync<={request_sync[1:0],mailbox_request};' in mux_code
assert 'wiremailbox_new=request_sync[2]!=mailbox_ack;' in mux_code
assert 'latest_snapshot<=mailbox_data;' in mux_code
assert 'mailbox_ack<=request_sync[2];' in mux_code
assert Fraction(16, 1_000_000_000) < Fraction(3, 125_000_000)

# Work budgets are source-design bounds with generous FSM overhead, not
# measured execution times or proof of HDL handshaking. 1000 clocks allows
# the 14x32 BCD shifts, two 32-bit divisions, and >400 overhead clocks.
clock_hz = 50_000_000
actuator_shift_clocks = 14 * 32
actuator_division_clocks = 2 * 32
actuator_budget_clocks = 1000
assert actuator_shift_clocks + actuator_division_clocks < actuator_budget_clocks
assert actuator_budget_clocks < clock_hz // 20
assert Fraction(actuator_budget_clocks, clock_hz) == Fraction(20, 1_000_000)
bq_budget_clocks = 1 + 12 * (1 + 16 + 1) + 1
assert bq_budget_clocks == 218 < clock_hz // 10
assert Fraction(bq_budget_clocks, clock_hz) == Fraction(436, 100_000_000)

print(f'PASS: {temperature_count} exact temperature/BCD16 cases; {angle_count} pulse/angle cases; '
      f'{len(division_cases)} seeded/edge divmod cases; {len(counter_cases)} BCD32 counter cases')
print(f'PASS: {ultrasonic_count} accepted ultrasonic pulse conversions retain exact nearest-mm rounding')
print(f'PASS: {len(pixel_edges)} LCD pixel-bundle register edges; layout/character/glyph/RGB alignment guards')
print('PASS: BQ countdown minimum-wait arithmetic; scoped held-mailbox hold/max-delay source guards')
print('PASS: no run-time / or % in affected UI/sensor sources; 32-bit divider widths; '
      'conservative actuator budget 20 us and BQ formatter arithmetic budget 218 clocks/4.36 us')
print('NO HDL compilation/simulation, synthesis resource measurement, timing closure, compile-time speedup or board validation')
