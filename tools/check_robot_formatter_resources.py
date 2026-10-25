"""Narrow source/resource-shape guards and independent numeric references.

No HDL compile, elaboration, simulation, programmer or estimated LUT totals.
The guard checks that snapshot inputs share ONE magnitude network after the
ten-word selector and text templates remain byte-wide. Python's signed
decimal reference covers full uint32 and int32 boundaries independently of a
two's-complement/BCD implementation.
"""
from pathlib import Path
import random
import re

ROOT = Path(__file__).resolve().parents[1]
MASK32 = (1 << 32) - 1


def source_contract():
    source = (ROOT / 'src/robot_status_text.v').read_text(encoding='utf-8')
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    compact = re.sub(r'\s+', '', code)
    assert 'reg[31:0]values[0:9];' in compact
    assert 'reg[9:0]signs,fractional,angle_decimal;' in compact
    assert 'reg[3:0]field;' in compact, 'ten numeric slots require a four-bit selector'
    assert 'wire[31:0]selected_raw=values[field];' in compact
    assert "wire[31:0]selected_magnitude=signs[field]?~selected_raw+32'd1:selected_raw;" in compact
    assert 'shift<=selected_conversion;bcd<=0;bits<=0;state<=CONVERT;' in compact
    assert "wire[25:0]gain_product=selected_raw[15:0]*26'd625;" in compact
    assert "wire[31:0]selected_conversion=fractional[field]?{10'd0,gain_product[25:4]}:selected_magnitude;" in compact
    assert len(re.findall(r"\*\s*26'd625", code)) == 1, 'Q8 scaling duplicated per gain'
    assert 'wire[87:0]number=angle_decimal[field]?degrees_text(bcd,signs[field]):fractional[field]?fixed4_text(bcd):decimal_text(bcd,signs[field]);' in compact
    assert "fractional<=10'b1110011100;" in compact
    fixed=re.search(r'function\s+\[87:0\]\s+fixed4_text;(.*?)endfunction',code,re.S)[1]
    fixed_compact=re.sub(r'\s+','',fixed)
    for token in ('for(k=0;k<4;k=k+1)fixed4_text[k*8+:8]="0"+d[k*4+:4];',
                  'fixed4_text[39:32]=".";', 'for(k=6;k>=4;k=k-1)begin',
                  'if(d[k*4+:4]!=0||seen||k==4)begin',
                  'fixed4_text[(k+1)*8+:8]="0"+d[k*4+:4];seen=1;'):
        assert token in fixed_compact, token
    dashboard = re.search(r'10:begin(.*?)\n\s*end', code.split('case(view)',1)[1], re.S)[1]
    dashboard_compact = re.sub(r'\s+', '', dashboard)
    snapshots = (
        "values[0]<={{16{motor_target_left[15]}},motor_target_left};",
        "values[1]<=speed_left;", "values[2]<={16'd0,speed_kp_left};",
        "values[3]<={16'd0,speed_ki_left};", "values[4]<={16'd0,speed_kd_left};",
        "values[5]<={{16{motor_target_right[15]}},motor_target_right};",
        "values[6]<=speed_right;", "values[7]<={16'd0,speed_kp_right};",
        "values[8]<={16'd0,speed_ki_right};", "values[9]<={16'd0,speed_kd_right};",
        "signs<={3'd0,speed_right[31],motor_target_right[15],3'd0,speed_left[31],motor_target_left[15]};",
    )
    assert all(snapshot in dashboard_compact for snapshot in snapshots)
    position = re.search(r'11:begin(.*?)\n\s*end', code.split('case(view)',1)[1], re.S)[1]
    position_compact = re.sub(r'\s+', '', position)
    position_snapshots = (
        'values[0]<=motor_position_target_left;', 'values[1]<=position_left;',
        "values[2]<={16'd0,position_kp_left};", "values[3]<={16'd0,position_ki_left};",
        "values[4]<={16'd0,position_kd_left};", 'values[5]<=motor_position_target_right;',
        'values[6]<=position_right;', "values[7]<={16'd0,position_kp_right};",
        "values[8]<={16'd0,position_ki_right};", "values[9]<={16'd0,position_kd_right};",
        "signs<={3'd0,position_right[31],motor_position_target_right[31],3'd0,position_left[31],motor_position_target_left[31]};",
    )
    assert all(snapshot in position_compact for snapshot in position_snapshots)
    assert 'wireheld_pid_view=held_view==10||held_view==11;' in compact
    assert 'if(!held_pid_view&&held_view!=16&&rowfield==14)' in compact
    assert 'if(!held_pid_view&&held_view!=16&&rowfield==15)' in compact
    assert not re.search(r'\b(?:mag|abs32)\s*\(', code), 'per-input negate duplicated'
    assert len(re.findall(r'~\s*selected_raw\s*\+\s*32\s*\'d1', code)) == 1
    assert 'selected_raw[31]?' not in compact, 'uint32 high bit must not become a sign'
    assert 'if(bits==31)' in compact and '[30:0]' not in compact.split('selected_magnitude=', 1)[1].split(';', 1)[0]
    # Every diagnostics counter remains an exact unsigned32 snapshot input.
    diagnostics = re.search(r'16:begin(.*?)end', code, re.S)[1]
    for slot, name in enumerate(('rx_bytes', 'node_count', 'lidar_points',
                                 'uart_errors', 'packet_errors', 'restart_count')):
        assert f'values[{slot}]<={name};' in re.sub(r'\s+', '', diagnostics)
    assert 'values[6]<=selected_value;values[7]<=selected_step;' in compact
    assert 'reg[7:0]row_ids[0:11];' in compact
    assert 'reg[7:0]prefix_id,body_id,formatted_id,emit_id;' in compact
    assert 'reg[87:0]emit_number;' in compact
    # The sole128-bit input is captured raw C1 bytes, not a text-word mux.
    text_code=code.replace('input wire [127:0] lidar_reply_capture,','')
    assert not re.search(r'\b(?:reg|wire|function)\s*\[127:0\]', text_code), 'wide static text selection reintroduced'
    assert code.count('hx(debug_word[')==8, 'C1 words must share one nibble-to-ASCII path'
    assert 'parameterintegerPARALLEL_COMPAT=1' in compact
    assert 'generateif(PARALLEL_COMPAT!=0)begin:parallel_compatibility' in compact
    assert "assigntext_rows=2048'd0;" in compact
    assert 'syn_romstyle="block_rom"' in compact and 'memory[0:4095]' in compact
    assert 'always@(posedgeclk)if(read_enable)data<=memory[address];' in compact
    assert 'for(init_index=0;init_index<1024;init_index=init_index+1)' in compact
    emit = code.split('EMIT:if(text_byte_ready)begin', 1)[1].split('COMMIT:begin', 1)[0]
    assert not re.search(r'\b(?:values|signs|bcd|shift)\s*(?:\[[^]]+\])?\s*<=', emit)
    assert 'flags<=held_flags;text_valid<=1;state<=IDLE;' in compact
    return source


def double_dabble(raw, negative):
    """Model one physical32-bit shared negate + serial decimal converter."""
    shift = ((~raw + 1) & MASK32) if negative else raw
    digits = [0] * 10
    for _ in range(32):
        digits = [digit + 3 if digit >= 5 else digit for digit in digits]
        packed = sum(digit << (4 * position) for position, digit in enumerate(digits))
        packed = ((packed << 1) | (shift >> 31)) & ((1 << 40) - 1)
        digits = [(packed >> (4 * position)) & 15 for position in range(10)]
        shift = (shift << 1) & MASK32
    assert all(digit < 10 for digit in digits)
    return digits


def serial_characters(digits, negative):
    """One character per step, including a leading sign slot and zero case."""
    remaining = list(reversed(digits))
    seen = False
    output = []
    for index in range(10, -1, -1):
        current = remaining[0]
        if index == 10:
            char = '-' if negative and current != 0 else ' '
        else:
            following = remaining[1] if len(remaining) > 1 else 0
            if seen or current != 0 or index == 0:
                char = chr(ord('0') + current)
                seen = True
            elif negative and (following != 0 or index == 1):
                char = '-'
            else:
                char = ' '
            remaining.pop(0)
        output.append(char)
    return ''.join(output)


def main():
    if (ROOT/'src/robot_status_text.v').read_text(encoding='utf-8').startswith('// Compact atomic text:'):
        import unittest
        from check_compact_robot_ui import Tests
        result=unittest.TextTestRunner().run(unittest.TestSuite(Tests(name) for name in (
            'test_serial_decimal_six_digits_and_degrees',
            'test_one_small_formatter_and_flag_contract',
            'test_rom_labels_and_full_byte_protocol')))
        if not result.wasSuccessful():raise SystemExit(1)
        print('PASS compact formatter: signed20-bit metric domain, one serial BCD path, 2KB ROM, atomic byte protocol.')
        print('Full uint32 diagnostic/PID dashboards were removed; old resource guards below describe the historical formatter.')
        return
    source_contract()
    rng = random.Random(202610241350)
    unsigned_edges = {0, 1, 9, 10, 99, 100, (1 << 31) - 1, 1 << 31, MASK32}
    for exponent in range(10):
        for offset in (-1, 0, 1):
            value = 10 ** exponent + offset
            if 0 <= value <= MASK32:
                unsigned_edges.add(value)
    cases = [(raw, False) for raw in unsigned_edges]
    signed_edges = {-(1 << 31), -1, 0, 1, (1 << 31) - 1}
    for exponent in range(10):
        for offset in (-1, 0, 1):
            for sign in (-1, 1):
                value = sign * (10 ** exponent + offset)
                if -(1 << 31) <= value < (1 << 31):
                    signed_edges.add(value)
    cases += [(value & MASK32, value < 0) for value in signed_edges]
    for _ in range(10000):
        raw = rng.getrandbits(32)
        cases += [(raw, False), (raw, bool(raw >> 31))]
    for raw, negative in cases:
        expected_value = raw - (1 << 32) if negative else raw
        actual = serial_characters(double_dabble(raw, negative), negative)
        assert actual == f'{expected_value:11d}', (raw, negative, actual, expected_value)
        assert len(actual) == 11
    # Keep the sign metadata and raw word together through arbitrary snapshot
    # stalls; later live changes must not alter any queued field's number.
    for _ in range(1000):
        live = [rng.getrandbits(32) for _ in range(10)]
        signs = [bool(raw >> 31) if rng.randrange(2) else False for raw in live]
        held = tuple(zip(live, signs))
        expected = tuple(raw - (1 << 32) if sign else raw for raw, sign in held)
        for _ in range(rng.randrange(20)):
            live = [rng.getrandbits(32) for _ in range(10)]
        actual = tuple(int(serial_characters(double_dabble(raw, sign), sign)) for raw, sign in held)
        assert actual == expected
    # Every unsigned16 Q8 gain scales exactly through ONE selected625/16 path.
    for raw in range(65536):
        scaled=(raw*625)>>4
        assert scaled == raw*10000//256
        assert raw*625 < 1<<26 and scaled < 1<<32
    for raw in (0,1,2,127,128,255,256,257,4096,65535):
        scaled=(raw*625)>>4
        digits=double_dabble(scaled,False)
        # The RTL fixed-point view inserts a decimal after four low BCD digits;
        # it must retain zero-valued integer and fraction digits.
        tail=(''.join(str(d) for d in reversed(digits[4:])).lstrip('0') or '0')
        tail+='.'+''.join(str(d) for d in reversed(digits[:4]))
        expected=f'{scaled//10000}.{scaled%10000:04d}'
        assert tail==expected and len(tail.rjust(11))==11
    # The dashboard repurposes rows11..15. Every field must select exactly
    # one frozen slot; especially the gap between LKD and RTGT cannot advance
    # the selector six times or apply left gain metadata to the right target.
    scheduled={2:0,3:1,4:2,5:3,6:4,11:5,12:6,13:7,14:8,15:9}
    field=0
    observed={}
    gain_bits=0b1110011100
    for row in range(16):
        converts=2<=row<=6 or row>=11
        if converts:
            observed[row]=field
            assert bool((gain_bits>>field)&1)==(row in (4,5,6,13,14,15))
        if row<15 and (2<=row<=6 or 11<=row<=14):
            field+=1
    assert observed==scheduled and field==9
    print(f'PASS: byte-wide templateROM, optional wide compatibility, one shared32-bit magnitude path; '
          f'{len(cases)} full-width signed/unsigned decimal references; '
          '10000 held snapshot slots unaffected by later live changes; '
          '65536 exact Q8 scales through one selected path.')
    print('No HDL tools or resource/timing prediction.')


if __name__ == '__main__':
    main()
