#!/usr/bin/env python3
"""Independent BQ display mapping/packing arithmetic; no HDL or chip writes."""
from fractions import Fraction
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
FULL_POINTS = {'A': 83, 'B': 88}


def rounded_reference(raw, full):
    """Exact rational round-half-up, independent of the shift/subtract method."""
    value = Fraction(raw * 100, full) + Fraction(1, 2)
    return min(100, value.numerator // value.denominator)


def scaled_by_subtraction(raw, full):
    if raw >= full:
        return 100, 0
    remainder = (raw << 6) + (raw << 5) + (raw << 2) + full // 2
    assert 0 <= remainder <= 8744 < (1 << 14)
    result, clocks = 0, 1  # Include the final below-divisor decision.
    while remainder >= full:
        remainder -= full
        result += 1
        clocks += 1
    assert result < 100 and clocks <= 100
    return result, clocks


def packed_bcd(value):
    return int(f'{value:05d}', 16)


def header(tag, fresh, error, displayed, millivolts):
    if fresh:
        prefix = 'LOW' if displayed <= 20 else 'MAP'
        centivolts = millivolts // 10
        return f'{tag} {prefix}{displayed:03d}% {centivolts//100:02d}.{centivolts%100:02d}V'
    if error in (0, 0x80):
        return f'{tag} BQ WAIT       '
    return f'{tag} BQ ERR 0X{error:02X}   '


def check_display_mapping():
    source = (ROOT / 'src/bq34z100_status_text.v').read_text(encoding='utf-8')
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    # Existing twelve captured fields and one converter remain unchanged.
    assert 'reg [15:0] values[0:11]' in code
    assert 'reg [19:0] digits[0:11]' in code
    assert code.count('reg [15:0] binary_shift;') == 1
    assert code.count('reg [19:0] decimal;') == 1
    assert 'reg [13:0] soc_remainder;' in code and 'SCALE=5' in code
    assert 'values[0]<={8\'d0,soc_a};values[1]<={8\'d0,soc_b};' in code
    assert "wire [6:0] soc_full=field[0] ? 7'd88 : 7'd83;" in code
    assert "wire [6:0] soc_half=field[0] ? 7'd44 : 7'd41;" in code
    assert 'if (soc_raw>=soc_full)' in code
    assert "binary_shift<=16'd100;state<=CONVERT;" in code
    assert 'soc_remainder<=soc_remainder-soc_full;' in code
    assert "binary_shift<=binary_shift+16'd1;" in code
    assert "soc_bcd<=20'h00020 ? \" LOW\" : \" MAP\"" in code
    assert 'if (valid) row=' in code
    assert 'held_fresh<={valid_a,valid_b}' in code
    assert 'valid_flags<=held_fresh & held_configured;text_valid<=1;state<=IDLE;' in code
    assert 'if (field==11) state<=PACK;' in code
    assert 'fresh ? hex_digit(raw_soc[7:4]) : 8\'h2d' in code
    assert 'fresh ? hex_digit(raw_soc[3:0]) : 8\'h2d' in code
    operators = re.sub(r'"[^"\n]*"', '""', code)
    operators = operators.replace('CLK_HZ/10', 'CONSTANT_REFRESH_CYCLES')
    assert not re.search(r'(?<!/)/(?!/)|%', operators), 'no inferred display divider/modulo'
    # The pre-existing add-three loop uses n*4 as a constant generate index;
    # the new SCALE/LOAD arithmetic itself has no multiplier of any width.
    load_scale = code.split('LOAD:begin', 1)[1].split('CONVERT:begin', 1)[0]
    assert '*' not in load_scale
    assert len(re.findall(r'\bbegin\b', code)) == len(re.findall(r'\bend\b', code))

    cases, maximum_scale_clocks = 0, 0
    for tag, full in FULL_POINTS.items():
        previous = -1
        for raw in range(256):
            displayed, clocks = scaled_by_subtraction(raw, full)
            assert displayed == rounded_reference(raw, full)
            assert previous <= displayed <= 100
            previous = displayed
            maximum_scale_clocks = max(maximum_scale_clocks, clocks)
            # The RTL compares the *displayed BCD*, so the warning agrees
            # exactly with the number on screen, including rounded boundaries.
            assert (packed_bcd(displayed) <= 0x20) == (displayed <= 20)
            for fresh in (False, True):
                for error in (0, 1, 0x80, 0xEA, 0xFF):
                    for millivolts in (0, 9000, 11109, 12600, 65535):
                        text = header(tag, fresh, error, displayed, millivolts)
                        assert len(text) == 16
                        assert (' LOW' in text) == (fresh and displayed <= 20)
            # The raw byte is not scaled or overwritten in diagnostics.
            raw_text = f'{tag} VD=15545 R0X{raw:02X}'
            assert len(raw_text) == 16 and int(raw_text[-2:], 16) == raw
            assert len(f'{tag} VD=----- R0X--') == 16
            cases += 1
        assert rounded_reference(0, full) == 0
        assert rounded_reference(full - 1, full) == 99
        assert rounded_reference(full, full) == 100
        assert rounded_reference(255, full) == 100
    assert rounded_reference(17, 83) == 20 and rounded_reference(18, 83) == 22
    assert rounded_reference(18, 88) == 20 and rounded_reference(19, 88) == 22
    for raw, expected in ((11, 13), (33, 38), (55, 63), (77, 88)):
        assert rounded_reference(raw, 88) == expected  # Exact half ties.
    assert maximum_scale_clocks == 100
    # Twelve shared conversions plus two maximum scale jobs retain the
    # fifteen-row atomic packet and finish far before its100 ms refresh.
    total_clocks = 12 * (1 + 16 + 1) + 2 + 2 * maximum_scale_clocks
    assert total_clocks == 418 and total_clocks / 50_000_000 < 8.5e-6
    assert (2 + 2 + 1 + 2 * 5) * 16 * 8 == 1920
    return cases


if __name__ == '__main__':
    cases = check_display_mapping()
    print(f'PASS: {cases} raw/full-point display mappings; rounded20% LOW boundaries, stale suppression, raw hexadecimal and15x16 packing; one shared converter, scale<=100clks/field')
    print('Independent arithmetic/source guards only; no HDL compile/simulation, chip write or true SOC accuracy validation')
