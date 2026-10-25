#!/usr/bin/env python3
"""Exhaustive sensor formatter integer/ASCII equivalence; no HDL tools.

Compare the previous 32-bit restoring divider / ten-digit BCD domain with
bounded 22-bit divide / six-digit BCD. Raw signed16, unsigned distance16 and
error8 inputs are exhaustive. Sample counters and metadata stay 32 bits.
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def restoring_divide(dividend, width, quotient_width):
    quotient = remainder = 0
    dividend_mask = (1 << width) - 1
    quotient_mask = (1 << quotient_width) - 1
    for _ in range(width):
        trial = (remainder << 1) | (dividend >> (width - 1))
        bit = int(trial >= 1000)
        remainder = trial - (1000 if bit else 0)
        dividend = (dividend << 1) & dividend_mask
        quotient = ((quotient << 1) | bit) & quotient_mask
    return quotient, remainder


# Adjust three BCD nibbles per table read, keeping exhaustive checks quick.
def adjust_three_nibbles(word):
    return sum((((word >> shift) & 15) +
                (3 if ((word >> shift) & 15) >= 5 else 0)) << shift
               for shift in (0, 4, 8))


ADD3 = tuple(adjust_three_nibbles(word) for word in range(4096))


def bcd_six(value, width):
    decimal = 0
    for bit in range(width - 1, -1, -1):
        adjusted = ADD3[decimal & 4095] | (ADD3[decimal >> 12] << 12)
        decimal = ((adjusted << 1) | ((value >> bit) & 1)) & 0xFFFFFF
    return decimal


def decimal_reference(value):
    # The old ten-digit BCD emits only the lower six digits. All bounded
    # sensor fields fit six digits, so zero high nibbles have no influence.
    return int(f'{value:06d}', 16)


def gyro_row(field, negative, decimal):
    digits = f'{decimal:06x}'
    sign = '-' if negative and decimal else '+'
    return f'G{"XYZ"[field]} {sign}{digits[:4]}.{digits[4:]}DPS  '.encode()


def accel_row(field, negative, decimal):
    digits = f'{decimal & 0xFFFF:04x}'
    sign = '-' if negative and decimal else '+'
    return f'A{"XYZ"[field]} {sign}{digits[0]}.{digits[1:]}G      '.encode()


def source_guards():
    source = (ROOT / 'src/sensor_demo_text.v').read_text(encoding='utf-8')
    source = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    required = (
        'reg [17:0] value', 'reg [21:0] dividend', 'reg [11:0] quotient',
        'reg [23:0] decimal', 'reg [4:0] bit_count',
        'reg [31:0] report_count, sequence_count', 'reg [167:0] held_metadata',
        'output reg [167:0] metadata', 'input wire [31:0] imu_samples, us_samples',
        'input wire [31:0] command_count, rx_errors',
        'wire [10:0] trial = {remainder, dividend[21]}',
        "dividend<={6'd0,magnitudes[field]} * 22'd122 + 22'd500",
        "value<={2'd0,magnitudes[field]} * 18'd7",
        "quotient<={quotient[10:0],divide_bit}",
        "value<={6'd0,quotient[10:0],divide_bit}",
        'if (bit_count==21)', 'if (bit_count==17)', 'for (d=0; d<6; d=d+1)',
        'decimal<={adjusted[22:0],value[17]}',
        'metadata<=held_metadata; text_valid<=1; state<=IDLE',
    )
    for token in required:
        assert token in source, token
    assert not re.search(r'\b(?:value|dividend|quotient)\[3[01]\]', source)


def main():
    source_guards()
    gyro_bcd = [bcd_six(magnitude * 7, 18) for magnitude in range(32769)]
    accel_bcd = []
    for magnitude in range(32769):
        dividend = magnitude * 122 + 500
        assert dividend < 1 << 22
        old = restoring_divide(dividend, 32, 32)
        new = restoring_divide(dividend, 22, 12)
        assert old == new == divmod(dividend, 1000)
        assert new[0] < 1 << 12
        accel_bcd.append(bcd_six(new[0], 18))
        assert gyro_bcd[magnitude] == decimal_reference(magnitude * 7)
        assert accel_bcd[magnitude] == decimal_reference(dividend // 1000)
    for raw in range(65536):
        negative = bool(raw & 0x8000)
        magnitude = ((~raw + 1) & 0xFFFF) if negative else raw
        assert magnitude <= 32768
        expected_gyro = decimal_reference(magnitude * 7)
        expected_accel = decimal_reference((magnitude * 122 + 500) // 1000)
        for field in range(3):
            actual_g = gyro_row(field, negative, gyro_bcd[magnitude])
            actual_a = accel_row(field, negative, accel_bcd[magnitude])
            assert actual_g == gyro_row(field, negative, expected_gyro)
            assert actual_a == accel_row(field, negative, expected_accel)
            assert len(actual_g) == len(actual_a) == 16
    for distance in range(65536):
        decimal = bcd_six(distance, 18)
        expected = decimal_reference(distance)
        assert decimal == expected
        # Existing four-character range field intentionally retains its
        # current lower four digits when the uint16 distance exceeds9999.
        for prefix in ('US ', 'LD '):
            row = f'{prefix}{decimal & 0xFFFF:04x} MM      '.encode()
            assert row == f'{prefix}{expected & 0xFFFF:04x} MM      '.encode()
            assert len(row) == 16
    for error in range(256):
        assert bcd_six(error, 18) == decimal_reference(error)
    assert gyro_row(0, True, 0)[3:4] == b'+'
    assert accel_row(0, True, 0)[3:4] == b'+'
    print('PASS: all65536 signed16 raw axis values, all65536 uint16 distances, '
          'all256 errors; unchanged signed ASCII and 16-byte row lengths.')
    print('PASS: all32769 reachable magnitudes: 32-bit vs22-bit serial /1000 '
          'quotient/remainder identical; six-digit BCD matches decimal reference.')
    print('Metadata168 and every32-bit sample/report counter retained; '
          'no HDL compilation, simulation, programming or resource/timing claim.')


if __name__ == '__main__':
    main()
