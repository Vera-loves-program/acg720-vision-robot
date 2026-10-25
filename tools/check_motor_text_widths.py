#!/usr/bin/env python3
"""Independent motor formatter range/decimal checks; never HDL simulation."""
from pathlib import Path
import random
import re

ROOT = Path(__file__).resolve().parents[1]
RNG = random.Random(20261024)


def convert(value, width, digits):
    packed = 0
    for bit in reversed(range(width)):
        adjusted = packed
        for digit in range(digits):
            offset = digit * 4
            if (packed >> offset) & 15 >= 5:
                adjusted += 3 << offset
        packed = ((adjusted << 1) | ((value >> bit) & 1)) & ((1 << (digits * 4)) - 1)
    return ''.join(str((packed >> (4 * i)) & 15) for i in reversed(range(digits)))


maximum_scaled = ((2**32 - 1) * 10000) // 256
assert maximum_scaled < 2**38 < 10**12
assert (2**32 - 1) * 10000 < 2**46
assert 65535 * 100000 + 65535 < 2**38
samples = {0, 1, 255, 256, 32767, 32768, 65535, 99999999, 100000000,
           2147483647, 2147483648, 4294967295, maximum_scaled,
           (2**31 * 10000) // 256, 65535 * 100000 + 65535}
samples.update(RNG.randrange(maximum_scaled + 1) for _ in range(1000))
for value in samples:
    original = convert(value, 64, 20)
    reduced = convert(value, 38, 12)
    assert original[-12:] == reduced == f'{value:012d}', value
    for displayed_digits in (4, 5, 8, 10):
        assert original[-displayed_digits:] == reduced[-displayed_digits:]
    assert (value > 99999999) == (value >= 100000000)
for value in range(65536):
    original = (value * 10000) >> 8
    product46 = (value * 10000) & ((1 << 46) - 1)
    assert (product46 >> 8) == original
for value in (0, 1, 2**31 - 1, 2**31, 2**32 - 1):
    assert ((value * 10000) & ((1 << 46) - 1)) >> 8 == value * 10000 // 256

source = (ROOT / 'src/motor_status_text.v').read_text(encoding='utf-8')
for guard in ('reg [37:0] values [0:15];', 'reg [47:0] decimal;',
              'reg [37:0] shift_value;', 'bit_count==37',
              'for (n=0;n<12;n=n+1)', 'selected_scaled[45:8]',
              'step_scaled[45:8]', "values[field]>38'd99999999"):
    assert guard in source, guard
assert "64'd" not in source and 'reg [79:0]' not in source
# A streamed row must not turn the LCD's constant-labelled rows into a
# dynamically addressed128-bit register bank. Preserve fixed per-row stores.
bare = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
row_writes = re.findall(r'\brows\[([^]]+)\]\s*<=', bare)
assert all(index.isdigit() or index == 'i' for index in row_writes), row_writes
assert not re.search(r'\brows\[[^]]+\]\s*<=\s*formatted_row', bare)
assert set(index for index in row_writes if index.isdigit()) == {str(i) for i in range(16)}
print(f'PASS: {len(samples)} full-field 64/20-digit vs 38/12-digit BCD conversions; '
      '65536 Q8 values and full uint32 boundary products')
print('NO HDL compilation/simulation, synthesis measurement or timing closure')
