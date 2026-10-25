"""Independent compact motor text arithmetic and byte-event publication checks.

Reads source contracts; runs no HDL compiler/simulator, synthesis or device.
The complete legacy generate branch is protected byte-for-byte, and compact
publication is checked under changing inputs, backpressure and reset cuts.
"""
from pathlib import Path
from dataclasses import dataclass
import hashlib
import random
import re
from check_motor_integration import uncomment, check_blocks

ROOT = Path(__file__).resolve().parents[1]
raw = (ROOT / 'src/motor_status_text.v').read_bytes()
legacy = raw[raw.index(b'    localparam IDLE='):raw.rindex(b'    end endgenerate')]
assert hashlib.sha256(legacy).hexdigest() == '9948daedcdcce7fd9963adb135ffae253ce6c2097266d16fc23c95adeb6232ec'
source = uncomment(raw.decode('utf-8-sig'))
check_blocks(source, 'motor_status_text.v')
code = re.sub(r'\s+', '', source)
compact = code.split('generateif(COMPACT_ROBOT!=0)begin:compact_robot', 1)[1].split('endelsebegin:full_formatter', 1)[0]
assert 'parameterintegerCOMPACT_ROBOT=0' in code
for token in (
    'reg[31:0]compact_duties;', 'reg[15:0]compact_binary;', 'reg[19:0]compact_bcd;',
    "wire[15:0]compact_magnitude=compact_duties[15]?(~compact_duties[15:0]+16'd1):compact_duties[15:0];",
    'localparamintegerCOMPACT_PERIOD=CLK_HZ/10<1?1:CLK_HZ/10;',
    'compact_duties<={duty_right,duty_left};',
    "scope_code<=(selected_field>=4&&selected_field<=7)?3'd4:{1'b0,selected_wheel,edit_position};",
    'compact_bcd<={compact_adjusted[18:0],compact_binary[15]};',
    'if(compact_bit==15)compact_state<=C_STORE;',
    'assigntext_byte_data=compact_emit[127:120];',
    'assigntext_byte_index={compact_row,compact_column};',
    'assigntext_byte_valid=reset_n&&compact_state==C_EMIT;',
    'C_EMIT:if(text_byte_ready)begin',
    "compact_emit<={compact_emit[119:0],8'd0};",
    "if(compact_row==11)compact_duties<={16'd0,compact_duties[31:16]};",
    'if(compact_row==15)compact_state<=C_COMMIT;',
    'C_COMMIT:begintext_valid<=1;compact_state<=C_IDLE;end',
):
    assert token in compact, token
assert compact.count('text_rows<=0;') == 2
assert 'values[' not in compact and 'rows[' not in compact
assert 'selected_scaled' not in compact and 'selected_step' not in compact and 'selected_value' not in compact
assert set(re.findall(r'compact_bcd\[([^\]]+)\]', compact)) == {'15:12', '11:8', '7:4', '3:0'}


def pwm_text(value, side):
    # Preserve the legacy public signed16 formatting, including the low-four
    # digits for values beyond physical PWM range: -32768 => -2768/1000.
    assert -32768 <= value <= 32767
    return f'{side} PWM {"-" if value < 0 else "+"}{abs(value) % 10000:04d}/1000'.encode()


def bcd16(unsigned):
    digits = 0
    for bit in range(15, -1, -1):
        adjusted = digits
        for slot in range(5):
            if (digits >> (slot * 4)) & 15 >= 5:
                adjusted += 3 << (slot * 4)
        digits = ((adjusted << 1) | ((unsigned >> bit) & 1)) & 0xfffff
    return digits


scope_labels = {int(i): label.encode() for i, label in re.findall(
    r'(\d+)\s*:\s*compact_scope\s*=\s*"([^"]+)"\s*;', source)}
scope_labels[4] = b'EDIT BOTH SHARED'
assert set(scope_labels) == set(range(5)) and all(len(v) == 16 for v in scope_labels.values())


def scope_index(wheel, position, field):
    return 4 if 4 <= field <= 7 else wheel * 2 + position


def expected_frame(values):
    left, right, wheel, position, field = values
    rows = [b' ' * 16 for _ in range(16)]
    rows[1] = scope_labels[scope_index(wheel, position, field)]
    rows[11], rows[12] = pwm_text(left, 'L'), pwm_text(right, 'R')
    return b''.join(rows)


@dataclass
class Stream:
    state: str = 'IDLE'
    age: int = 0
    row: int = 0
    column: int = 0
    words: int = 0
    scope: int = 0
    binary: int = 0
    decimal: int = 0
    bit: int = 0
    negative: bool = False
    emit: bytes = b''
    captured: tuple | None = None
    period: int = 500

    def reset(self):
        self.__dict__.update(Stream(period=self.period).__dict__)

    def tick(self, values, ready):
        valid = self.state == 'EMIT'
        accepted = (self.row * 16 + self.column, self.emit[0]) if valid and ready else None
        held = (self.row * 16 + self.column, self.emit[0]) if valid else None
        committed = self.state == 'COMMIT'
        due = self.age == self.period - 1
        self.age = 0 if due else self.age + 1
        if self.state == 'IDLE' and due:
            self.captured = values
            left, right, wheel, position, field = values
            self.words = ((right & 65535) << 16) | (left & 65535)
            self.scope = scope_index(wheel, position, field)
            self.row, self.state = 0, 'LOAD'
        elif self.state == 'LOAD':
            if self.row in (11, 12):
                raw = self.words & 65535
                self.negative = bool(raw & 32768)
                self.binary = ((~raw + 1) & 65535) if self.negative else raw
                self.decimal = self.bit = 0
                self.state = 'CONVERT'
            else:
                self.state = 'STORE'
        elif self.state == 'CONVERT':
            adjusted = self.decimal
            for digit in range(5):
                if ((self.decimal >> (digit * 4)) & 15) >= 5:
                    adjusted += 3 << (digit * 4)
            self.decimal = ((adjusted << 1) | (self.binary >> 15)) & 0xfffff
            self.binary = (self.binary << 1) & 65535
            self.bit += 1
            if self.bit == 16:
                self.state = 'STORE'
        elif self.state == 'STORE':
            if self.row == 1:
                self.emit = scope_labels[self.scope]
            elif self.row in (11, 12):
                digits = ''.join(str((self.decimal >> bit) & 15) for bit in (12, 8, 4, 0))
                self.emit = f'{"L" if self.row == 11 else "R"} PWM {"-" if self.negative else "+"}{digits}/1000'.encode()
            else:
                self.emit = b' ' * 16
            self.column, self.state = 0, 'EMIT'
        elif self.state == 'EMIT' and ready:
            self.emit = self.emit[1:] + b'\x00'
            if self.column == 15:
                if self.row == 11:
                    self.words >>= 16
                if self.row == 15:
                    self.state = 'COMMIT'
                else:
                    self.row, self.state = self.row + 1, 'LOAD'
            else:
                self.column += 1
        elif self.state == 'COMMIT':
            self.state = 'IDLE'
        return accepted, held, committed


# Compare all65536 signed inputs with their native decimal reference, not the
# implementation's BCD slices alone. INT16_MIN must stay magnitude32768.
for value in range(-32768, 32768):
    unsigned = value & 65535
    magnitude = ((~unsigned + 1) & 65535) if value < 0 else unsigned
    packed = bcd16(magnitude)
    decoded = ''.join(str((packed >> bit) & 15) for bit in (16, 12, 8, 4, 0))
    assert int(decoded) == abs(value)
    assert f'L PWM {"-" if value < 0 else "+"}{decoded[-4:]}/1000'.encode() == pwm_text(value, 'L')

rng = random.Random(20261024)
stream = Stream()
pending = None
observed = []
frames = stalls = 0
for cycle in range(120000):
    values = (rng.randrange(-32768, 32768), rng.randrange(-32768, 32768),
              rng.randrange(2), rng.randrange(2), rng.randrange(10))
    ready = rng.randrange(5) < 2
    if stream.row * 16 + stream.column == 255 and cycle % 23 < 17:
        ready = False
    accepted, held, committed = stream.tick(values, ready)
    if pending is not None:
        assert held == pending, 'unaccepted byte or index changed'
    pending = held if held is not None and not ready else None
    stalls += int(pending is not None)
    if accepted is not None:
        observed.append(accepted)
    if committed:
        assert [i for i, _ in observed] == list(range(256))
        assert bytes(v for _, v in observed) == expected_frame(stream.captured)
        observed = []
        frames += 1
assert frames >= 100 and stalls > 20000

# Reset a partial publication at boundary bytes and PWM rows. The first next
# publication begins at0, never commits an incomplete old frame, and is coherent.
for cut in (0, 1, 15, 16, 175, 176, 191, 192, 207, 255):
    stream = Stream(period=400)
    values = (-32768, 32767, 1, 1, 3)
    for _ in range(2000):
        accepted, _, _ = stream.tick(values, True)
        if accepted is not None and accepted[0] == cut:
            break
    else:
        raise AssertionError(('cut not reached', cut))
    stream.reset()
    observed = []
    for _ in range(2000):
        accepted, _, committed = stream.tick(values, True)
        if accepted is not None:
            observed.append(accepted)
        if committed:
            break
    assert [i for i, _ in observed] == list(range(256))
    assert bytes(v for _, v in observed) == expected_frame(values)

print(f'PASS:full legacy branch byte-for-byte;65536 signed16 values including INT16_MIN;{frames} atomic compact frames/{stalls} stalls;10 reset cuts.')
print('PASS:row1 scope and rows11/12 PWM match legacy strings;256 ordered bytes,commit after acceptance,10Hz timer,compact parallel output constant0.')
print('No HDL compiler/simulator,synthesis,place-and-route,programmer or hardware was used.')
