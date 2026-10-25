"""VOFA command/FireWater contracts and independent integer/protocol models.

Does not compile, simulate, synthesize, route or flash HDL. Byte-event parsing,
full-width binary-to-decimal arithmetic and stalled stream framing are checked
independently from the RTL; static guards bind the relevant RTL architecture.
"""
from pathlib import Path
from fractions import Fraction
import random
import re
from check_motor_integration import uncomment, check_blocks, module_ports

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src' / 'motor_uart_commands.v'
source = uncomment(SOURCE.read_text(encoding='utf-8-sig'))
check_blocks(source, SOURCE.name)
code = re.sub(r'\s+', '', source)
ports = module_ports(source)['robot_vofa_uart']
expected = {'clk', 'reset_n', 'byte_valid', 'byte_data', 'emergency_stop_pulse',
            'cmd_valid', 'cmd_ready', 'cmd_seq', 'cmd_op', 'cmd_wheel', 'cmd_field',
            'cmd_value', 'rsp_valid', 'rsp_ready', 'rsp_seq', 'rsp_op', 'rsp_wheel',
            'rsp_field', 'rsp_status', 'rsp_value', 'sample_values',
            'turn_complete_valid', 'turn_complete_ready', 'turn_complete_busy',
            'tx_byte_valid', 'tx_byte_ready', 'tx_byte_data', 'tx_line_count', 'parse_errors'}
assert ports == expected, ports ^ expected
for contract in (
    'inputwire[CHANNELS*32-1:0]sample_values,', 'reg[FRAME_WORDS*32-1:0]held_values;',
    'localparamintegerFRAME_WORDS=CHANNELS<6?6:CHANNELS;',
    'assignrsp_ready=!ack_pending;',
    'if(cmd_valid&&cmd_ready)cmd_valid<=0;',
    'if(rsp_valid&&rsp_ready)outstanding<=0;',
    'if(number_valid&&rx_field==4&&!outstanding&&!cmd_valid&&',
    'elsebegincmd_value<=received_value;cmd_valid<=1;end',
    'if(byte_data=="!")beginemergency_stop_pulse<=1;collecting<=0;cmd_valid<=0;',
    'if(gain_convert_busy||(cmd_valid&&!cmd_ready))outstanding<=0;',
    'if(collecting&&!byte_valid)begin',
    'magnitude>32\'d214748364',
    'rx_digit>(negative?4\'d8:4\'d7)',
    'elsemagnitude<=(magnitude<<3)+(magnitude<<1)+{28\'d0,rx_digit};',
    "held_values<={ack_value,{24'd0,ack_status},{28'd0,ack_field},{30'd0,ack_wheel},{28'd0,ack_op},{16'd0,ack_seq}};",
    "held_values<={32'd0,{24'd0,notice_status},32'd0,32'd0,32'hFFFFFFFF,{16'd0,notice_seq}};",
    'ack_status<=rsp_status;ack_value<=rsp_value;',
    'held_values<=sample_values;',
    'wire[31:0]selected_value=held_values[31:0];',
    'wire[3:0]current_digit=decimal_digits[39:36];',
    'SEPARATOR:if(tx_byte_ready)beginheld_values<=held_values>>32;channel<=channel+1\'b1;state<=LOAD;end',
    'if(!digit_started&&digit_index!=0&&current_digit==0)begindigit_index<=digit_index-1\'b1;decimal_digits<={decimal_digits[35:0],4\'d0};end',
    'assignconsume_ack=state==IDLE&&ack_pending;',
    'assignconsume_notice=state==IDLE&&!ack_pending&&notice_pending;',
    '(state==DIGIT&&(digit_started||digit_index==0||current_digit!=0))',
    'decimal_digits<={adjusted_digits[38:0],binary_shift[31]};',
    'if(conversion_step==31)begindigit_index<=9;state<=number_negative?SIGN:DIGIT;end',
    'NEWLINE:if(tx_byte_ready)begintx_line_count<=tx_line_count+1\'b1;state<=IDLE;end',
    'parameterintegerCHANNELS=4',
    'outputreg[3:0]cmd_op,', 'inputwire[3:0]rsp_op,',
    'outputreg[1:0]cmd_wheel,', 'inputwire[1:0]rsp_wheel,',
    'reg[1:0]parsed_wheel;', 'reg[1:0]ack_wheel;',
    '2:if(magnitude<=2&&!negative)parsed_wheel<=magnitude[1:0];elsebad<=1;',
    '1:if(magnitude<=9&&!negative)parsed_op<=magnitude[3:0];elsebad<=1;',
    'assignturn_complete_ready=!event_pending;',
    'assignturn_complete_busy=event_pending||frame_event;',
    'if(turn_complete_valid&&turn_complete_ready)event_pending<=1;',
    'assignconsume_event=state==IDLE&&!ack_pending&&!notice_pending&&event_pending;',
    'if(frame_event&&prefix_index==12)state<=NEWLINE;',
    'wiregain_packet=(parsed_op==2||parsed_op==3)&&parsed_field<=2;',
    'gain_dividend<=magnitude<<8;',
    'if(gain_step==31)begincmd_value<=$signed(gain_dividend_next);cmd_valid<=1;gain_convert_busy<=0;end',
    'wire[14:0]gain_trial={gain_remainder,gain_dividend[31]};',
    'wire[31:0]gain_dividend_next={gain_dividend[30:0],gain_subtract};',
    'if(fraction_digits>=4)bad<=1;',
):
    assert contract in code, contract
assert source.count('function [39:0] add_three') == 1
assert len(re.findall(r'\breg\s+\[39:0\]\s+decimal_digits\b', source)) == 1
assert 'held_values[channel]' not in code and 'decimal_digits[digit_index' not in code
assert set(re.findall(r'held_values\[([^\]]+)\]', code)) == {'31:0'}
assert code.count("decimal_digits<={decimal_digits[35:0],4'd0};") == 2
assert not re.search(r'\b(real|shortreal|DPB|Gowin_DP|MULT|DIV)\b', source)
assert not re.search(r'[/%]\s*(?:selected|held_|magnitude|decimal)', source)


def source_prefix(ack, failed, event=False):
    """Decode actual small RTL character tables, not a duplicate test table."""
    function = source.split('function [7:0] prefix_byte;', 1)[1].split('endfunction', 1)[0]
    tables = function.split('case(index)')[1:]
    table = tables[0 if event else 2 if ack else 1].split('endcase', 1)[0]
    mapping = {}
    for key, expr in re.findall(r'(\d+|default)\s*:\s*prefix_byte\s*=\s*([^;]+);', table):
        alternatives = re.findall(r'"(.)"', expr)
        mapping[key] = alternatives[0 if failed or len(alternatives) == 1 else 1]
    length = 13 if event else (10 if failed else 9) if ack else 8
    return ''.join(mapping.get(str(i), mapping['default']) for i in range(length)).encode()


assert source_prefix(False, False) == b'samples:'
assert source_prefix(True, False) == b'[ACK OK] '
assert source_prefix(True, True) == b'[ACK ERR] '
assert source_prefix(False, False, event=True)==b'TURN_COMPLETE'


def parse_packet(line):
    """Strict complete-line protocol reference, no settings side effects."""
    assert isinstance(line, bytes)
    if not line.startswith(b'#') or not line.endswith((b'\r', b'\n')):
        raise ValueError('frame')
    words = line[1:-1].split(b',')
    if len(words) != 5 or any(not re.fullmatch(rb'[+]?[0-9]+', w) for w in words[:4]):
        raise ValueError('number/field count')
    nums = tuple(int(w) for w in words[:4])
    if any(w.startswith(b'-') for w in words[:4]):
        raise ValueError('negative metadata')
    if not (0 <= nums[0] <= 65535 and 0 <= nums[1] <= 9 and
            0 <= nums[2] <= 2 and 0 <= nums[3] <= 9):
        raise ValueError('range')
    if nums[1] in (2, 3) and nums[3] <= 2:
        if not re.fullmatch(rb'[+]?[0-9]+(?:\.[0-9]{1,4})?', words[4]):
            raise ValueError('gain syntax')
        value = Fraction(words[4].decode())
        if value >= 128:
            raise ValueError('gain range')
        native = int(value*256)
    else:
        if not re.fullmatch(rb'[+-]?[0-9]+', words[4]):
            raise ValueError('integer required')
        native = int(words[4])
        if not -(1 << 31) <= native <= (1 << 31)-1:
            raise ValueError('signed32 range')
    return nums + (native,)


class ByteReceiver:
    """Protocol-level timed byte model, including busy and out-of-band stop."""
    def __init__(self):
        self.line = None
        self.pending = None
        self.errors = []
        self.stop_count = 0

    def receive(self, byte):
        if byte == ord('!'):
            self.stop_count += 1
            self.line = None
        elif byte == ord('#'):
            self.line = bytearray(b'#')
        elif self.line is not None:
            self.line.append(byte)
            if byte in (10, 13):
                try:
                    packet = parse_packet(bytes(self.line))
                    if self.pending is not None:
                        self.errors.append(3)
                    else:
                        self.pending = packet
                except ValueError:
                    self.errors.append(1)
                self.line = None

    def feed(self, text):
        for byte in text:
            self.receive(byte)

    def timeout(self):
        if self.line is not None:
            self.errors.append(2)
            self.line = None


accepted = 0
for seq in (0, 1, 65535):
    for op in range(10):
        for wheel in range(3):
            for field in range(10):
                values = ('0', '1', '10', '127', '0.004', '0.1', '127.9999') if op in (2, 3) and field <= 2 else tuple(map(str, (-(1 << 31), -200, -1, 0, 1, 2560, 32767, (1 << 31)-1)))
                for value in values:
                    model = ByteReceiver()
                    frame = ('#%d,%d,%d,%d,%s\n' % (seq, op, wheel, field, value)).encode()
                    packet = parse_packet(frame)
                    model.feed(frame[:-1] + b'\r\n')
                    assert model.pending == packet and not model.errors
                    accepted += 1
invalid = (b'#1,2,0,3,2147483648\n', b'#1,2,0,3,-2147483649\n',
           b'#65536,2,0,0,1\n', b'#1,10,0,0,1\n', b'#1,2,3,0,1\n',
           b'#1,2,0,10,1\n', b'#-0,2,0,0,1\n', b'#1,2,0,3,1.0\n',
           b'#1,2,0,0,+\n', b'#1,2,0,0,\n', b'#1,2,0,0,1,2\n',
           b'#1,2,0,0, 1\n', b'#1,,0,0,1\n', b'#1,2,0,0,1x\n',
           b'#1,2,0,0,128\n', b'#1,2,0,0,-0\n', b'#1,2,0,0,0.00390625\n',
           b'#1,2,0,0,.5\n', b'#1,2,0,0,1.\n')
for frame in invalid:
    model = ByteReceiver()
    model.feed(frame)
    assert model.pending is None and model.errors == [1], frame
model = ByteReceiver()
model.feed(b'noise#2,2,0,0,10\n')
model.feed(b'#3,1,0,0,0\n')
assert model.pending == (2, 2, 0, 0, 2560) and model.errors == [3]
model.feed(b'#9,2,0,0,!')
assert model.stop_count == 1 and model.line is None
model.pending = None
model.feed(b'#4,2,0,0,12')
assert model.pending is None
model.timeout()
assert model.errors[-1] == 2
model.feed(b'#99,2,0,0,999#5,0,0,0,0\n')
assert model.pending == (5, 0, 0, 0, 0)


def restoring_divide(numerator, divisor):
    quotient = numerator & 0xffffffff
    remainder = 0
    for _ in range(32):
        trial = (remainder << 1) | ((quotient >> 31) & 1)
        subtract = trial >= divisor
        remainder = trial-divisor if subtract else trial
        quotient = ((quotient << 1) | int(subtract)) & 0xffffffff
    return quotient, remainder


gain_cases = 0
for scale in (1, 10, 100, 1000, 10000):
    for magnitude in range(0, 128*scale, max(1, scale//13)):
        quotient, remainder = restoring_divide(magnitude << 8, scale)
        assert quotient == (magnitude*256)//scale and remainder == (magnitude*256) % scale
        assert 0 <= quotient <= 32767
        gain_cases += 1


def bcd_decimal(value):
    """Independent full-width unsigned double-dabble arithmetic reference."""
    unsigned = value & 0xffffffff
    negative = bool(unsigned & (1 << 31))
    magnitude = ((~unsigned + 1) & 0xffffffff) if negative else unsigned
    digits = [0]*10
    for bit in range(31, -1, -1):
        digits = [d+3 if d >= 5 else d for d in digits]
        packed = sum(d << (n*4) for n, d in enumerate(digits))
        packed = ((packed << 1) | ((magnitude >> bit) & 1)) & ((1 << 40)-1)
        digits = [(packed >> (n*4)) & 15 for n in range(10)]
    text = ''.join(str(d) for d in reversed(digits)).lstrip('0') or '0'
    return ('-' if negative else '') + text


rng = random.Random(20261024)
values = [-(1 << 31), (1 << 31)-1, -1, 0, 1, -10, 10, 32767, -65536]
values += [rng.randrange(-(1 << 31), 1 << 31) for _ in range(8192)]
for value in values:
    assert bcd_decimal(value) == str(value), value
samples = [-(1 << 31), (1 << 31)-1, -10, 10]
line = source_prefix(False, False) + ','.join(map(bcd_decimal, samples)).encode() + b'\n'
ack = source_prefix(True, False) + b'12,2,0,0,0,2560\n'
both_ack = source_prefix(True, False) + b'14,2,2,0,0,2560\n'
error = source_prefix(True, True) + b'13,2,0,0,4,0\n'
assert line.count(b',') == 3 and line.endswith(b'\n')
# FireWater's ordinary CSV records must contain only numeric fields after ':';
# ACKs have no colon and a nonnumeric first token, so they remain console text.
for frame in (ack, both_ack, error):
    assert b':' not in frame and not re.fullmatch(rb'[+-]?[0-9]+', frame.split(b',')[0])
class FixedWireFormatter:
    """Independent edge-event model of fixed-word/fixed-nibble publication.

    Inbound sample pins may change every clock and replies may arrive halfway
    through a decimal number. Only accepted characters advance the wire stream;
    unprinted leading-zero nibbles may advance without a UART transaction.
    """
    def __init__(self, channels=4, shift_on_stall=False, shift_skipped=True):
        self.channels = channels
        self.width_mask = (1 << (max(6,channels) * 32)) - 1
        self.state = 'IDLE'
        self.words = self.decimal = self.binary = self.bit = 0
        self.channel = self.last_channel = self.remaining_digits = self.prefix_index = 0
        self.prefix = b''
        self.negative = self.started = self.sample_due = False
        self.ack_pending = self.notice_pending = None
        self.event_pending=False
        self.frame_kind = None
        self.expected = self.line = b''
        self.frames = []
        self.stalls = self.zero_skips = self.accepted = self.reply_accepts = 0
        self.stalled_states = set()
        self.shift_on_stall, self.shift_skipped = shift_on_stall, shift_skipped
        self.previous_stall = None

    @staticmethod
    def pack(words):
        return sum((value & 0xffffffff) << (32 * slot) for slot, value in enumerate(words))

    def output(self):
        if self.state == 'PREFIX':
            return True, self.prefix[self.prefix_index]
        if self.state == 'SIGN':
            return True, ord('-')
        if self.state == 'DIGIT':
            digit = (self.decimal >> 36) & 15
            return self.started or self.remaining_digits == 0 or digit != 0, ord('0') + digit
        if self.state == 'SEPARATOR':
            return True, ord(',')
        return self.state == 'NEWLINE', 10

    def start(self, kind, values):
        self.frame_kind = kind
        self.prefix = source_prefix(kind not in ('sample','event'), kind in ('error', 'notice'),event=kind=='event')
        self.words = self.pack(values) & self.width_mask
        self.last_channel = len(values) - 1
        self.expected = self.prefix + (b'' if kind=='event' else ','.join(map(str, values)).encode()) + b'\n'
        self.state = 'PREFIX'
        self.line = b''

    def tick(self, sample, ready, due=False, response=None, notice=None,event=False):
        valid, byte = self.output()
        if self.previous_stall is not None:
            assert (valid, byte) == self.previous_stall, 'pending byte changed during backpressure'
        self.previous_stall = (valid, byte) if valid and not ready else None
        if valid and not ready:
            self.stalls += 1
            self.stalled_states.add(self.state)
        if valid and ready:
            self.line += bytes((byte,))
            self.accepted += 1
        # Input acceptance and publication both see the old mailbox contents.
        # A backend ACK arriving on this edge cannot retroactively replace a
        # sample selected on the same edge or splice into its decimal digits.
        ack_before, notice_before, due_before = self.ack_pending, self.notice_pending, self.sample_due
        event_before=self.event_pending
        if event and not event_before:self.event_pending=True
        response_accepted = response is not None and ack_before is None
        if response_accepted:
            self.ack_pending = tuple(response)
            self.reply_accepts += 1
        if notice is not None and notice_before is None:
            self.notice_pending = tuple(notice)
        if due:
            self.sample_due = True
        if self.state == 'IDLE':
            self.channel = self.prefix_index = 0
            if ack_before is not None:
                self.ack_pending = None
                self.start('error' if ack_before[4] else 'ack', ack_before)
            elif notice_before is not None:
                self.notice_pending = None
                self.start('notice', notice_before)
            elif event_before:
                self.event_pending=False
                self.start('event',())
            elif due_before:
                self.sample_due = False
                self.start('sample', tuple(sample))
        elif self.state == 'PREFIX' and ready:
            if self.prefix_index == len(self.prefix) - 1:
                self.state = 'NEWLINE' if self.frame_kind=='event' else 'LOAD'
            else:
                self.prefix_index += 1
        elif self.state == 'LOAD':
            unsigned = self.words & 0xffffffff
            self.negative = bool(unsigned & 0x80000000)
            self.binary = ((~unsigned + 1) & 0xffffffff) if self.negative else unsigned
            self.decimal = self.bit = 0
            self.started = False
            self.state = 'CONVERT'
        elif self.state == 'CONVERT':
            adjusted = self.decimal
            for slot in range(10):
                nibble = (self.decimal >> (slot * 4)) & 15
                if nibble >= 5:
                    adjusted += 3 << (slot * 4)
            self.decimal = ((adjusted << 1) | (self.binary >> 31)) & ((1 << 40) - 1)
            self.binary = (self.binary << 1) & 0xffffffff
            self.bit += 1
            if self.bit == 32:
                self.remaining_digits = 9
                self.state = 'SIGN' if self.negative else 'DIGIT'
        elif self.state == 'SIGN' and ready:
            self.state = 'DIGIT'
        elif self.state == 'DIGIT':
            digit = (self.decimal >> 36) & 15
            if not self.started and self.remaining_digits != 0 and digit == 0:
                self.remaining_digits -= 1
                self.zero_skips += 1
                if self.shift_skipped:
                    self.decimal = (self.decimal << 4) & ((1 << 40) - 1)
            elif ready:
                self.started = True
                self.decimal = (self.decimal << 4) & ((1 << 40) - 1)
                if self.remaining_digits == 0:
                    self.state = 'NEWLINE' if self.channel == self.last_channel else 'SEPARATOR'
                else:
                    self.remaining_digits -= 1
        elif self.state == 'SEPARATOR':
            if ready or self.shift_on_stall:
                self.words >>= 32
            if ready:
                self.channel += 1
                self.state = 'LOAD'
        elif self.state == 'NEWLINE' and ready:
            self.frames.append((self.frame_kind, self.line, self.expected))
            self.state = 'IDLE'
        return response_accepted


def stalled_formatter_run(shift_on_stall=False, shift_skipped=True):
    local_rng = random.Random(20261024)
    formatter = FixedWireFormatter(shift_on_stall=shift_on_stall, shift_skipped=shift_skipped)
    responses = [(65535, 7, 1, 9, 0, -(1 << 31)), (12, 2, 2, 0, 4, (1 << 31)-1)]
    reply_index = 0
    notice_sent = False
    forced_state_stalls = {}
    for tick in range(36000):
        # Every input sample changes while prior captured words are formatting.
        # Include signed endpoints at both ends and many leading-zero cases.
        sample = [-(1 << 31)] + [((tick * (slot + 3) * 997) & 0x7fffffff) * (-1 if slot % 2 else 1)
                                  for slot in range(1, 3)] + [(1 << 31)-1]
        if tick < 80:
            sample = samples
        ready = local_rng.randrange(4) == 0
        # Hold the first occurrence of each publishing phase for7clocks.
        if formatter.state in ('PREFIX', 'SIGN', 'DIGIT', 'SEPARATOR', 'NEWLINE'):
            count = forced_state_stalls.get(formatter.state, 0)
            if count < 7:
                ready = False
                forced_state_stalls[formatter.state] = count + 1
        response = responses[reply_index] if tick >= 80 and reply_index < len(responses) else None
        notice = (13, -1, 0, 0, 3, 0) if tick >= 200 and not notice_sent else None
        if formatter.tick(sample, ready, due=tick % 1500 == 0, response=response, notice=notice,event=tick==200):
            reply_index += 1
        if notice is not None:
            notice_sent = True
    assert formatter.reply_accepts == 2
    assert len(formatter.frames) >= 20
    assert [kind for kind, _, _ in formatter.frames[:5]] == ['sample', 'ack', 'error', 'notice','event']
    assert sum(kind=='event' for kind,_,_ in formatter.frames)==1
    assert formatter.stalled_states == {'PREFIX', 'SIGN', 'DIGIT', 'SEPARATOR', 'NEWLINE'}
    if not shift_on_stall and shift_skipped:
        assert formatter.zero_skips > 200 and formatter.stalls > 3500, (formatter.zero_skips,formatter.stalls)
    return formatter


formatted = stalled_formatter_run()
for kind, observed, expected_frame in formatted.frames:
    assert observed == expected_frame, (kind, observed, expected_frame)
    assert observed.count(b',') == (3 if kind == 'sample' else 0 if kind=='event' else 5)
    assert observed.endswith(b'\n')
# Negative controls: the checker detects advancing a word on a rejected comma
# or failing to advance a leading-zero nibble. Neither mutation is in the RTL.
bad_word = stalled_formatter_run(shift_on_stall=True)
bad_nibble = stalled_formatter_run(shift_skipped=False)
assert any(actual != expected_frame for _, actual, expected_frame in bad_word.frames)
assert any(actual != expected_frame for _, actual, expected_frame in bad_nibble.frames)
worst_sample_bytes = len(b'samples:') + 4*len(b'-2147483648') + 3 + 1
assert worst_sample_bytes == 56
assert worst_sample_bytes*20 < 115200//10
print(f'PASS VOFA: {accepted} valid commands, {len(invalid)} malformed frames, busy/stop/timeout/restart, '
      f'{gain_cases} decimal-gain divisions, {len(values)} signed32 conversions, '
      f'{len(formatted.frames)} atomic fixed-wire frames/{formatted.stalls} publication stalls, '
      'ACK switching and INT32_MIN; '
      f'20 Hz worst-case telemetry {worst_sample_bytes*20} B/s < 11520 B/s. No HDL tools run.')
