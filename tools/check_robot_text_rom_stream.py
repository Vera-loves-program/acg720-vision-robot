#!/usr/bin/env python3
"""Independent synchronous ROM/byte-emitter event model; never executes HDL.

Checks source contracts against a separate clock-edge model, exact original
ASCII literals, joined UART/LCD backpressure, atomic snapshots, reset, numeric
prefix transitions and optional parallel compatibility output. This is not
HDL simulation, synthesis, timing closure or proof of hardware behavior.
"""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import random
import re

from check_formatter_stream import ROBOT_DEFAULTS, ROBOT_TITLES, robot_rows, robot_flags

ROOT = Path(__file__).resolve().parents[1]
FROZEN_WORDS = [
    "                ",
    "1 MOTOR DIR/PID ",
    "2 LIDAR RX/MAP  ",
    "3 POSE/GEOMETRY ",
    "4 MANUAL/NAV    ",
    "ALL STOP LATCHED",
    "ARM FIRST       ",
    "ARRIVED         ",
    "BACK TO NAV     ",
    "C1 DIAGNOSTICS  ",
    "C1 HEALTH ERROR ",
    "C1 LIVE         ",
    "C1 NO RX        ",
    "C1 REQ OFF      ",
    "C1 STALE        ",
    "C1 STARTING     ",
    "C1 WAIT HEALTH  ",
    "C1 WAIT NODE    ",
    "C1 WAIT SCAN    ",
    "CAL DISARMED    ",
    "CAL JOG ACTIVE  ",
    "CAL TEST ARMED  ",
    "CALIBRATE FIRST ",
    "CAMERA ON FPGA  ",
    "CAMERA TASK ONE ",
    "CHECK LIDAR     ",
    "CHECK SETTINGS  ",
    "CHOOSE ONE STEP ",
    "CNT/10MS CNT PWM",
    "COUNTERS UINT32 ",
    "CRUISE COUNT/10M",
    "CURRENT POSE    ",
    "DEG / OFFSET MM ",
    "EDIT POS LOOP   ",
    "EDIT PWM LIMITS ",
    "EDIT SPEED LOOP ",
    "ENC COUNT / REV ",
    "ENCODER / MOTOR ",
    "FPGA CAMERA     ",
    "FPGA CAMERA LCD ",
    "FRONT UNKNOWN   ",
    "GAINS READY     ",
    "GOAL POSITION   ",
    "GOING TO TARGET ",
    "GYRO CAL OK     ",
    "GYRO CAL WAIT   ",
    "GYRO CALIBRATION",
    "GYRO Z INVERT   ",
    "GYRO ZERO FIRST ",
    "HOME TO CONTROL ",
    "KEEP BODY STILL ",
    "KEEP STILL GYRO ",
    "LEFT / MOTOR1   ",
    "LEFT UNKNOWN    ",
    "LIDAR INSTALL   ",
    "LIDAR MAP       ",
    "LIDAR X OFF MM  ",
    "LIDAR Y OFF MM  ",
    "LIDAR ZERO DEG  ",
    "LIMITS / TARGET ",
    "LOCAL ARMED     ",
    "LOCAL CAMERA LCD",
    "LOCAL DISARMED  ",
    "LOCAL VIDEO ONLY",
    "MANUAL ARMED    ",
    "MANUAL DRIVE    ",
    "MANUAL DRIVING  ",
    "MANUAL SPEEDS   ",
    "MM / YAW MDEG   ",
    "MM, COUNTS/REV  ",
    "MOTOR ARMED     ",
    "MOTOR DISARMED  ",
    "MOTOR FAULT     ",
    "MOTOR LIMITS    ",
    "MOTOR RUNNING   ",
    "MOTOR SETUP     ",
    "NAV CALIBRATION ",
    "NAV FAULT       ",
    "NAV SPEED SETUP ",
    "NEED SPEED GAINS",
    "NO LIDAR GUARD  ",
    "NO SAFE PATH    ",
    "NO VIDEO TO SBC ",
    "OBSTACLE STOP   ",
    "OPEN VALUE EDIT ",
    "PID VALUE EDIT  ",
    "PLANNING        ",
    "POINT NAVIGATION",
    "POSITION PID    ",
    "POSITION TEST   ",
    "PWM / DEADBAND  ",
    "RANGE MM / PTS  ",
    "RAW MANUAL RUN  ",
    "RIGHT / MOTOR2  ",
    "RIGHT UNKNOWN   ",
    "ROBOT HOME      ",
    "SELECT A PAGE   ",
    "SENSOR UNKNOWN  ",
    "SET CURRENT X MM",
    "SET CURRENT Y MM",
    "SET GEOMETRY    ",
    "SET GOAL X MM   ",
    "SET GOAL Y MM   ",
    "SET KP KI KD    ",
    "SET SPEED ONLY  ",
    "SET YAW MDEG    ",
    "SET/VEL CNT/10MS",
    "SPEED LOOP TEST ",
    "SPEED PID       ",
    "SPEED PID FIRST ",
    "STOP AT ANY TIME",
    "STOP BEFORE EDIT",
    "TEST AT LOW RATE",
    "TUNE BOTH GAINS ",
    "TUNE BOTH WHEELS",
    "TURN COUNT/10MS ",
    "TURNING         ",
    "UNITS CNT/10MS  ",
    "USB CAM ON SBC  ",
    "VALUE / STEP    ",
    "VALUE EDITOR    ",
    "WHEEL CIRC MM   ",
    "WHEEL GEOMETRY  ",
    "WHEEL TEST      ",
    "WHEEL TRACK MM  ",
    "XY MM YAW MDEG  ",
    "YAW MDEG Z RAW  ",
    "Z AXIS VERTICAL "
]
FROZEN_PREFIXES = [
    "     ",
    "CIRC ",
    "CPR  ",
    "CRUIS",
    "FR   ",
    "GEO  ",
    "GX   ",
    "GY   ",
    "LF   ",
    "LINK ",
    "LPOS ",
    "LPWM ",
    "LSET ",
    "LSPD ",
    "MAPPT",
    "NODE ",
    "PERR ",
    "POS  ",
    "PTS  ",
    "PWM  ",
    "RPOS ",
    "RPWM ",
    "RSET ",
    "RSPD ",
    "RST  ",
    "RT   ",
    "RX   ",
    "SCANS",
    "SETX ",
    "SETY ",
    "SPD  ",
    "STEP ",
    "SYAW ",
    "TRACK",
    "TURN ",
    "UERR ",
    "VALUE",
    "X    ",
    "XOFF ",
    "Y    ",
    "YAW  ",
    "YOFF ",
    "ZERO ",
    "ZINV ",
    "ZRAW "
]
FROZEN_PREFIXES += ['LTGT ','LKP  ','LKI  ','LKD  ','RTGT ','RKP  ','RKI  ','RKD  ']
FROZEN_PREFIXES += ['BK   ','SELF ','DROP ']


def compact(source):
    return re.sub(r"\s+", "", re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S))


def decimal_tail(value, negative=False):
    """Double-dabble plus independent fixed-byte prefix-zero ASCII reference."""
    assert 0 <= value < 1 << 32
    bcd = 0
    for bit in range(31, -1, -1):
        adjusted = sum((((bcd >> (digit * 4)) & 15) +
                        (3 if ((bcd >> (digit * 4)) & 15) >= 5 else 0))
                       << (digit * 4) for digit in range(10))
        bcd = ((adjusted << 1) | ((value >> bit) & 1)) & ((1 << 40) - 1)
    digits = [(bcd >> (k * 4)) & 15 for k in range(10)]
    nonzero = [any(digits[k:]) for k in range(10)]
    result = []
    for k in range(10, -1, -1):
        if k == 0 or k < 10 and nonzero[k]:
            result.append(48 + digits[k])
        elif negative and k > 0 and (nonzero[k - 1] or k == 1):
            result.append(45)
        else:
            result.append(32)
    return bytes(result)


@dataclass(frozen=True)
class Row:
    rom_id: int
    numeric: bool = False
    tail: bytes = b" " * 11
    overlay: tuple = ()


class RomFrameModel:
    """One registered ROM Q, explicit STORE byte0 and accepted-byte prefetch.

    Legacy views schedule eight numeric slots; the speed dashboard schedules
    ten. Each uses32 invalid conversion cycles. Numeric ASCII is supplied by
    independent decimal/fixed4 references, never by executing the RTL.
    """
    def __init__(self, words, parallel_compat=True):
        self.words = words
        self.parallel_compat = parallel_compat
        self.reset()

    def reset(self):
        self.state = "IDLE"
        self.q = None
        self.row = 0
        self.column = 15  # Deliberately dirty: STORE must explicitly request0.
        self.active_id = 0
        self.numeric = False
        self.convert_remaining = 0
        self.number = b" " * 11
        self.descriptors = ()
        self.expected = b""
        self.held_flags = 0
        self.flags = 0
        self.text_valid = False
        self.compat_pending = bytearray()
        self.text_rows = bytes(256)
        self.accepted = []
        self.sinks = [[], []]
        self.commits = 0
        self.live = {}
        self.last_offer = None
        self.dashboard = False

    def start(self, descriptors, expected, flags):
        assert self.state == "IDLE" and len(descriptors) == 16 and len(expected) == 256
        self.descriptors = tuple(deepcopy(descriptors))
        self.expected = bytes(expected)
        self.held_flags = flags
        self.dashboard=(flags&7)==1 and ((flags>>28)&7) in (2,3)
        self.row = 0
        self.state = "LOAD"
        self.text_valid = False
        self.compat_pending.clear()
        self.accepted.clear()
        self.sinks = [[], []]

    def offer(self):
        if self.state != "EMIT":
            return None
        assert self.q is not None
        desc = self.descriptors[self.row]
        data = self.number[0] if self.numeric and self.column >= 5 else self.q
        for column, replacement in desc.overlay:
            if self.column == column:
                data = replacement
        return self.row * 16 + self.column, data

    def step(self, ready_a=True, ready_b=True, reset=False):
        if reset:
            self.reset()
            return
        self.text_valid = False
        ready = ready_a and ready_b
        before = self.offer()
        if before is not None:
            # Mirrored top-level joining: A valid includes B ready, vice versa.
            a_accept = ready_b and ready_a
            b_accept = ready_a and ready_b
            if a_accept:
                self.sinks[0].append(before)
            if b_accept:
                self.sinks[1].append(before)
            assert a_accept == b_accept

        if self.state == "LOAD":
            desc = self.descriptors[self.row]
            self.active_id, self.numeric = desc.rom_id, desc.numeric
            converts=(2 <= self.row <= 6 or self.row >= 11) if self.dashboard else \
                     (2 <= self.row <= 7 or self.row in (12,13))
            if converts:
                self.convert_remaining = 32
                self.state = "CONVERT"
            else:
                self.state = "STORE"
        elif self.state == "CONVERT":
            assert self.convert_remaining > 0
            self.convert_remaining -= 1
            if not self.convert_remaining:
                self.state = "STORE"
        elif self.state == "STORE":
            # Synchronous ROM uses the PRE-edge row ID and explicit column0.
            self.q = self.words[self.active_id][0]
            self.column = 0
            self.number = self.descriptors[self.row].tail
            self.state = "EMIT"
        elif self.state == "EMIT":
            if ready:
                index, data = before
                assert index == len(self.accepted), (index, len(self.accepted))
                self.accepted.append((index, data))
                self.compat_pending.append(data)
                if self.column != 15 and (not self.numeric or self.column < 4):
                    self.q = self.words[self.active_id][self.column + 1]
                if self.column == 15:
                    self.state = "COMMIT" if self.row == 15 else "LOAD"
                    if self.row != 15:
                        self.row += 1
                else:
                    if self.numeric and self.column >= 5:
                        self.number = self.number[1:] + b"\0"
                    self.column += 1
            else:
                assert self.offer() == before, "ROM/data/index advanced under backpressure"
        elif self.state == "COMMIT":
            assert len(self.accepted) == 256
            assert bytes(data for _, data in self.accepted) == self.expected
            assert self.sinks[0] == self.sinks[1] == self.accepted
            self.flags = self.held_flags
            self.text_rows = bytes(self.compat_pending) if self.parallel_compat else bytes(256)
            self.text_valid = True
            self.commits += 1
            self.state = "IDLE"
        self.last_offer = before


def descriptors_for(sample, words):
    lookup = {word: index for index, word in words.items()}
    expected = robot_rows(sample)
    view=(sample['page']&7)*8+(sample['subpage']&7)
    result = []
    for index, row in enumerate(expected):
        if view==16 and index in (8,10,11,12,13,14,15):
            label={8:'RX',10:'ST',11:'H0',12:'H1',13:'S0',14:'S1',15:'TX'}[index]
            template=(label+' 00000000     ').encode()
            result.append(Row(lookup[template],overlay=tuple((k,row[k]) for k in range(3,11))))
        elif index == 14 and view not in (10,11,16):
            template = b"M 00 B00 D00    "
            # Only these six hexadecimal digits may differ from the ROM.
            positions = (2, 3, 6, 7, 10, 11)
            assert template in lookup, "missing match-score ROM template"
            result.append(Row(lookup[template], overlay=tuple((k, row[k]) for k in positions)))
        elif index == 15 and view not in (10,11,16):
            template = b"GEON GYRN C100  "
            positions = (3, 8, 12, 13)
            assert template in lookup, "missing flags ROM template"
            result.append(Row(lookup[template], overlay=tuple((k, row[k]) for k in positions)))
        elif row in lookup:
            result.append(Row(lookup[row]))
        else:
            prefix, tail = row[:5], row[5:]
            assert prefix.decode() in FROZEN_PREFIXES, (index, row)
            if b'.' in tail:
                parsed=int(tail.decode().replace('.',''))
                digits=decimal_tail(parsed).decode().strip().zfill(5)
                expected_tail=((digits[:-4] or '0')+'.'+digits[-4:]).rjust(11).encode()
            else:
                parsed = int(tail.decode())
                expected_tail = decimal_tail(abs(parsed), parsed < 0)
            assert expected_tail == tail, (index, row)
            assert prefix + b" " * 11 in lookup, (index, prefix)
            result.append(Row(lookup[prefix + b" " * 11], numeric=True, tail=expected_tail))
    return result, b"".join(expected)


def check_events(words):
    rng = random.Random(0x275)
    # Every approved view, sign extrema, uint32 high-bit counters and mixed rows.
    cases = []
    for variant in range(8):
        for view in ROBOT_TITLES:
            s = {**ROBOT_DEFAULTS, "page": view >> 3, "subpage": view & 7}
            s.update(value=(-2147483648, -1, 0, 2147483647)[variant % 4],
                     step=(0, 1, 2147483648, 4294967295)[variant % 4],
                     x=-2147483648, y=2147483647, heading=-90000,
                     speed_l=-2147483648, speed_r=2147483647,
                     position_l=-1, position_r=0, duty_l=-32768, duty_r=32767,
                     set_l=65535, set_r=1, gyro_raw=-32768,
                     xoff=-32768, yoff=32767, goal_x=4000, goal_y=-4000,
                     circ=204, cpr=65535, track=600, cruise=200, turn=1,
                     rx=4294967295, node=2147483648, points=1, uerr=0,
                     motor_target_l=-32768,motor_target_r=32767,
                     kp_l=(0,1,128,255,256,257,4096,65535)[variant],
                     ki_l=variant,kd_l=65535-variant,kp_r=variant*256,ki_r=65535,kd_r=0,
                     pos_target_l=-2147483648+variant,pos_target_r=2147483647-variant,
                     pkp_l=variant*256,pki_l=variant,pkd_l=65535-variant,
                     pkp_r=65535,pki_r=0,pkd_r=variant,
                     lidar_debug_status=0x64001300+variant,lidar_last_rx_word=0xA55A4081+variant,
                     lidar_reply_capture=0xA55A030000000600A55A050000004081+variant,
                     lidar_last_tx_word=0xA525A520+variant,
                     perr=2147483647, restarts=1234567890, map_points=4294967295, scans=2147483648,
                     score=variant * 31, baseline=255-variant, margin=variant * 17,
                     front=(49, 50, 16000, 16001)[variant % 4], left=65535, right=1000,
                     armed=bool(variant & 1), gains=bool(variant & 2), gyro=bool(variant & 4),
                     geometry=bool(variant & 1), stop=bool(variant & 2), scan_enable=bool(variant & 4),
                     lidar_error=(0, 1, 2, 4, 5, 0x13, 0x14, 0xff)[variant],
                     nav_fault=variant * 17, nav_status=variant, wheel=bool(variant & 1),
                     enc_l=bool(variant & 2), enc_r=bool(variant & 4),
                     drive_l=bool(variant & 4), drive_r=bool(variant & 2))
            cases.append(s)
    accepted = stalls = transitions = 0
    for compatibility in (False, True):
        for sample in cases:
            desc, expected = descriptors_for(sample, words)
            model = RomFrameModel(words, compatibility)
            flags = robot_flags(sample)
            model.flags = 0x12345678
            model.text_rows = b"Z" * 256 if compatibility else bytes(256)
            previous_rows = model.text_rows
            model.start(desc, expected, flags)
            forced_stalls = set()
            cycles = 0
            while not model.commits:
                offered = model.offer()
                if offered is not None:
                    index, _ = offered
                    if index not in forced_stalls and index % 16 in (0, 4, 5, 15):
                        # Both consumers block separately at the ROM/numeric
                        # boundary, first byte, and last byte of every row.
                        for ra, rb in ((False, True), (True, False), (False, False)):
                            q = model.q
                            model.step(ra, rb)
                            assert model.offer() == offered and model.q == q
                            assert model.flags == 0x12345678 and model.text_rows == previous_rows
                            stalls += 1
                        forced_stalls.add(index)
                    ready_a, ready_b = rng.randrange(4) != 0, rng.randrange(4) != 0
                else:
                    ready_a, ready_b = bool(rng.getrandbits(1)), bool(rng.getrandbits(1))
                # Live sensor/UI changes must not leak into the held frame.
                model.live = {"page": rng.randrange(8), "rx": rng.getrandbits(32),
                              "flags": rng.getrandbits(32)}
                old_count = len(model.accepted)
                old_state = model.state
                model.step(ready_a, ready_b)
                accepted += len(model.accepted) - old_count
                transitions += old_state != model.state
                cycles += 1
                assert cycles < 12000, "joined stream stranded under finite backpressure"
                if model.state != "IDLE":
                    assert not model.text_valid and model.flags == 0x12345678
                    assert model.text_rows == previous_rows
            assert model.flags == flags and model.text_valid
            assert model.text_rows == (expected if compatibility else bytes(256))
            assert model.accepted == list(enumerate(expected))

    # Reset before every possible byte and immediately before COMMIT. Neither
    # a suffix nor stale metadata may become a newly published frame.
    first, second = cases[5], cases[-1]
    first_desc, first_expected = descriptors_for(first, words)
    second_desc, second_expected = descriptors_for(second, words)
    # The real compatibility shift is not cleared between snapshots. A full
    # next frame must still replace all256 bytes and publish new flags once.
    for compatibility in (False, True):
        model = RomFrameModel(words, compatibility)
        for sample, descriptors, expected in ((first, first_desc, first_expected),
                                               (second, second_desc, second_expected)):
            before_flags, before_rows = model.flags, model.text_rows
            wanted_commits = model.commits + 1
            model.start(descriptors, expected, robot_flags(sample))
            while model.commits < wanted_commits:
                if model.state != "COMMIT":
                    assert model.flags == before_flags and model.text_rows == before_rows
                model.step()
            assert model.flags == robot_flags(sample)
            assert model.text_rows == (expected if compatibility else bytes(256))
    for compatibility in (False, True):
        for cut in range(257):
            model = RomFrameModel(words, compatibility)
            model.start(first_desc, first_expected, robot_flags(first))
            while len(model.accepted) < cut:
                model.step()
            assert not model.text_valid
            model.step(reset=True)
            assert model.state == "IDLE" and not model.text_valid and model.flags == 0
            assert model.offer() is None and model.text_rows == bytes(256)
            model.start(second_desc, second_expected, robot_flags(second))
            while not model.commits:
                model.step()
            assert model.accepted == list(enumerate(second_expected))
            assert model.flags == robot_flags(second)
    state_cuts = 0
    for compatibility in (False, True):
        for target, remaining in (("LOAD", None), ("STORE", None), ("EMIT", None),
                                  ("CONVERT", 32), ("CONVERT", 16), ("CONVERT", 1),
                                  ("COMMIT", None)):
            model = RomFrameModel(words, compatibility)
            model.start(first_desc, first_expected, robot_flags(first))
            while model.state != target or remaining is not None and model.convert_remaining != remaining:
                model.step()
            assert not model.text_valid
            model.step(reset=True)
            assert model.offer() is None and model.flags == 0 and model.text_rows == bytes(256)
            model.start(second_desc, second_expected, robot_flags(second))
            while not model.commits:
                model.step()
            assert model.accepted == list(enumerate(second_expected))
            assert model.flags == robot_flags(second)
            state_cuts += 1
    return len(cases) * 2, accepted, stalls, transitions, 257 * 2 + state_cuts


def rom_source_checks(source):
    code = compact(source)
    helper = source.split("module robot_status_label_rom(", 1)[1].split("endmodule", 1)[0]
    helper_code = compact(helper)
    assert '(*syn_romstyle="block_rom"*)reg[7:0]memory[0:4095];' in helper_code
    assert "always@(posedgeclk)if(read_enable)data<=memory[address];" in helper_code
    loop = re.search(r"for\(init_index=0;init_index<(\d+);init_index=init_index\+1\)begin(.*?)end",
                     helper_code)
    assert loop is not None
    bound = int(loop[1])
    assert bound <= 2000, "ROM initializer exceeds Gowin's2000-iteration limit"
    writes = re.findall(r"memory\[init_index(?:\+(\d+))?\]=8'h([0-9a-fA-F]+);", loop[2])
    assert len(writes) == 4
    image = [None] * 4096
    for index in range(bound):
        for offset, value in writes:
            address = index + int(offset or 0)
            assert 0 <= address < len(image) and image[address] is None
            image[address] = int(value, 16)
    assert image == [32] * 4096, "ROM SPACE initialization has holes or overlaps"
    assigned = set()
    for address, value in re.findall(r"memory\[(\d+)\]=8'h([0-9a-fA-F]+);", helper):
        address, value = int(address), int(value, 16)
        assert address not in assigned and address < 4096
        assigned.add(address)
        image[address] = value
    words = {index: bytes(image[index * 16:index * 16 + 16]) for index in range(256)}
    declarations = {}
    for label, ident, literal in re.findall(
            r'localparam\s+\[7:0\]\s+TEXT(\d+)=8\'d(\d+);\s*//\s*"([^"\n]*)"', source):
        label, ident = int(label), int(ident)
        assert label == ident and ident not in declarations
        assert len(literal) == 16 and literal.isascii()
        declarations[ident] = literal.encode("ascii")
    comments = {int(ident): literal.encode("ascii") for ident, literal in re.findall(
        r'//\s*TEXT(\d+):\s*"([^"\n]*)"', helper)}
    assert len(declarations) == 256 and set(declarations) == set(range(256))
    assert {ident: declarations[ident] for ident in (203,204,205)} == {
        203:b"LOCAL LIVE SCAN ", 204:b"WORLD MAP       ", 205:b"WORLD MAP HOLD  "}, \
        "map-mode labels must retain exact IDs, widths and spaces"
    assert comments == declarations, "ROM comments and named constants disagree"
    assert all(words[ident] == literal for ident, literal in declarations.items()), \
        "actual ROM bytes differ from their declared16-character strings"
    assert set(word.encode("ascii") for word in FROZEN_WORDS) <= set(words.values()), \
        "a pre-optimization static literal or its spacing changed"
    assert set((word + " " * 11).encode("ascii") for word in FROZEN_PREFIXES) <= set(words.values()), \
        "a pre-optimization numeric prefix or its spacing changed"

    required = (
        "parameterintegerPARALLEL_COMPAT=1",
        "reg[7:0]row_ids[0:11];",
        "wire[31:0]selected_raw=values[field];",
        "wire[31:0]selected_magnitude=signs[field]?~selected_raw+32'd1:selected_raw;",
        "assigntext_byte_valid=reset_n&&state==EMIT;",
        "assigntext_byte_index={rowfield,byte_in_row};",
        "wirelabel_read=state==STORE||(state==EMIT&&text_byte_ready&&byte_in_row!=15&&(!emit_numeric||byte_in_row<4));",
        "wire[3:0]label_column=state==STORE?4'd0:byte_in_row+4'd1;",
        ".clk(clk),.read_enable(label_read),.address({emit_id,label_column}),.data(label_byte)",
        "assigntext_byte_data=emit_numeric&&byte_in_row>=5?emit_number[87:80]:decorated_byte;",
        "LOAD:beginemit_id<=formatted_id;emit_numeric<=formatted_numeric;",
        "shift<=selected_conversion;bcd<=0;bits<=0;state<=CONVERT;",
        "STORE:beginemit_number<=number;byte_in_row<=0;state<=EMIT;end",
        "EMIT:if(text_byte_ready)beginif(emit_numeric&&byte_in_row>=5)emit_number<={emit_number[79:0],8'd0};",
        "if(rowfield==15)state<=COMMIT;",
        "COMMIT:beginflags<=held_flags;text_valid<=1;state<=IDLE;end",
        "generateif(PARALLEL_COMPAT!=0)begin:parallel_compatibility",
        "if(text_byte_valid&&text_byte_ready)accepted_frame<={accepted_frame[2039:0],text_byte_data};",
        "if(state==COMMIT)published_frame<=accepted_frame;",
        "endelsebegin:no_parallel_compatibilityassigntext_rows=2048'd0;",
        "held_score<=match_score;held_baseline<=match_baseline;held_margin<=match_margin;held_lidar_error<=lidar_error;",
        "held_debug_status<=lidar_debug_status;held_last_rx_word<=lidar_last_rx_word;",
        "wire[87:0]number=fractional[field]?fixed4_text(bcd):decimal_text(bcd,signs[field]);",
        "wire[25:0]gain_product=selected_raw[15:0]*26'd625;",
        "wireheld_pid_view=held_view==10||held_view==11;",
        "if(!held_pid_view&&held_view!=16&&rowfield==14)",
        "if(!held_pid_view&&held_view!=16&&rowfield==15)",
        "if(held_pid_view?((rowfield>=2&&rowfield<=6)||rowfield>=11):",
        "if(held_pid_view?((rowfield>=2&&rowfield<=6)||(rowfield>=11&&rowfield<=14)):",
        "if(held_view==16&&(rowfield==8||rowfield==10||rowfield>=11))",
        "wire[31:0]debug_word=rowfield==8?held_last_rx_word:",
        "rowfield==11?held_reply_h0:rowfield==12?held_reply_h1:",
        "rowfield==13?held_reply_s0:rowfield==14?held_reply_s1:",
        "rowfield==15?held_last_tx_word:held_debug_status;",
        "held_reply_h0<=lidar_reply_capture[127:96];held_reply_h1<=lidar_reply_capture[95:64];",
        "held_reply_s0<=lidar_reply_capture[63:32];held_reply_s1<=lidar_reply_capture[31:0];",
        "held_last_tx_word<=lidar_last_tx_word;",
    )
    for token in required:
        assert token in code, ("ROM emitter source/model contract changed", token)
    assert "reg[127:0]" not in code and "function[127:0]" not in code
    assert "rows[" not in code, "wide row bank or runtime whole-row selector reintroduced"
    assert "read_enable" not in re.search(r"assigntext_byte_valid=([^;]+)", code)[1]
    emit = code.split("EMIT:if(text_byte_ready)begin", 1)[1].split("COMMIT:begin", 1)[0]
    for live in ("match_score", "match_baseline", "match_margin", "lidar_error", "speed_left",
                 "selected_value", "page", "subpage", "nav_status", "selected_field"):
        assert not re.search(r"\b" + live + r"\b", emit)
    assert "if(!reset_n)beginaccepted_frame<=0;published_frame<=0;end" in code
    return words


def check_mutations(source):
    # Ensure that real mistakes in a synchronous read port are rejected by
    # the lexical coupling rather than quietly testing a detached model.
    replacements = (
        ("state==STORE ? 4'd0 : byte_in_row+4'd1", "byte_in_row"),
        ("byte_in_row!=15 && (!emit_numeric || byte_in_row<4)", "byte_in_row!=15"),
        ("state==EMIT && text_byte_ready &&", "state==EMIT &&"),
        ("if(emit_numeric && byte_in_row>=5)", "if(emit_numeric && byte_in_row>=4)"),
        (".address({emit_id,label_column})", ".address({emit_id,byte_in_row})"),
        ("memory[16]=8'h53;", "memory[16]=8'h58;"),
        ("memory[init_index+3072]=8'h20;", "memory[init_index+3071]=8'h20;"),
        ("!held_pid_view && held_view!=16 && rowfield==14", "rowfield==14"),
        ("!held_pid_view && held_view!=16 && rowfield==15", "rowfield==15"),
        ("gain_product=selected_raw[15:0]*26'd625", "gain_product=selected_raw[15:0]*26'd624"),
        ("held_debug_status<=lidar_debug_status;held_last_rx_word<=lidar_last_rx_word;",
         "held_debug_status<=0;held_last_rx_word<=lidar_last_rx_word;"),
        ("held_reply_h0<=lidar_reply_capture[127:96];", "held_reply_h0<=lidar_reply_capture[31:0];"),
    )
    for old, new in replacements:
        assert old in source, ("mutation fixture no longer matches", old)
        mutated = source.replace(old, new, 1)
        try:
            rom_source_checks(mutated)
        except AssertionError:
            pass
        else:
            raise AssertionError(("faulty ROM emitter escaped the source contract", old))


def main():
    source = (ROOT / "src/robot_status_text.v").read_text(encoding="utf-8-sig")
    words = rom_source_checks(source)
    check_mutations(source)
    frames, accepted, stalls, transitions, resets = check_events(words)
    print(f"PASS robot ROM emitter: {frames} complete frames, {accepted} joined bytes, "
          f"{stalls} directed stalls, {transitions} state transitions, {resets} reset cuts.")
    print("PASS legacy128 literals/45 prefixes plus two-wheel decimal4 PID/diagnostic hex rows; "
          "synchronous byte0/15, ten/eight-slot schedules, atomic flags and optional parallel output.")
    print("No HDL compilation, simulation, programming or resource/timing claim.")


if __name__ == "__main__":
    main()

