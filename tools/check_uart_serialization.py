#!/usr/bin/env python3
"""Independent byte-order / RAM-bank ownership checks; no HDL simulation.

Compares full report bytes and an abstract three-bank publication algorithm.
Does not invoke or emulate an HDL tool, synthesis, clock timing or UART pins.
"""
from pathlib import Path
import random
import re

ROOT = Path(__file__).resolve().parents[1]
MASK32 = (1 << 32) - 1
PREFIXES = (b' SEQ=', b' IMU_N=', b' US_N=', b' CMD=', b' RXERR=')
MOTOR_PREFIX = b' | PID SPD=COUNT/10MS POS=COUNT '
ROBOT_PREFIX = b' | NAV: '


class ShiftBytes:
    def __init__(self, payload):
        self.width = len(payload) * 8
        self.mask = (1 << self.width) - 1
        self.word = int.from_bytes(payload, 'big')

    def take(self):
        value = self.word >> (self.width - 8)
        self.word = (self.word << 8) & self.mask
        return bytes((value,))


def indexed_rows(payload):
    return b'|'.join(payload[k:k+16] for k in range(0, len(payload), 16))


def ram_rows(payload):
    memory = list(payload)
    result = bytearray()
    for index in range(len(payload)):
        result.append(memory[index])
        if index % 16 == 15 and index != len(payload)-1:
            result.extend(b'|')
    return bytes(result)


def signed_speed(value):
    value = value & MASK32
    negative = bool(value & (1 << 31))
    magnitude = ((~value + 1) & MASK32) if negative else value
    return (b'-' if negative else b'+') + str(magnitude).encode()


def test_report(sensor, speeds, optimized):
    result = bytearray()
    # ACTUATOR_TEST skips row0 and serializes just the six axis rows1..6.
    six_rows = sensor[16:112]
    for index in range(16, 112):
        result.extend(six_rows[index-16:index-15] if optimized else sensor[index:index+1])
        if index % 16 == 15:
            result.extend(b'\r\n')
    for wheel, value in zip((b'L', b'R'), speeds):
        result.extend(b'ENC ' + wheel + b'=' + signed_speed(value) + b' COUNT/10MS\r\n')
    return bytes(result)


def normal_report(sensor, fields, last, motor, robot, motor_seen, robot_seen, robot_mode, optimized):
    result = bytearray(indexed_rows(sensor))
    if optimized:
        packed = int.from_bytes(b''.join(v.to_bytes(4, 'big') for v in fields), 'big')
        for prefix in PREFIXES:
            value = packed >> 128
            packed = (packed << 32) & ((1 << 160) - 1)
            result.extend(prefix + str(value).encode())
        assert packed == 0
    else:
        for prefix, value in zip(PREFIXES, fields):
            result.extend(prefix + str(value).encode())
    result.extend(b' LAST=' + bytes((last or ord('-'),)))
    if motor_seen:
        result.extend(MOTOR_PREFIX)
        result.extend(ram_rows(motor) if optimized else indexed_rows(motor))
        # Preserve the existing placement: NAV follows a present PID block.
        if robot_mode and robot_seen:
            result.extend(ROBOT_PREFIX)
            result.extend(ram_rows(robot) if optimized else indexed_rows(robot))
    result.extend(b'\r\n')
    return bytes(result)


class SnapshotBanks:
    """Independent byte-handshake/frame-publication reference, not RTL execution.

    A transaction arbitrates input bytes and a UART RAM read. Producer offers
    are tuples(index,byte), held by the caller until accepted. Expected-index
    guards consume malformed bytes but never let them write protected memory.
    """
    def __init__(self, robot_mode=True):
        self.robot_mode = robot_mode
        self.ram = [None] * 2048
        self.latest = [0, 0]
        self.held = [0, 0]
        self.write_bank = [1, 1]
        self.expected = [0, 0]
        self.open = [False, False]
        self.prefix = [None, None]
        self.seen = [False, False]
        self.published = [None, None]
        self.report = [None, None]
        self.active = False
        self.round_robin = 0
        self.writes = self.reads = self.publications = self.restarts = self.aborts = 0
        self.accepted = [0, 0]

    @staticmethod
    def address(stream, bank, index):
        return (stream << 10) | (bank << 8) | index

    def verify(self):
        for stream in (0, 1):
            if self.seen[stream]:
                start = self.address(stream, self.latest[stream], 0)
                assert bytes(self.ram[start:start+256]) == self.published[stream]
            if self.active and self.report[stream] is not None:
                start = self.address(stream, self.held[stream], 0)
                assert bytes(self.ram[start:start+256]) == self.report[stream]

    def transaction(self, offers=None, begin=False, end=False, read=None, reset_n=True):
        offers = offers or {}
        assert all(s in (0, 1) and 0 <= i <= 255 and 0 <= value <= 255
                   for s, (i, value) in offers.items())
        if not reset_n:
            # RAM is not cleared: only its metadata/validity is reset.
            self.latest, self.held, self.write_bank = [0, 0], [0, 0], [1, 1]
            self.expected, self.open, self.prefix = [0, 0], [False, False], [None, None]
            self.seen, self.published, self.report = [False, False], [None, None], [None, None]
            self.active, self.round_robin = False, 0
            return dict(ready=(False, False), accepted=set(), value=None, write=None)
        old_active, old_held = self.active, self.held.copy()
        # Publication later in this transaction cannot change the bank/content
        # captured by report_begin: this report sees the preceding whole frame.
        if begin:
            assert not self.active and read is None
            self.held = self.latest.copy()
            self.report = self.published.copy()
            self.active = True
        chosen = None
        if read is None:
            pending = [stream for stream in offers if stream == 0 or self.robot_mode]
            if len(pending) == 2:
                chosen = self.round_robin
            elif pending:
                chosen = pending[0]
        ready = (chosen == 0, not self.robot_mode or chosen == 1)
        accepted = {s for s in offers if ready[s]}
        value = None
        write = None
        if read is not None:
            stream, index = read
            assert old_active and self.report[stream] is not None
            value = self.ram[self.address(stream, old_held[stream], index)]
            assert value == self.report[stream][index]
            self.reads += 1
            assert chosen is None
        if chosen is not None:
            stream = chosen
            index, byte = offers[stream]
            self.accepted[stream] += 1
            self.round_robin = 1 - stream
            in_order = index == 0 or (self.open[stream] and index == self.expected[stream])
            if not in_order:
                self.open[stream] = False
                self.expected[stream] = 0
                self.prefix[stream] = None
                self.aborts += 1
            else:
                if index == 0:
                    if self.open[stream]:
                        self.restarts += 1
                    choices = [bank for bank in range(3) if bank != self.latest[stream]
                               and (not old_active or bank != old_held[stream])]
                    assert choices
                    self.write_bank[stream] = choices[0]
                    self.open[stream] = True
                    self.expected[stream] = 1
                    self.prefix[stream] = bytearray()
                bank = self.write_bank[stream]
                assert bank != self.latest[stream]
                assert not old_active or bank != old_held[stream]
                assert not self.active or bank != self.held[stream]
                self.ram[self.address(stream, bank, index)] = byte
                self.prefix[stream].append(byte)
                write = stream, bank, index, byte
                self.writes += 1
                if index == 255:
                    assert len(self.prefix[stream]) == 256
                    self.latest[stream] = bank
                    self.published[stream] = bytes(self.prefix[stream])
                    self.seen[stream] = True
                    self.open[stream] = False
                    self.expected[stream] = 0
                    self.publications += 1
                elif index != 0:
                    self.expected[stream] += 1
        if not self.robot_mode and 1 in accepted:
            self.accepted[1] += 1  # consumed/discarded, never uses RAM/round-robin
        if end:
            assert self.active
            self.active = False
        self.verify()
        return dict(ready=ready, accepted=accepted, value=value, write=write)


def send_frames(store, payloads, read_every=0):
    """Concurrent producers retain each byte while read/other stream owns RAM."""
    index = {stream: 0 for stream in payloads}
    grants = []
    transaction = 0
    while index:
        assert transaction < 10000
        offers = {stream: (position, payloads[stream][position])
                  for stream, position in index.items()}
        read = None
        if read_every and transaction % read_every == 0:
            readable = [s for s in (0, 1) if store.active and store.report[s] is not None]
            if readable:
                read = readable[transaction % len(readable)], transaction % 256
        result = store.transaction(offers, read=read)
        if read is not None:
            assert not result['ready'][0] and (not store.robot_mode or not result['ready'][1])
            assert result['write'] is None
        for stream in result['accepted']:
            if stream in index:
                grants.append(stream)
                index[stream] += 1
                if index[stream] == 256:
                    del index[stream]
        transaction += 1
    return grants


def bank_ownership_checks(rng):
    store = SnapshotBanks()
    first = {0: bytes(range(256)), 1: bytes(reversed(range(256)))}
    grants = send_frames(store, first)
    assert grants == [0, 1] * 256  # bounded fair service when both remain valid
    assert store.seen == [True, True]
    assert store.published == [first[0], first[1]]
    store.transaction(begin=True)
    held = store.report.copy()
    for _ in range(3):
        payloads = {0: rng.randbytes(256), 1: rng.randbytes(256)}
        send_frames(store, payloads, read_every=3)
        assert store.published == [payloads[0], payloads[1]]
        assert store.report == held
    store.transaction(end=True)

    # A final-byte publication and report_begin share an edge. The report
    # must lock OLD latest, while a subsequent byte0 chooses another bank.
    payload = bytes([53]) * 256
    for index in range(255):
        store.transaction({0: (index, payload[index])})
    previous = store.published[0]
    old_latest = store.latest[0]
    result = store.transaction({0: (255, payload[255])}, begin=True)
    assert result['write'][1] != old_latest
    assert store.report[0] == previous and store.published[0] == payload
    assert store.held[0] == old_latest
    send_frames(store, {0: bytes([71]) * 256}, read_every=2)
    for index in range(256):
        assert store.transaction(read=(0, index))['value'] == previous[index]
    store.transaction(end=True)

    # Byte0 restarts an incomplete prefix exactly when a report begins.
    for index in range(80):
        store.transaction({0: (index, 17)})
    previous = store.published[0]
    result = store.transaction({0: (0, 29)}, begin=True)
    assert result['write'][1] != store.held[0]
    for index in range(1, 256):
        store.transaction({0: (index, 29)})
    assert store.published[0] == bytes([29]) * 256 and store.report[0] == previous

    # Unexpected bytes after a commit must NOT write the remembered writebank:
    # it now IS latest, and may also be report-held in the next transaction.
    before = list(store.published)
    for sequence in ((255,), (1, 255), (0, 1, 3, 2, 255), (0, 1, 2, 2, 3, 255)):
        for index in sequence:
            count = store.writes
            result = store.transaction({0: (index, 199)})
            assert result['accepted'] == {0}
            if result['write'] is None:
                assert store.writes == count
        assert store.published == before
    for index in range(123):
        store.transaction({0: (index, 41)})
    assert store.published == before
    send_frames(store, {0: bytes([97]) * 256})
    assert store.published[0] == bytes([97]) * 256
    store.transaction(end=True)

    # Seeded concurrency: inputs change only after their previous offer was
    # accepted. Rare new0 boundaries interrupt an unfinished accepted prefix.
    pending = [None, None]
    positions = [0, 0]
    previous_accepted = set()
    for event in range(30000):
        for stream in (0, 1):
            if pending[stream] is None and rng.randrange(90) == 0:
                pending[stream], positions[stream] = rng.randbytes(256), 0
            elif (pending[stream] is not None and stream in previous_accepted and
                  rng.randrange(1000) == 0):
                pending[stream], positions[stream] = rng.randbytes(256), 0
        begin = not store.active and rng.randrange(100) == 0
        end = store.active and rng.randrange(160) == 0
        read = None
        readable = [s for s in (0, 1) if store.active and store.report[s] is not None]
        if readable and rng.randrange(7) == 0:
            read = rng.choice(readable), rng.randrange(256)
        offers = {s: (positions[s], pending[s][positions[s]]) for s in (0, 1) if pending[s] is not None}
        result = store.transaction(offers, begin=begin, end=end, read=read)
        previous_accepted = result['accepted']
        for stream in previous_accepted:
            positions[stream] += 1
            if positions[stream] == 256:
                pending[stream] = None
    if store.active:
        store.transaction(end=True)
    assert store.publications > 10 and store.reads > 1000 and store.aborts >= 4

    # Inactive robot stream is drained without occupying the single RAM port.
    legacy = SnapshotBanks(robot_mode=False)
    for index in range(256):
        result = legacy.transaction({0: (index, index), 1: (index, 255 - index)})
        assert result['ready'] == (True, True) and result['accepted'] == {0, 1}
        assert result['write'][0] == 0
    assert legacy.seen == [True, False] and legacy.published[0] == bytes(range(256))
    legacy.transaction(begin=True)
    result = legacy.transaction({1: (7, 5)}, read=(0, 7))
    assert result['ready'] == (False, True) and result['accepted'] == {1}
    assert result['write'] is None and result['value'] == 7
    untouched = legacy.ram.copy()
    result = legacy.transaction({0: (0, 99), 1: (0, 99)}, reset_n=False)
    assert result['ready'] == (False, False) and not result['accepted']
    assert legacy.ram == untouched and legacy.seen == [False, False]
    result = legacy.transaction({0: (255, 99)})
    assert result['write'] is None and legacy.seen == [False, False]
    return store


def source_guards():
    source = (ROOT / 'src/sensor_demo_uart.v').read_text(encoding='utf-8')
    bare = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    # Wide shifting held PID/NAV buffers and their extra load mux are gone.
    assert not re.search(r'\b(?:held_motor|held_robot|latest_motor|latest_robot)\b', bare)
    assert not re.search(r'\b(?:motor_capture|robot_capture)\b', bare)
    assert 'held_text<={' not in bare
    required = (
        'tx_data<=held_text[text_byte_msb-:8]',
        "wire [9:0] text_byte_msb={~text_index,3'b111}",
        'held_text<=display_text',
        'TEXT_RAM_WAIT:if(text_read_valid)',
        'tx_data<=text_read_data',
        'binary_shift<=held_meta[159:128]',
        "held_meta<={held_meta[127:0],32'd0}",
        'held_last_command<=formatted_meta[7:0]',
        'reg [7:0] text_ram[0:2047]',
        'if(memory_write)text_ram[memory_address]<=memory_write_data',
        'else read_data<=text_ram[memory_address]',
        '.reset_n(reset_n && !ACTUATOR_TEST)',
        'held_motor_bank<=latest_motor_bank;held_robot_bank<=latest_robot_bank',
        'free_bank(latest_motor_bank,held_motor_bank,report_active)',
        'free_bank(latest_robot_bank,held_robot_bank,report_active)',
        'wire grant_motor=!read_request', 'wire grant_robot=!read_request',
        'BIT_COUNTER_WIDTH=BIT_CLOCKS<=1 ? 1 : $clog2(BIT_CLOCKS)',
        'parameter UART_BAUD = 115200',
        'WAIT_ACCEPT: if (tx_busy) state<=WAIT_DONE',
        'WAIT_DONE: if (!tx_busy)',
    )
    for token in required:
        assert token in bare, token
    helper = bare.split('module sensor_demo_text_snapshot_store', 1)[1].split('endmodule', 1)[0]
    assert 'DPB' not in helper and 'posedge read_clk' not in helper
    assert len(re.findall(r'always\s*@\(posedge clk\)', helper)) == 1
    assert re.findall(r'text_ram\[[^\]]+\]\s*<=', helper) == ['text_ram[memory_address]<=']
    for token in ('if(grant_motor)begin', 'if(grant_robot)begin',
                  "motor_byte_index==8'd0", "robot_byte_index==8'd0",
                  'motor_frame_open && motor_byte_index==motor_expected_index',
                  'robot_frame_open && robot_byte_index==robot_expected_index',
                  'grant_motor && motor_byte_order_ok', 'grant_robot && robot_byte_order_ok',
                  "{1'b0,motor_input_bank,motor_byte_index}",
                  "{1'b1,robot_input_bank,robot_byte_index}",
                  "motor_byte_index==8'd255", "robot_byte_index==8'd255",
                  'assign motor_byte_ready=reset_n && grant_motor',
                  'assign robot_byte_ready=reset_n && (!ROBOT_MODE || grant_robot)',
                  'motor_expected_index<=8\'d1;motor_frame_open<=1',
                  'robot_expected_index<=8\'d1;robot_frame_open<=1',
                  'motor_frame_open<=0;motor_expected_index<=0',
                  'robot_frame_open<=0;robot_expected_index<=0'):
        assert token in helper, token
    assert '[2047:0]' not in bare
    for old_port in ('motor_text_data', 'robot_text_data', 'motor_text_valid',
                     'robot_text_valid', 'motor_valid', 'robot_valid',
                     'motor_copy_index', 'robot_copy_index'):
        assert not re.search(r'\b'+old_port+r'\b', bare), old_port
    assert not re.search(r'\b(?:1023|2047)\s*-', bare)
    # No wide source bus or variable byte selector remains in this helper.
    assert '-:8' not in helper and '2047' not in helper.replace('text_ram[0:2047]', '')
    memory_write = helper.split('wire memory_write=', 1)[1].split(';', 1)[0]
    assert 'motor_byte_order_ok' in memory_write and 'robot_byte_order_ok' in memory_write
    # UART report state-machine semantics were intentionally left intact.
    assert bare.count('held_motor_seen<=motor_seen') == 1
    assert bare.count('held_robot_seen<=robot_seen') == 1


def main():
    source_guards()
    rng = random.Random(20261023)
    edges = (0, 1, 9, 10, 99, 100, 65535, (1 << 31)-1, 1 << 31, MASK32)
    fixtures = [
        (bytes(128), bytes(256), bytes(256)),
        (bytes([255])*128, bytes([255])*256, bytes([255])*256),
        (bytes(range(128)), bytes(range(256)), bytes(reversed(range(256)))),
    ]
    fixtures.extend((rng.randbytes(128), rng.randbytes(256), rng.randbytes(256))
                    for _ in range(256))
    reports = 0
    for index, (sensor, motor, robot) in enumerate(fixtures):
        fields = tuple(edges[(index+i) % len(edges)] if index < len(edges)
                       else rng.getrandbits(32) for i in range(5))
        last = (0, ord('P'), ord('!'), 255)[index % 4]
        for motor_seen in (False, True):
            for robot_seen in (False, True):
                for robot_mode in (False, True):
                    args = (sensor, fields, last, motor, robot, motor_seen, robot_seen, robot_mode)
                    assert normal_report(*args, optimized=False) == normal_report(*args, optimized=True)
                    reports += 1
        for speeds in ((0, 0), (-1, 1), (-(1 << 31), (1 << 31)-1),
                       (rng.getrandbits(32), rng.getrandbits(32))):
            assert test_report(sensor, speeds, False) == test_report(sensor, speeds, True)
            reports += 1
    # Constant width reduction cannot change the existing TX bit intervals.
    for index in range(128):
        assert ((((~index) & 127) << 3) | 7) == 1023-index*8
    bit_clocks = (50_000_000 + 115200//2) // 115200
    assert bit_clocks == 434 and (bit_clocks - 1).bit_length() == 9
    assert bit_clocks - 1 < (1 << 9)
    store = bank_ownership_checks(rng)
    print(f'PASS: {reports} complete indexed-vs-RAM byte streams, '
          'metadata edges, separators, TEST six-axis/signed-speed mode and source guards.')
    print(f'PASS: byte-handshake triple-bank model: {store.writes} writes, {store.reads} reads, '
          f'{store.publications} complete publications, {store.restarts} byte0 restarts, {store.aborts} malformed-prefix aborts; '
          'zero held-bank writes or mixed/uninitialized published bytes.')
    print('Snapshot content/order and existing 434-clock UART bits preserved by arithmetic/source checks.')
    print('No HDL compilation, simulation, programming or resource/timing claim.')


if __name__ == '__main__':
    main()
