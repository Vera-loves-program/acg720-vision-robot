#!/usr/bin/env python3
"""Independent LCD byte-bank ownership/arithmetic checks; no HDL execution.

The transaction model uses distinct 50/45 MHz event phases and old-state
request/ACK observations. It is not an HDL simulator and cannot establish
synthesis inference, physical timing closure or hardware behavior.
"""
from dataclasses import dataclass, replace
from pathlib import Path
import random
import re
import unittest

from check_motor_integration import uncomment, check_blocks, instances

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Source:
    bank: int = 1
    pending: bool = False
    opened: bool = False
    complete: bool = False
    expected: int = 0
    request: int = 0
    published_bank: int = 0
    meta: int = 0
    ack_sync: int = 0


@dataclass
class Pixel:
    request_sync: int = 0
    ack: int = 0
    active: int = 0
    selected: int = 0
    seen: bool = False
    meta: int = 0


class SnapshotModel:
    """Byte transactions and physical-bank ownership, independently modeled."""

    def __init__(self):
        self.ram = [[None] * 256 for _ in range(2)]
        self.q = [None, None]
        self.writes = self.reads = self.discarded = 0
        self.publications = []
        self.display_commits = []
        self.reset()

    def reset(self):
        # RAM and RAM Q do not reset; only the common ownership epoch does.
        self.s, self.p = Source(), Pixel()

    @property
    def ready(self):
        return not self.s.pending and not self.s.complete

    def visible_byte(self, run=True):
        return self.q[self.p.selected] if run and self.p.seen else 32

    def step(self, *, write=False, read=False, valid=False, index=0, data=0,
             commit=False, meta=0, run=True, restart=False, read_en=False,
             read_index=0):
        s, p = replace(self.s), replace(self.p)
        ns, np = replace(s), replace(p)
        accepted = write and valid and self.ready
        ordered = index == 0 or (s.opened and index == s.expected)
        writing = accepted and ordered
        reading = read and run and p.seen and read_en
        if writing:
            # A stronger invariant than avoiding one identical address:
            # the source may never write ANY byte in the active bank.
            assert not p.seen or s.bank != p.active
        assert not (writing and reading and s.bank == p.active)
        old_read_value = self.ram[p.active][read_index] if reading else None

        published = displayed = False
        if write:
            ns.ack_sync = ((s.ack_sync << 1) | p.ack) & 7
            if s.pending and s.request == (s.ack_sync >> 2):
                ns.pending = False
                ns.bank ^= 1
            if accepted:
                if index == 0:
                    ns.opened, ns.expected = True, 1
                elif s.opened and index == s.expected:
                    if index == 255:
                        ns.opened, ns.complete = False, True
                    else:
                        ns.expected += 1
                else:
                    ns.opened = False
                    self.discarded += 1
            if commit and s.complete and not s.pending:
                ns.published_bank, ns.meta = s.bank, meta
                ns.request ^= 1
                ns.pending, ns.complete = True, False
                assert all(value is not None for value in self.ram[s.bank])
                self.publications.append((ns.request, s.bank, meta,
                                          bytes(self.ram[s.bank])))
                published = True
        if read:
            np.request_sync = ((p.request_sync << 1) | s.request) & 7
            if run and restart and (p.request_sync >> 2) != p.ack:
                np.active, np.meta = s.published_bank, s.meta
                np.seen, np.ack = True, p.request_sync >> 2
                expected = self.publications[-1]
                assert expected[:3] == (np.ack, np.active, np.meta)
                assert bytes(self.ram[np.active]) == expected[3]
                self.display_commits.append(expected)
                displayed = True
            if reading:
                np.selected = p.active
                self.q[p.active] = old_read_value
                self.reads += 1
        if writing:
            self.ram[s.bank][index] = data
            self.writes += 1
        self.s, self.p = ns, np
        return accepted, published, displayed


def payload(epoch):
    return bytes(((epoch * 53) ^ (index * 17) ^ (index >> 2)) & 255
                 for index in range(256))


def emit(model, epoch):
    data = payload(epoch)
    for index, value in enumerate(data):
        assert model.ready
        assert model.step(write=True, valid=True, index=index, data=value)[0]
    assert model.s.complete and not model.s.pending
    assert not model.ready
    assert model.step(write=True, commit=True, meta=epoch)[1]
    return data


def activate(model):
    for _ in range(3):
        model.step(read=True)
    assert model.step(read=True, restart=True)[2]
    for _ in range(4):
        model.step(write=True)
    assert model.ready


class SnapshotTests(unittest.TestCase):
    def test_complete_commit_metadata_and_all_addresses(self):
        model = SnapshotModel()
        self.assertEqual(model.visible_byte(), 32)
        expected = emit(model, 0x1234)
        self.assertFalse(model.p.seen)
        activate(model)
        self.assertEqual(model.p.meta, 0x1234)
        for index, value in enumerate(expected):
            model.step(read=True, read_en=True, read_index=index)
            self.assertEqual(model.visible_byte(), value)
        self.assertEqual(model.writes, 256)

    def test_last_synchronizer_edge_uses_old_request(self):
        model = SnapshotModel()
        emit(model, 1)
        # Three read edges make request_sync[2] visible AFTER edge three.
        # A frame restart on exactly that third edge must still wait.
        for edge in range(3):
            self.assertFalse(model.step(read=True, restart=edge == 2)[2])
        self.assertFalse(model.p.seen)
        self.assertEqual(model.p.request_sync >> 2, 1)
        self.assertFalse(model.step(read=True)[2])
        self.assertTrue(model.step(read=True, restart=True)[2])

    def test_byte255_alone_or_premature_commit_never_publishes(self):
        model = SnapshotModel()
        for index in (255, 12, 1, 200):
            accepted, published, _ = model.step(write=True, valid=True,
                index=index, data=19, commit=True, meta=17)
            self.assertTrue(accepted)
            self.assertFalse(published)
        for index in range(256):
            result = model.step(write=True, valid=True, index=index,
                                data=index, commit=index == 255, meta=88)
            self.assertFalse(result[1])
        self.assertEqual(model.publications, [])
        self.assertTrue(model.s.complete)
        self.assertTrue(model.step(write=True, commit=True, meta=89)[1])
        activate(model)
        self.assertEqual(model.p.meta, 89)
        self.assertEqual(model.display_commits[-1][3], bytes(range(256)))

    def test_missing_duplicate_out_of_order_abort_without_writes(self):
        model = SnapshotModel()
        for index in (8, 9, 255):
            model.step(write=True, valid=True, index=index, data=17)
        self.assertEqual(model.writes, 0)
        for index in (0, 1, 1, 2, 255):
            model.step(write=True, valid=True, index=index, data=18)
        self.assertEqual(model.writes, 2)
        self.assertFalse(model.s.opened)
        self.assertFalse(model.step(write=True, commit=True, meta=2)[1])
        expected = emit(model, 3)
        activate(model)
        self.assertEqual(model.display_commits[-1][3], expected)

    def test_byte0_restarts_unpublished_prefix(self):
        model = SnapshotModel()
        for index in range(71):
            model.step(write=True, valid=True, index=index, data=42)
        expected = emit(model, 4)
        activate(model)
        self.assertEqual(model.display_commits[-1][3], expected)

    def test_pending_bank_frozen_until_frame_boundary_ack_returns(self):
        model = SnapshotModel()
        first = emit(model, 1)
        activate(model)
        second = emit(model, 2)
        before = [row[:] for row in model.ram]
        for index in range(1000):
            accepted, _, displayed = model.step(write=True, read=True,
                valid=True, index=0, data=0, read_en=True, read_index=index & 255)
            self.assertFalse(accepted or displayed)
            self.assertEqual(model.visible_byte(), first[index & 255])
        self.assertEqual(model.ram, before)
        self.assertTrue(model.step(read=True, restart=True)[2])
        for _ in range(3):
            self.assertFalse(model.ready)
            model.step(write=True)
        self.assertFalse(model.ready)
        model.step(write=True)
        self.assertTrue(model.ready)
        self.assertEqual(model.display_commits[-1][3], second)

    def test_read_run_pause_preserves_ownership_and_pending_frame(self):
        model = SnapshotModel()
        emit(model, 6)
        activate(model)
        emit(model, 7)
        before = replace(model.p)
        for _ in range(300):
            model.step(write=True, read=True, run=False, restart=True, read_en=True)
            self.assertEqual(model.visible_byte(False), 32)
        self.assertEqual((model.p.active, model.p.ack, model.p.seen, model.p.meta),
                         (before.active, before.ack, before.seen, before.meta))
        self.assertFalse(model.ready)
        self.assertTrue(model.step(read=True, run=True, restart=True)[2])
        self.assertEqual(model.p.meta, 7)

    def test_common_reset_mid_frame_consumes_tail_then_recovers(self):
        model = SnapshotModel()
        emit(model, 8)
        activate(model)
        for index in range(93):
            model.step(write=True, valid=True, index=index, data=5)
        ram_before = [row[:] for row in model.ram]
        model.reset()
        for index in range(93, 256):
            self.assertTrue(model.ready)
            self.assertTrue(model.step(write=True, valid=True, index=index, data=6)[0])
        self.assertFalse(model.step(write=True, commit=True, meta=99)[1])
        self.assertEqual(model.ram, ram_before)
        self.assertEqual(model.visible_byte(), 32)
        emit(model, 10)
        activate(model)
        self.assertEqual(model.p.meta, 10)

    def test_bank_select_q_stays_paired_with_synchronous_ram_q(self):
        model = SnapshotModel()
        old, new = emit(model, 1), None
        activate(model)
        model.step(read=True, read_en=True, read_index=61)
        old_q = model.visible_byte()
        new = emit(model, 2)
        for _ in range(3):
            model.step(read=True)
        model.step(read=True, restart=True)
        # A bank swap is a blanking event, not an additional read. The Q
        # selector must retain its old bank until the first new read edge.
        self.assertEqual(model.visible_byte(), old_q)
        model.step(read=True, read_en=True, read_index=61)
        self.assertEqual(model.visible_byte(), new[61])
        self.assertNotEqual(old[61], new[61])

    def test_two_sink_atomic_join_random_read_priority_and_clock_phases(self):
        rng = random.Random(20261024)
        models = [SnapshotModel(), SnapshotModel()]
        index, epoch, need_commit = [0, 0], [1, 1001], [False, False]
        rr = 0
        uart_accepts, lcd_accepts = [[], []], [[], []]
        events = 0
        for tick in range(160_000):
            write_edge, read_edge = tick % 9 == 0, tick % 10 == 0
            if not (write_edge or read_edge):
                continue
            # Common ownership reset while a producer keeps streaming.
            if tick in (27_000, 90_000):
                for model in models:
                    model.reset()
            can = [model.ready for model in models]
            valid = [not need_commit[k] for k in range(2)]
            uart_valid = [valid[k] and can[k] for k in range(2)]
            read_priority = rng.randrange(7) == 0
            grants = [False, False]
            if write_edge and not read_priority:
                winner = rr if uart_valid[rr] else 1 - rr
                if uart_valid[winner]:
                    grants[winner], rr = True, 1 - winner
            for k, model in enumerate(models):
                ready = grants[k] and can[k]
                joined_valid = valid[k] and grants[k]
                current_index, current_epoch = index[k], epoch[k]
                current_data = payload(current_epoch)[current_index]
                did_commit = write_edge and need_commit[k]
                result = model.step(write=write_edge, read=read_edge,
                    valid=joined_valid, index=current_index, data=current_data,
                    commit=did_commit, meta=current_epoch,
                    run=not (35_000 <= tick < 42_000),
                    restart=read_edge and (tick // 10) % 509 == 0,
                    read_en=rng.randrange(5) != 0, read_index=rng.randrange(256))
                actual = result[0]
                self.assertEqual(actual, write_edge and valid[k] and ready)
                if write_edge and valid[k] and grants[k]:
                    uart_accepts[k].append((current_epoch, current_index, current_data))
                if actual:
                    lcd_accepts[k].append((current_epoch, current_index, current_data))
                    if current_index == 255:
                        need_commit[k] = True
                    else:
                        index[k] += 1
                if did_commit:
                    need_commit[k] = False
                    index[k] = 0
                    epoch[k] += 1
                events += 1
        self.assertEqual(uart_accepts, lcd_accepts)
        for k, model in enumerate(models):
            self.assertGreater(len(model.display_commits), 10)
            self.assertGreater(model.reads, 1000)
            for _, _, meta, data in model.publications:
                self.assertEqual(data, payload(meta))
        print(f'  independent event model: {events} clock events, '
              f'{sum(map(len, uart_accepts))} atomic sink accepts, '
              f'{sum(len(m.display_commits) for m in models)} frame commits')

    def test_four_stage_byte_and_legacy_character_alignment(self):
        rng = random.Random(73024)
        for byte_mode in (False, True):
            l = a = b = rgb = None
            reference = []
            for pixel in range(1200):
                source = rng.randrange(5)
                char = rng.randrange(256)
                # Distinct bytes expose a byte-Q/fallback-selector mix-up.
                packet = (pixel, source, char, (char + 73) & 255,
                          (char + 149) & 255, rng.randrange(3) != 0)
                expected_char = (packet[3] if source == 1 else packet[4]) \
                    if byte_mode and source in (1, 2) else char
                reference.append((pixel, expected_char, packet[5]))
                new_a = None if l is None else (
                    l[0], ((l[3] if l[1] == 1 else l[4])
                    if byte_mode and l[1] in (1, 2) else l[2]), l[5])
                # L1 descriptor, A2 RAM-Q/registered selector, B3 glyph, RGB4.
                rgb, b, a, l = b, a, new_a, packet
                if pixel >= 3:
                    self.assertEqual(rgb, reference[pixel - 3])

    def test_source_contracts(self):
        source = uncomment((ROOT / 'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8-sig'))
        compact = re.sub(r'\s+', '', source)
        for module in ('lcd1024_interaction_ui', 'lcd_text_byte_snapshot',
                       'lcd_text_snapshot_bank'):
            body = re.search(r'\bmodule\s+' + module + r'\b(.*?)\bendmodule\b',
                             source, re.S)
            self.assertIsNotNone(body)
            check_blocks('module ' + module + body[1] + 'endmodule', module)
        helper = compact.split('modulelcd_text_byte_snapshot(', 1)[1].split('endmodule', 1)[0]
        ready = re.search(r'assignbyte_ready=([^;]+);', helper)[1]
        self.assertEqual(ready, 'reset_n&&!publish_pending&&!frame_complete')
        for required in (
            "wireordered_byte=byte_index==8'd0||(frame_open&&byte_index==expected_index);",
            'wirebyte_write=byte_accept&&ordered_byte;',
            "if(frame_commit&&frame_complete&&!publish_pending)begin",
            'snapshot_bank<=write_bank;snapshot_meta<=frame_meta;',
            'snapshot_request<=~snapshot_request;publish_pending<=1;',
            'ack_sync<={ack_sync[1:0],snapshot_ack};',
            'request_sync<={request_sync[1:0],snapshot_request};',
            'if(read_run&&frame_restart&&request_sync[2]!=snapshot_ack)begin',
            'active_bank<=snapshot_bank;read_meta<=snapshot_meta;',
            'read_seen<=1;snapshot_ack<=request_sync[2];',
            'if(read_run&&read_seen&&read_en)read_bank_q<=active_bank;',
            "assignread_byte=read_seen&&read_run?(read_bank_q?bank1_q:bank0_q):8'h20;",
        ):
            self.assertIn(required, helper)
        self.assertEqual(helper.count('negedgereset_n'), 2)
        self.assertNotIn('negedgeread_run', helper)
        self.assertIn('.write_enable(byte_write&&!write_bank)', helper)
        self.assertIn('.read_enable(read_run&&read_seen&&read_en&&!active_bank)', helper)
        self.assertIn('.write_enable(byte_write&&write_bank)', helper)
        self.assertIn('.read_enable(read_run&&read_seen&&read_en&&active_bank)', helper)
        bank = compact.split('modulelcd_text_snapshot_bank(', 1)[1].split('endmodule', 1)[0]
        for required in (
            '.BIT_WIDTH_0(8),.BIT_WIDTH_1(8)',
            ".WRITE_MODE0(2'b00),.WRITE_MODE1(2'b00)",
            ".READ_MODE0(1'b0),.READ_MODE1(1'b0)",
            ".ADA({3'b000,write_index,3'b000}),.ADB({3'b000,read_index,3'b000})",
            '.CEA(reset_n&&write_enable),.CEB(reset_n&&read_enable)',
            ".WREA(1'b1),.WREB(1'b0)",
            ".OCEA(1'b1),.OCEB(1'b1),.RESETA(1'b0),.RESETB(1'b0)",
            'assignread_data=read_word[7:0];',
        ):
            self.assertIn(required, bank)
        for index in range(256):
            physical = index << 3
            self.assertLess(physical, 1 << 14)
            self.assertEqual(physical & 7, 0)
            self.assertEqual(physical >> 3, index)
        self.assertIn('parameterBYTE_TEXT_MODE=0', compact)
        self.assertIn('if(!BYTE_TEXT_MODE&&frame_restart&&robot_request_sync[2]!=robot_status_ack)', compact)
        self.assertIn('if(!BYTE_TEXT_MODE&&pid_request_sync[2]!=pid_text_ack)', compact)
        self.assertIn('byte_text_read_index={robot_label_row_l,char_ix};', compact)
        self.assertIn("robot_byte_source_a<=BYTE_TEXT_MODE&&robot_control_l?(robot_label_source_l==3'd1?2'd1:robot_label_source_l==3'd2?2'd2:2'd0):2'd0;", compact)
        self.assertIn("glyph_dynamic_character_a=robot_byte_source_a==2'd1?robot_byte_character:robot_byte_source_a==2'd2?pid_byte_character:character_a;", compact)
        self.assertIn('font_b<=glyph(glyph_character_a);', compact)
        # A second register on the synchronous RAM Q would add a fifth stage.
        self.assertNotRegex(source, r'\w+\s*<=\s*(?:robot|pid)_byte_character\s*;')


if __name__ == '__main__':
    unittest.main(verbosity=2)
