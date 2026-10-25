"""Independent C1 protocol/number checks; not HDL compilation or simulation.

Runs no synthesis, simulator or programmer. Checks documented wire encodings,
UART sampling math, conservative sector rotation and byte-stream resynchronism.
"""
from pathlib import Path
from math import ceil
import random
import re

ROOT = Path(__file__).resolve().parents[1]


def encode(angle, distance_q2, quality=25, start=False):
    return bytes(((quality << 2) | (1 if start else 2),
                  ((angle & 127) << 1) | 1, angle >> 7,
                  distance_q2 & 255, distance_q2 >> 8))


def decode(packet):
    a, b, c, d, e = packet
    angle = (c << 7) | (b >> 1)
    structural = bool((a & 1) ^ ((a >> 1) & 1)) and bool(b & 1) and angle < 23040
    return structural, angle, (e << 8) | d, a >> 2, bool(a & 1)


def parse_reference(stream):
    """SDK normal-node policy, plus the documented [0,360) angle bound.

    Source: Slamtec/rplidar_sdk sdk/src/dataunpacker/unpacker/
    handler_normalnode.cpp, onData(). No inter-byte deadline is in this policy.
    """
    window, result = bytearray(), []
    for value in stream:
        if not window and not ((value ^ (value >> 1)) & 1):
            continue
        if len(window) == 1 and not (value & 1):
            window.clear()
            continue
        window.append(value)
        if len(window) == 5:
            good, angle, q2, quality, start = decode(window)
            if good:
                result.append((angle, q2, quality, start))
            window.clear()
    return result


def old_timed_reference(events):
    """Old extra framing restrictions; byte events are (microseconds, byte).

    A gap of >=2000 us necessarily crossed two old 1 ms ticks, clearing both
    partial-node storage and the previously-seen candidate. This independent
    event model reproduces RX growing while NODE and PERR both stay zero.
    """
    window, previous, result, errors, last_time = bytearray(), None, [], 0, None
    for timestamp, value in events:
        if last_time is not None and timestamp - last_time >= 2000:
            window.clear()
            previous = None
        last_time = timestamp
        window.append(value)
        if len(window) < 5:
            continue
        good, angle, q2, quality, start = decode(window)
        if good and (previous is None or start or previous <= angle <= previous + 5760):
            if previous is not None:
                result.append((angle, q2, quality, start))
            previous = angle
            window.clear()
        else:
            errors += 1
            previous = None
            del window[0]
    return result, errors


def timed_events(records, gaps, byte_gap=22):
    """Generate a legal byte stream without assuming uniform packet pacing."""
    timestamp = 0
    events = []
    for index, record in enumerate(records):
        timestamp += gaps[index % len(gaps)]
        for value in encode(*record[:2], quality=record[2], start=record[3]):
            events.append((timestamp, value))
            timestamp += byte_gap
    return events


def descriptor_matches(stream, health):
    """Find exact descriptor ends, allowing garbage and repeated A5 prefixes."""
    pattern = bytes.fromhex('A5 5A 03 00 00 00 06' if health else 'A5 5A 05 00 00 40 81')
    matched, ends = 0, []
    for index, value in enumerate(stream):
        if value == pattern[matched]:
            if matched == 6:
                ends.append(index)
                matched = 0
            else:
                matched += 1
        else:
            matched = 1 if value == 0xA5 else 0
    return ends


def motor_request(rpm):
    packet = bytearray((0xA5, 0xA8, 2, rpm & 255, rpm >> 8))
    checksum = 0
    for value in packet:
        checksum ^= value
    return bytes(packet + bytes((checksum,)))


def stop_reference(arrivals):
    """Independent command/receive-event policy, not an RTL simulation.

    Send STOP, settle100ms, request0RPM, observe100ms. A final50ms without a
    received byte confirms only RX quiet. Three continuous-data attempts fail.
    """
    now, requests = 0, []
    for _ in range(3):
        requests.append((now, bytes.fromhex('A5 25')))
        now += 44 + 100_000
        requests.append((now, motor_request(0)))
        now += 132 + 100_000
        if not any(now - 50_000 <= timestamp <= now for timestamp in arrivals):
            return requests, 0
    return requests, 0x15


def frozen_response_reference(attempts):
    """First attempt, at most8bytes; a later retry never completes old capture."""
    first = bytes(next(iter(attempts), b''))[:8]
    return first.rjust(8, b'\x00')


def cold_start_reference(first_node_ms, initial_budget_ms, *, attempts=3):
    """Independent command/receive-event counterexample, not RTL simulation.

    The peer acknowledges SCAN immediately but must accelerate on every SCAN.
    A timeout leaves the host in a1s retry wait, during which valid peer bytes
    are still counted by RX but are not processed as measurements. STOP then
    restarts the peer's acceleration. Use a5k-node/s peer (25bytes/ms).
    """
    rx, nodes = 0, 0
    for index in range(attempts):
        rx += 7  # One valid Standard descriptor for each SCAN request.
        if first_node_ms <= initial_budget_ms:
            nodes += 1
            return dict(rx=rx+5, nodes=nodes, retries=index, ready=True)
        rx += max(0, initial_budget_ms+1000-first_node_ms)*25
    return dict(rx=rx, nodes=nodes, retries=attempts, ready=False)


def established_link_reference(events_ms, *, stop_ms=None, observe_ms=10_000):
    """First-node8s deadline, then500ms stale/2s retry; STOP always wins.

    Events are valid decoded nodes, including legal no-return/quality0 nodes.
    Metadata validity is deliberately independent of usable obstacle returns.
    """
    nodes = sorted(events_ms)
    seen, last, ready, retry = False, None, False, False
    accepted = 0
    for now in range(observe_ms+1):
        if stop_ms is not None and now >= stop_ms:
            return dict(seen=False, ready=False, retry=False, nodes=accepted, stopped=True)
        if now in nodes:
            seen, last, ready = True, now, True
            accepted += 1
        elif not seen:
            retry = now >= 8000
        else:
            ready = now-last < 500
            retry = now-last >= 2000
        if retry:
            break
    return dict(seen=seen, ready=ready, retry=retry, nodes=accepted, stopped=False)


def decode_requests(wire):
    """Peer-side framing from protocolp6; incomplete payloads wait for bytes."""
    index, decoded = 0, []
    while index + 1 < len(wire):
        if wire[index] != 0xA5:
            index += 1
            continue
        command = wire[index + 1]
        if command & 0x80:
            if index + 2 >= len(wire):
                break
            size = wire[index + 2] + 4
            if index + size > len(wire):
                break
            checksum = 0
            for value in wire[index:index + size]:
                checksum ^= value
            if checksum == 0:
                decoded.append(command)
            index += size
        else:
            decoded.append(command)
            index += 2
    return decoded


def sector(raw, forward):
    angle = (raw + 23040 - forward) % 23040
    if angle < 2880 or angle >= 20160:
        return 0
    if angle < 8640:
        return 1
    if angle < 14400:
        return 3
    return 2


def main():
    source = (ROOT / 'src/rplidar_c1.v').read_text(encoding='utf-8')
    assert source.count('module ') == 5 and source.count('endmodule') == 5
    assert 'candidate_q2>=16\'d200' in source
    assert 'candidate_q2<=16\'d48000' in source
    assert 'forward_angle_q6' in source
    assert 'parameter BAUD = 460800' in source
    assert 'rx_frame_error' in source
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    for required in (
            'wire [39:0] candidate={rx_byte,node_window};',
            'wire node_commit=state==STREAM && rx_valid && !rx_frame_error',
            '&& node_fill==4 && candidate_ok;',
            'case(node_fill)',
            'if(rx_byte[0]^rx_byte[1]) begin',
            'if(rx_byte[0]) begin',
            'node_window[7:0]<=rx_byte;node_fill<=1;',
            'node_window[15:8]<=rx_byte;node_fill<=2;',
            'node_window[23:16]<=rx_byte;node_fill<=3;',
            'node_window[31:24]<=rx_byte;node_fill<=4;',
            'assign debug_status={state,node_fill,health_status,error_code,last_rx_word[7:0]};',
            'last_rx_word<={last_rx_word[23:0],rx_byte};',
            'last_tx_word<={last_tx_word[23:0],tx_byte};',
            'output reg [127:0] reply_capture,',
            'output reg [31:0] last_tx_word,',
            'if(tx_start && !tx_busy)',
            'pending_control<=2;running<=0;ready<=0;',
            'pending_control<=1;stopping<=1;',
            'pending_control!=0 && !command_in_flight',
            'if(command==8\'hA8)begin extra_index<=0;state<=TX_EXTRA;end',
            'tx_byte<=motor_payload_byte(extra_index,motor_rpm);',
            'if(extra_index==3)state<=after_command;',
            'command<=8\'hA8;motor_rpm<=16\'d600;',
            'command<=8\'hA8;motor_rpm<=0;after_command<=STOP_VERIFY;',
            'if(stream_age_ms>=50 && !rx_valid)',
            'else if(stop_attempts<2)',
            'else begin state<=DISABLED;error_code<=8\'h15;end',
            'if(error_code!=8\'h15)error_code<=0;',
            'reply_capture[127:64]<={reply_capture[119:64],rx_byte};',
            'reply_capture[63:0]<={reply_capture[55:0],rx_byte};',
            'after_command==HEALTH_DESC && !health_capture_started',
            'after_command==SCAN_DESC && !scan_capture_started',
            "state<=HEALTH_DATA; health_index<=0;error_code<=8'h03;",
            "node_window<=0; error_code<=8'h05;",
            "state<=TX_PREFIX;error_code<=8'h04;",
            "if (stream_age_ms>=500)begin ready<=0;error_code<=8'h13;end",
            'stream_age_ms<=0; ready<=1;error_code<=0;',
            "localparam [12:0] FIRST_NODE_WAIT_MS=13'd8000;",
            'reg [12:0] wait_ms;', 'reg stream_seen_node;',
            'stream_seen_node<=0;wait_ms<=FIRST_NODE_WAIT_MS;',
            'if (!stream_seen_node) begin',
            'if(wait_ms==0 && !node_commit)begin',
            'end else if (ms_tick) begin',
            'stream_seen_node<=1;',
            "command<=8'h52; after_command<=HEALTH_DESC;",
            "command<=8'h20;after_command<=SCAN_DESC;",
            "command<=8'h25; after_command<=STOP_WAIT;",
            "command<=8'h40;after_command<=START_WAIT;",
            "tx_byte<=8'hA5; tx_start<=1; state<=PREFIX_BUSY;",
            'tx_byte<=command; tx_start<=1; state<=COMMAND_BUSY;',
            'wait_ms<=100;', 'wait_ms<=10;', 'wait_ms<=1000;',
    ):
        assert required in code, required
    for removed in ('receive_gap_expired', 'byte_gap_ms', 'candidate_seen', 'candidate_progress'):
        assert removed not in code, removed
    assert re.findall(r'\bbegin\b', code).__len__() == re.findall(r'\bend\b', code).__len__()
    # Common no-payload commands retain exactly2bytes; A8 alone adds4bytes.
    assert len(re.findall(r'\btx_byte\s*<=', code)) == 4  # reset + A5 + opcode + A8extra

    # Known Standard header/node encodings, including zero-distance returns.
    assert bytes.fromhex('A5 5A 05 00 00 40 81')[2:] == bytes((5, 0, 0, 64, 129))
    assert bytes.fromhex('A5 5A 03 00 00 00 06')[2:] == bytes((3, 0, 0, 0, 6))
    for command in (0x20, 0x25, 0x40, 0x52):
        assert len(bytes((0xA5, command))) == 2
    assert motor_request(0).hex() == 'a5a80200000f'
    assert motor_request(600).hex() == 'a5a802580255'
    for rpm in range(65536):
        packet = motor_request(rpm)
        checksum = 0
        for value in packet:
            checksum ^= value
        assert len(packet) == 6 and checksum == 0
        assert int.from_bytes(packet[3:5], 'little') == rpm
    # A peer protocol decoder recovers the queued STOP at every old-command
    # boundary; interrupting an A8 immediately after its length loses BOTH the
    # following STOP and RPM0, proving why control must finish the old packet.
    for old_packet in (b'\xa5\x20', b'\xa5\x52', motor_request(600), motor_request(0)):
        for boundary in range(len(old_packet) + 1):
            accepted = old_packet[:boundary]
            queued = old_packet[boundary:] + b'\xa5\x25' + motor_request(0)
            assert decode_requests(accepted + queued) == [old_packet[1], 0x25, 0xA8]
    aborted = motor_request(600)[:3] + b'\xa5\x25' + motor_request(0)
    assert decode_requests(aborted) == []
    # Quiet verification and a peer ignoring STOP must have different outcomes.
    quiet_requests, quiet_status = stop_reference(range(0, 10_000, 200))
    assert quiet_status == 0 and len(quiet_requests) == 2
    active_requests, active_status = stop_reference(range(0, 1_000_000, 200))
    assert active_status == 0x15 and len(active_requests) == 6
    assert [packet for _, packet in active_requests] == [b'\xa5\x25', motor_request(0)] * 3
    assert all(b - a >= 100_000 for (a, _), (b, _) in zip(active_requests, active_requests[1:]))
    delayed_requests, delayed_status = stop_reference(range(0, 300_000, 200))
    assert delayed_status == 0 and len(delayed_requests) == 4
    # Preserve the first answer, even a short/incorrect one, across auto-retry.
    health_wire = bytes.fromhex('A5 5A 03 00 00 00 06 00 00 00')
    scan_wire = bytes.fromhex('A5 5A 05 00 00 40 81') + encode(0, 4000)
    assert frozen_response_reference([health_wire, b'wrong']) == health_wire[:8]
    assert frozen_response_reference([b'\x12\x34', health_wire]) == b'\0' * 6 + b'\x12\x34'
    assert frozen_response_reference([scan_wire]) == scan_wire[:8]
    # Actual2026-10-24 photograph: S0=00A55A05,S1=00004081.
    # The leading00 can be left padding of a7-byte capture, not a received
    # malformed byte. In either interpretation the full valid header is here.
    photo_scan = bytes.fromhex('00 A5 5A 05 00 00 40 81')
    assert descriptor_matches(photo_scan, False) == [7]
    assert frozen_response_reference([scan_wire[:7]]) == photo_scan
    # A peer starting after2s can produce RX>0/NODE0 forever if each retry
    # sends STOP. The new initial-only grace accepts the very same bytes.
    for delay in (2100, 2300, 3000, 7900):
        old = cold_start_reference(delay, 2000)
        new = cold_start_reference(delay, 8000)
        assert old['nodes'] == 0 and old['retries'] == 3
        assert new['nodes'] == 1 and new['ready'] and new['retries'] == 0
    old = cold_start_reference(2300, 2000, attempts=130)
    assert old['rx'] > 2_000_000 and old['nodes'] == 0
    assert established_link_reference([], observe_ms=7999)['retry'] is False
    assert established_link_reference([], observe_ms=8000)['retry'] is True
    for delay in (2100, 3000, 7900, 8000):
        at_node = established_link_reference([delay], observe_ms=delay)
        assert at_node['nodes'] == 1 and at_node['ready'] and not at_node['retry']
        assert established_link_reference([delay], observe_ms=delay+499)['ready']
        assert not established_link_reference([delay], observe_ms=delay+500)['ready']
        assert established_link_reference([delay], observe_ms=delay+2000)['retry']
    for stop_ms in (0, 1, 499, 1999, 2000, 2100, 3000, 7900, 7999, 8000):
        result = established_link_reference([7900], stop_ms=stop_ms)
        assert result['stopped'] and not result['ready'] and not result['seen'] and not result['retry']
    # Exercise the offline investigator independently of itsCLI. No serial
    # import/device open exists in this helper; use the real screenshot words.
    import runpy
    offline = runpy.run_path(str(ROOT / 'tools/inspect_c1_capture.py'))
    health_info = offline['inspect'](bytes.fromhex('A55A030000000600'), lcd_window=True)
    scan_info = offline['inspect'](photo_scan, lcd_window=True)
    assert health_info['health_status'] == 0
    assert scan_info['descriptors'][0]['kind'] == 'STANDARD_SCAN'
    assert scan_info['after_scan_descriptor']['nodes'] == 0
    assert 'padding_ambiguity' in scan_info
    assert offline['status_decode'](0x81)['state'] == 'DISABLED'
    captured = offline['inspect'](scan_wire + encode(90*64, 0, 0))
    assert captured['after_scan_descriptor']['nodes'] == 2
    assert captured['after_scan_descriptor']['usable_hits'] == 1
    rng = random.Random(20261025)
    for health in (False, True):
        pattern = bytes.fromhex('A5 5A 03 00 00 00 06' if health else 'A5 5A 05 00 00 40 81')
        for _ in range(256):
            wire = rng.randbytes(64) + b'\xa5' + pattern + rng.randbytes(64) + pattern
            expected = [index+6 for index in range(len(wire)-6) if wire[index:index+7] == pattern]
            assert descriptor_matches(wire, health) == expected
    assert encode(90 * 64, 1000 * 4, 15, True).hex() == '3d012da00f'
    for angle in range(23040):
        packet = encode(angle, 4321, (angle % 63) + 1, bool(angle == 0))
        good, a, d, q, s = decode(packet)
        assert good and (a, d, q, s) == (angle, 4321, (angle % 63) + 1, angle == 0)
        broken = bytearray(packet)
        broken[0] ^= 2
        assert not decode(broken)[0]
        broken = bytearray(packet)
        broken[1] &= 254
        assert not decode(broken)[0]
    for q2 in range(65536):
        good, _, actual_q2, _, _ = decode(encode(12345, q2))
        assert good and actual_q2 == q2
        usable = 200 <= q2 <= 48000
        if usable:
            assert 50 <= q2 // 4 <= 12000

    # Steady scan, then deletion/insertion: a long intact tail must recover.
    records = [(i * 23040 // 500, 4000 + i * 3, 25, i == 0)
               for _ in range(3) for i in range(500)]
    wire = b''.join(encode(*r[:2], quality=r[2], start=r[3]) for r in records)
    assert parse_reference(wire) == records
    damaged = wire[:627] + wire[628:1661] + b'\xff\xa5\x00' + wire[1661:]
    assert parse_reference(damaged)[-500:] == records[-500:]
    # Standard SCAN has no CRC and no mandated 1 ms receive deadline. The old
    # added timeout/two-node condition lost every node if each complete packet
    # was separated by a >=2 ms interval; no invalid candidate was ever seen,
    # so the observable counters were RX>0, NODE=0, PERR=0.
    paced = timed_events(records[:500], [2500])
    old_nodes, old_errors = old_timed_reference(paced)
    assert len(paced) == 2500 and not old_nodes and old_errors == 0
    assert parse_reference(value for _, value in paced) == records[:500]
    timed_case_count = 0
    for gaps in ([0], [100], [900], [1100], [1999], [2000], [2500],
                 [0, 22, 4000, 100, 1100, 3000], [1, 5000, 0, 2000]):
        for byte_gap in (22, 100, 800, 2500):
            paced = timed_events(records[:500], gaps, byte_gap=byte_gap)
            assert parse_reference(value for _, value in paced) == records[:500]
            timed_case_count += 1
    # A pause may also occur inside a node; the partial packet must survive it.
    tail = [(angle, 4000, 25, False) for angle in (100, 200, 300, 400, 500)]
    wire = b''.join(encode(a, d, q, s) for a, d, q, s in tail)
    assert tail[1] in parse_reference(wire)
    assert parse_reference(wire) == tail
    # Nonmonotonic angles are valid wire records; ordering is not a packet CRC.
    reordered = [records[i] for i in (7, 3, 499, 10, 2, 0)]
    assert parse_reference(b''.join(encode(*r[:2], quality=r[2], start=r[3])
                                   for r in reordered)) == reordered
    # Zero range/quality still reports a decoded node, but never a usable hit.
    assert parse_reference(encode(12, 0, 0)) == [(12, 0, 0, False)]

    # NCO samples a UART bit at three consecutive 16x ticks around its center.
    # Include 2-3 clk input synchronization latency and +/-2% peer baud error.
    clock, baud = 50_000_000, 460800
    worst_margin = 1.0
    for peer_scale in (0.98, 1.0, 1.02):
        for latency in (2, 3):
            for bit in range(9):  # eight data bits and the stop bit
                for tick in (23 + bit * 16, 24 + bit * 16, 25 + bit * 16):
                    clocks = latency + ceil(tick * clock / (baud * 16))
                    position = clocks * baud * peer_scale / clock
                    lo, hi = bit + 1, bit + 2
                    assert lo < position < hi, (peer_scale, bit, position)
                    worst_margin = min(worst_margin, position - lo, hi - position)
    # 16x tick intervals at 50 MHz alternate 6/7 clk; TX bits 108/109 clk.
    rx_ticks = [ceil(i * clock / (baud * 16)) for i in range(1, 10001)]
    tx_ticks = [ceil(i * clock / baud) for i in range(1, 10001)]
    assert {b-a for a,b in zip(rx_ticks, rx_ticks[1:])} == {6, 7}
    assert {b-a for a,b in zip(tx_ticks, tx_ticks[1:])} == {108, 109}

    # All degree orientations preserve the four 90-degree sectors.
    for forward in range(0, 23040, 64):
        for relative, expected in ((0, 0), (2879, 0), (2880, 1), (8639, 1),
                                   (8640, 3), (14399, 3), (14400, 2),
                                   (20159, 2), (20160, 0), (23039, 0)):
            assert sector((relative + forward) % 23040, forward) == expected
    print('PASS: 23040 angle encodings, 65536 distances, SDK byte framing/recovery, '
          '512 noisy descriptor sequences, startup diagnostics,65536RPMchecksums.')
    print('PASS: bounded STOP/RPM0/quiet-vs-unconfirmed events, atomic packet control, '
          'and frozen first8-byte answers across retry.')
    print('PASS: actual photograph descriptor; delayed first-node2.1/3/7.9s, '
          'finite8s initial grace, established500ms/2s deadlines and STOP priority.')
    print(f'PASS: {timed_case_count} timed-byte pacing cases; old RX=2500/NODE=0/PERR=0 '
          'counterexample now yields all 500 legal nodes.')
    print('PASS: 360 forward orientations and fractional UART sampling arithmetic.')
    print(f'Minimum +/-2% baud sampling margin: {worst_margin:.3f} bit.')
    print('No HDL compiler, simulator, programmer or hardware was used.')


if __name__ == '__main__':
    main()
