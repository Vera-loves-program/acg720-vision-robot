"""Offline C1 UART/LCD evidence inspector; never opens or writes any device.

Examples:
  python tools/inspect_c1_capture.py --health A55A030000000600 \
      --scan 00A55A0500004081 --status 00000081 --tx 0200000F
  python tools/inspect_c1_capture.py --capture saved_uart_rx.bin
  python tools/inspect_c1_capture.py --hex 'A5 5A 05 00 00 40 81 3D 01 2D A0 0F'

LCD captures contain at most8bytes per phase, LEFT-padded with00 before full.
They do not prove how many bytes arrived, whether the motor is physically
rotating, or that a known descriptor is followed by measurements. Use a longer
raw capture to distinguish a delayed first sample from a lost/invalid sample.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re


STATES = ('DISABLED', 'START_WAIT', 'TX_PREFIX', 'PREFIX_BUSY', 'PREFIX_DONE',
          'TX_COMMAND', 'COMMAND_BUSY', 'COMMAND_DONE', 'STOP_WAIT',
          'HEALTH_DESC', 'HEALTH_DATA', 'SCAN_DESC', 'STREAM', 'RETRY_WAIT',
          'TX_EXTRA', 'EXTRA_BUSY', 'EXTRA_DONE', 'STOP_VERIFY',
          'MOTOR_START_WAIT', 'STOP_MOTOR_WAIT')


def hex_bytes(value: str) -> bytes:
    compact = re.sub(r'\s|[_:,\-]', '', value)
    compact = re.sub(r'0[xX]', '', compact)
    if len(compact) % 2 or re.search(r'[^0-9a-fA-F]', compact):
        raise ValueError('Expected complete hexadecimal bytes')
    return bytes.fromhex(compact)


def word(value: str) -> int:
    data = hex_bytes(value)
    if len(data) != 4:
        raise ValueError('LCD word must contain exactly8hex digits')
    return int.from_bytes(data, 'big')


def descriptors(data: bytes) -> list[dict]:
    result = []
    for offset in range(max(0, len(data)-6)):
        if data[offset:offset+2] != b'\xa5\x5a':
            continue
        flags_length = int.from_bytes(data[offset+2:offset+6], 'little')
        size, mode, kind = flags_length & 0x3fffffff, flags_length >> 30, data[offset+6]
        label = {(3, 0, 6): 'HEALTH', (5, 1, 0x81): 'STANDARD_SCAN',
                 (20, 0, 4): 'DEVICE_INFO', (84, 1, 0x85): 'DENSE_EXPRESS'}.get((size, mode, kind), 'OTHER')
        result.append(dict(offset=offset, payload_size=size, mode=mode,
                           type_hex=f'{kind:02X}', kind=label))
    return result


def standard_nodes(data: bytes) -> dict:
    """Official normal-node sync policy with documented angle bound."""
    pending, count, usable, starts, invalid, preview = [], 0, 0, 0, 0, []
    for value in data:
        if not pending and not ((value ^ (value >> 1)) & 1):
            invalid += 1
            continue
        if len(pending) == 1 and not (value & 1):
            pending.clear()
            invalid += 1
            continue
        pending.append(value)
        if len(pending) != 5:
            continue
        first, lo, hi, d0, d1 = pending
        pending.clear()
        angle, distance_q2 = (hi << 7) | (lo >> 1), d0 | (d1 << 8)
        if angle >= 360*64:
            invalid += 1
            continue
        count += 1
        quality, start = first >> 2, bool(first & 1)
        usable += bool(quality and 200 <= distance_q2 <= 48000)
        starts += start
        if len(preview) < 4:
            preview.append(dict(angle_deg=angle/64, distance_mm=distance_q2/4,
                                quality=quality, scan_start=start))
    return dict(nodes=count, usable_hits=usable, scan_starts=starts,
                invalid_fragments=invalid, incomplete_node_bytes=len(pending), preview=preview)


def inspect(data: bytes, *, lcd_window=False) -> dict:
    found = descriptors(data)
    result = dict(bytes=len(data), hex=data[:64].hex(' ').upper(), descriptors=found)
    scans = [item for item in found if item['kind'] == 'STANDARD_SCAN']
    health = [item for item in found if item['kind'] == 'HEALTH']
    if health:
        offset = health[0]['offset']+7
        if len(data) > offset:
            result['health_status'] = data[offset]
        if len(data) >= offset+3:
            result['health_error'] = int.from_bytes(data[offset+1:offset+3], 'little')
    if scans:
        offset = scans[0]['offset']+7
        result['after_scan_descriptor'] = standard_nodes(data[offset:])
        if offset == len(data):
            result['note'] = 'Valid Standard descriptor; no following measurement byte is present in this capture.'
    if lcd_window:
        if len(data) != 8:
            raise ValueError('Each LCD phase capture must contain exactly8bytes')
        result['capture_limit'] = 'At most8phase bytes, left-padded00; byte count is not encoded.'
        if data[0] == 0 and any(item['offset'] == 1 for item in found):
            result['padding_ambiguity'] = 'Initial00 may be padding of7received bytes or a received leading00; it is not evidence of malformed framing.'
    return result


def status_decode(value: int) -> dict:
    state = value >> 27
    return dict(state=STATES[state] if state < len(STATES) else f'UNKNOWN_{state}',
                state_id=state, partial_node_bytes=(value >> 24) & 7,
                health_status=(value >> 16) & 255, error_hex=f'{(value >> 8) & 255:02X}',
                latest_rx_byte_hex=f'{value & 255:02X}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--capture', type=Path, help='Existing rawUART RX binary file')
    source.add_argument('--hex', help='RawUART RX hexadecimal bytes')
    parser.add_argument('--health', help='LCD H0+H1:16hex digits')
    parser.add_argument('--scan', help='LCD S0+S1:16hex digits')
    parser.add_argument('--status', help='LCD ST:8hex digits')
    parser.add_argument('--tx', help='LCD TX:8hex digits, last4locally transmitted bytes')
    args = parser.parse_args(argv)
    if not any((args.capture, args.hex, args.health, args.scan, args.status, args.tx)):
        parser.error('Supply an existing capture or LCD/hex evidence')
    try:
        result = dict(mode='OFFLINE ONLY; no serial port or hardware is accessed')
        if args.capture:
            result['raw'] = inspect(args.capture.read_bytes())
        elif args.hex is not None:
            result['raw'] = inspect(hex_bytes(args.hex))
        for name in ('health', 'scan'):
            if getattr(args, name) is not None:
                result[name] = inspect(hex_bytes(getattr(args, name)), lcd_window=True)
        if args.status is not None:
            result['status'] = status_decode(word(args.status))
        if args.tx is not None:
            value = word(args.tx)
            result['tx'] = dict(last4hex=f'{value:08X}',
                note='Local transmitter accepted these bytes; this alone is not an electrical acknowledgement.')
            if value == 0x0200000f:
                result['tx']['matches'] = 'Tail of A5A80200000F: requested0RPM afterSTOP'
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
