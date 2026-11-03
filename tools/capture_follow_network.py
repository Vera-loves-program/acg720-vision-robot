"""Passive Pi Ethernet probe using only Python's standard library.

Run on the Pi with sudo; no tcpdump installation is needed. This tool never
sends UDP, writes PWM, changes network settings, or restarts the vision worker.
AF_PACKET normally omits Ethernet FCS and outgoing UDP checksums may be
offloaded, so only the application CCITT/XOR checksums are assessed here.
"""
import argparse
import binascii
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
from robot_follow import packet, read_report


def decode_ethernet(frame):
    if len(frame) < 14:
        return None
    offset = 14
    ethertype = struct.unpack_from('>H', frame, 12)[0]
    if ethertype == 0x8100 and len(frame) >= 18:
        ethertype = struct.unpack_from('>H', frame, 16)[0]
        offset = 18
    if ethertype != 0x0800 or len(frame) < offset + 20:
        return None
    ip = frame[offset:]
    ihl = (ip[0] & 15) * 4
    total = struct.unpack_from('>H', ip, 2)[0]
    # Fragmented or truncated datagrams are not interpreted as metadata.
    if ip[0] >> 4 != 4 or ihl < 20 or ip[9] != 17 or len(ip) < total or total < ihl + 8:
        return None
    if struct.unpack_from('>H', ip, 6)[0] & 0x3fff:
        return None
    src_port, dst_port, length = struct.unpack_from('>3H', ip, ihl)
    if length < 8 or ihl + length > total:
        return None
    raw = ip[ihl + 8:ihl + length]
    magic = raw[:4]
    if magic not in (b'FOL1', b'TGT1', b'VEC1', b'GOL1', b'RBT1'):
        return None
    result = dict(magic=magic.decode('ascii'), src_ip=socket.inet_ntoa(ip[12:16]),
                  dst_ip=socket.inet_ntoa(ip[16:20]), src_port=src_port, dst_port=dst_port,
                  destination_mac=frame[:6].hex(':'), payload_bytes=len(raw))
    if magic == b'RBT1':
        report = read_report(raw)
        result.update(checksum_valid=report is not None, report=report)
    else:
        valid = len(raw) == 32 and raw[4] == 1 and binascii.crc_hqx(raw[:30], 0xffff) == int.from_bytes(raw[30:], 'big')
        result.update(checksum_valid=valid)
        if len(raw) >= 8:
            result.update(flags=raw[5], sequence=int.from_bytes(raw[6:8], 'big'))
        if valid and magic == b'FOL1':
            values = struct.unpack('>h10H', raw[8:30])
            result.update(speed=values[0], pan_us=values[1], tilt_us=values[2],
                          pan_limits=list(values[3:5]), tilt_limits=list(values[5:7]),
                          target_age_ms=values[7], confidence=values[8],
                          event_ack=values[9], selected_id=values[10])
    return result


def self_test():
    def ethernet(raw, fragment=0):
        header = struct.pack('>BBHHHBBH4s4s', 0x45, 0, 28 + len(raw), 1, fragment,
                             64, 17, 0, socket.inet_aton('192.168.10.3'),
                             socket.inet_aton('192.168.10.255'))
        return bytes.fromhex('ffffffffffff02ac720000030800') + header + struct.pack('>4H', 6102, 5001, len(raw) + 8, 0) + raw
    raw = packet(b'FOL1', 24, 15130, struct.pack('>h10H', 0, 1200, 1076, 380, 2550, 645, 2420, 65535, 0, 39, 0))
    decoded = decode_ethernet(ethernet(raw))
    assert decoded['checksum_valid'] and decoded['sequence'] == 15130
    assert decoded['pan_us'] == 1200 and decoded['tilt_us'] == 1076
    assert decoded['pan_limits'] == [380, 2550] and decoded['event_ack'] == 39
    corrupt = raw[:-1] + bytes([raw[-1] ^ 1])
    assert not decode_ethernet(ethernet(corrupt))['checksum_valid']
    assert decode_ethernet(ethernet(raw, 1)) is None
    assert decode_ethernet(ethernet(raw)[:-1]) is None
    assert decode_ethernet(b'') is None
    report = b'RBT1\x01' + struct.pack('>BHBHBHiHHHhhHB', 128, 39, 9, 0, 20, 5, -42873, 0, 0, 0, 0, 0, 0, 0)
    xor = 0
    for byte in report:
        xor ^= byte
    decoded_report = decode_ethernet(ethernet(report + bytes([xor])))
    assert decoded_report['checksum_valid'] and decoded_report['report']['ack'] == 0
    assert decoded_report['report']['event'] == 39 and decoded_report['report']['yaw_mdeg'] == -42873
    print('PASS passive probe parser: field FOL/RBT, exact limits/event/ACK, CRC reject, fragment/truncation reject; no socket opened')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not 1 <= args.seconds <= 30:
        parser.error('--seconds must be1..30')
    if not hasattr(socket, 'AF_PACKET'):
        parser.error('Capture is Linux/Pi only; --self-test also works on a PC.')
    output = args.output or ROOT / 'dataset/follow_logs' / ('network_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    evidence = dict(interface=args.interface, seconds=args.seconds,
                    captured_at=datetime.now().astimezone().isoformat(),
                    notes='Passive AF_PACKET; no UDP send/PWM/network mutation. FCS not exposed; UDP checksum offload not assessed.')
    for key, command in [('addresses', ['ip', '-br', '-4', 'addr']),
                         ('route', ['ip', '-4', 'route', 'get', '192.168.10.255'])]:
        r = subprocess.run(command, capture_output=True, text=True, timeout=3)
        evidence[key] = r.stdout + r.stderr
    try:
        with urllib.request.urlopen('http://127.0.0.1:8080/state', timeout=2) as response:
            state = json.load(response)
        evidence['state_before'] = {k: state.get(k) for k in ('software', 'tracking', 'target_selected', 'tracking_preflight', 'control_error', 'error', 'fpga_follow')}
    except Exception as exc:
        evidence['state_error'] = str(exc)
    try:
        nic = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        nic.bind((args.interface, 0))
    except PermissionError:
        parser.exit(1, 'Passive capture requires sudo python3; no packets were sent.\n')
    counts, samples = Counter(), []
    end = time.monotonic() + args.seconds
    with nic:
        while time.monotonic() < end:
            nic.settimeout(max(.001, end - time.monotonic()))
            try:
                frame, address = nic.recvfrom(65535)
            except socket.timeout:
                break
            decoded = decode_ethernet(frame)
            if decoded is None:
                continue
            # PACKET_OUTGOING=4 on Linux. Preserve this separately from IP
            # origin so a route assumption cannot masquerade as a wire capture.
            decoded['outgoing'] = address[2] == 4
            decoded['capture_offset_s'] = round(args.seconds - (end - time.monotonic()), 4)
            key = decoded['magic'] + ('/out' if decoded['outgoing'] else '/in')
            counts[key] += 1
            counts[key + ('/valid' if decoded['checksum_valid'] else '/invalid')] += 1
            if len(samples) < 100:
                samples.append(decoded)
    evidence.update(counts=dict(counts), samples=samples)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(output=str(output), counts=dict(counts)), ensure_ascii=False))
    for magic in ('FOL1', 'TGT1', 'RBT1'):
        candidates = [s for s in samples if s['magic'] == magic]
        if candidates:
            print(json.dumps(candidates[-1], ensure_ascii=False))


if __name__ == '__main__':
    main()
