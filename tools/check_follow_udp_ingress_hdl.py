"""Exercise real GMII UDP RX -> metadata mailbox -> 50MHz follow/PWM RTL.

Offline only. Does not open a NIC, serial port, camera, or hardware programmer.
The wire FOL1/TGT1 CRC is produced by the actual Pi packet() function.
"""
import argparse
import binascii
import hashlib
import json
from pathlib import Path
import random
import shlex
import socket
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
from robot_follow import packet


def checksum(raw):
    total = sum(struct.unpack('>' + 'H' * (len(raw) // 2), raw))
    while total >> 16:
        total = (total & 65535) + (total >> 16)
    return (~total) & 65535


def frame(payload, *, destination='192.168.10.255', port=5001, unicast=False,
          bad_fcs=False):
    ethernet = bytes.fromhex('02ac72000002' if unicast else 'ffffffffffff')
    ethernet += bytes.fromhex('02ac72000003') + b'\x08\x00'
    header = struct.pack('>BBHHHBBH4s4s', 0x45, 0, 28 + len(payload), 37, 0x4000, 64, 17, 0,
                         socket.inet_aton('192.168.10.3'), socket.inet_aton(destination))
    header = header[:10] + struct.pack('>H', checksum(header)) + header[12:]
    body = ethernet + header + struct.pack('>4H', 6102, port, 8 + len(payload), 0) + payload
    body = body.ljust(60, b'\0')  # Ethernet minimum excluding FCS/preamble.
    fcs = binascii.crc32(body) ^ (1 if bad_fcs else 0)
    return b'\x55' * 7 + b'\xd5' + body + struct.pack('<I', fcs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wsl')
    args = parser.parse_args()
    out = ROOT / 'tmp/follow-udp-ingress-20261103'
    out.mkdir(parents=True, exist_ok=True)
    def fol(seq, pan=1200, tilt=1076):
        return packet(b'FOL1', 24, seq,
                      struct.pack('>h10H', 0, pan, tilt, 380, 2550, 645, 2420,
                                  65535, 0, 39, 0))
    tgt = packet(b'TGT1', 1, 15131, struct.pack('>HBB', 7, 2, 93).ljust(16, b'\0') + b'\0' * 6)
    burst_tgt = packet(b'TGT1', 1, 15137, struct.pack('>HBB', 7, 2, 93).ljust(16, b'\0') + b'\0' * 6)
    vec = packet(b'VEC1', 0, 77, struct.pack('>HHhhhhHhhI', 0, 65535, 16384, 0, 0, 0, 2048, 0, 261, 0))
    corrupt = bytearray(fol(15131)); corrupt[-1] ^= 1
    bad_ip_checksum = bytearray(frame(fol(15135)))
    bad_ip_checksum[32] ^= 1  # IPv4 checksum, not Ethernet FCS.
    bad_ip_checksum[-4:] = struct.pack('<I', binascii.crc32(bad_ip_checksum[8:-4]))
    vectors = [frame(fol(15130)), frame(tgt), frame(bytes(corrupt)),
               frame(fol(15131)), frame(fol(15132), bad_fcs=True),
               frame(fol(15132), destination='192.168.10.7'),
               frame(fol(15132), port=5002),
               frame(fol(15132), destination='192.168.10.2', unicast=True),
               frame(fol(15131)), frame(fol(15133, pan=1800, tilt=1300)),
               frame(b'control!'), frame(bytes(range(17))), frame(bytes(range(18))),
               frame(fol(15134) + b'\0' * 32), frame(fol(15134, pan=1800, tilt=1300)),
               bytes(bad_ip_checksum), frame(fol(15135, pan=1800, tilt=1300)),
               frame(fol(15136, pan=1800, tilt=1300)), frame(vec), frame(burst_tgt)]
    assert all(len(v) <= 128 for v in vectors)
    (out / 'frames.hex').write_text(''.join(f'{b:02x}\n' for v in vectors for b in v.ljust(128, b'\0')), encoding='ascii')
    (out / 'lengths.hex').write_text(''.join(f'{len(v):02x}\n' for v in vectors), encoding='ascii')
    rng = random.Random(20261103)
    headers = [[0] * 9, [65535] * 9] + [[rng.randrange(65536) for _ in range(9)] for _ in range(512)]
    (out / 'checksum.hex').write_text(''.join(f'{word:04x}\n' for h in headers
        for word in h + [checksum(struct.pack('>9H', *h))]), encoding='ascii')
    sources = ['tests/tb_follow_udp_ingress.v', 'src/vendor/ethernet/eth_udp_rx_gmii.v',
               'src/vendor/ethernet/ip_checksum.v', 'src/vendor/ethernet/crc32_d8.v',
               'src/udp_detection_feedback.v', 'src/robot_subsystem.v']
    target = 'tmp/follow-udp-ingress-20261103/test.vvp'
    def run(command):
        if args.wsl:
            root = '/mnt/' + ROOT.drive[0].lower() + ROOT.as_posix()[2:]
            command = ['wsl', '-d', args.wsl, '--', 'sh', '-lc',
                       'cd ' + shlex.quote(root) + ' && ' + shlex.join(command)]
        return subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', timeout=120)
    compiled = run(['iverilog', '-g2012', '-s', 'tb_follow_udp_ingress', '-o', target, *sources])
    (out / 'compile.log').write_text(compiled.stdout + compiled.stderr, encoding='utf-8')
    if compiled.returncode:
        raise SystemExit(compiled.stdout + compiled.stderr)
    result = run(['vvp', target])
    log = result.stdout + result.stderr
    (out / 'run.log').write_text(log, encoding='utf-8')
    checksum_sources = ['tests/tb_ip_checksum_pipeline.v', 'src/vendor/ethernet/ip_checksum.v']
    checksum_target = 'tmp/follow-udp-ingress-20261103/checksum.vvp'
    checksum_compile = run(['iverilog', '-g2012', '-s', 'tb_ip_checksum_pipeline', '-o', checksum_target, *checksum_sources])
    (out / 'checksum-compile.log').write_text(checksum_compile.stdout + checksum_compile.stderr, encoding='utf-8')
    if checksum_compile.returncode:
        raise SystemExit(checksum_compile.stdout + checksum_compile.stderr)
    checksum_result = run(['vvp', checksum_target])
    checksum_log = checksum_result.stdout + checksum_result.stderr
    (out / 'checksum-run.log').write_text(checksum_log, encoding='utf-8')
    evidence = dict(passed=result.returncode == 0 and 'PASS' in log and
                    checksum_result.returncode == 0 and 'PASS' in checksum_log,
                    sources={s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest()
                             for s in sources + checksum_sources + ['pc/robot_follow.py']},
                    frame_sha256=hashlib.sha256(b''.join(vectors)).hexdigest(), log=log,
                    checksum_log=checksum_log)
    (out / 'result.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    print(log, end='')
    print(checksum_log, end='')
    if not evidence['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
