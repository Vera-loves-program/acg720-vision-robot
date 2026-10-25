"""Patch the v18 transient UDP error without changing its FPGA protocol.

Run on the Pi; it edits only pc/robot_follow.py and preserves one backup.
Restart the gimbal process manually afterwards. No network/PWM calls here.
"""
from pathlib import Path
import argparse


def apply(root):
    path = Path(root)/'pc/robot_follow.py'
    original = path.read_text(encoding='utf-8')
    if 'def _transmit(self, raw):' in original or "self.record('link_send_error', error=str(exc))" in original:
        return 'Network recovery fix already present; no files changed.'
    pattern = "self.sock.sendto(raw, ('192.168.10.255', 5001))"
    lines = original.splitlines(keepends=True)
    result = []
    changed = 0
    for line in lines:
        if line.strip() == pattern:
            indent = line[:len(line)-len(line.lstrip())]
            result += [indent+'try:\n',indent+'    '+pattern+'\n',
                       indent+'except OSError as exc:\n',
                       indent+"    self.record('link_send_error', error=str(exc))\n",
                       indent+'    self.follower.output = 0\n',indent+'    return\n']
            changed += 1
        else:
            result.append(line)
    if changed != 2:
        raise ValueError(f'Expected the two v18 UDP send sites; found {changed}. No file changed.')
    updated = ''.join(result)
    compile(updated,str(path),'exec')
    backup = path.with_name('robot_follow.before-network-fix.py')
    if not backup.exists():
        backup.write_bytes(path.read_bytes())
    path.write_text(updated,encoding='utf-8')
    return 'Fixed both UDP sends; calibration/models untouched. Restart the gimbal process.'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    print(apply(parser.parse_args().root))
