"""Offline Pi tracking and startup regressions: mocks, no devices.

Run from the source repository with Python3.10+, Node and a local POSIX shell.
Individual logs, exit codes and exact input hashes are retained under tmp/.
No result from this runner is a claim about real-camera inference or hardware.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = ROOT/'tmp/pi-tracking-regression/current'
    out.mkdir(parents=True, exist_ok=True)
    scripts = [
        'tools/check_target_tracking.py', 'tools/check_gimbal_tracking_loop.py',
        'tools/check_gimbal_tracking_memory.py', 'tools/check_usb_gimbal.py',
        'tools/check_tilt_operating_window.py', 'tools/check_robot_follow.py',
        'tools/check_visual_task.py', 'tools/check_independent_face_tracking.py',
        'tools/check_vision_worker_startup.py',
        'tools/check_gimbal_image_calibration.py', 'tools/check_gimbal_calibration_flow.py',
        'tools/check_gimbal_limit_diagnostics.py', 'tools/check_follow_send_cadence.py',
        'tools/check_gimbal_save_transactions.py',
        'tools/check_gimbal_mechanical_response.py', 'tests/test_orangepi_shell_bundle.py',
    ]
    commands = [(Path(p).stem, [sys.executable, '-X', 'utf8', p]) for p in scripts]
    commands += [('test_gimbal_control', [sys.executable, '-X', 'utf8', '-m', 'unittest', 'tests.test_gimbal_control']),
                 ('capture_self_test', [sys.executable, '-X', 'utf8', 'tools/capture_gimbal_tracking.py', '--self-test'])]
    node = shutil.which('node')
    if not node:
        raise SystemExit('Node required: actual page callback checks must not silently skip')
    commands += [('check_usb_gimbal_ui', [node, 'tools/check_usb_gimbal_ui.cjs', '--python', sys.executable])]
    inputs = [p for folder in ('pc','tests') for p in (ROOT/folder).rglob('*')
              if p.is_file() and p.suffix in ('.py','.html','.cjs')]
    inputs += list((ROOT/'tools').glob('check_*.py'))
    inputs += [ROOT/'tools/check_usb_gimbal_ui.cjs', ROOT/'tools/capture_gimbal_tracking.py',
               ROOT/'tools/package_orangepi_client.py', ROOT/'config/gimbal.example.json']
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in inputs}
    runs = []
    for name, command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                encoding='utf-8', errors='replace', timeout=240)
        log = result.stdout + result.stderr
        (out/(name+'.log')).write_text(log, encoding='utf-8')
        count = re.findall(r'Ran (\d+) tests? in', log)
        runs.append(dict(name=name, exit_code=result.returncode, passed=result.returncode == 0,
                         unittest_count=int(count[-1]) if count else None,
                         log_sha256=hashlib.sha256(log.encode('utf-8')).hexdigest()))
        print(name, 'PASS' if result.returncode == 0 else 'FAIL', count[-1:] or '', flush=True)
        if result.returncode:
            print(log[-3500:], flush=True)
    changed = [p for p, sha in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=sha]
    evidence = dict(all_passed=all(r['passed'] for r in runs) and not changed,
                    input_sha256=hashes, source_changes_during_run=changed, runs=runs,
                    unittest_count=sum(r['unittest_count'] or 0 for r in runs),
                    hardware_tested=False, real_model_inference_tested=False)
    (out/'result.json').write_text(json.dumps(evidence, indent=2)+'\n', encoding='utf-8')
    print('Overall:', evidence['all_passed'], 'unittest cases:', evidence['unittest_count'], flush=True)
    return 0 if evidence['all_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
