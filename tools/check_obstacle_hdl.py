"""Run the real collision-guard/navigation RTL without connecting hardware.

Use Icarus on PATH, or --xsim-bin <Vivado/bin>. Simulator exit codes alone
are insufficient: XSim 2018 can return zero after a Verilog $fatal.
"""
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SUITES = {
    'guard': ('tb_lidar_body_guard', (
        'src/rplidar_c1.v', 'src/robot_navigation.v', 'src/unsigned_divmod.v',
        'tests/tb_lidar_body_guard.v'), ('actual RTL projections;', 'PASS: actual guard -> navigation GOTO:')),
    'ui': ('tb_robot_avoid_ui', (
        'src/robot_ui_control.v', 'src/robot_control_panel.v',
        'tests/tb_robot_avoid_ui.v'), ('PASS: task3 3601 captured heading goals,',)),
    'route': ('tb_robot_route', (
        'src/robot_mission_control.v', 'src/lidar_path_planner.v',
        'tests/tb_robot_route.v'), ('PASS: actual inflated BFS + mission:',)),
}


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--xsim-bin', type=Path, help='Vivado bin directory')
    parser.add_argument('--suite', choices=('all', *SUITES), default='all')
    args = parser.parse_args()
    if args.xsim_bin:
        suffix = '.bat' if sys.platform == 'win32' else ''
        executables = [args.xsim_bin / (name + suffix)
                       for name in ('xvlog', 'xelab', 'xsim')]
        for executable in executables:
            if not executable.is_file():
                parser.error(f'Simulator executable missing: {executable}')
    elif not (shutil.which('iverilog') and shutil.which('vvp')):
        parser.error('Install Icarus on PATH or supply --xsim-bin <Vivado/bin>')

    work = ROOT / 'tmp' / 'obstacle-audit' / datetime.now().strftime('hdl-%Y%m%d-%H%M%S-%f')
    work.mkdir(parents=True)
    print(f'HDL logs: {work}', flush=True)
    for name in (SUITES if args.suite == 'all' else (args.suite,)):
        top, files, markers = SUITES[name]
        suite_work = work / name
        suite_work.mkdir()
        sources = [str(ROOT / path) for path in files]
        if args.xsim_bin:
            commands = [
                [str(executables[0]), *sources],
                [str(executables[1]), top, '-s', 'regression', '--timescale', '1ns/1ps'],
                [str(executables[2]), 'regression', '-runall'],
            ]
        else:
            commands = [
                [shutil.which('iverilog'), '-g2012', '-s', top, '-o', 'regression.vvp', *sources],
                [shutil.which('vvp'), 'regression.vvp'],
            ]
        final_output = ''
        for index, command in enumerate(commands):
            result = subprocess.run(command, cwd=suite_work, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True,
                                    encoding='utf-8', errors='replace', timeout=180)
            final_output = result.stdout
            (suite_work / f'{index + 1}.log').write_text(final_output, encoding='utf-8')
            if result.returncode or re.search(r'(?im)^\s*(?:fatal|error)\s*:', final_output):
                print(final_output)
                return 1
        if not all(marker in final_output for marker in markers):
            print(final_output)
            print(f'FAIL: {name} ended without completion markers')
            return 1
        for line in final_output.splitlines():
            if line.startswith('PASS:'):
                print(line, flush=True)
    print('Offline RTL simulation only; no synthesis, timing analysis, programming or device motion.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
