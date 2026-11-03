"""Offline Icarus RTL tests for labelled presets; no hardware/network commands."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CORE = [
    'src/robot_subsystem.v', 'src/robot_preset_demo.v', 'src/robot_ui_control.v',
    'src/robot_navigation_command_stage.v',
    'src/robot_control_panel.v', 'src/robot_status_text.v', 'src/robot_navigation.v',
    'src/unsigned_divmod.v', 'src/rplidar_c1.v', 'src/lidar_grid_map.v',
    'src/lidar_path_planner.v', 'src/robot_mission_control.v',
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wsl', help='Windows host: WSL distribution containing Icarus')
    ap.add_argument('--only', choices=('unit','navigation','subsystem','motor','stage'))
    ap.add_argument('--prim', type=Path, default=Path(
        'D:/GOWIN/Gowin_V1.9.11.03_Education_x64/IDE/simlib/gw5a/prim_sim.v'))
    args = ap.parse_args()
    out = ROOT / 'tmp/preset-hdl'
    out.mkdir(parents=True, exist_ok=True)

    def linux(path):
        p = str(path).replace('\\', '/')
        return '/mnt/' + p[0].lower() + p[2:] if len(p) > 1 and p[1] == ':' else p

    def run(argv):
        if args.wsl:
            argv = ['wsl', '-d', args.wsl, '--', 'sh', '-lc',
                    'cd ' + shlex.quote(linux(ROOT)) + ' && ' + shlex.join(argv)]
        return subprocess.run(argv, cwd=ROOT, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', timeout=180)

    cases = {
        'unit': ('tb_robot_preset_demo', ['tests/tb_robot_preset_demo.v', 'src/robot_preset_demo.v']),
        'navigation': ('tb_preset_navigation', ['tests/tb_preset_navigation.v',
            'src/robot_preset_demo.v', 'src/robot_navigation.v', 'src/unsigned_divmod.v',
            'src/robot_navigation_command_stage.v']),
        'subsystem': ('tb_preset_subsystem', ['tests/tb_preset_subsystem.v', *CORE]),
        'motor': ('tb_motor_edit_equivalence', ['tests/tb_motor_edit_equivalence.v',
            'tests/reference/motor_tuning_control_reference.v', 'src/motor_tuning_control.v']),
        'stage': ('tb_navigation_command_stage', ['tests/tb_navigation_command_stage.v',
            'src/robot_navigation_command_stage.v']),
    }
    selected = {args.only: cases[args.only]} if args.only else cases
    if 'subsystem' in selected:
        assert args.prim.is_file(), 'Official Gowin RAM/GSR simulation model required'
    evidence = {'hardware_tested': False, 'runs': []}
    for name, (top, sources) in selected.items():
        paths = sources + ([linux(args.prim) if args.wsl else str(args.prim)] if name == 'subsystem' else [])
        vvp = f'tmp/preset-hdl/{name}.vvp'
        compiled = run(['iverilog', '-g2012', '-s', top, '-o', vvp, *paths])
        (out / (name + '-compile.log')).write_text(compiled.stdout + compiled.stderr, encoding='utf-8')
        if compiled.returncode:
            raise SystemExit(compiled.stdout + compiled.stderr)
        tested = run(['vvp', vvp])
        log = tested.stdout + tested.stderr
        (out / (name + '-run.log')).write_text(log, encoding='utf-8')
        passed = tested.returncode == 0 and 'PASS:' in tested.stdout and 'FATAL:' not in log
        evidence['runs'].append(dict(name=name, passed=passed, exit_code=tested.returncode,
            sources={s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest() for s in sources},
            log=tested.stdout))
        print(tested.stdout, end='', flush=True)
        if not passed:
            print(tested.stderr)
            break
    evidence['all_passed'] = len(evidence['runs']) == len(selected) and all(r['passed'] for r in evidence['runs'])
    evidence['selected_only'] = args.only
    (out / ('result-'+args.only+'.json' if args.only else 'result.json')).write_text(json.dumps(evidence, indent=2) + '\n', encoding='utf-8')
    raise SystemExit(0 if evidence['all_passed'] else 1)


if __name__ == '__main__':
    main()
