"""Freeze a standalone project, build with Gowin, and fail closed for upload.

Never programs hardware. Original project/options/constraints/IP stay intact.
Tool exit zero or an existing bitstream alone cannot authorize publication.
"""
from argparse import ArgumentParser
from datetime import datetime, timedelta, timezone
from pathlib import Path
import hashlib
import html
import json
import re
import shutil
import subprocess
import time
from run_core_gowin_build import timing_summary

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_timing(path):
    timing = timing_summary(path)
    plain = html.unescape(re.sub(r'<[^>]+>', ' ', path.read_text(encoding='utf-8', errors='replace')))
    for check in ('Recovery', 'Removal'):
        found = re.search(r'Numbers of ' + check + r' Violated Endpoints\s+(\d+)', plain, re.I)
        timing[check.lower() + '_violated_endpoints'] = int(found[1]) if found else None
    # Missing checks are UNKNOWN, not zero. A later manual timing audit must
    # provide evidence before the upload gate can be considered satisfied.
    timing['timing_closed'] = timing['timing_closed'] and all(
        timing[key] == 0 for key in ('recovery_violated_endpoints', 'removal_violated_endpoints'))
    return timing


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--gowin', type=Path, required=True, help='Path to official gw_sh.exe')
    parser.add_argument('--work-root', type=Path, default=Path('D:/fpga-builds'))
    parser.add_argument('--report-clock-paths', action='store_true',
                        help='Append report-only per-clock queries inside the isolated SDC copy')
    parser.add_argument('--reuse-synthesis', type=Path,
                        help='Reuse a completed netlist only if all original input hashes match')
    args = parser.parse_args()
    if not args.gowin.is_file():
        parser.error(f'Gowin executable not found: {args.gowin}')
    local = datetime.now(timezone(timedelta(hours=8)))
    workspace = args.work_root.resolve() / local.strftime('acg720-%Y%m%d-%H%M%S-%f')
    workspace.mkdir(parents=True)
    shutil.copytree(ROOT / 'src', workspace / 'src')
    shutil.copyfile(ROOT / 'vision_robot_ui.gprj', workspace / 'vision_robot_ui.gprj')
    shutil.copyfile(ROOT / 'build.tcl', workspace / 'build.tcl')
    (workspace / 'impl').mkdir()
    for path in (ROOT / 'impl').glob('*process_config.json'):
        shutil.copyfile(path, workspace / 'impl' / path.name)
    inputs = [p for p in workspace.rglob('*') if p.is_file()]
    hashes = {p.relative_to(workspace).as_posix(): digest(p) for p in inputs}
    assert all(digest(ROOT / name) == sha for name, sha in hashes.items())
    source_hashes = dict(hashes)
    report_commands = []
    if args.report_clock_paths:
        # Automatically generated PLL clock names do not exist yet when SDC
        # is parsed. Query only the clocks explicitly defined in the SDC.
        for clock in ('clk50M', 'rgmii_rx_clk', 'lcd_pclk'):
            report_commands.append('report_timing -setup -from_clock [get_clocks {' + clock +
                '}] -to_clock [get_clocks {' + clock + '}] -max_paths 25 -max_common_paths 1')
        isolated_sdc = workspace / 'src/lcd_interaction.sdc'
        isolated_sdc.write_bytes(isolated_sdc.read_bytes() +
            b'\n# Report requests only; all constraints above are unchanged.\n' +
            ('\n'.join(report_commands) + '\n').encode('ascii'))
        hashes['src/lcd_interaction.sdc'] = digest(isolated_sdc)
    reused_synthesis = None
    if args.reuse_synthesis:
        previous = json.loads(args.reuse_synthesis.read_text(encoding='utf-8'))
        assert previous.get('source_inputs', previous['inputs']) == source_hashes, \
            'Refusing netlist reuse: original source/project/profile/constraint hashes differ'
        previous_workspace = Path(previous['project']).parent
        netlist = previous_workspace / 'impl/gwsynthesis/vision_robot_ui.vg'
        previous_log = (previous_workspace / 'build.log').read_text(encoding='utf-8', errors='replace')
        assert netlist.is_file() and netlist.stat().st_size > 0 and 'GowinSynthesis finish' in previous_log, \
            'A completed synthesis netlist is required'
        shutil.copytree(previous_workspace / 'impl/gwsynthesis', workspace / 'impl/gwsynthesis')
        assert digest(workspace / 'impl/gwsynthesis/vision_robot_ui.vg') == digest(netlist)
        script = workspace / 'build.tcl'
        original_script = script.read_text(encoding='utf-8')
        assert original_script.count('run all') == 1
        script.write_text(original_script.replace('run all', 'run pnr'), encoding='utf-8')
        hashes['build.tcl'] = digest(script)
        reused_synthesis = dict(manifest=str(args.reuse_synthesis.resolve()),
            netlist_sha256=digest(netlist), all_original_inputs_identical=True)
    manifest_path = workspace / 'build-result.json'
    log_path = workspace / 'build.log'
    manifest = dict(started_local=local.isoformat(), project=str(workspace / 'vision_robot_ui.gprj'),
                    tool=str(args.gowin.resolve()), inputs=hashes, source_inputs=source_hashes,
                    report_only_commands=report_commands, build_completed=False,
                    reused_synthesis=reused_synthesis,
                    upload_allowed=False, scope='offline synthesis/P&R, no device actions')
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    pointer = ROOT / 'tmp/obstacle-audit/current-build.json'
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(json.dumps(dict(manifest=str(manifest_path), log=str(log_path)), indent=2), encoding='utf-8')
    print(f'Frozen build: {workspace}\nBuild log: {log_path}', flush=True)
    started = time.time()
    with log_path.open('wb') as log:
        result = subprocess.run([str(args.gowin.resolve()), str(workspace / 'build.tcl')],
                                cwd=workspace, stdout=log, stderr=subprocess.STDOUT)
    output = log_path.read_text(encoding='utf-8', errors='replace')
    if reused_synthesis is not None:
        # Gowin can automatically invalidate copied synthesis state after a
        # project relocation, even for run pnr. Report what actually happened.
        reused_synthesis['tool_reran_synthesis'] = 'GowinSynthesis finish' in output
        reused_synthesis['used_without_resynthesis'] = not reused_synthesis['tool_reran_synthesis']
    errors = re.findall(r'^.*(?:ERROR\s*\(|FATAL\b).*$', output, re.M | re.I)
    changed = [name for name, sha in source_hashes.items() if digest(ROOT / name) != sha]
    fs = workspace / 'impl/pnr/vision_robot_ui.fs'
    report = workspace / 'impl/pnr/vision_robot_ui_tr_content.html'
    fresh_fs = fs.is_file() and fs.stat().st_size > 0 and fs.stat().st_mtime >= started
    fresh_report = report.is_file() and report.stat().st_mtime >= started
    completed = result.returncode == 0 and not errors and fresh_fs and fresh_report and not changed
    timing = audit_timing(report) if fresh_report else dict(timing_closed=False, report_missing=True)
    manifest.update(finished_local=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                    elapsed_seconds=round(time.time() - started, 1), exit_code=result.returncode,
                    errors=errors, source_changes_during_build=changed, fresh_bitstream=fresh_fs,
                    build_completed=completed, timing=timing,
                    upload_allowed=completed and timing['timing_closed'])
    if fresh_fs:
        manifest['bitstream'] = dict(path=str(fs), bytes=fs.stat().st_size, sha256=digest(fs))
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in manifest.items() if key not in ('inputs', 'source_inputs')},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if manifest['upload_allowed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
