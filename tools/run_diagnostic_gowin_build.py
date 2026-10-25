"""Build an exact-profile temporary clone with report-only SDC additions.

Original project, clocks, constraints, process profile and IP are never edited.
gw_sh auto-saves file-list changes; isolation is required for extra reports.
No programming or device actions are performed.
"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import time
import zipfile
from run_core_gowin_build import ROOT, TOOL, DEST, sha, timing_summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-pnr',type=Path,help='Failed manifest with completed synthesis and unchanged HDL/profile')
    args=parser.parse_args()
    tag=datetime.now(timezone(timedelta(hours=8))).strftime('%H%M%S')
    resumed=json.loads(args.resume_pnr.read_text(encoding='utf-8')) if args.resume_pnr else None
    workspace=Path(resumed['build_project']).parent if resumed else DEST/('build-'+tag+'-project')
    assert workspace.resolve().is_relative_to(DEST.resolve())
    workspace.mkdir(parents=True,exist_ok=bool(resumed))
    profile=ROOT/'impl/vision_robot_ui_process_config.json'
    (workspace/'impl').mkdir(exist_ok=bool(resumed))
    if not resumed:shutil.copyfile(profile,workspace/'impl'/profile.name)
    assert sha(profile)==sha(workspace/'impl'/profile.name)
    original_sdc=ROOT/'src/lcd_interaction.sdc'
    report_commands=(
        'report_timing -setup -from_clock [get_clocks {clk50M}] -to_clock [get_clocks {clk50M}] -max_paths 25 -max_common_paths 1',
        'report_timing -setup -from_clock [get_clocks {rgmii_rx_clk}] -to_clock [get_clocks {rgmii_rx_clk}] -max_paths 25 -max_common_paths 1',
        'report_timing -setup -from_clock [get_clocks {lcd_pclk}] -to_clock [get_clocks {lcd_pclk}] -max_paths 25 -max_common_paths 1',
    )
    wrapper=workspace/'report_only.sdc'
    wrapper.write_bytes(original_sdc.read_bytes()+b'\n# Report requests only; constraints above are unchanged.\n'+
                        ('\n'.join(report_commands)+'\n').encode('ascii'))
    assert wrapper.read_bytes().startswith(original_sdc.read_bytes())
    project=workspace/'vision_robot_ui.gprj'
    original_project=(ROOT/project.name).read_text(encoding='utf-8')
    def absolute_file(match):
        source=(ROOT/match[1]).resolve()
        return 'path="'+(wrapper if source==original_sdc.resolve() else source).as_posix()+'"'
    project.write_text(re.sub(r'path="([^"]+)"',absolute_file,original_project),encoding='utf-8')
    original_files=[ROOT/project.name,*ROOT.joinpath('src').rglob('*.v'),original_sdc,
                    ROOT/'src/lcd_interaction.cst',*ROOT.joinpath('impl').glob('*process_config.json')]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in original_files if p.is_file()}
    netlist=workspace/'impl/gwsynthesis/vision_robot_ui.vg'
    if resumed:
        assert hashes==resumed['inputs'],'Cannot reuse a netlist after its original source inputs change'
        assert netlist.is_file() and netlist.stat().st_size>0,'Completed synthesis netlist required'
    result_path=DEST/('build-'+tag+'.json');log_path=DEST/('build-'+tag+'.log');script=DEST/('build-'+tag+'.tcl')
    script.write_text('cd {'+ROOT.as_posix()+'}\nopen_project {'+project.as_posix()+'}\nrun '+('pnr' if resumed else 'all')+'\nexit\n',encoding='utf-8')
    manifest=dict(started_local=datetime.now(timezone(timedelta(hours=8))).isoformat(),user_authorized=True,
                  command=[str(TOOL),str(script)],inputs=hashes,log=str(log_path),
                  scope='synthesis/P&R only; no device actions; isolated identical-profile clone with report-only SDC',
                  build_project=str(project),output_directory=str(workspace/'impl'),
                  original_project_sha256=sha(ROOT/project.name),effective_project_sha256=sha(project),
                  profile_sha256=sha(profile),original_sdc_sha256=sha(original_sdc),
                  effective_sdc_sha256=sha(wrapper),report_only_commands=report_commands)
    if resumed:manifest.update(resumed_from=str(args.resume_pnr),reused_netlist_sha256=sha(netlist))
    with zipfile.ZipFile(DEST/('build-'+tag+'-sources.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:archive.write(ROOT/name,name)
        archive.write(project,'effective/vision_robot_ui.gprj')
        archive.write(wrapper,'effective/report_only.sdc')
        archive.write(workspace/'impl'/profile.name,'effective/impl/'+profile.name)
    result_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (DEST/'current-build.json').write_text(json.dumps(dict(result=str(result_path),log=str(log_path))),encoding='utf-8')
    started=time.time();print('BUILD '+tag+' started; isolated report-only clone; log='+str(log_path),flush=True)
    with log_path.open('wb') as log:
        proc=subprocess.run([str(TOOL),str(script)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    log=log_path.read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'^.*ERROR\s*\([^\n]*',log,re.M)
    changed=[name for name,digest in hashes.items() if sha(ROOT/name)!=digest]
    fs=workspace/'impl/pnr/vision_robot_ui.fs';report=workspace/'impl/pnr/vision_robot_ui_tr_content.html'
    fresh=fs.exists() and fs.stat().st_mtime>=started
    completed=proc.returncode==0 and not errors and fresh and not changed
    manifest.update(exit_code=proc.returncode,elapsed_seconds=round(time.time()-started,1),errors=errors,
                    finished_local=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                    inputs_changed_during_build=changed,fresh_fs=fresh,success=completed,build_completed=completed,
                    effective_profile_sha256_after=sha(workspace/'impl'/profile.name))
    manifest['profile_preserved']=manifest['effective_profile_sha256_after']==manifest['profile_sha256']
    manifest['timing']=timing_summary(report) if report.exists() and report.stat().st_mtime>=started else dict(timing_closed=False,report_missing=True)
    manifest['fully_validated_build']=completed and manifest['profile_preserved'] and manifest['timing']['timing_closed']
    if fresh:manifest['bitstream']=dict(path=str(fs),bytes=fs.stat().st_size,sha256=sha(fs))
    for relative in ['impl/pnr/vision_robot_ui.rpt.txt','impl/pnr/vision_robot_ui_tr_content.html','impl/gwsynthesis/vision_robot_ui_syn_rsc.xml','impl/pnr/cmd.do']:
        path=workspace/relative
        if path.exists() and path.stat().st_mtime>=started:(DEST/('build-'+tag+'-'+path.name)).write_bytes(path.read_bytes())
    result_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    if fresh:
        with zipfile.ZipFile(DEST/('build-'+tag+'-artifact.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
            archive.write(fs,'vision_robot_ui.fs');archive.write(result_path,'build.json');archive.write(log_path,'build.log')
    print(json.dumps({k:v for k,v in manifest.items() if k!='inputs'},ensure_ascii=False,indent=2),flush=True)
    return 0 if completed and manifest['profile_preserved'] else 1


if __name__=='__main__':raise SystemExit(main())
