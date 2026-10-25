"""Authorized offline Gowin build, immutable input manifest and fresh outputs.

Never changes constraints, project/options or vendor IP. Never programs a
device. stdout is saved while gw_sh runs the user's existing project profile.
"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import html
import json
import re
import subprocess
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
TOOL=Path('D:/GOWIN/Gowin_V1.9.11.03_Education_x64/IDE/bin/gw_sh.exe')
DEST=ROOT/'tmp/core-route-20261025'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def timing_summary(report):
    """Keep a completed route distinct from meeting the real constraints."""
    body=report.read_text(encoding='utf-8',errors='replace')
    section=body.split('name="Max_Frequency_Report"',1)[-1].split('</table>',1)[0]
    clocks=[]
    for row in re.findall(r'<tr[^>]*>(.*?)</tr>',section,re.S):
        cells=[html.unescape(re.sub('<[^>]+>','',cell)).strip()
               for cell in re.findall(r'<td[^>]*>(.*?)</td>',row,re.S)]
        if len(cells)!=6:continue
        required=re.match(r'([\d.]+)\(MHz\)',cells[2])
        actual=re.match(r'([\d.]+)\(MHz\)',cells[3])
        if required and actual:
            clocks.append(dict(clock=cells[1],required_mhz=float(required[1]),actual_mhz=float(actual[1])))
    clean=html.unescape(re.sub('<[^>]+>',' ',body))
    numbers={}
    for kind in ('Setup','Hold'):
        match=re.search(r'Numbers of '+kind+r' Violated Endpoints\s+(\d+)',clean)
        numbers[kind.lower()+'_violated_endpoints']=int(match[1]) if match else None
    closed=bool(clocks) and all(c['actual_mhz']>=c['required_mhz'] for c in clocks)
    closed=closed and all(v==0 for v in numbers.values())
    return dict(clocks=clocks,**numbers,timing_closed=closed)


def main():
    DEST.mkdir(exist_ok=True)
    tag=datetime.now(timezone(timedelta(hours=8))).strftime('%H%M%S')
    result_path=DEST/('build-'+tag+'.json')
    log_path=DEST/('build-'+tag+'.log')
    script=DEST/('build-'+tag+'.tcl')
    script.write_text('cd {'+ROOT.as_posix()+'}\nopen_project vision_robot_ui.gprj\nrun all\nexit\n',encoding='utf-8')
    inputs=[ROOT/'vision_robot_ui.gprj',*ROOT.joinpath('src').rglob('*.v'),ROOT/'src/lcd_interaction.cst',ROOT/'src/lcd_interaction.sdc',*ROOT.joinpath('impl').glob('*process_config.json')]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in inputs if p.is_file()}
    with zipfile.ZipFile(DEST/('build-'+tag+'-sources.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:archive.write(ROOT/name,name)
    manifest=dict(started_local=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                  user_authorized=True,command=[str(TOOL),str(script)],inputs=hashes,
                  log=str(log_path),scope='synthesis/P&R only; no flashing or device actions')
    result_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (DEST/'current-build.json').write_text(json.dumps(dict(result=str(result_path),log=str(log_path))),encoding='utf-8')
    started=time.time()
    print('BUILD '+tag+' started; log='+str(log_path),flush=True)
    with log_path.open('wb') as log:
        proc=subprocess.run([str(TOOL),str(script)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    log=log_path.read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'^.*ERROR\s*\([^\n]*',log,re.M)
    manifest.update(exit_code=proc.returncode,elapsed_seconds=round(time.time()-started,1),
                    errors=errors,finished_local=datetime.now(timezone(timedelta(hours=8))).isoformat())
    changed=[name for name,digest in hashes.items() if sha(ROOT/name)!=digest]
    manifest['inputs_changed_during_build']=changed
    fs=ROOT/'impl/pnr/vision_robot_ui.fs'
    manifest['fresh_fs']=fs.exists() and fs.stat().st_mtime>=started
    if manifest['fresh_fs']:
        manifest['bitstream']=dict(path=str(fs),bytes=fs.stat().st_size,sha256=sha(fs))
    for name in ['impl/pnr/vision_robot_ui.rpt.txt','impl/pnr/vision_robot_ui_tr_content.html','impl/gwsynthesis/vision_robot_ui_syn_rsc.xml']:
        source=ROOT/name
        if source.exists() and source.stat().st_mtime>=started:
            (DEST/('build-'+tag+'-'+source.name)).write_bytes(source.read_bytes())
    manifest['success']=proc.returncode==0 and not errors and manifest['fresh_fs'] and not changed
    manifest['build_completed']=manifest['success']
    report=ROOT/'impl/pnr/vision_robot_ui_tr_content.html'
    manifest['timing']=timing_summary(report) if report.exists() and report.stat().st_mtime>=started else dict(timing_closed=False,report_missing=True)
    manifest['fully_validated_build']=manifest['build_completed'] and manifest['timing']['timing_closed']
    result_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    if manifest['fresh_fs']:
        with zipfile.ZipFile(DEST/('build-'+tag+'-artifact.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
            archive.write(fs,'vision_robot_ui.fs')
            archive.write(result_path,'build.json')
            archive.write(log_path,'build.log')
    print(json.dumps({k:v for k,v in manifest.items() if k!='inputs'},ensure_ascii=False,indent=2),flush=True)
    return 0 if manifest['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
