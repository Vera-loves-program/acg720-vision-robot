"""Run actual mission completion-event RTL; no hardware or device actions."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wsl')
    parser.add_argument('--only',choices=('mission','left','map'))
    parser.add_argument('--baseline-mission',type=Path,help='Read-only negative control; requires --only mission')
    args=parser.parse_args()
    if args.baseline_mission and args.only!='mission':parser.error('--baseline-mission requires --only mission')
    out=ROOT/'tmp/resume-observation-hdl';out.mkdir(parents=True,exist_ok=True)
    if args.baseline_mission:out=out/'baseline-current-tests';out.mkdir(exist_ok=True)
    def run(argv):
        if args.wsl:
            root='/mnt/'+ROOT.drive[0].lower()+ROOT.as_posix()[2:]
            argv=['wsl','-d',args.wsl,'--','sh','-lc','cd '+shlex.quote(root)+' && '+shlex.join(argv)]
        return subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=180)
    cases={
      'mission':('tb_mission_observation_latch',['tests/tb_mission_observation_latch.v','src/robot_mission_control.v']),
      'left':('tb_lidar_detour_preference',['tests/tb_lidar_detour_preference.v','src/lidar_path_planner.v']),
      'map':('tb_real_map_resume',['tests/tb_real_map_resume.v','src/robot_mission_control.v','src/lidar_path_planner.v','src/lidar_grid_map.v']),
    }
    runs=[]
    for name,(top,sources) in cases.items():
        if args.only and args.only!=name:continue
        if args.baseline_mission:sources[-1]=str(args.baseline_mission.resolve())
        paths=list(sources)
        if args.wsl:
            paths=['/mnt/'+p[0].lower()+p.replace('\\','/')[2:] if len(p)>1 and p[1]==':' else p for p in paths]
        if name=='map':
            model=Path('D:/GOWIN/Gowin_V1.9.11.03_Education_x64/IDE/simlib/gw5a/prim_sim.v')
            assert model.is_file(),'Official Gowin DPB/GSR models required'
            paths.append('/mnt/d'+model.as_posix()[2:] if args.wsl else str(model))
        target=(out.relative_to(ROOT)/(name+'.vvp')).as_posix()
        c=run(['iverilog','-g2012','-s',top,'-o',target,*paths])
        (out/(name+'-compile.log')).write_text(c.stdout+c.stderr,encoding='utf-8')
        if c.returncode:raise SystemExit(c.stdout+c.stderr)
        r=run(['vvp',target]);log=r.stdout+r.stderr
        (out/(name+'-run.log')).write_text(log,encoding='utf-8')
        result=dict(name=name,passed=r.returncode==0 and 'PASS' in log,exit_code=r.returncode,
                    sources={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},log=log)
        runs.append(result);print(r.stdout,end='',flush=True)
        if not result['passed']:break
    evidence=dict(all_passed=bool(runs) and all(r['passed'] for r in runs),selected_only=args.only,runs=runs,hardware_tested=False)
    (out/'result.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if evidence['all_passed'] else 1)

if __name__=='__main__':main()
