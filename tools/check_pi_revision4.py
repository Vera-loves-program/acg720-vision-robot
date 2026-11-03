"""Run the focused Pi revision4 offline suites and preserve exact evidence.

No deployment, camera/servo/serial access, model inference or service actions.
The worker tests mock image/model calls; control/association/packet code is real.
"""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=(
    'check_face_mode_scheduling.py',
    'check_tracking_response.py',
    'check_independent_face_tracking.py',
    'check_usb_gimbal.py',
    'check_robot_follow.py',
    'check_visual_task.py',
    'check_tilt_operating_window.py',
    'check_target_tracking.py',
    'check_gimbal_tracking_memory.py',
)
SOURCES=('pc/usb_gimbal.py','pc/usb_gimbal_ui.html','pc/robot_follow.py','pc/target_tracking.py')
TEAMMATE_SHA256={
    'pc/usb_gimbal.py':'f1f4e1c4ab53403775faa02911eed551a1e47363285eb7c23d2206b6538873aa',
    'pc/usb_gimbal_ui.html':'7db7430cc955ecc646d8c02145871d1d51d0240105426d7b911806e4a8e9d3f3',
    'pc/robot_follow.py':'c3df9fd179c0e7de81d69450fad17e74ab5bf7a770b622135a324e829a0b166d',
    'pc/target_tracking.py':'7d6a05fc35aa8c7434aa943a1ba3b306c67e41be2018368d0ce0d9ae7c5b158a',
}


def main():
    output=ROOT/'tmp/pi-merge-20261103/checks'
    output.mkdir(parents=True,exist_ok=True)
    runs=[]
    for name in SCRIPTS:
        command=[sys.executable,'-X','utf8',str(ROOT/'tools'/name)]
        result=subprocess.run(command,cwd=ROOT,capture_output=True,encoding='utf-8',errors='replace',timeout=180)
        text=result.stdout+result.stderr
        log=output/(Path(name).stem+'.log')
        log.write_text(text,encoding='utf-8')
        counts=re.findall(r'Ran (\d+) tests? in ',text)
        passed=result.returncode==0 and bool(counts) and re.search(r'^OK\s*$',text,re.M) is not None
        count=int(counts[-1]) if counts else 0
        runs.append(dict(script='tools/'+name,passed=passed,tests=count,exit_code=result.returncode,
                         log=str(log.relative_to(ROOT)),command=command))
        print(f'{name}: {"PASS" if passed else "FAIL"} ({count} tests)',flush=True)
        if not passed:
            print(text[-3000:]);break
    evidence=dict(software_version=22,bundle_revision=4,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCES},
        teammate_revision3_source_sha256=TEAMMATE_SHA256,
        opencv_available=importlib.util.find_spec('cv2') is not None,
        camera_tested=False,hardware_tested=False,real_model_inference_tested=False,
        runs=runs,tests=sum(r['tests'] for r in runs),
        all_passed=len(runs)==len(SCRIPTS) and all(r['passed'] for r in runs))
    manifest=output/'results.json'
    manifest.write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(f'Total: {evidence["tests"]}; evidence: {manifest}',flush=True)
    raise SystemExit(0 if evidence['all_passed'] else 1)


if __name__=='__main__':main()
