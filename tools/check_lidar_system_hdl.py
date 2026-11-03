"""Run actual clocked HDL regressions. Requires Icarus and official Gowin DPB model.

Windows: python tools/check_lidar_system_hdl.py --wsl Ubuntu-22.04
No synthesis, flashing, serial ports, camera, motor or Pi actions.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time
from check_world_mapping_motion_hdl import write_wrappers as write_mapping_pose_wrappers

ROOT=Path(__file__).resolve().parents[1]
CASES=[
 ('tb_lidar_body_guard',['tests/tb_lidar_body_guard.v','src/rplidar_c1.v']),
 ('tb_rplidar_raw_diagnostics',['tests/tb_lidar_body_guard.v','src/rplidar_c1.v']),
 ('tb_lidar_planner_start',['tests/tb_lidar_planner_start.v','src/lidar_path_planner.v']),
 ('tb_lidar_detour_preference',['tests/tb_lidar_detour_preference.v','src/lidar_path_planner.v']),
 ('tb_mission_observation_resume',['tests/tb_mission_observation_resume.v','src/robot_mission_control.v']),
 ('tb_robot_map_viewport',['tests/tb_robot_map_viewport.v','src/robot_control_panel.v','src/rplidar_c1.v']),
 ('tb_nav_observation_gate',['tests/tb_nav_observation_gate.v','src/robot_navigation.v','src/unsigned_divmod.v']),
 ('tb_diagnostic_snapshot',['tests/tb_diagnostic_snapshot.v','src/robot_subsystem.v','src/motor_tuning_control.v']),
 ('tb_lidar_lookahead',['tests/tb_lidar_lookahead.v','tmp/lidar-20261103/hdl/actual_lookahead.v','src/rplidar_c1.v','src/robot_navigation.v','src/unsigned_divmod.v']),
 ('tb_map_ray_evidence',['tests/tb_map_ray_evidence.v','src/lidar_grid_map.v']),
 ('tb_robot_rejection_status',['tests/tb_robot_rejection_status.v','tmp/lidar-20261103/hdl/actual_rejection_pipeline.v',
    'src/robot_status_text.v','src/lidar_grid_map.v','src/lidar_path_planner.v']),
 ('tb_world_mapping_motion',['tests/tb_world_mapping_motion.v','tmp/lidar-20261103/hdl/actual_mapping_pose_gate.v',
    'src/robot_navigation.v','src/unsigned_divmod.v','src/rplidar_c1.v','src/lidar_grid_map.v']),
]
def linux_path(p):
    s=str(p).replace('\\','/')
    return '/mnt/'+s[0].lower()+s[2:] if len(s)>1 and s[1]==':' else s

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wsl',default=None)
    parser.add_argument('--only',choices=[case[0] for case in CASES],help='Run one named group; results then cover only this selection')
    parser.add_argument('--gowin-simlib',type=Path,default=Path('D:/GOWIN/Gowin_V1.9.11.03_Education_x64/IDE/simlib/gw5a/prim_sim.v'))
    args=parser.parse_args()
    output=ROOT/'tmp/lidar-20261103/hdl';output.mkdir(parents=True,exist_ok=True)
    source_bytes=(ROOT/'src/robot_subsystem.v').read_bytes()
    source=source_bytes.decode('utf-8-sig').replace('\r\n','\n')
    snippet=source[source.index('    wire signed [31:0] mission_dx='):source.index('    reg lookahead_record_valid;')]
    snippet=snippet.replace('wire nav_obstacle_ahead=','assign nav_obstacle_ahead=')
    header='''module actual_root_lookahead(
input wire clk,reset_n,map_reset,avoid_lidar_ready,body_guard_ready,scan_start,scan_complete,
input wire [3:0] nav_status,
input wire ui_command_valid,follow_command_valid,mission_command_valid,
input wire [2:0] ui_command,follow_command,mission_command,
input wire signed [31:0] mission_goal_x,mission_goal_y,robot_x,robot_y,
input wire [15:0] body_front,body_left,body_right,drive_margin,
input wire signed [15:0] body_point_x,body_point_y,
input wire map_point_valid,confirmed_point_valid,
output wire nav_obstacle_ahead);
'''
    (output/'actual_lookahead.v').write_text(header+snippet+'endmodule\n',encoding='utf-8')
    # Execute the actual register/priority statements with the actual text
    # producer. This is source extraction, not a behavioral reimplementation
    # or a full-SoC simulation; retain the original producer SHA below.
    rejection=source[source.index('    reg [16:0] diagnostic_conditions_q;'):source.index('    wire [15:0] diagnostic_trigger_delta=')]
    rejection=rejection.replace('    reg [7:0] reject_diagnostic;\n','')
    effective=next(line for line in source.splitlines() if 'wire [7:0] effective_planner_error=' in line)
    rejection_header='''module actual_rejection_pipeline(
input wire clk,reset_n,scan_enable,lidar_running,complete_scan_seen,
input wire [5:0] complete_scan_age,
input wire gyro_valid,gyro_calibrated,calibrating,gyro_fresh_diag,
input wire projection_bad,last_scan_bad,lidar_ready,body_guard_ready,
input wire virtual_contact,nav_obstacle_ahead,body_front_stop,body_rear_stop,body_turn_stop,
input wire [3:0] nav_required_coverage,body_free_coverage,
input wire map_local,mission_waiting_map,no_path,
input wire [7:0] mission_error,planner_error,
output reg [7:0] reject_diagnostic);
'''
    (output/'actual_rejection_pipeline.v').write_text(rejection_header+effective+'\n'+rejection+'endmodule\n',encoding='utf-8')
    write_mapping_pose_wrappers(output)
    if not args.gowin_simlib.is_file():parser.error('Official gw5a prim_sim.v required for actual snapshot DPB behavior')
    def run(argv):
        if args.wsl:
            argv=['wsl','-d',args.wsl,'--','sh','-lc','cd '+shlex.quote(linux_path(ROOT))+' && '+shlex.join(argv)]
        return subprocess.run(argv,cwd=ROOT,capture_output=True,encoding='utf-8',errors='replace',timeout=240)
    results=[]
    selected=[case for case in CASES if args.only is None or case[0]==args.only]
    for top,files in selected:
        paths=files+([str(args.gowin_simlib)] if top in ('tb_map_ray_evidence','tb_robot_rejection_status','tb_world_mapping_motion') else [])
        compile_paths=[linux_path(x) for x in paths] if args.wsl else paths
        target='tmp/lidar-20261103/hdl/'+top+'.vvp'
        start=time.monotonic()
        c=run(['iverilog','-g2012','-s',top,'-o',target,*compile_paths])
        (output/(top+'-compile.log')).write_text(c.stdout+c.stderr,encoding='utf-8')
        if c.returncode:raise SystemExit('HDL compile failed: '+top+'\n'+c.stdout+c.stderr)
        r=run(['vvp',target]);log=r.stdout+r.stderr
        (output/(top+'-run.log')).write_text(log,encoding='utf-8')
        passed=r.returncode==0 and 'PASS' in log
        results.append(dict(top=top,passed=passed,exit_code=r.returncode,seconds=round(time.monotonic()-start,3),
            sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},
            original_subsystem_sha256=hashlib.sha256(source_bytes).hexdigest()
                if top in ('tb_lidar_lookahead','tb_robot_rejection_status','tb_world_mapping_motion') else None))
        print(top+': '+('PASS' if passed else 'FAIL'),flush=True)
        print(r.stdout.replace('\x00','')[-600:],flush=True)
        if not passed:break
    manifest=dict(tool='Icarus Verilog actual HDL execution',runs=results,
        selected_only=args.only,all_passed=len(results)==len(selected) and all(r['passed'] for r in results),
        hardware_tested=False,cdc_proof=False,official_ram_model_sha256=hashlib.sha256(args.gowin_simlib.read_bytes()).hexdigest())
    (output/'results.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    raise SystemExit(0 if manifest['all_passed'] else 1)

if __name__=='__main__':main()
