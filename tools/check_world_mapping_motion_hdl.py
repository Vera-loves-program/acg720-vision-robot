"""Generate actual subsystem pose gates for the moving-world-map HDL test.

The legacy negative control changes only the capture qualifier back to the
original correction_allowed. No FPGA source or device is changed by this tool.
The shared system runner imports write_wrappers before compiling the testbench.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def write_wrappers(output: Path) -> Path:
    source = (ROOT / 'src/robot_subsystem.v').read_text(encoding='utf-8')
    begin = source.index('    wire world_map_pose_valid=')
    end = source.index('    wire [15:0] mission_arrival_tolerance;', begin)
    body = source[begin:end].replace('wire world_map_pose_valid=', 'assign world_map_pose_valid=', 1)
    if 'mapping_allowed' not in body:
        raise AssertionError('actual moving-world capture qualifier is missing')
    header = '''(
input wire clk,reset_n,tick_10ms,map_reset,map_local,pose_valid,pose_update,
input wire mapping_allowed,correction_allowed,
input wire [5:0] static_gyro_age,
input wire signed [31:0] robot_x,robot_y,
input wire [15:0] heading_q6,
output wire world_map_pose_valid,
output reg signed [31:0] map_pose_x,map_pose_y,
output reg [15:0] map_pose_heading);
reg map_local_previous;
reg [1:0] map_pose_age;
reg map_pose_seen,map_pose_slow;
'''
    text = 'module actual_root_mapping_pose_gate' + header + body + 'endmodule\n'
    text += ('module legacy_root_mapping_pose_gate' + header +
             body.replace('mapping_allowed', 'correction_allowed') + 'endmodule\n')
    marker = '.correction_enable('
    expression = source.split(marker, 1)[1].split('),.correction_ack(', 1)[0]
    if 'correction_allowed' not in expression:
        raise AssertionError('scan-match correction must retain its original low-speed qualifier')
    text += '''module actual_root_mapping_correction_gate(
input wire map_local,world_map_pose_valid,correction_allowed,
input wire manual_active,manual_turn_active,remote_turn_active,map_reset,
output wire permitted);
assign permitted=''' + expression + ';\nendmodule\n'
    output.mkdir(parents=True, exist_ok=True)
    target = output / 'actual_mapping_pose_gate.v'
    target.write_text(text, encoding='utf-8', newline='\n')
    return target


if __name__ == '__main__':
    print(write_wrappers(ROOT / 'tmp/lidar-20261103/hdl'))
