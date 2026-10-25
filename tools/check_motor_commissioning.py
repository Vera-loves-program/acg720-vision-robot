"""Independent integer and source contracts for wheel commissioning.

No HDL compilation, elaboration, clocked RTL simulation, synthesis, timing,
network/device access or flashing. Resource savings are not inferred here.
"""
from itertools import product
from pathlib import Path
import re
from check_motor_integration import uncomment,check_blocks,module_ports,instances

ROOT=Path(__file__).resolve().parents[1]
names=('motor_tuning_control','motor_axis_control','motor_drive_control',
       'tb6612_axis_pwm','actuator_test_control','motor_uart_commands','motor_status_text')
sources={name:uncomment((ROOT/'src'/f'{name}.v').read_text(encoding='utf-8-sig')) for name in names}
for name,text in sources.items():check_blocks(text,name)
codes={name:re.sub(r'\s+','',text) for name,text in sources.items()}
tuning=codes['motor_tuning_control']
axis=codes['motor_axis_control']
drive=codes['motor_drive_control']
bridge=codes['tb6612_axis_pwm']

assert 'wiresigned[31:0]effective_argument=ui_action_valid?ui_argument:uart_argument;' in tuning
assert len(re.findall(r'\buart_argument\b',sources['motor_tuning_control']))==2
for action,limit,destination in ((16,1,'selected_wheel'),(17,1,'edit_position'),(18,9,'selected_field'),(20,2,'step_select')):
    clause=tuning.split(str(action)+':if(',1)[1].split(';',1)[0]
    assert 'effective_argument' in clause and destination+'<=' in clause
    assert ('effective_argument=='+str(limit) if action in (16,17) else 'effective_argument<='+str(limit)) in clause
assert 'if(act==19)requested_value={effective_argument[31],effective_argument};' in tuning
assert 'selected_field==0?9:selected_field-1\'b1;' in tuning
assert 'selected_field==9?0:selected_field+1\'b1;' in tuning
assert '8,9:high_limit=1;' in tuning
for field,kind in ((8,'encoder'),(9,'drive')):
    assert str(field)+':selected_value=selected_wheel?{31\'d0,'+kind+'_invert_right}:{31\'d0,'+kind+'_invert_left};' in tuning
shortcut=tuning.split('21,22:begin',1)[1].split('23:begin',1)[0]
for contract in ('armed<=0;running<=0;position_unlocked<=0;mode_position<=0;',
                 'zero_position<=1;target_position_left<=0;target_position_right<=0;',
                 "selected_field<=act==21?4'd8:4'd9;",
                 'if(selected_wheel)encoder_invert_right<=~encoder_invert_right;elseencoder_invert_left<=~encoder_invert_left;',
                 'if(selected_wheel)drive_invert_right<=~drive_invert_right;elsedrive_invert_left<=~drive_invert_left;'):
    assert contract in shortcut,contract
isolate=tuning.split('23:begin',1)[1].split('24:begin',1)[0]
assert 'armed<=0;running<=0;' in isolate
assert 'if(selected_wheel)begintarget_speed_left<=0;target_position_left<=0;end' in isolate
assert 'elsebegintarget_speed_right<=0;target_position_right<=0;end' in isolate
assert '_kp_' not in isolate and '_ki_' not in isolate and '_kd_' not in isolate
assert 'wiremotion_requested=run_enable||calibration_enable;' in axis
assert 'wiresafe_zero=!motion_requested&&(zero_position||encoder_invert!=applied_invert||drive_invert!=applied_drive_invert);' in axis
assert 'elseif(!motion_requested)beginapplied_invert<=encoder_invert;applied_drive_invert<=drive_invert;end' in axis
assert 'wirebridge_enabled=reset_n&&motion_requested&&!safety_stop&&fault==0;' in axis
assert 'wireenabled=run_enable&&!calibration_enable&&!safety_stop&&fault==0;' in axis
assert 'wire[9:0]calibration_limit=max_duty_permille>200?10\'d200:max_duty_permille;' in axis
assert 'calibration_duty>calibration_limit_signed?calibration_limit_signed:calibration_duty<-calibration_limit_signed?-calibration_limit_signed:calibration_duty;' in axis
assert 'assignduty=bridge_enabled?(calibration_enable?calibration_command:pid_duty):16\'sd0;' in axis
assert '.enable(bridge_enabled),.drive_invert(applied_drive_invert),.duty_permille(duty)' in axis
assert 'if(clear_fault&&!motion_requested)fault<=0;' in axis
assert 'if(!bridge_enabled)previous_invalid<=safe_zero?0:sampled_invalid;' in axis
assert 'target_effective<=0;pid_duty<=0;' in axis
common=axis.split('if(bridge_enabled&&sample_ready)begin',1)[1].split('end',1)[0]
for contract in ('previous_invalid<=sampled_invalid;',
                 'if(encoder_overflow)fault[3]<=1;',
                 'if(abs32(speed_raw)>MAX_SPEED_COUNTS)fault[4]<=1;',
                 "if(sampled_invalid==16'hffff||invalid_delta>=INVALID_PER_SAMPLE)fault[2]<=1;"):
    assert contract in common,contract
    assert axis.count(contract)==1,contract
assert 'wirerequested_direction=(!duty_permille[15])^drive_invert;' in bridge
assert 'wire[15:0]magnitude=duty_permille[15]?-duty_permille:duty_permille;' in bridge
assert 'requested_direction==active_direction' in bridge
assert 'wirecombined_stop=safety_stop||fault_left!=0||fault_right!=0;' in drive
for side in ('left','right'):
    assert '.drive_invert(drive_invert_'+side+'),.calibration_enable(calibration_enable),.calibration_duty(calibration_duty_'+side+')' in drive
assert codes['actuator_test_control'].count(".drive_invert(1'b0)")==2
assert '9:field_label="FIELD MOTOR DIR ";' in sources['motor_status_text']
assert len('FIELD MOTOR DIR ')==16
# Every instance in the changed backend must name all declared ports. Top
# integration is checked separately after its owner finishes binding ports.
declared={}
for text in sources.values():declared.update(module_ports(text))
for owner,text in sources.items():
    for module,instance,pins in instances(text,declared):
        assert set(pins)==declared[module],(owner,instance,set(pins)^declared[module])
        assert len(pins)==len(set(pins)),(owner,instance,'duplicate port')

# Exhaust every signed16 calibration request at representative full10-bit
# duty limits, especially -32768, zero and the hard 200-permille boundary.
clamp_cases=0
for maximum in (0,1,49,99,100,199,200,201,900,1023):
    limit=min(maximum,200)
    for raw in range(65536):
        signed=raw if raw<32768 else raw-65536
        numerical=max(-limit,min(limit,signed))
        # Full-word signed compare and encoded16 result, independently of
        # the magnitude-based physical specification.
        encoded=(limit if signed>limit else -limit if signed< -limit else signed)&0xffff
        represented=encoded if encoded<32768 else encoded-65536
        assert represented==numerical
        assert abs(represented)<=min(maximum,200)
        clamp_cases+=1

direction_cases=0
for wiring_motor,wiring_encoder,sign in product((-1,1),repeat=3):
    motor_invert=wiring_motor<0
    encoder_invert=wiring_encoder<0
    physical=sign*wiring_motor*(-1 if motor_invert else 1)
    feedback=physical*wiring_encoder*(-1 if encoder_invert else 1)
    assert physical==feedback==sign
    direction_cases+=1
for word,invert in product(range(65536),(False,True)):
    value=word if word<32768 else word-65536
    requested_direction=(not bool(word&0x8000))^invert
    assert requested_direction==((value>=0)^invert)
    assert abs(value)==(word if value>=0 else (-word)&0xffff)
    direction_cases+=1

permission_cases=0
for reset,run,cal,stop,fault in product((False,True),repeat=5):
    physical=reset and (run or cal) and not stop and not fault
    pid=run and not cal and not stop and not fault
    may_apply_direction=not run and not cal
    assert not (physical and may_apply_direction)
    if cal:assert not pid
    if not reset or stop or fault:assert not physical
    permission_cases+=1

# A sampled monotonically increasing invalid count may lag the live count.
# Aligning the idle baseline to sampled_invalid avoids unsigned underflow
# when enable arrives while an older captured sample is still completing.
baseline_cases=0
for baseline,captured,newer_live in ((0,0,1),(2,3,4),(7,7,8),(65534,65535,65535)):
    assert captured>=baseline
    assert (captured-baseline)&0xffffffff==captured-baseline
    if newer_live>captured:assert ((captured-newer_live)&0xffffffff)>=8
    baseline_cases+=1

print(f'PASS: commissioning source/ports; {clamp_cases} signed16 calibration clamps; {direction_cases} motor/encoder polarity cases')
print(f'PASS: UI argument priority, per-wheel direction shortcuts/isolation, {permission_cases} stop/owner truth cases, {baseline_cases} sample-baseline boundaries')
print('NO HDL compilation/simulation, synthesis/resource measurements, timing or hardware validation')
