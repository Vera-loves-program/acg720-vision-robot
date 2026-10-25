"""BQ bytewise block-audit equivalence and static resource contracts.

These are independent byte/protocol calculations and source guards, not HDL
compilation, elaboration, HDL simulation, I2C activity or a flash-write test.
"""
from pathlib import Path
import random
import re
from check_motor_integration import uncomment, check_blocks

ROOT=Path(__file__).resolve().parents[1]
source=uncomment((ROOT/'src/bq34z100_board_setup.v').read_text(encoding='utf-8-sig'))
code=re.sub(r'\s+','',source)
check_blocks(source,'bq34z100_board_setup')

# Keep exact bytes, not a digest; retain one narrow shared equality comparator
# for unchanged-byte writes, checksum-verified readback and the final audits.
assert 'reg[15:0]response;' in code
assert 'reg[255:0]block_data,expected_data;' in code
assert 'reg[7:0]audit_bytes[0:127];' in code
assert 'wirebyte_equal=block_byte==comparison_byte;' in code
assert 'state==AUDIT_COMPARE?audit_byte:expected_byte' in code
for operand in ('block_data','expected_data'):
    assert operand+"[{df_index[4:0],3'b000}+:8]" in code
assert 'block_data!=expected_data' not in code
assert 'modified_block(block_data,df_operation,selected_divider,saved_threshold)==block_data' not in code
for bank in ('final_cal_expected','final_config_expected','final_power_expected'):
    assert bank not in source
assert 'response<=master_read_data[15:0];' in code
assert "if(request_read&&request_register==8'h40&&request_read_count==32)block_data<=master_read_data;" in code
assert 'DF_SAVE:begindf_sum<=0;df_index<=0;df_readback_match<=1;state<=DF_SUM;' in code
assert 'if(df_verifying&&!byte_equal)df_readback_match<=0;' in code
assert "DF_VALIDATE:if(response[7:0]!=(8'hff-df_sum))" in code
assert "if(!df_readback_match)beginif(error_code==0)beginerror_code<=8'he3;failure_phase<=DF_VALIDATE;" in code
assert 'if(state==AUDIT_STORE)audit_bytes[{audit_bank,df_index[4:0]}]<=audit_write_byte;elseaudit_byte<=audit_bytes[{audit_bank,df_index[4:0]}];' in code
assert 'AUDIT_READ:state<=AUDIT_COMPARE;' in code
assert 'audit_valid[bank]<=0;df_index<=0;state<=AUDIT_STORE;' in code
assert 'audit_valid[audit_bank]<=1;df_index<=0;state<=audit_return;' in code
assert 'if(!audit_valid[bank])begin' in code
assert 'failure_phase<=audit_failure_phase;state<=FAILURE;' in code
assert len(re.findall(r'\baudit_bytes\s*\[',source))==3

# The original threshold and cleanup/reseal markers remain in independent
# registers; audit RAM is never the recovery copy used to restore protection.
for guard in (
    'restore_required&&saved_threshold_valid',
    "data_flash(8'd68,DF_THRESHOLD_RESTORE,RESTORE_OK,RESTORE_FAIL)",
    'restore_required<=1;',
    'saved_threshold<={block_data[7:0],block_data[15:8]};',
    'permission_cleanup_attempted',
    'threshold_cleanup_attempted',
    'security_pending<=was_sealed;',
    "issue_write(8'h00,2,16'h0041,1000,POST_STATUS,FAILURE)",
    "issue_write(8'h60,1,{8'h00,(8'hff-df_sum)},300,DF_VERIFY_SELECT,df_failure)",
    'pause_ms(2000,STABLE_CONTROL)',
    'pause_ms(1000,STABLE_CONTROL)',
    'stable_samples==2',
    'diagnostic_valid<=1;',
):
    assert guard in code,guard

def modify(original,operation,divider,threshold):
    """Old whole-block policy, expressed in wire byte order."""
    result=bytearray(original)
    if operation==1:
        result[0:2]=bytes(2)
    elif operation==4:
        result[0:2]=threshold.to_bytes(2,'big')
    elif operation==2:
        result[14:16]=divider.to_bytes(2,'big')
    elif operation==3:
        result[0]|=8
        result[1]|=1
        result[7]=3
    return bytes(result)

def changed_fields(original,operation,divider,threshold):
    """Narrow field checks replacing a 256-bit modified-block compare."""
    if operation==1:
        return original[0:2]!=bytes(2)
    if operation==4:
        return int.from_bytes(original[0:2],'big')!=threshold
    if operation==2:
        return int.from_bytes(original[14:16],'big')!=divider
    if operation==3:
        return not (original[0]&8) or not (original[1]&1) or original[7]!=3
    return False

def checksum(data):
    return (255-sum(data))&255

def streamed_checksum(data):
    acc=0
    for byte in data:
        acc=(acc+byte)&255
    return (255-acc)&255

def streamed_match(actual,expected):
    equal=True
    for byte,wanted in zip(actual,expected,strict=True):
        equal=equal and byte==wanted
    return equal

rng=random.Random(0x34A0D17)
cases=0
for iteration in range(4096):
    original=bytes(rng.randrange(256) for _ in range(32))
    divider=rng.randrange(65536)
    threshold=rng.randrange(65536)
    packed=int.from_bytes(original,'little')
    for index in range(32):
        assert ((packed>>(index<<3))&255)==original[index]
    assert checksum(original)==streamed_checksum(original)
    for operation in range(8):
        expected=modify(original,operation,divider,threshold)
        assert changed_fields(original,operation,divider,threshold)==(expected!=original)
        assert checksum(expected)==streamed_checksum(expected)
        old_writes=[(0x40+i,expected[i],110) for i in range(32) if expected[i]!=original[i]]
        byte_writes=[(0x40+i,byte,110) for i,byte in enumerate(expected) if byte!=original[i]]
        assert old_writes==byte_writes
        assert streamed_match(expected,expected)
        # Every position must participate in comparison, even if the corrupted
        # data has a recomputed checksum that passes the separate DF CRC step.
        if iteration<64:
            for index in range(32):
                corrupt=bytearray(expected)
                corrupt[index]^=1<<(iteration%8)
                assert (checksum(corrupt)+sum(corrupt))&255==255
                assert not streamed_match(corrupt,expected)
        cases+=1
    # Equal-sum corruption is undetectable by the BQ sum alone, but must still
    # be rejected by the exact-byte readback/final audit comparison.
    corrupt=bytearray(original)
    corrupt[0]=(corrupt[0]+1)&255
    corrupt[31]=(corrupt[31]-1)&255
    assert checksum(corrupt)==checksum(original)
    assert not streamed_match(corrupt,original)

    # The three exact snapshots survive unrelated block reads and edits. A
    # fresh/partial bank cannot be accepted until all 32 addresses are valid.
    snapshots=[original,modify(original,3,divider,threshold),modify(original,4,divider,threshold)]
    audit_mem=[None]*128
    valid=[False]*3
    for bank,block in enumerate(snapshots):
        for index,byte in enumerate(block):
            assert not valid[bank]
            audit_mem[(bank<<5)|index]=byte
        valid[bank]=True
    for bank,block in enumerate(snapshots):
        assert valid[bank]
        restored=bytes(audit_mem[(bank<<5)|index] for index in range(32))
        assert restored==block and streamed_match(block,restored)
    assert all(byte is None for byte in audit_mem[96:])

# Source storage accounting only: not a synthesized flip-flop/LUT estimate.
removed_work_bits=3*256+(256-16)
added_control_bits=8+8+8+2+3+1+1
assert removed_work_bits-added_control_bits==977
assert 32/50_000_000==0.00000064
assert 64/50_000_000==0.00000128
print(f'PASS: {cases} whole-block/field/write-list equivalence cases; exact audits reject every byte and equal-sum corruption')
print('PASS: three 32-byte RAM snapshots, valid-after-completion, fixed recovery registers and unchanged flash delays')
print('NO HDL compilation/simulation, inferred-RAM verification, I2C, flash configuration or timing/resource validation')
