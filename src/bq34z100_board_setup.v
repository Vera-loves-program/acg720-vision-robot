// Explicit commissioning client for the fabricated two-battery extension board.
// No request is issued until begin_setup. The caller must stop both motors and
// both servos, give this client exclusive ownership of ONE gauge's I2C master,
// and keep the gauge's regulated supply stable throughout commissioning.
//
// Board: BAT divider 240 kohm / 16.5 kohm, 3 series cells, external TS sensor.
// Nominal Voltage Divider = 1000*(240000+16500)/16500 = 15545. A sane divider
// already calibrated for external 3S operation is preserved. This procedure is
// BOARD CONFIGURATION, not current/voltage/temperature calibration or SOC learning.
// CC gain, offsets, ChemID, capacity, Qmax, resistance tables and IT_ENABLE are
// never changed. A B3950 sensor still needs a matching temperature coefficient
// set; TEMPS=1 alone does not calibrate its resistance/temperature curve.
//
// Every DF block is read in full and checksum-validated before editing. Only
// changed bytes and the new checksum are written, then all 32 bytes + checksum
// are reselected/read back. Unrelated and reserved bytes are preserved.
// DF fields are big endian; standard Control() words are little endian.
//
// Low unconfigured Voltage() can block flash writes. TI's documented recovery
// uses CAL_ENABLE / ENTER_CAL to temporarily set Flash Update OK Cell Volt=0.
// Exit CAL and restore/verify its original permission BEFORE changing board
// parameters, so a later CAL toggle cannot undo a newly committed Pack Config.
// The original validated threshold is kept in RAM and restored/read back after
// board writes and before resetting or reporting success. A restore failure
// latches restore_required; another begin_setup rechecks identity, unseals, enters
// CAL if needed, and retries CLEANUP ONLY. Own CAL/sealing cleanup failures also
// retain retry markers. E9 means cleanup recovered (run setup again afterward).
// No automatic flash retries occur.
// A power interruption cannot be repaired from this volatile recovery copy.
// A separate reset_needed marker is set BEFORE any possible divider/config
// checksum commit. Lost write ACK/readback therefore cannot discard the need to
// apply committed flash. Reset is permitted only after both board blocks were
// checksum/readback-verified in this attempt. Already-correct DF with stale live
// registers gets at most one reset per explicit setup, including after FPGA reset.
//
// Delays: >=40 ms DF selection, >=110 ms data write, >=300 ms checksum commit,
// >=100 ms control commands (CAL commands >=300 ms), 1000 ms after reset.
// Final full flash audit, 2 s settling, and three CS/PC/voltage checks separated
// by >=1 s must pass. Host stability policy: 8..14 V, span <=STABILITY_MV.
// These delays/ranges are host policy, not a guarantee of voltage calibration.
// No writes on power-up/polling. Read-only diagnostic outputs retain the last
// checksum-valid DF values; diagnostic_valid marks the completed final audit.
// Security: only default unseal keys are attempted, once. Original sealed state
// is restored. Already-active calibration and unknown dividers/cell counts abort.
//
// error_code: 00 success; 01..05 I2C; E1 device mismatch; E2 DF checksum;
// E3 full-block readback mismatch; E4 calibration already active/not enabled;
// E5 unknown divider; E6 unknown cell count/invalid threshold; E7 unseal failed;
// E8 live activity/config/voltage/stability check; E9 cleanup recovered, run setup again;
// F1 threshold restore failed (restore_required=1); F2 CAL exit failed;
// F3 original sealed state could not be restored. Fresh threshold zero is E6:
// after a power interruption its prior protection value is unknowable; do not
// accept disabled flash protection or invent a previous value. phase is the FSM.
//
// Sources: TI SLUUBW5 sections 2.1.1, 2.2.33, chapter 7, appendix C;
// https://www.ti.com/lit/ug/sluubw5/sluubw5.pdf
// https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/777063/bq34z100evm-not-able-to-access-change-registers-in-bqstudio--volsel
// https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/736380/bq34z100-g1-configuring-sleep-full-sleep-modes
module bq34z100_board_setup #(
    parameter integer CLK_HZ=50000000,
    parameter integer STABILITY_MV=250
)(
    input wire clk, reset_n, begin_setup,
    output reg busy, done,
    output reg [7:0] error_code,
    output wire [7:0] phase,
    output reg restore_required,
    output reg [15:0] last_df_pack,last_divider,last_control,last_voltage,firmware_version,
    output reg [7:0] last_cells,failure_phase,
    output reg diagnostic_valid,
    output reg request_start,
    output reg [7:0] request_register,
    output reg request_read,
    output reg [5:0] request_read_count,
    output reg [1:0] request_write_count,
    output reg [15:0] request_write_data,
    input wire master_busy, master_done,
    input wire [7:0] master_error,
    input wire [255:0] master_read_data
);
    localparam [7:0] IDLE=0, TYPE_COMMAND=1, TYPE_READ=2, TYPE_CHECK=3,
        STATUS_COMMAND=4, STATUS_READ=5, STATUS_CHECK=6,
        UNSEAL_LOW=7, UNSEAL_HIGH=8, UNSEAL_STATUS=9,
        UNSEAL_READ=10, UNSEAL_CHECK=11,
        GET_POWER=12, SAVE_POWER=13, GET_CAL=14, SAVE_CAL=15,
        GET_CONFIG=16, CHECK_CONFIG=17, CAL_ENABLE=18, CAL_ENTER=19,
        CAL_STATUS=20, CAL_READ=21, CAL_CHECK=22, ZERO_THRESHOLD=23,
        WRITE_DIVIDER=24, WRITE_CONFIG=25, CONFIGURED=26,
        FAILURE=27, CLEAN_START=28, RESTORE_OK=29, RESTORE_FAIL=30,
        CLEAN_EXIT=31, EXIT_FAIL=32, CLEAN_STATUS=33, CLEAN_READ=34,
        CLEAN_CHECK=35, CLEAR_STATUS=36, CLEAR_READ=37, CLEAR_CHECK=38,
        CAL_CLEAN_FAIL=39, RESET_OR_SEAL=40, RESET_GAUGE=41,
        POST_STATUS=42, POST_READ=43, POST_CHECK=44,
        POST_CONFIG=45, POST_CONFIG_CHECK=46, POST_VOLTAGE=47,
        POST_VOLTAGE_CHECK=48, RESEAL=49, SEAL_STATUS=50,
        SEAL_READ=51, SEAL_CHECK=52, SEAL_FAIL=53, FINISH=54,
        PRESEAL_READ=55, PRESEAL_CHECK=56, RECOVER_CAL=57,
        FW_COMMAND=58, FW_READ=59, FW_CHECK=60,
        DF_CONTROL=64, DF_CLASS=65, DF_BLOCK=66, DF_READ=67,
        DF_SAVE=68, DF_SUM=69, DF_CHECKSUM_READ=70, DF_VALIDATE=71,
        DF_PREPARE=72, DF_SCAN=73, DF_NEXT=74, DF_COMMIT=75,
        DF_VERIFY_SELECT=76,
        BUS_ISSUE=80, BUS_WAIT=81, BUS_DELAY=82,
        CAL_PERMISSION_READ=83, CAL_PERMISSION_CHECK=84,
        PERMISSION_RESTORE=85, PERMISSION_READ=86, PERMISSION_CHECK=87,
        PERMISSION_VERIFY_READ=88, PERMISSION_VERIFY_CHECK=89, PERMISSION_FAIL=90,
        FINAL_ACCESS=91, FINAL_ACCESS_READ=92, FINAL_ACCESS_CHECK=93,
        FINAL_CAL=94, FINAL_CAL_CHECK=95, FINAL_CONFIG=96, FINAL_CONFIG_CHECK=97,
        FINAL_POWER=98, FINAL_POWER_CHECK=99,
        STABLE_CONTROL=100, STABLE_CONTROL_READ=101, STABLE_CONTROL_CHECK=102,
        PERMISSION_STATUS=103, PERMISSION_STATUS_READ=104, PERMISSION_STATUS_CHECK=105,
        PERMISSION_DF_READ=106, PERMISSION_DF_CHECK=107,
        CHECK_CONFIG_POLICY=108, AUDIT_STORE=109, AUDIT_READ=110,
        AUDIT_COMPARE=111, FINAL_POWER_OK=112;
    localparam [2:0] DF_ONLY_READ=0, DF_THRESHOLD_ZERO=1,
        DF_DIVIDER=2, DF_CONFIG=3, DF_THRESHOLD_RESTORE=4;
    localparam integer MS_CYCLES=(CLK_HZ+999)/1000;
    reg [7:0] state, bus_return, bus_failure, df_return, df_failure;
    reg [31:0] bus_delay_cycles, delay_count;
    // Ordinary commands return at most one word. The full 32-byte DF read is
    // captured directly into block_data when its bus transaction completes.
    reg [15:0] response;
    reg [255:0] block_data, expected_data;
    // Keep exact final audit snapshots in synchronous RAM rather than three
    // 256-bit flip-flop banks and three parallel 256-bit comparators. Banks
    // 0/1/2 are calibration/configuration/power; bank 3 is never used. A bank
    // becomes valid only after all 32 bytes have been written in this attempt.
    (* syn_ramstyle = "block_ram" *) reg [7:0] audit_bytes [0:127];
    reg [7:0] audit_byte, audit_return, audit_failure_phase;
    reg [1:0] audit_bank;
    reg [2:0] audit_valid;
    reg df_readback_match,audit_from_expected;
    reg [7:0] df_class, df_sum;
    reg [5:0] df_index;
    reg [2:0] df_operation;
    reg df_verifying;
    reg was_sealed, known_sealed, cal_touched, saved_threshold_valid, security_pending, device_verified;
    reg reset_needed, configuration_verified, reset_attempted, recovery_only;
    reg original_cal_permission,permission_changed,permission_cleanup_attempted,threshold_cleanup_attempted,board_write_pending;
    reg [7:0] unseal_return;
    reg [3:0] transition_attempts;
    reg [1:0] stable_samples;
    reg [15:0] stable_min,stable_max;
    reg [15:0] saved_threshold, saved_divider, selected_divider;

    assign phase=state;

    // A five-bit byte index makes the two existing work-buffer selectors
    // explicitly 32:1. Reuse them for checksum, writes and bytewise audits.
    wire [7:0] block_byte=block_data[{df_index[4:0],3'b000} +:8];
    wire [7:0] expected_byte=expected_data[{df_index[4:0],3'b000} +:8];
    wire [7:0] audit_write_byte=audit_from_expected ? expected_byte : block_byte;
    wire [7:0] comparison_byte=state==AUDIT_COMPARE ? audit_byte : expected_byte;
    wire byte_equal=block_byte==comparison_byte;
    wire [255:0] modified_data=modified_block(block_data,df_operation,selected_divider,saved_threshold);
    reg block_changes;
    always @* begin
        case (df_operation)
            DF_THRESHOLD_ZERO:block_changes=block_data[15:0]!=16'd0;
            DF_THRESHOLD_RESTORE:block_changes={block_data[7:0],block_data[15:8]}!=saved_threshold;
            DF_DIVIDER:block_changes={block_data[119:112],block_data[127:120]}!=selected_divider;
            DF_CONFIG:block_changes=!block_data[3] || !block_data[8] || block_data[63:56]!=8'd3;
            default:block_changes=0;
        endcase
    end

    // NORMAL single-port RAM template: a write and a read never occur on the
    // same clock. No reset touches RAM contents; audit_valid owns their lifetime.
    always @(posedge clk) begin
        if (state==AUDIT_STORE)
            audit_bytes[{audit_bank,df_index[4:0]}]<=audit_write_byte;
        else audit_byte<=audit_bytes[{audit_bank,df_index[4:0]}];
    end

    // Called only by this FSM. Request values are stable for an entire cycle
    // before BUS_ISSUE pulses start, and remain stable through master completion.
    task issue_read;
        input [7:0] address;
        input [5:0] count;
        input [7:0] following, failed;
        begin
            request_register<=address;request_read<=1;request_read_count<=count;
            request_write_count<=0;request_write_data<=0;
            bus_delay_cycles<=0;bus_return<=following;bus_failure<=failed;state<=BUS_ISSUE;
        end
    endtask

    task save_audit;
        input [1:0] bank;
        input from_expected;
        input [7:0] following;
        begin
            audit_from_expected<=from_expected;audit_bank<=bank;audit_return<=following;
            audit_valid[bank]<=0;df_index<=0;state<=AUDIT_STORE;
        end
    endtask
    task check_audit;
        input [1:0] bank;
        input [7:0] following, failed_phase;
        begin
            if (!audit_valid[bank]) begin
                error_code<=8'he3;failure_phase<=failed_phase;state<=FAILURE;
            end else begin
                audit_bank<=bank;audit_return<=following;
                audit_failure_phase<=failed_phase;df_index<=0;state<=AUDIT_READ;
            end
        end
    endtask
    task issue_write;
        input [7:0] address;
        input [1:0] count;
        input [15:0] value;
        input integer wait_ms;
        input [7:0] following, failed;
        begin
            request_register<=address;request_read<=0;request_read_count<=0;
            request_write_count<=count;request_write_data<=value;
            bus_delay_cycles<=MS_CYCLES*wait_ms;
            bus_return<=following;bus_failure<=failed;state<=BUS_ISSUE;
        end
    endtask
    task control_word;
        input [15:0] command;
        input [7:0] following, failed;
        begin issue_write(8'h00,2,command,100,following,failed);end
    endtask
    // These are host policy delays, not claimed TI transition time limits.
    task pause_ms;
        input integer wait_ms;
        input [7:0] following;
        begin
            bus_delay_cycles<=MS_CYCLES*wait_ms;
            delay_count<=MS_CYCLES*wait_ms;
            bus_return<=following;state<=BUS_DELAY;
        end
    endtask
    task data_flash;
        input [7:0] subclass;
        input [2:0] operation;
        input [7:0] following, failed;
        begin
            df_class<=subclass;df_operation<=operation;df_return<=following;
            df_failure<=failed;df_verifying<=0;state<=DF_CONTROL;
        end
    endtask

    function [255:0] modified_block;
        input [255:0] original;
        input [2:0] operation;
        input [15:0] divider, threshold;
        reg [255:0] result;
        begin
            result=original;
            case (operation)
                DF_THRESHOLD_ZERO:begin result[7:0]=0;result[15:8]=0;end
                DF_THRESHOLD_RESTORE:begin result[7:0]=threshold[15:8];result[15:8]=threshold[7:0];end
                DF_DIVIDER:begin result[119:112]=divider[15:8];result[127:120]=divider[7:0];end
                DF_CONFIG:begin
                    result[7:0]=original[7:0]|8'h08;  // VOLTSEL, overall bit 11
                    result[15:8]=original[15:8]|8'h01; // TEMPS, overall bit 0
                    result[63:56]=8'd3;               // Number of Series Cells
                end
                default:begin end
            endcase
            modified_block=result;
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE;busy<=0;done<=0;error_code<=0;restore_required<=0;
            request_start<=0;request_register<=0;request_read<=1;
            request_read_count<=0;request_write_count<=0;request_write_data<=0;
            bus_return<=IDLE;bus_failure<=FAILURE;bus_delay_cycles<=0;delay_count<=0;
            response<=0;block_data<=0;expected_data<=0;df_class<=0;df_sum<=0;df_index<=0;
            audit_return<=IDLE;audit_failure_phase<=0;audit_bank<=0;audit_valid<=0;df_readback_match<=0;audit_from_expected<=0;
            df_return<=IDLE;df_failure<=FAILURE;df_operation<=DF_ONLY_READ;df_verifying<=0;
            was_sealed<=0;known_sealed<=0;cal_touched<=0;saved_threshold_valid<=0;security_pending<=0;device_verified<=0;
            reset_needed<=0;configuration_verified<=0;reset_attempted<=0;recovery_only<=0;
            original_cal_permission<=0;permission_changed<=0;permission_cleanup_attempted<=0;threshold_cleanup_attempted<=0;board_write_pending<=0;
            unseal_return<=GET_POWER;transition_attempts<=0;stable_samples<=0;stable_min<=0;stable_max<=0;
            last_df_pack<=0;last_divider<=0;last_cells<=0;last_control<=0;last_voltage<=0;
            firmware_version<=0;failure_phase<=0;diagnostic_valid<=0;
            saved_threshold<=0;saved_divider<=0;selected_divider<=0;
        end else begin
            request_start<=0;done<=0;
            case (state)
                IDLE:if (begin_setup) begin
                    busy<=1;error_code<=0;failure_phase<=0;diagnostic_valid<=0;
                    configuration_verified<=0;reset_attempted<=0;device_verified<=0;
                    permission_cleanup_attempted<=0;threshold_cleanup_attempted<=0;board_write_pending<=0;stable_samples<=0;
                    audit_valid<=0;
                    // Keep the only valid RAM copy if a prior restore failed.
                    if ((restore_required && saved_threshold_valid) || cal_touched || security_pending || permission_changed) begin
                        recovery_only<=1;state<=TYPE_COMMAND;
                    end
                    else begin
                        recovery_only<=0;was_sealed<=0;known_sealed<=0;cal_touched<=0;
                        saved_threshold_valid<=0;state<=TYPE_COMMAND;
                    end
                end
                TYPE_COMMAND:control_word(16'h0001,TYPE_READ,FAILURE);
                TYPE_READ:issue_read(8'h00,2,TYPE_CHECK,FAILURE);
                TYPE_CHECK:if (response[15:0]!=16'h0100) begin error_code<=8'he1;failure_phase<=TYPE_CHECK;state<=FAILURE;end
                    else begin device_verified<=1;state<=FW_COMMAND;end
                FW_COMMAND:control_word(16'h0002,FW_READ,FAILURE);
                FW_READ:issue_read(8'h00,2,FW_CHECK,FAILURE);
                FW_CHECK:begin firmware_version<=response[15:0];state<=STATUS_COMMAND;end
                STATUS_COMMAND:control_word(16'h0000,STATUS_READ,FAILURE);
                STATUS_READ:issue_read(8'h00,2,STATUS_CHECK,FAILURE);
                STATUS_CHECK:begin
                    if (!recovery_only) was_sealed<=response[13];
                    known_sealed<=response[13];
                    if (recovery_only) cal_touched<=response[12];
                    if (response[12] && !recovery_only) begin error_code<=8'he4;failure_phase<=STATUS_CHECK;state<=FAILURE;end
                    else if (response[13]) begin
                        unseal_return<=recovery_only ? RECOVER_CAL : GET_POWER;state<=UNSEAL_LOW;
                    end
                    else if (recovery_only) state<=RECOVER_CAL;
                    else state<=GET_POWER;
                end
                UNSEAL_LOW:begin
                    if (was_sealed) security_pending<=1;
                    control_word(16'h0414,UNSEAL_HIGH,FAILURE);
                end
                UNSEAL_HIGH:control_word(16'h3672,UNSEAL_STATUS,FAILURE);
                UNSEAL_STATUS:control_word(16'h0000,UNSEAL_READ,FAILURE);
                UNSEAL_READ:issue_read(8'h00,2,UNSEAL_CHECK,FAILURE);
                UNSEAL_CHECK:begin
                    known_sealed<=response[13];
                    if (response[13]) begin error_code<=8'he7;failure_phase<=UNSEAL_CHECK;state<=FAILURE;end
                    else state<=unseal_return;
                end
                GET_POWER:data_flash(8'd68,DF_ONLY_READ,SAVE_POWER,FAILURE);
                SAVE_POWER:begin
                    saved_threshold<={block_data[7:0],block_data[15:8]};
                    if ({block_data[7:0],block_data[15:8]}==0 ||
                        {block_data[7:0],block_data[15:8]}>16'd4200) begin error_code<=8'he6;failure_phase<=SAVE_POWER;state<=FAILURE;end
                    else begin saved_threshold_valid<=1;save_audit(2,0,GET_CAL);end
                end
                GET_CAL:data_flash(8'd104,DF_ONLY_READ,SAVE_CAL,FAILURE);
                SAVE_CAL:begin
                    saved_divider<={block_data[119:112],block_data[127:120]};save_audit(0,0,GET_CONFIG);
                end
                GET_CONFIG:data_flash(8'd64,DF_ONLY_READ,CHECK_CONFIG,FAILURE);
                CHECK_CONFIG:save_audit(1,0,CHECK_CONFIG_POLICY);
                CHECK_CONFIG_POLICY:begin
                    original_cal_permission<=block_data[6];
                    if (block_data[63:56]!=1 && block_data[63:56]!=3) begin error_code<=8'he6;failure_phase<=CHECK_CONFIG;state<=FAILURE;end
                    else if (block_data[3] && block_data[63:56]==3 && saved_divider>=13000 && saved_divider<=18000) begin
                        selected_divider<=saved_divider;
                        if (block_data[8]) begin configuration_verified<=1;state<=CLEAN_START;end
                        else state<=CAL_PERMISSION_READ;
                    end else if (saved_divider==5000 || saved_divider==15545) begin
                        selected_divider<=16'd15545;state<=CAL_PERMISSION_READ;
                    end else begin error_code<=8'he5;failure_phase<=CHECK_CONFIG;state<=FAILURE;end
                end
                // DF CAL_EN (bit 14) permits entry; ControlStatus CALEN (bit
                // 12) means active. Never toggle permission from activity.
                // Preserve the original flash setting, but inspect current RAM
                // permission before a toggle; flash and live RAM may disagree.
                CAL_PERMISSION_READ:issue_read(8'h3a,2,CAL_PERMISSION_CHECK,FAILURE);
                CAL_PERMISSION_CHECK:begin
                    if (response[14]!=original_cal_permission) permission_changed<=1;
                    if (response[14]) state<=CAL_ENTER;
                    else state<=CAL_ENABLE;
                end
                CAL_ENABLE:begin
                    cal_touched<=1;permission_changed<=1;
                    issue_write(8'h00,2,16'h002d,300,CAL_ENTER,FAILURE);
                end
                CAL_ENTER:begin
                    cal_touched<=1;transition_attempts<=0;
                    issue_write(8'h00,2,16'h0081,300,CAL_STATUS,FAILURE);
                end
                CAL_STATUS:control_word(16'h0000,CAL_READ,FAILURE);
                CAL_READ:issue_read(8'h00,2,CAL_CHECK,FAILURE);
                CAL_CHECK:if (!response[12]) begin
                        if (transition_attempts<9) begin transition_attempts<=transition_attempts+1'b1;pause_ms(300,CAL_STATUS);end
                        else begin error_code<=8'he4;failure_phase<=CAL_CHECK;state<=FAILURE;end
                    end
                    else if (recovery_only) state<=CLEAN_START;
                    else state<=ZERO_THRESHOLD;
                RECOVER_CAL:if (!restore_required) state<=CLEAN_START;
                    else state<=CAL_PERMISSION_READ;
                ZERO_THRESHOLD:begin
                    // Mark BEFORE a possible commit so a lost ACK/readback also
                    // enters the restore path; never assume the write failed.
                    restore_required<=1;
                    board_write_pending<=1;
                    data_flash(8'd68,DF_THRESHOLD_ZERO,CLEAN_EXIT,FAILURE);
                end
                WRITE_DIVIDER:data_flash(8'd104,DF_DIVIDER,WRITE_CONFIG,FAILURE);
                WRITE_CONFIG:data_flash(8'd64,DF_CONFIG,CONFIGURED,FAILURE);
                CONFIGURED:begin configuration_verified<=1;board_write_pending<=0;state<=CLEAN_START;end
                FAILURE:begin
                    board_write_pending<=0;
                    if (device_verified) state<=CLEAN_START;
                    else state<=FINISH;
                end
                CLEAN_START:if (restore_required && saved_threshold_valid) begin
                        threshold_cleanup_attempted<=1;
                        data_flash(8'd68,DF_THRESHOLD_RESTORE,RESTORE_OK,RESTORE_FAIL);
                    end
                    else state<=CLEAN_EXIT;
                RESTORE_OK:begin restore_required<=0;state<=CLEAN_EXIT;end
                RESTORE_FAIL:begin error_code<=8'hf1;failure_phase<=RESTORE_FAIL;restore_required<=1;state<=CLEAN_EXIT;end
                CLEAN_EXIT:if (cal_touched) begin
                        transition_attempts<=0;
                        issue_write(8'h00,2,16'h0080,300,CLEAN_STATUS,EXIT_FAIL);
                    end else state<=PERMISSION_RESTORE;
                EXIT_FAIL:begin
                    if (error_code!=8'hf1) begin error_code<=8'hf2;failure_phase<=EXIT_FAIL;end
                    state<=CLEAN_STATUS;
                end
                CLEAN_STATUS:control_word(16'h0000,CLEAN_READ,CAL_CLEAN_FAIL);
                CLEAN_READ:issue_read(8'h00,2,CLEAN_CHECK,CAL_CLEAN_FAIL);
                CLEAN_CHECK:begin
                    known_sealed<=response[13];
                    if (response[12]) begin
                        if (transition_attempts<9) begin transition_attempts<=transition_attempts+1'b1;pause_ms(300,CLEAN_STATUS);end
                        else state<=CAL_CLEAN_FAIL;
                    end else begin cal_touched<=0;state<=PERMISSION_RESTORE;end
                end
                // CAL_ENABLE toggles permission. Restore its original DF value
                // BEFORE any divider/config commit. Read first to handle a
                // missing ACK without blindly toggling it a second time.
                PERMISSION_RESTORE:if (permission_changed) begin
                        if (permission_cleanup_attempted) state<=PERMISSION_FAIL;
                        else begin permission_cleanup_attempted<=1;state<=PERMISSION_READ;end
                    end else if (board_write_pending && error_code==0 && !recovery_only) state<=PERMISSION_STATUS;
                    else state<=RESET_OR_SEAL;
                PERMISSION_READ:issue_read(8'h3a,2,PERMISSION_CHECK,PERMISSION_FAIL);
                PERMISSION_CHECK:if (response[14]==original_cal_permission) begin
                        permission_changed<=0;
                        state<=PERMISSION_DF_READ;
                    end else issue_write(8'h00,2,16'h002d,300,PERMISSION_VERIFY_READ,PERMISSION_FAIL);
                PERMISSION_VERIFY_READ:issue_read(8'h3a,2,PERMISSION_VERIFY_CHECK,PERMISSION_FAIL);
                PERMISSION_VERIFY_CHECK:if (response[14]!=original_cal_permission) state<=PERMISSION_FAIL;
                    else begin
                        permission_changed<=0;
                        state<=PERMISSION_DF_READ;
                    end
                PERMISSION_DF_READ:data_flash(8'd64,DF_ONLY_READ,PERMISSION_DF_CHECK,PERMISSION_FAIL);
                PERMISSION_DF_CHECK:if (block_data[6]!=original_cal_permission) begin
                        permission_changed<=1;state<=PERMISSION_FAIL;
                    end else state<=PERMISSION_STATUS;
                PERMISSION_STATUS:control_word(16'h0000,PERMISSION_STATUS_READ,PERMISSION_FAIL);
                PERMISSION_STATUS_READ:issue_read(8'h00,2,PERMISSION_STATUS_CHECK,PERMISSION_FAIL);
                PERMISSION_STATUS_CHECK:if (response[12]) begin cal_touched<=1;state<=CLEAN_EXIT;end
                    else if (board_write_pending && error_code==0 && !recovery_only) state<=WRITE_DIVIDER;
                    else state<=RESET_OR_SEAL;
                PERMISSION_FAIL:begin
                    if (error_code!=8'hf1) begin error_code<=8'hf2;failure_phase<=PERMISSION_FAIL;end
                    board_write_pending<=0;
                    // A normal bootstrap still has a temporary zero threshold.
                    // Restore that once before ending with the permission fault.
                    if (restore_required && saved_threshold_valid && !threshold_cleanup_attempted) begin
                        threshold_cleanup_attempted<=1;
                        data_flash(8'd68,DF_THRESHOLD_RESTORE,RESTORE_OK,RESTORE_FAIL);
                    end
                    else state<=RESEAL;
                end
                CAL_CLEAN_FAIL:begin
                    if (error_code!=8'hf1) begin error_code<=8'hf2;failure_phase<=CAL_CLEAN_FAIL;end
                    board_write_pending<=0;
                    if (restore_required && saved_threshold_valid && !threshold_cleanup_attempted) begin
                        threshold_cleanup_attempted<=1;
                        data_flash(8'd68,DF_THRESHOLD_RESTORE,RESTORE_OK,RESTORE_FAIL);
                    end
                    else state<=RESEAL;
                end
                RESET_OR_SEAL:if (error_code==0 && !restore_required && !cal_touched && !permission_changed) begin
                        if (reset_needed && configuration_verified && !reset_attempted && !recovery_only) state<=RESET_GAUGE;
                        else if (!recovery_only) state<=FINAL_ACCESS;
                        else state<=RESEAL;
                    end else if (restore_required && saved_threshold_valid && !threshold_cleanup_attempted) state<=CLEAN_START;
                    else state<=RESEAL;
                // RESET is UNSEALED-only. TI says reset can automatically
                // reseal a previously sealed device; inspect before resealing.
                RESET_GAUGE:begin
                    reset_attempted<=1;
                    issue_write(8'h00,2,16'h0041,1000,POST_STATUS,FAILURE);
                end
                POST_STATUS:control_word(16'h0000,POST_READ,FAILURE);
                POST_READ:issue_read(8'h00,2,POST_CHECK,FAILURE);
                POST_CHECK:begin
                    known_sealed<=response[13];
                    if (response[12]) begin cal_touched<=1;error_code<=8'he8;failure_phase<=POST_CHECK;state<=FAILURE;end
                    else if (response[13]) begin unseal_return<=FINAL_CAL;state<=UNSEAL_LOW;end
                    else state<=FINAL_CAL;
                end
                FINAL_ACCESS:control_word(16'h0000,FINAL_ACCESS_READ,FAILURE);
                FINAL_ACCESS_READ:issue_read(8'h00,2,FINAL_ACCESS_CHECK,FAILURE);
                FINAL_ACCESS_CHECK:if (response[12]) begin cal_touched<=1;error_code<=8'he8;failure_phase<=FINAL_ACCESS_CHECK;state<=FAILURE;end
                    else if (response[13]) begin unseal_return<=FINAL_CAL;state<=UNSEAL_LOW;end
                    else state<=FINAL_CAL;
                // Verify the FINAL flash, after every CAL toggle and RESET.
                // The earlier staged readback alone cannot certify persistence.
                FINAL_CAL:data_flash(8'd104,DF_ONLY_READ,FINAL_CAL_CHECK,FAILURE);
                FINAL_CAL_CHECK:check_audit(0,FINAL_CONFIG,FINAL_CAL_CHECK);
                FINAL_CONFIG:data_flash(8'd64,DF_ONLY_READ,FINAL_CONFIG_CHECK,FAILURE);
                FINAL_CONFIG_CHECK:check_audit(1,FINAL_POWER,FINAL_CONFIG_CHECK);
                FINAL_POWER:data_flash(8'd68,DF_ONLY_READ,FINAL_POWER_CHECK,FAILURE);
                FINAL_POWER_CHECK:if ({block_data[7:0],block_data[15:8]}==0) begin
                        error_code<=8'he3;failure_phase<=FINAL_POWER_CHECK;state<=FAILURE;
                    end else check_audit(2,FINAL_POWER_OK,FINAL_POWER_CHECK);
                FINAL_POWER_OK:begin
                        diagnostic_valid<=1;stable_samples<=0;
                        // Host policy: allow conversion/mode settling, then
                        // require three independent checks spaced >=1 second.
                        pause_ms(2000,STABLE_CONTROL);
                    end
                STABLE_CONTROL:control_word(16'h0000,STABLE_CONTROL_READ,FAILURE);
                STABLE_CONTROL_READ:issue_read(8'h00,2,STABLE_CONTROL_CHECK,FAILURE);
                STABLE_CONTROL_CHECK:if (response[12]) begin cal_touched<=1;error_code<=8'he8;failure_phase<=STABLE_CONTROL_CHECK;state<=FAILURE;end
                    else state<=POST_CONFIG;
                POST_CONFIG:issue_read(8'h3a,2,POST_CONFIG_CHECK,FAILURE);
                POST_CONFIG_CHECK:if (response[15:0]!=last_df_pack || (response[15:0]&16'h0801)!=16'h0801) begin
                        if (configuration_verified && !reset_attempted) begin reset_needed<=1;state<=RESET_GAUGE;end
                        else begin error_code<=8'he8;failure_phase<=POST_CONFIG_CHECK;state<=FAILURE;end
                    end
                    else state<=POST_VOLTAGE;
                POST_VOLTAGE:issue_read(8'h08,2,POST_VOLTAGE_CHECK,FAILURE);
                POST_VOLTAGE_CHECK:begin
                    if (response[15:0]<8000 || response[15:0]>14000) begin
                        if (configuration_verified && !reset_attempted) begin reset_needed<=1;state<=RESET_GAUGE;end
                        else begin error_code<=8'he8;failure_phase<=POST_VOLTAGE_CHECK;state<=RESEAL;end
                    end else if (stable_samples!=0 &&
                        ((response[15:0]>stable_min && response[15:0]-stable_min>STABILITY_MV) ||
                         (response[15:0]<stable_max && stable_max-response[15:0]>STABILITY_MV))) begin
                        error_code<=8'he8;failure_phase<=POST_VOLTAGE_CHECK;state<=RESEAL;
                    end else if (stable_samples==2) begin reset_needed<=0;state<=RESEAL;end
                    else begin
                        if (stable_samples==0) begin stable_min<=response[15:0];stable_max<=response[15:0];end
                        else begin
                            if (response[15:0]<stable_min) stable_min<=response[15:0];
                            if (response[15:0]>stable_max) stable_max<=response[15:0];
                        end
                        stable_samples<=stable_samples+1'b1;pause_ms(1000,STABLE_CONTROL);
                    end
                end
                // Re-read actual security state: a lost ACK on an unseal or
                // RESET command must not leave an originally sealed part open.
                RESEAL:if (was_sealed) control_word(16'h0000,PRESEAL_READ,SEAL_FAIL);
                    else state<=FINISH;
                PRESEAL_READ:issue_read(8'h00,2,PRESEAL_CHECK,SEAL_FAIL);
                PRESEAL_CHECK:begin
                    known_sealed<=response[13];
                    if (response[13]) begin security_pending<=0;state<=FINISH;end
                    else control_word(16'h0020,SEAL_STATUS,SEAL_FAIL);
                end
                SEAL_STATUS:control_word(16'h0000,SEAL_READ,SEAL_FAIL);
                SEAL_READ:issue_read(8'h00,2,SEAL_CHECK,SEAL_FAIL);
                SEAL_CHECK:begin
                    known_sealed<=response[13];
                    if (!response[13]) state<=SEAL_FAIL;
                    else begin security_pending<=0;state<=FINISH;end
                end
                SEAL_FAIL:begin
                    security_pending<=was_sealed;
                    if (error_code!=8'hf1 && error_code!=8'hf2) begin error_code<=8'hf3;failure_phase<=SEAL_FAIL;end
                    state<=FINISH;
                end
                FINISH:begin
                    if (recovery_only && error_code==0) error_code<=8'he9;
                    busy<=0;done<=1;state<=IDLE;
                end

                DF_CONTROL:issue_write(8'h61,1,16'h0000,40,DF_CLASS,df_failure);
                DF_CLASS:issue_write(8'h3e,1,{8'h00,df_class},40,DF_BLOCK,df_failure);
                DF_BLOCK:issue_write(8'h3f,1,16'h0000,40,DF_READ,df_failure);
                DF_READ:issue_read(8'h40,32,DF_SAVE,df_failure);
                DF_SAVE:begin df_sum<=0;df_index<=0;df_readback_match<=1;state<=DF_SUM;end
                DF_SUM:begin
                    df_sum<=df_sum+block_byte;
                    if (df_verifying && !byte_equal) df_readback_match<=0;
                    if (df_index==31) state<=DF_CHECKSUM_READ;
                    else df_index<=df_index+1'b1;
                end
                DF_CHECKSUM_READ:issue_read(8'h60,1,DF_VALIDATE,df_failure);
                DF_VALIDATE:if (response[7:0]!=(8'hff-df_sum)) begin
                    if (error_code==0) begin error_code<=8'he2;failure_phase<=DF_VALIDATE;end
                    state<=df_failure;
                end else begin
                    case (df_class)
                        8'd64:begin last_df_pack<={block_data[7:0],block_data[15:8]};last_cells<=block_data[63:56];end
                        8'd104:last_divider<={block_data[119:112],block_data[127:120]};
                        default:begin end
                    endcase
                    if (df_verifying) begin
                        if (!df_readback_match) begin
                            if (error_code==0) begin error_code<=8'he3;failure_phase<=DF_VALIDATE;end
                            state<=df_failure;
                        end else state<=df_return;
                    end else if (df_operation==DF_ONLY_READ) state<=df_return;
                    else state<=DF_PREPARE;
                end
                DF_PREPARE:begin
                    expected_data<=modified_data;df_sum<=0;df_index<=0;
                    if (df_operation==DF_DIVIDER)
                        save_audit(0,1,block_changes ? DF_SCAN : df_return);
                    else if (df_operation==DF_CONFIG)
                        save_audit(1,1,block_changes ? DF_SCAN : df_return);
                    else if (!block_changes) state<=df_return;
                    else state<=DF_SCAN;
                end
                DF_SCAN:if (df_index==32) state<=DF_COMMIT;
                    else begin
                        df_sum<=df_sum+expected_byte;
                        if (!byte_equal)
                            issue_write(8'h40+df_index,1,{8'h00,expected_byte},110,DF_NEXT,df_failure);
                        else df_index<=df_index+1'b1;
                    end
                DF_NEXT:begin df_index<=df_index+1'b1;state<=DF_SCAN;end
                DF_COMMIT:begin
                    diagnostic_valid<=0;
                    // A lost ACK after this request cannot establish whether
                    // flash committed. Retain apply-needed across all retries.
                    if ((df_class==104 && df_operation==DF_DIVIDER) ||
                        (df_class==64 && df_operation==DF_CONFIG)) reset_needed<=1;
                    issue_write(8'h60,1,{8'h00,(8'hff-df_sum)},300,DF_VERIFY_SELECT,df_failure);
                end
                DF_VERIFY_SELECT:begin df_verifying<=1;state<=DF_CONTROL;end

                AUDIT_STORE:if (df_index==31) begin
                        audit_valid[audit_bank]<=1;df_index<=0;state<=audit_return;
                    end else df_index<=df_index+1'b1;
                // One clock presents the synchronous RAM address, the next
                // compares its byte. All 32 bytes must match before success.
                AUDIT_READ:state<=AUDIT_COMPARE;
                AUDIT_COMPARE:if (!byte_equal) begin
                        error_code<=8'he3;failure_phase<=audit_failure_phase;state<=FAILURE;
                    end else if (df_index==31) state<=audit_return;
                    else begin df_index<=df_index+1'b1;state<=AUDIT_READ;end

                BUS_ISSUE:if (!master_busy) begin request_start<=1;state<=BUS_WAIT;end
                BUS_WAIT:if (master_done) begin
                    if (master_error!=0) begin
                        if (error_code==0) begin error_code<=master_error;failure_phase<=bus_return;end
                        state<=bus_failure;
                    end else begin
                        response<=master_read_data[15:0];delay_count<=bus_delay_cycles;
                        if (request_read && request_register==8'h40 && request_read_count==32)
                            block_data<=master_read_data;
                        if (request_read && request_register==8'h00 && request_read_count==2 &&
                            bus_return!=TYPE_CHECK && bus_return!=FW_CHECK) last_control<=master_read_data[15:0];
                        if (request_read && request_register==8'h08 && request_read_count==2) last_voltage<=master_read_data[15:0];
                        if (bus_delay_cycles==0) state<=bus_return;
                        else state<=BUS_DELAY;
                    end
                end
                // Count down to a zero detect rather than feeding a 32-bit
                // variable subtract/compare back through the FSM and counter
                // enable. N loaded clocks add one conservative clock before
                // return, so none of the specified minimum waits get shorter.
                BUS_DELAY:if (delay_count==0) state<=bus_return;
                    else delay_count<=delay_count-1'b1;
                default:begin error_code<=8'he8;failure_phase<=state;state<=FAILURE;end
            endcase
        end
    end
endmodule
