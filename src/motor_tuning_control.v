// Central settings: LCD and UART share ONE owner, no duplicate gain registers.
// LCD retains direction/JOG; VOFA may update gains and setpoints live.
// Units: encoder counts/10ms, absolute encoder counts, PWM permille; gains Q8.
// Position mode is locked until an explicit stopped UNLOCK POS confirmation.
// That confirmation records user intent, not automatic proof of correct tuning.
module motor_tuning_control #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, safety_stop,
    input wire [7:0] fault_left, fault_right,
    input wire ui_action_valid,
    input wire [7:0] ui_action,
    input wire signed [31:0] ui_argument,
    input wire uart_action_valid,
    input wire [7:0] uart_action,
    input wire signed [31:0] uart_argument,
    input wire uart_heartbeat,uart_live_update,
    output reg armed, running, mode_position, position_unlocked, link_fault,remote_owner,
    output reg clear_fault, clear_stop, zero_position,
    output reg selected_wheel, edit_position,
    output reg [3:0] selected_field,
    output reg [1:0] step_select,
    output reg encoder_invert_left, encoder_invert_right,
    output reg drive_invert_left, drive_invert_right,
    output reg signed [15:0] target_speed_left, target_speed_right,
    output reg signed [31:0] target_position_left, target_position_right,
    output reg [15:0] speed_kp_left, speed_ki_left, speed_kd_left,
    output reg [15:0] speed_kp_right, speed_ki_right, speed_kd_right,
    output reg [15:0] position_kp_left, position_ki_left, position_kd_left,
    output reg [15:0] position_kp_right, position_ki_right, position_kd_right,
    output reg [9:0] max_duty_permille, min_duty_permille,
    output reg [15:0] speed_deadband, position_deadband,
    output reg signed [31:0] selected_value,
    output wire [31:0] selected_step
);
    reg signed [32:0] requested_value;
    reg signed [31:0] bounded_value;
    reg signed [31:0] low_limit, high_limit;
    wire take_action=ui_action_valid || uart_action_valid;
    wire [7:0] act=ui_action_valid ? ui_action : uart_action;
    wire signed [31:0] effective_argument=ui_action_valid ? ui_argument : uart_argument;
    wire from_remote=!ui_action_valid && uart_action_valid;
    wire live_parameter_write=from_remote && uart_live_update && selected_field<=7;
    assign selected_step=selected_field<3 ?
        (step_select==0 ? 32'd1 : step_select==1 ? 32'd16 : 32'd256) :
        (step_select==0 ? 32'd1 : step_select==1 ? 32'd10 : 32'd100);
    wire config_ok=mode_position ?
        (position_unlocked &&
         ((target_speed_left!=0 && (speed_kp_left!=0 || speed_ki_left!=0) && position_kp_left!=0) ||
          (target_speed_right!=0 && (speed_kp_right!=0 || speed_ki_right!=0) && position_kp_right!=0))) :
        ((target_speed_left!=0 && (speed_kp_left!=0 || speed_ki_left!=0)) ||
         (target_speed_right!=0 && (speed_kp_right!=0 || speed_ki_right!=0)));
    always @* begin
        selected_value=0;
        case (selected_field)
            0:selected_value=edit_position ? (selected_wheel ? position_kp_right : position_kp_left) :
                                             (selected_wheel ? speed_kp_right : speed_kp_left);
            1:selected_value=edit_position ? (selected_wheel ? position_ki_right : position_ki_left) :
                                             (selected_wheel ? speed_ki_right : speed_ki_left);
            2:selected_value=edit_position ? (selected_wheel ? position_kd_right : position_kd_left) :
                                             (selected_wheel ? speed_kd_right : speed_kd_left);
            3:begin
                if (edit_position) selected_value=selected_wheel ? target_position_right : target_position_left;
                else selected_value=selected_wheel ? $signed(target_speed_right) : $signed(target_speed_left);
            end
            4:selected_value={22'd0,min_duty_permille};
            5:selected_value={22'd0,max_duty_permille};
            6:selected_value={16'd0,speed_deadband};
            7:selected_value={16'd0,position_deadband};
            8:selected_value=selected_wheel ? {31'd0,encoder_invert_right} : {31'd0,encoder_invert_left};
            9:selected_value=selected_wheel ? {31'd0,drive_invert_right} : {31'd0,drive_invert_left};
            default:selected_value=0;
        endcase
        low_limit=0; high_limit=32767;
        case (selected_field)
            3:begin low_limit=edit_position ? -2000000 : -200; high_limit=-low_limit; end
            4:high_limit={22'd0,max_duty_permille};
            5:begin low_limit=50; high_limit=900; end
            6:high_limit=32;
            7:high_limit=1000;
            8,9:high_limit=1;
            default:begin end
        endcase
        if (act==19) requested_value={effective_argument[31],effective_argument};
        else if (act==5) requested_value={selected_value[31],selected_value}-$signed({1'b0,selected_step});
        else requested_value={selected_value[31],selected_value}+$signed({1'b0,selected_step});
        if (requested_value<$signed({low_limit[31],low_limit})) bounded_value=low_limit;
        else if (requested_value>$signed({high_limit[31],high_limit})) bounded_value=high_limit;
        else bounded_value=requested_value[31:0];
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            armed<=0; running<=0; mode_position<=0; position_unlocked<=0;
            link_fault<=0; clear_fault<=0; clear_stop<=0; zero_position<=0;
            selected_wheel<=0; edit_position<=0; selected_field<=0; step_select<=0;
            // This assembled chassis reports negative counts for physical
            // left-wheel forward motion with the original polarity. Correct
            // feedback only; motor direction and right feedback stay separate.
            encoder_invert_left<=1; encoder_invert_right<=0;
            drive_invert_left<=0; drive_invert_right<=0;
            // Visible initial setpoints for the paired speed dashboard.
            // Measured chassis defaults, Q8: P=30, I=1, D=0.
            // Reset still revokes ARM/RUN; a setpoint never starts the wheels.
            target_speed_left<=10; target_speed_right<=10;
            target_position_left<=0; target_position_right<=0;
            speed_kp_left<=7680; speed_ki_left<=256; speed_kd_left<=0;
            speed_kp_right<=7680; speed_ki_right<=256; speed_kd_right<=0;
            // Outer position loop runs at20Hz; conservative unmeasured seed.
            // P=0.125 COUNT/10ms per COUNT error (Q8=32), I=D=0.
            position_kp_left<=32; position_ki_left<=0; position_kd_left<=0;
            position_kp_right<=32; position_ki_right<=0; position_kd_right<=0;
            max_duty_permille<=600; min_duty_permille<=0;
            speed_deadband<=0; position_deadband<=4;
            remote_owner<=0;
        end else begin
            clear_fault<=0; clear_stop<=0; zero_position<=0;
            // Remote RUN is latched until STOP, a real motor fault or reset.
            // uart_heartbeat remains an unused compatibility input only.
            if (take_action) begin
                case (act)
                    1:selected_wheel<=~selected_wheel;
                    2:edit_position<=~edit_position;
                    3:selected_field<=selected_field==0 ? 9 : selected_field-1'b1;
                    4:selected_field<=selected_field==9 ? 0 : selected_field+1'b1;
                    7:step_select<=step_select==2 ? 0 : step_select+1'b1;
                    16:if (effective_argument==0 || effective_argument==1) selected_wheel<=effective_argument[0];
                    17:if (effective_argument==0 || effective_argument==1) edit_position<=effective_argument[0];
                    18:if (effective_argument>=0 && effective_argument<=9) selected_field<=effective_argument[3:0];
                    20:if (effective_argument>=0 && effective_argument<=2) step_select<=effective_argument[1:0];
                    8:if (!running && !safety_stop && fault_left==0 && fault_right==0 && !link_fault) begin
                        armed<=1; remote_owner<=from_remote;
                    end
                    9:if (armed && !running && !safety_stop && !link_fault &&
                          fault_left==0 && fault_right==0 && config_ok) begin
                        running<=1; remote_owner<=from_remote;
                    end
                    10:begin armed<=0; running<=0; end
                    11:if (!running) begin
                        armed<=0; clear_fault<=1; clear_stop<=1; link_fault<=0;
                    end
                    12:begin armed<=0; running<=0; mode_position<=0; end
                    13:if (!running && position_unlocked) begin armed<=0; mode_position<=1; end
                    14:if (!armed && !running) begin
                        zero_position<=1; target_position_left<=0; target_position_right<=0;
                    end
                    15:if (!armed && !running) position_unlocked<=1;
                    21,22:begin
                        // Local direction shortcuts alter ONE selected bit,
                        // never the other wheel or the other direction type.
                        armed<=0;running<=0;position_unlocked<=0;mode_position<=0;
                        zero_position<=1;target_position_left<=0;target_position_right<=0;
                        selected_field<=act==21 ? 4'd8 : 4'd9;
                        if(act==21)begin
                            if(selected_wheel)encoder_invert_right<=~encoder_invert_right;
                            else encoder_invert_left<=~encoder_invert_left;
                        end else begin
                            if(selected_wheel)drive_invert_right<=~drive_invert_right;
                            else drive_invert_left<=~drive_invert_left;
                        end
                    end
                    23:begin
                        // Isolate a single-wheel test without changing the
                        // selected wheel's speed/position targets or gains.
                        armed<=0;running<=0;
                        if(selected_wheel)begin target_speed_left<=0;target_position_left<=0;end
                        else begin target_speed_right<=0;target_position_right<=0;end
                    end
                    24:begin
                        // One START for the paired speed dashboard. A zero
                        // gain or target is a valid controller setting, not a
                        // permission failure. Initial zero gains produce no drive.
                        // Actual stop/fault conditions retain final priority.
                        armed<=0;running<=0;mode_position<=0;edit_position<=0;
                        position_unlocked<=0;
                        if(!safety_stop && !link_fault && fault_left==0 && fault_right==0)begin
                            armed<=1;running<=1;remote_owner<=from_remote;
                        end
                    end
                    25:if(effective_argument>=0 && effective_argument<=19 &&
                          (effective_argument[3:0]<=3 || effective_argument==5))begin
                        // Direct L/R Kp, Ki, Kd, target or the shared PWM cap.
                        // Selection stops motion without clearing either wheel.
                        armed<=0;running<=0;edit_position<=0;
                        selected_wheel<=effective_argument[4];
                        selected_field<=effective_argument[3:0];
                    end
                    5,6,19:begin
                        // VOFA gain/target writes do not interrupt a running
                        // experiment. Direction edits and all legacy edits
                        // stop first; no parameter write can start motion.
                        if(!live_parameter_write)begin armed<=0;running<=0;end
                        case (selected_field)
                            0:if (edit_position) begin
                                if (selected_wheel) position_kp_right<=bounded_value[15:0];
                                else position_kp_left<=bounded_value[15:0];
                            end else begin
                                if(!live_parameter_write)begin position_unlocked<=0;mode_position<=0;end
                                if (selected_wheel) speed_kp_right<=bounded_value[15:0];
                                else speed_kp_left<=bounded_value[15:0];
                            end
                            1:if (edit_position) begin
                                if (selected_wheel) position_ki_right<=bounded_value[15:0];
                                else position_ki_left<=bounded_value[15:0];
                            end else begin
                                if(!live_parameter_write)begin position_unlocked<=0;mode_position<=0;end
                                if (selected_wheel) speed_ki_right<=bounded_value[15:0];
                                else speed_ki_left<=bounded_value[15:0];
                            end
                            2:if (edit_position) begin
                                if (selected_wheel) position_kd_right<=bounded_value[15:0];
                                else position_kd_left<=bounded_value[15:0];
                            end else begin
                                if(!live_parameter_write)begin position_unlocked<=0;mode_position<=0;end
                                if (selected_wheel) speed_kd_right<=bounded_value[15:0];
                                else speed_kd_left<=bounded_value[15:0];
                            end
                            3:if (edit_position) begin
                                if (selected_wheel) target_position_right<=bounded_value;
                                else target_position_left<=bounded_value;
                            end else begin
                                if (selected_wheel) target_speed_right<=bounded_value[15:0];
                                else target_speed_left<=bounded_value[15:0];
                            end
                            4:min_duty_permille<=bounded_value[9:0];
                            5:begin
                                max_duty_permille<=bounded_value[9:0];
                                if (min_duty_permille>bounded_value[9:0]) min_duty_permille<=bounded_value[9:0];
                            end
                            6:speed_deadband<=bounded_value[15:0];
                            7:position_deadband<=bounded_value[15:0];
                            8:begin
                                position_unlocked<=0; mode_position<=0;
                                zero_position<=1;
                                target_position_left<=0; target_position_right<=0;
                                if (selected_wheel) encoder_invert_right<=bounded_value[0];
                                else encoder_invert_left<=bounded_value[0];
                            end
                            9:begin
                                position_unlocked<=0;mode_position<=0;
                                zero_position<=1;
                                target_position_left<=0;target_position_right<=0;
                                if(selected_wheel)drive_invert_right<=bounded_value[0];
                                else drive_invert_left<=bounded_value[0];
                            end
                            default:begin end
                        endcase
                    end
                    default:begin end
                endcase
            end
            // Last assignments have priority over ARM/RUN on this same clock.
            if (safety_stop || fault_left!=0 || fault_right!=0 || link_fault ||
                (ui_action_valid && ui_action==10) || (uart_action_valid && uart_action==10)) begin
                armed<=0; running<=0;
            end
        end
    end
endmodule

// Serializes VOFA transactions through the existing single settings owner.
// No PID/target/direction setting is duplicated here. Replies sample the
// owner's settled readback, never merely echo an accepted request.
module motor_vofa_settings_adapter #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, safety_stop, core_stop_override,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [15:0] cmd_seq,
    input wire [3:0] cmd_op,
    input wire [1:0] cmd_wheel,
    input wire [3:0] cmd_field,
    input wire signed [31:0] cmd_value,
    output reg rsp_valid,
    input wire rsp_ready,
    output reg [15:0] rsp_seq,
    output reg [3:0] rsp_op,
    output reg [1:0] rsp_wheel,
    output reg [3:0] rsp_field,
    output reg [7:0] rsp_status,
    output reg signed [31:0] rsp_value,
    output reg core_action_valid,
    output reg [7:0] core_action,
    output reg signed [31:0] core_argument,
    output reg core_live_update,
    output reg heartbeat, takeover,
    output wire turn_request,
    output wire signed [15:0] turn_degrees,
    input wire turn_ready, turn_active,
    output wire task_valid,
    output wire [3:0] task_action,
    output wire signed [31:0] task_value,
    input wire task_ready, task_rsp_valid,
    input wire [7:0] task_rsp_status,
    input wire signed [31:0] task_rsp_value,
    input wire core_armed, core_running, core_mode_position, core_link_fault,
    input wire [7:0] core_fault_left, core_fault_right,
    input wire signed [31:0] core_selected_value,
    input wire [9:0] core_min_duty, core_max_duty
);
    localparam [4:0] IDLE=0, SELECT_WHEEL=1, SELECT_LOOP=2,
        SELECT_FIELD=3, CHECK_VALUE=4, WRITE_VALUE=5, RUN_SPEED=6,
        POSITION_STOP=7, POSITION_UNLOCK=8, POSITION_MODE=9,
        POSITION_ARM=10, POSITION_RUN=11, STOP=12,
        CLEAR_STOP=13, CLEAR_FAULT=14, WAIT_ONE=15, WAIT_TWO=16,
        REPLY=17, TURN_STOP=18, TURN_START=19,TASK_SEND=20,TASK_WAIT=21;
    localparam [7:0] OK=0, RANGE_ERROR=2, BLOCKED=4, INTERRUPTED=5,TASK_TIMEOUT=6;
    localparam integer TASK_TIMEOUT_CYCLES=CLK_HZ/10<1 ? 1 : CLK_HZ/10;
    localparam integer TASK_AGE_WIDTH=TASK_TIMEOUT_CYCLES<=1 ? 1 : $clog2(TASK_TIMEOUT_CYCLES);
    reg [4:0] state;
    reg [15:0] held_seq;
    reg [3:0] held_op;
    reg [1:0] held_wheel;
    reg active_wheel, held_position, cancelled, unknown_field;
    reg [3:0] held_field;
    reg signed [31:0] held_value;
    reg signed [31:0] task_result;
    reg [TASK_AGE_WIDTH-1:0] task_age;
    reg [7:0] transaction_error;
    reg value_in_range;
    wire is_set=held_op==2 || held_op==3;
    wire is_get=held_op==4 || held_op==7;
    wire command_wheel_valid=cmd_wheel<=1 ||
        (cmd_wheel==2 && (cmd_op==2 || cmd_op==3) && cmd_field<=7);
    wire blocked_now=safety_stop || core_stop_override || core_link_fault ||
        core_fault_left!=0 || core_fault_right!=0;
    wire cancel_new=state!=IDLE && !rsp_valid && !cancelled &&
        blocked_now && (held_op==1 || held_op==8 || is_set);
    assign cmd_ready=state==IDLE && !rsp_valid;
    assign turn_request=state==TURN_START && !cancelled && !blocked_now && turn_ready;
    assign turn_degrees=held_value[15:0];
    // Task actions are serialized through the same UI owner as LCD presses.
    // They never issue raw wheel RUN/PID actions or take the motor owner.
    // The handler must return one registered acceptance/rejection response;
    // a bounded wait also recovers the UART if that handler is unavailable.
    assign task_valid=state==TASK_SEND;
    assign task_action=held_field;
    assign task_value=held_value;

    // The wire codec has already converted decimal gains into unsigned Q8.
    // Reject values outside the core's actual limits rather than let a
    // clamped write masquerade as a successful requested setting.
    always @* begin
        value_in_range=0;
        case(held_field)
            0,1,2:value_in_range=held_value>=0 && held_value<=32767;
            3:if(held_position)
                value_in_range=held_value>=-2000000 && held_value<=2000000;
              else value_in_range=held_value>=-200 && held_value<=200;
            4:value_in_range=held_value>=0 &&
                held_value<=$signed({22'd0,core_max_duty});
            5:value_in_range=held_value>=50 && held_value<=900 &&
                held_value>=$signed({22'd0,core_min_duty});
            6:value_in_range=held_value>=0 && held_value<=32;
            7:value_in_range=held_value>=0 && held_value<=1000;
            8,9:value_in_range=held_value==0 || held_value==1;
            default:begin end
        endcase
    end

    // Each state exposes one whole-clock action. The receiving owner consumes
    // it on the next edge; explicit wait states precede every reply. Selector
    // actions only change the readback view, never the running loop mode.
    always @* begin
        core_action_valid=1;
        core_action=0;
        core_argument=0;
        core_live_update=0;
        case(state)
            SELECT_WHEEL:begin core_action=16;core_argument={31'd0,active_wheel};end
            SELECT_LOOP:begin core_action=17;core_argument={31'd0,held_position};end
            SELECT_FIELD:begin core_action=18;core_argument={28'd0,held_field};end
            WRITE_VALUE:begin
                core_action=19;core_argument=held_value;
                core_live_update=held_field<=7;
                if(cancelled || blocked_now)core_action_valid=0;
            end
            RUN_SPEED:core_action=24;
            POSITION_STOP,STOP,CLEAR_STOP,TURN_STOP:core_action=10;
            POSITION_UNLOCK:core_action=15;
            POSITION_MODE:core_action=13;
            POSITION_ARM:core_action=8;
            POSITION_RUN:core_action=9;
            CLEAR_FAULT:core_action=11;
            default:core_action_valid=0;
        endcase
        if(held_op==1 && (cancelled || blocked_now))core_action_valid=0;
        if(!core_action_valid)core_live_update=0;
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            state<=IDLE;held_seq<=0;held_op<=0;held_wheel<=0;active_wheel<=0;
            held_position<=0;held_field<=0;held_value<=0;
            task_result<=0;task_age<=0;
            transaction_error<=OK;cancelled<=0;unknown_field<=0;
            rsp_valid<=0;rsp_seq<=0;rsp_op<=0;rsp_wheel<=0;
            rsp_field<=0;rsp_status<=OK;rsp_value<=0;
            heartbeat<=0;takeover<=0;
        end else begin
            heartbeat<=0;takeover<=0;
            if(rsp_valid && rsp_ready)rsp_valid<=0;
            case(state)
                IDLE:if(cmd_valid && cmd_ready)begin
                    held_seq<=cmd_seq;held_op<=cmd_op;held_wheel<=cmd_wheel;
                    active_wheel<=cmd_wheel[0]; // BOTH starts with the left owner bank.
                    held_position<=cmd_op==3 || cmd_op==7;
                    held_field<=cmd_field;held_value<=cmd_value;
                    task_result<=0;task_age<=0;
                    // A one-clock UI STOP may coincide with acceptance. Latch
                    // it now; testing only later states would restart on the
                    // next edge after the user's STOP pulse has disappeared.
                    cancelled<=blocked_now && (cmd_op==1 || cmd_op==8 || cmd_op==2 || cmd_op==3);
                    transaction_error<=blocked_now && (cmd_op==1 || cmd_op==8) ? BLOCKED :
                        blocked_now && (cmd_op==2 || cmd_op==3) ? INTERRUPTED : OK;
                    unknown_field<=0;
                    takeover<=cmd_op==0 || cmd_op==1 || cmd_op==5 || cmd_op==8 ||
                        ((cmd_op==2 || cmd_op==3) &&
                         (cmd_field==8 || cmd_field==9));
                    if(!command_wheel_valid)begin
                        transaction_error<=RANGE_ERROR;unknown_field<=1;
                        takeover<=0;state<=WAIT_ONE;
                    end else begin
                    case(cmd_op)
                        0:state<=STOP;
                        1:state<=blocked_now ? WAIT_ONE : cmd_wheel ? POSITION_STOP : RUN_SPEED;
                        2,3,4,7:if(cmd_field<=9)state<=SELECT_WHEEL;
                            else begin
                                transaction_error<=RANGE_ERROR;
                                unknown_field<=1;state<=WAIT_ONE;
                            end
                        5:state<=CLEAR_STOP;
                        6:state<=WAIT_ONE; // Old heartbeat is an ACKed no-op.
                        8:if(cmd_value==0 || cmd_value< -180 || cmd_value>180 || cmd_wheel || cmd_field!=0)begin
                            transaction_error<=RANGE_ERROR;state<=WAIT_ONE;
                          end else state<=blocked_now ? WAIT_ONE : TURN_STOP;
                        9:if(cmd_wheel!=0 || cmd_field>8 ||
                             ((cmd_field==1 || cmd_field==2) ?
                                (cmd_value< -3900 || cmd_value>3900) : cmd_value!=0))begin
                            transaction_error<=RANGE_ERROR;unknown_field<=1;state<=WAIT_ONE;
                          end else state<=TASK_SEND;
                        default:state<=WAIT_ONE;
                    endcase
                    end
                end
                SELECT_WHEEL:state<=SELECT_LOOP;
                SELECT_LOOP:state<=SELECT_FIELD;
                SELECT_FIELD:state<=CHECK_VALUE;
                CHECK_VALUE:begin
                    if(is_set && !value_in_range)begin
                        if(!cancelled)transaction_error<=RANGE_ERROR;
                        state<=WAIT_ONE;
                    end else if(is_set && !cancelled && !cancel_new)
                        state<=WRITE_VALUE;
                    else state<=WAIT_ONE;
                end
                WRITE_VALUE:state<=WAIT_ONE;
                RUN_SPEED:state<=WAIT_ONE;
                POSITION_STOP:state<=POSITION_UNLOCK;
                POSITION_UNLOCK:state<=POSITION_MODE;
                POSITION_MODE:state<=POSITION_ARM;
                POSITION_ARM:state<=POSITION_RUN;
                POSITION_RUN:state<=WAIT_ONE;
                STOP:state<=WAIT_ONE;
                CLEAR_STOP:state<=CLEAR_FAULT;
                CLEAR_FAULT:state<=WAIT_ONE;
                TURN_STOP:state<=TURN_START;
                TURN_START:begin
                    if(!turn_ready || cancelled || blocked_now)transaction_error<=BLOCKED;
                    state<=WAIT_ONE;
                end
                TASK_SEND:begin
                    if(task_ready)begin
                        task_age<=0;
                        if(task_rsp_valid)begin
                            transaction_error<=task_rsp_status;task_result<=task_rsp_value;state<=WAIT_ONE;
                        end else state<=TASK_WAIT;
                    end else if(task_age==TASK_TIMEOUT_CYCLES-1)begin
                        transaction_error<=TASK_TIMEOUT;state<=WAIT_ONE;
                    end else task_age<=task_age+1'b1;
                end
                TASK_WAIT:begin
                    if(task_rsp_valid)begin
                        transaction_error<=task_rsp_status;task_result<=task_rsp_value;state<=WAIT_ONE;
                    end else if(task_age==TASK_TIMEOUT_CYCLES-1)begin
                        transaction_error<=TASK_TIMEOUT;state<=WAIT_ONE;
                    end else task_age<=task_age+1'b1;
                end
                WAIT_ONE:state<=WAIT_TWO;
                WAIT_TWO:state<=REPLY;
                REPLY:begin
                    // BOTH visits the single owner twice. Do not advance to
                    // the right wheel until the left write has actually been
                    // read back. Cancellation wins on this boundary too.
                    // A partial write is reported ERR, never rolled back or
                    // restarted after a user STOP/fault.
                    if(is_set && held_wheel==2 && !active_wheel &&
                       transaction_error==OK && !cancelled && !cancel_new &&
                       core_selected_value==held_value)begin
                        active_wheel<=1;state<=SELECT_WHEEL;
                    end else begin
                    rsp_seq<=held_seq;rsp_op<=held_op;rsp_wheel<=held_wheel;
                    rsp_field<=held_field;rsp_status<=transaction_error;
                    rsp_value<=0;rsp_valid<=1;state<=IDLE;
                    if(unknown_field)begin
                        // Invalid metadata never becomes a RUN/STOP/readback
                        // status reply just because the old owner is running.
                        rsp_value<=0;
                    end else if(is_set || is_get)begin
                        rsp_value<=unknown_field ? 32'sd0 : core_selected_value;
                        if(is_set && transaction_error==OK &&
                           core_selected_value!=held_value)rsp_status<=INTERRUPTED;
                    end else if(held_op==0)begin
                        rsp_value<={31'd0,core_running};
                        if(core_armed || core_running)rsp_status<=BLOCKED;
                    end else if(held_op==1)begin
                        rsp_value<={31'd0,core_running};
                        if(cancelled || blocked_now || !core_armed || !core_running ||
                           core_mode_position!=held_wheel)rsp_status<=BLOCKED;
                    end else if(held_op==5)begin
                        if(safety_stop || core_link_fault ||
                           core_fault_left!=0 || core_fault_right!=0)
                            rsp_status<=BLOCKED;
                    end else if(held_op==6)
                        rsp_value<={31'd0,core_running};
                    else if(held_op==8)begin
                        rsp_value<={31'd0,turn_active};
                        if(transaction_error==OK && (cancelled || blocked_now || !turn_active))rsp_status<=BLOCKED;
                    end else if(held_op==9)rsp_value<=task_result;
                    end
                end
                default:state<=IDLE;
            endcase
            // UI STOP wins even in the middle of a multi-clock RUN sequence.
            // A cancelled SET may finish selecting its readback field, but it
            // can never issue WRITE19 or start motion. A stop on the reply edge
            // inserts fresh settling clocks before reporting the final state.
            if(cancel_new)begin
                cancelled<=1;
                transaction_error<=held_op==1 || held_op==8 ? BLOCKED : INTERRUPTED;
                if(held_op==1 || held_op==8 || state==WRITE_VALUE || state==REPLY)
                    state<=WAIT_ONE;
                if(state==REPLY)rsp_valid<=0;
            end
        end
    end
endmodule
