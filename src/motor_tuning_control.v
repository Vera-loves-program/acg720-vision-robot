// Central settings: LCD and UART share ONE owner, no duplicate gain registers.
// Gain edits and target/direction/mode changes disarm before changing settings.
// Units: encoder counts/10ms, absolute encoder counts, PWM permille; gains Q8.
// Position mode is locked until an explicit stopped UNLOCK POS confirmation.
// That confirmation records user intent, not automatic proof of correct tuning.
module motor_tuning_control #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, safety_stop,
    input wire [7:0] fault_left, fault_right,
    input wire ui_action_valid,
    input wire [7:0] ui_action,
    input wire uart_action_valid,
    input wire [7:0] uart_action,
    input wire signed [31:0] uart_argument,
    input wire uart_heartbeat,
    output reg armed, running, mode_position, position_unlocked, link_fault,
    output reg clear_fault, clear_stop, zero_position,
    output reg selected_wheel, edit_position,
    output reg [3:0] selected_field,
    output reg [1:0] step_select,
    output reg encoder_invert_left, encoder_invert_right,
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
    reg remote_owner;
    reg [31:0] heartbeat_age;
    reg signed [32:0] requested_value;
    reg signed [31:0] bounded_value;
    reg signed [31:0] low_limit, high_limit;
    wire take_action=ui_action_valid || uart_action_valid;
    wire [7:0] act=ui_action_valid ? ui_action : uart_action;
    wire from_remote=!ui_action_valid && uart_action_valid;
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
            default:selected_value=0;
        endcase
        low_limit=0; high_limit=32767;
        case (selected_field)
            3:begin low_limit=edit_position ? -2000000 : -200; high_limit=-low_limit; end
            4:high_limit={22'd0,max_duty_permille};
            5:begin low_limit=50; high_limit=900; end
            6:high_limit=32;
            7:high_limit=1000;
            8:high_limit=1;
            default:begin end
        endcase
        if (act==19) requested_value={uart_argument[31],uart_argument};
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
            encoder_invert_left<=0; encoder_invert_right<=0;
            target_speed_left<=0; target_speed_right<=0;
            target_position_left<=0; target_position_right<=0;
            speed_kp_left<=0; speed_ki_left<=0; speed_kd_left<=0;
            speed_kp_right<=0; speed_ki_right<=0; speed_kd_right<=0;
            position_kp_left<=0; position_ki_left<=0; position_kd_left<=0;
            position_kp_right<=0; position_ki_right<=0; position_kd_right<=0;
            max_duty_permille<=200; min_duty_permille<=0;
            speed_deadband<=1; position_deadband<=4;
            remote_owner<=0; heartbeat_age<=0;
        end else begin
            clear_fault<=0; clear_stop<=0; zero_position<=0;
            if (!armed || !remote_owner) heartbeat_age<=0;
            else if (uart_heartbeat || uart_action_valid) heartbeat_age<=0;
            else if (heartbeat_age>=CLK_HZ*2-1) begin
                link_fault<=1; armed<=0; running<=0;
            end else heartbeat_age<=heartbeat_age+1'b1;
            if (take_action) begin
                case (act)
                    1:selected_wheel<=~selected_wheel;
                    2:edit_position<=~edit_position;
                    3:selected_field<=selected_field==0 ? 8 : selected_field-1'b1;
                    4:selected_field<=selected_field==8 ? 0 : selected_field+1'b1;
                    7:step_select<=step_select==2 ? 0 : step_select+1'b1;
                    16:if (uart_argument==0 || uart_argument==1) selected_wheel<=uart_argument[0];
                    17:if (uart_argument==0 || uart_argument==1) edit_position<=uart_argument[0];
                    18:if (uart_argument>=0 && uart_argument<=8) selected_field<=uart_argument[3:0];
                    20:if (uart_argument>=0 && uart_argument<=2) step_select<=uart_argument[1:0];
                    8:if (!running && !safety_stop && fault_left==0 && fault_right==0 && !link_fault) begin
                        armed<=1; remote_owner<=from_remote; heartbeat_age<=0;
                    end
                    9:if (armed && !running && !safety_stop && !link_fault &&
                          fault_left==0 && fault_right==0 && config_ok) begin
                        running<=1; remote_owner<=from_remote; heartbeat_age<=0;
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
                    5,6,19:begin
                        // Stop before every edit; settings never restart motion.
                        armed<=0; running<=0;
                        case (selected_field)
                            0:if (edit_position) begin
                                if (selected_wheel) position_kp_right<=bounded_value[15:0];
                                else position_kp_left<=bounded_value[15:0];
                            end else begin
                                position_unlocked<=0; mode_position<=0;
                                if (selected_wheel) speed_kp_right<=bounded_value[15:0];
                                else speed_kp_left<=bounded_value[15:0];
                            end
                            1:if (edit_position) begin
                                if (selected_wheel) position_ki_right<=bounded_value[15:0];
                                else position_ki_left<=bounded_value[15:0];
                            end else begin
                                position_unlocked<=0; mode_position<=0;
                                if (selected_wheel) speed_ki_right<=bounded_value[15:0];
                                else speed_ki_left<=bounded_value[15:0];
                            end
                            2:if (edit_position) begin
                                if (selected_wheel) position_kd_right<=bounded_value[15:0];
                                else position_kd_left<=bounded_value[15:0];
                            end else begin
                                position_unlocked<=0; mode_position<=0;
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
                            default:begin end
                        endcase
                    end
                    default:begin end
                endcase
            end
            // Last assignments have priority over ARM/RUN on this same clock.
            if (safety_stop || fault_left!=0 || fault_right!=0 || link_fault ||
                (ui_action_valid && ui_action==10) || (uart_action_valid && uart_action==10) ||
                (armed && remote_owner && heartbeat_age>=CLK_HZ*2-1 &&
                 !uart_heartbeat && !uart_action_valid)) begin
                armed<=0; running<=0;
            end
        end
    end
endmodule
