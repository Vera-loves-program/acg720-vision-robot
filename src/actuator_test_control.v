// Local commissioning only: independent open-loop wheels and two POSITION
// servos. No camera/PCLK/DDR dependency; UART cannot start any actuator here.
// User-confirmed: MOTOR1=left, MOTOR2=right; SERVO1=base, SERVO0=upper.
// Both servos have a nominal 360-degree travel. Actual mechanical pulse
// limits are UNKNOWN at reset and must be recorded by the operator.
module actuator_test_control #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, safety_stop,
    input wire touch_ready, snapshot_valid, snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0,y0,x1,y1,
    input wire [3:0] id0,
    input wire rx_byte_valid,
    input wire [31:0] tx_line_count,
    input wire [2047:0] speed_text,
    input wire speed_text_valid,
    input wire bq_setup_busy,
    output reg bq_setup_pulse,
    output reg bq_info,
    output reg emergency_stop_pulse, clear_stop_pulse,
    output wire pwma,ain1,ain2,pwmb,bin1,bin2,servo0,servo1,
    output reg [2047:0] status_text,
    output reg [15:0] status_flags,
    output reg status_valid
);
    localparam integer MS_CYCLES=CLK_HZ/1000;
    localparam integer WHEEL_LIMIT=500; // permille, not a current limit
    reg signed [15:0] wheel_left, wheel_right;
    reg run_left,run_right;
    // Index 0 is SERVO0 (upper), index 1 is SERVO1 (base).
    reg [11:0] target_us[0:1],lower_us[0:1],upper_us[0:1];
    reg [1:0] servo_enabled,calibration,min_saved,max_saved;
    reg coarse[0:1];
    reg [1:0] cancel_pending;
    reg [127:0] speed_row_left,speed_row_right;
    wire [11:0] applied0,applied1;
    wire [1:0] pulse_active;
    wire [11:0] applied[0:1];
    assign applied[0]=applied0; assign applied[1]=applied1;
    wire [1:0] limits_valid;
    assign limits_valid[0]=min_saved[0] && max_saved[0] && lower_us[0]<upper_us[0];
    assign limits_valid[1]=min_saved[1] && max_saved[1] && lower_us[1]<upper_us[1];
    wire allow_output=reset_n && !safety_stop && touch_ready && !bq_setup_busy;
    tb6612_axis_pwm #(.CLK_HZ(CLK_HZ)) left_pwm (
        .clk(clk),.reset_n(reset_n),.enable(allow_output && run_left),.drive_invert(1'b0),
        .duty_permille(wheel_left),.pwm(pwma),.in1(ain1),.in2(ain2));
    tb6612_axis_pwm #(.CLK_HZ(CLK_HZ)) right_pwm (
        .clk(clk),.reset_n(reset_n),.enable(allow_output && run_right),.drive_invert(1'b0),
        .duty_permille(wheel_right),.pwm(pwmb),.in1(bin1),.in2(bin2));
    servo_pulse_output #(.CLK_HZ(CLK_HZ)) upper_servo (
        .clk(clk),.reset_n(reset_n),.enable(allow_output && servo_enabled[0]),
        .pulse_us(target_us[0]),.applied_us(applied0),.active(pulse_active[0]),.pwm(servo0));
    servo_pulse_output #(.CLK_HZ(CLK_HZ)) base_servo (
        .clk(clk),.reset_n(reset_n),.enable(allow_output && servo_enabled[1]),
        .pulse_us(target_us[1]),.applied_us(applied1),.active(pulse_active[1]),.pwm(servo1));

    function rect;
        input [10:0] x,y,l,r,t,b;
        begin rect=x>=l && x<r && y>=t && y<b; end
    endfunction
    // Codes shared with actuator_test_panel geometry (all coordinates active LCD).
    function [7:0] button_at;
        input [10:0] x,y;
        reg [10:0] sy;
        reg upper_card;
        reg [7:0] offset;
        begin
            button_at=0;
            if (rect(x,y,16,320,548,588)) button_at=23;
            else if (rect(x,y,720,1008,480,536)) button_at=24;
            else if (rect(x,y,720,1008,548,588)) button_at=25;
            else if (rect(x,y,884,996,60,88)) button_at=26;
            else if (x>=28 && x<308) begin
                upper_card=y>=296;
                sy=upper_card ? y-240 : y;
                offset=upper_card ? 4 : 0;
                if (sy>=168 && sy<208) begin
                    if (x<112) button_at=offset+1;
                    else if (x>=124 && x<208) button_at=offset+2;
                    else if (x>=220) button_at=offset+3;
                end else if (sy>=216 && sy<256) button_at=offset+4;
            end else if (x>=348 && x<692 && y>=56 && y<520) begin
                upper_card=y>=296;
                sy=upper_card ? y-240 : y;
                offset=upper_card ? 16 : 9;
                if (rect(x,sy,552,692,60,88)) button_at=offset+6;
                else if (sy>=168 && sy<208) begin
                    if (x<444) button_at=offset;
                    else if (x>=456 && x<552) button_at=offset+1;
                    else if (x>=564) button_at=offset+2;
                end else if (sy>=216 && sy<256) begin
                    if (x<444) button_at=offset+3;
                    else if (x>=456 && x<552) button_at=offset+4;
                    else if (x>=564) button_at=offset+5;
                end
            end
        end
    endfunction
    reg [31:0] ms_count;
    reg [15:0] down_age;
    reg [1:0] previous_count;
    reg [10:0] down_x,down_y;
    reg [3:0] down_id;
    reg [7:0] down_button,action;
    reg invalid_tap,action_valid,previous_ready;
    wire [10:0] dx=x0>=down_x ? x0-down_x : down_x-x0;
    wire [10:0] dy=y0>=down_y ? y0-down_y : down_y-y0;
    wire [7:0] current_button=button_at(x0,y0);
    wire stop_contact=snapshot_valid && touch_ready &&
        ((contact_count!=0 && button_at(x0,y0)==24) ||
         (contact_count==2 && button_at(x1,y1)==24));
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ms_count<=0;down_age<=65535;previous_count<=0;down_x<=0;down_y<=0;
            down_id<=0;down_button<=0;action<=0;invalid_tap<=1;
            action_valid<=0;emergency_stop_pulse<=0;previous_ready<=0;
        end else begin
            action_valid<=0;emergency_stop_pulse<=0;previous_ready<=touch_ready;
            if (ms_count==MS_CYCLES-1) begin
                ms_count<=0;
                if (down_age!=65535) down_age<=down_age+1'b1;
            end else ms_count<=ms_count+1'b1;
            if (!touch_ready || (snapshot_valid && snapshot_cancel)) begin
                previous_count<=0;invalid_tap<=1;down_button<=0;down_age<=65535;
                if ((previous_ready && !touch_ready) ||
                    (snapshot_valid && snapshot_cancel && (run_left || run_right || servo_enabled!=0)))
                    emergency_stop_pulse<=1;
            end else if (snapshot_valid) begin
                previous_count<=contact_count;
                if (contact_count==1 && previous_count==0) begin
                    down_x<=x0;down_y<=y0;down_id<=id0;down_age<=0;
                    down_button<=current_button;invalid_tap<=0;
                end else if (contact_count==2) invalid_tap<=1;
                else if (contact_count==1) begin
                    if (previous_count==2 || id0!=down_id || dx>16 || dy>16 ||
                        current_button!=down_button) invalid_tap<=1;
                end else if (contact_count==0 && previous_count==1 && !invalid_tap &&
                             down_age<=600 && down_button!=0) begin
                    action<=down_button;action_valid<=1;invalid_tap<=1;down_button<=0;
                end
            end
            if (stop_contact) begin
                emergency_stop_pulse<=1;invalid_tap<=1;down_button<=0;action_valid<=0;
            end
        end
    end

    reg bt_seen;
    reg [31:0] rx_byte_count;
    reg [31:0] bt_age;
    wire bt_active=bt_seen && bt_age<CLK_HZ*3;
    integer i,axis,step_amount,next_pulse;
    reg [7:0] servo_action;
    // A touch command has ample time for two bounded, sequential divisions.
    // Keep their results off the single-cycle action -> target_us path.
    localparam [1:0] MATH_IDLE=0,MATH_ANGLE=1,MATH_PULSE=2;
    reg [1:0] servo_math_state;
    reg servo_math_pending,servo_request_axis,servo_math_axis;
    reg signed [5:0] servo_request_step,servo_math_step;
    reg servo_div_start;
    reg [31:0] servo_div_numerator,servo_div_denominator;
    wire servo_div_busy,servo_div_done;
    wire [31:0] servo_div_quotient,servo_div_remainder;
    wire signed [10:0] servo_angle_next=
        $signed({2'b00,servo_div_quotient[8:0]})+
        $signed({{5{servo_math_step[5]}},servo_math_step});
    wire [8:0] servo_angle_clamped=servo_angle_next<0 ? 9'd0 :
        servo_angle_next>360 ? 9'd360 : servo_angle_next[8:0];
    wire [11:0] servo_calculated_pulse=12'd500+servo_div_quotient[11:0];
    unsigned_divmod u_servo_math (
        .clk(clk),.reset_n(reset_n),.start(servo_div_start),
        .numerator(servo_div_numerator),.denominator(servo_div_denominator),
        .busy(servo_div_busy),.done(servo_div_done),
        .quotient(servo_div_quotient),.remainder(servo_div_remainder));
    // For 500..2500 us, this is exactly the old rounded degree numerator:
    // ((pulse-500)*360+1000)/2000 == ((pulse-500)*9+25)/50.
    function [14:0] degree_numerator;
        input [11:0] pulse;
        reg [14:0] delta;
        begin
            delta=pulse>=12'd500 ? {3'b000,pulse}-15'd500 : 15'd0;
            degree_numerator=(delta<<3)+delta+15'd25;
        end
    endfunction
    // For nonnegative integer angles, floor((a*2000+180)/360)
    // equals floor((a*50+4)/9). Clamp electrical angles before dividing.
    function [14:0] pulse_numerator;
        input [8:0] angle;
        reg [14:0] extended;
        begin
            extended={6'b0,angle};
            pulse_numerator=(extended<<5)+(extended<<4)+(extended<<1)+15'd4;
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            wheel_left<=0;wheel_right<=0;run_left<=0;run_right<=0;
            servo_enabled<=0;calibration<=2'b11;min_saved<=0;max_saved<=0;
            clear_stop_pulse<=0;bq_setup_pulse<=0;bq_info<=0;bt_seen<=0;bt_age<=CLK_HZ*3;rx_byte_count<=0;
            servo_math_state<=MATH_IDLE;servo_math_pending<=0;
            servo_request_axis<=0;servo_math_axis<=0;servo_request_step<=0;servo_math_step<=0;
            servo_div_start<=0;servo_div_numerator<=0;servo_div_denominator<=1;
            cancel_pending<=0;
            speed_row_left<="L SPD WAIT      ";speed_row_right<="R SPD WAIT      ";
            for (i=0;i<2;i=i+1) begin
                target_us[i]<=1500;lower_us[i]<=1500;upper_us[i]<=1500;coarse[i]<=0;
            end
        end else begin
            clear_stop_pulse<=0;bq_setup_pulse<=0;servo_div_start<=0;
            case (servo_math_state)
                MATH_IDLE:if (servo_math_pending && !servo_div_busy) begin
                    servo_math_pending<=0;
                    if (!calibration[servo_request_axis]) begin
                        servo_math_axis<=servo_request_axis;servo_math_step<=servo_request_step;
                        servo_div_numerator<={17'b0,degree_numerator(target_us[servo_request_axis])};
                        servo_div_denominator<=32'd50;servo_div_start<=1;
                        servo_math_state<=MATH_ANGLE;
                    end
                end
                MATH_ANGLE:if (servo_div_done) begin
                    servo_div_numerator<={17'b0,pulse_numerator(servo_angle_clamped)};
                    servo_div_denominator<=32'd9;servo_div_start<=1;
                    servo_math_state<=MATH_PULSE;
                end
                MATH_PULSE:if (servo_div_done) begin
                    servo_math_state<=MATH_IDLE;
                    if (!calibration[servo_math_axis] && !cancel_pending[servo_math_axis]) begin
                        if (servo_calculated_pulse<lower_us[servo_math_axis]) target_us[servo_math_axis]<=lower_us[servo_math_axis];
                        else if (servo_calculated_pulse>upper_us[servo_math_axis]) target_us[servo_math_axis]<=upper_us[servo_math_axis];
                        else target_us[servo_math_axis]<=servo_calculated_pulse;
                    end
                end
                default:servo_math_state<=MATH_IDLE;
            endcase
            if (speed_text_valid) begin
                speed_row_left<=speed_text[2047-6*128 -:128];
                speed_row_right<=speed_text[2047-8*128 -:128];
            end
            // Capture once more AFTER disable reaches the pulse generator.
            // This also covers disable coincident with a frame-boundary step.
            for (i=0;i<2;i=i+1) begin
                if (cancel_pending[i]) target_us[i]<=applied[i];
            end
            cancel_pending<=0;
            if (rx_byte_valid) begin bt_seen<=1;bt_age<=0;rx_byte_count<=rx_byte_count+1'b1;end
            else if (bt_age<CLK_HZ*3) bt_age<=bt_age+1'b1;
            if (action_valid && action==23) clear_stop_pulse<=1;
            if (action_valid && action==26 && touch_ready) bq_info<=!bq_info;
            // Flash setup is an explicit local action, accepted only while
            // wheels and both PWM channels are OFF. STOP may remain latched.
            if (action_valid && action==25 && touch_ready && !bq_setup_busy &&
                !run_left && !run_right && servo_enabled==0) bq_setup_pulse<=1;
            if (action_valid && touch_ready && !safety_stop && !emergency_stop_pulse && !bq_setup_busy) begin
                case (action)
                    1:begin wheel_left<=(wheel_left<=-WHEEL_LIMIT) ? -WHEEL_LIMIT : wheel_left-50;run_left<=0;end
                    2:begin wheel_left<=(wheel_left>=WHEEL_LIMIT) ? WHEEL_LIMIT : wheel_left+50;run_left<=0;end
                    3:run_left<=wheel_left!=0 && !run_left;
                    4:begin wheel_left<=0;run_left<=0;end
                    5:begin wheel_right<=(wheel_right<=-WHEEL_LIMIT) ? -WHEEL_LIMIT : wheel_right-50;run_right<=0;end
                    6:begin wheel_right<=(wheel_right>=WHEEL_LIMIT) ? WHEEL_LIMIT : wheel_right+50;run_right<=0;end
                    7:run_right<=wheel_right!=0 && !run_right;
                    8:begin wheel_right<=0;run_right<=0;end
                    default:begin end
                endcase
                if (action>=9 && action<=22) begin
                    axis=action>=16 ? 0 : 1;
                    servo_action=action>=16 ? action-16 : action-9;
                    step_amount=calibration[axis] ? (coarse[axis] ? 100 : 10) : (coarse[axis] ? 10 : 1);
                    case (servo_action)
                        0,1:begin
                            if (calibration[axis]) begin
                                servo_math_state<=MATH_IDLE;servo_math_pending<=0;servo_div_start<=0;
                                next_pulse=target_us[axis]+(servo_action==0 ? -step_amount : step_amount);
                                if (next_pulse<500) next_pulse=500;
                                if (next_pulse>2500) next_pulse=2500;
                                target_us[axis]<=next_pulse[11:0];
                            end else begin
                                // Commands are queued while the shared arithmetic
                                // unit finishes (<2 us at 50 MHz, well below touch rate).
                                servo_math_pending<=1;servo_request_axis<=axis[0];
                                servo_request_step<=servo_action==0 ? -step_amount : step_amount;
                            end
                        end
                        2:begin
                            servo_math_state<=MATH_IDLE;servo_math_pending<=0;servo_div_start<=0;
                            servo_enabled[axis]<=!servo_enabled[axis];
                            if (servo_enabled[axis]) begin
                                target_us[axis]<=applied[axis];cancel_pending[axis]<=1;
                            end
                        end
                        3:coarse[axis]<=!coarse[axis];
                        // Only save a pulse actually emitted after the ramp has settled.
                        // Operator must choose a margin BEFORE a mechanical collision.
                        4:if (calibration[axis] && pulse_active[axis] && target_us[axis]==applied[axis]) begin
                            lower_us[axis]<=applied[axis];min_saved[axis]<=1;
                        end
                        5:if (calibration[axis] && pulse_active[axis] && target_us[axis]==applied[axis]) begin
                            upper_us[axis]<=applied[axis];max_saved[axis]<=1;
                        end
                        6:begin
                            servo_math_state<=MATH_IDLE;servo_math_pending<=0;servo_div_start<=0;
                            // Mode changes always release PWM. No automatic homing.
                            servo_enabled[axis]<=0;
                            target_us[axis]<=applied[axis];
                            cancel_pending[axis]<=1;
                            if (!calibration[axis]) calibration[axis]<=1;
                            else if (limits_valid[axis] &&
                                     applied[axis]>=lower_us[axis] && applied[axis]<=upper_us[axis]) begin
                                calibration[axis]<=0;
                                target_us[axis]<=applied[axis];
                            end
                        end
                        default:begin end
                    endcase
                end
            end
            // STOP wins over every command, including a simultaneous release tap.
            if (safety_stop || emergency_stop_pulse || stop_contact || !touch_ready || bq_setup_busy) begin
                run_left<=0;run_right<=0;servo_enabled<=0;
                target_us[0]<=applied0;target_us[1]<=applied1;
                cancel_pending<=2'b11;
                servo_math_state<=MATH_IDLE;servo_math_pending<=0;servo_div_start<=0;
            end
        end
    end

    // Snapshot at the original 20 Hz rate. One 32-cycle, four-digit BCD
    // converter formats every numeric field; truncation retains count % 10000.
    // Consumers capture the completed rows only when status_valid pulses.
    localparam [2:0] REPORT_IDLE=0,REPORT_PREPARE=1,REPORT_ANGLE=2,
        REPORT_SHIFT=3,REPORT_COMMIT=4;
    reg [2:0] report_state;
    reg [31:0] report_count,report_shift;
    reg [31:0] report_values[0:13];
    reg [3:0] report_index;
    reg [5:0] report_bit;
    reg [15:0] report_bcd;
    reg [1:0] report_negative;
    reg report_div_start;
    reg [31:0] report_div_numerator;
    wire report_div_busy,report_div_done;
    wire [31:0] report_div_quotient,report_div_remainder;
    unsigned_divmod u_report_math (
        .clk(clk),.reset_n(reset_n),.start(report_div_start),
        .numerator(report_div_numerator),.denominator(32'd50),
        .busy(report_div_busy),.done(report_div_done),
        .quotient(report_div_quotient),.remainder(report_div_remainder));
    function [15:0] bcd_adjust;
        input [15:0] value;
        integer nibble;
        reg [15:0] adjusted;
        begin
            adjusted=value;
            for (nibble=0;nibble<4;nibble=nibble+1)
                if (value[nibble*4 +:4]>=5) adjusted[nibble*4 +:4]=value[nibble*4 +:4]+4'd3;
            bcd_adjust=adjusted;
        end
    endfunction
    function [7:0] decimal_digit;
        input [3:0] value;
        begin decimal_digit=8'h30+{4'b0,value};end
    endfunction
    wire [15:0] report_adjusted=bcd_adjust(report_bcd);
    wire [31:0] report_ascii={decimal_digit(report_bcd[15:12]),decimal_digit(report_bcd[11:8]),
        decimal_digit(report_bcd[7:4]),decimal_digit(report_bcd[3:0])};
    wire [15:0] left_magnitude=wheel_left<0 ? -wheel_left : wheel_left;
    wire [15:0] right_magnitude=wheel_right<0 ? -wheel_right : wheel_right;
    integer report_reset_index;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            report_state<=REPORT_IDLE;report_count<=0;report_index<=0;report_bit<=0;
            report_shift<=0;report_bcd<=0;report_negative<=0;
            report_div_start<=0;report_div_numerator<=0;
            status_text<=0;status_flags<=0;status_valid<=0;
            for (report_reset_index=0;report_reset_index<14;report_reset_index=report_reset_index+1)
                report_values[report_reset_index]<=0;
        end else begin
            status_valid<=0;report_div_start<=0;
            if (report_count==CLK_HZ/20-1) begin
                report_count<=0;
                if (report_state==REPORT_IDLE) begin
                    report_values[0]<={16'b0,left_magnitude};report_values[1]<={16'b0,right_magnitude};
                    report_negative<={wheel_right<0,wheel_left<0};
                    report_values[2]<={20'b0,target_us[1]};report_values[3]<={20'b0,applied1};
                    report_values[5]<={20'b0,lower_us[1]};report_values[6]<={20'b0,upper_us[1]};
                    report_values[7]<={20'b0,target_us[0]};report_values[8]<={20'b0,applied0};
                    report_values[10]<={20'b0,lower_us[0]};report_values[11]<={20'b0,upper_us[0]};
                    report_values[12]<=rx_byte_count;report_values[13]<=tx_line_count;
                    status_flags<={max_saved[0],min_saved[0],max_saved[1],min_saved[1],
                        touch_ready,bt_active,bt_seen,safety_stop,limits_valid[0],limits_valid[1],
                        servo_enabled[0],servo_enabled[1],run_right,run_left,calibration[0],calibration[1]};
                    status_text[2047-1*128 -:128]<=speed_row_left;
                    status_text[2047-3*128 -:128]<=speed_row_right;
                    status_text[2047-4*128 -:128]<="US 0000 / 0000  ";
                    status_text[2047-5*128 -:128]<="CMD 0000 DEG EST";
                    status_text[2047-6*128 -:128]<="L0000 H0000 US  ";
                    status_text[2047-8*128 -:128]<="US 0000 / 0000  ";
                    status_text[2047-9*128 -:128]<="CMD 0000 DEG EST";
                    status_text[2047-10*128 -:128]<="L0000 H0000 US  ";
                    status_text[2047-7*128 -:128]<=calibration[1] ?
                        (coarse[1] ? "COARSE 100 US   " : "FINE    10 US   ") :
                        (coarse[1] ? "COARSE  10 DEG  " : "FINE     1 DEG  ");
                    status_text[2047-11*128 -:128]<=calibration[0] ?
                        (coarse[0] ? "COARSE 100 US   " : "FINE    10 US   ") :
                        (coarse[0] ? "COARSE  10 DEG  " : "FINE     1 DEG  ");
                    status_text[2047-12*128 -:128]<=!bt_seen ? "BT NO RX YET    " : bt_active ? "BT RX ACTIVE    " : "BT RX IDLE      ";
                    status_text[2047-13*128 -:128]<=safety_stop ? "STOP LATCHED    " :
                        bq_setup_busy ? "BQ SETUP BUSY   " : "LOCAL TEST READY";
                    report_index<=0;report_state<=REPORT_PREPARE;
                end
            end else report_count<=report_count+1'b1;
            case (report_state)
                REPORT_PREPARE:begin
                    if (report_index==4 || report_index==9) begin
                        if (!report_div_busy) begin
                            report_div_numerator<={17'b0,degree_numerator(report_values[report_index-2][11:0])};
                            report_div_start<=1;report_state<=REPORT_ANGLE;
                        end
                    end else begin
                        report_shift<=report_values[report_index];report_bcd<=0;report_bit<=0;
                        report_state<=REPORT_SHIFT;
                    end
                end
                REPORT_ANGLE:if (report_div_done) begin
                    report_shift<=report_div_quotient;report_bcd<=0;report_bit<=0;
                    report_state<=REPORT_SHIFT;
                end
                REPORT_SHIFT:begin
                    report_bcd<={report_adjusted[14:0],report_shift[31]};
                    report_shift<={report_shift[30:0],1'b0};
                    if (report_bit==31) report_state<=REPORT_COMMIT;
                    else report_bit<=report_bit+1'b1;
                end
                REPORT_COMMIT:begin
                    case (report_index)
                        0:status_text[2047-0*128 -:128]<={"SET ",report_negative[0] ? 8'h2d : 8'h2b,report_ascii[23:8],"% PWM    "};
                        1:status_text[2047-2*128 -:128]<={"SET ",report_negative[1] ? 8'h2d : 8'h2b,report_ascii[23:8],"% PWM    "};
                        2:status_text[2047-4*128-3*8 -:32]<=report_ascii;
                        3:status_text[2047-4*128-10*8 -:32]<=report_ascii;
                        4:status_text[2047-5*128-4*8 -:32]<=report_ascii;
                        5:status_text[2047-6*128-1*8 -:32]<=report_ascii;
                        6:status_text[2047-6*128-7*8 -:32]<=report_ascii;
                        7:status_text[2047-8*128-3*8 -:32]<=report_ascii;
                        8:status_text[2047-8*128-10*8 -:32]<=report_ascii;
                        9:status_text[2047-9*128-4*8 -:32]<=report_ascii;
                        10:status_text[2047-10*128-1*8 -:32]<=report_ascii;
                        11:status_text[2047-10*128-7*8 -:32]<=report_ascii;
                        12:status_text[2047-14*128 -:128]<={"RX BYTE ",report_ascii,"    "};
                        13:status_text[2047-15*128 -:128]<={"TX SENT ",report_ascii,"    "};
                        default:begin end
                    endcase
                    if (report_index==13) begin status_valid<=1;report_state<=REPORT_IDLE;end
                    else begin report_index<=report_index+1'b1;report_state<=REPORT_PREPARE;end
                end
                default:begin end
            endcase
        end
    end
endmodule
