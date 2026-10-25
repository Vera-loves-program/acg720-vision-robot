// Coherent 16x16 LCD/UART motor snapshot at 10Hz. Serial double-dabble keeps
// decimal conversion off the control and pixel critical paths. No rpm/metres
// are invented: speed is quadrature counts/10ms and position is counts.
module motor_status_text #(
    parameter integer CLK_HZ=50000000,
    parameter integer COMPACT_ROBOT=0
)(
    input wire clk, reset_n, armed, running, safety_stop, link_fault,
    input wire mode_position, position_unlocked, selected_wheel, edit_position,
    input wire [3:0] selected_field,
    input wire signed [31:0] selected_value,
    input wire [31:0] selected_step,
    input wire signed [15:0] target_effective_left, target_effective_right,
    input wire signed [31:0] speed_left, speed_right, position_left, position_right,
    input wire signed [15:0] duty_left, duty_right,
    input wire [7:0] fault_left, fault_right,
    input wire [15:0] invalid_left, invalid_right,
    output reg [2047:0] text_rows,
    output reg text_valid,
    output wire text_byte_valid,
    output wire [7:0] text_byte_data,
    output wire [7:0] text_byte_index,
    input wire text_byte_ready
);
    generate if(COMPACT_ROBOT!=0)begin:compact_robot
        // Active robot views consume only scope(row1) and L/R PWM(rows11/12).
        // Publish all256 byte positions, but do not build the legacy parallel
        // text bank or collect its unused gains/targets/position/error fields.
        localparam [2:0] C_IDLE=0,C_LOAD=1,C_CONVERT=2,C_STORE=3,
            C_EMIT=4,C_COMMIT=5;
        reg [2:0] compact_state,scope_code;
        localparam integer COMPACT_PERIOD=CLK_HZ/10<1 ? 1 : CLK_HZ/10;
        localparam integer COMPACT_AGE_WIDTH=COMPACT_PERIOD<=1 ? 1 : $clog2(COMPACT_PERIOD);
        reg [COMPACT_AGE_WIDTH-1:0] compact_age;
        reg [3:0] compact_row,compact_column,compact_bit;
        reg [31:0] compact_duties;
        reg [15:0] compact_binary;
        reg [19:0] compact_bcd;
        reg compact_negative;
        reg [127:0] compact_emit;
        wire [15:0] compact_magnitude=compact_duties[15] ?
            (~compact_duties[15:0]+16'd1) : compact_duties[15:0];
        function [19:0] compact_add_three;
            input [19:0] digits;
            integer n;
            begin
                compact_add_three=digits;
                for(n=0;n<5;n=n+1)
                    if(digits[n*4+:4]>=5)
                        compact_add_three[n*4+:4]=digits[n*4+:4]+4'd3;
            end
        endfunction
        function [7:0] compact_ascii;
            input [3:0] digit;
            begin compact_ascii=8'h30+{4'd0,digit};end
        endfunction
        function [127:0] compact_scope;
            input [2:0] scope;
            begin
                case(scope)
                    0:compact_scope="EDIT L SPEED    ";
                    1:compact_scope="EDIT L POSITION ";
                    2:compact_scope="EDIT R SPEED    ";
                    3:compact_scope="EDIT R POSITION ";
                    default:compact_scope="EDIT BOTH SHARED";
                endcase
            end
        endfunction
        wire [19:0] compact_adjusted=compact_add_three(compact_bcd);
        wire [7:0] compact_sign=compact_negative ? "-" : "+";
        wire [31:0] compact_digits={compact_ascii(compact_bcd[15:12]),
            compact_ascii(compact_bcd[11:8]),compact_ascii(compact_bcd[7:4]),
            compact_ascii(compact_bcd[3:0])};
        assign text_byte_valid=reset_n && compact_state==C_EMIT;
        assign text_byte_index={compact_row,compact_column};
        assign text_byte_data=compact_emit[127:120];
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin
                compact_state<=C_IDLE;scope_code<=0;compact_age<=0;
                compact_row<=0;compact_column<=0;compact_bit<=0;
                compact_duties<=0;compact_binary<=0;compact_bcd<=0;
                compact_negative<=0;compact_emit<=0;
                text_rows<=0;text_valid<=0;
            end else begin
                // Constant in this branch; no2048-bit compatibility storage.
                text_rows<=0;text_valid<=0;
                if(compact_age>=COMPACT_PERIOD-1)compact_age<=0;
                else compact_age<=compact_age+1'b1;
                case(compact_state)
                    C_IDLE:if(compact_age>=COMPACT_PERIOD-1)begin
                        compact_duties<={duty_right,duty_left};
                        scope_code<=(selected_field>=4 && selected_field<=7) ?
                            3'd4 : {1'b0,selected_wheel,edit_position};
                        compact_row<=0;compact_state<=C_LOAD;
                    end
                    C_LOAD:begin
                        if(compact_row==11 || compact_row==12)begin
                            compact_binary<=compact_magnitude;
                            compact_negative<=compact_duties[15] && compact_magnitude!=0;
                            compact_bcd<=0;compact_bit<=0;compact_state<=C_CONVERT;
                        end else compact_state<=C_STORE;
                    end
                    C_CONVERT:begin
                        compact_bcd<={compact_adjusted[18:0],compact_binary[15]};
                        compact_binary<={compact_binary[14:0],1'b0};
                        if(compact_bit==15)compact_state<=C_STORE;
                        else compact_bit<=compact_bit+1'b1;
                    end
                    C_STORE:begin
                        if(compact_row==1)compact_emit<=compact_scope(scope_code);
                        else if(compact_row==11)compact_emit<={"L PWM ",compact_sign,compact_digits,"/1000"};
                        else if(compact_row==12)compact_emit<={"R PWM ",compact_sign,compact_digits,"/1000"};
                        else compact_emit<="                ";
                        compact_column<=0;compact_state<=C_EMIT;
                    end
                    C_EMIT:if(text_byte_ready)begin
                        compact_emit<={compact_emit[119:0],8'd0};
                        if(compact_column==15)begin
                            if(compact_row==11)compact_duties<={16'd0,compact_duties[31:16]};
                            if(compact_row==15)compact_state<=C_COMMIT;
                            else begin compact_row<=compact_row+1'b1;compact_state<=C_LOAD;end
                        end else compact_column<=compact_column+1'b1;
                    end
                    C_COMMIT:begin text_valid<=1;compact_state<=C_IDLE;end
                    default:compact_state<=C_IDLE;
                endcase
            end
        end
    end else begin:full_formatter
    localparam IDLE=0, LOAD=1, CONVERT=2, STORE=3, COMMIT=4, EMIT=5;
    reg [2:0] state;
    reg [31:0] divider;
    reg [3:0] field;
    reg [5:0] bit_count;
    // Largest public-input value: uint32 step * 10000 / 256 < 2^38.
    // Twelve decimal digits cover every field, including the two BAD counts.
    // Keeping 64 shift bits and twenty BCD digits would add unused conversion
    // stages and arithmetic; displayed characters still use the same slices.
    reg [37:0] values [0:15];
    reg [15:0] negative;
    reg [127:0] rows [0:15];
    reg [127:0] state_row, scope_row, field_row, mode_row;
    reg fractional;
    reg [37:0] shift_value;
    reg [47:0] decimal;
    // The UART consumes each row as it is formatted. A small row shifter
    // avoids selecting a byte from a completed 2048-bit parallel snapshot.
    // All inputs/BCD/field remain held throughout EMIT, including stalls.
    reg [127:0] formatted_row;
    reg [127:0] emit_shift;
    reg [3:0] byte_in_row;
    assign text_byte_valid=reset_n && state==EMIT;
    assign text_byte_data=emit_shift[127:120];
    assign text_byte_index={field,byte_in_row};
    integer i;
    function [47:0] add_three;
        input [47:0] d;
        integer n;
        begin
            add_three=d;
            for (n=0;n<12;n=n+1)
                if (d[n*4 +: 4]>=5) add_three[n*4 +: 4]=d[n*4 +: 4]+4'd3;
        end
    endfunction
    function [31:0] abs32;
        input signed [31:0] v;
        begin abs32=v[31] ? ~v+32'd1 : v; end
    endfunction
    function [7:0] a;
        input [3:0] d;
        begin a=8'h30+{4'd0,d}; end
    endfunction
    function [127:0] field_label;
        input [3:0] f;
        begin
            case(f)
                0:field_label="FIELD KP        "; 1:field_label="FIELD KI        ";
                2:field_label="FIELD KD        "; 3:field_label="FIELD TARGET    ";
                4:field_label="FIELD START PWM "; 5:field_label="FIELD MAX PWM   ";
                6:field_label="FIELD SPEED DB  "; 7:field_label="FIELD POS DB    ";
                8:field_label="FIELD ENC DIR   "; 9:field_label="FIELD MOTOR DIR ";
                default:field_label="FIELD INVALID   ";
            endcase
        end
    endfunction
    wire [47:0] adjusted=add_three(decimal);
    wire [45:0] selected_scaled=abs32(selected_value)*14'd10000;
    wire [45:0] step_scaled=selected_step*14'd10000;
    // The eight-count display saturates rather than silently wrapping a value.
    wire big_count=values[field]>38'd99999999;
    wire [63:0] count_digits=big_count ? "OVERFLOW" :
        {a(decimal[31:28]),a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),
         a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
    wire [7:0] sign_char=negative[field] && values[field]!=0 ? "-" : "+";
    always @* begin
        formatted_row=0;
        case(field)
            0:formatted_row=state_row; 1:formatted_row=scope_row; 2:formatted_row=field_row;
            3:if (fractional) formatted_row={"VAL ",a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),".",
                a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"    "};
              else formatted_row={"VAL ",sign_char,a(decimal[39:36]),a(decimal[35:32]),a(decimal[31:28]),
                a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),a(decimal[15:12]),
                a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])," "};
            4:if (fractional) formatted_row={"STEP ",a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),".",
                a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"   "};
              else formatted_row={"STEP ",a(decimal[19:16]),a(decimal[15:12]),a(decimal[11:8]),
                a(decimal[7:4]),a(decimal[3:0]),"      "};
            5:formatted_row={"L TGT ",sign_char,count_digits," "};
            6:formatted_row={"L SPD ",sign_char,count_digits," "};
            7:formatted_row={"R TGT ",sign_char,count_digits," "};
            8:formatted_row={"R SPD ",sign_char,count_digits," "};
            9:formatted_row={"L POS ",sign_char,count_digits," "};
            10:formatted_row={"R POS ",sign_char,count_digits," "};
            11:formatted_row={"L PWM ",sign_char,a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"/1000"};
            12:formatted_row={"R PWM ",sign_char,a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"/1000"};
            13:formatted_row={"FAULT L ",a(decimal[23:20]),a(decimal[19:16]),a(decimal[15:12]),
                " R",a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
            14:formatted_row={"BAD L",a(decimal[39:36]),a(decimal[35:32]),a(decimal[31:28]),a(decimal[27:24]),a(decimal[23:20]),
                "R",a(decimal[19:16]),a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
            15:formatted_row=mode_row;
        endcase
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; divider<=0; field<=0; bit_count<=0;
            negative<=0; fractional<=0; shift_value<=0; decimal<=0;
            state_row<=0; scope_row<=0; field_row<=0; mode_row<=0;
            text_rows<=0; text_valid<=0;
            emit_shift<=0;byte_in_row<=0;
            for (i=0;i<16;i=i+1) begin values[i]<=0; rows[i]<=0; end
        end else begin
            text_valid<=0;
            if (divider>=CLK_HZ/10-1) divider<=0;
            else divider<=divider+1'b1;
            case(state)
                IDLE:if (divider>=CLK_HZ/10-1) begin
                    state_row<=safety_stop ? "MOTOR E-STOP    " : link_fault ? "MOTOR LINK FAULT" :
                        (fault_left!=0 || fault_right!=0) ? "MOTOR FAULT     " :
                        running ? "MOTOR RUN       " : armed ? "MOTOR ARMED     " : "MOTOR DISARMED  ";
                    scope_row<=(selected_field>=4 && selected_field<=7) ? "EDIT BOTH SHARED" :
                        selected_wheel ? (edit_position ? "EDIT R POSITION " : "EDIT R SPEED    ") :
                                         (edit_position ? "EDIT L POSITION " : "EDIT L SPEED    ");
                    field_row<=field_label(selected_field);
                    mode_row<=mode_position ? "MODE POSITION   " :
                              position_unlocked ? "MODE SPEED OPEN " : "MODE SPEED LOCK ";
                    fractional<=selected_field<3;
                    if (selected_field<3) begin
                        values[3]<=selected_scaled[45:8];
                        values[4]<=step_scaled[45:8];
                    end else begin values[3]<={6'd0,abs32(selected_value)}; values[4]<={6'd0,selected_step}; end
                    values[5]<={6'd0,abs32($signed(target_effective_left))};
                    values[6]<={6'd0,abs32(speed_left)};
                    values[7]<={6'd0,abs32($signed(target_effective_right))};
                    values[8]<={6'd0,abs32(speed_right)};
                    values[9]<={6'd0,abs32(position_left)};
                    values[10]<={6'd0,abs32(position_right)};
                    values[11]<={6'd0,abs32($signed(duty_left))};
                    values[12]<={6'd0,abs32($signed(duty_right))};
                    values[13]<={30'd0,fault_left}*38'd1000+{30'd0,fault_right};
                    values[14]<={22'd0,invalid_left}*38'd100000+{22'd0,invalid_right};
                    negative[3]<=selected_value[31]; negative[4]<=0;
                    negative[5]<=target_effective_left[15]; negative[6]<=speed_left[31];
                    negative[7]<=target_effective_right[15]; negative[8]<=speed_right[31];
                    negative[9]<=position_left[31]; negative[10]<=position_right[31];
                    negative[11]<=duty_left[15]; negative[12]<=duty_right[15];
                    field<=0; state<=LOAD;
                end
                LOAD:begin shift_value<=values[field]; decimal<=0; bit_count<=0; state<=CONVERT; end
                CONVERT:begin
                    decimal<={adjusted[46:0],shift_value[37]}; shift_value<={shift_value[36:0],1'b0};
                    if (bit_count==37) state<=STORE;
                    else bit_count<=bit_count+1'b1;
                end
                STORE:begin
                    emit_shift<=formatted_row;byte_in_row<=0;state<=EMIT;
                end
                EMIT:if(text_byte_ready)begin
                    if(byte_in_row==15)begin
                        case(field)
                            0:rows[0]<=state_row; 1:rows[1]<=scope_row; 2:rows[2]<=field_row;
                            3:if (fractional) rows[3]<={"VAL ",a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),".",
                                a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"    "};
                              else rows[3]<={"VAL ",sign_char,a(decimal[39:36]),a(decimal[35:32]),a(decimal[31:28]),
                                a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),a(decimal[15:12]),
                                a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])," "};
                            4:if (fractional) rows[4]<={"STEP ",a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),".",
                                a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"   "};
                              else rows[4]<={"STEP ",a(decimal[19:16]),a(decimal[15:12]),a(decimal[11:8]),
                                a(decimal[7:4]),a(decimal[3:0]),"      "};
                            5:rows[5]<={"L TGT ",sign_char,count_digits," "};
                            6:rows[6]<={"L SPD ",sign_char,count_digits," "};
                            7:rows[7]<={"R TGT ",sign_char,count_digits," "};
                            8:rows[8]<={"R SPD ",sign_char,count_digits," "};
                            9:rows[9]<={"L POS ",sign_char,count_digits," "};
                            10:rows[10]<={"R POS ",sign_char,count_digits," "};
                            11:rows[11]<={"L PWM ",sign_char,a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"/1000"};
                            12:rows[12]<={"R PWM ",sign_char,a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0]),"/1000"};
                            13:rows[13]<={"FAULT L ",a(decimal[23:20]),a(decimal[19:16]),a(decimal[15:12]),
                                " R",a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
                            14:rows[14]<={"BAD L",a(decimal[39:36]),a(decimal[35:32]),a(decimal[31:28]),a(decimal[27:24]),a(decimal[23:20]),
                                "R",a(decimal[19:16]),a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
                            15:rows[15]<=mode_row;
                        endcase
                        if(field==15)state<=COMMIT;
                        else begin field<=field+1'b1;state<=LOAD;end
                    end else begin
                        emit_shift<={emit_shift[119:0],8'd0};byte_in_row<=byte_in_row+1'b1;
                    end
                end
                COMMIT:begin
                    for (i=0;i<16;i=i+1) text_rows[2047-i*128 -: 128]<=rows[i];
                    text_valid<=1; state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
    end endgenerate
endmodule
