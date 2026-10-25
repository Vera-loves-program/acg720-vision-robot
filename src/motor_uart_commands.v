// Framed motor commands, separate from legacy P/F/S/N and RTC calibration.
// @A ARM, @R RUN, @S STOP, @C CLEAR, @Z ZERO, @U UNLOCK POSITION,
// @H heartbeat; @W0/1 wheel, @L0/1 edit loop, @F0..9 field,
// @V signed_integer set selected field, @Q0..2 step, @M0/1 mode.
// CR or LF commits a complete command. '!' is immediate emergency stop.
// Integers are checked before multiplication; malformed/partial input never runs.
module motor_uart_commands #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, byte_valid,
    input wire [7:0] byte_data,
    output reg action_valid,
    output reg [7:0] action,
    output reg signed [31:0] argument,
    output reg emergency_stop_pulse, heartbeat,
    output reg [31:0] parse_errors
);
    reg collecting, have_opcode, negative, have_sign, have_digit, bad;
    reg [7:0] opcode;
    reg [31:0] magnitude, age;
    wire numeric_opcode=opcode=="W" || opcode=="L" || opcode=="F" ||
                        opcode=="V" || opcode=="Q" || opcode=="M";
    wire plain_opcode=opcode=="A" || opcode=="R" || opcode=="S" ||
                      opcode=="C" || opcode=="Z" || opcode=="U" || opcode=="H";
    wire [3:0] digit=byte_data-8'h30;
    wire number_overflow=magnitude>32'd214748364 ||
        (magnitude==32'd214748364 && digit>(negative ? 4'd8 : 4'd7));
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            collecting<=0; have_opcode<=0; negative<=0; have_sign<=0;
            have_digit<=0; bad<=0; opcode<=0; magnitude<=0; age<=0;
            action_valid<=0; action<=0; argument<=0;
            emergency_stop_pulse<=0; heartbeat<=0; parse_errors<=0;
        end else begin
            action_valid<=0; emergency_stop_pulse<=0; heartbeat<=0;
            if (collecting) begin
                if (age>=CLK_HZ/4-1) begin
                    collecting<=0; parse_errors<=parse_errors+1'b1;
                end else age<=age+1'b1;
            end
            if (byte_valid) begin
                if (byte_data=="!") begin
                    emergency_stop_pulse<=1; collecting<=0;
                end else if (byte_data=="@") begin
                    collecting<=1; have_opcode<=0; negative<=0;
                    have_sign<=0; have_digit<=0; bad<=0;
                    magnitude<=0; opcode<=0; age<=0;
                end else if (collecting) begin
                    age<=0;
                    if (byte_data==8'h0d || byte_data==8'h0a) begin
                        collecting<=0;
                        if (!bad && have_opcode &&
                            ((plain_opcode && !have_sign && !have_digit) ||
                             (numeric_opcode && have_digit && (!negative || opcode=="V")))) begin
                            argument<=negative ? -$signed(magnitude) : $signed(magnitude);
                            action_valid<=1;
                            case (opcode)
                                "A":action<=8; "R":action<=9; "S":action<=10;
                                "C":action<=11; "Z":action<=14; "U":action<=15;
                                "W":action<=16; "L":action<=17; "F":action<=18;
                                "V":action<=19; "Q":action<=20;
                                "M":begin
                                    if (magnitude==0) action<=12;
                                    else if (magnitude==1) action<=13;
                                    else begin action_valid<=0; parse_errors<=parse_errors+1'b1; end
                                end
                                "H":begin action_valid<=0; heartbeat<=1; end
                                default:action_valid<=0;
                            endcase
                        end else parse_errors<=parse_errors+1'b1;
                    end else if (!have_opcode) begin
                        opcode<=byte_data; have_opcode<=1;
                    end else if (!have_sign && !have_digit &&
                                 (byte_data=="-" || byte_data=="+")) begin
                        have_sign<=1; negative<=byte_data=="-";
                    end else if (byte_data>="0" && byte_data<="9") begin
                        have_digit<=1;
                        if (number_overflow) bad<=1;
                        else magnitude<=(magnitude<<3)+(magnitude<<1)+{28'd0,digit};
                    end else bad<=1;
                end
            end
        end
    end
endmodule

// Bluetooth VOFA+ FireWater transport. No floating-point datapath or printf.
// Host: #sequence,operation,wheel,field,value followed by LF/CR.
// Operation: 0 STOP, 1 RUN pair, 2 SET speed, 3 SET position, 4 GET,
//            5 CLEAR faults, 6 compatibility no-op, 7 GET position-loop field,
//            8 relative gyro TURN in signed integer degrees (+left/-right).
//            9 task command, wheel0: field0 origin/front+X,1 goalXmm,
//              2 goalYmm,3 GOTO,4 FIND DOG,5 FACE,6 GO NEAR,7 task STOP.
// Task9 uses the ordinary UI/mission owner; its ACK means accepted/rejected,
// not that motion/search has finished. Coordinate fields accept integers.
// SET gains accept human decimal (10 or 10.0000 -> native Q8 value 2560).
// Wheel2 broadcasts SET operations2/3 fields0..7 through the same owner;
// one ACK with wheel2 follows both actual readbacks. Other operations stay0/1.
// Other values and all ACK readbacks use native integers; gain readback is Q8.
// Backend owns validation, mutation and readback; an ACK means its response,
// never merely that this parser received a number. Send next command after ACK.
// TX samples: CHANNELS signed integers, channel 0 at sample_values[31:0], 20 Hz.
// ACKs start with nonnumeric '[ACK OK]' / '[ACK ERR]' and cannot be curves.
// UART framing is provided by the existing shared 115200 8N1 transmitter.
module robot_vofa_uart #(
    parameter integer CLK_HZ=50000000,
    parameter integer SAMPLE_HZ=20,
    parameter integer CHANNELS=4
)(
    input wire clk, reset_n,
    input wire byte_valid,
    input wire [7:0] byte_data,
    output reg emergency_stop_pulse,
    output reg cmd_valid,
    input wire cmd_ready,
    output reg [15:0] cmd_seq,
    output reg [3:0] cmd_op,
    output reg [1:0] cmd_wheel,
    output reg [3:0] cmd_field,
    output reg signed [31:0] cmd_value,
    input wire rsp_valid,
    output wire rsp_ready,
    input wire [15:0] rsp_seq,
    input wire [3:0] rsp_op,
    input wire [1:0] rsp_wheel,
    input wire [3:0] rsp_field,
    input wire [7:0] rsp_status,
    input wire signed [31:0] rsp_value,
    input wire [CHANNELS*32-1:0] sample_values,
    input wire turn_complete_valid,
    output wire turn_complete_ready,
    output wire turn_complete_busy,
    output wire tx_byte_valid,
    input wire tx_byte_ready,
    output reg [7:0] tx_byte_data,
    output reg [31:0] tx_line_count,
    output reg [31:0] parse_errors
);
    localparam integer RX_TIMEOUT_CYCLES=CLK_HZ<2 ? 1 : CLK_HZ/2;
    localparam integer SAMPLE_CYCLES=CLK_HZ/SAMPLE_HZ<1 ? 1 : CLK_HZ/SAMPLE_HZ;
    localparam integer RX_AGE_WIDTH=RX_TIMEOUT_CYCLES<=1 ? 1 : $clog2(RX_TIMEOUT_CYCLES);
    localparam integer SAMPLE_AGE_WIDTH=SAMPLE_CYCLES<=1 ? 1 : $clog2(SAMPLE_CYCLES);
    // Six words retain the complete ACK even with only four plotted channels.
    localparam integer FRAME_WORDS=CHANNELS<6 ? 6 : CHANNELS;
    // A request stays stable until accepted, then stays outstanding until its
    // response is queued. This also blocks a second command changing selection.
    reg outstanding;
    reg collecting, negative, have_sign, have_digit, bad;
    reg [2:0] rx_field;
    reg [31:0] magnitude;
    reg [RX_AGE_WIDTH-1:0] rx_age;
    reg [15:0] parsed_seq;
    reg [3:0] parsed_op;
    reg [1:0] parsed_wheel;
    reg [3:0] parsed_field;
    reg have_dot;
    reg [2:0] fraction_digits;
    reg [13:0] fraction_divisor;
    wire gain_packet=(parsed_op==2 || parsed_op==3) && parsed_field<=2;
    wire [3:0] rx_digit=byte_data-8'h30;
    wire number_overflow=magnitude>32'd214748364 ||
        (magnitude==32'd214748364 && rx_digit>(negative ? 4'd8 : 4'd7));
    wire number_valid=have_digit && !bad && (!negative || rx_field==4);
    wire signed [31:0] received_value=negative ? -$signed(magnitude) : $signed(magnitude);
    // Shared restoring divider: shift human decimal by eight binary places,
    // then divide by its 1/10/100/1000/10000 scale. One 15-bit subtractor serves
    // all six gains; there are no float operators or parallel constant dividers.
    reg gain_convert_busy;
    reg [31:0] gain_dividend;
    reg [13:0] gain_divisor, gain_remainder;
    reg [5:0] gain_step;
    wire [14:0] gain_trial={gain_remainder,gain_dividend[31]};
    wire gain_subtract=gain_trial>={1'b0,gain_divisor};
    wire [14:0] gain_remainder_next=gain_subtract ? gain_trial-{1'b0,gain_divisor} : gain_trial;
    wire [31:0] gain_dividend_next={gain_dividend[30:0],gain_subtract};
    // Parse errors have their own one-entry mailbox so telemetry or a backend
    // ACK cannot interleave characters with an error response.
    reg notice_pending;
    reg [15:0] notice_seq;
    reg [7:0] notice_status;
    wire consume_notice;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            collecting<=0;negative<=0;have_sign<=0;have_digit<=0;bad<=0;
            rx_field<=0;magnitude<=0;rx_age<=0;
            parsed_seq<=0;parsed_op<=0;parsed_wheel<=0;parsed_field<=0;
            have_dot<=0;fraction_digits<=0;fraction_divisor<=1;
            gain_convert_busy<=0;gain_dividend<=0;gain_divisor<=1;gain_remainder<=0;gain_step<=0;
            cmd_valid<=0;cmd_seq<=0;cmd_op<=0;cmd_wheel<=0;cmd_field<=0;cmd_value<=0;
            outstanding<=0;emergency_stop_pulse<=0;parse_errors<=0;
            notice_pending<=0;notice_seq<=0;notice_status<=0;
        end else begin
            emergency_stop_pulse<=0;
            if(consume_notice)notice_pending<=0;
            if(cmd_valid && cmd_ready)cmd_valid<=0;
            if(rsp_valid && rsp_ready)outstanding<=0;
            if(gain_convert_busy)begin
                gain_dividend<=gain_dividend_next;gain_remainder<=gain_remainder_next[13:0];
                if(gain_step==31)begin
                    cmd_value<=$signed(gain_dividend_next);cmd_valid<=1;gain_convert_busy<=0;
                end else gain_step<=gain_step+1'b1;
            end
            if(collecting && !byte_valid)begin
                if(rx_age>=RX_TIMEOUT_CYCLES-1)begin
                    collecting<=0;parse_errors<=parse_errors+1'b1;
                    if(!notice_pending || consume_notice)begin
                        notice_pending<=1;notice_seq<=parsed_seq;notice_status<=2;
                    end
                end else rx_age<=rx_age+1'b1;
            end
            if(byte_valid)begin
                if(byte_data=="!")begin
                    emergency_stop_pulse<=1;collecting<=0;cmd_valid<=0;
                    gain_convert_busy<=0;
                    if(gain_convert_busy || (cmd_valid && !cmd_ready))outstanding<=0;
                end else if(byte_data=="#")begin
                    collecting<=1;negative<=0;have_sign<=0;have_digit<=0;bad<=0;
                    rx_field<=0;magnitude<=0;rx_age<=0;
                    parsed_seq<=0;parsed_op<=0;parsed_wheel<=0;parsed_field<=0;
                    have_dot<=0;fraction_digits<=0;fraction_divisor<=1;
                end else if(collecting)begin
                    rx_age<=0;
                    if(byte_data==8'h0d || byte_data==8'h0a)begin
                        collecting<=0;
                        if(number_valid && rx_field==4 && !outstanding && !cmd_valid &&
                           (!have_dot || fraction_digits!=0) &&
                           (!gain_packet || (!negative && magnitude<({18'd0,fraction_divisor}<<7))))begin
                            cmd_seq<=parsed_seq;cmd_op<=parsed_op;
                            cmd_wheel<=parsed_wheel;cmd_field<=parsed_field;
                            outstanding<=1;
                            if(gain_packet)begin
                                gain_convert_busy<=1;gain_dividend<=magnitude<<8;
                                gain_divisor<=fraction_divisor;gain_remainder<=0;gain_step<=0;
                            end else begin cmd_value<=received_value;cmd_valid<=1;end
                        end else begin
                            parse_errors<=parse_errors+1'b1;
                            if(!notice_pending || consume_notice)begin
                                notice_pending<=1;notice_seq<=parsed_seq;
                                notice_status<=outstanding || cmd_valid ? 3 : 1;
                            end
                        end
                    end else if(byte_data==",")begin
                        if(!number_valid)bad<=1;
                        case(rx_field)
                            0:if(magnitude<=65535 && !negative)parsed_seq<=magnitude[15:0];else bad<=1;
                            1:if(magnitude<=9 && !negative)parsed_op<=magnitude[3:0];else bad<=1;
                            2:if(magnitude<=2 && !negative)parsed_wheel<=magnitude[1:0];else bad<=1;
                            3:if(magnitude<=9 && !negative)parsed_field<=magnitude[3:0];else bad<=1;
                            default:bad<=1;
                        endcase
                        if(rx_field<4)rx_field<=rx_field+1'b1;
                        magnitude<=0;negative<=0;have_sign<=0;have_digit<=0;
                        have_dot<=0;fraction_digits<=0;fraction_divisor<=1;
                    end else if(byte_data==".")begin
                        if(rx_field==4 && gain_packet && have_digit && !negative && !have_dot)have_dot<=1;
                        else bad<=1;
                    end else if(!have_sign && !have_digit && (byte_data=="-" || byte_data=="+"))begin
                        negative<=byte_data=="-";have_sign<=1;
                    end else if(byte_data>="0" && byte_data<="9")begin
                        have_digit<=1;
                        if(number_overflow)bad<=1;
                        else magnitude<=(magnitude<<3)+(magnitude<<1)+{28'd0,rx_digit};
                        if(have_dot)begin
                            if(fraction_digits>=4)bad<=1;
                            else begin
                                fraction_digits<=fraction_digits+1'b1;
                                fraction_divisor<=(fraction_divisor<<3)+(fraction_divisor<<1);
                            end
                        end
                    end else bad<=1;
                end
            end
        end
    end

    // One backend-response slot. It is consumed only between complete lines.
    reg ack_pending;
    reg [15:0] ack_seq;
    reg [3:0] ack_op;
    reg [1:0] ack_wheel;
    reg [3:0] ack_field;
    reg [7:0] ack_status;
    reg signed [31:0] ack_value;
    wire consume_ack;
    assign rsp_ready=!ack_pending;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            ack_pending<=0;ack_seq<=0;ack_op<=0;ack_wheel<=0;
            ack_field<=0;ack_status<=0;ack_value<=0;
        end else begin
            if(consume_ack)ack_pending<=0;
            if(rsp_valid && rsp_ready)begin
                ack_pending<=1;ack_seq<=rsp_seq;ack_op<=rsp_op;
                ack_wheel<=rsp_wheel;ack_field<=rsp_field;
                ack_status<=rsp_status;ack_value<=rsp_value;
            end
        end
    end

    // A completed turn is a single queued line, never a recurring status sample.
    // The producer holds valid while this slot is occupied; line bytes share
    // the same ready/valid serializer as ACKs and waveforms.
    reg event_pending;
    wire consume_event;
    assign turn_complete_ready=!event_pending;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)event_pending<=0;
        else begin
            if(consume_event)event_pending<=0;
            if(turn_complete_valid && turn_complete_ready)event_pending<=1;
        end
    end

    // Shared sign/magnitude + double-dabble unit for all channels and ACK values.
    localparam [3:0] IDLE=0,PREFIX=1,LOAD=2,CONVERT=3,SIGN=4,
        DIGIT=5,SEPARATOR=6,NEWLINE=7;
    reg [3:0] state, digit_index;
    reg [4:0] channel, last_channel;
    reg [3:0] prefix_index;
    reg frame_ack, frame_error, frame_event, digit_started, number_negative;
    assign turn_complete_busy=event_pending || frame_event;
    // One atomic sample snapshot, shifted by a FIXED word after each comma.
    // LOAD always reads its low32 bits. Avoid a wide signed32 channel mux
    // feeding the shared decimal unit across the placement area.
    reg [FRAME_WORDS*32-1:0] held_values;
    reg [31:0] binary_shift;
    reg [39:0] decimal_digits;
    reg [5:0] conversion_step;
    reg [SAMPLE_AGE_WIDTH-1:0] sample_age;
    reg sample_due;
    wire [31:0] selected_value=held_values[31:0];
    wire [31:0] selected_magnitude=selected_value[31] ? (~selected_value+32'd1) : selected_value;
    // Decimal output also walks fixed wires rather than a ten-way nibble mux.
    wire [3:0] current_digit=decimal_digits[39:36];
    function [39:0] add_three;
        input [39:0] digits;
        integer d;
        begin
            add_three=digits;
            for(d=0;d<10;d=d+1)
                if(digits[d*4 +: 4]>=5)add_three[d*4 +: 4]=digits[d*4 +: 4]+4'd3;
        end
    endfunction
    wire [39:0] adjusted_digits=add_three(decimal_digits);
    function [7:0] prefix_byte;
        input ack, failed, event_line;
        input [3:0] index;
        begin
            if(event_line)begin
                case(index)
                    0:prefix_byte="T";1:prefix_byte="U";2:prefix_byte="R";3:prefix_byte="N";
                    4:prefix_byte="_";5:prefix_byte="C";6:prefix_byte="O";7:prefix_byte="M";
                    8:prefix_byte="P";9:prefix_byte="L";10:prefix_byte="E";11:prefix_byte="T";
                    default:prefix_byte="E";
                endcase
            end else if(!ack)begin
                case(index)
                    0:prefix_byte="s";1:prefix_byte="a";2:prefix_byte="m";3:prefix_byte="p";
                    4:prefix_byte="l";5:prefix_byte="e";6:prefix_byte="s";default:prefix_byte=":";
                endcase
            end else begin
                case(index)
                    0:prefix_byte="[";1:prefix_byte="A";2:prefix_byte="C";3:prefix_byte="K";
                    4:prefix_byte=" ";5:prefix_byte=failed ? "E" : "O";
                    6:prefix_byte=failed ? "R" : "K";
                    7:prefix_byte=failed ? "R" : "]";
                    8:prefix_byte=failed ? "]" : " ";default:prefix_byte=" ";
                endcase
            end
        end
    endfunction
    assign consume_ack=state==IDLE && ack_pending;
    assign consume_notice=state==IDLE && !ack_pending && notice_pending;
    assign consume_event=state==IDLE && !ack_pending && !notice_pending && event_pending;
    assign tx_byte_valid=state==PREFIX || state==SIGN ||
                         (state==DIGIT && (digit_started || digit_index==0 || current_digit!=0)) ||
                         state==SEPARATOR || state==NEWLINE;
    always @*begin
        tx_byte_data=8'h0a;
        case(state)
            PREFIX:tx_byte_data=prefix_byte(frame_ack,frame_error,frame_event,prefix_index);
            SIGN:tx_byte_data="-";
            DIGIT:tx_byte_data=8'h30+{4'd0,current_digit};
            SEPARATOR:tx_byte_data=",";
            default:tx_byte_data=8'h0a;
        endcase
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;channel<=0;last_channel<=0;digit_index<=9;prefix_index<=0;
            frame_ack<=0;frame_error<=0;frame_event<=0;digit_started<=0;number_negative<=0;
            binary_shift<=0;decimal_digits<=0;conversion_step<=0;
            sample_age<=0;sample_due<=0;tx_line_count<=0;
            held_values<=0;
        end else begin
            if(sample_age>=SAMPLE_CYCLES-1)begin sample_age<=0;sample_due<=1;end
            else sample_age<=sample_age+1'b1;
            case(state)
                IDLE:begin
                    channel<=0;prefix_index<=0;frame_event<=0;
                    if(ack_pending)begin
                        // Slot0 leaves first. The192-bit ACK is zero-extended
                        // into the same snapshot; all six native integers stay.
                        held_values<={ack_value,{24'd0,ack_status},{28'd0,ack_field},
                                     {30'd0,ack_wheel},{28'd0,ack_op},{16'd0,ack_seq}};
                        frame_ack<=1;frame_error<=ack_status!=0;last_channel<=5;state<=PREFIX;
                    end else if(notice_pending)begin
                        held_values<={32'd0,{24'd0,notice_status},32'd0,32'd0,
                                     32'hFFFFFFFF,{16'd0,notice_seq}};
                        frame_ack<=1;frame_error<=1;last_channel<=5;state<=PREFIX;
                    end else if(event_pending)begin
                        frame_ack<=0;frame_error<=0;frame_event<=1;state<=PREFIX;
                    end else if(sample_due)begin
                        held_values<=sample_values;
                        frame_ack<=0;frame_error<=0;last_channel<=CHANNELS-1;sample_due<=0;state<=PREFIX;
                    end
                end
                PREFIX:if(tx_byte_ready)begin
                    if(frame_event && prefix_index==12)state<=NEWLINE;
                    else if(!frame_event && prefix_index==(frame_ack ? (frame_error ? 9 : 8) : 7))state<=LOAD;
                    else prefix_index<=prefix_index+1'b1;
                end
                LOAD:begin
                    binary_shift<=selected_magnitude;number_negative<=selected_value[31];
                    decimal_digits<=0;conversion_step<=0;digit_started<=0;state<=CONVERT;
                end
                CONVERT:begin
                    decimal_digits<={adjusted_digits[38:0],binary_shift[31]};
                    binary_shift<={binary_shift[30:0],1'b0};
                    if(conversion_step==31)begin digit_index<=9;state<=number_negative ? SIGN : DIGIT;end
                    else conversion_step<=conversion_step+1'b1;
                end
                SIGN:if(tx_byte_ready)state<=DIGIT;
                DIGIT:begin
                    if(!digit_started && digit_index!=0 && current_digit==0)begin
                        digit_index<=digit_index-1'b1;
                        decimal_digits<={decimal_digits[35:0],4'd0};
                    end
                    else if(tx_byte_ready)begin
                        digit_started<=1;
                        decimal_digits<={decimal_digits[35:0],4'd0};
                        if(digit_index==0)state<=channel==last_channel ? NEWLINE : SEPARATOR;
                        else digit_index<=digit_index-1'b1;
                    end
                end
                SEPARATOR:if(tx_byte_ready)begin
                    held_values<=held_values>>32;channel<=channel+1'b1;state<=LOAD;
                end
                NEWLINE:if(tx_byte_ready)begin tx_line_count<=tx_line_count+1'b1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
