// Framed motor commands, separate from legacy P/F/S/N and RTC calibration.
// @A ARM, @R RUN, @S STOP, @C CLEAR, @Z ZERO, @U UNLOCK POSITION,
// @H heartbeat; @W0/1 wheel, @L0/1 edit loop, @F0..8 field,
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
