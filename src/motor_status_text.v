// Coherent 16x16 LCD/UART motor snapshot at 10Hz. Serial double-dabble keeps
// decimal conversion off the control and pixel critical paths. No rpm/metres
// are invented: speed is quadrature counts/10ms and position is counts.
module motor_status_text #(parameter integer CLK_HZ=50000000)(
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
    output reg text_valid
);
    localparam IDLE=0, LOAD=1, CONVERT=2, STORE=3, COMMIT=4;
    reg [2:0] state;
    reg [31:0] divider;
    reg [3:0] field;
    reg [5:0] bit_count;
    reg [63:0] values [0:15];
    reg [15:0] negative;
    reg [127:0] rows [0:15];
    reg [127:0] state_row, scope_row, field_row, mode_row;
    reg fractional;
    reg [63:0] shift_value;
    reg [79:0] decimal;
    integer i;
    function [79:0] add_three;
        input [79:0] d;
        integer n;
        begin
            add_three=d;
            for (n=0;n<20;n=n+1)
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
                8:field_label="FIELD ENC DIR   "; default:field_label="FIELD INVALID   ";
            endcase
        end
    endfunction
    wire [79:0] adjusted=add_three(decimal);
    // The eight-count display saturates rather than silently wrapping a value.
    wire big_count=values[field]>64'd99999999;
    wire [63:0] count_digits=big_count ? "OVERFLOW" :
        {a(decimal[31:28]),a(decimal[27:24]),a(decimal[23:20]),a(decimal[19:16]),
         a(decimal[15:12]),a(decimal[11:8]),a(decimal[7:4]),a(decimal[3:0])};
    wire [7:0] sign_char=negative[field] && values[field]!=0 ? "-" : "+";
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; divider<=0; field<=0; bit_count<=0;
            negative<=0; fractional<=0; shift_value<=0; decimal<=0;
            state_row<=0; scope_row<=0; field_row<=0; mode_row<=0;
            text_rows<=0; text_valid<=0;
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
                        values[3]<=({32'd0,abs32(selected_value)}*64'd10000)>>8;
                        values[4]<=({32'd0,selected_step}*64'd10000)>>8;
                    end else begin values[3]<={32'd0,abs32(selected_value)}; values[4]<={32'd0,selected_step}; end
                    values[5]<={32'd0,abs32($signed(target_effective_left))};
                    values[6]<={32'd0,abs32(speed_left)};
                    values[7]<={32'd0,abs32($signed(target_effective_right))};
                    values[8]<={32'd0,abs32(speed_right)};
                    values[9]<={32'd0,abs32(position_left)};
                    values[10]<={32'd0,abs32(position_right)};
                    values[11]<={32'd0,abs32($signed(duty_left))};
                    values[12]<={32'd0,abs32($signed(duty_right))};
                    values[13]<={56'd0,fault_left}*64'd1000+{56'd0,fault_right};
                    values[14]<={48'd0,invalid_left}*64'd100000+{48'd0,invalid_right};
                    negative[3]<=selected_value[31]; negative[4]<=0;
                    negative[5]<=target_effective_left[15]; negative[6]<=speed_left[31];
                    negative[7]<=target_effective_right[15]; negative[8]<=speed_right[31];
                    negative[9]<=position_left[31]; negative[10]<=position_right[31];
                    negative[11]<=duty_left[15]; negative[12]<=duty_right[15];
                    field<=0; state<=LOAD;
                end
                LOAD:begin shift_value<=values[field]; decimal<=0; bit_count<=0; state<=CONVERT; end
                CONVERT:begin
                    decimal<={adjusted[78:0],shift_value[63]}; shift_value<={shift_value[62:0],1'b0};
                    if (bit_count==63) state<=STORE;
                    else bit_count<=bit_count+1'b1;
                end
                STORE:begin
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
                    if (field==15) state<=COMMIT;
                    else begin field<=field+1'b1; state<=LOAD; end
                end
                COMMIT:begin
                    for (i=0;i<16;i=i+1) text_rows[2047-i*128 -: 128]<=rows[i];
                    text_valid<=1; state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
