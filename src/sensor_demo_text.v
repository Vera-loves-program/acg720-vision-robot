// Format ONE coherent sample into eight 16-character ASCII rows.
// LSM6DSV16X setup: +/-2000 dps (0.070 dps/LSB), +/-4g (0.000122g/LSB).
// Gyro: 2 decimals; accel: 3 decimals rounded to 0.001g, including gravity.
// No attitude estimation/calibration is implied. Invalid axes are dashes.
// Both Bluetooth and LCD use the same rows; conversion runs at 50 MHz, not
// on the per-pixel path. Serial divide and double-dabble avoid a long divider.
module sensor_demo_text #(
    parameter integer CLK_HZ = 50000000,
    parameter integer REPORT_HZ = 10,
    parameter integer LIDAR_MODE = 0
)(
    input wire clk, reset_n,
    input wire imu_valid, us_valid,
    input wire [7:0] imu_error, us_error,
    input wire [31:0] imu_samples, us_samples,
    input wire signed [15:0] gx, gy, gz, ax, ay, az,
    input wire [15:0] distance_mm,
    input wire [23:0] period_cycles,
    input wire [31:0] command_count, rx_errors,
    input wire [7:0] last_command,
    output reg [1023:0] text_rows,
    output reg [167:0] metadata,
    output reg [113:0] processed_axes,
    output reg processed_imu_valid,
    output reg text_valid
);
    localparam IDLE=0, LOAD=1, DIVIDE=2, BCD_START=3, BCD=4, STORE=5, COMMIT=6;
    reg [2:0] state;
    reg [31:0] report_count, sequence_count;
    wire [31:0] interval = period_cycles == 0 ? CLK_HZ / REPORT_HZ : {8'd0,period_cycles};
    reg [15:0] magnitudes [0:5];
    reg [5:0] signs;
    reg signed [18:0] processed_work[0:5];
    reg imu_ok, us_ok;
    reg [7:0] imu_err, us_err;
    reg [15:0] distance;
    reg [167:0] held_metadata;
    reg [3:0] field;
    reg [4:0] bit_count;
    // abs(signed16)<=32768: gyro*7<=229376 (18 bits), and
    // accel*122+500<=3998196 (22 bits), /1000<=3998 (12 bits).
    // Six BCD digits cover every emitted numeric field. Metadata/counters
    // remain full width; these bounds apply only to the displayed values.
    reg [17:0] value;
    reg [21:0] dividend;
    reg [11:0] quotient;
    reg [9:0] remainder;
    reg [23:0] decimal;
    reg [127:0] rows [0:7];
    reg [23:0] imu_error_ascii, us_error_ascii;
    integer i;
    function [15:0] magnitude;
        input [15:0] raw;
        begin magnitude = raw[15] ? (~raw + 16'd1) : raw; end
    endfunction
    function [23:0] add_three;
        input [23:0] digits;
        integer d;
        reg [23:0] adjusted;
        begin
            adjusted=digits;
            for (d=0; d<6; d=d+1)
                if (digits[d*4 +: 4]>=5) adjusted[d*4 +: 4]=digits[d*4 +: 4]+4'd3;
            add_three=adjusted;
        end
    endfunction
    function [7:0] ascii;
        input [3:0] digit;
        begin ascii=8'h30+{4'd0,digit}; end
    endfunction
    wire [23:0] adjusted = add_three(decimal);
    wire [10:0] trial = {remainder, dividend[21]};
    wire divide_bit = trial >= 11'd1000;
    wire [10:0] next_remainder = divide_bit ? trial - 11'd1000 : trial;
    wire [7:0] sign_ascii = signs[field[2:0]] && decimal != 0 ? 8'h2d : 8'h2b;
    wire [7:0] axis_ascii = (field==0 || field==3) ? "X" : (field==1 || field==4) ? "Y" : "Z";
    wire [127:0] gyro_row = {"G",axis_ascii," ",sign_ascii,
        ascii(decimal[23:20]),ascii(decimal[19:16]),ascii(decimal[15:12]),ascii(decimal[11:8]),
        ".",ascii(decimal[7:4]),ascii(decimal[3:0]),"DPS  "};
    wire [127:0] accel_row = {"A",axis_ascii," ",sign_ascii,ascii(decimal[15:12]),
        ".",ascii(decimal[11:8]),ascii(decimal[7:4]),ascii(decimal[3:0]),"G      "};
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; report_count<=0; sequence_count<=0; field<=0;
            signs<=0; imu_ok<=0; us_ok<=0; imu_err<=1; us_err<=5; distance<=0;
            held_metadata<=0; metadata<=0; text_valid<=0;processed_axes<=0;processed_imu_valid<=0;
            value<=0; dividend<=0; quotient<=0; remainder<=0; decimal<=0; bit_count<=0;
            imu_error_ascii<="001"; us_error_ascii<="005";
            text_rows<={"SENSOR WAIT     ","GX ----.--DPS   ","GY ----.--DPS   ","GZ ----.--DPS   ",
                        "AX -.---G       ","AY -.---G       ","AZ -.---G       ","US WAIT         "};
            for (i=0;i<6;i=i+1)begin magnitudes[i]<=0;processed_work[i]<=0;end
            for (i=0;i<8;i=i+1) rows[i]<=0;
        end else begin
            text_valid<=0;
            if (report_count>=interval-1) report_count<=0;
            else report_count<=report_count+1'b1;
            case (state)
                IDLE: if (report_count>=interval-1) begin
                    magnitudes[0]<=magnitude(gx); magnitudes[1]<=magnitude(gy); magnitudes[2]<=magnitude(gz);
                    magnitudes[3]<=magnitude(ax); magnitudes[4]<=magnitude(ay); magnitudes[5]<=magnitude(az);
                    signs<={az[15],ay[15],ax[15],gz[15],gy[15],gx[15]};
                    imu_ok<=imu_valid; us_ok<=us_valid; imu_err<=imu_error; us_err<=us_error; distance<=distance_mm;
                    held_metadata<={sequence_count,imu_samples,us_samples,command_count,rx_errors,last_command};
                    sequence_count<=sequence_count+1'b1;
                    field<=0; state<=LOAD;
                end
                LOAD: begin
                    if (field<3) begin
                        value<={2'd0,magnitudes[field]} * 18'd7; // centi-dps
                        state<=BCD_START;
                    end else if (field<6) begin
                        dividend<={6'd0,magnitudes[field]} * 22'd122 + 22'd500;
                        quotient<=0; remainder<=0; bit_count<=0; state<=DIVIDE;
                    end else begin
                        value<=field==6 ? {2'd0,distance} : field==7 ? {10'd0,imu_err} : {10'd0,us_err};
                        state<=BCD_START;
                    end
                end
                DIVIDE: begin
                    dividend<={dividend[20:0],1'b0};
                    quotient<={quotient[10:0],divide_bit}; remainder<=next_remainder[9:0];
                    if (bit_count==21) begin value<={6'd0,quotient[10:0],divide_bit}; state<=BCD_START; end
                    else bit_count<=bit_count+1'b1;
                end
                BCD_START: begin
                    // Reuse the already scaled value before BCD consumes it.
                    // Gyro is centi-dps, acceleration milli-g; no new dividers.
                    if(field<6)processed_work[field[2:0]]<=signs[field[2:0]] ?
                        -$signed({1'b0,value}) : $signed({1'b0,value});
                    decimal<=0;bit_count<=0;state<=BCD;
                end
                BCD: begin
                    decimal<={adjusted[22:0],value[17]}; value<={value[16:0],1'b0};
                    if (bit_count==17) state<=STORE;
                    else bit_count<=bit_count+1'b1;
                end
                STORE: begin
                    if (field<3) rows[field+1'b1]<=imu_ok ? gyro_row : {"G",axis_ascii," ----.--DPS   "};
                    else if (field<6) rows[field+1'b1]<=imu_ok ? accel_row : {"A",axis_ascii," -.---G       "};
                    else if (field==6) rows[7]<={LIDAR_MODE ? "LD " : "US ",ascii(decimal[15:12]),ascii(decimal[11:8]),
                        ascii(decimal[7:4]),ascii(decimal[3:0])," MM      "};
                    else if (field==7) imu_error_ascii<={ascii(decimal[11:8]),ascii(decimal[7:4]),ascii(decimal[3:0])};
                    else us_error_ascii<={ascii(decimal[11:8]),ascii(decimal[7:4]),ascii(decimal[3:0])};
                    if (field==8) state<=COMMIT;
                    else begin field<=field+1'b1; state<=LOAD; end
                end
                COMMIT: begin
                    text_rows<={imu_ok ? "I:OK    " : {"I:E",imu_error_ascii,"  "},
                        us_ok ? (LIDAR_MODE ? "L:OK    " : "U:OK    ") : {LIDAR_MODE ? "L:E" : "U:E",us_error_ascii,"  "},
                        rows[1],rows[2],rows[3],rows[4],rows[5],rows[6],
                        us_ok ? rows[7] : {LIDAR_MODE ? "LD INVALID " : "US INVALID ",us_error_ascii,"  "}};
                    processed_axes<={processed_work[5],processed_work[4],processed_work[3],
                                     processed_work[2],processed_work[1],processed_work[0]};
                    processed_imu_valid<=imu_ok;
                    metadata<=held_metadata; text_valid<=1; state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// Robot-only IMU byte snapshot: one18-bit decimal converter and one serial
// /1000 divider. The receiving dual-bank RAM owns the complete256-byte frame;
// no parallel ASCII row bank, metadata bank or duplicate processed-axis bank.
// Rows1..6 retain the legacy six-axis spelling, precision and sensor units.
module sensor_demo_text_compact #(
    parameter integer CLK_HZ=50000000,
    parameter integer REPORT_HZ=10
)(
    input wire clk,reset_n,imu_valid,
    input wire signed [15:0] gx,gy,gz,ax,ay,az,
    output reg text_valid,
    output wire text_byte_valid,
    output reg [7:0] text_byte_data,
    output reg [7:0] text_byte_index,
    input wire text_byte_ready
);
    localparam integer INTERVAL=CLK_HZ/REPORT_HZ<1 ? 1 : CLK_HZ/REPORT_HZ;
    localparam integer TIMER_BITS=INTERVAL<=1 ? 1 : $clog2(INTERVAL);
    localparam [2:0] IDLE=0,LOAD=1,SCALE=2,DIVIDE=3,BCD_START=4,BCD=5,EMIT=6,COMMIT=7;
    reg [2:0] state,field;
    reg [TIMER_BITS-1:0] report_count;
    reg [15:0] magnitudes[0:5];
    reg [5:0] signs;
    reg imu_ok;
    reg [17:0] value;
    reg [21:0] dividend;
    reg [11:0] quotient;
    reg [9:0] remainder;
    reg [4:0] bit_count;
    reg [23:0] decimal;
    wire [21:0] raw_magnitude={6'd0,magnitudes[field]};
    wire [10:0] divide_trial={remainder,dividend[21]};
    wire divide_bit=divide_trial>=11'd1000;
    wire [10:0] next_remainder=divide_bit ? divide_trial-11'd1000 : divide_trial;
    function [15:0] magnitude;
        input [15:0] raw;
        begin magnitude=raw[15] ? (~raw+16'd1) : raw;end
    endfunction
    function [23:0] add_three;
        input [23:0] digits;
        integer digit;
        reg [23:0] adjusted;
        begin
            adjusted=digits;
            for(digit=0;digit<6;digit=digit+1)
                if(digits[digit*4+:4]>=5)adjusted[digit*4+:4]=digits[digit*4+:4]+4'd3;
            add_three=adjusted;
        end
    endfunction
    function [7:0] ascii;
        input [3:0] digit;
        begin ascii=8'h30+{4'd0,digit};end
    endfunction
    wire [23:0] adjusted=add_three(decimal);
    wire [3:0] column=text_byte_index[3:0];
    wire [7:0] axis=(field==0 || field==3) ? "X" : (field==1 || field==4) ? "Y" : "Z";
    wire [7:0] sign_ascii=signs[field] && decimal!=0 ? "-" : "+";
    wire [127:0] header=imu_ok ? "IMU READY       " : "IMU INVALID     ";
    assign text_byte_valid=state==EMIT;
    always @* begin
        text_byte_data=8'h20;
        if(text_byte_index<16)text_byte_data=header[127-{column,3'b000}-:8];
        else if(text_byte_index<112)begin
            if(column==0)text_byte_data=field<3 ? "G" : "A";
            else if(column==1)text_byte_data=axis;
            else if(imu_ok && field<3)case(column)
                3:text_byte_data=sign_ascii;
                4:text_byte_data=ascii(decimal[23:20]);5:text_byte_data=ascii(decimal[19:16]);
                6:text_byte_data=ascii(decimal[15:12]);7:text_byte_data=ascii(decimal[11:8]);
                8:text_byte_data=".";9:text_byte_data=ascii(decimal[7:4]);10:text_byte_data=ascii(decimal[3:0]);
                11:text_byte_data="D";12:text_byte_data="P";13:text_byte_data="S";
                default:begin end
            endcase
            else if(imu_ok)case(column)
                3:text_byte_data=sign_ascii;4:text_byte_data=ascii(decimal[15:12]);5:text_byte_data=".";
                6:text_byte_data=ascii(decimal[11:8]);7:text_byte_data=ascii(decimal[7:4]);8:text_byte_data=ascii(decimal[3:0]);
                9:text_byte_data="G";
                default:begin end
            endcase
            else if(field<3)case(column)
                3,4,5,6,8,9:text_byte_data="-";
                7:text_byte_data=".";10:text_byte_data="D";11:text_byte_data="P";12:text_byte_data="S";
                default:begin end
            endcase
            else case(column)
                3,5,6,7:text_byte_data="-";4:text_byte_data=".";8:text_byte_data="G";
                default:begin end
            endcase
        end
    end
    integer index;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;field<=0;report_count<=0;signs<=0;imu_ok<=0;
            value<=0;dividend<=0;quotient<=0;remainder<=0;bit_count<=0;decimal<=0;
            text_valid<=0;text_byte_index<=0;
            for(index=0;index<6;index=index+1)magnitudes[index]<=0;
        end else begin
            text_valid<=0;
            if(report_count==INTERVAL-1)report_count<=0;else report_count<=report_count+1'b1;
            case(state)
                IDLE:if(report_count==INTERVAL-1)begin
                    magnitudes[0]<=magnitude(gx);magnitudes[1]<=magnitude(gy);magnitudes[2]<=magnitude(gz);
                    magnitudes[3]<=magnitude(ax);magnitudes[4]<=magnitude(ay);magnitudes[5]<=magnitude(az);
                    signs<={az[15],ay[15],ax[15],gz[15],gy[15],gx[15]};imu_ok<=imu_valid;
                    field<=0;text_byte_index<=0;state<=EMIT;
                end
                LOAD:begin
                    if(!imu_ok)begin decimal<=0;state<=EMIT;end
                    else if(field<3)begin
                        value<=({2'd0,magnitudes[field]}<<3)-{2'd0,magnitudes[field]};state<=BCD_START;
                    end else begin
                        dividend<=(raw_magnitude<<7)-(raw_magnitude<<2);state<=SCALE;
                    end
                end
                SCALE:begin
                    dividend<=dividend-(raw_magnitude<<1)+22'd500;
                    quotient<=0;remainder<=0;bit_count<=0;state<=DIVIDE;
                end
                DIVIDE:begin
                    dividend<={dividend[20:0],1'b0};quotient<={quotient[10:0],divide_bit};remainder<=next_remainder[9:0];
                    if(bit_count==21)begin value<={6'd0,quotient[10:0],divide_bit};state<=BCD_START;end
                    else bit_count<=bit_count+1'b1;
                end
                BCD_START:begin decimal<=0;bit_count<=0;state<=BCD;end
                BCD:begin
                    decimal<={adjusted[22:0],value[17]};value<={value[16:0],1'b0};
                    if(bit_count==17)state<=EMIT;else bit_count<=bit_count+1'b1;
                end
                EMIT:if(text_byte_ready)begin
                    if(text_byte_index==255)state<=COMMIT;
                    else begin
                        text_byte_index<=text_byte_index+1'b1;
                        if(text_byte_index==15)begin field<=0;state<=LOAD;end
                        else if(column==15 && text_byte_index<111)begin field<=field+1'b1;state<=LOAD;end
                    end
                end
                COMMIT:begin text_valid<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
