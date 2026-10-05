// Format ONE coherent sample into eight 16-character ASCII rows.
// LSM6DSV16X setup: +/-2000 dps (0.070 dps/LSB), +/-4g (0.000122g/LSB).
// Gyro: 2 decimals; accel: 3 decimals rounded to 0.001g, including gravity.
// No attitude estimation/calibration is implied. Invalid axes are dashes.
// Both Bluetooth and LCD use the same rows; conversion runs at 50 MHz, not
// on the per-pixel path. Serial divide and double-dabble avoid a long divider.
module sensor_demo_text #(
    parameter integer CLK_HZ = 50000000,
    parameter integer REPORT_HZ = 10
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
    output reg text_valid
);
    localparam IDLE=0, LOAD=1, DIVIDE=2, BCD_START=3, BCD=4, STORE=5, COMMIT=6;
    reg [2:0] state;
    reg [31:0] report_count, sequence_count;
    wire [31:0] interval = period_cycles == 0 ? CLK_HZ / REPORT_HZ : {8'd0,period_cycles};
    reg [15:0] magnitudes [0:5];
    reg [5:0] signs;
    reg imu_ok, us_ok;
    reg [7:0] imu_err, us_err;
    reg [15:0] distance;
    reg [167:0] held_metadata;
    reg [3:0] field;
    reg [5:0] bit_count;
    reg [31:0] value, dividend, quotient;
    reg [9:0] remainder;
    reg [39:0] decimal;
    reg [127:0] rows [0:7];
    reg [23:0] imu_error_ascii, us_error_ascii;
    integer i;
    function [15:0] magnitude;
        input [15:0] raw;
        begin magnitude = raw[15] ? (~raw + 16'd1) : raw; end
    endfunction
    function [39:0] add_three;
        input [39:0] digits;
        integer d;
        reg [39:0] adjusted;
        begin
            adjusted=digits;
            for (d=0; d<10; d=d+1)
                if (digits[d*4 +: 4]>=5) adjusted[d*4 +: 4]=digits[d*4 +: 4]+4'd3;
            add_three=adjusted;
        end
    endfunction
    function [7:0] ascii;
        input [3:0] digit;
        begin ascii=8'h30+{4'd0,digit}; end
    endfunction
    wire [39:0] adjusted = add_three(decimal);
    wire [10:0] trial = {remainder, dividend[31]};
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
            held_metadata<=0; metadata<=0; text_valid<=0;
            value<=0; dividend<=0; quotient<=0; remainder<=0; decimal<=0; bit_count<=0;
            imu_error_ascii<="001"; us_error_ascii<="005";
            text_rows<={"SENSOR WAIT     ","GX ----.--DPS   ","GY ----.--DPS   ","GZ ----.--DPS   ",
                        "AX -.---G       ","AY -.---G       ","AZ -.---G       ","US WAIT         "};
            for (i=0;i<6;i=i+1) magnitudes[i]<=0;
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
                        value<={16'd0,magnitudes[field]} * 32'd7; // centi-dps
                        state<=BCD_START;
                    end else if (field<6) begin
                        dividend<={16'd0,magnitudes[field]} * 32'd122 + 32'd500;
                        quotient<=0; remainder<=0; bit_count<=0; state<=DIVIDE;
                    end else begin
                        value<=field==6 ? {16'd0,distance} : field==7 ? {24'd0,imu_err} : {24'd0,us_err};
                        state<=BCD_START;
                    end
                end
                DIVIDE: begin
                    dividend<={dividend[30:0],1'b0};
                    quotient<={quotient[30:0],divide_bit}; remainder<=next_remainder[9:0];
                    if (bit_count==31) begin value<={quotient[30:0],divide_bit}; state<=BCD_START; end
                    else bit_count<=bit_count+1'b1;
                end
                BCD_START: begin decimal<=0; bit_count<=0; state<=BCD; end
                BCD: begin
                    decimal<={adjusted[38:0],value[31]}; value<={value[30:0],1'b0};
                    if (bit_count==31) state<=STORE;
                    else bit_count<=bit_count+1'b1;
                end
                STORE: begin
                    if (field<3) rows[field+1'b1]<=imu_ok ? gyro_row : {"G",axis_ascii," ----.--DPS   "};
                    else if (field<6) rows[field+1'b1]<=imu_ok ? accel_row : {"A",axis_ascii," -.---G       "};
                    else if (field==6) rows[7]<={"US ",ascii(decimal[15:12]),ascii(decimal[11:8]),
                        ascii(decimal[7:4]),ascii(decimal[3:0])," MM      "};
                    else if (field==7) imu_error_ascii<={ascii(decimal[11:8]),ascii(decimal[7:4]),ascii(decimal[3:0])};
                    else us_error_ascii<={ascii(decimal[11:8]),ascii(decimal[7:4]),ascii(decimal[3:0])};
                    if (field==8) state<=COMMIT;
                    else begin field<=field+1'b1; state<=LOAD; end
                end
                COMMIT: begin
                    text_rows<={imu_ok ? "I:OK    " : {"I:E",imu_error_ascii,"  "},
                        us_ok ? "U:OK    " : {"U:E",us_error_ascii,"  "},
                        rows[1],rows[2],rows[3],rows[4],rows[5],rows[6],
                        us_ok ? rows[7] : {"US INVALID ",us_error_ascii,"  "}};
                    metadata<=held_metadata; text_valid<=1; state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
