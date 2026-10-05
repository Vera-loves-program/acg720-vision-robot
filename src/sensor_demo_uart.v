// Human-readable unit-bearing telemetry, 115200 8N1, default 10 Hz.
// LCD and Bluetooth share the same processed snapshot. Old numeric CSV is
// intentionally replaced; use a text serial terminal, not VOFA FireWater.
module sensor_demo_uart #(
    parameter CLK_HZ = 50000000,
    parameter UART_BAUD = 115200,
    parameter REPORT_HZ = 10
)(
    input wire clk, reset_n,
    input wire imu_valid,
    input wire [7:0] imu_who, imu_error,
    input wire [31:0] imu_samples,
    input wire signed [15:0] gx, gy, gz, ax, ay, az,
    input wire us_valid,
    input wire [7:0] us_error,
    input wire [31:0] us_samples,
    input wire [15:0] distance_mm,
    input wire [23:0] period_cycles,
    input wire [31:0] command_count,
    input wire [7:0] last_command,
    input wire [31:0] rx_errors,
    output wire [1023:0] display_text,
    output wire display_text_valid,
    output wire tx
);
    wire [167:0] formatted_meta;
    sensor_demo_text #(.CLK_HZ(CLK_HZ), .REPORT_HZ(REPORT_HZ)) u_format (
        .clk(clk), .reset_n(reset_n), .imu_valid(imu_valid), .us_valid(us_valid),
        .imu_error(imu_error), .us_error(us_error), .imu_samples(imu_samples),
        .us_samples(us_samples), .gx(gx), .gy(gy), .gz(gz), .ax(ax), .ay(ay), .az(az),
        .distance_mm(distance_mm), .period_cycles(period_cycles),
        .command_count(command_count), .rx_errors(rx_errors), .last_command(last_command),
        .text_rows(display_text), .metadata(formatted_meta), .text_valid(display_text_valid)
    );
    localparam IDLE=0, BODY=1, BODY_SEPARATOR=2, PREFIX=3, LOAD=4, CONVERT=5,
        DIGIT=6, AFTER_DIGIT=7, NEXT_FIELD=8, LAST_PREFIX=9, LAST_CHAR=10,
        CR=11, LF=12, WAIT_ACCEPT=13, WAIT_DONE=14;
    reg [3:0] state, after_byte;
    reg [1023:0] held_text;
    reg [167:0] held_meta;
    reg [6:0] text_index;
    reg [2:0] field_index, prefix_index;
    reg [3:0] digit_index;
    reg digit_started;
    reg [31:0] binary_shift;
    reg [39:0] decimal_digits;
    reg [5:0] conversion_step;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    sensor_demo_uart_tx_8n1 #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) u_tx (
        .clk(clk), .reset_n(reset_n), .start(tx_start), .data(tx_data), .busy(tx_busy), .tx(tx)
    );
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
    function [63:0] prefix;
        input [2:0] f;
        begin
            case (f)
                0: prefix=" SEQ=   "; 1: prefix=" IMU_N= "; 2: prefix=" US_N=  ";
                3: prefix=" CMD=   "; default: prefix=" RXERR= ";
            endcase
        end
    endfunction
    wire [3:0] prefix_length=(field_index==1 || field_index==4) ? 7 : field_index==2 ? 6 : 5;
    wire [63:0] prefix_word=prefix(field_index);
    wire [47:0] last_word=" LAST=";
    wire [39:0] adjusted_digits=add_three(decimal_digits);
    wire [3:0] current_digit=decimal_digits[digit_index*4 +: 4];
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; after_byte<=IDLE; held_text<=0; held_meta<=0;
            text_index<=0; field_index<=0; prefix_index<=0;
            binary_shift<=0; decimal_digits<=0; conversion_step<=0;
            digit_index<=9; digit_started<=0; tx_start<=0; tx_data<=0;
        end else begin
            tx_start<=0;
            case (state)
                IDLE: if (display_text_valid) begin
                    held_text<=display_text; held_meta<=formatted_meta;
                    text_index<=0; state<=BODY;
                end
                BODY: begin
                    tx_data<=held_text[1023-text_index*8 -: 8]; tx_start<=1;
                    if (text_index==127) begin
                        field_index<=0; prefix_index<=0; after_byte<=PREFIX;
                    end else begin
                        text_index<=text_index+1'b1;
                        after_byte<=text_index[3:0]==15 ? BODY_SEPARATOR : BODY;
                    end
                    state<=WAIT_ACCEPT;
                end
                BODY_SEPARATOR: begin
                    tx_data<="|"; tx_start<=1; after_byte<=BODY; state<=WAIT_ACCEPT;
                end
                PREFIX: begin
                    tx_data<=prefix_word[63-prefix_index*8 -: 8]; tx_start<=1;
                    if (prefix_index==prefix_length-1) begin prefix_index<=0; after_byte<=LOAD; end
                    else begin prefix_index<=prefix_index+1'b1; after_byte<=PREFIX; end
                    state<=WAIT_ACCEPT;
                end
                LOAD: begin
                    binary_shift<=held_meta[167-field_index*32 -: 32];
                    decimal_digits<=0; conversion_step<=0; state<=CONVERT;
                end
                CONVERT: begin
                    decimal_digits<={adjusted_digits[38:0],binary_shift[31]}; binary_shift<={binary_shift[30:0],1'b0};
                    if (conversion_step==31) begin digit_index<=9; digit_started<=0; state<=DIGIT; end
                    else conversion_step<=conversion_step+1'b1;
                end
                DIGIT: begin
                    if (!digit_started && digit_index!=0 && current_digit==0) digit_index<=digit_index-1'b1;
                    else begin
                        tx_data<=8'h30+{4'd0,current_digit}; tx_start<=1; digit_started<=1;
                        after_byte<=AFTER_DIGIT; state<=WAIT_ACCEPT;
                    end
                end
                AFTER_DIGIT: begin
                    if (digit_index==0) state<=NEXT_FIELD;
                    else begin digit_index<=digit_index-1'b1; state<=DIGIT; end
                end
                NEXT_FIELD: begin
                    prefix_index<=0;
                    if (field_index==4) state<=LAST_PREFIX;
                    else begin field_index<=field_index+1'b1; state<=PREFIX; end
                end
                LAST_PREFIX: begin
                    tx_data<=last_word[47-prefix_index*8 -: 8]; tx_start<=1;
                    if (prefix_index==5) after_byte<=LAST_CHAR;
                    else begin prefix_index<=prefix_index+1'b1; after_byte<=LAST_PREFIX; end
                    state<=WAIT_ACCEPT;
                end
                LAST_CHAR: begin
                    tx_data<=held_meta[7:0]==0 ? "-" : held_meta[7:0];
                    tx_start<=1; after_byte<=CR; state<=WAIT_ACCEPT;
                end
                CR: begin tx_data<=8'h0d; tx_start<=1; after_byte<=LF; state<=WAIT_ACCEPT; end
                LF: begin tx_data<=8'h0a; tx_start<=1; after_byte<=IDLE; state<=WAIT_ACCEPT; end
                WAIT_ACCEPT: if (tx_busy) state<=WAIT_DONE;
                WAIT_DONE: if (!tx_busy) state<=after_byte;
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// Private UART transmitter, kept in this file so only one project source needs
// adding. A start pulse is accepted only while idle; data is then latched.
module sensor_demo_uart_tx_8n1 #(
    parameter CLK_HZ = 50000000,
    parameter BAUD = 115200
)(
    input wire clk,
    input wire reset_n,
    input wire start,
    input wire [7:0] data,
    output reg busy,
    output reg tx
);
    localparam BIT_CLOCKS = (CLK_HZ + BAUD/2) / BAUD;
    reg [31:0] bit_clock_count;
    reg [3:0] bit_index;
    reg [7:0] latched_data;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy <= 0;
            tx <= 1;
            bit_clock_count <= 0;
            bit_index <= 0;
            latched_data <= 0;
        end else if (!busy) begin
            tx <= 1;
            if (start) begin
                latched_data <= data;
                bit_clock_count <= 0;
                bit_index <= 0;
                busy <= 1;
                tx <= 0; // Start bit begins here.
            end
        end else if (bit_clock_count == BIT_CLOCKS - 1) begin
            bit_clock_count <= 0;
            if (bit_index < 8) begin
                tx <= latched_data[bit_index]; // LSB first.
                bit_index <= bit_index + 1'b1;
            end else if (bit_index == 8) begin
                tx <= 1; // Stop bit.
                bit_index <= 9;
            end else begin
                tx <= 1;
                busy <= 0; // The complete stop-bit interval has elapsed.
            end
        end else begin
            bit_clock_count <= bit_clock_count + 1'b1;
        end
    end
endmodule
