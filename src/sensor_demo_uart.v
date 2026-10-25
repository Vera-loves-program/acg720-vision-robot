// Human-readable unit-bearing telemetry, 115200 8N1, default 10 Hz.
// LCD and Bluetooth share the same processed snapshot. Old numeric CSV is
// legacy format. VOFA_MODE keeps only the common LCD sensor formatter; the
// active robot uses its separate FireWater/ACK transmitter and drains bytes.
// Test mode emits six processed IMU lines and signed left/right encoder speed
// in COUNT/10MS. The wheel pair is captured when completed IMU text arrives:
// this is a near-time snapshot, not an atomic IMU/encoder hardware sample.
// PID/NAV producers stream one indexed byte per accepted handshake directly
// into a single-clock Normal RAM. A report locks completed bank identities.
// Until all 256 ordered bytes arrive, reports use the preceding complete text.
module sensor_demo_uart #(
    parameter CLK_HZ = 50000000,
    parameter UART_BAUD = 115200,
    parameter REPORT_HZ = 10,
    parameter ACTUATOR_TEST = 0,
    parameter ROBOT_MODE = 0,
    parameter VOFA_MODE = 0
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
    input wire motor_byte_valid,
    input wire [7:0] motor_byte_data,motor_byte_index,
    output wire motor_byte_ready,
    input wire robot_byte_valid,
    input wire [7:0] robot_byte_data,robot_byte_index,
    output wire robot_byte_ready,
    input wire signed [31:0] wheel_speed_left, wheel_speed_right,
    output wire [1023:0] display_text,
    output wire display_text_valid,
    output wire [31:0] tx_line_count,
    output wire [113:0] processed_axes,
    output wire processed_imu_valid,
    output wire tx
);
    wire [167:0] formatted_meta;
    sensor_demo_text #(.CLK_HZ(CLK_HZ), .REPORT_HZ(REPORT_HZ),.LIDAR_MODE(ROBOT_MODE)) u_format (
        .clk(clk), .reset_n(reset_n), .imu_valid(imu_valid), .us_valid(us_valid),
        .imu_error(imu_error), .us_error(us_error), .imu_samples(imu_samples),
        .us_samples(us_samples), .gx(gx), .gy(gy), .gz(gz), .ax(ax), .ay(ay), .az(az),
        .distance_mm(distance_mm), .period_cycles(period_cycles),
        .command_count(command_count), .rx_errors(rx_errors), .last_command(last_command),
        .text_rows(display_text), .metadata(formatted_meta), .text_valid(display_text_valid),
        .processed_axes(processed_axes),.processed_imu_valid(processed_imu_valid)
    );
    generate if(VOFA_MODE!=0)begin:vofa_lcd_only
        assign motor_byte_ready=1'b1;
        assign robot_byte_ready=1'b1;
        assign tx_line_count=32'd0;
        assign tx=1'b1;
    end else begin:legacy_sender
    reg [31:0] line_count;
    assign tx_line_count=line_count;
    localparam IDLE=0, BODY=1, BODY_SEPARATOR=2, PREFIX=3, LOAD=4, CONVERT=5,
        DIGIT=6, AFTER_DIGIT=7, NEXT_FIELD=8, LAST_PREFIX=9, LAST_CHAR=10,
        CR=11, LF=12, WAIT_ACCEPT=13, WAIT_DONE=14,
        MOTOR_PREFIX=15, MOTOR_BODY=16, MOTOR_SEPARATOR=17,
        TEST_BODY=18, TEST_CR=19, TEST_LF=20, TEST_SPEED_PREFIX=21,
        TEST_SPEED_LOAD=22, TEST_SPEED_SIGN=23, TEST_SPEED_UNIT=24,
        ROBOT_PREFIX=25,ROBOT_BODY=26,ROBOT_SEPARATOR=27,TEXT_RAM_WAIT=28;
    reg [4:0] state, after_byte;
    reg [1023:0] held_text;
    reg [159:0] held_meta;
    reg [7:0] held_last_command;
    wire motor_seen,robot_seen;
    reg held_motor_seen,held_robot_seen,reading_robot;
    localparam [63:0] robot_prefix=" | NAV: ";
    reg [7:0] motor_index;
    reg [4:0] motor_prefix_index;
    localparam MOTOR_PREFIX_LENGTH=32;
    wire [MOTOR_PREFIX_LENGTH*8-1:0] motor_prefix=" | PID SPD=COUNT/10MS POS=COUNT ";
    reg [6:0] text_index;
    wire [9:0] text_byte_msb={~text_index,3'b111};
    reg [2:0] field_index, prefix_index;
    reg [3:0] digit_index;
    reg digit_started;
    reg [31:0] binary_shift;
    reg [39:0] decimal_digits;
    reg [5:0] conversion_step;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    wire text_read_valid;
    wire [7:0] text_read_data;
    wire report_begin=state==IDLE && display_text_valid;
    wire text_read_request=state==MOTOR_BODY || state==ROBOT_BODY;
    sensor_demo_text_snapshot_store #(.ROBOT_MODE(ROBOT_MODE)) u_text_store (
        .clk(clk),.reset_n(reset_n && !ACTUATOR_TEST),
        .motor_byte_valid(motor_byte_valid),.motor_byte_data(motor_byte_data),
        .motor_byte_index(motor_byte_index),.motor_byte_ready(motor_byte_ready),
        .robot_byte_valid(robot_byte_valid),.robot_byte_data(robot_byte_data),
        .robot_byte_index(robot_byte_index),.robot_byte_ready(robot_byte_ready),
        .report_begin(report_begin),.report_active(state!=IDLE),
        .motor_seen(motor_seen),.robot_seen(robot_seen),
        .read_request(text_read_request),.read_robot(state==ROBOT_BODY),
        .read_index(motor_index),.read_valid(text_read_valid),.read_data(text_read_data)
    );
    reg signed [31:0] held_speed_left, held_speed_right;
    reg [2:0] test_line;
    reg [3:0] unit_index;
    wire [31:0] selected_speed=test_line==7 ? held_speed_right : held_speed_left;
    // Unsigned two's-complement magnitude also represents abs(-2147483648).
    wire [31:0] speed_magnitude=selected_speed[31] ? (~selected_speed+32'd1) : selected_speed;
    wire [47:0] speed_prefix=test_line==7 ? "ENC R=" : "ENC L=";
    localparam [87:0] speed_unit=" COUNT/10MS";
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
            state<=IDLE; after_byte<=IDLE; held_text<=0; held_meta<=0;held_last_command<=0;
            text_index<=0; field_index<=0; prefix_index<=0;
            held_motor_seen<=0;held_robot_seen<=0;reading_robot<=0;
            motor_index<=0; motor_prefix_index<=0;
            binary_shift<=0; decimal_digits<=0; conversion_step<=0;
            digit_index<=9; digit_started<=0; tx_start<=0; tx_data<=0;
            line_count<=0;held_speed_left<=0;held_speed_right<=0;test_line<=0;unit_index<=0;
        end else begin
            tx_start<=0;
            case (state)
                IDLE: if (display_text_valid) begin
                    held_text<=display_text;
                    held_meta<=formatted_meta[167:8];held_last_command<=formatted_meta[7:0];
                    held_motor_seen<=motor_seen;
                    held_robot_seen<=robot_seen;
                    held_speed_left<=wheel_speed_left;held_speed_right<=wheel_speed_right;
                    // Skip sensor status row0; rows1..6 hold GX/GY/GZ/AX/AY/AZ.
                    text_index<=ACTUATOR_TEST ? 7'd16 : 7'd0;test_line<=0;
                    state<=ACTUATOR_TEST ? TEST_BODY : BODY;
                end
                BODY: begin
                    tx_data<=held_text[text_byte_msb-:8]; tx_start<=1;
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
                    binary_shift<=held_meta[159:128];
                    held_meta<={held_meta[127:0],32'd0};
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
                    if (digit_index==0) begin
                        if (ACTUATOR_TEST) begin unit_index<=0;state<=TEST_SPEED_UNIT;end
                        else state<=NEXT_FIELD;
                    end
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
                    tx_data<=held_last_command==0 ? "-" : held_last_command;
                    tx_start<=1; after_byte<=held_motor_seen ? MOTOR_PREFIX : CR; state<=WAIT_ACCEPT;
                    motor_index<=0; motor_prefix_index<=0;
                end
                MOTOR_PREFIX:begin
                    tx_data<=motor_prefix[MOTOR_PREFIX_LENGTH*8-1-motor_prefix_index*8 -: 8]; tx_start<=1;
                    if (motor_prefix_index==MOTOR_PREFIX_LENGTH-1) after_byte<=MOTOR_BODY;
                    else begin motor_prefix_index<=motor_prefix_index+1'b1; after_byte<=MOTOR_PREFIX; end
                    state<=WAIT_ACCEPT;
                end
                MOTOR_BODY:begin
                    reading_robot<=0;state<=TEXT_RAM_WAIT;
                end
                MOTOR_SEPARATOR:begin tx_data<="|"; tx_start<=1; after_byte<=MOTOR_BODY; state<=WAIT_ACCEPT; end
                ROBOT_PREFIX:begin
                    tx_data<=robot_prefix[63-motor_prefix_index*8-:8];tx_start<=1;
                    if(motor_prefix_index==7)after_byte<=ROBOT_BODY;
                    else begin motor_prefix_index<=motor_prefix_index+1'b1;after_byte<=ROBOT_PREFIX;end
                    state<=WAIT_ACCEPT;
                end
                ROBOT_BODY:begin
                    reading_robot<=1;state<=TEXT_RAM_WAIT;
                end
                TEXT_RAM_WAIT:if(text_read_valid)begin
                    tx_data<=text_read_data;tx_start<=1;
                    if(motor_index==255)begin
                        after_byte<=!reading_robot && ROBOT_MODE && held_robot_seen ? ROBOT_PREFIX : CR;
                        motor_index<=0;motor_prefix_index<=0;
                    end else begin
                        motor_index<=motor_index+1'b1;
                        if(reading_robot)after_byte<=motor_index[3:0]==15 ? ROBOT_SEPARATOR : ROBOT_BODY;
                        else after_byte<=motor_index[3:0]==15 ? MOTOR_SEPARATOR : MOTOR_BODY;
                    end
                    state<=WAIT_ACCEPT;
                end
                ROBOT_SEPARATOR:begin tx_data<="|";tx_start<=1;after_byte<=ROBOT_BODY;state<=WAIT_ACCEPT;end
                TEST_BODY:begin
                    tx_data<=held_text[text_byte_msb-:8];tx_start<=1;
                    if (text_index!=111) text_index<=text_index+1'b1;
                    after_byte<=text_index[3:0]==15 ? TEST_CR : TEST_BODY;
                    state<=WAIT_ACCEPT;
                end
                TEST_CR:begin tx_data<=8'h0d;tx_start<=1;after_byte<=TEST_LF;state<=WAIT_ACCEPT;end
                TEST_LF:begin
                    tx_data<=8'h0a;tx_start<=1;
                    if (test_line<5) begin test_line<=test_line+1'b1;after_byte<=TEST_BODY;end
                    else if (test_line<7) begin
                        test_line<=test_line+1'b1;prefix_index<=0;after_byte<=TEST_SPEED_PREFIX;
                    end else after_byte<=IDLE;
                    state<=WAIT_ACCEPT;
                end
                TEST_SPEED_PREFIX:begin
                    tx_data<=speed_prefix[47-prefix_index*8 -:8];tx_start<=1;
                    if (prefix_index==5) after_byte<=TEST_SPEED_SIGN;
                    else begin prefix_index<=prefix_index+1'b1;after_byte<=TEST_SPEED_PREFIX;end
                    state<=WAIT_ACCEPT;
                end
                TEST_SPEED_SIGN:begin
                    tx_data<=selected_speed[31] ? "-" : "+";tx_start<=1;
                    after_byte<=TEST_SPEED_LOAD;state<=WAIT_ACCEPT;
                end
                TEST_SPEED_LOAD:begin
                    binary_shift<=speed_magnitude;decimal_digits<=0;conversion_step<=0;state<=CONVERT;
                end
                TEST_SPEED_UNIT:begin
                    tx_data<=speed_unit[87-unit_index*8 -:8];tx_start<=1;
                    if (unit_index==10) after_byte<=TEST_CR;
                    else begin unit_index<=unit_index+1'b1;after_byte<=TEST_SPEED_UNIT;end
                    state<=WAIT_ACCEPT;
                end
                CR: begin tx_data<=8'h0d; tx_start<=1; after_byte<=LF; state<=WAIT_ACCEPT; end
                LF: begin tx_data<=8'h0a; tx_start<=1; after_byte<=IDLE; state<=WAIT_ACCEPT; end
                WAIT_ACCEPT: if (tx_busy) state<=WAIT_DONE;
                WAIT_DONE: if (!tx_busy) begin
                    state<=after_byte;
                    if (tx_data==8'h0a) line_count<=line_count+1'b1;
                end
                default: state<=IDLE;
            endcase
        end
    end
    end endgenerate
endmodule

// Three frame banks per byte stream in ONE single-clock Normal RAM.
// A producer presents valid/data/index independently of ready, and holds all
// three while ready=0. Ready denotes acceptance of that byte, not an invitation
// to start a transfer. UART reads have priority; two valid streams alternate.
// Index0 opens/restarts an unpublished frame, and its byte is written directly
// to the combinationally chosen FREE bank on THAT edge. Only ordered bytes
// 0..255 publish a frame. Missing-prefix or unexpected indices abort the open
// frame and are consumed without writing/publishing; a new0 may always restart.
// report_begin captures the OLD latest bank (and parent captures OLD seen),
// even when byte255 publishes on that edge. free_bank always excludes latest,
// which also protects the newly held bank when report_active was still0.
// One bank is latest, one may be held by a report, one remains writable. No
// full-frame source bus, capture register, wide variable slice or second port.
// The reset is only for metadata: text_ram is never reset or asynchronously read.
module sensor_demo_text_snapshot_store #(parameter ROBOT_MODE=0)(
    input wire clk,reset_n,
    input wire motor_byte_valid,
    input wire [7:0] motor_byte_data,motor_byte_index,
    output wire motor_byte_ready,
    input wire robot_byte_valid,
    input wire [7:0] robot_byte_data,robot_byte_index,
    output wire robot_byte_ready,
    input wire report_begin,report_active,
    output reg motor_seen,robot_seen,
    input wire read_request,read_robot,
    input wire [7:0] read_index,
    output reg read_valid,
    output reg [7:0] read_data
);
    reg motor_frame_open,robot_frame_open,round_robin;
    reg [1:0] latest_motor_bank,latest_robot_bank;
    reg [1:0] held_motor_bank,held_robot_bank;
    reg [1:0] write_motor_bank,write_robot_bank;
    reg [7:0] motor_expected_index,robot_expected_index;
    function [1:0] free_bank;
        input [1:0] latest_bank,held_bank;
        input held;
        begin
            if(latest_bank!=2'd0 && (!held || held_bank!=2'd0))free_bank=2'd0;
            else if(latest_bank!=2'd1 && (!held || held_bank!=2'd1))free_bank=2'd1;
            else free_bank=2'd2;
        end
    endfunction
    wire grant_motor=!read_request && motor_byte_valid &&
        (!ROBOT_MODE || !robot_byte_valid || !round_robin);
    wire grant_robot=!read_request && ROBOT_MODE && robot_byte_valid &&
        (!motor_byte_valid || round_robin);
    assign motor_byte_ready=reset_n && grant_motor;
    // An unused NAV producer must not stall in sensor-only builds. Reset still
    // deasserts BOTH ready pins, including ACTUATOR_TEST's permanently held reset.
    assign robot_byte_ready=reset_n && (!ROBOT_MODE || grant_robot);
    wire motor_byte_order_ok=motor_byte_index==8'd0 ||
        (motor_frame_open && motor_byte_index==motor_expected_index);
    wire robot_byte_order_ok=robot_byte_index==8'd0 ||
        (robot_frame_open && robot_byte_index==robot_expected_index);
    wire [1:0] motor_input_bank=motor_byte_index==8'd0 ?
        free_bank(latest_motor_bank,held_motor_bank,report_active) : write_motor_bank;
    wire [1:0] robot_input_bank=robot_byte_index==8'd0 ?
        free_bank(latest_robot_bank,held_robot_bank,report_active) : write_robot_bank;
    wire memory_write=reset_n && ((grant_motor && motor_byte_order_ok) ||
        (grant_robot && robot_byte_order_ok));
    // 0/bank/index is PID; 1/bank/index is NAV. Unused bank3 keeps the
    // 11-bit stream/bank/index concatenation free of address adders.
    wire [10:0] memory_address=read_request ?
        {read_robot,read_robot ? held_robot_bank : held_motor_bank,read_index} :
        grant_motor ? {1'b0,motor_input_bank,motor_byte_index} :
                      {1'b1,robot_input_bank,robot_byte_index};
    wire [7:0] memory_write_data=grant_motor ? motor_byte_data : robot_byte_data;
    (* syn_ramstyle="block_ram" *) reg [7:0] text_ram[0:2047];
    always @(posedge clk) begin
        if(memory_write)text_ram[memory_address]<=memory_write_data;
        else read_data<=text_ram[memory_address];
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            motor_frame_open<=0;robot_frame_open<=0;
            round_robin<=0;latest_motor_bank<=0;latest_robot_bank<=0;
            held_motor_bank<=0;held_robot_bank<=0;write_motor_bank<=1;write_robot_bank<=1;
            motor_expected_index<=0;robot_expected_index<=0;
            motor_seen<=0;robot_seen<=0;read_valid<=0;
        end else begin
            read_valid<=read_request;
            if(report_begin)begin
                held_motor_bank<=latest_motor_bank;held_robot_bank<=latest_robot_bank;
            end
            if(grant_motor)round_robin<=1;
            else if(grant_robot)round_robin<=0;
            if(grant_motor)begin
                if(motor_byte_index==8'd0)begin
                    write_motor_bank<=motor_input_bank;
                    motor_expected_index<=8'd1;motor_frame_open<=1;
                end else if(!motor_frame_open || motor_byte_index!=motor_expected_index)begin
                    motor_frame_open<=0;motor_expected_index<=0;
                end else if(motor_byte_index==8'd255)begin
                    latest_motor_bank<=write_motor_bank;motor_seen<=1;
                    motor_frame_open<=0;motor_expected_index<=0;
                end else motor_expected_index<=motor_expected_index+1'b1;
            end
            if(grant_robot)begin
                if(robot_byte_index==8'd0)begin
                    write_robot_bank<=robot_input_bank;
                    robot_expected_index<=8'd1;robot_frame_open<=1;
                end else if(!robot_frame_open || robot_byte_index!=robot_expected_index)begin
                    robot_frame_open<=0;robot_expected_index<=0;
                end else if(robot_byte_index==8'd255)begin
                    latest_robot_bank<=write_robot_bank;robot_seen<=1;
                    robot_frame_open<=0;robot_expected_index<=0;
                end else robot_expected_index<=robot_expected_index+1'b1;
            end
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
    localparam BIT_COUNTER_WIDTH=BIT_CLOCKS<=1 ? 1 : $clog2(BIT_CLOCKS);
    reg [BIT_COUNTER_WIDTH-1:0] bit_clock_count;
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
