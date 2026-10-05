// Sensor demonstration telemetry: 50 MHz, 115200 baud, 8 data bits, no parity,
// one stop bit. A complete snapshot is taken at the selected report interval.
// The default interval is 100 ms. UART idles high.
//
// Numeric CSV only, followed by CR LF (VOFA+ FireWater compatible):
// seq,imu_valid,imu_who,imu_error,imu_samples,gx,gy,gz,ax,ay,az,
// us_valid,us_error,us_samples,distance_mm,cmd_count,last_cmd_ascii,rx_errors
// The six axes are signed raw sensor counts; scaling belongs to the IMU setup.
// Invalid IMU axes are 0. Invalid ultrasonic distance is -1. Consult the valid
// flags and sample counters before interpreting a value as a new measurement.
// This module transmits telemetry only; it does not accept motor commands.
module sensor_demo_uart #(
    parameter CLK_HZ = 50000000,
    parameter UART_BAUD = 115200,
    parameter REPORT_HZ = 10
)(
    input wire clk,
    input wire reset_n,
    input wire imu_valid,
    input wire [7:0] imu_who,
    input wire [7:0] imu_error,
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
    output wire tx
);
    localparam DEFAULT_REPORT_CLOCKS = CLK_HZ / REPORT_HZ;
    wire [31:0] report_clocks = (period_cycles == 0) ?
        DEFAULT_REPORT_CLOCKS : {8'd0, period_cycles};
    localparam IDLE = 0, LOAD_FIELD = 1, CONVERT = 2, PREPARE_DIGITS = 3,
        SEND_DIGIT = 4, AFTER_DIGIT = 5, SEND_SEPARATOR = 6,
        NEXT_FIELD = 7, SEND_LF = 8, WAIT_ACCEPT = 9, WAIT_DONE = 10;

    reg [3:0] state, after_byte;
    reg [31:0] report_counter, sequence_counter;
    reg [31:0] snapshot [0:17];
    reg [17:0] negative;
    reg [4:0] field_index;
    reg [31:0] binary_shift;
    reg [39:0] decimal_digits;
    reg [5:0] conversion_step;
    reg [3:0] digit_index;
    reg digit_started;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    integer reset_field;

    sensor_demo_uart_tx_8n1 #(
        .CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)
    ) u_tx (
        .clk(clk), .reset_n(reset_n), .start(tx_start), .data(tx_data),
        .busy(tx_busy), .tx(tx)
    );

    // Work in 16 bits before extending, including the -32768 corner case.
    function [31:0] axis_magnitude;
        input [15:0] raw;
        reg [15:0] magnitude;
        begin
            magnitude = raw[15] ? (~raw + 16'd1) : raw;
            axis_magnitude = {16'd0, magnitude};
        end
    endfunction

    // Iterative binary-to-BCD conversion: one bit per clock, 32 clocks per
    // field. No variable division and no floating-point logic are required.
    function [39:0] add_three;
        input [39:0] digits;
        integer digit;
        reg [39:0] adjusted;
        begin
            adjusted = digits;
            for (digit = 0; digit < 10; digit = digit + 1) begin
                if (digits[digit*4 +: 4] >= 4'd5)
                    adjusted[digit*4 +: 4] = digits[digit*4 +: 4] + 4'd3;
            end
            add_three = adjusted;
        end
    endfunction
    wire [39:0] adjusted_digits = add_three(decimal_digits);
    wire [3:0] current_digit = decimal_digits[digit_index*4 +: 4];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            after_byte <= IDLE;
            report_counter <= 0;
            sequence_counter <= 0;
            negative <= 0;
            field_index <= 0;
            binary_shift <= 0;
            decimal_digits <= 0;
            conversion_step <= 0;
            digit_index <= 0;
            digit_started <= 0;
            tx_start <= 0;
            tx_data <= 0;
            for (reset_field = 0; reset_field < 18; reset_field = reset_field + 1)
                snapshot[reset_field] <= 0;
        end else begin
            tx_start <= 0;
            if (report_counter >= report_clocks - 1)
                report_counter <= 0;
            else
                report_counter <= report_counter + 1'b1;

            case (state)
                IDLE: begin
                    // All data and validity flags belong to this same clock.
                    // If a non-default very low baud rate overruns a report
                    // interval, skip that interval rather than replace data
                    // partway through transmitting a line.
                    if (report_counter >= report_clocks - 1) begin
                        snapshot[0] <= sequence_counter;
                        sequence_counter <= sequence_counter + 1'b1;
                        snapshot[1] <= {31'd0, imu_valid};
                        snapshot[2] <= {24'd0, imu_who};
                        snapshot[3] <= {24'd0, imu_error};
                        snapshot[4] <= imu_samples;
                        snapshot[5] <= imu_valid ? axis_magnitude(gx) : 32'd0;
                        snapshot[6] <= imu_valid ? axis_magnitude(gy) : 32'd0;
                        snapshot[7] <= imu_valid ? axis_magnitude(gz) : 32'd0;
                        snapshot[8] <= imu_valid ? axis_magnitude(ax) : 32'd0;
                        snapshot[9] <= imu_valid ? axis_magnitude(ay) : 32'd0;
                        snapshot[10] <= imu_valid ? axis_magnitude(az) : 32'd0;
                        snapshot[11] <= {31'd0, us_valid};
                        snapshot[12] <= {24'd0, us_error};
                        snapshot[13] <= us_samples;
                        snapshot[14] <= us_valid ? {16'd0, distance_mm} : 32'd1;
                        snapshot[15] <= command_count;
                        snapshot[16] <= {24'd0, last_command};
                        snapshot[17] <= rx_errors;
                        negative <= {
                            3'd0, !us_valid, 3'd0,
                            imu_valid && az[15], imu_valid && ay[15],
                            imu_valid && ax[15], imu_valid && gz[15],
                            imu_valid && gy[15], imu_valid && gx[15], 5'd0
                        };
                        field_index <= 0;
                        state <= LOAD_FIELD;
                    end
                end
                LOAD_FIELD: begin
                    binary_shift <= snapshot[field_index];
                    decimal_digits <= 0;
                    conversion_step <= 0;
                    state <= CONVERT;
                end
                CONVERT: begin
                    decimal_digits <= {adjusted_digits[38:0], binary_shift[31]};
                    binary_shift <= {binary_shift[30:0], 1'b0};
                    if (conversion_step == 31)
                        state <= PREPARE_DIGITS;
                    else
                        conversion_step <= conversion_step + 1'b1;
                end
                PREPARE_DIGITS: begin
                    digit_index <= 9;
                    digit_started <= 0;
                    if (negative[field_index]) begin
                        tx_data <= 8'h2d; // '-'
                        tx_start <= 1;
                        after_byte <= SEND_DIGIT;
                        state <= WAIT_ACCEPT;
                    end else begin
                        state <= SEND_DIGIT;
                    end
                end
                SEND_DIGIT: begin
                    if (!digit_started && digit_index != 0 && current_digit == 0)
                        digit_index <= digit_index - 1'b1;
                    else begin
                        tx_data <= 8'h30 + {4'd0, current_digit};
                        tx_start <= 1;
                        digit_started <= 1;
                        after_byte <= AFTER_DIGIT;
                        state <= WAIT_ACCEPT;
                    end
                end
                AFTER_DIGIT: begin
                    if (digit_index == 0)
                        state <= SEND_SEPARATOR;
                    else begin
                        digit_index <= digit_index - 1'b1;
                        state <= SEND_DIGIT;
                    end
                end
                SEND_SEPARATOR: begin
                    if (field_index == 17) begin
                        tx_data <= 8'h0d; // CR
                        after_byte <= SEND_LF;
                    end else begin
                        tx_data <= 8'h2c; // ','
                        after_byte <= NEXT_FIELD;
                    end
                    tx_start <= 1;
                    state <= WAIT_ACCEPT;
                end
                NEXT_FIELD: begin
                    field_index <= field_index + 1'b1;
                    state <= LOAD_FIELD;
                end
                SEND_LF: begin
                    tx_data <= 8'h0a;
                    tx_start <= 1;
                    after_byte <= IDLE;
                    state <= WAIT_ACCEPT;
                end
                // Observe busy rising before waiting for it to fall. This
                // avoids treating the pre-launch idle cycle as completion.
                WAIT_ACCEPT: if (tx_busy) state <= WAIT_DONE;
                WAIT_DONE: if (!tx_busy) state <= after_byte;
                default: state <= IDLE;
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
