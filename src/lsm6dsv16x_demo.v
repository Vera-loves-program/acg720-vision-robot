// LSM6DSV16X six-axis demo, based on the actual local ST register driver:
// D:/M0/mspm0-modules-main/Examples/lsm6dsv16x-oled-hardware-spi/
// Drivers/LSM6DSV16X/lsm6dsv16x_reg.h and lsm6dsv16x_reg.c.
// The IMU uses I2C; the name of that example refers to the OLED's SPI.
// No Euler angles, fusion, motor control, or calibration are implemented.
// Outputs are signed raw counts: gyro * 0.07 = deg/s;
// accelerometer * 0.122 = mg (range +/-4g, including gravity).
// valid means a COMPLETE sample was received within the last 200 ms.
// The previous counts remain visible after failure but MUST NOT be used when
// valid=0. sample_count changes only for successful complete 12-byte reads.
// error_code: 01 boot/init, 10 wrong WHO_AM_I, 11 reset did not finish,
// 12 config readback mismatch, 14 no fresh sample for 200 ms,
// 21 slave NACK, 22 I2C timeout, 23 stuck bus, 24 bad I2C request.
module lsm6dsv16x_demo #(
    parameter integer CLK_HZ = 50000000,
    parameter [6:0] I2C_ADDRESS = 7'h6a,
    parameter integer AUTO_ADDRESS = 1
)(
    input wire clk,
    input wire reset_n,
    inout wire scl,
    inout wire sda,
    output reg valid,
    output reg [7:0] who_am_i,
    output reg [7:0] error_code,
    output reg [31:0] sample_count,
    output reg signed [15:0] gx,
    output reg signed [15:0] gy,
    output reg signed [15:0] gz,
    output reg signed [15:0] ax,
    output reg signed [15:0] ay,
    output reg signed [15:0] az
);
    localparam integer BOOT_CYCLES = CLK_HZ / 100; // 10 ms
    localparam integer RESET_POLL_CYCLES = CLK_HZ / 1000; // 1 ms
    localparam integer POLL_CYCLES = CLK_HZ / 500; // 2 ms
    localparam integer STALE_CYCLES = CLK_HZ / 5; // 200 ms
    localparam integer RETRY_CYCLES = CLK_HZ; // 1 s after failure
    localparam [4:0] BOOT_WAIT = 0, WHO_START = 1, WHO_FINISH = 2,
        RESET_START = 3, RESET_SENT = 4, RESET_WAIT = 5,
        RESET_POLL_START = 6, RESET_POLL_FINISH = 7,
        CONFIG_WRITE = 8, CONFIG_SENT = 9, CONFIG_READ_FINISH = 10,
        SAMPLE_WAIT = 11, STATUS_START = 12, STATUS_FINISH = 13,
        DATA_START = 14, DATA_FINISH = 15, WAIT_I2C = 16,
        RETRY_WAIT = 17, RESET_BOOT_WAIT = 18;
    reg [4:0] state, after_transaction;
    reg [31:0] delay_count, sample_age;
    reg [5:0] reset_polls;
    reg [2:0] config_index;
    reg streaming, tried_other_address;
    reg i2c_start, i2c_read;
    reg [6:0] i2c_address;
    reg [7:0] i2c_register, i2c_write;
    reg [3:0] i2c_count;
    wire i2c_busy, i2c_done;
    wire [7:0] i2c_error;
    wire [95:0] i2c_data;

    // These encodings are specific to LSM6DSV16X. LSM6DS3/MPU/BNO drivers
    // have different addresses and must not be substituted here.
    function [7:0] config_register;
        input [2:0] index;
        begin
            case (index)
                0: config_register = 8'h12; // CTRL3: BDU/IF_INC
                1: config_register = 8'h15; // CTRL6: gyro full scale
                2: config_register = 8'h17; // CTRL8: accel full scale
                3: config_register = 8'h10; // CTRL1: accel ODR/mode
                default: config_register = 8'h11; // CTRL2: gyro ODR/mode
            endcase
        end
    endfunction
    function [7:0] config_value;
        input [2:0] index;
        begin
            case (index)
                0: config_value = 8'h44; // BDU=1, IF_INC=1
                1: config_value = 8'h04; // +/-2000 dps
                2: config_value = 8'h01; // +/-4 g
                default: config_value = 8'h05; // high performance, 60 Hz
            endcase
        end
    endfunction
    function [7:0] config_mask;
        input [2:0] index;
        begin
            case (index)
                0: config_mask = 8'h45;
                1: config_mask = 8'h0f;
                2: config_mask = 8'h03;
                default: config_mask = 8'h7f;
            endcase
        end
    endfunction

    task request_read;
        input [7:0] register_number;
        input [3:0] number_of_bytes;
        input [4:0] return_state;
        begin
            i2c_register <= register_number;
            i2c_count <= number_of_bytes;
            i2c_read <= 1;
            i2c_start <= 1;
            after_transaction <= return_state;
            state <= WAIT_I2C;
        end
    endtask
    task request_write;
        input [7:0] register_number;
        input [7:0] value;
        input [4:0] return_state;
        begin
            i2c_register <= register_number;
            i2c_write <= value;
            i2c_read <= 0;
            i2c_start <= 1;
            after_transaction <= return_state;
            state <= WAIT_I2C;
        end
    endtask

    sensor_i2c_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(100000)) bus_master (
        .clk(clk), .reset_n(reset_n), .start(i2c_start),
        .address(i2c_address), .register_address(i2c_register),
        .read_not_write(i2c_read), .write_data(i2c_write),
        .read_count(i2c_count), .scl(scl), .sda(sda),
        .busy(i2c_busy), .done(i2c_done), .error(i2c_error),
        .read_data(i2c_data)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= BOOT_WAIT;
            after_transaction <= WHO_FINISH;
            valid <= 0;
            who_am_i <= 0;
            error_code <= 8'h01;
            sample_count <= 0;
            gx <= 0; gy <= 0; gz <= 0;
            ax <= 0; ay <= 0; az <= 0;
            delay_count <= 0;
            sample_age <= 0;
            reset_polls <= 0;
            config_index <= 0;
            streaming <= 0;
            tried_other_address <= 0;
            i2c_start <= 0;
            i2c_read <= 0;
            i2c_address <= I2C_ADDRESS;
            i2c_register <= 0;
            i2c_write <= 0;
            i2c_count <= 1;
        end else begin
            i2c_start <= 0;
            if (streaming && sample_age < STALE_CYCLES)
                sample_age <= sample_age + 1'b1;
            if (streaming && sample_age >= STALE_CYCLES - 1)
                valid <= 0;

            // This also catches a responsive device with no new data-ready
            // flags; old counts never remain marked valid indefinitely.
            // Finish an in-flight transfer first so its error/completion is
            // consumed. DATA_FINISH may publish a fresh complete sample.
            if (streaming && sample_age >= STALE_CYCLES - 1 && !i2c_busy &&
                state != WAIT_I2C && state != DATA_FINISH) begin
                valid <= 0;
                streaming <= 0;
                error_code <= 8'h14;
                delay_count <= 0;
                state <= RETRY_WAIT;
            end else begin
                case (state)
                    BOOT_WAIT: begin
                        if (delay_count >= BOOT_CYCLES - 1) begin
                            delay_count <= 0;
                            state <= WHO_START;
                        end else delay_count <= delay_count + 1'b1;
                    end
                    WHO_START: request_read(8'h0f, 1, WHO_FINISH);
                    WHO_FINISH: begin
                        who_am_i <= i2c_data[7:0];
                        if (i2c_data[7:0] == 8'h70) begin
                            state <= RESET_START;
                        end else if (AUTO_ADDRESS && !tried_other_address) begin
                            tried_other_address <= 1;
                            i2c_address <= i2c_address ^ 7'h01;
                            state <= WHO_START;
                        end else begin
                            valid <= 0;
                            streaming <= 0;
                            error_code <= 8'h10;
                            delay_count <= 0;
                            state <= RETRY_WAIT;
                        end
                    end
                    // Match GLOBAL_RST in the working local C example:
                    // FUNC_CFG_ACCESS (01h), SW_POR bit 2, not CTRL3 bit 0.
                    RESET_START: request_write(8'h01, 8'h04, RESET_SENT);
                    RESET_SENT: begin
                        reset_polls <= 0;
                        delay_count <= 0;
                        state <= RESET_BOOT_WAIT;
                    end
                    RESET_BOOT_WAIT: begin
                        if (delay_count >= BOOT_CYCLES - 1) begin
                            delay_count <= 0;
                            state <= RESET_POLL_START;
                        end else delay_count <= delay_count + 1'b1;
                    end
                    RESET_WAIT: begin
                        if (delay_count >= RESET_POLL_CYCLES - 1) begin
                            delay_count <= 0;
                            state <= RESET_POLL_START;
                        end else delay_count <= delay_count + 1'b1;
                    end
                    RESET_POLL_START: request_read(8'h01, 1, RESET_POLL_FINISH);
                    RESET_POLL_FINISH: begin
                        if (!i2c_data[2]) begin
                            config_index <= 0;
                            state <= CONFIG_WRITE;
                        end else if (reset_polls >= 49) begin
                            valid <= 0;
                            streaming <= 0;
                            error_code <= 8'h11;
                            delay_count <= 0;
                            state <= RETRY_WAIT;
                        end else begin
                            reset_polls <= reset_polls + 1'b1;
                            delay_count <= 0;
                            state <= RESET_WAIT;
                        end
                    end
                    CONFIG_WRITE: request_write(config_register(config_index),
                                                config_value(config_index), CONFIG_SENT);
                    CONFIG_SENT: request_read(config_register(config_index),
                                              1, CONFIG_READ_FINISH);
                    CONFIG_READ_FINISH: begin
                        if ((i2c_data[7:0] & config_mask(config_index)) !=
                            (config_value(config_index) & config_mask(config_index))) begin
                            valid <= 0;
                            streaming <= 0;
                            error_code <= 8'h12;
                            delay_count <= 0;
                            state <= RETRY_WAIT;
                        end else if (config_index == 4) begin
                            streaming <= 1;
                            sample_age <= 0;
                            delay_count <= 0;
                            state <= SAMPLE_WAIT;
                        end else begin
                            config_index <= config_index + 1'b1;
                            state <= CONFIG_WRITE;
                        end
                    end
                    SAMPLE_WAIT: begin
                        if (delay_count >= POLL_CYCLES - 1) begin
                            delay_count <= 0;
                            state <= STATUS_START;
                        end else delay_count <= delay_count + 1'b1;
                    end
                    STATUS_START: request_read(8'h1e, 1, STATUS_FINISH);
                    STATUS_FINISH: begin
                        if (i2c_data[1:0] == 2'b11)
                            state <= DATA_START;
                        else begin
                            delay_count <= 0;
                            state <= SAMPLE_WAIT;
                        end
                    end
                    DATA_START: request_read(8'h22, 12, DATA_FINISH);
                    DATA_FINISH: begin
                        gx <= i2c_data[15:0];
                        gy <= i2c_data[31:16];
                        gz <= i2c_data[47:32];
                        ax <= i2c_data[63:48];
                        ay <= i2c_data[79:64];
                        az <= i2c_data[95:80];
                        sample_count <= sample_count + 1'b1;
                        valid <= 1;
                        error_code <= 0;
                        sample_age <= 0;
                        delay_count <= 0;
                        state <= SAMPLE_WAIT;
                    end
                    WAIT_I2C: begin
                        if (i2c_done) begin
                            if (i2c_error == 0) begin
                                state <= after_transaction;
                            end else if (after_transaction == WHO_FINISH &&
                                         i2c_error == 8'h01 && AUTO_ADDRESS &&
                                         !tried_other_address) begin
                                tried_other_address <= 1;
                                i2c_address <= i2c_address ^ 7'h01;
                                state <= WHO_START;
                            end else begin
                                valid <= 0;
                                streaming <= 0;
                                error_code <= 8'h20 | i2c_error;
                                delay_count <= 0;
                                state <= RETRY_WAIT;
                            end
                        end
                    end
                    RETRY_WAIT: begin
                        if (delay_count >= RETRY_CYCLES - 1) begin
                            delay_count <= 0;
                            sample_age <= 0;
                            tried_other_address <= 0;
                            i2c_address <= I2C_ADDRESS;
                            state <= BOOT_WAIT;
                        end else delay_count <= delay_count + 1'b1;
                    end
                    default: begin
                        valid <= 0;
                        streaming <= 0;
                        error_code <= 8'h12;
                        delay_count <= 0;
                        state <= RETRY_WAIT;
                    end
                endcase
            end
        end
    end
endmodule
