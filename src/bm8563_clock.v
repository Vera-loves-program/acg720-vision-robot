// Onboard BM8563T, 7-bit I2C address 0x51. Independent of the P7 IMU bus.
// Boot is READ ONLY: never overwrite battery-backed time or clear VL on boot.
// Poll 00..08 in one burst, then validate status, BCD and calendar boundaries.
// Explicit Bluetooth setting: STOP, write 02..08, START, then read actual time.
module bm8563_clock #(
    parameter integer CLK_HZ = 50000000
)(
    input wire clk,
    input wire reset_n,
    inout wire scl,
    inout wire sda,
    input wire set_valid,
    input wire [55:0] set_calendar,
    output wire set_busy,
    output reg valid,
    output reg [23:0] time_bcd,
    output reg [7:0] error_code
);
    localparam READ_START = 0, READ_WAIT = 1, DELAY = 2,
        WRITE_START = 3, WRITE_WAIT = 4, CLEAN_START = 5, CLEAN_WAIT = 6;
    reg [2:0] state;
    reg [31:0] poll_count;
    reg pending, writing, verifying, write_failed;
    reg [55:0] setting;
    reg [3:0] write_index;
    reg master_start, master_read;
    reg [7:0] master_register, master_write;
    wire master_busy, master_done;
    wire [7:0] master_error;
    wire [95:0] master_data;
    assign set_busy = pending || writing || verifying;
    // Reserved bits and flags are removed for calendar checking only. VL and
    // STOP/TEST flags are checked separately before any valid time is published.
    wire [55:0] read_calendar = {master_data[71:64],
        (master_data[63:56] & 8'h1f), (master_data[55:48] & 8'h07),
        (master_data[47:40] & 8'h3f), (master_data[39:32] & 8'h3f),
        (master_data[31:24] & 8'h7f), (master_data[23:16] & 8'h7f)};
    wire calendar_ok, setting_ok;
    rtc_calendar_valid u_read_check (.calendar(read_calendar), .valid(calendar_ok));
    rtc_calendar_valid u_set_check (.calendar(set_calendar), .valid(setting_ok));
    sensor_i2c_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(100000)) u_rtc_i2c (
        .clk(clk), .reset_n(reset_n), .start(master_start), .address(7'h51),
        .register_address(master_register), .read_not_write(master_read),
        .write_data(master_write), .read_count(4'd9), .scl(scl), .sda(sda),
        .busy(master_busy), .done(master_done), .error(master_error),
        .read_data(master_data)
    );
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= DELAY;
            poll_count <= CLK_HZ / 5 - CLK_HZ / 100; // first read after 10 ms
            pending <= 0;
            writing <= 0;
            verifying <= 0;
            write_failed <= 0;
            setting <= 0;
            write_index <= 0;
            master_start <= 0;
            master_read <= 1;
            master_register <= 0;
            master_write <= 0;
            valid <= 0;
            time_bcd <= 0;
            error_code <= 8'h01;
        end else begin
            master_start <= 0;
            if (set_valid && !set_busy && setting_ok) begin
                setting <= set_calendar;
                pending <= 1;
                valid <= 0;
            end
            case (state)
                DELAY: begin
                    if (pending) begin
                        pending <= 0;
                        writing <= 1;
                        write_index <= 0;
                        valid <= 0;
                        state <= WRITE_START;
                    end else if (poll_count >= CLK_HZ / 5 - 1) begin
                        poll_count <= 0;
                        state <= READ_START;
                    end else poll_count <= poll_count + 1'b1;
                end
                READ_START: begin
                    if (!master_busy) begin
                        master_read <= 1;
                        master_register <= 0;
                        master_start <= 1;
                        state <= READ_WAIT;
                    end
                end
                READ_WAIT: begin
                    if (master_done) begin
                        verifying <= 0;
                        valid <= 0;
                        if (write_failed) begin
                            // A partially written calendar must not look valid.
                            // Preserve the write error until a full new setting.
                        end else if (master_error != 0) begin
                            error_code <= 8'h20 | master_error;
                        end else if (master_data[23]) begin
                            error_code <= 8'h02; // seconds.VL: time unreliable
                        end else if ((master_data[7:0] & 8'ha8) != 0) begin
                            error_code <= 8'h03; // STOP or TEST bits set
                        end else if (!calendar_ok) begin
                            error_code <= 8'h04;
                        end else begin
                            time_bcd <= read_calendar[23:0];
                            valid <= 1;
                            error_code <= 0;
                        end
                        poll_count <= 0;
                        state <= DELAY;
                    end
                end
                WRITE_START: begin
                    if (!master_busy) begin
                        master_read <= 0;
                        if (write_index == 0) begin
                            master_register <= 0;
                            master_write <= 8'h20; // stop before editing date
                        end else if (write_index == 8) begin
                            master_register <= 0;
                            master_write <= 0; // normal mode, resume clock
                        end else begin
                            master_register <= write_index + 1'b1; // 02..08
                            master_write <= setting >> ((write_index - 1'b1) * 8);
                        end
                        master_start <= 1;
                        state <= WRITE_WAIT;
                    end
                end
                WRITE_WAIT: begin
                    if (master_done) begin
                        if (master_error != 0) begin
                            error_code <= 8'h20 | master_error;
                            write_failed <= 1;
                            valid <= 0;
                            state <= CLEAN_START;
                        end else if (write_index == 8) begin
                            write_failed <= 0;
                            writing <= 0;
                            verifying <= 1;
                            state <= READ_START; // display actual readback
                        end else begin
                            write_index <= write_index + 1'b1;
                            state <= WRITE_START;
                        end
                    end
                end
                CLEAN_START: begin
                    // One bounded attempt to release STOP after a failed set.
                    // Failure still remains visible; this is not a successful set.
                    if (!master_busy) begin
                        master_read <= 0;
                        master_register <= 0;
                        master_write <= 0;
                        master_start <= 1;
                        state <= CLEAN_WAIT;
                    end
                end
                CLEAN_WAIT: begin
                    if (master_done) begin
                        writing <= 0;
                        verifying <= 0;
                        poll_count <= 0;
                        state <= DELAY;
                    end
                end
                default: begin
                    valid <= 0;
                    error_code <= 8'h04;
                    poll_count <= 0;
                    state <= DELAY;
                end
            endcase
        end
    end
endmodule
