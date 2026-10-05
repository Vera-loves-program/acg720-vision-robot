// ASCII TYYMMDDwHHMMSS; (13 digits, w=0 Sunday .. 6 Saturday).
// For example T2610062123456; means 2026-10-06 Tuesday 12:34:56.
// Bytes come from the existing HC-06 receiver after stop-bit verification.
module uart_rtc_set_parser #(
    parameter integer CLK_HZ = 50000000
)(
    input wire clk,
    input wire reset_n,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire set_busy,
    output reg set_valid,
    output reg [55:0] set_calendar
);
    reg collecting;
    reg [3:0] digit_count;
    reg [51:0] digits;
    reg [31:0] gap_count;
    wire [55:0] candidate = {digits[51:44], digits[43:36],
        4'b0000, digits[27:24], digits[35:28], digits[23:16],
        digits[15:8], digits[7:0]};
    wire calendar_ok;
    rtc_calendar_valid u_calendar_check (.calendar(candidate), .valid(calendar_ok));
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            collecting <= 0;
            digit_count <= 0;
            digits <= 0;
            gap_count <= 0;
            set_valid <= 0;
            set_calendar <= 0;
        end else begin
            set_valid <= 0;
            if (collecting) begin
                if (gap_count >= CLK_HZ / 2 - 1) begin
                    collecting <= 0;
                    gap_count <= 0;
                end else gap_count <= gap_count + 1'b1;
            end
            if (byte_valid) begin
                gap_count <= 0;
                if (byte_data == 8'h54) begin // T starts/restarts a frame.
                    collecting <= 1;
                    digit_count <= 0;
                    digits <= 0;
                end else if (collecting) begin
                    if (byte_data >= 8'h30 && byte_data <= 8'h39 && digit_count < 13) begin
                        digits <= {digits[47:0], byte_data[3:0]};
                        digit_count <= digit_count + 1'b1;
                    end else begin
                        collecting <= 0;
                        if (byte_data == 8'h3b && digit_count == 13 &&
                            calendar_ok && !set_busy) begin
                            set_calendar <= candidate;
                            set_valid <= 1;
                        end
                    end
                end
            end
        end
    end
endmodule
