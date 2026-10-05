// BM8563 registers 02..08, first byte at bits 7:0. Years are 2000..2099.
// Callers must remove RTC flag/reserved bits before passing the calendar.
module rtc_calendar_valid (
    input wire [55:0] calendar,
    output wire valid
);
    wire [7:0] sec = calendar[7:0];
    wire [7:0] minute = calendar[15:8];
    wire [7:0] hour = calendar[23:16];
    wire [7:0] day = calendar[31:24];
    wire [7:0] weekday = calendar[39:32];
    wire [7:0] month = calendar[47:40];
    wire [7:0] year = calendar[55:48];
    function bcd_ok;
        input [7:0] value;
        begin bcd_ok = (value[7:4] <= 9 && value[3:0] <= 9); end
    endfunction
    // 10*tens+ones fits seven bits for all valid two-digit BCD values.
    wire [6:0] year_binary = {3'b000, year[7:4]} * 7'd10 +
                            {3'b000, year[3:0]};
    wire leap_year = (year_binary[1:0] == 0);
    reg [7:0] last_day;
    always @* begin
        case (month)
            8'h02: last_day = leap_year ? 8'h29 : 8'h28;
            8'h04, 8'h06, 8'h09, 8'h11: last_day = 8'h30;
            default: last_day = 8'h31;
        endcase
    end
    assign valid = bcd_ok(sec) && sec <= 8'h59 &&
        bcd_ok(minute) && minute <= 8'h59 &&
        bcd_ok(hour) && hour <= 8'h23 &&
        bcd_ok(day) && day >= 8'h01 && day <= last_day &&
        weekday <= 8'd6 && bcd_ok(month) &&
        month >= 8'h01 && month <= 8'h12 && bcd_ok(year);
endmodule
