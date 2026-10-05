// ACG720: two cascaded 74HC595, {active-low DP/g/f/e/d/c/b/a, active-high digit}.
// Send bit 15 first, as in the board vendor's chapter 13/34 examples.
// Digit 0 is the rightmost nibble. F displays a dash; E displays E.
// One digit per 1 ms (125 Hz complete frames); 1 MHz shift clock at 50 MHz.
module seg7_clock_display #(
    parameter integer CLK_HZ = 50000000
)(
    input wire clk,
    input wire reset_n,
    input wire [31:0] digits,
    output reg dio,
    output reg sclk,
    output reg rclk
);
    localparam integer SLOT_CYCLES = CLK_HZ / 1000;
    localparam integer HALF_SHIFT = CLK_HZ / 2000000;
    localparam IDLE = 0, SHIFT_LOW = 1, SHIFT_HIGH = 2,
        LATCH_LOW = 3, LATCH_HIGH = 4;
    reg [2:0] state, digit_index;
    reg [31:0] slot_count, half_count, frame_digits;
    reg [15:0] shift_word, active_word;
    reg [3:0] bit_index;
    reg sending_blank;
    wire [31:0] selected_frame = (digit_index == 0) ? digits : frame_digits;
    wire [3:0] selected_digit = selected_frame >> (digit_index * 4);
    function [7:0] font;
        input [3:0] value;
        begin
            case (value)
                0: font = 8'hc0; 1: font = 8'hf9;
                2: font = 8'ha4; 3: font = 8'hb0;
                4: font = 8'h99; 5: font = 8'h92;
                6: font = 8'h82; 7: font = 8'hf8;
                8: font = 8'h80; 9: font = 8'h90;
                10: font = 8'h88; 11: font = 8'h83;
                12: font = 8'hc6; 13: font = 8'ha1;
                14: font = 8'h86; default: font = 8'hbf;
            endcase
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            digit_index <= 0;
            slot_count <= 0;
            half_count <= 0;
            frame_digits <= 32'hffffffff;
            shift_word <= 16'hff00;
            active_word <= 16'hff00;
            bit_index <= 15;
            sending_blank <= 0;
            dio <= 1;
            sclk <= 0;
            rclk <= 0;
        end else begin
            if (slot_count == SLOT_CYCLES - 1) slot_count <= 0;
            else slot_count <= slot_count + 1'b1;
            if (state == IDLE) begin
                half_count <= 0;
                if (slot_count == 0) begin
                    if (digit_index == 0) frame_digits <= digits;
                    active_word <= {font(selected_digit), (8'b00000001 << digit_index)};
                    // Turn all digits off before changing segment/digit data.
                    shift_word <= 16'hff00;
                    dio <= 1;
                    bit_index <= 15;
                    sending_blank <= 1;
                    state <= SHIFT_LOW;
                end
            end else if (half_count == HALF_SHIFT - 1) begin
                half_count <= 0;
                case (state)
                    SHIFT_LOW: begin sclk <= 1; state <= SHIFT_HIGH; end
                    SHIFT_HIGH: begin
                        sclk <= 0;
                        if (bit_index == 0) state <= LATCH_LOW;
                        else begin
                            shift_word <= {shift_word[14:0], 1'b0};
                            dio <= shift_word[14];
                            bit_index <= bit_index - 1'b1;
                            state <= SHIFT_LOW;
                        end
                    end
                    LATCH_LOW: begin rclk <= 1; state <= LATCH_HIGH; end
                    LATCH_HIGH: begin
                        rclk <= 0;
                        if (sending_blank) begin
                            shift_word <= active_word;
                            dio <= active_word[15];
                            bit_index <= 15;
                            sending_blank <= 0;
                            state <= SHIFT_LOW;
                        end else begin
                            digit_index <= digit_index + 1'b1;
                            state <= IDLE;
                        end
                    end
                    default: state <= IDLE;
                endcase
            end else half_count <= half_count + 1'b1;
        end
    end
endmodule
