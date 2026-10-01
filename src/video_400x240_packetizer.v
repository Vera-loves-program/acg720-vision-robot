// OV5640 800x480 RGB565 -> one 802-byte UDP payload per selected row.
// Payload: big-endian uint16 row number, followed by 400 big-endian RGB565
// pixels. Take every other input row and column; row 0 starts a new frame.
// The chapter-59 16-to-8 asynchronous FIFO crosses camera PCLK to 125 MHz.
module video_400x240_packetizer (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        frame_start,
    input  wire        pixel_valid,
    input  wire [15:0] pixel,
    input  wire [10:0] x,
    input  wire [10:0] y,
    output reg  [15:0] word_out = 16'd0,
    output reg         word_valid = 1'b0,
    output reg         frame_sent = 1'b0
);
    reg row_seen = 1'b0;
    reg [7:0] last_row = 8'd0;
    reg [8:0] samples = 9'd0;
    reg hold_valid = 1'b0;
    reg [15:0] held_pixel = 16'd0;
    wire sample = pixel_valid && y >= 11'd1 && y <= 11'd480 &&
                  y[0] && x <= 11'd800 && !x[0];
    wire [7:0] row_index = (y - 11'd1) >> 1;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            row_seen <= 1'b0;
            last_row <= 0;
            samples <= 0;
            hold_valid <= 1'b0;
            held_pixel <= 0;
            word_out <= 0;
            word_valid <= 1'b0;
            frame_sent <= 1'b0;
        end else begin
            word_valid <= 1'b0;
            frame_sent <= 1'b0;
            if (frame_start) row_seen <= 1'b0;
            if (hold_valid) begin
                word_out <= held_pixel;
                word_valid <= 1'b1;
                hold_valid <= 1'b0;
            end else if (sample) begin
                if (!row_seen || row_index != last_row) begin
                    row_seen <= 1'b1;
                    last_row <= row_index;
                    samples <= 9'd1;
                    word_out <= {8'd0,row_index};
                    word_valid <= 1'b1;
                    held_pixel <= pixel;
                    hold_valid <= 1'b1;
                end else if (samples < 9'd400) begin
                    word_out <= pixel;
                    word_valid <= 1'b1;
                    samples <= samples + 1'b1;
                    if (row_index == 8'd239 && samples == 9'd399)
                        frame_sent <= 1'b1;
                end
            end
        end
    end
endmodule
