// Causal 3x3 Gaussian, weights 1 2 1 / 2 4 2 / 1 2 1.
// One output for each input pixel, preserving the DDR frame geometry.
// The filter is centred one pixel up/left; top two rows and left two columns
// pass through unchanged. Two line buffers hold the previous RGB565 rows.
module gaussian3x3_rgb565 (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        frame_start,
    input  wire        enable_filter,
    input  wire        in_valid,
    input  wire [15:0] in_pixel,
    input  wire [10:0] in_x,
    input  wire [10:0] in_y,
    output reg         out_valid = 1'b0,
    output reg  [15:0] out_pixel = 16'd0,
    output reg  [10:0] out_x = 11'd0,
    output reg  [10:0] out_y = 11'd0,
    output reg         active_filter = 1'b1
);
    reg [15:0] row_1 [0:800];
    reg [15:0] row_2 [0:800];
    reg [15:0] now_1 = 0, now_2 = 0;
    reg [15:0] old1_1 = 0, old1_2 = 0;
    reg [15:0] old2_1 = 0, old2_2 = 0;

    wire [15:0] p1 = row_1[in_x];
    wire [15:0] p2 = row_2[in_x];

    function [4:0] mix5;
        input [4:0] a,b,c,d,e,f,g,h,i;
        reg [8:0] sum;
        begin
            sum = {4'b0,a} + ({4'b0,b} << 1) + {4'b0,c}
                + ({4'b0,d} << 1) + ({4'b0,e} << 2)
                + ({4'b0,f} << 1) + {4'b0,g}
                + ({4'b0,h} << 1) + {4'b0,i};
            mix5 = sum[8:4];
        end
    endfunction
    function [5:0] mix6;
        input [5:0] a,b,c,d,e,f,g,h,i;
        reg [9:0] sum;
        begin
            sum = {4'b0,a} + ({4'b0,b} << 1) + {4'b0,c}
                + ({4'b0,d} << 1) + ({4'b0,e} << 2)
                + ({4'b0,f} << 1) + {4'b0,g}
                + ({4'b0,h} << 1) + {4'b0,i};
            mix6 = sum[9:4];
        end
    endfunction

    wire [15:0] filtered = {
        mix5(old2_2[15:11], old2_1[15:11], p2[15:11],
             old1_2[15:11], old1_1[15:11], p1[15:11],
             now_2[15:11],  now_1[15:11],  in_pixel[15:11]),
        mix6(old2_2[10:5], old2_1[10:5], p2[10:5],
             old1_2[10:5], old1_1[10:5], p1[10:5],
             now_2[10:5],  now_1[10:5],  in_pixel[10:5]),
        mix5(old2_2[4:0], old2_1[4:0], p2[4:0],
             old1_2[4:0], old1_1[4:0], p1[4:0],
             now_2[4:0],  now_1[4:0],  in_pixel[4:0])
    };

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            out_valid <= 1'b0;
            out_pixel <= 16'd0;
            out_x <= 11'd0;
            out_y <= 11'd0;
            active_filter <= 1'b1;
            now_1 <= 0; now_2 <= 0;
            old1_1 <= 0; old1_2 <= 0;
            old2_1 <= 0; old2_2 <= 0;
        end else begin
            if (frame_start) active_filter <= enable_filter;
            out_valid <= in_valid;
            if (in_valid) begin
                out_x <= in_x;
                out_y <= in_y;
                out_pixel <= (active_filter && in_x >= 11'd2 &&
                              in_x <= 11'd800 && in_y >= 11'd3)
                             ? filtered : in_pixel;
                if (in_x <= 11'd800) begin
                    row_1[in_x] <= in_pixel;
                    row_2[in_x] <= p1;
                    now_2 <= now_1; now_1 <= in_pixel;
                    old1_2 <= old1_1; old1_1 <= p1;
                    old2_2 <= old2_1; old2_1 <= p2;
                end
            end
        end
    end
endmodule
