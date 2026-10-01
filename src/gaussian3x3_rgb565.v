// Causal 3x3 Gaussian: 1 2 1 / 2 4 2 / 1 2 1, normalized by 16.
// DVP coordinates are ONE-based. The first two rows/columns pass through.
// Pipeline: synchronous line read -> horizontal sums -> vertical sum/output.
// Pixels, coordinates, valid and frame_start have the same three-stage delay.
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
    output reg         out_frame_start = 1'b0,
    output reg         active_filter = 1'b1
);
    // Do not reset the RAM arrays. Two complete input rows populate them
    // before filtering starts, allowing synchronous block-RAM inference.
    reg [15:0] row_1 [0:1023];
    reg [15:0] row_2 [0:1023];
    reg [15:0] p1_q, p2_q;
    reg s0_valid = 0, s1_valid = 0;
    reg s0_filter = 1, s1_filter = 0;
    reg [15:0] raw_s0 = 0, raw_s1 = 0;
    reg [10:0] x_s0 = 0, y_s0 = 0, x_s1 = 0, y_s1 = 0;
    reg frame_s0 = 0, frame_s1 = 0;
    reg [15:0] now_1 = 0, now_2 = 0;
    reg [15:0] old1_1 = 0, old1_2 = 0;
    reg [15:0] old2_1 = 0, old2_2 = 0;
    reg [6:0] top_r = 0, mid_r = 0, bottom_r = 0;
    reg [6:0] top_b = 0, mid_b = 0, bottom_b = 0;
    reg [7:0] top_g = 0, mid_g = 0, bottom_g = 0;

    // Read previous rows first; write on the following edge at the delayed
    // address. An advancing DVP stream avoids same-address read/write.
    always @(posedge clk) begin
        if (reset_n && in_valid && in_x <= 11'd800) begin
            p1_q <= row_1[in_x[9:0]];
            p2_q <= row_2[in_x[9:0]];
        end
        if (reset_n && s0_valid && x_s0 <= 11'd800) begin
            row_1[x_s0[9:0]] <= raw_s0;
            row_2[x_s0[9:0]] <= p1_q;
        end
    end

    function [6:0] horizontal5;
        input [4:0] left, middle, right;
        begin
            horizontal5 = ({2'b0,left} + {2'b0,right}) +
                          {1'b0,middle,1'b0};
        end
    endfunction
    function [7:0] horizontal6;
        input [5:0] left, middle, right;
        begin
            horizontal6 = ({2'b0,left} + {2'b0,right}) +
                          {1'b0,middle,1'b0};
        end
    endfunction
    function [4:0] vertical5;
        input [6:0] top, middle, bottom;
        reg [8:0] sum;
        begin
            sum = ({2'b0,top} + {2'b0,bottom}) + {1'b0,middle,1'b0};
            vertical5 = sum[8:4];
        end
    endfunction
    function [5:0] vertical6;
        input [7:0] top, middle, bottom;
        reg [9:0] sum;
        begin
            sum = ({2'b0,top} + {2'b0,bottom}) + {1'b0,middle,1'b0};
            vertical6 = sum[9:4];
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            s0_valid <= 0; s1_valid <= 0; out_valid <= 0;
            frame_s0 <= 0; frame_s1 <= 0; out_frame_start <= 0;
            s0_filter <= 1; s1_filter <= 0; active_filter <= 1;
            raw_s0 <= 0; raw_s1 <= 0; out_pixel <= 0;
            x_s0 <= 0; y_s0 <= 0; x_s1 <= 0; y_s1 <= 0;
            out_x <= 0; out_y <= 0;
            now_1 <= 0; now_2 <= 0;
            old1_1 <= 0; old1_2 <= 0; old2_1 <= 0; old2_2 <= 0;
            top_r <= 0; mid_r <= 0; bottom_r <= 0;
            top_g <= 0; mid_g <= 0; bottom_g <= 0;
            top_b <= 0; mid_b <= 0; bottom_b <= 0;
        end else begin
            if (frame_start) active_filter <= enable_filter;
            s0_valid <= in_valid;
            s1_valid <= s0_valid;
            out_valid <= s1_valid;
            frame_s0 <= frame_start;
            frame_s1 <= frame_s0;
            out_frame_start <= frame_s1;

            if (in_valid) begin
                raw_s0 <= in_pixel;
                x_s0 <= in_x; y_s0 <= in_y;
                s0_filter <= frame_start ? enable_filter : active_filter;
            end
            if (s0_valid) begin
                raw_s1 <= raw_s0;
                x_s1 <= x_s0; y_s1 <= y_s0;
                s1_filter <= s0_filter && x_s0 >= 11'd3 &&
                             x_s0 <= 11'd800 && y_s0 >= 11'd3;
                top_r <= horizontal5(old2_2[15:11], old2_1[15:11], p2_q[15:11]);
                mid_r <= horizontal5(old1_2[15:11], old1_1[15:11], p1_q[15:11]);
                bottom_r <= horizontal5(now_2[15:11], now_1[15:11], raw_s0[15:11]);
                top_g <= horizontal6(old2_2[10:5], old2_1[10:5], p2_q[10:5]);
                mid_g <= horizontal6(old1_2[10:5], old1_1[10:5], p1_q[10:5]);
                bottom_g <= horizontal6(now_2[10:5], now_1[10:5], raw_s0[10:5]);
                top_b <= horizontal5(old2_2[4:0], old2_1[4:0], p2_q[4:0]);
                mid_b <= horizontal5(old1_2[4:0], old1_1[4:0], p1_q[4:0]);
                bottom_b <= horizontal5(now_2[4:0], now_1[4:0], raw_s0[4:0]);
                // At x=1 discard the preceding line's horizontal history.
                now_2 <= x_s0 == 11'd1 ? 16'd0 : now_1;
                now_1 <= raw_s0;
                old1_2 <= x_s0 == 11'd1 ? 16'd0 : old1_1;
                old1_1 <= p1_q;
                old2_2 <= x_s0 == 11'd1 ? 16'd0 : old2_1;
                old2_1 <= p2_q;
            end
            if (s1_valid) begin
                out_x <= x_s1; out_y <= y_s1;
                out_pixel <= s1_filter ?
                    {vertical5(top_r,mid_r,bottom_r),
                     vertical6(top_g,mid_g,bottom_g),
                     vertical5(top_b,mid_b,bottom_b)} : raw_s1;
            end
        end
    end
endmodule
