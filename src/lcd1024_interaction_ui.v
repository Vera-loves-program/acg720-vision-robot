// 1024x600 RGB565 dashboard around the existing 800x480 live camera image.
// Text is real FPGA-rendered 5x7 bitmap type, scaled 2x. No image ROM.
module lcd1024_interaction_ui (
    input  wire        clk,
    input  wire        run,
    input  wire [15:0] camera_pixel,
    input  wire        camera_pixel_valid,
    input  wire        camera_ready,
    input  wire        ddr_ready,
    input  wire        ddr_pll_ready,
    input  wire        net_ready,
    input  wire        filter_active,
    input  wire        debug_mode,
    input  wire        stop_latched,
    input  wire        touch_ready,
    input  wire [3:0]  touch_error,
    input  wire        selection_valid,
    input  wire [9:0]  selection_x, selection_y,
    input  wire [1:0]  zoom_level,
    output wire        camera_pop,
    output wire        frame_restart,
    output reg  [15:0] rgb = 16'h0000,
    output reg         de = 1'b0,
    output reg         hs = 1'b1,
    output reg         vs = 1'b1,
    output reg         frame_heartbeat = 1'b0,
    output reg         camera_underflow = 1'b0
);
    localparam [10:0] H_SYNC=20, H_START=160, H_END=1184, H_TOTAL=1344;
    localparam [9:0] V_SYNC=3, V_START=23, V_END=623, V_TOTAL=635;
    // Portfolio palette: cream #F4F4F0, blue #0062AD, yellow #FFD15C,
    // orange #FF7F00, thick near-black borders and offset shadows.
    localparam [15:0] CREAM=16'hf7be, PANEL=16'hfe8b,
                      BLUE=16'h0315, ORANGE=16'hfbe0,
                      WHITE=16'hffff, MUTED=16'he73b,
                      GREEN=16'h4ded, RED=16'hfa69,
                      INK=16'h10a2, BLACK=16'h0000;
    reg [10:0] h = 0;
    reg [9:0] v = 0;
    reg [4:0] frame_div = 0;
    wire active = h>=H_START && h<H_END && v>=V_START && v<V_END;
    wire [10:0] ax = h-H_START;
    wire [9:0] ay = v-V_START;
    wire video_area = active && ax>=11'd16 && ax<11'd816 &&
                      ay>=10'd56 && ay<10'd536;
    assign frame_restart = run && h==H_TOTAL-1'b1 && v==V_TOTAL-1'b1;
    assign camera_pop = run && video_area && camera_pixel_valid && !camera_underflow;

    // Five columns, left to right from the most-significant byte.
    // Within each column bit 0 is the top row and bit 6 is the bottom row.
    function [39:0] glyph;
        input [7:0] c;
        begin
            case(c)
                "A":glyph=40'h7e1111117e; "B":glyph=40'h7f49494936;
                "C":glyph=40'h3e41414122; "D":glyph=40'h7f4141221c;
                "E":glyph=40'h7f49494941; "F":glyph=40'h7f09090901;
                "G":glyph=40'h3e4149497a; "H":glyph=40'h7f0808087f;
                "I":glyph=40'h00417f4100; "J":glyph=40'h2040413f01;
                "K":glyph=40'h7f08142241; "L":glyph=40'h7f40404040;
                "M":glyph=40'h7f020c027f; "N":glyph=40'h7f0408107f;
                "O":glyph=40'h3e4141413e; "P":glyph=40'h7f09090906;
                "Q":glyph=40'h3e4151215e; "R":glyph=40'h7f09192946;
                "S":glyph=40'h4649494931; "T":glyph=40'h01017f0101;
                "U":glyph=40'h3f4040403f; "V":glyph=40'h1f2040201f;
                "W":glyph=40'h7f2018207f; "X":glyph=40'h6314081463;
                "Y":glyph=40'h0304780403; "Z":glyph=40'h6151494543;
                "0":glyph=40'h3e4549513e; "1":glyph=40'h00427f4000;
                "2":glyph=40'h4261514946; "3":glyph=40'h2141454b31;
                "4":glyph=40'h1814127f10; "5":glyph=40'h2745454539;
                "6":glyph=40'h3c4a494930; "7":glyph=40'h0171090503;
                "8":glyph=40'h3649494936; "9":glyph=40'h064949291e;
                ":":glyph=40'h0036360000; "/":glyph=40'h2010080402;
                "-":glyph=40'h0808080808; ".":glyph=40'h0060600000;
                default:glyph=40'h0000000000;
            endcase
        end
    endfunction

    reg [127:0] label_text;
    reg [10:0] label_x;
    reg [9:0] label_y;
    reg [15:0] label_color;
    reg label_enable;
    always @* begin
        label_text = 128'd0;
        label_x = 0; label_y = 0;
        label_color = INK;
        label_enable = 1'b1;
        if (ay>=10'd14 && ay<10'd30) begin
            label_text="VERA VISION R3  "; label_x=11'd20; label_y=10'd14;
            label_color=WHITE;
        end else if (ay>=10'd72 && ay<10'd88) begin
            label_text={"VISION LAB","      "}; label_x=11'd838; label_y=10'd72;
            label_color=BLUE;
        end else if (ay>=10'd94 && ay<10'd110) begin
            label_text=!camera_ready ? {"CAM WAIT","        "} : !ddr_ready ? {"DDR WAIT","        "} : {"CAM DDR OK","      "}; label_x=11'd838; label_y=10'd94;
            label_color=camera_ready && ddr_ready ? GREEN : RED;
        end else if (ay>=10'd122 && ay<10'd138) begin
            label_text=debug_mode ? {"DEBUG ON","        "} : {"VIS DEBUG","       "}; label_x=11'd838; label_y=10'd122;
            label_color=WHITE;
        end else if (ay>=10'd154 && ay<10'd170) begin
            label_text={"DOUBLE TAP","      "}; label_x=11'd838; label_y=10'd154;
            label_color=INK;
        end else if (ay>=10'd182 && ay<10'd198) begin
            label_text=touch_ready ? {"TOUCH OK","        "} : touch_error!=0 ? {"TP ERROR ",8'h30+{4'd0,touch_error},"      "} : {"TOUCH WAIT","      "}; label_x=11'd838; label_y=10'd182;
            label_color=touch_ready ? GREEN : RED;
        end else if (ay>=10'd210 && ay<10'd226) begin
            label_text=filter_active ? {"GAUSS ON","        "} : {"GAUSS OFF","       "}; label_x=11'd838; label_y=10'd210;
            label_color=filter_active ? GREEN : INK;
        end else if (ay>=10'd238 && ay<10'd254) begin
            label_text=net_ready ? {"UDP 400X240","     "} : {"PHY WAIT","        "}; label_x=11'd838; label_y=10'd238;
            label_color=net_ready ? BLUE : RED;
        end else if (ay>=10'd270 && ay<10'd286) begin
            label_text=selection_valid ? {"ROI PENDING","     "} : {"NO TARGET","       "}; label_x=11'd838; label_y=10'd270;
            label_color=selection_valid ? ORANGE : INK;
        end else if (ay>=10'd298 && ay<10'd314) begin
            label_text={"TRACKER OFF","     "}; label_x=11'd838; label_y=10'd298;
            label_color=INK;
        end else if (ay>=10'd326 && ay<10'd342) begin
            label_text=zoom_level==0 ? {"HOST REQ 1X","     "} : {"HOST REQ 2X","     "}; label_x=11'd838; label_y=10'd326;
            label_color=BLUE;
        end else if (ay>=10'd354 && ay<10'd370) begin
            label_text={"LCD 1X","          "}; label_x=11'd838; label_y=10'd354;
            label_color=INK;
        end else if (ay>=10'd402 && ay<10'd418) begin
            label_text={"CLEAR ROI","       "}; label_x=11'd838; label_y=10'd402;
            label_color=BLUE;
        end else if (ay>=10'd454 && ay<10'd470) begin
            label_text={"CAPTURE 5S","      "}; label_x=11'd838; label_y=10'd454;
            label_color=WHITE;
        end else if (ay>=10'd494 && ay<10'd510) begin
            label_text={"PC RECEIVER","     "}; label_x=11'd838; label_y=10'd494;
            label_color=INK;
        end else if (ay>=10'd518 && ay<10'd534) begin
            label_text=stop_latched ? {"STOP FLAG","       "} : {"NO MOTOR","        "}; label_x=11'd838; label_y=10'd518;
            label_color=stop_latched ? RED : INK;
        end else if (ay>=10'd555 && ay<10'd571) begin
            label_text={"UDP 400X240","     "}; label_x=11'd20; label_y=10'd555;
            label_color=BLUE;
        end else if (ay>=10'd577 && ay<10'd593) begin
            label_text=selection_valid ? "ROI REQUEST ONLY" : debug_mode ? "DOUBLE TAP VIDEO" : "CAMERA 800X480  "; label_x=11'd20; label_y=10'd577;
            label_color=INK;
        end else label_enable = 1'b0;
    end

    wire [10:0] dx = ax-label_x;
    wire [9:0] dy = ay-label_y;
    wire in_label = label_enable && ax>=label_x && dx<11'd256 &&
                    ay>=label_y && dy<10'd16;
    wire [3:0] char_ix = dx[7:4];
    wire [3:0] glyph_x_now = dx[3:1];
    wire [3:0] glyph_y_now = dy[3:1];
    wire [7:0] character = label_text[127-(char_ix*8) -: 8];

    // Three registered display stages: character/background -> glyph -> RGB.
    // Capture the FWFT camera pixel in the FIRST stage, on camera_pop's edge.
    // Delay DE/HS/VS with RGB so the image geometry stays aligned.
    reg [7:0] character_a = 0;
    reg [3:0] glyph_x_a = 0, glyph_y_a = 0;
    reg [3:0] glyph_x_b = 0, glyph_y_b = 0;
    reg [39:0] font_b = 0;
    reg in_label_a = 0, in_label_b = 0;
    reg [15:0] label_color_a = 0, label_color_b = 0;
    reg [15:0] base_color_a = BLACK, base_color_b = BLACK;
    reg de_a = 0, de_b = 0;
    reg hs_a = 1, hs_b = 1, vs_a = 1, vs_b = 1;
    wire [39:0] font = font_b;
    wire [3:0] glyph_x = glyph_x_b;
    wire [3:0] glyph_y = glyph_y_b;
    wire text_pixel = in_label_b && glyph_x<4'd5 && glyph_y<4'd7 &&
                      font[32-(glyph_x*8)+glyph_y];

    wire [10:0] roi_cx={1'b0,selection_x}+11'd16;
    wire [9:0] roi_cy=selection_y+10'd56;
    wire [10:0] roi_x0=roi_cx>11'd56 ? roi_cx-11'd40 : 11'd16;
    wire [10:0] roi_x1=roi_cx<11'd776 ? roi_cx+11'd40 : 11'd815;
    wire [9:0] roi_y0=roi_cy>10'd86 ? roi_cy-10'd30 : 10'd56;
    wire [9:0] roi_y1=roi_cy<10'd506 ? roi_cy+10'd30 : 10'd535;
    reg [15:0] base_color;
    always @* begin
        base_color = CREAM;
        if (active) begin
            if (ax[4:0]==5'd2 && ay[4:0]==5'd2) base_color=MUTED;
            if (ay<10'd46) base_color = BLUE;
            else if (ay>=10'd546) base_color = CREAM;
            else if (ax>=11'd828 && ax<11'd1018 &&
                     ay>=10'd56 && ay<10'd536) base_color = PANEL;
            if (video_area) base_color = camera_pixel_valid && !camera_underflow
                                         ? camera_pixel : BLACK;
            // Black frame and offset shadow echo the portfolio's bold cards.
            if ((ax>=11'd17 && ax<11'd825 && ay>=10'd540 && ay<10'd545) ||
                (ax>=11'd820 && ax<11'd825 && ay>=10'd57 && ay<10'd545))
                base_color = INK;
            if ((ax>=11'd12 && ax<11'd820 &&
                 ((ay>=10'd52 && ay<10'd56) ||
                  (ay>=10'd536 && ay<10'd540))) ||
                (ay>=10'd52 && ay<10'd540 &&
                 ((ax>=11'd12 && ax<11'd16) ||
                  (ax>=11'd816 && ax<11'd820))))
                base_color = INK;
            if (ay>=10'd46 && ay<10'd50) base_color = INK;
            if (ay>=10'd545 && ay<10'd549) base_color = INK;
            if (ax>=11'd824 && ax<11'd828 && ay>=10'd56 && ay<10'd536)
                base_color = INK;
            if (ax>=11'd1018 && ax<11'd1022 && ay>=10'd56 && ay<10'd536)
                base_color = INK;
            if (ax>=11'd828 && ax<11'd1018 &&
                (ay==10'd190 || ay==10'd318 || ay==10'd478))
                base_color = INK;
            // Clickable sidebar cards use exactly the gesture hit rectangles.
            if (ax>=11'd832 && ax<11'd1008 &&
                ((ay>=10'd112 && ay<10'd152) ||
                 (ay>=10'd440 && ay<10'd488)))
                base_color = BLUE;
            if (ax>=11'd832 && ax<11'd1008 && ay>=10'd392 && ay<10'd432)
                base_color = CREAM;
            if (ax>=11'd832 && ax<11'd1008 &&
                (ay==10'd112 || ay==10'd151 || ay==10'd392 ||
                 ay==10'd431 || ay==10'd440 || ay==10'd487))
                base_color=INK;
            if (debug_mode && video_area &&
                ((ax==11'd416 && ay>=10'd270 && ay<10'd322) ||
                 (ay==10'd296 && ax>=11'd390 && ax<11'd442)))
                base_color = ORANGE;
            // A selected point is an ROI REQUEST, never a detected/tracked box.
            if (selection_valid && video_area &&
                (((ax==roi_x0 || ax==roi_x1) && ay>=roi_y0 && ay<=roi_y1) ||
                 ((ay==roi_y0 || ay==roi_y1) && ax>=roi_x0 && ax<=roi_x1)))
                base_color = ORANGE;
            if (stop_latched && ax>=11'd828 && ax<11'd1018 &&
                ay>=10'd516 && ay<10'd536) base_color = RED;
        end
    end

    always @(posedge clk or negedge run) begin
        if (!run) begin
            h<=0; v<=0; rgb<=BLACK;
            de<=0; hs<=1; vs<=1;
            frame_div<=0; frame_heartbeat<=0;
            camera_underflow<=0;
            character_a<=0; font_b<=0;
            glyph_x_a<=0; glyph_y_a<=0; glyph_x_b<=0; glyph_y_b<=0;
            in_label_a<=0; in_label_b<=0;
            label_color_a<=0; label_color_b<=0;
            base_color_a<=BLACK; base_color_b<=BLACK;
            de_a<=0; de_b<=0; hs_a<=1; hs_b<=1; vs_a<=1; vs_b<=1;
        end else begin
            if (frame_restart) camera_underflow<=0;
            else if (video_area && !camera_pixel_valid)
                camera_underflow<=1;
            if (h==H_TOTAL-1'b1) begin
                h<=0;
                if (v==V_TOTAL-1'b1) begin
                    v<=0;
                    if (frame_div==5'd28) begin
                        frame_div<=0; frame_heartbeat<=~frame_heartbeat;
                    end else frame_div<=frame_div+1'b1;
                end else v<=v+1'b1;
            end else h<=h+1'b1;
            character_a<=character;
            glyph_x_a<=glyph_x_now; glyph_y_a<=glyph_y_now;
            in_label_a<=active && in_label;
            label_color_a<=label_color;
            base_color_a<=base_color;
            de_a<=active; hs_a<=(h>=H_SYNC); vs_a<=(v>=V_SYNC);

            font_b<=glyph(character_a);
            glyph_x_b<=glyph_x_a; glyph_y_b<=glyph_y_a;
            in_label_b<=in_label_a; label_color_b<=label_color_a;
            base_color_b<=base_color_a;
            de_b<=de_a; hs_b<=hs_a; vs_b<=vs_a;

            de<=de_b; hs<=hs_b; vs<=vs_b;
            rgb<=text_pixel ? label_color_b : base_color_b;
        end
    end
endmodule

