// R4: separate cards, clipped bitmap text and real LCD 1x/2x crop zoom.
// Camera/DDR timing and the 800x480 viewport remain at the verified geometry.
module lcd1024_interaction_ui (
    input wire clk, run,
    input wire [15:0] camera_pixel,
    input wire camera_pixel_valid,
    input wire camera_ready, ddr_ready, ddr_pll_ready, net_ready,
    input wire filter_active, debug_mode, stop_latched, touch_ready,
    input wire [3:0] touch_error,
    input wire [1:0] touch_count,
    input wire selection_valid,
    input wire [9:0] selection_x, selection_y,
    input wire [1:0] zoom_level,
    output wire camera_pop, frame_restart,
    output wire display_zoom_2x,
    output reg [15:0] rgb = 16'h0000,
    output reg de = 1'b0, hs = 1'b1, vs = 1'b1,
    output reg frame_heartbeat = 1'b0,
    output wire camera_underflow
);
    localparam [10:0] H_SYNC=20, H_START=160, H_END=1184, H_TOTAL=1344;
    localparam [9:0] V_SYNC=3, V_START=23, V_END=623, V_TOTAL=635;
    // EDispense_Vision-inspired neutral cards, blue action, quiet thin borders.
    localparam [15:0] BACKGROUND=16'hf79e, WHITE=16'hffff,
        BORDER=16'hd6bb, BLUE=16'h03df, SOFT_BLUE=16'he79f,
        GREEN=16'h13a8, RED=16'hd965, INK=16'h1926,
        MUTED=16'h63b1, ORANGE=16'hfbe0, BLACK=16'h0000;
    reg [10:0] h = 0;
    reg [9:0] v = 0;
    reg [4:0] frame_div = 0;
    wire active = h>=H_START && h<H_END && v>=V_START && v<V_END;
    wire [10:0] ax = h-H_START;
    wire [9:0] ay = v-V_START;
    wire video_area = active && ax>=11'd16 && ax<11'd816 &&
        ay>=10'd56 && ay<10'd536;
    // A synchronous line-RAM read is issued one pixel before its display slot.
    wire video_fetch = active && ax>=11'd15 && ax<11'd815 &&
        ay>=10'd56 && ay<10'd536;
    wire [10:0] fetch_x_wide = ax-11'd15;
    wire [9:0] fetch_y_wide = ay-10'd56;
    assign frame_restart = run && h==H_TOTAL-1'b1 && v==V_TOTAL-1'b1;
    wire [15:0] display_pixel;
    wire display_valid, zoom_active, zoom_ready;
    assign display_zoom_2x=zoom_active;
    lcd_camera_zoom u_zoom (
        .clk(clk), .run(run), .frame_restart(frame_restart),
        .zoom_request(zoom_level), .video_area(video_area),
        .video_fetch(video_fetch), .video_fetch_x(fetch_x_wide[9:0]),
        .video_fetch_y(fetch_y_wide[8:0]),
        .camera_pixel(camera_pixel), .camera_pixel_valid(camera_pixel_valid),
        .camera_pop(camera_pop), .display_pixel(display_pixel),
        .display_valid(display_valid), .zoom_active(zoom_active),
        .zoom_ready(zoom_ready), .frame_underflow(camera_underflow)
    );
    wire selection_visible = selection_valid && (!zoom_active ||
        (selection_x>=10'd200 && selection_x<10'd600 &&
         selection_y>=10'd120 && selection_y<10'd360));
    wire [7:0] error_ascii = touch_error<4'd10 ?
        8'h30+{4'd0,touch_error} : 8'h41+{4'd0,touch_error}-8'd10;
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
    reg [10:0] label_x, label_right;
    reg [9:0] label_y;
    reg [15:0] label_color;
    reg label_enable;
    always @* begin
        label_text=128'd0; label_x=0; label_y=0;
        label_right=11'd1024; label_color=INK; label_enable=1'b1;
        if (ay>=10'd14 && ay<10'd30 && ax>=11'd20 && ax<11'd280) begin
            label_text="VISION ROBOT R4 "; label_x=11'd20; label_y=10'd14;
            label_right=11'd280; label_color=INK;
        end
        else if (ay>=10'd14 && ay<10'd30 && ax>=11'd852 && ax<11'd1008) begin
            label_text="FPGA UI         "; label_x=11'd852; label_y=10'd14;
            label_right=11'd1008; label_color=BLUE;
        end
        else if (ay>=10'd62 && ay<10'd78 && ax>=11'd840 && ax<11'd1000) begin
            label_text="STATUS          "; label_x=11'd840; label_y=10'd62;
            label_right=11'd1000; label_color=MUTED;
        end
        else if (ay>=10'd84 && ay<10'd100 && ax>=11'd840 && ax<11'd1000) begin
            label_text=!camera_ready ? "CAM WAIT        " : !ddr_pll_ready ? "PLL WAIT        " : !ddr_ready ? "DDR WAIT        " : "CAM DDR OK      "; label_x=11'd840; label_y=10'd84;
            label_right=11'd1000; label_color=camera_ready && ddr_ready && ddr_pll_ready ? GREEN : RED;
        end
        else if (ay>=10'd126 && ay<10'd142 && ax>=11'd840 && ax<11'd1000) begin
            label_text=debug_mode ? "DEBUG ON        " : "VIS DEBUG       "; label_x=11'd840; label_y=10'd126;
            label_right=11'd1000; label_color=debug_mode ? WHITE : BLUE;
        end
        else if (ay>=10'd168 && ay<10'd184 && ax>=11'd840 && ax<11'd1000) begin
            label_text=touch_ready ? "TOUCH OK        " : touch_error!=0 ? {"TP ERR ",error_ascii,"        "} : "TOUCH WAIT      "; label_x=11'd840; label_y=10'd168;
            label_right=11'd1000; label_color=touch_ready ? GREEN : RED;
        end
        else if (ay>=10'd192 && ay<10'd208 && ax>=11'd840 && ax<11'd1000) begin
            label_text={"FINGERS ",8'h30+{6'd0,touch_count},"       "}; label_x=11'd840; label_y=10'd192;
            label_right=11'd1000; label_color=MUTED;
        end
        else if (ay>=10'd242 && ay<10'd258 && ax>=11'd840 && ax<11'd1000) begin
            label_text=filter_active ? "GAUSS ON        " : "GAUSS OFF       "; label_x=11'd840; label_y=10'd242;
            label_right=11'd1000; label_color=filter_active ? BLUE : MUTED;
        end
        else if (ay>=10'd268 && ay<10'd284 && ax>=11'd840 && ax<11'd1000) begin
            label_text=net_ready ? "PHY INIT        " : "PHY WAIT        "; label_x=11'd840; label_y=10'd268;
            label_right=11'd1000; label_color=net_ready ? GREEN : RED;
        end
        else if (ay>=10'd314 && ay<10'd330 && ax>=11'd840 && ax<11'd1000) begin
            label_text=!selection_valid ? "NO TARGET       " : selection_visible ? "ROI SET         " : "ROI OUT         "; label_x=11'd840; label_y=10'd314;
            label_right=11'd1000; label_color=selection_valid ? BLUE : MUTED;
        end
        else if (ay>=10'd338 && ay<10'd354 && ax>=11'd840 && ax<11'd1000) begin
            label_text="TRACK OFF       "; label_x=11'd840; label_y=10'd338;
            label_right=11'd1000; label_color=MUTED;
        end
        else if (ay>=10'd362 && ay<10'd378 && ax>=11'd840 && ax<11'd1000) begin
            label_text=zoom_active && !zoom_ready ? "ZOOM WAIT       " : zoom_active ? "VIEW 2X         " : "VIEW 1X         "; label_x=11'd840; label_y=10'd362;
            label_right=11'd1000; label_color=zoom_active && !zoom_ready ? RED : BLUE;
        end
        else if (ay>=10'd406 && ay<10'd422 && ax>=11'd840 && ax<11'd1000) begin
            label_text="RESET VIEW      "; label_x=11'd840; label_y=10'd406;
            label_right=11'd1000; label_color=BLUE;
        end
        else if (ay>=10'd454 && ay<10'd470 && ax>=11'd840 && ax<11'd1000) begin
            label_text="CAPTURE 5S      "; label_x=11'd840; label_y=10'd454;
            label_right=11'd1000; label_color=WHITE;
        end
        else if (ay>=10'd506 && ay<10'd522 && ax>=11'd840 && ax<11'd1000) begin
            label_text=stop_latched ? "STOP FLAG       " : "NO MOTOR        "; label_x=11'd840; label_y=10'd506;
            label_right=11'd1000; label_color=stop_latched ? RED : MUTED;
        end
        else if (ay>=10'd558 && ay<10'd574 && ax>=11'd20 && ax<11'd280) begin
            label_text="CAM 800X480     "; label_x=11'd20; label_y=10'd558;
            label_right=11'd280; label_color=INK;
        end
        else if (ay>=10'd558 && ay<10'd574 && ax>=11'd300 && ax<11'd560) begin
            label_text="UDP 400X240     "; label_x=11'd300; label_y=10'd558;
            label_right=11'd560; label_color=INK;
        end
        else if (ay>=10'd558 && ay<10'd574 && ax>=11'd600 && ax<11'd820) begin
            label_text=filter_active ? "GAUSS 3X3       " : "RAW RGB565      "; label_x=11'd600; label_y=10'd558;
            label_right=11'd820; label_color=BLUE;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd20 && ax<11'd280) begin
            label_text="PINCH TO ZOOM   "; label_x=11'd20; label_y=10'd582;
            label_right=11'd280; label_color=MUTED;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd300 && ax<11'd560) begin
            label_text=debug_mode ? "DOUBLE TAP ROI  " : "DEBUG FOR ROI   "; label_x=11'd300; label_y=10'd582;
            label_right=11'd560; label_color=MUTED;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd600 && ax<11'd856) begin
            label_text="HOST SAVES PHOTO"; label_x=11'd600; label_y=10'd582;
            label_right=11'd856; label_color=MUTED;
        end
        else label_enable=1'b0;
    end

    wire [10:0] dx = ax-label_x;
    wire [9:0] dy = ay-label_y;
    wire in_label = label_enable && ax>=label_x && ax<label_right && dx<11'd192 &&
                    ay>=label_y && dy<10'd16;
    // 10px glyph plus 2px spacing; the column stays readable without wide gaps.
    wire [3:0] char_ix = dx/11'd12;
    wire [3:0] glyph_x_now = (dx%11'd12)>>1;
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

    wire [10:0] source_dx={1'b0,selection_x}-11'd200;
    wire [9:0] source_dy=selection_y-10'd120;
    wire [10:0] view_x=zoom_active ? {source_dx[9:0],1'b0} : {1'b0,selection_x};
    wire [9:0] view_y=zoom_active ? {source_dy[8:0],1'b0} : selection_y;
    wire [10:0] roi_cx=view_x+11'd16;
    wire [9:0] roi_cy=view_y+10'd56;
    wire [10:0] half_width=zoom_active ? 11'd80 : 11'd40;
    wire [9:0] half_height=zoom_active ? 10'd60 : 10'd30;
    wire [10:0] roi_x0=roi_cx>11'd16+half_width ? roi_cx-half_width : 11'd16;
    wire [10:0] roi_x1=roi_cx+half_width<11'd816 ? roi_cx+half_width : 11'd815;
    wire [9:0] roi_y0=roi_cy>10'd56+half_height ? roi_cy-half_height : 10'd56;
    wire [9:0] roi_y1=roi_cy+half_height<10'd536 ? roi_cy+half_height : 10'd535;

    // Every card is its own rectangle. No separator crosses a text row.
    function in_rect;
        input [10:0] x, left, right;
        input [9:0] y, top, bottom;
        begin in_rect=x>=left && x<right && y>=top && y<bottom; end
    endfunction
    reg [15:0] base_color;
    always @* begin
        base_color=BACKGROUND;
        if (active) begin
            if (ay<10'd44) base_color=WHITE;
            if (ay==10'd44 || ay==10'd546) base_color=BORDER;
            if (in_rect(ax,11'd12,11'd820,ay,10'd52,10'd540)) base_color=BORDER;
            if (video_area) base_color=display_valid ? display_pixel : BLACK;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd52,10'd108)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd53,10'd107)) base_color=WHITE;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd160,10'd224)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd161,10'd223)) base_color=WHITE;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd232,10'd296)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd233,10'd295)) base_color=WHITE;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd304,10'd384)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd305,10'd383)) base_color=WHITE;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd496,10'd536)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd497,10'd535)) base_color=WHITE;
            if (in_rect(ax,11'd832,11'd1008,ay,10'd112,10'd152)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd113,10'd151)) base_color=debug_mode ? BLUE : SOFT_BLUE;
            if (in_rect(ax,11'd832,11'd1008,ay,10'd392,10'd432)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd393,10'd431)) base_color=SOFT_BLUE;
            if (in_rect(ax,11'd832,11'd1008,ay,10'd440,10'd488)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd441,10'd487)) base_color=BLUE;
            if (debug_mode && video_area &&
                ((ax==11'd416 && ay>=10'd284 && ay<10'd309) ||
                 (ay==10'd296 && ax>=11'd404 && ax<11'd429))) base_color=BLUE;
            // User ROI only, not a detector/tracker result.
            if (selection_visible && video_area &&
                (((ax==roi_x0 || ax==roi_x1) && ay>=roi_y0 && ay<=roi_y1) ||
                 ((ay==roi_y0 || ay==roi_y1) && ax>=roi_x0 && ax<=roi_x1)))
                base_color=ORANGE;
        end
    end

    always @(posedge clk or negedge run) begin
        if (!run) begin
            h<=0; v<=0; rgb<=BLACK;
            de<=0; hs<=1; vs<=1;
            frame_div<=0; frame_heartbeat<=0;
            character_a<=0; font_b<=0;
            glyph_x_a<=0; glyph_y_a<=0; glyph_x_b<=0; glyph_y_b<=0;
            in_label_a<=0; in_label_b<=0;
            label_color_a<=0; label_color_b<=0;
            base_color_a<=BLACK; base_color_b<=BLACK;
            de_a<=0; de_b<=0; hs_a<=1; hs_b<=1; vs_a<=1; vs_b<=1;
        end else begin
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
