// R5: source-anchored 1x..4x zoom, unit-bearing sensors, zoom/stop buttons only.
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
    input wire zoom_mode,
    input wire [8:0] zoom_step, zoom_top, focus_y,
    input wire [9:0] zoom_left, focus_x,
    input wire detection_request,
    input wire [255:0] detection_data,
    output reg detection_ack = 0,
    input wire sensor_text_request,
    input wire [1023:0] sensor_text_data,
    output reg sensor_text_ack = 0,
    output wire camera_pop, frame_restart,
    output wire [8:0] display_step, display_top,
    output wire [9:0] display_left,
    output wire [27:0] ddr_read_base,
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
    // Unit-bearing sensor text is held in the 50 MHz source mailbox until
    // this frame-boundary ACK. No raw multi-bit sensor bus crosses to pixels.
    reg [2:0] sensor_request_sync = 0;
    reg [1023:0] sensor_rows = {"SENSOR WAIT     ","GX ----.--DPS   ","GY ----.--DPS   ","GZ ----.--DPS   ",
        "AX -.---G       ","AY -.---G       ","AZ -.---G       ","US WAIT         "};
    always @(posedge clk or negedge run) begin
        if (!run) begin
            sensor_request_sync<=0; sensor_text_ack<=0;
            sensor_rows<={"SENSOR WAIT     ","GX ----.--DPS   ","GY ----.--DPS   ","GZ ----.--DPS   ",
                "AX -.---G       ","AY -.---G       ","AZ -.---G       ","US WAIT         "};
        end else begin
            sensor_request_sync<={sensor_request_sync[1:0],sensor_text_request};
            if (frame_restart && sensor_request_sync[2]!=sensor_text_ack) begin
                sensor_rows<=sensor_text_data;
                sensor_text_ack<=sensor_request_sync[2];
            end
        end
    end
    // A toggle handshake protects the multi-bit CDC mailbox. Commit only
    // in vertical blanking, so one LCD frame never contains two different boxes.
    reg [2:0] detection_sync = 0;
    reg detection_seen = 0, detection_valid = 0, detection_box = 0;
    reg [5:0] detection_age_frames = 6'd60;
    reg [9:0] dog_x0 = 0, dog_x1 = 0;
    reg [8:0] dog_y0 = 0, dog_y1 = 0;
    reg [9:0] dog_confidence = 0;
    always @(posedge clk or negedge run) begin
        if (!run) begin
            detection_sync<=0; detection_ack<=0;
            detection_seen<=0; detection_valid<=0; detection_box<=0;
            detection_age_frames<=6'd60;
            dog_x0<=0; dog_x1<=0; dog_y0<=0; dog_y1<=0; dog_confidence<=0;
        end else begin
            detection_sync<={detection_sync[1:0],detection_request};
            if (frame_restart) begin
                if (detection_sync[2]!=detection_ack) begin
                    detection_ack<=detection_sync[2]; detection_seen<=1;
                    detection_age_frames<=0;
                    detection_valid<=detection_data[209];
                    detection_box<=detection_data[208];
                    dog_confidence<=detection_data[201:192];
                    // Host uses 400x240, exclusive right/bottom edges;
                    // camera source uses 800x480, inclusive overlay edges.
                    dog_x0<={detection_data[136:128],1'b0};
                    dog_y0<={detection_data[119:112],1'b0};
                    dog_x1<=({detection_data[104:96],1'b0})-10'd1;
                    dog_y1<=({detection_data[87:80],1'b0})-9'd1;
                end else if (detection_age_frames<6'd60) begin
                    detection_age_frames<=detection_age_frames+1'b1;
                    if (detection_age_frames==6'd59) begin
                        detection_valid<=0; detection_box<=0;
                    end
                end
            end
        end
    end
    wire [7:0] dog_whole=8'h30+dog_confidence/10'd1000;
    wire [7:0] dog_tenths=8'h30+(dog_confidence/10'd100)%10'd10;
    wire [7:0] dog_hundredths=8'h30+(dog_confidence/10'd10)%10'd10;
    wire [15:0] display_pixel;
    wire display_valid, zoom_active, zoom_ready;
    wire [9:0] sampled_x;
    wire [8:0] sampled_y;
    lcd_camera_zoom u_zoom (
        .clk(clk), .run(run), .frame_restart(frame_restart),
        .step_request(zoom_step), .left_request(zoom_left), .top_request(zoom_top),
        .video_area(video_area),
        .video_fetch(video_fetch), .video_fetch_x(fetch_x_wide[9:0]),
        .video_fetch_y(fetch_y_wide[8:0]),
        .camera_pixel(camera_pixel), .camera_pixel_valid(camera_pixel_valid),
        .camera_pop(camera_pop), .display_pixel(display_pixel),
        .display_valid(display_valid), .zoom_active(zoom_active),
        .display_source_x(sampled_x), .display_source_y(sampled_y),
        .active_step(display_step), .active_left(display_left), .active_top(display_top),
        .ddr_read_base(ddr_read_base),
        .zoom_ready(zoom_ready), .frame_underflow(camera_underflow)
    );
    wire [18:0] view_width_q8=display_step*10'd800+19'd255;
    wire [17:0] view_height_q8=display_step*9'd480+18'd255;
    wire selection_visible = selection_valid && selection_x>=display_left &&
        selection_x<display_left+view_width_q8[17:8] && selection_y>=display_top &&
        selection_y<display_top+view_height_q8[16:8];
    // Scale text computed once per frame with a serial divider, not with a
    // variable divide on the per-pixel rendering path. Rounded hundredths.
    reg [15:0] scale_dividend=0, scale_quotient=0, scale_hundred=100;
    reg [9:0] scale_remainder=0;
    reg [8:0] scale_divisor=256;
    reg [4:0] scale_cycles=0;
    wire [10:0] scale_trial={scale_remainder,scale_dividend[15]};
    wire scale_bit=scale_trial>={2'd0,scale_divisor};
    wire [15:0] scale_next={scale_quotient[14:0],scale_bit};
    always @(posedge clk or negedge run) begin
        if (!run) begin
            scale_dividend<=0; scale_quotient<=0; scale_hundred<=100;
            scale_remainder<=0; scale_divisor<=256; scale_cycles<=0;
        end else if (h==11'd1 && v==10'd0) begin
            scale_dividend<=16'd25600+{8'd0,display_step[8:1]};
            scale_divisor<=display_step; scale_quotient<=0; scale_remainder<=0; scale_cycles<=16;
        end else if (scale_cycles!=0) begin
            scale_dividend<={scale_dividend[14:0],1'b0}; scale_quotient<=scale_next;
            scale_remainder<=scale_bit ? scale_trial-{2'd0,scale_divisor} : scale_trial;
            scale_cycles<=scale_cycles-1'b1;
            if (scale_cycles==1) scale_hundred<=scale_next;
        end
    end
    wire [7:0] scale_units=8'h30+scale_hundred/16'd100;
    wire [7:0] scale_tenths=8'h30+(scale_hundred/16'd10)%16'd10;
    wire [7:0] scale_hundredths=8'h30+scale_hundred%16'd10;
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
                "+":glyph=40'h08083e0808;
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
            label_text="STUDY ASSIST R5 "; label_x=11'd20; label_y=10'd14;
            label_right=11'd280; label_color=INK;
        end
        else if (ay>=10'd14 && ay<10'd30 && ax>=11'd852 && ax<11'd1008) begin
            label_text=touch_ready ? "TP OK           " : "TP WAIT         "; label_x=11'd852; label_y=10'd14;
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
        else if (ay>=10'd126 && ay<10'd142 && ax>=11'd836 && ax<11'd1008) begin
            label_text=zoom_active && !zoom_ready ? "ZOOM WAIT       " : {"VIEW ",scale_units,".",scale_tenths,scale_hundredths,"X      "};
            label_x=11'd836; label_y=10'd126; label_right=11'd1008; label_color=BLUE;
        end
        else if (ay>=10'd174 && ay<10'd190 && ax>=11'd836 && ax<11'd1008) begin
            label_text="ZOOM IN MODE    "; label_x=11'd836; label_y=10'd174;
            label_right=11'd1008; label_color=zoom_mode ? WHITE : BLUE;
        end
        else if (ay>=10'd220 && ay<10'd236 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[1023 -: 128]; label_x=11'd836; label_y=10'd220;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd252 && ay<10'd268 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[895 -: 128]; label_x=11'd836; label_y=10'd252;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd284 && ay<10'd300 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[767 -: 128]; label_x=11'd836; label_y=10'd284;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd316 && ay<10'd332 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[639 -: 128]; label_x=11'd836; label_y=10'd316;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd348 && ay<10'd364 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[511 -: 128]; label_x=11'd836; label_y=10'd348;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd380 && ay<10'd396 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[383 -: 128]; label_x=11'd836; label_y=10'd380;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd412 && ay<10'd428 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[255 -: 128]; label_x=11'd836; label_y=10'd412;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd444 && ay<10'd460 && ax>=11'd836 && ax<11'd1008) begin
            label_text=sensor_rows[127 -: 128]; label_x=11'd836; label_y=10'd444;
            label_right=11'd1008; label_color=INK;
        end
        else if (ay>=10'd510 && ay<10'd526 && ax>=11'd836 && ax<11'd1008) begin
            label_text=stop_latched ? "STOP LATCHED    " : "EMERGENCY STOP  "; label_x=11'd836; label_y=10'd510;
            label_right=11'd1008; label_color=WHITE;
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
            label_text=zoom_mode ? "TAP TAP FOCUS   " : debug_mode ? "TAP TAP SELECT  " : "DEBUG TO SELECT "; label_x=11'd300; label_y=10'd582;
            label_right=11'd560; label_color=MUTED;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd600 && ax<11'd856) begin
            label_text=!detection_seen ? "AI WAIT         " : detection_age_frames==6'd60 ? "AI STALE        " :
                !detection_valid ? "AI INVALID      " : !detection_box ? "NO DOG          " :
                {"DOG ",dog_whole,".",dog_tenths,dog_hundredths,"        "};
            label_x=11'd600; label_y=10'd582;
            label_right=11'd856; label_color=detection_valid && detection_box ? GREEN : MUTED;
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

    // Compare overlays in source coordinates. The same transform as the
    // image fetch is used, so there is no variable pixel-path division.
    wire [10:0] raw_source_x=ax-11'd16;
    wire [9:0] raw_source_y=ay-10'd56;
    wire [9:0] source_x=zoom_active ? sampled_x : raw_source_x[9:0];
    wire [8:0] source_y=zoom_active ? sampled_y : raw_source_y[8:0];
    // Four source pixels give a visible border at 1x, using the existing
    // sampled coordinates automatically follows R5 pan/zoom and clips it.
    wire dog_border=detection_valid && detection_box && source_x>=dog_x0 &&
        source_x<=dog_x1 && source_y>=dog_y0 && source_y<=dog_y1 &&
        (source_x-dog_x0<10'd4 || dog_x1-source_x<10'd4 ||
         source_y-dog_y0<9'd4 || dog_y1-source_y<9'd4);
    wire [9:0] roi_x0=selection_x>40 ? selection_x-10'd40 : 10'd0;
    wire [9:0] roi_x1=selection_x<759 ? selection_x+10'd40 : 10'd799;
    wire [8:0] roi_y0=selection_y>30 ? selection_y-10'd30 : 9'd0;
    wire [8:0] roi_y1=selection_y<449 ? selection_y+10'd30 : 9'd479;
    wire [9:0] focus_dx=source_x>=focus_x ? source_x-focus_x : focus_x-source_x;
    wire [8:0] focus_dy=source_y>=focus_y ? source_y-focus_y : focus_y-source_y;

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
            if (in_rect(ax,11'd832,11'd1008,ay,10'd160,10'd200)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd161,10'd199)) base_color=zoom_mode ? BLUE : SOFT_BLUE;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd212,10'd476)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd213,10'd475)) base_color=WHITE;
            if (in_rect(ax,11'd832,11'd1008,ay,10'd496,10'd536)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd497,10'd535)) base_color=RED;
            if (zoom_mode && video_area && display_valid &&
                ((focus_dx<=6 && focus_dy==0) || (focus_dy<=6 && focus_dx==0))) base_color=BLUE;
            // User ROI only, not a detector/tracker result.
            if (!zoom_mode && selection_visible && video_area && display_valid &&
                (((source_x==roi_x0 || source_x==roi_x1) && source_y>=roi_y0 && source_y<=roi_y1) ||
                 ((source_y==roi_y0 || source_y==roi_y1) && source_x>=roi_x0 && source_x<=roi_x1)))
                base_color=ORANGE;
            if (video_area && display_valid && dog_border) base_color=GREEN;
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
