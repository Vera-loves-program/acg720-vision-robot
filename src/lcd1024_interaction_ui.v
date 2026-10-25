// Source-anchored 1x..4x zoom, unit-bearing sensors and motor PID tuning page.
// Camera/DDR timing and the 800x480 viewport remain at the verified geometry.
module lcd1024_interaction_ui #(
    parameter BYTE_TEXT_MODE=0
) (
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
    input wire pid_page,
    input wire pid_text_request,
    input wire [2047:0] pid_text_data,
    output reg pid_text_ack = 0,
    input wire actuator_test,
    input wire test_status_request,
    input wire [2064:0] test_status_data,
    output reg test_status_ack = 0,
    input wire battery_status_request,
    input wire [1921:0] battery_status_data,
    output reg battery_status_ack = 0,
    input wire robot_control,
    input wire robot_status_request,
    input wire [2079:0] robot_status_data,
    output reg robot_status_ack=0,
    // Optional synchronous byte snapshots replace the two wide PID/NAV
    // mailboxes. Their one-clock RAM Q is the existing character A stage.
    input wire [7:0] robot_byte_character,pid_byte_character,sensor_byte_character,
    input wire [31:0] robot_byte_meta,
    output wire robot_byte_read_en,pid_byte_read_en,sensor_byte_read_en,
    output wire [7:0] byte_text_read_index,
    input wire [1:0] map_cell,
    input wire [39:0] map_car_model,
    input wire map_read_valid,
    output wire map_read_en,
    output wire [6:0] map_read_x,map_read_y,
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
    reg [2:0] robot_request_sync=0;
    reg [2079:0] robot_status=0;
    always @(posedge clk or negedge run)begin
        if(!run)begin robot_request_sync<=0;robot_status_ack<=0;robot_status<=0;end
        else begin
            robot_request_sync<={robot_request_sync[1:0],robot_status_request};
            if(!BYTE_TEXT_MODE && frame_restart && robot_request_sync[2]!=robot_status_ack)begin
                robot_status<=robot_status_data;robot_status_ack<=robot_request_sync[2];
            end
        end
    end
    reg [2:0] test_request_sync=0;
    reg [2064:0] test_status=0;
    reg [2:0] battery_request_sync=0;
    reg [1921:0] battery_status={2'b00,"A BQ WAIT       ","B BQ WAIT       ",
        "A N~----- I-----","B N~----- I-----","A:--- B:--- NTC~",1280'b0};
    always @(posedge clk or negedge run) begin
        if (!run) begin
            battery_request_sync<=0;battery_status_ack<=0;
            battery_status<={2'b00,"A BQ WAIT       ","B BQ WAIT       ",
                "A N~----- I-----","B N~----- I-----","A:--- B:--- NTC~",1280'b0};
        end else begin
            battery_request_sync<={battery_request_sync[1:0],battery_status_request};
            if (frame_restart && battery_request_sync[2]!=battery_status_ack) begin
                battery_status<=battery_status_data;battery_status_ack<=battery_request_sync[2];
            end
        end
    end
    always @(posedge clk or negedge run) begin
        if (!run) begin test_request_sync<=0;test_status_ack<=0;test_status<=0;end
        else begin
            test_request_sync<={test_request_sync[1:0],test_status_request};
            if (frame_restart && test_request_sync[2]!=test_status_ack) begin
                test_status<=test_status_data;
                test_status_ack<=test_request_sync[2];
            end
        end
    end
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
            if (!BYTE_TEXT_MODE && frame_restart && sensor_request_sync[2]!=sensor_text_ack) begin
                sensor_rows<=sensor_text_data;
                sensor_text_ack<=sensor_request_sync[2];
            end
        end
    end
    // Parameter/status strings are prepared in the control-clock domain.
    // Hold the 16x16-byte mailbox until the frame-boundary toggle ACK, as for
    // the sensor mailbox above. Latching the page here avoids mid-frame tears.
    reg [2:0] pid_request_sync = 0, pid_page_sync = 0;
    reg pid_page_legacy = 0;
    // Robot titles, menu highlights and PID cards select the SAME mailbox
    // page committed at frame_restart; the independent legacy toggle must
    // not select cards ahead of the matching robot status/title snapshot.
    wire [31:0] robot_display_flags=BYTE_TEXT_MODE ? robot_byte_meta : robot_status[2079:2048];
    wire pid_page_display=robot_control ? (robot_display_flags[2:0]==3'd1) : pid_page_legacy;
    reg [2047:0] pid_rows = {"PID WAIT DATA   ",1920'd0};
    always @(posedge clk or negedge run) begin
        if (!run) begin
            pid_request_sync<=0; pid_page_sync<=0; pid_text_ack<=0;
            pid_page_legacy<=0; pid_rows<={"PID WAIT DATA   ",1920'd0};
        end else begin
            pid_request_sync<={pid_request_sync[1:0],pid_text_request};
            pid_page_sync<={pid_page_sync[1:0],pid_page};
            if (frame_restart) begin
                pid_page_legacy<=pid_page_sync[2];
                if (!BYTE_TEXT_MODE && pid_request_sync[2]!=pid_text_ack) begin
                    pid_rows<=pid_text_data;
                    pid_text_ack<=pid_request_sync[2];
                end
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
                "~":glyph=40'h0804081008;
                "=":glyph=40'h1414141414; ">":glyph=40'h0041221408;
                "<":glyph=40'h0814224100; "%":glyph=40'h6313086463;
                "_":glyph=40'h4040404040; "?":glyph=40'h0201510906;
                "[":glyph=40'h007f414100; "]":glyph=40'h0041417f00;
                default:glyph=40'h0000000000;
            endcase
        end
    endfunction

    reg [127:0] label_text;
    reg [10:0] label_x, label_right;
    reg [9:0] label_y;
    reg [15:0] label_color;
    reg label_enable;
    wire [127:0] test_label_text;
    wire [10:0] test_label_x,test_label_right;
    wire [9:0] test_label_y;
    wire [15:0] test_label_color,test_base_color;
    wire test_label_enable;
    wire [2:0] robot_label_source;
    wire [3:0] robot_label_row;
    wire [6:0] robot_label_literal;
    wire [10:0] robot_label_x,robot_label_right;
    wire [9:0] robot_label_y;
    wire [15:0] robot_label_color,robot_base_color;
    wire robot_label_enable,robot_override_label,robot_override_base;
    robot_control_panel u_robot_panel (
        .x(ax),.y(ay),.flags(robot_display_flags),
        .map_cell(map_cell),.map_read_valid(map_read_valid),
        .map_car_model(map_car_model),
        .map_read_en(map_read_en),.map_read_x(map_read_x),.map_read_y(map_read_y),
        .label_source(robot_label_source),.label_row(robot_label_row),.label_literal(robot_label_literal),
        .label_x(robot_label_x),.label_right(robot_label_right),
        .label_y(robot_label_y),.label_color(robot_label_color),.base_color(robot_base_color),
        .label_enable(robot_label_enable),.override_label(robot_override_label),.override_base(robot_override_base)
    );
    actuator_test_panel u_test_panel (
        .x(ax),.y(ay),.rows(test_status[2047:0]),.flags(test_status[2063:2048]),
        .sensor_rows(sensor_rows),.label_text(test_label_text),
        .battery_rows(battery_status[1919:1280]),.battery_valid(battery_status[1921:1920]),
        .diagnostic_rows(battery_status[1279:0]),.bq_info(test_status[2064]),
        .label_x(test_label_x),.label_right(test_label_right),.label_y(test_label_y),
        .label_color(test_label_color),.base_color(test_base_color),.label_enable(test_label_enable)
    );
    wire [9:0] pid_text_y=ay-10'd76;
    wire [3:0] pid_text_row=pid_text_y/10'd24;
    wire [4:0] pid_text_dy=pid_text_y%10'd24;
    always @* begin
        label_text=128'd0; label_x=0; label_y=0;
        label_right=11'd1024; label_color=INK; label_enable=1'b1;
        if (pid_page_display && ax>=11'd36 && ax<11'd228 && ay>=10'd76 && ay<10'd452 && pid_text_dy<16) begin
            label_text=pid_rows[2047-(pid_text_row*128) -: 128];
            label_x=11'd36; label_y=10'd76+pid_text_row*10'd24;
            label_right=11'd228; label_color=INK;
        end
        else if (pid_page_display && (ax>=11'd408 && ax<11'd796) &&
            (ay>=10'd86 && ay<10'd438)) begin
            // Text starts 8 pixels inside each 188-pixel button. A label
            // is enabled only in its own rectangle, including row gaps.
            label_x=ax<11'd608 ? 11'd416 : 11'd616;
            label_right=ax<11'd608 ? 11'd596 : 11'd796;
            if (ay>=86 && ay<102) begin
                label_text=ax<608 ? "WHEEL           " : "EDIT LOOP       "; label_y=86; label_color=BLUE;
            end else if (ay>=142 && ay<158) begin
                label_text=ax<608 ? "PREV FIELD      " : "NEXT FIELD      "; label_y=142; label_color=BLUE;
            end else if (ay>=198 && ay<214) begin
                label_text=ax<608 ? "VALUE -         " : "VALUE +         "; label_y=198; label_color=BLUE;
            end else if (ay>=254 && ay<270) begin
                label_text=ax<608 ? "STEP            " : "ZERO POSITION   "; label_y=254; label_color=BLUE;
            end else if (ay>=310 && ay<326) begin
                label_text=ax<608 ? "SPEED MODE      " : "POSITION MODE   "; label_y=310; label_color=BLUE;
            end else if (ay>=366 && ay<382) begin
                label_text=ax<608 ? "ARM             " : "RUN             "; label_y=366; label_color=WHITE;
            end else if (ay>=422 && ay<438) begin
                label_text=ax<608 ? "CLEAR FAULT     " : "STOP            "; label_y=422;
                label_color=ax<608 ? BLUE : WHITE;
            end else label_enable=0;
            if ((ax>=596 && ax<608) || ax<label_x) label_enable=0;
        end
        else if (pid_page_display && ax>=11'd408 && ax<11'd796 && ay>=10'd478 && ay<10'd494) begin
            label_text="UNLOCK POS      "; label_x=424; label_y=478; label_right=796; label_color=BLUE;
        end
        else if (pid_page_display && ax>=11'd408 && ax<11'd796 && ay>=10'd516 && ay<10'd532) begin
            label_text="STOP BEFORE EDIT"; label_x=408; label_y=516; label_right=796; label_color=MUTED;
        end
        else if (ay>=10'd14 && ay<10'd30 && ax>=11'd20 && ax<11'd280) begin
            label_text=pid_page_display ? "MOTOR PID TUNING" : "STUDY ASSIST R5 "; label_x=11'd20; label_y=10'd14;
            label_right=11'd280; label_color=INK;
        end
        else if (ay>=10'd14 && ay<10'd30 && ax>=11'd852 && ax<11'd1008) begin
            label_text=touch_ready ? "TP OK           " : "TP WAIT         "; label_x=11'd852; label_y=10'd14;
            label_right=11'd1008; label_color=BLUE;
        end
        else if (ay>=10'd14 && ay<10'd30 && ax>=11'd600 && ax<11'd800) begin
            label_text=zoom_active && !zoom_ready ? "ZOOM WAIT       " : {"VIEW ",scale_units,".",scale_tenths,scale_hundredths,"X      "};
            label_x=11'd600; label_y=10'd14; label_right=11'd800; label_color=BLUE;
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
            label_text=pid_page_display ? "RETURN CAMERA   " : "PID MODE        ";
            label_x=11'd836; label_y=10'd126; label_right=11'd1008; label_color=pid_page_display ? WHITE : BLUE;
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
            label_text=pid_page_display ? "SPD=COUNT/10MS  " : "CAM 800X480     "; label_x=11'd20; label_y=10'd558;
            label_right=11'd280; label_color=INK;
        end
        else if (ay>=10'd558 && ay<10'd574 && ax>=11'd300 && ax<11'd560) begin
            label_text=pid_page_display ? "POS=COUNT       " : "UDP 400X240     "; label_x=11'd300; label_y=10'd558;
            label_right=11'd560; label_color=INK;
        end
        else if (ay>=10'd558 && ay<10'd574 && ax>=11'd600 && ax<11'd820) begin
            label_text=filter_active ? "GAUSS 3X3       " : "RAW RGB565      "; label_x=11'd600; label_y=10'd558;
            label_right=11'd820; label_color=BLUE;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd20 && ax<11'd280) begin
            label_text=pid_page_display ? "SPEED THEN POS  " : "PINCH TO ZOOM   "; label_x=11'd20; label_y=10'd582;
            label_right=11'd280; label_color=MUTED;
        end
        else if (ay>=10'd582 && ay<10'd598 && ax>=11'd300 && ax<11'd560) begin
            label_text=pid_page_display ? "ONE TAP PER STEP" : zoom_mode ? "TAP TAP FOCUS   " : debug_mode ? "TAP TAP SELECT  " : "DEBUG TO SELECT "; label_x=11'd300; label_y=10'd582;
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
        if (actuator_test) begin
            label_text=test_label_text;label_x=test_label_x;label_y=test_label_y;
            label_right=test_label_right;label_color=test_label_color;label_enable=test_label_enable;
        end
        // Robot pages provide ALL of their text, including PID and blank slots.
        // The constant top-level ROBOT build can therefore prune the complete
        // legacy label-selection tree instead of retaining overlapping layouts.
        if(robot_control)begin
            // ROBOT text is a descriptor, not a 128-bit whole-row selector.
            // Keep the legacy/test row path intact for ROBOT_MODE=0.
            label_text=128'd0;label_x=robot_label_x;label_y=robot_label_y;
            label_right=robot_label_right;label_color=robot_label_color;label_enable=robot_label_enable;
        end
    end

    wire [10:0] dx = ax-label_x;
    wire [9:0] dy = ay-label_y;
    wire in_label = label_enable && ax>=label_x && ax<label_right && dx<11'd192 &&
                    ay>=label_y && dy<10'd16;
    // Split layout selection from character addressing. The layout task mux,
    // coordinate subtraction and /12/string mux must not all occupy one pixel
    // clock. All fields in this snapshot belong to the same source pixel.
    reg [127:0] label_text_l = 0;
    reg robot_control_l = 0;
    reg [2:0] robot_label_source_l = 0;
    reg [3:0] robot_label_row_l = 0;
    reg [6:0] robot_label_literal_l = 0;
    reg [10:0] label_dx_l = 0;
    reg [3:0] label_dy_l = 0;
    reg in_label_l = 0;
    reg [15:0] label_color_l = 0, base_color_l = BLACK;
    reg de_l = 0, hs_l = 1, vs_l = 1;
    // 10px glyph plus 2px spacing; only snapshot values feed this stage.
    wire [3:0] char_ix = label_dx_l/11'd12;
    wire [3:0] glyph_x_now = (label_dx_l%11'd12)>>1;
    wire [3:0] glyph_y_now = label_dy_l[3:1];
    wire [7:0] character = label_text_l[127-(char_ix*8) -: 8];
    assign byte_text_read_index={robot_label_row_l,char_ix};
    assign robot_byte_read_en=BYTE_TEXT_MODE && run && in_label_l &&
        robot_control_l && robot_label_source_l==3'd1;
    assign pid_byte_read_en=BYTE_TEXT_MODE && run && in_label_l &&
        robot_control_l && robot_label_source_l==3'd2;
    assign sensor_byte_read_en=BYTE_TEXT_MODE && run && in_label_l &&
        robot_control_l && robot_label_source_l==3'd3;
    // Frame-boundary mailboxes remain stable throughout the visible raster.
    // Select ONE dynamic byte in the character stage; do not first select a
    // 128-bit row through every layout branch. Concatenated MSBs avoid wide
    // subtract/multiply index arithmetic and preserve left-to-right bytes.
    wire [10:0] robot_byte_msb={~robot_label_row_l,~char_ix,3'b111};
    wire [9:0] sensor_byte_msb={~robot_label_row_l[2:0],~char_ix,3'b111};
    wire [9:0] battery_byte_msb={3'd4-robot_label_row_l[2:0],~char_ix,3'b111};
    wire [639:0] robot_battery_rows=battery_status[1919:1280];
    reg [7:0] robot_dynamic_character;
    always @* begin
        case(robot_label_source_l)
            3'd1:robot_dynamic_character=BYTE_TEXT_MODE ? 8'h20 : robot_status[robot_byte_msb-:8];
            3'd2:robot_dynamic_character=BYTE_TEXT_MODE ? 8'h20 : pid_rows[robot_byte_msb-:8];
            3'd3:robot_dynamic_character=BYTE_TEXT_MODE ? 8'h20 : sensor_rows[sensor_byte_msb-:8];
            3'd4:robot_dynamic_character=robot_battery_rows[battery_byte_msb-:8];
            default:robot_dynamic_character=8'h20;
        endcase
    end
    wire [7:0] robot_constant_character_a;
    robot_label_rom u_robot_labels (
        .clk(clk),.label_id(robot_label_literal_l),.character_index(char_ix),
        .character(robot_constant_character_a)
    );

    // Four registered stages: layout/background -> character -> glyph -> RGB.
    // Capture the FWFT camera pixel in the FIRST layout stage, on camera_pop's
    // original edge. RGB/DE/HS/VS all gain one pixel clock of latency; raster
    // periods, fetch/pop timing, frame_restart, PLL and output edge stay intact.
    reg [7:0] character_a = 0;
    reg robot_constant_a = 0;
    reg [1:0] robot_byte_source_a = 0;
    wire [7:0] glyph_dynamic_character_a=robot_byte_source_a==2'd1 ? robot_byte_character :
        robot_byte_source_a==2'd2 ? pid_byte_character :
        robot_byte_source_a==2'd3 ? sensor_byte_character : character_a;
    wire [7:0] glyph_character_a=robot_constant_a ? robot_constant_character_a : glyph_dynamic_character_a;
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
    wire pid_left_button=ax>=11'd408 && ax<11'd596;
    wire pid_right_button=ax>=11'd608 && ax<11'd796;
    wire pid_button_row=(ay>=72 && ay<116) || (ay>=128 && ay<172) ||
        (ay>=184 && ay<228) || (ay>=240 && ay<284) ||
        (ay>=296 && ay<340) || (ay>=352 && ay<396) || (ay>=408 && ay<452);
    always @* begin
        base_color=BACKGROUND;
        if (active) begin
            if (ay<10'd44) base_color=WHITE;
            if (ay==10'd44 || ay==10'd546) base_color=BORDER;
            if (in_rect(ax,11'd12,11'd820,ay,10'd52,10'd540)) base_color=BORDER;
            if (video_area) base_color=display_valid ? display_pixel : BLACK;
            if (in_rect(ax,11'd828,11'd1012,ay,10'd52,10'd108)) base_color=BORDER;
            if (in_rect(ax,11'd829,11'd1011,ay,10'd53,10'd107)) base_color=WHITE;
            if (in_rect(ax,11'd832,11'd1008,ay,10'd112,10'd152)) base_color=BORDER;
            if (in_rect(ax,11'd833,11'd1007,ay,10'd113,10'd151)) base_color=pid_page_display ? BLUE : SOFT_BLUE;
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
            // Keep consuming the camera stream with exactly the same zoom
            // geometry; only the final LCD colors are covered by this page.
            // Camera/DDR clocks continue; the top separately gates UDP video.
            if (pid_page_display && video_area) begin
                base_color=BACKGROUND;
                if (in_rect(ax,11'd28,11'd388,ay,10'd64,10'd464)) base_color=BORDER;
                if (in_rect(ax,11'd29,11'd387,ay,10'd65,10'd463)) base_color=WHITE;
                if ((pid_left_button || pid_right_button) && pid_button_row) begin
                    base_color=SOFT_BLUE;
                    if (ay>=352 && ay<396) base_color=pid_left_button ? BLUE : GREEN;
                    if (pid_right_button && ay>=408 && ay<452) base_color=RED;
                end
                if (in_rect(ax,11'd408,11'd796,ay,10'd464,10'd508)) base_color=SOFT_BLUE;
            end
        end
        if (actuator_test) base_color=test_base_color;
        if(robot_control && robot_override_base)base_color=robot_base_color;
    end

    always @(posedge clk or negedge run) begin
        if (!run) begin
            h<=0; v<=0; rgb<=BLACK;
            de<=0; hs<=1; vs<=1;
            frame_div<=0; frame_heartbeat<=0;
            label_text_l<=0; label_dx_l<=0; label_dy_l<=0;
            robot_control_l<=0;robot_label_source_l<=0;robot_label_row_l<=0;robot_label_literal_l<=0;
            in_label_l<=0; label_color_l<=0; base_color_l<=BLACK;
            de_l<=0; hs_l<=1; vs_l<=1;
            character_a<=0;robot_constant_a<=0;robot_byte_source_a<=0;font_b<=0;
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
            label_text_l<=label_text; label_dx_l<=dx; label_dy_l<=dy[3:0];
            robot_control_l<=robot_control;robot_label_source_l<=robot_label_source;
            robot_label_row_l<=robot_label_row;robot_label_literal_l<=robot_label_literal;
            in_label_l<=active && in_label;
            label_color_l<=label_color; base_color_l<=base_color;
            de_l<=active; hs_l<=(h>=H_SYNC); vs_l<=(v>=V_SYNC);

            character_a<=robot_control_l ? robot_dynamic_character : character;
            robot_constant_a<=robot_control_l && robot_label_source_l==3'd0;
            // The external byte RAMs clock their Q on this SAME A edge.
            // Register only the selector here: registering their Q again
            // would add an unwanted fifth pixel stage.
            robot_byte_source_a<=BYTE_TEXT_MODE && robot_control_l ?
                (robot_label_source_l==3'd1 ? 2'd1 :
                 robot_label_source_l==3'd2 ? 2'd2 :
                 robot_label_source_l==3'd3 ? 2'd3 : 2'd0) : 2'd0;
            glyph_x_a<=glyph_x_now; glyph_y_a<=glyph_y_now;
            in_label_a<=in_label_l;
            label_color_a<=label_color_l;
            base_color_a<=base_color_l;
            de_a<=de_l; hs_a<=hs_l; vs_a<=vs_l;

            font_b<=glyph(glyph_character_a);
            glyph_x_b<=glyph_x_a; glyph_y_b<=glyph_y_a;
            in_label_b<=in_label_a; label_color_b<=label_color_a;
            base_color_b<=base_color_a;
            de_b<=de_a; hs_b<=hs_a; vs_b<=vs_a;

            de<=de_b; hs<=hs_b; vs<=vs_b;
            rgb<=text_pixel ? label_color_b : base_color_b;
        end
    end
endmodule

// One ordered 256-byte text snapshot, with independent physical DPBs for
// the active LCD bank and the producer's inactive bank. The source never
// starts a new frame while a published bank awaits a frame-boundary ACK.
// This is ownership isolation, not a promise of old/new collision data:
// actual DPB CEA/CEB stay off for every bank owned by the other clock.
//
// reset_n MUST assert in BOTH domains together (top uses ready && pixel PLL
// lock). read_run only pauses reads/commits; it never resets bank ownership.
// Neither RAM contents nor their output registers require a reset. Until a
// complete frame has committed, read_seen=0 and the visible byte is SPACE.
module lcd_text_byte_snapshot (
    input wire write_clk,reset_n,
    input wire byte_valid,
    input wire [7:0] byte_data,byte_index,
    output wire byte_ready,
    input wire frame_commit,
    input wire [31:0] frame_meta,
    input wire read_clk,read_run,frame_restart,read_en,
    input wire [7:0] read_index,
    output wire [7:0] read_byte,
    output reg [31:0] read_meta,
    output reg read_seen
);
    reg write_bank,publish_pending,frame_open,frame_complete;
    reg [7:0] expected_index;
    reg snapshot_request,snapshot_bank;
    reg [31:0] snapshot_meta;
    reg [2:0] ack_sync;
    reg [2:0] request_sync;
    reg snapshot_ack,active_bank,read_bank_q;

    // State-only permission is essential to the top's atomic UART/LCD join:
    // UART valid is gated by this permission, never by its own grant/ready.
    // Missing/invalid indices are consumed and discarded, so a reset during
    // an ongoing producer frame can recover at its next byte zero.
    assign byte_ready=reset_n && !publish_pending && !frame_complete;
    wire byte_accept=byte_valid && byte_ready;
    wire ordered_byte=byte_index==8'd0 || (frame_open && byte_index==expected_index);
    wire byte_write=byte_accept && ordered_byte;

    always @(posedge write_clk or negedge reset_n) begin
        if(!reset_n)begin
            write_bank<=1'b1;publish_pending<=0;frame_open<=0;frame_complete<=0;
            expected_index<=0;snapshot_request<=0;snapshot_bank<=0;snapshot_meta<=0;ack_sync<=0;
        end else begin
            ack_sync<={ack_sync[1:0],snapshot_ack};
            if(publish_pending && snapshot_request==ack_sync[2])begin
                publish_pending<=0;write_bank<=~write_bank;
            end
            if(byte_accept)begin
                if(byte_index==8'd0)begin
                    frame_open<=1;expected_index<=8'd1;
                end else if(frame_open && byte_index==expected_index)begin
                    if(byte_index==8'd255)begin
                        frame_open<=0;frame_complete<=1;
                    end else expected_index<=expected_index+1'b1;
                end else begin
                    // No RAM write on malformed/incomplete frames: an idle
                    // write_bank may refer to a previous completed snapshot.
                    frame_open<=0;
                end
            end
            // The formatter raises text_valid only AFTER accepted byte255.
            // Its matching flags are updated at COMMIT, so publishing here
            // keeps the page/title and all characters in the same epoch.
            if(frame_commit && frame_complete && !publish_pending)begin
                snapshot_bank<=write_bank;snapshot_meta<=frame_meta;
                snapshot_request<=~snapshot_request;publish_pending<=1;
                frame_complete<=0;
            end
        end
    end

    always @(posedge read_clk or negedge reset_n) begin
        if(!reset_n)begin
            request_sync<=0;snapshot_ack<=0;active_bank<=0;read_bank_q<=0;
            read_seen<=0;read_meta<=0;
        end else begin
            request_sync<={request_sync[1:0],snapshot_request};
            if(read_run && frame_restart && request_sync[2]!=snapshot_ack)begin
                active_bank<=snapshot_bank;read_meta<=snapshot_meta;
                read_seen<=1;snapshot_ack<=request_sync[2];
            end
            if(read_run && read_seen && read_en)read_bank_q<=active_bank;
        end
    end

    wire [7:0] bank0_q,bank1_q;
    lcd_text_snapshot_bank u_bank0 (
        .write_clk(write_clk),.read_clk(read_clk),.reset_n(reset_n),
        .write_enable(byte_write && !write_bank),
        .read_enable(read_run && read_seen && read_en && !active_bank),
        .write_index(byte_index),.read_index(read_index),.write_data(byte_data),.read_data(bank0_q)
    );
    lcd_text_snapshot_bank u_bank1 (
        .write_clk(write_clk),.read_clk(read_clk),.reset_n(reset_n),
        .write_enable(byte_write && write_bank),
        .read_enable(read_run && read_seen && read_en && active_bank),
        .write_index(byte_index),.read_index(read_index),.write_data(byte_data),.read_data(bank1_q)
    );
    assign read_byte=read_seen && read_run ? (read_bank_q ? bank1_q : bank0_q) : 8'h20;
endmodule

// 256 addressed bytes use the low 256 bytes of each 2KiB DPB. At width8 the
// three low physical address bits are zero; ADA[13:3] is the byte address.
// CEA and CEB enforce physical-bank ownership even for adjacent bytes that
// share the same ADA[13:4] physical row. OCE alone would not isolate a port.
module lcd_text_snapshot_bank (
    input wire write_clk,read_clk,reset_n,write_enable,read_enable,
    input wire [7:0] write_index,read_index,write_data,
    output wire [7:0] read_data
);
    wire [15:0] read_word;
    DPB #(
        .BIT_WIDTH_0(8),.BIT_WIDTH_1(8),
        .WRITE_MODE0(2'b00),.WRITE_MODE1(2'b00),
        .READ_MODE0(1'b0),.READ_MODE1(1'b0),
        .BLK_SEL_0(3'b000),.BLK_SEL_1(3'b000),.RESET_MODE("SYNC")
    ) u_ram (
        .DOA(),.DOB(read_word),.DIA({8'd0,write_data}),.DIB(16'd0),
        .BLKSELA(3'b000),.BLKSELB(3'b000),
        .ADA({3'b000,write_index,3'b000}),.ADB({3'b000,read_index,3'b000}),
        .WREA(1'b1),.WREB(1'b0),.CLKA(write_clk),.CLKB(read_clk),
        .CEA(reset_n && write_enable),.CEB(reset_n && read_enable),
        .OCEA(1'b1),.OCEB(1'b1),.RESETA(1'b0),.RESETB(1'b0)
    );
    assign read_data=read_word[7:0];
endmodule
