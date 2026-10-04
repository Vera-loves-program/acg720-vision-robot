// R5: source-anchored pinch and independent zoom/target modes. No motor output.
// Q8 inverse scale 256..64 = 1x..4x; variable division takes 24 control clocks.
module touch_gestures #(parameter TAP_MAX_MS=600, DOUBLE_TAP_MAX_MS=450)(
    input wire clk, reset_n, touch_ready, snapshot_valid, snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0, y0, x1, y1,
    input wire [3:0] id0, id1,
    input wire [8:0] view_step,
    input wire [9:0] view_left,
    input wire [8:0] view_top,
    input wire debug_toggle_pulse, clear_pulse, zoom_reset_pulse,
    output reg debug_mode, selection_valid,
    output reg [9:0] selection_x, selection_y,
    output reg selection_toggle, zoom_mode,
    output reg [8:0] zoom_step,
    output reg [9:0] zoom_left,
    output reg [8:0] zoom_top,
    output reg [9:0] focus_x,
    output reg [8:0] focus_y,
    output reg zoom_toggle, capture_toggle, clear_toggle,
    output reg [3:0] click_event
);
    reg [15:0] ms_divider, down_age, tap_age;
    wire ms_tick=ms_divider==16'd49999;
    reg [1:0] previous_count;
    reg [10:0] down_x, down_y, previous_tap_x, previous_tap_y;
    reg [3:0] down_id;
    reg [8:0] down_step, down_top;
    reg [9:0] down_left;
    reg drag_or_multi, tap_pending;
    wire [10:0] move_dx=x0>=down_x ? x0-down_x : down_x-x0;
    wire [10:0] move_dy=y0>=down_y ? y0-down_y : down_y-y0;
    wire [10:0] tap_dx=down_x>=previous_tap_x ? down_x-previous_tap_x : previous_tap_x-down_x;
    wire [10:0] tap_dy=down_y>=previous_tap_y ? down_y-previous_tap_y : previous_tap_y-down_y;
    wire in_video0=x0>=16 && x0<816 && y0>=56 && y0<536;
    wire in_video1=x1>=16 && x1<816 && y1>=56 && y1<536;
    wire down_in_video=down_x>=16 && down_x<816 && down_y>=56 && down_y<536;
    wire down_in_sidebar=down_x>=832 && down_x<1008;
    wire [10:0] down_vx=down_x-11'd16, down_vy=down_y-11'd56;
    wire [19:0] down_scaled_x=down_vx*down_step, down_scaled_y=down_vy*down_step;
    wire [9:0] down_camera_x=down_left+down_scaled_x[17:8];
    wire [9:0] down_camera_y={1'b0,down_top}+{1'b0,down_scaled_y[16:8]};
    wire [10:0] selected_dx=down_camera_x>=selection_x ? down_camera_x-selection_x : selection_x-down_camera_x;
    wire [10:0] selected_dy=down_camera_y>=selection_y ? down_camera_y-selection_y : selection_y-down_camera_y;
    wire [10:0] span_dx=x0>=x1 ? x0-x1 : x1-x0, span_dy=y0>=y1 ? y0-y1 : y1-y0;
    // L1 span: keep finger orientation steady during a pinch.
    wire [11:0] span={1'b0,span_dx}+{1'b0,span_dy};
    wire [11:0] sum_x={1'b0,x0}+{1'b0,x1}, sum_y={1'b0,y0}+{1'b0,y1};
    wire [9:0] mid_x=sum_x[10:1]-10'd16;
    wire [9:0] mid_y_wide=sum_y[10:1]-10'd56;
    wire [8:0] mid_y=mid_y_wide[8:0];
    reg pinch_active;
    reg [3:0] pinch_id0, pinch_id1;
    reg [11:0] pinch_span, last_span;
    reg [9:0] last_mid_x;
    reg [8:0] last_mid_y;
    reg [8:0] pinch_step;
    reg [23:0] pinch_anchor_x, pinch_anchor_y;
    wire same_pair=(id0==pinch_id0 && id1==pinch_id1) || (id0==pinch_id1 && id1==pinch_id0);
    wire [11:0] span_change=span>=last_span ? span-last_span : last_span-span;
    wire [9:0] mid_dx=mid_x>=last_mid_x ? mid_x-last_mid_x : last_mid_x-mid_x;
    wire [8:0] mid_dy=mid_y>=last_mid_y ? mid_y-last_mid_y : last_mid_y-mid_y;
    wire abort_calculation=!touch_ready || (snapshot_valid &&
        (snapshot_cancel || (pinch_active && (contact_count!=2 || !same_pair || !in_video0 || !in_video1 || span<8))));
    wire [19:0] mid_scaled_x=mid_x*view_step;
    wire [17:0] mid_scaled_y=mid_y*view_step;
    localparam CALC_IDLE=0, CALC_DIVIDE=1, CALC_APPLY=2;
    reg [1:0] calc_state;
    reg [4:0] divide_count;
    reg [23:0] dividend, quotient;
    reg [12:0] remainder;
    reg [11:0] divisor;
    reg [8:0] apply_step;
    reg [9:0] apply_mid_x;
    reg [8:0] apply_mid_y;
    reg [23:0] apply_anchor_x, apply_anchor_y;
    wire [13:0] trial={remainder,dividend[23]};
    wire divide_bit=trial>={2'b00,divisor};
    wire [23:0] next_quotient={quotient[22:0],divide_bit};
    wire [13:0] next_remainder=divide_bit ? trial-{2'b00,divisor} : trial;
    wire [19:0] apply_product_x=apply_mid_x*apply_step;
    wire [17:0] apply_product_y=apply_mid_y*apply_step;
    wire signed [24:0] desired_x=$signed({1'b0,apply_anchor_x})-$signed({5'd0,apply_product_x});
    wire signed [24:0] desired_y=$signed({1'b0,apply_anchor_y})-$signed({7'd0,apply_product_y});
    wire [18:0] width_q8=apply_step*10'd800+19'd255;
    wire [17:0] height_q8=apply_step*9'd480+18'd255;
    wire [9:0] max_left=10'd800-width_q8[17:8];
    wire [8:0] max_top=9'd480-height_q8[16:8];
    wire [12:0] plus_step=(zoom_step*4'd7)>>3;
    wire [12:0] minus_step=(zoom_step*4'd9+13'd7)>>3;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ms_divider<=0; down_age<=0; tap_age<=16'hffff; previous_count<=0;
            down_x<=0; down_y<=0; down_id<=0; down_step<=256; down_left<=0; down_top<=0;
            previous_tap_x<=0; previous_tap_y<=0; drag_or_multi<=0; tap_pending<=0;
            pinch_active<=0; pinch_id0<=0; pinch_id1<=0; pinch_span<=0; last_span<=0;
            last_mid_x<=0; last_mid_y<=0;
            pinch_step<=256; pinch_anchor_x<=0; pinch_anchor_y<=0;
            debug_mode<=0; selection_valid<=0; selection_x<=0; selection_y<=0; selection_toggle<=0;
            zoom_mode<=0; zoom_step<=256; zoom_left<=0; zoom_top<=0; focus_x<=400; focus_y<=240;
            zoom_toggle<=0; capture_toggle<=0; clear_toggle<=0; click_event<=0;
            calc_state<=CALC_IDLE; divide_count<=0; dividend<=0; quotient<=0; remainder<=0; divisor<=1;
            apply_step<=256; apply_mid_x<=0; apply_mid_y<=0; apply_anchor_x<=0; apply_anchor_y<=0;
        end else begin
            click_event<=0;
            if (ms_tick) begin
                ms_divider<=0;
                if (down_age!=16'hffff) down_age<=down_age+1'b1;
                if (tap_age!=16'hffff) tap_age<=tap_age+1'b1;
            end else ms_divider<=ms_divider+1'b1;
            if (calc_state==CALC_DIVIDE && !abort_calculation) begin
                dividend<={dividend[22:0],1'b0}; quotient<=next_quotient; remainder<=next_remainder[12:0];
                if (divide_count==23) begin
                    apply_step<=next_quotient<64 ? 9'd64 : next_quotient>256 ? 9'd256 : next_quotient[8:0];
                    calc_state<=CALC_APPLY;
                end else divide_count<=divide_count+1'b1;
            end else if (calc_state==CALC_APPLY && !abort_calculation) begin
                zoom_step<=apply_step;
                zoom_left<=desired_x<0 ? 10'd0 : (desired_x>>>8)>$signed({15'd0,max_left}) ? max_left : desired_x[17:8];
                zoom_top<=desired_y<0 ? 9'd0 : (desired_y>>>8)>$signed({16'd0,max_top}) ? max_top : desired_y[16:8];
                zoom_toggle<=!zoom_toggle; calc_state<=CALC_IDLE;
            end
            if (!touch_ready || (snapshot_valid && snapshot_cancel)) begin
                previous_count<=0; drag_or_multi<=1; tap_pending<=0; pinch_active<=0;
                down_age<=16'hffff; calc_state<=CALC_IDLE;
            end else if (snapshot_valid) begin
                previous_count<=contact_count;
                if (contact_count==2) begin
                    drag_or_multi<=1; tap_pending<=0;
                    if (in_video0 && in_video1 && id0!=id1 &&
                        (span>=32 || (pinch_active && same_pair && span>=8))) begin
                        if (!pinch_active || !same_pair) begin
                            pinch_active<=1; pinch_span<=span; last_span<=span;
                            last_mid_x<=mid_x; last_mid_y<=mid_y;
                            pinch_step<=view_step; pinch_id0<=id0; pinch_id1<=id1;
                            pinch_anchor_x<={6'd0,view_left,8'd0}+{4'd0,mid_scaled_x};
                            pinch_anchor_y<={7'd0,view_top,8'd0}+{6'd0,mid_scaled_y};
                            calc_state<=CALC_IDLE;
                        end else if (calc_state==CALC_IDLE && (span_change>=2 || mid_dx>=2 || mid_dy>=2)) begin
                            dividend<=pinch_step*pinch_span; divisor<=span; quotient<=0; remainder<=0; divide_count<=0;
                            apply_anchor_x<=pinch_anchor_x; apply_anchor_y<=pinch_anchor_y;
                            apply_mid_x<=mid_x; apply_mid_y<=mid_y; calc_state<=CALC_DIVIDE; last_span<=span;
                            last_mid_x<=mid_x; last_mid_y<=mid_y;
                        end
                    end else begin pinch_active<=0; calc_state<=CALC_IDLE; end
                end else begin
                    pinch_active<=0;
                    if (previous_count==2) calc_state<=CALC_IDLE;
                    if (contact_count==1 && previous_count==0) begin
                        down_x<=x0; down_y<=y0; down_id<=id0;
                        down_step<=view_step; down_left<=view_left; down_top<=view_top;
                        down_age<=0; drag_or_multi<=0;
                    end else if (contact_count==1) begin
                        if (previous_count==2 || id0!=down_id || move_dx>16 || move_dy>16) drag_or_multi<=1;
                    end else if (contact_count==0 && previous_count==1 && !drag_or_multi && down_age<=TAP_MAX_MS) begin
                        if (down_in_sidebar && down_y>=112 && down_y<152) begin
                            debug_mode<=!debug_mode; zoom_mode<=0; tap_pending<=0; click_event<=1;
                        end else if (down_in_sidebar && down_y>=160 && down_y<200) begin
                            zoom_mode<=!zoom_mode; tap_pending<=0; calc_state<=CALC_IDLE; click_event<=5;
                        end else if (down_in_sidebar && down_y>=208 && down_y<240 && zoom_mode &&
                                     (down_x<916 || down_x>=924)) begin
                            apply_step<=down_x<920 ? (plus_step<64 ? 9'd64 : plus_step[8:0]) : (minus_step>256 ? 9'd256 : minus_step[8:0]);
                            apply_anchor_x<={6'd0,focus_x,8'd0}; apply_anchor_y<={7'd0,focus_y,8'd0};
                            apply_mid_x<=400; apply_mid_y<=240; calc_state<=CALC_APPLY; tap_pending<=0; click_event<=6;
                        end else if (down_in_sidebar && down_y>=392 && down_y<432) begin
                            zoom_step<=256; zoom_left<=0; zoom_top<=0; zoom_toggle<=!zoom_toggle;
                            tap_pending<=0; calc_state<=CALC_IDLE; click_event<=2;
                        end else if (down_in_sidebar && down_y>=440 && down_y<488) begin
                            capture_toggle<=!capture_toggle; tap_pending<=0; click_event<=3;
                        end else if (down_in_video && (zoom_mode || debug_mode)) begin
                            if (tap_pending && tap_age<=DOUBLE_TAP_MAX_MS && tap_dx<=32 && tap_dy<=32) begin
                                if (zoom_mode) begin
                                    focus_x<=down_camera_x; focus_y<=down_camera_y[8:0];
                                    apply_anchor_x<={6'd0,down_camera_x,8'd0}; apply_anchor_y<={7'd0,down_camera_y[8:0],8'd0};
                                    apply_mid_x<=400; apply_mid_y<=240; apply_step<=zoom_step; calc_state<=CALC_APPLY; click_event<=7;
                                end else begin
                                    selection_valid<=!(selection_valid && selected_dx<=40 && selected_dy<=30);
                                    selection_x<=down_camera_x; selection_y<=down_camera_y; selection_toggle<=!selection_toggle; click_event<=4;
                                end
                                tap_pending<=0;
                            end else begin
                                previous_tap_x<=down_x; previous_tap_y<=down_y; tap_age<=0; tap_pending<=1;
                            end
                        end else tap_pending<=0;
                    end
                end
            end
            if (debug_toggle_pulse) begin debug_mode<=!debug_mode; zoom_mode<=0; tap_pending<=0; end
            if (clear_pulse) begin
                selection_valid<=0; selection_toggle<=!selection_toggle; clear_toggle<=!clear_toggle;
                zoom_step<=256; zoom_left<=0; zoom_top<=0; zoom_toggle<=!zoom_toggle;
                pinch_active<=0; tap_pending<=0; calc_state<=CALC_IDLE;
            end
            if (zoom_reset_pulse) begin
                zoom_step<=256; zoom_left<=0; zoom_top<=0; zoom_toggle<=!zoom_toggle;
                pinch_active<=0; tap_pending<=0; calc_state<=CALC_IDLE;
            end
        end
    end
endmodule
