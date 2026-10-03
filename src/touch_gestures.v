// Touch intent only: does not identify people and never commands a motor.
// All inputs/outputs belong to clk (50 MHz). Cross-clock bundles need a
// handshake or stable-data toggle mailbox in the integrating top module.
module touch_gestures #(
    parameter DEBUG_Y_MIN=112, DEBUG_Y_MAX=152,
    parameter CLEAR_Y_MIN=392, CLEAR_Y_MAX=432,
    parameter CAPTURE_Y_MIN=440, CAPTURE_Y_MAX=488,
    parameter TAP_MAX_MS=600,
    parameter DOUBLE_TAP_MAX_MS=450
)(
    input wire clk, reset_n, touch_ready, snapshot_valid, snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0, y0, x1, y1,
    input wire [3:0] id0, id1,
    input wire view_zoom_2x,
    input wire debug_toggle_pulse, clear_pulse, zoom_reset_pulse,
    output reg debug_mode,
    output reg selection_valid,
    output reg [9:0] selection_x, selection_y,
    output reg selection_toggle,
    output reg [1:0] zoom_level,
    output reg zoom_toggle,
    output reg capture_toggle,
    output reg clear_toggle,
    output reg [3:0] click_event
);
    reg [15:0] ms_divider;
    wire ms_tick = ms_divider==16'd49999;
    reg [15:0] down_age, tap_age;
    reg [1:0] previous_count;
    reg [10:0] down_x, down_y;
    reg [3:0] down_id;
    reg down_zoom_2x;
    reg drag_or_multi, tap_pending;
    reg [10:0] previous_tap_x, previous_tap_y;
    wire [10:0] move_dx = x0>=down_x ? x0-down_x : down_x-x0;
    wire [10:0] move_dy = y0>=down_y ? y0-down_y : down_y-y0;
    wire [10:0] tap_dx = down_x>=previous_tap_x ?
        down_x-previous_tap_x : previous_tap_x-down_x;
    wire [10:0] tap_dy = down_y>=previous_tap_y ?
        down_y-previous_tap_y : previous_tap_y-down_y;
    wire in_video0 = x0>=11'd16 && x0<11'd816 && y0>=11'd56 && y0<11'd536;
    wire in_video1 = x1>=11'd16 && x1<11'd816 && y1>=11'd56 && y1<11'd536;
    wire down_in_video = down_x>=11'd16 && down_x<11'd816 &&
                         down_y>=11'd56 && down_y<11'd536;
    wire down_in_sidebar = down_x>=11'd832 && down_x<11'd1008;
    wire [10:0] down_video_x = down_x-11'd16;
    wire [10:0] down_video_y = down_y-11'd56;
    // Always publish coordinates in the original 800x480 camera space.
    // Local 2x view shows source [200,600) x [120,360), repeated 2x2.
    wire [9:0] down_camera_x = down_zoom_2x ?
        10'd200+down_video_x[10:1] : down_video_x[9:0];
    wire [9:0] down_camera_y = down_zoom_2x ?
        10'd120+down_video_y[10:1] : down_video_y[9:0];
    wire [10:0] span_dx = x0>=x1 ? x0-x1 : x1-x0;
    wire [10:0] span_dy = y0>=y1 ? y0-y1 : y1-y0;
    wire [11:0] span = {1'b0,span_dx}+{1'b0,span_dy};
    reg pinch_active;
    reg [11:0] pinch_anchor;
    reg [3:0] pinch_id0, pinch_id1;
    wire same_pair = (id0==pinch_id0 && id1==pinch_id1) ||
                     (id0==pinch_id1 && id1==pinch_id0);
    // 25% expansion / 20% contraction and a 24 px minimum displacement.
    // Pinch is a viewing operation and does not depend on target/debug mode.
    // Distinct IDs prevent a malformed pair from masquerading as two fingers.
    wire [14:0] span_times4 = {1'b0,span,2'b00};
    wire [14:0] anchor_times5 = {1'b0,pinch_anchor,2'b00}+{3'b000,pinch_anchor};
    wire [14:0] span_times5 = {1'b0,span,2'b00}+{3'b000,span};
    wire [14:0] anchor_times4 = {1'b0,pinch_anchor,2'b00};

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ms_divider<=0; down_age<=0; tap_age<=16'hffff;
            previous_count<=0; down_x<=0; down_y<=0; down_id<=0;
            down_zoom_2x<=0;
            drag_or_multi<=0; tap_pending<=0; previous_tap_x<=0; previous_tap_y<=0;
            pinch_active<=0; pinch_anchor<=0; pinch_id0<=0; pinch_id1<=0;
            debug_mode<=0; selection_valid<=0; selection_x<=0; selection_y<=0;
            selection_toggle<=0; zoom_level<=0; zoom_toggle<=0;
            capture_toggle<=0; clear_toggle<=0; click_event<=0;
        end else begin
            click_event<=0;
            if (ms_tick) begin
                ms_divider<=0;
                if (down_age!=16'hffff) down_age<=down_age+1'b1;
                if (tap_age!=16'hffff) tap_age<=tap_age+1'b1;
            end else ms_divider<=ms_divider+1'b1;

            if (!touch_ready) begin
                previous_count<=0; drag_or_multi<=1; tap_pending<=0; pinch_active<=0;
            end else if (snapshot_valid && snapshot_cancel) begin
                // A palm/proximity/out-of-range report is not a finger release.
                // Abort both halves of a double tap and never emit a UI event.
                previous_count<=0; drag_or_multi<=1; tap_pending<=0; pinch_active<=0;
                down_age<=16'hffff;
            end else if (snapshot_valid) begin
                previous_count<=contact_count;
                if (contact_count==2) begin
                    drag_or_multi<=1; tap_pending<=0;
                    if (in_video0 && in_video1 && id0!=id1 &&
                        (span>=12'd32 || (pinch_active && same_pair))) begin
                        if (!pinch_active || !same_pair) begin
                            pinch_active<=1; pinch_anchor<=span;
                            pinch_id0<=id0; pinch_id1<=id1;
                        end else if (zoom_level==0 &&
                                     span_times4>=anchor_times5 &&
                                     {1'b0,span}>={1'b0,pinch_anchor}+13'd24) begin
                            zoom_level<=1; zoom_toggle<=!zoom_toggle; pinch_anchor<=span;
                        end else if (zoom_level==1 &&
                                     span_times5<=anchor_times4 &&
                                     {1'b0,span}+13'd24<={1'b0,pinch_anchor}) begin
                            zoom_level<=0; zoom_toggle<=!zoom_toggle; pinch_anchor<=span;
                        end
                    end else pinch_active<=0;
                end else begin
                    pinch_active<=0;
                    if (contact_count==1 && previous_count==0) begin
                        down_x<=x0; down_y<=y0; down_id<=id0;
                        // Capture the view actually displayed at finger-down,
                        // not the request still travelling to the LCD clock.
                        down_zoom_2x<=view_zoom_2x;
                        down_age<=0; drag_or_multi<=0;
                    end else if (contact_count==1) begin
                        if (previous_count==2 || id0!=down_id || move_dx>16 || move_dy>16)
                            drag_or_multi<=1;
                    end else if (contact_count==0 && previous_count==1 &&
                                 !drag_or_multi && down_age<=TAP_MAX_MS) begin
                        // Commit taps on release; holding does not retrigger.
                        if (down_in_sidebar && down_y>=DEBUG_Y_MIN && down_y<DEBUG_Y_MAX) begin
                            debug_mode<=!debug_mode; tap_pending<=0; click_event<=1;
                        end else if (down_in_sidebar && down_y>=CLEAR_Y_MIN && down_y<CLEAR_Y_MAX) begin
                            selection_valid<=0; selection_toggle<=!selection_toggle;
                            clear_toggle<=!clear_toggle;
                            if (zoom_level!=0) zoom_toggle<=!zoom_toggle;
                            zoom_level<=0; tap_pending<=0; click_event<=2;
                        end else if (down_in_sidebar && down_y>=CAPTURE_Y_MIN && down_y<CAPTURE_Y_MAX) begin
                            capture_toggle<=!capture_toggle; tap_pending<=0; click_event<=3;
                        end else if (debug_mode && down_in_video) begin
                            if (tap_pending && tap_age<=DOUBLE_TAP_MAX_MS && tap_dx<=32 && tap_dy<=32) begin
                                selection_x<=down_camera_x; selection_y<=down_camera_y;
                                selection_valid<=1; selection_toggle<=!selection_toggle;
                                tap_pending<=0; click_event<=4;
                            end else begin
                                previous_tap_x<=down_x; previous_tap_y<=down_y;
                                tap_age<=0; tap_pending<=1;
                            end
                        end else tap_pending<=0;
                    end
                end
            end
            // External commands have priority over touch events on that cycle.
            if (debug_toggle_pulse) begin debug_mode<=!debug_mode; tap_pending<=0; end
            if (clear_pulse) begin
                selection_valid<=0; selection_toggle<=!selection_toggle;
                clear_toggle<=!clear_toggle; pinch_active<=0;
                if (zoom_level!=0) zoom_toggle<=!zoom_toggle;
                zoom_level<=0; tap_pending<=0;
            end
            if (zoom_reset_pulse) begin
                if (zoom_level!=0) zoom_toggle<=!zoom_toggle;
                zoom_level<=0; pinch_active<=0;
            end
        end
    end
endmodule
