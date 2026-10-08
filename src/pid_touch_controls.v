// LCD motor-tuning commands, clocked in the same 50 MHz domain as GT911.
// Non-stop controls require one completed, short, single-finger tap. STOP
// takes priority on contact; cancel, lost touch and page changes also stop.
// This decoder never drives a motor and never repeats a held +/- button.
module pid_touch_controls #(
    parameter CLK_HZ = 50000000,
    parameter TAP_MAX_MS = 600
)(
    input wire clk, reset_n,
    input wire touch_ready, snapshot_valid, snapshot_cancel,
    input wire [1:0] contact_count,
    input wire [10:0] x0, y0, x1, y1,
    input wire [3:0] id0, id1,
    output reg pid_page,
    output reg action_valid,
    output reg [7:0] action,
    output reg emergency_stop_pulse
);
    localparam [7:0] A_NONE=0, A_WHEEL=1, A_LOOP=2,
        A_PREV=3, A_NEXT=4, A_MINUS=5, A_PLUS=6, A_STEP=7,
        A_ARM=8, A_RUN=9, A_STOP=10, A_CLEAR=11,
        A_SPEED=12, A_POSITION=13, A_ZERO=14, A_UNLOCK=15, A_PAGE=16;
    localparam integer MS_DIV = CLK_HZ/1000;
    reg [31:0] ms_divider;
    reg [15:0] down_age;
    reg [1:0] previous_count;
    reg [10:0] down_x, down_y;
    reg [3:0] down_id;
    reg [7:0] down_button;
    reg invalid_tap, previous_ready;
    wire [10:0] move_dx=x0>=down_x ? x0-down_x : down_x-x0;
    wire [10:0] move_dy=y0>=down_y ? y0-down_y : down_y-y0;

    function in_rect;
        input [10:0] x, y, left, right, top, bottom;
        begin in_rect=x>=left && x<right && y>=top && y<bottom; end
    endfunction

    // Keep these rectangles aligned with lcd1024_interaction_ui.v.
    function [7:0] button_at;
        input [10:0] x, y;
        input page;
        reg left_column, right_column;
        begin
            button_at=A_NONE;
            left_column=x>=408 && x<596;
            right_column=x>=608 && x<796;
            if (in_rect(x,y,832,1008,112,152)) button_at=A_PAGE;
            else if (page && in_rect(x,y,408,796,464,508)) button_at=A_UNLOCK;
            else if (page && (left_column || right_column)) begin
                if (y>=72 && y<116) button_at=left_column ? A_WHEEL : A_LOOP;
                else if (y>=128 && y<172) button_at=left_column ? A_PREV : A_NEXT;
                else if (y>=184 && y<228) button_at=left_column ? A_MINUS : A_PLUS;
                else if (y>=240 && y<284) button_at=left_column ? A_STEP : A_ZERO;
                else if (y>=296 && y<340) button_at=left_column ? A_SPEED : A_POSITION;
                else if (y>=352 && y<396) button_at=left_column ? A_ARM : A_RUN;
                else if (y>=408 && y<452) button_at=left_column ? A_CLEAR : A_STOP;
            end
        end
    endfunction

    wire sidebar_stop0=contact_count!=0 && in_rect(x0,y0,832,1008,496,536);
    wire sidebar_stop1=contact_count==2 && in_rect(x1,y1,832,1008,496,536);
    wire panel_stop0=pid_page && contact_count!=0 && in_rect(x0,y0,608,796,408,452);
    wire panel_stop1=pid_page && contact_count==2 && in_rect(x1,y1,608,796,408,452);
    wire [7:0] current_button=button_at(x0,y0,pid_page);

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pid_page<=0; action_valid<=0; action<=A_NONE; emergency_stop_pulse<=0;
            ms_divider<=0; down_age<=16'hffff; previous_count<=0;
            down_x<=0; down_y<=0; down_id<=0; down_button<=A_NONE;
            invalid_tap<=1; previous_ready<=0;
        end else begin
            action_valid<=0;
            emergency_stop_pulse<=0;
            previous_ready<=touch_ready;
            if (ms_divider==MS_DIV-1) begin
                ms_divider<=0;
                if (down_age!=16'hffff) down_age<=down_age+1'b1;
            end else ms_divider<=ms_divider+1'b1;

            if (!touch_ready || (snapshot_valid && snapshot_cancel)) begin
                previous_count<=0; invalid_tap<=1;
                down_age<=16'hffff; down_button<=A_NONE;
                if (pid_page && ((previous_ready && !touch_ready) ||
                    (snapshot_valid && snapshot_cancel))) begin
                    action<=A_STOP; action_valid<=1;
                end
            end else if (snapshot_valid) begin
                previous_count<=contact_count;
                if (contact_count==1 && previous_count==0) begin
                    down_x<=x0; down_y<=y0; down_id<=id0;
                    down_age<=0; down_button<=current_button; invalid_tap<=0;
                end else if (contact_count==2) begin
                    invalid_tap<=1;
                end else if (contact_count==1) begin
                    if (previous_count==2 || id0!=down_id || move_dx>16 || move_dy>16 ||
                        current_button!=down_button) invalid_tap<=1;
                end else if (contact_count==0 && previous_count==1 &&
                    !invalid_tap && down_age<=TAP_MAX_MS && down_button!=A_NONE) begin
                    invalid_tap<=1; down_button<=A_NONE;
                    if (down_button==A_PAGE) begin
                        pid_page<=!pid_page;
                        action<=A_STOP; action_valid<=1;
                    end else if (pid_page && down_button!=A_STOP) begin
                        action<=down_button; action_valid<=1;
                    end
                end
            end

            // Last assignment wins: a STOP cannot be replaced by ARM, RUN,
            // a release event, or a parameter change in the same clock.
            // On a canceled contact snapshot only the conservative STOP
            // path may use the reported coordinates; no start is accepted.
            if (snapshot_valid && touch_ready &&
                (sidebar_stop0 || sidebar_stop1 || panel_stop0 || panel_stop1)) begin
                action<=A_STOP; action_valid<=1;
                emergency_stop_pulse<=sidebar_stop0 || sidebar_stop1;
                invalid_tap<=1; down_button<=A_NONE;
            end
        end
    end
endmodule
