// Atomic command/data register boundary. Live sensor checks stay in the
// navigator; a hard cancellation suppresses queued motion immediately.
module robot_navigation_command_stage(
    input wire clk,reset_n,cancel,
    input wire in_enable,in_valid,in_hold,in_avoid,
    input wire [2:0] in_command,
    input wire [1:0] in_half_direction,
    input wire signed [15:0] in_left,in_right,
    input wire signed [31:0] in_heading,in_goal_x,in_goal_y,
    input wire [15:0] in_cruise,in_turn,in_tolerance,
    output wire enable,valid,
    output reg hold_heading,avoid,
    output reg [2:0] command,
    output reg [1:0] half_direction,
    output reg signed [15:0] left_speed,right_speed,
    output reg signed [31:0] heading,goal_x,goal_y,
    output reg [15:0] cruise,turn_speed,tolerance
);
    reg enable_q,valid_q;
    assign enable=enable_q && !cancel;
    // A queued STOP remains legal during cancellation (including CLEAR STOP),
    // so the existing navigator's latched fault reset semantics are preserved.
    assign valid=valid_q && (!cancel || command==0);
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            enable_q<=0;valid_q<=0;command<=0;half_direction<=0;
            hold_heading<=0;avoid<=0;left_speed<=0;right_speed<=0;
            heading<=0;goal_x<=0;goal_y<=0;cruise<=0;turn_speed<=0;tolerance<=0;
        end else begin
            enable_q<=in_enable && !cancel;
            valid_q<=in_valid && (!cancel || in_command==0);
            command<=in_command;half_direction<=in_half_direction;
            hold_heading<=in_hold;avoid<=in_avoid;
            left_speed<=in_left;right_speed<=in_right;
            heading<=in_heading;goal_x<=in_goal_x;goal_y<=in_goal_y;
            cruise<=in_cruise;turn_speed<=in_turn;tolerance<=in_tolerance;
        end
    end
endmodule
