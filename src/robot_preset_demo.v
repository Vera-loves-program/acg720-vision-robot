// Explicitly labelled scripted demonstration. No visual detections are used or
// fabricated. Distances are time based; gyro turns use the existing navigator.
module robot_preset_demo #(
    parameter [8:0] START_TICKS=9'd500,
    parameter [15:0] DRIVE1_TICKS=16'd300, DRIVE2_TICKS=16'd250,
    parameter [15:0] DRIVE3_TICKS=16'd300, SEARCH_DRIVE_TICKS=16'd250,
    parameter [15:0] PAUSE_TICKS=16'd150,
    parameter signed [15:0] DRIVE_SPEED=16'sd10,
    parameter [15:0] TURN_TIMEOUT_TICKS=16'd6000
)(
    input wire clk,reset_n,tick_10ms,start,search_mode,cancel,ready,
    input wire signed [31:0] heading_mdeg,
    input wire nav_run,turn_done,
    input wire [3:0] nav_status,
    input wire [7:0] nav_fault,
    output wire engaged,moving_phase,
    output reg [3:0] phase,
    output reg [2:0] seconds_left,
    output reg command_valid,
    output reg [2:0] command,
    output wire signed [15:0] speed,
    output reg signed [31:0] turn_heading,
    output wire servo_enable,
    output wire [11:0] pan_us,tilt_us
);
    localparam IDLE=4'd0, DELAY=4'd1, WAIT_READY=4'd2,
        DRIVE1=4'd3, PAUSE1=4'd4, TURN1=4'd5, PAUSE2=4'd6,
        DRIVE2=4'd7, PAUSE3=4'd8, TURN2=4'd9, PAUSE4=4'd10,
        DRIVE3=4'd11, DONE=4'd12, FAULT=4'd13, FINE=4'd14, PAUSE5=4'd15;
    reg mode;
    reg [8:0] delay_ticks;
    reg [15:0] elapsed,turn_ticks;
    reg turn_busy_seen,turn_finished;
    // Navigator headings lie within +/-180000mdeg; even a +150deg target
    // fits signed20 bits. Separate addition, wrap and publication edges to
    // avoid a32-bit add/compare/subtract chain on the50MHz control clock.
    reg signed [19:0] initial_heading,raw_turn_heading;
    wire signed [19:0] current_heading={{1{heading_mdeg[18]}},heading_mdeg[18:0]};
    reg [1:0] turn_setup;
    reg [5:0] head_ticks;
    reg [4:0] head_offset;
    reg head_down;
    assign engaged=phase!=IDLE;
    assign moving_phase=phase==DRIVE1 || phase==DRIVE2 || phase==DRIVE3 ||
        phase==TURN1 || phase==TURN2 || phase==FINE;
    wire turning=phase==TURN1 || phase==TURN2 || phase==FINE;
    wire live=phase!=IDLE && phase!=DELAY && phase!=WAIT_READY && phase!=DONE && phase!=FAULT;
    assign speed=DRIVE_SPEED;
    // Small, slow, explicitly scripted head sweep, not a simulated tracker.
    // Values lie inside the measured PAN380..2550 / TILT645..1555 windows.
    assign servo_enable=live && ready && nav_fault==0 && nav_status!=7 && nav_status!=8 && !cancel;
    assign pan_us=12'd1488+{7'd0,head_offset};
    assign tilt_us=12'd1076;

    function signed [19:0] wrapped;
        input signed [19:0] angle;
        begin
            if(angle>20'sd180000)wrapped=angle-20'sd360000;
            else if(angle< -20'sd180000)wrapped=angle+20'sd360000;
            else wrapped=angle;
        end
    endfunction
    task stop_command;
        begin command_valid<=1;command<=0;end
    endtask
    task begin_drive;
        input [3:0] next_phase;
        begin phase<=next_phase;elapsed<=0;turn_setup<=0;command_valid<=1;command<=1;end
    endtask
    task begin_turn;
        input [3:0] next_phase;
        input signed [19:0] target;
        begin
            phase<=next_phase;elapsed<=0;turn_ticks<=0;
            turn_busy_seen<=0;turn_finished<=0;
            raw_turn_heading<=target;turn_setup<=1;
        end
    endtask
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            phase<=IDLE;mode<=0;delay_ticks<=0;elapsed<=0;turn_ticks<=0;
            seconds_left<=0;command_valid<=0;command<=0;turn_heading<=0;
            turn_busy_seen<=0;turn_finished<=0;initial_heading<=0;
            raw_turn_heading<=0;turn_setup<=0;
            head_ticks<=0;head_offset<=12;head_down<=0;
        end else begin
            command_valid<=0;
            // STOP/page exit/fault always wins over START on the same clock.
            if(cancel)begin
                if(engaged)stop_command;
                phase<=IDLE;seconds_left<=0;turn_setup<=0;
            end else if(start)begin
                phase<=DELAY;mode<=search_mode;delay_ticks<=START_TICKS;
                seconds_left<=5;elapsed<=0;turn_ticks<=0;
                turn_busy_seen<=0;turn_finished<=0;
                turn_setup<=0;
                head_ticks<=0;head_offset<=12;head_down<=0;
                stop_command;
            end else if(live && (!ready || nav_fault!=0 || nav_status==9))begin
                phase<=FAULT;turn_setup<=0;stop_command;
            end else begin
                if(turn_setup==1)begin turn_heading<=wrapped(raw_turn_heading);turn_setup<=2;end
                else if(turn_setup==2)begin command_valid<=1;command<=2;turn_setup<=0;end
                // The navigator's completion is a one-clock pulse, unrelated
                // to the10ms tick. Never miss it or reuse an earlier turn.
                if(turning && nav_status==4)turn_busy_seen<=1;
                if(turning && turn_busy_seen && turn_done)turn_finished<=1;
                if(tick_10ms)begin
                    if(servo_enable)begin
                        if(head_ticks==49)begin
                            head_ticks<=0;
                            if(head_offset==24)begin head_down<=1;head_offset<=23;end
                            else if(head_offset==0)begin head_down<=0;head_offset<=1;end
                            else head_offset<=head_down ? head_offset-1'b1 : head_offset+1'b1;
                        end else head_ticks<=head_ticks+1'b1;
                    end
                    if(turning)begin
                        if(turn_ticks>=TURN_TIMEOUT_TICKS-1'b1)begin phase<=FAULT;stop_command;end
                        else begin
                            turn_ticks<=turn_ticks+1'b1;
                            if(turn_finished)begin
                                phase<=phase==TURN1 ? PAUSE2 : phase==TURN2 ? PAUSE4 : PAUSE5;
                                elapsed<=0;stop_command;
                            end
                        end
                    end else case(phase)
                        DELAY:begin
                            if(delay_ticks<=1)begin phase<=WAIT_READY;seconds_left<=0;end
                            else begin
                                delay_ticks<=delay_ticks-1'b1;
                                seconds_left<=delay_ticks>401 ? 5 : delay_ticks>301 ? 4 :
                                    delay_ticks>201 ? 3 : delay_ticks>101 ? 2 : 1;
                            end
                        end
                        WAIT_READY:if(ready && nav_fault==0)begin
                            initial_heading<=current_heading;
                            if(mode)begin_turn(TURN1,current_heading+20'sd150000);
                            else begin_drive(DRIVE1);
                        end
                        DRIVE1,DRIVE2,DRIVE3:begin
                            // Time advances only when the guarded navigator
                            // actually permits motion. An obstacle pauses it.
                            if(nav_run && nav_status==3)begin
                                if(elapsed>=(phase==DRIVE1 ? DRIVE1_TICKS :
                                    phase==DRIVE2 ? DRIVE2_TICKS : mode ? SEARCH_DRIVE_TICKS : DRIVE3_TICKS)-1'b1)begin
                                    phase<=phase==DRIVE1 ? PAUSE1 : phase==DRIVE2 ? PAUSE3 : DONE;
                                    elapsed<=0;stop_command;
                                end else elapsed<=elapsed+1'b1;
                            end
                        end
                        PAUSE1,PAUSE2,PAUSE3,PAUSE4,PAUSE5:begin
                            if(elapsed>=PAUSE_TICKS-1'b1)begin
                                if(phase==PAUSE1)begin_turn(TURN1,initial_heading+20'sd30000);
                                else if(phase==PAUSE2)begin
                                    if(mode)begin_turn(TURN2,current_heading+20'sd150000);
                                    else begin_drive(DRIVE2);
                                end else if(phase==PAUSE3)begin_turn(TURN2,initial_heading);
                                else if(phase==PAUSE4)begin
                                    if(mode)begin_turn(FINE,current_heading-20'sd8000);
                                    else begin_drive(DRIVE3);
                                end else begin_drive(DRIVE3);
                            end else elapsed<=elapsed+1'b1;
                        end
                        default:begin end
                    endcase
                end
            end
        end
    end
endmodule
