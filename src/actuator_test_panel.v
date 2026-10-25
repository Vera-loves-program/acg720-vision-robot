// Combinational layout only. Uses the existing LCD glyph/RGB pipeline.
// Data/flags have already crossed in a frame-latched handshake mailbox.
module actuator_test_panel (
    input wire [10:0] x,
    input wire [9:0] y,
    input wire [2047:0] rows,
    input wire [15:0] flags,
    input wire [1023:0] sensor_rows,
    input wire [639:0] battery_rows,
    input wire [1:0] battery_valid,
    input wire [1279:0] diagnostic_rows,
    input wire bq_info,
    output reg [127:0] label_text,
    output reg [10:0] label_x,label_right,
    output reg [9:0] label_y,
    output reg [15:0] label_color,base_color,
    output reg label_enable
);
    localparam [15:0] BG=16'hf79e,WHITE=16'hffff,INK=16'h1926,
        MUTED=16'h63b1,BLUE=16'h03df,SOFT=16'he79f,GREEN=16'h13a8,
        RED=16'hd965,BORDER=16'hd6bb,ORANGE=16'hfbe0;
    integer card,top,slot;
    reg enabled,cal,min_ok,max_ok,limits_ok;
    function rect;
        input [10:0] l,r;
        input [9:0] t,b;
        begin rect=x>=l && x<r && y>=t && y<b;end
    endfunction
    // Only one label can cover any pixel. All literals are 16 ASCII bytes.
    task label;
        input [10:0] left,right;
        input [9:0] top_y;
        input [127:0] text;
        input [15:0] color;
        begin
            if (x>=left && x<right && y>=top_y && y<top_y+16) begin
                label_enable=1;label_text=text;label_x=left;label_right=right;
                label_y=top_y;label_color=color;
            end
        end
    endtask
    task button;
        input [10:0] left,right;
        input [9:0] top_y,bottom_y;
        input [127:0] text;
        input [15:0] fill,ink;
        begin
            if (rect(left,right,top_y,bottom_y)) base_color=fill;
            label(left+8,right-4,top_y+(bottom_y-top_y-16)/2,text,ink);
        end
    endtask
    always @* begin
        label_text=0;label_x=0;label_right=1024;label_y=0;
        label_color=INK;base_color=BG;label_enable=0;
        top=0;slot=0;enabled=0;cal=0;min_ok=0;max_ok=0;limits_ok=0;
        if (y<44) base_color=WHITE;
        if (y==44) base_color=BORDER;
        label(20,300,14,"ACTUATOR TEST   ",INK);
        label(300,504,14,battery_rows[639:512],battery_valid[1] ? GREEN : MUTED);
        label(516,720,14,battery_rows[511:384],battery_valid[0] ? GREEN : MUTED);
        label(732,1008,14,rows[2047-13*128 -:128],flags[8] ? RED : BLUE);
        for (card=0;card<2;card=card+1) begin
            top=card*240;
            if (rect(16,320,top+56,top+280) || rect(336,704,top+56,top+280)) base_color=WHITE;
            // Left: open-loop wheels. Editing always stops that wheel.
            label(28,308,top+68,card==0 ? "LEFT / MOTOR1   " : "RIGHT / MOTOR2  ",BLUE);
            label(28,308,top+100,rows[2047-card*2*128 -:128],INK);
            label(28,308,top+132,rows[2047-(card*2+1)*128 -:128],MUTED);
            button(28,112,top+168,top+208,"- 5%            ",SOFT,BLUE);
            button(124,208,top+168,top+208,"+ 5%            ",SOFT,BLUE);
            enabled=card==0 ? flags[2] : flags[3];
            button(220,308,top+168,top+208,enabled ? "STOP            " : "RUN             ",enabled ? RED : GREEN,WHITE);
            button(28,308,top+216,top+256,"ZERO AND STOP   ",SOFT,BLUE);
            label(28,308,top+260,"SPD COUNT/10MS  ",MUTED);
            // Middle: SERVO1 base first, SERVO0 upper second.
            slot=card==0 ? 4 : 8;
            cal=card==0 ? flags[0] : flags[1];
            enabled=card==0 ? flags[4] : flags[5];
            min_ok=card==0 ? flags[12] : flags[14];
            max_ok=card==0 ? flags[13] : flags[15];
            limits_ok=card==0 ? flags[6] : flags[7];
            label(348,544,top+68,card==0 ? "SERVO1 BASE     " : "SERVO0 UPPER    ",BLUE);
            button(552,692,top+60,top+88,cal ? "CAL US          " : "ANGLE           ",cal ? SOFT : BLUE,cal ? BLUE : WHITE);
            label(348,692,top+96,rows[2047-slot*128 -:128],INK);
            label(348,692,top+120,rows[2047-(slot+1)*128 -:128],cal ? MUTED : BLUE);
            label(348,548,top+144,rows[2047-(slot+2)*128 -:128],limits_ok ? GREEN : MUTED);
            label(552,692,top+144,limits_ok ? "SAVED           " : "UNSET           ",limits_ok ? GREEN : RED);
            button(348,444,top+168,top+208,"-               ",SOFT,BLUE);
            button(456,552,top+168,top+208,"+               ",SOFT,BLUE);
            button(564,692,top+168,top+208,enabled ? "PWM OFF         " : "PWM ON          ",enabled ? RED : GREEN,WHITE);
            button(348,444,top+216,top+256,rows[2047-(slot+3)*128 -:48]=="COARSE" ? "COARSE          " : "FINE            ",SOFT,BLUE);
            button(456,552,top+216,top+256,min_ok ? "MIN OK          " : "SET MIN         ",SOFT,cal ? BLUE : MUTED);
            button(564,692,top+216,top+256,max_ok ? "MAX OK          " : "SET MAX         ",SOFT,cal ? BLUE : MUTED);
            label(348,692,top+260,rows[2047-(slot+3)*128 -:128],MUTED);
        end
        if (rect(720,1008,56,464)) base_color=WHITE;
        label(732,880,68,bq_info ? "BQ DATA         " : "BLUETOOTH       ",BLUE);
        button(884,996,60,88,bq_info ? "BACK            " : "BQ INFO         ",SOFT,BLUE);
        label(732,1008,100,bq_info ? diagnostic_rows[1279 -:128] : rows[2047-12*128 -:128],flags[10] ? GREEN : MUTED);
        label(732,1008,128,bq_info ? diagnostic_rows[1151 -:128] : rows[2047-15*128 -:128],BLUE);
        label(732,1008,bq_info ? 156 : 148,bq_info ? diagnostic_rows[1023 -:128] : rows[2047-14*128 -:128],MUTED);
        label(732,1008,bq_info ? 184 : 172,bq_info ? diagnostic_rows[895 -:128] : {"IMU ",sensor_rows[1023 -:64],"    "},INK);
        label(732,1008,bq_info ? 212 : 196,bq_info ? diagnostic_rows[767 -:128] : sensor_rows[895 -:128],INK);
        label(732,1008,bq_info ? 260 : 228,bq_info ? diagnostic_rows[639 -:128] : sensor_rows[767 -:128],INK);
        label(732,1008,bq_info ? 288 : 260,bq_info ? diagnostic_rows[511 -:128] : sensor_rows[639 -:128],INK);
        label(732,1008,bq_info ? 316 : 308,bq_info ? diagnostic_rows[383 -:128] : sensor_rows[511 -:128],INK);
        label(732,1008,bq_info ? 344 : 340,bq_info ? diagnostic_rows[255 -:128] : sensor_rows[383 -:128],INK);
        label(732,1008,372,bq_info ? diagnostic_rows[127 -:128] : sensor_rows[255 -:128],INK);
        label(732,1008,404,bq_info ? "DF/FW FROM SETUP" : "GYRO DPS / ACC G",MUTED);
        label(732,1008,436,flags[11] ? "TOUCH OK        " : "TOUCH WAIT      ",flags[11] ? GREEN : RED);
        button(720,1008,480,536,"ALL STOP        ",RED,WHITE);
        button(720,1008,548,588,"BQ SETUP        ",SOFT,BLUE);
        button(16,320,548,588,"CLEAR STOP      ",SOFT,BLUE);
        label(348,700,540,battery_rows[383:256],MUTED);
        label(348,700,560,battery_rows[255:128],MUTED);
        label(348,700,580,battery_rows[127:0],MUTED);
    end
endmodule
