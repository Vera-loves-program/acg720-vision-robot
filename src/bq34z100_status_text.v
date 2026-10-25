// LCD-only full-point mapping: raw A83/B88 are the user's observed full points.
// MAP is a relative display estimate, not verified SOC/capacity calibration.
// Gauge registers are untouched; diagnostic R0Xxx retains the reported raw SOC.
module bq34z100_status_text #(parameter integer CLK_HZ=50000000,parameter integer MINIMAL=0)(
    input wire clk,reset_n,
    input wire valid_a,valid_b,
    input wire [7:0] soc_a,soc_b,error_a,error_b,
    input wire [15:0] voltage_a,voltage_b,
    input wire [15:0] temperature_a,temperature_b,internal_a,internal_b,pack_a,pack_b,
    input wire setup_busy_a,setup_busy_b,configured_a,configured_b,
    input wire [7:0] setup_error_a,setup_error_b,setup_phase_a,setup_phase_b,
    input wire [15:0] control_a,control_b,df_pack_a,df_pack_b,divider_a,divider_b,
    input wire [15:0] firmware_a,firmware_b,setup_voltage_a,setup_voltage_b,
    input wire [7:0] cells_a,cells_b,failure_a,failure_b,
    input wire debug_valid_a,debug_valid_b,
    output reg [1919:0] text_rows,
    output reg [1:0] valid_flags,
    output reg text_valid
);
    generate if(MINIMAL!=0)begin:core_voltage_soc
        // Runtime only: two voltage/SOC rows. No temperature, flash audit or
        // status-dashboard banks are retained in the robot routing profile.
        reg [$clog2(CLK_HZ/10+1)-1:0] countdown;
        reg [2:0] state;
        reg [1:0] field;
        reg [4:0] bits_left;
        reg [15:0] values[0:3];
        reg [19:0] digits[0:3];
        reg [15:0] binary_shift;
        reg [19:0] decimal;
        reg [13:0] remainder;
        reg [1:0] held_valid;
        reg [7:0] held_error_a,held_error_b;
        wire [6:0] full_soc=field[0] ? 7'd88 : 7'd83;
        reg [19:0] adjusted;
        integer k,n;
        always @* begin
            adjusted=decimal;
            for(n=0;n<5;n=n+1)if(decimal[n*4+:4]>=5)adjusted[n*4+:4]=decimal[n*4+:4]+4'd3;
        end
        function [7:0] digit;
            input [3:0] x;begin digit={4'h3,x};end
        endfunction
        function [127:0] voltage_row;
            input [7:0] tag;
            input fresh;
            input [19:0] s,v;
            begin voltage_row=fresh ? {tag," ",digit(v[19:16]),digit(v[15:12]),".",digit(v[11:8]),digit(v[7:4]),"V ",
                digit(s[11:8]),digit(s[7:4]),digit(s[3:0]),"%   "} : {tag," BQ WAIT       "};end
        endfunction
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin
                countdown<=0;state<=0;field<=0;bits_left<=0;binary_shift<=0;decimal<=0;remainder<=0;
                held_valid<=0;held_error_a<=0;held_error_b<=0;valid_flags<=0;text_valid<=0;
                text_rows<={"A BQ WAIT       ","B BQ WAIT       ",1664'd0};
                for(k=0;k<4;k=k+1)begin values[k]<=0;digits[k]<=0;end
            end else begin
                text_valid<=0;
                if(countdown!=0)countdown<=countdown-1'b1;
                case(state)
                    0:if(countdown==0)begin
                        countdown<=CLK_HZ/10-1;field<=0;state<=1;
                        values[0]<={8'd0,soc_a};values[1]<={8'd0,soc_b};values[2]<=voltage_a;values[3]<=voltage_b;
                        held_valid<={valid_a && error_a==0,valid_b && error_b==0};
                        held_error_a<=error_a;held_error_b<=error_b;
                    end
                    1:begin
                        decimal<=0;bits_left<=16;
                        if(field<2 && values[field]<{9'd0,full_soc})begin
                            remainder<=(values[field]<<6)+(values[field]<<5)+(values[field]<<2)+(field[0] ? 14'd44 : 14'd41);
                            binary_shift<=0;state<=2;
                        end else begin binary_shift<=field<2 ? 16'd100 : values[field];state<=3;end
                    end
                    2:if(remainder>={7'd0,full_soc})begin remainder<=remainder-full_soc;binary_shift<=binary_shift+1'b1;end
                        else state<=3;
                    3:begin
                        decimal<={adjusted[18:0],binary_shift[15]};binary_shift<={binary_shift[14:0],1'b0};
                        bits_left<=bits_left-1'b1;if(bits_left==1)state<=4;
                    end
                    4:begin digits[field]<=decimal;if(field==3)state<=5;else begin field<=field+1'b1;state<=1;end end
                    5:begin
                        text_rows<={voltage_row("A",held_valid[1],digits[0],digits[2]),voltage_row("B",held_valid[0],digits[1],digits[3]),1664'd0};
                        valid_flags<=held_valid;text_valid<=1;state<=0;
                    end
                    default:state<=0;
                endcase
            end
        end
    end else begin:legacy_full_dashboard
    // One 16-bit double-dabble converter is shared across twelve fields.
    // Snapshot -> optional shared SOC scale -> LOAD/16 shifts/STORE -> PACK.
    // At most100 SCALE clocks per SOC, <8.5 us total each100 ms at50 MHz.
    // No division/modulo, extra converter or RAM is inferred for the mapping.
    localparam [2:0] IDLE=0,LOAD=1,CONVERT=2,STORE=3,PACK=4,SCALE=5;
    reg [2:0] state;
    reg [31:0] update_count;
    reg [3:0] field,bit_count;
    reg [15:0] values[0:11];
    reg [19:0] digits[0:11];
    reg [15:0] binary_shift;
    reg [13:0] soc_remainder;
    reg [19:0] decimal;
    reg [3:0] temp_negative;
    reg [1:0] held_fresh,held_external,held_debug,held_configured;
    reg [15:0] held_pack[0:1],held_control[0:1],held_df_pack[0:1],held_firmware[0:1];
    reg [7:0] held_cells[0:1],held_failure[0:1],held_error[0:1];
    reg [127:0] held_status;
    integer i;
    function [7:0] ascii;
        input [3:0] nibble;
        begin ascii={4'h3,nibble};end
    endfunction
    function [7:0] hex_digit;
        input [3:0] value;
        begin hex_digit=value<10 ? 8'h30+value : 8'h41+value-10;end
    endfunction
    function [31:0] hex4;
        input [15:0] value;
        begin hex4={hex_digit(value[15:12]),hex_digit(value[11:8]),hex_digit(value[7:4]),hex_digit(value[3:0])};end
    endfunction
    function [19:0] add_three;
        input [19:0] original;
        integer n;
        begin
            add_three=original;
            for (n=0;n<5;n=n+1)
                if (original[n*4 +:4]>=5) add_three[n*4 +:4]=original[n*4 +:4]+4'd3;
        end
    endfunction
    wire [19:0] adjusted=add_three(decimal);
    // SCALE only runs for fields0/1, so use an8-bit two-way SOC mux rather
    // than widening every display field. For raw<full, numerator<=8744.
    wire [7:0] soc_raw=field[0] ? values[1][7:0] : values[0][7:0];
    wire [6:0] soc_full=field[0] ? 7'd88 : 7'd83;
    wire [6:0] soc_half=field[0] ? 7'd44 : 7'd41;
    // Exact algebra for old (raw*10-27315) rounding half away from zero:
    // positive magnitude=raw-2731; negative magnitude=2732-raw (tenths C).
    // N~ remains the gauge's uncalibrated curve; no B3950 curve is invented.
    function [15:0] temperature_magnitude;
        input [15:0] raw;
        begin temperature_magnitude=raw>=16'd2732 ? raw-16'd2731 : 16'd2732-raw;end
    endfunction
    function [39:0] temperature;
        input valid,negative;
        input [15:0] magnitude;
        input [19:0] bcd;
        begin
            if (!valid || magnitude>16'd999) temperature="-----";
            else temperature={negative ? 8'h2d : 8'h2b,ascii(bcd[11:8]),ascii(bcd[7:4]),".",ascii(bcd[3:0])};
        end
    endfunction
    function [39:0] decimal5;
        input [19:0] bcd;
        begin decimal5={ascii(bcd[19:16]),ascii(bcd[15:12]),ascii(bcd[11:8]),ascii(bcd[7:4]),ascii(bcd[3:0])};end
    endfunction
    function [23:0] setup_state;
        input busy,configured,fresh;
        input [7:0] error,phase;
        begin
            if (busy) setup_state={"P",hex_digit(phase[7:4]),hex_digit(phase[3:0])};
            else if (error!=0) setup_state={"E",hex_digit(error[7:4]),hex_digit(error[3:0])};
            else setup_state=configured ? (fresh ? "OK " : "CHK") : "---";
        end
    endfunction
    function [127:0] row;
        input [7:0] tag;
        input valid;
        input [7:0] error;
        input [19:0] soc_bcd,millivolt_bcd;
        begin
            // Drop only the millivolt ones digit: e.g. 11109 mV -> 11.10V.
            if (valid) row={tag,soc_bcd<=20'h00020 ? " LOW" : " MAP",
                ascii(soc_bcd[11:8]),ascii(soc_bcd[7:4]),ascii(soc_bcd[3:0]),"% ",
                ascii(millivolt_bcd[19:16]),ascii(millivolt_bcd[15:12]),".",
                ascii(millivolt_bcd[11:8]),ascii(millivolt_bcd[7:4]),"V"};
            else if (error==8'h80 || error==0) row={tag," BQ WAIT       "};
            else row={tag," BQ ERR 0X",hex_digit(error[7:4]),hex_digit(error[3:0]),"   "};
        end
    endfunction
    function [639:0] diagnostics;
        input [7:0] tag;
        input fresh,df_valid;
        input [15:0] voltage,pack,control,df_pack,firmware;
        input [19:0] divider_bcd,setup_voltage_bcd;
        input [7:0] cells,failure,raw_soc;
        begin
            diagnostics={
                tag," V=",fresh ? hex4(voltage) : "----"," P=",fresh ? hex4(pack) : "----"," ",
                tag," CS=",fresh ? hex4(control) : "----"," F=",hex_digit(failure[7:4]),hex_digit(failure[3:0]),"  ",
                tag," DF=",df_valid ? hex4(df_pack) : "----"," N=",df_valid ? hex_digit(cells[7:4]) : 8'h2d,df_valid ? hex_digit(cells[3:0]) : 8'h2d,"  ",
                tag," VD=",df_valid ? decimal5(divider_bcd) : "-----"," R0X",
                    fresh ? hex_digit(raw_soc[7:4]) : 8'h2d,fresh ? hex_digit(raw_soc[3:0]) : 8'h2d,
                tag," SV",df_valid ? decimal5(setup_voltage_bcd) : "-----"," FW",df_valid ? hex4(firmware) : "----"};
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE;update_count<=0;field<=0;bit_count<=0;
            binary_shift<=0;soc_remainder<=0;decimal<=0;temp_negative<=0;
            held_fresh<=0;held_external<=0;held_debug<=0;held_configured<=0;held_status<=0;
            text_valid<=0;valid_flags<=0;
            text_rows<={"A BQ WAIT       ","B BQ WAIT       ",
                "A N~----- I-----","B N~----- I-----","A:--- B:--- NTC~",1280'b0};
            for (i=0;i<12;i=i+1) begin values[i]<=0;digits[i]<=0;end
            for (i=0;i<2;i=i+1) begin
                held_pack[i]<=0;held_control[i]<=0;held_df_pack[i]<=0;held_firmware[i]<=0;
                held_cells[i]<=0;held_failure[i]<=0;held_error[i]<=0;
            end
        end else begin
            text_valid<=0;
            if (update_count==CLK_HZ/10-1) update_count<=0;
            else update_count<=update_count+1'b1;
            case (state)
                IDLE:if (update_count==CLK_HZ/10-1) begin
                    held_fresh<={valid_a,valid_b};held_external<={valid_a && pack_a[0],valid_b && pack_b[0]};
                    held_debug<={debug_valid_a,debug_valid_b};held_configured<={configured_a,configured_b};
                    held_pack[0]<=pack_a;held_pack[1]<=pack_b;held_control[0]<=control_a;held_control[1]<=control_b;
                    held_df_pack[0]<=df_pack_a;held_df_pack[1]<=df_pack_b;
                    held_firmware[0]<=firmware_a;held_firmware[1]<=firmware_b;
                    held_cells[0]<=cells_a;held_cells[1]<=cells_b;held_failure[0]<=failure_a;held_failure[1]<=failure_b;
                    held_error[0]<=error_a;held_error[1]<=error_b;
                    held_status<={"A:",setup_state(setup_busy_a,configured_a,valid_a,setup_error_a,setup_phase_a),
                        " B:",setup_state(setup_busy_b,configured_b,valid_b,setup_error_b,setup_phase_b)," NTC~"};
                    values[0]<={8'd0,soc_a};values[1]<={8'd0,soc_b};values[2]<=voltage_a;values[3]<=voltage_b;
                    values[4]<=temperature_magnitude(temperature_a);values[5]<=temperature_magnitude(temperature_b);
                    values[6]<=temperature_magnitude(internal_a);values[7]<=temperature_magnitude(internal_b);
                    temp_negative<={internal_b<16'd2732,internal_a<16'd2732,temperature_b<16'd2732,temperature_a<16'd2732};
                    values[8]<=divider_a;values[9]<=divider_b;values[10]<=setup_voltage_a;values[11]<=setup_voltage_b;
                    field<=0;state<=LOAD;
                end
                LOAD:begin
                    decimal<=0;bit_count<=0;
                    if (field<2) begin
                        if (soc_raw>=soc_full) begin binary_shift<=16'd100;state<=CONVERT;end
                        else begin
                            // raw*100 + floor(full/2), using shifts/adds only.
                            soc_remainder<={soc_raw,6'd0}+{1'd0,soc_raw,5'd0}+
                                {4'd0,soc_raw,2'd0}+soc_half;
                            binary_shift<=0;state<=SCALE;
                        end
                    end else begin binary_shift<=values[field];state<=CONVERT;end
                end
                SCALE:begin
                    if (soc_remainder>=soc_full) begin
                        soc_remainder<=soc_remainder-soc_full;
                        binary_shift<=binary_shift+16'd1;
                    end else state<=CONVERT;
                end
                CONVERT:begin
                    decimal<={adjusted[18:0],binary_shift[15]};binary_shift<={binary_shift[14:0],1'b0};
                    if (bit_count==15) state<=STORE;
                    else bit_count<=bit_count+1'b1;
                end
                STORE:begin
                    digits[field]<=decimal;
                    if (field==11) state<=PACK;
                    else begin field<=field+1'b1;state<=LOAD;end
                end
                PACK:begin
                    text_rows<={row("A",held_fresh[1],held_error[0],digits[0],digits[2]),
                        row("B",held_fresh[0],held_error[1],digits[1],digits[3]),
                        "A N~",temperature(held_external[1],temp_negative[0],values[4],digits[4])," I",temperature(held_fresh[1],temp_negative[2],values[6],digits[6]),
                        "B N~",temperature(held_external[0],temp_negative[1],values[5],digits[5])," I",temperature(held_fresh[0],temp_negative[3],values[7],digits[7]),
                        held_status,
                        diagnostics("A",held_fresh[1],held_debug[1],values[2],held_pack[0],held_control[0],held_df_pack[0],held_firmware[0],digits[8],digits[10],held_cells[0],held_failure[0],values[0][7:0]),
                        diagnostics("B",held_fresh[0],held_debug[0],values[3],held_pack[1],held_control[1],held_df_pack[1],held_firmware[1],digits[9],digits[11],held_cells[1],held_failure[1],values[1][7:0])};
                    valid_flags<=held_fresh & held_configured;text_valid<=1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
    end endgenerate
endmodule
