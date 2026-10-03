// GT911 host, 50 MHz clock. Independent of camera and DDR reset/calibration.
// Coordinates are normalized from the controller's reported resolution to LCD.
// No writes to the panel configuration/checksum region 0x8047..0x8100.
// See docs/GT911触摸驱动说明.md for board wiring and protocol references.
module gt911_touch #(
    parameter SWAP_XY = 0,
    parameter INVERT_X = 0,
    parameter INVERT_Y = 0,
    // Standard 8-byte contacts begin with ID at 0x814f. Some HotKnot firmware
    // reserves the first record; select 0x8157 only after checking its firmware.
    parameter [15:0] POINT_BASE = 16'h814f
)(
    input wire clk, reset_n,
    inout wire tp_scl, tp_sda, tp_int,
    output reg tp_reset_n,
    output reg snapshot_valid, snapshot_cancel,
    output reg [1:0] contact_count,
    output reg [10:0] x0, y0, x1, y1,
    output reg [3:0] id0, id1,
    output reg ack_seen, identified, ready,
    output reg [3:0] error_code,
    output reg [31:0] product_id,
    output reg [15:0] resolution_x, resolution_y
);
    localparam RESET_HOLD=0, ADDRESS_SETUP=1, ADDRESS_HOLD=2,
        INT_SYNC=3, BOOT_WAIT=4, META_REQ=5, META_WAIT=6,
        NORMAL_REQ=7, NORMAL_WAIT=8, POLL_DELAY=9,
        STATUS_REQ=10, STATUS_WAIT=11, POINT_REQ=12, POINT_WAIT=13,
        SCALE_START=14, SCALE_RUN=15, CLEAR_REQ=16, CLEAR_WAIT=17,
        RETRY_WAIT=18;
    reg [4:0] state;
    reg [24:0] wait_count;
    reg int_drive, int_value;
    reg [6:0] device_addr;
    assign tp_int = int_drive ? int_value : 1'bz;

    reg request, read_mode;
    reg [15:0] register_addr;
    reg [4:0] read_count;
    wire busy, done, bus_error, address_ack;
    wire [3:0] bus_error_code;
    wire [127:0] read_data;
    gt911_i2c_reg16 u_bus (
        .clk(clk), .reset_n(reset_n), .request(request), .read_mode(read_mode),
        .device_addr(device_addr), .register_addr(register_addr),
        .read_count(read_count), .write_data(8'h00),
        .scl(tp_scl), .sda(tp_sda), .busy(busy), .done(done),
        .error(bus_error), .error_code(bus_error_code),
        .address_ack(address_ack), .read_data(read_data)
    );

    reg [1:0] pending_count;
    reg pending_cancel;
    reg [15:0] raw_x0, raw_y0, raw_x1, raw_y1;
    reg [3:0] pending_id0, pending_id1;
    reg [10:0] pending_x0, pending_y0, pending_x1, pending_y1;
    reg [1:0] scale_axis;
    reg [5:0] divide_steps;
    reg [31:0] dividend, quotient;
    reg [16:0] remainder, denominator;
    wire [16:0] shifted_remainder = {remainder[15:0], dividend[31]};
    wire divide_take = shifted_remainder >= denominator;
    wire [31:0] next_quotient = {quotient[30:0], divide_take};
    wire [16:0] next_remainder = divide_take ?
        shifted_remainder - denominator : shifted_remainder;
    wire [15:0] mapped_res_x = SWAP_XY ? resolution_y : resolution_x;
    wire [15:0] mapped_res_y = SWAP_XY ? resolution_x : resolution_y;
    wire [15:0] mapped_x0 = SWAP_XY ? raw_y0 : raw_x0;
    wire [15:0] mapped_y0 = SWAP_XY ? raw_x0 : raw_y0;
    wire [15:0] mapped_x1 = SWAP_XY ? raw_y1 : raw_x1;
    wire [15:0] mapped_y1 = SWAP_XY ? raw_x1 : raw_y1;
    reg [15:0] scale_coordinate, scale_resolution;
    reg [10:0] scale_limit;
    reg scale_invert;
    always @* begin
        case (scale_axis)
            0: begin scale_coordinate=mapped_x0; scale_resolution=mapped_res_x;
                     scale_limit=11'd1023; scale_invert=INVERT_X; end
            1: begin scale_coordinate=mapped_y0; scale_resolution=mapped_res_y;
                     scale_limit=11'd599; scale_invert=INVERT_Y; end
            2: begin scale_coordinate=mapped_x1; scale_resolution=mapped_res_x;
                     scale_limit=11'd1023; scale_invert=INVERT_X; end
            default: begin scale_coordinate=mapped_y1; scale_resolution=mapped_res_y;
                     scale_limit=11'd599; scale_invert=INVERT_Y; end
        endcase
    end
    wire [10:0] scaled_result = scale_invert ?
        scale_limit - next_quotient[10:0] : next_quotient[10:0];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=RESET_HOLD; wait_count<=0; tp_reset_n<=0;
            int_drive<=1; int_value<=0; device_addr<=7'h5d;
            request<=0; read_mode<=1; register_addr<=16'h8140; read_count<=10;
            snapshot_valid<=0; snapshot_cancel<=0;
            contact_count<=0; x0<=0; y0<=0; x1<=0; y1<=0;
            id0<=0; id1<=0; ack_seen<=0; identified<=0; ready<=0;
            error_code<=0; product_id<=0; resolution_x<=0; resolution_y<=0;
            pending_count<=0; pending_cancel<=0; pending_id0<=0; pending_id1<=0;
            raw_x0<=0; raw_y0<=0; raw_x1<=0; raw_y1<=0;
            pending_x0<=0; pending_y0<=0; pending_x1<=0; pending_y1<=0;
            scale_axis<=0; divide_steps<=0; dividend<=0; quotient<=0;
            remainder<=0; denominator<=1;
        end else begin
            request<=0; snapshot_valid<=0; snapshot_cancel<=0;
            if (address_ack) ack_seen<=1;
            case (state)
                RESET_HOLD: if (wait_count==25'd999999) begin // 20 ms
                    wait_count<=0; int_value<=(device_addr==7'h14);
                    state<=ADDRESS_SETUP;
                end else wait_count<=wait_count+1'b1;
                ADDRESS_SETUP: if (wait_count==25'd99999) begin // 2 ms
                    wait_count<=0; tp_reset_n<=1; state<=ADDRESS_HOLD;
                end else wait_count<=wait_count+1'b1;
                ADDRESS_HOLD: if (wait_count==25'd499999) begin // 10 ms
                    wait_count<=0; int_value<=0; state<=INT_SYNC;
                end else wait_count<=wait_count+1'b1;
                INT_SYNC: if (wait_count==25'd2499999) begin // INT low for 50 ms
                    wait_count<=0; int_drive<=0; state<=BOOT_WAIT;
                end else wait_count<=wait_count+1'b1;
                BOOT_WAIT: if (wait_count==25'd2499999) begin // 50 ms floating
                    wait_count<=0; state<=META_REQ;
                end else wait_count<=wait_count+1'b1;
                META_REQ: if (!busy) begin
                    read_mode<=1; register_addr<=16'h8140; read_count<=10;
                    request<=1; state<=META_WAIT;
                end
                META_WAIT: if (done) begin
                    product_id<=read_data[31:0];
                    resolution_x<={read_data[63:56],read_data[55:48]};
                    resolution_y<={read_data[79:72],read_data[71:64]};
                    if (bus_error) begin
                        error_code<=bus_error_code; ready<=0;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else if (read_data[23:0]!=24'h313139) begin // ASCII 911
                        error_code<=4'd6; identified<=0;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else if ({read_data[63:56],read_data[55:48]}<16'd2 ||
                                 {read_data[63:56],read_data[55:48]}>16'd4096 ||
                                 {read_data[79:72],read_data[71:64]}<16'd2 ||
                                 {read_data[79:72],read_data[71:64]}>16'd4096) begin
                        identified<=1; error_code<=4'd8; ready<=0;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else begin
                        identified<=1; error_code<=0; state<=NORMAL_REQ;
                    end
                end
                NORMAL_REQ: if (!busy) begin
                    // Command 0 = normal coordinate reporting. Command 2 is
                    // raw/diff mode in the GT911 guide, NOT a soft reset.
                    read_mode<=0; register_addr<=16'h8040; request<=1;
                    state<=NORMAL_WAIT;
                end
                NORMAL_WAIT: if (done) begin
                    if (bus_error) begin
                        error_code<=bus_error_code; ready<=0;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else begin
                        ready<=1; error_code<=0; wait_count<=0; state<=POLL_DELAY;
                    end
                end
                POLL_DELAY: if (wait_count==25'd249999) begin // 5 ms polling
                    wait_count<=0; state<=STATUS_REQ;
                end else wait_count<=wait_count+1'b1;
                STATUS_REQ: if (!busy) begin
                    read_mode<=1; register_addr<=16'h814e; read_count<=1;
                    request<=1; state<=STATUS_WAIT;
                end
                STATUS_WAIT: if (done) begin
                    if (bus_error) begin
                        error_code<=bus_error_code; ready<=0;
                        contact_count<=0; snapshot_valid<=1; snapshot_cancel<=1;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else if (!read_data[7]) begin
                        wait_count<=0; state<=POLL_DELAY;
                    end else if (read_data[3:0]>5 || read_data[6:5]!=0) begin
                        // Ignore invalid count, palm/large-area and proximity.
                        pending_count<=0; pending_cancel<=1;
                        error_code<=4'd9; state<=CLEAR_REQ;
                    end else if (read_data[3:0]==0) begin
                        pending_count<=0; pending_cancel<=0;
                        error_code<=0; state<=CLEAR_REQ;
                    end else begin
                        pending_count <= (read_data[3:0]>=2) ? 2'd2 : 2'd1;
                        pending_cancel<=0;
                        state<=POINT_REQ;
                    end
                end
                POINT_REQ: if (!busy) begin
                    read_mode<=1; register_addr<=POINT_BASE;
                    read_count <= (pending_count==2) ? 5'd16 : 5'd8;
                    request<=1; state<=POINT_WAIT;
                end
                POINT_WAIT: if (done) begin
                    if (bus_error) begin
                        error_code<=bus_error_code; ready<=0;
                        contact_count<=0; snapshot_valid<=1; snapshot_cancel<=1;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else if ({read_data[23:16],read_data[15:8]}>=resolution_x ||
                                 {read_data[39:32],read_data[31:24]}>=resolution_y ||
                                 (pending_count==2 &&
                                 ({read_data[87:80],read_data[79:72]}>=resolution_x ||
                                  {read_data[103:96],read_data[95:88]}>=resolution_y))) begin
                        pending_count<=0; pending_cancel<=1;
                        error_code<=4'd9; state<=CLEAR_REQ;
                    end else begin
                        raw_x0<={read_data[23:16],read_data[15:8]};
                        raw_y0<={read_data[39:32],read_data[31:24]};
                        pending_id0<=read_data[3:0];
                        raw_x1 <= (pending_count==2) ?
                            {read_data[87:80],read_data[79:72]} : 16'd0;
                        raw_y1 <= (pending_count==2) ?
                            {read_data[103:96],read_data[95:88]} : 16'd0;
                        pending_id1<=read_data[67:64];
                        error_code<=0; scale_axis<=0; state<=SCALE_START;
                    end
                end
                SCALE_START: begin
                    // Constant products followed by a 32-cycle divider. No
                    // variable combinational division in the 50 MHz path.
                    dividend<=scale_coordinate * scale_limit;
                    denominator<={1'b0,scale_resolution}-17'd1;
                    quotient<=0; remainder<=0; divide_steps<=0;
                    state<=SCALE_RUN;
                end
                SCALE_RUN: begin
                    dividend<={dividend[30:0],1'b0}; quotient<=next_quotient;
                    remainder<=next_remainder;
                    if (divide_steps==31) begin
                        case (scale_axis)
                            0: pending_x0<=scaled_result;
                            1: pending_y0<=scaled_result;
                            2: pending_x1<=scaled_result;
                            3: pending_y1<=scaled_result;
                        endcase
                        if (scale_axis==3) state<=CLEAR_REQ;
                        else begin scale_axis<=scale_axis+1'b1; state<=SCALE_START; end
                    end else divide_steps<=divide_steps+1'b1;
                end
                CLEAR_REQ: if (!busy) begin
                    read_mode<=0; register_addr<=16'h814e; request<=1;
                    state<=CLEAR_WAIT;
                end
                CLEAR_WAIT: if (done) begin
                    if (bus_error) begin
                        error_code<=bus_error_code; ready<=0;
                        contact_count<=0; snapshot_valid<=1; snapshot_cancel<=1;
                        wait_count<=0; state<=RETRY_WAIT;
                    end else begin
                        contact_count<=pending_count; snapshot_valid<=1;
                        // Invalid records cancel gestures. Publishing them as a
                        // normal zero-contact report would commit a pending tap.
                        snapshot_cancel<=pending_cancel;
                        x0<=pending_x0; y0<=pending_y0; id0<=pending_id0;
                        x1<=pending_x1; y1<=pending_y1; id1<=pending_id1;
                        wait_count<=0; state<=POLL_DELAY;
                    end
                end
                RETRY_WAIT: if (wait_count==25'd24999999) begin // 500 ms
                    device_addr <= (device_addr==7'h5d) ? 7'h14 : 7'h5d;
                    ready<=0; identified<=0; tp_reset_n<=0;
                    int_drive<=1; int_value<=0; wait_count<=0; state<=RESET_HOLD;
                end else wait_count<=wait_count+1'b1;
                default: begin
                    ready<=0; tp_reset_n<=0; int_drive<=1; int_value<=0;
                    error_code<=4'd5; wait_count<=0; state<=RESET_HOLD;
                end
            endcase
        end
    end
endmodule

// Open-drain 16-bit register I2C master, one-byte write / 1..16-byte read.
// First received byte is read_data[7:0]. Sample actual SDA for every slave ACK.
module gt911_i2c_reg16 (
    input wire clk, reset_n, request, read_mode,
    input wire [6:0] device_addr,
    input wire [15:0] register_addr,
    input wire [4:0] read_count,
    input wire [7:0] write_data,
    inout wire scl, sda,
    output reg busy, done, error, address_ack,
    output reg [3:0] error_code,
    output reg [127:0] read_data
);
    localparam IDLE=0, START_A=1, START_B=2, START_C=3,
        TX_SETUP=4, TX_RAISE=5, TX_HIGH=6, TX_LOW=7,
        ACK_SETUP=8, ACK_RAISE=9, ACK_HIGH=10, ACK_LOW=11,
        RESTART_A=12, RESTART_B=13, RESTART_C=14,
        RX_SETUP=15, RX_RAISE=16, RX_HIGH=17, RX_LOW=18,
        MACK_SETUP=19, MACK_RAISE=20, MACK_HIGH=21, MACK_LOW=22,
        STOP_A=23, STOP_B=24, STOP_C=25,
        RESTART_HIGH_HOLD=26, STOP_HIGH_HOLD=27;
    reg [4:0] state;
    reg scl_low, sda_low;
    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;
    reg scl_meta, scl_sync, sda_meta, sda_sync;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin scl_meta<=1; scl_sync<=1; sda_meta<=1; sda_sync<=1; end
        else begin scl_meta<=scl; scl_sync<=scl_meta; sda_meta<=sda; sda_sync<=sda_meta; end
    end
    reg [7:0] divider;
    reg [19:0] watchdog;
    reg [6:0] addr;
    reg [15:0] reg_addr;
    reg rd;
    reg [4:0] count;
    reg [7:0] payload, tx_byte;
    reg [1:0] tx_stage;
    reg [2:0] bit_index;
    reg [3:0] rx_index;
    wire tick = divider==8'd249; // 5 us, four phases per bit ~= 50 kHz
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE; busy<=0; done<=0; error<=0; address_ack<=0;
            error_code<=0; divider<=0; watchdog<=0; scl_low<=0; sda_low<=0;
            read_data<=0; addr<=0; reg_addr<=0; rd<=0; count<=1;
            payload<=0; tx_byte<=0; tx_stage<=0; bit_index<=7; rx_index<=0;
        end else begin
            done<=0;
            if (tick) divider<=0; else divider<=divider+1'b1;
            if (busy) watchdog<=watchdog+1'b1; else watchdog<=0;
            if (busy && watchdog==20'd999999) begin // 20 ms; includes stretch
                state<=IDLE; busy<=0; done<=1; error<=1;
                error_code <= (state==START_A) ? 4'd1 : 4'd2;
                scl_low<=0; sda_low<=0;
            end else if (state==IDLE) begin
                scl_low<=0; sda_low<=0;
                if (request) begin
                    addr<=device_addr; reg_addr<=register_addr; rd<=read_mode;
                    count <= (read_count>=1 && read_count<=16) ? read_count : 5'd1;
                    payload<=write_data; tx_byte<={device_addr,1'b0}; tx_stage<=0;
                    bit_index<=7; rx_index<=0; read_data<=0; busy<=1;
                    error<=0; address_ack<=0; error_code<=0;
                    watchdog<=0; divider<=0; state<=START_A;
                end
            end else if (tick) begin
                case (state)
                    START_A: begin
                        scl_low<=0; sda_low<=0;
                        if (scl_sync && sda_sync) state<=START_B;
                    end
                    START_B: begin sda_low<=1; state<=START_C; end
                    START_C: begin scl_low<=1; state<=TX_SETUP; end
                    TX_SETUP: begin sda_low<=!tx_byte[bit_index]; state<=TX_RAISE; end
                    TX_RAISE: begin scl_low<=0; state<=TX_HIGH; end
                    TX_HIGH: if (scl_sync) state<=TX_LOW;
                    TX_LOW: begin
                        scl_low<=1;
                        if (bit_index==0) state<=ACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=TX_SETUP; end
                    end
                    ACK_SETUP: begin sda_low<=0; state<=ACK_RAISE; end
                    ACK_RAISE: begin scl_low<=0; state<=ACK_HIGH; end
                    ACK_HIGH: if (scl_sync) begin
                        error<=sda_sync;
                        if (sda_sync)
                            error_code <= (tx_stage==0) ? 4'd3 :
                                ((tx_stage==3 && rd) ? 4'd7 : 4'd4);
                        if (tx_stage==0 && !sda_sync) address_ack<=1;
                        state<=ACK_LOW;
                    end
                    ACK_LOW: begin
                        scl_low<=1; bit_index<=7;
                        if (error) state<=STOP_A;
                        else case (tx_stage)
                            0: begin tx_byte<=reg_addr[15:8]; tx_stage<=1; state<=TX_SETUP; end
                            1: begin tx_byte<=reg_addr[7:0]; tx_stage<=2; state<=TX_SETUP; end
                            2: if (rd) state<=RESTART_A;
                               else begin tx_byte<=payload; tx_stage<=3; state<=TX_SETUP; end
                            3: if (rd) state<=RX_SETUP; else state<=STOP_A;
                        endcase
                    end
                    RESTART_A: begin sda_low<=0; state<=RESTART_B; end
                    RESTART_B: begin scl_low<=0; state<=RESTART_C; end
                    RESTART_C: if (scl_sync) state<=RESTART_HIGH_HOLD;
                    RESTART_HIGH_HOLD: if (scl_sync) begin
                        // Start the 5 us setup interval only after actual SCL
                        // high was observed, including after clock stretching.
                        sda_low<=1; tx_byte<={addr,1'b1}; tx_stage<=3; state<=START_C;
                    end else state<=RESTART_C;
                    RX_SETUP: begin sda_low<=0; state<=RX_RAISE; end
                    RX_RAISE: begin scl_low<=0; state<=RX_HIGH; end
                    RX_HIGH: if (scl_sync) begin
                        read_data[rx_index*8+bit_index]<=sda_sync; state<=RX_LOW;
                    end
                    RX_LOW: begin
                        scl_low<=1;
                        if (bit_index==0) state<=MACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=RX_SETUP; end
                    end
                    MACK_SETUP: begin
                        sda_low <= ({1'b0,rx_index}+5'd1 < count);
                        state<=MACK_RAISE;
                    end
                    MACK_RAISE: begin scl_low<=0; state<=MACK_HIGH; end
                    MACK_HIGH: if (scl_sync) state<=MACK_LOW;
                    MACK_LOW: begin
                        scl_low<=1; bit_index<=7;
                        if ({1'b0,rx_index}+5'd1>=count) state<=STOP_A;
                        else begin rx_index<=rx_index+1'b1; state<=RX_SETUP; end
                    end
                    STOP_A: begin scl_low<=1; sda_low<=1; state<=STOP_B; end
                    STOP_B: begin scl_low<=0; state<=STOP_C; end
                    STOP_C: if (scl_sync) state<=STOP_HIGH_HOLD;
                    STOP_HIGH_HOLD: if (scl_sync) begin
                        // Keep SDA low for a complete tick after observing SCL
                        // high. A stretched clock must not shorten STOP setup.
                        sda_low<=0; busy<=0; done<=1; state<=IDLE;
                    end else state<=STOP_C;
                    default: begin
                        state<=IDLE; busy<=0; done<=1; error<=1; error_code<=4'd5;
                        scl_low<=0; sda_low<=0;
                    end
                endcase
            end
        end
    end
endmodule
