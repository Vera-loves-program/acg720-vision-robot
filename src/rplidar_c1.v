// SLAMTEC C1M1: 3.3 V UART, 460800 8N1, Standard SCAN (A5 20).
// Local references: C1 datasheet rev1.2 pp6-7,10-13; S&C protocol v2.8
// pp8,12-15,26,32. C1 uses serial RPM control, not an external motor PWM.
// All logic runs on clk. Fractional UART enables do not create clocks.
// This front end never controls wheel outputs. Missing returns are UNKNOWN.
module rplidar_c1 #(
    parameter CLK_HZ = 50000000,
    parameter BAUD = 460800
)(
    input wire clk, reset_n,
    input wire scan_enable,
    input wire restart,
    input wire [14:0] forward_angle_q6,
    input wire lidar_rx,
    output wire lidar_tx,
    output reg point_valid,
    output reg [14:0] angle_q6,
    output reg [15:0] distance_mm,
    output reg [5:0] quality,
    output reg scan_start,
    output reg running, ready,
    output reg [7:0] error_code,
    output reg [7:0] health_status,
    output reg [15:0] health_error,
    output reg [31:0] rx_bytes, node_count, point_count, scan_count,
    output reg [31:0] uart_errors, packet_errors, restart_count,
    output wire [31:0] debug_status,
    output reg [31:0] last_rx_word,
    output reg [127:0] reply_capture,
    output reg [31:0] last_tx_word,
    output wire [15:0] front_mm, right_mm, back_mm, left_mm,
    output wire [3:0] sector_valid,
    output wire scan_complete
);
    localparam [4:0] DISABLED=0, START_WAIT=1, TX_PREFIX=2,
        PREFIX_BUSY=3, PREFIX_DONE=4, TX_COMMAND=5, COMMAND_BUSY=6,
        COMMAND_DONE=7, STOP_WAIT=8, HEALTH_DESC=9, HEALTH_DATA=10,
        SCAN_DESC=11, STREAM=12, RETRY_WAIT=13, TX_EXTRA=14,
        EXTRA_BUSY=15, EXTRA_DONE=16, STOP_VERIFY=17,
        MOTOR_START_WAIT=18, STOP_MOTOR_WAIT=19;
    // error: 00=disabled or confirmed live nodes, 01=boot/STOP wait,
    // 02=health descriptor wait, 03=health payload wait,
    // 04=SCAN descriptor wait, 05=SCAN accepted but no confirmed nodes yet,
    // 10=health timeout,
    // 11=scan descriptor timeout, 12=reported health error,
    // 13=stream stale, 14=bad health status, 15=STOP not confirmed by RX quiet.
    // STOP/RPM commands have no ACK: silence is not proof of physical rotation.
    // Counters and first response bytes retain evidence across automatic retries.
    reg [4:0] state, after_command;
    reg [7:0] command;
    reg stopping;
    reg [15:0] motor_rpm;
    reg [1:0] extra_index,stop_attempts;
    reg [1:0] pending_control;
    reg health_capture_started,scan_capture_started;
    reg health_capture_active,scan_capture_active;
    reg [3:0] health_capture_count,scan_capture_count;
    // A SCAN descriptor acknowledges the request immediately; measurements
    // start only after the motor stabilizes (protocol v2.8 p14). Do not apply
    // the established-stream dropout deadline while waiting for the FIRST
    // node, or STOP/retry can repeatedly interrupt a slow cold start.
    // Eight seconds is an engineering grace period, not a C1 timing spec.
    localparam [12:0] FIRST_NODE_WAIT_MS=13'd8000;
    reg [12:0] wait_ms;
    reg [10:0] stream_age_ms;
    reg stream_seen_node;
    reg [2:0] descriptor_index;
    reg [1:0] health_index;
    reg [31:0] node_window;
    reg [2:0] node_fill;
    assign debug_status={state,node_fill,health_status,error_code,last_rx_word[7:0]};
    reg tx_start;
    reg [7:0] tx_byte;
    wire tx_busy, rx_valid, rx_frame_error;
    wire command_in_flight=(state>=TX_PREFIX && state<=COMMAND_DONE) ||
        state==TX_EXTRA || state==EXTRA_BUSY || state==EXTRA_DONE;
    wire [7:0] rx_byte;
    rplidar_c1_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_rx (
        .clk(clk),.reset_n(reset_n),.rx(lidar_rx),.data(rx_byte),
        .valid(rx_valid),.frame_error(rx_frame_error)
    );
    rplidar_c1_uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_tx (
        .clk(clk),.reset_n(reset_n),.start(tx_start),.data(tx_byte),
        .busy(tx_busy),.tx(lidar_tx)
    );
    // A shared 1 ms enable bounds all long waits to small counters.
    localparam MS_CYCLES=CLK_HZ/1000;
    localparam MS_WIDTH=MS_CYCLES<=1 ? 1 : $clog2(MS_CYCLES);
    localparam [MS_WIDTH-1:0] MS_LAST=MS_CYCLES-1;
    reg [MS_WIDTH-1:0] ms_count;
    wire ms_tick=(ms_count==MS_LAST);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) ms_count<=0;
        else if (ms_tick) ms_count<=0;
        else ms_count<=ms_count+1'b1;
    end
    // Standard SCAN is five little-endian bytes. Match the official SDK's
    // normal-node handler: check S/!S at byte 0 and C at byte 1, then receive
    // the remaining three bytes. There is no per-byte deadline or requirement
    // to discard the first node until a second node has arrived in the protocol.
    // The old 1-2 ms window reset could receive bytes forever while throwing
    // away every partial node during legal gaps in a scan stream.
    wire [39:0] candidate={rx_byte,node_window};
    wire [14:0] candidate_angle={candidate[23:16],candidate[15:9]};
    wire [15:0] candidate_q2=candidate[39:24];
    wire candidate_ok=(candidate[0]^candidate[1]) && candidate[8]
        && (candidate_angle<15'd23040);
    wire candidate_usable=(candidate[7:2]!=0)
        && (candidate_q2>=16'd200) && (candidate_q2<=16'd48000);
    wire node_commit=state==STREAM && rx_valid && !rx_frame_error
        && node_fill==4 && candidate_ok;
    function [7:0] descriptor_byte;
        input [2:0] index;
        input health;
        begin
            case (index)
                0: descriptor_byte=8'hA5;
                1: descriptor_byte=8'h5A;
                2: descriptor_byte=health ? 8'h03 : 8'h05;
                3,4: descriptor_byte=8'h00;
                5: descriptor_byte=health ? 8'h00 : 8'h40;
                default: descriptor_byte=health ? 8'h06 : 8'h81;
            endcase
        end
    endfunction
    // MOTOR_SPEED_CTRL A8 has two little-endian RPM payload bytes. The packet
    // checksum XORs A5, A8, payload length 02 and both payload bytes (protocolp32).
    function [7:0] motor_payload_byte;
        input [1:0] index;
        input [15:0] rpm;
        begin
            case(index)
                0:motor_payload_byte=8'h02;
                1:motor_payload_byte=rpm[7:0];
                2:motor_payload_byte=rpm[15:8];
                default:motor_payload_byte=8'h0F^rpm[7:0]^rpm[15:8];
            endcase
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=DISABLED; after_command<=DISABLED; command<=0;
            stopping<=0; wait_ms<=0; stream_age_ms<=0;stream_seen_node<=0;
            motor_rpm<=0;extra_index<=0;stop_attempts<=0;pending_control<=0;
            health_capture_started<=0;scan_capture_started<=0;
            health_capture_active<=0;scan_capture_active<=0;
            health_capture_count<=0;scan_capture_count<=0;
            descriptor_index<=0; health_index<=0; node_window<=0;
            node_fill<=0; tx_start<=0; tx_byte<=0;
            point_valid<=0; angle_q6<=0; distance_mm<=0; quality<=0;
            scan_start<=0; running<=0; ready<=0; error_code<=0;
            health_status<=8'hFF; health_error<=0; rx_bytes<=0;
            node_count<=0; point_count<=0; scan_count<=0;
            uart_errors<=0; packet_errors<=0; restart_count<=0;
            last_rx_word<=0;reply_capture<=0;last_tx_word<=0;
        end else begin
            tx_start<=0; point_valid<=0; scan_start<=0;
            if (rx_valid) begin
                rx_bytes<=rx_bytes+1'b1;
                last_rx_word<={last_rx_word[23:0],rx_byte};
            end
            // This records bytes accepted by the local transmitter, not an
            // electrical acknowledgement from the C1's RX pin.
            if(tx_start && !tx_busy)last_tx_word<={last_tx_word[23:0],tx_byte};
            if(health_capture_active)begin
                if(state!=HEALTH_DESC && state!=HEALTH_DATA)health_capture_active<=0;
                else if(rx_valid && health_capture_count<8)begin
                    reply_capture[127:64]<={reply_capture[119:64],rx_byte};
                    health_capture_count<=health_capture_count+1'b1;
                end
            end
            if(scan_capture_active)begin
                if(state!=SCAN_DESC && state!=STREAM)scan_capture_active<=0;
                else if(rx_valid && scan_capture_count<8)begin
                    reply_capture[63:0]<={reply_capture[55:0],rx_byte};
                    scan_capture_count<=scan_capture_count+1'b1;
                end
            end
            if (rx_frame_error) begin
                uart_errors<=uart_errors+1'b1;
                node_fill<=0;
            end
            if (restart && scan_enable) begin
                // Explicit operator restart can clear a protective shutdown.
                // Boot text following RESET is discarded during START_WAIT.
                // Queue control until a complete A8 packet has left the UART;
                // interrupting its length/payload can make the peer consume the
                // following STOP as payload until its five-second timeout.
                pending_control<=2;running<=0;ready<=0;stream_seen_node<=0;
                descriptor_index<=0; node_fill<=0;
                error_code<=8'h01;
                restart_count<=restart_count+1'b1;
                reply_capture<=0;health_capture_count<=0;scan_capture_count<=0;
                health_capture_started<=0;scan_capture_started<=0;
                health_capture_active<=0;scan_capture_active<=0;
            end else if ((!scan_enable && state!=DISABLED && !stopping) ||
                         (restart && !scan_enable)) begin
                // An explicit STOP may be retried while already requested OFF.
                // A STOP pulse uses restart with scan_enable=0; a RESET pulse
                // uses restart with scan_enable=1. Both preserve physical TX.
                pending_control<=1;stopping<=1;error_code<=8'h01;
                running<=0;ready<=0;node_fill<=0;stream_seen_node<=0;
            end else if(pending_control!=0 && !command_in_flight)begin
                pending_control<=0;
                state<=TX_PREFIX;running<=0;ready<=0;
                if(pending_control==2 && scan_enable)begin
                    command<=8'h40;after_command<=START_WAIT;
                    wait_ms<=1000;stopping<=0;
                end else begin
                    command<=8'h25;after_command<=STOP_MOTOR_WAIT;
                    wait_ms<=100;stop_attempts<=0;stopping<=1;
                end
            end else begin
                case (state)
                    DISABLED: begin
                        running<=0; ready<=0; stopping<=0;stream_seen_node<=0;
                        if(error_code!=8'h15)error_code<=0;
                        if (scan_enable) begin
                            wait_ms<=1000; state<=START_WAIT; error_code<=8'h01;
                        end
                    end
                    START_WAIT: begin
                        if (wait_ms==0) begin
                            command<=8'h25; after_command<=STOP_WAIT;
                            wait_ms<=100; state<=TX_PREFIX;
                        end else if (ms_tick) wait_ms<=wait_ms-1'b1;
                    end
                    TX_PREFIX: if (!tx_busy) begin
                        tx_byte<=8'hA5; tx_start<=1; state<=PREFIX_BUSY;
                    end
                    PREFIX_BUSY: if (tx_busy) state<=PREFIX_DONE;
                    PREFIX_DONE: if (!tx_busy) state<=TX_COMMAND;
                    TX_COMMAND: if (!tx_busy) begin
                        tx_byte<=command; tx_start<=1; state<=COMMAND_BUSY;
                    end
                    COMMAND_BUSY: if (tx_busy) state<=COMMAND_DONE;
                    COMMAND_DONE: if (!tx_busy) begin
                        if(command==8'hA8)begin extra_index<=0;state<=TX_EXTRA;end
                        else begin
                            state<=after_command;
                            if(after_command==HEALTH_DESC && !health_capture_started)begin
                                health_capture_started<=1;health_capture_active<=1;
                                health_capture_count<=0;reply_capture[127:64]<=0;
                            end
                            if(after_command==SCAN_DESC && !scan_capture_started)begin
                                scan_capture_started<=1;scan_capture_active<=1;
                                scan_capture_count<=0;reply_capture[63:0]<=0;
                            end
                        end
                    end
                    TX_EXTRA:if(!tx_busy)begin
                        tx_byte<=motor_payload_byte(extra_index,motor_rpm);
                        tx_start<=1;state<=EXTRA_BUSY;
                    end
                    EXTRA_BUSY:if(tx_busy)state<=EXTRA_DONE;
                    EXTRA_DONE:if(!tx_busy)begin
                        if(extra_index==3)state<=after_command;
                        else begin extra_index<=extra_index+1'b1;state<=TX_EXTRA;end
                    end
                    STOP_WAIT: begin
                        if (wait_ms==0) begin
                            command<=8'h52; after_command<=HEALTH_DESC;
                            descriptor_index<=0; wait_ms<=2000; state<=TX_PREFIX;
                            error_code<=8'h02;
                        end else if (ms_tick) wait_ms<=wait_ms-1'b1;
                    end
                    MOTOR_START_WAIT:begin
                        if(wait_ms==0)begin
                            command<=8'h20;after_command<=SCAN_DESC;
                            descriptor_index<=0;wait_ms<=2000;state<=TX_PREFIX;
                        end else if(ms_tick)wait_ms<=wait_ms-1'b1;
                    end
                    STOP_MOTOR_WAIT:begin
                        if(wait_ms==0)begin
                            command<=8'hA8;motor_rpm<=0;after_command<=STOP_VERIFY;
                            wait_ms<=100;stream_age_ms<=0;state<=TX_PREFIX;
                        end else if(ms_tick)wait_ms<=wait_ms-1'b1;
                    end
                    STOP_VERIFY:begin
                        running<=0;ready<=0;
                        if(rx_valid)stream_age_ms<=0;
                        else if(ms_tick && stream_age_ms<100)stream_age_ms<=stream_age_ms+1'b1;
                        if(wait_ms==0)begin
                            if(stream_age_ms>=50 && !rx_valid)begin
                                state<=DISABLED;error_code<=0;
                            end else if(stop_attempts<2)begin
                                stop_attempts<=stop_attempts+1'b1;
                                command<=8'h25;after_command<=STOP_MOTOR_WAIT;
                                wait_ms<=100;state<=TX_PREFIX;
                            end else begin state<=DISABLED;error_code<=8'h15;end
                        end else if(ms_tick)wait_ms<=wait_ms-1'b1;
                    end
                    HEALTH_DESC, SCAN_DESC: begin
                        if (wait_ms==0) begin
                            error_code<=state==HEALTH_DESC ? 8'h10 : 8'h11;
                            wait_ms<=1000; state<=RETRY_WAIT; running<=0; ready<=0;
                            restart_count<=restart_count+1'b1;
                        end else begin
                            if (ms_tick) wait_ms<=wait_ms-1'b1;
                            if (rx_valid && !rx_frame_error) begin
                                if (rx_byte==descriptor_byte(descriptor_index,state==HEALTH_DESC)) begin
                                    if (descriptor_index==6) begin
                                        descriptor_index<=0;
                                        if (state==HEALTH_DESC) begin
                                            state<=HEALTH_DATA; health_index<=0;error_code<=8'h03;
                                        end else begin
                                            state<=STREAM; running<=1; ready<=0;
                                            stream_age_ms<=0; node_fill<=0;
                                            stream_seen_node<=0;wait_ms<=FIRST_NODE_WAIT_MS;
                                            node_window<=0; error_code<=8'h05;
                                        end
                                    end else descriptor_index<=descriptor_index+1'b1;
                                end else begin
                                    descriptor_index<=rx_byte==8'hA5 ? 3'd1 : 3'd0;
                                    packet_errors<=packet_errors+1'b1;
                                end
                            end
                        end
                    end
                    HEALTH_DATA: begin
                        if (wait_ms==0) begin
                            error_code<=8'h10; wait_ms<=1000; state<=RETRY_WAIT;
                            restart_count<=restart_count+1'b1;
                        end else begin
                            if (ms_tick) wait_ms<=wait_ms-1'b1;
                            if (rx_valid && !rx_frame_error) begin
                                case (health_index)
                                    0: begin health_status<=rx_byte; health_index<=1; end
                                    1: begin health_error[7:0]<=rx_byte; health_index<=2; end
                                    default: begin
                                        health_error[15:8]<=rx_byte;
                                        if (health_status<=1) begin
                                            // SDK startMotor() selects its stored desired RPM;
                                            // C1's documented typical10Hz is600RPM. Configure
                                            // that RPM, then issue Standard SCAN after10ms.
                                            command<=8'hA8;motor_rpm<=16'd600;
                                            after_command<=MOTOR_START_WAIT;
                                            descriptor_index<=0;wait_ms<=10;
                                            state<=TX_PREFIX;error_code<=8'h04;
                                        end else begin
                                            error_code<=health_status==2 ? 8'h12 : 8'h14;
                                            wait_ms<=1000; state<=RETRY_WAIT;
                                            restart_count<=restart_count+1'b1;
                                        end
                                    end
                                endcase
                            end
                        end
                    end
                    STREAM: begin
                        // Legal zero-distance nodes keep the link alive, but are
                        // never turned into obstacle points or free-space proof.
                        if (!stream_seen_node) begin
                            if(wait_ms==0 && !node_commit)begin
                                running<=0;ready<=0;error_code<=8'h13;
                                wait_ms<=1000;state<=RETRY_WAIT;
                                restart_count<=restart_count+1'b1;
                            end else if(ms_tick && wait_ms!=0)wait_ms<=wait_ms-1'b1;
                        end else if (ms_tick) begin
                            if (stream_age_ms<2000) stream_age_ms<=stream_age_ms+1'b1;
                            if (stream_age_ms>=500)begin ready<=0;error_code<=8'h13;end
                            if (stream_age_ms>=1999 && !node_commit) begin
                                running<=0; ready<=0; error_code<=8'h13;
                                wait_ms<=1000; state<=RETRY_WAIT;
                                restart_count<=restart_count+1'b1;
                            end
                        end
                        if (rx_valid && !rx_frame_error) begin
                            case(node_fill)
                              0: begin
                                if(rx_byte[0]^rx_byte[1]) begin
                                    node_window[7:0]<=rx_byte;node_fill<=1;
                                end else packet_errors<=packet_errors+1'b1;
                              end
                              1: begin
                                if(rx_byte[0]) begin
                                    node_window[15:8]<=rx_byte;node_fill<=2;
                                end else begin
                                    node_fill<=0;packet_errors<=packet_errors+1'b1;
                                end
                              end
                              2: begin node_window[23:16]<=rx_byte;node_fill<=3;end
                              3: begin node_window[31:24]<=rx_byte;node_fill<=4;end
                              4: begin
                                node_fill<=0;
                                if(candidate_ok) begin
                                    node_count<=node_count+1'b1;
                                    stream_age_ms<=0; ready<=1;error_code<=0;stream_seen_node<=1;
                                    angle_q6<=candidate_angle;
                                    distance_mm<={2'd0,candidate_q2[15:2]};
                                    quality<=candidate[7:2];
                                    scan_start<=candidate[0];
                                    if (candidate[0]) scan_count<=scan_count+1'b1;
                                    if (candidate_usable) begin
                                        point_valid<=1; point_count<=point_count+1'b1;
                                    end
                                end else packet_errors<=packet_errors+1'b1;
                              end
                              default:node_fill<=0;
                            endcase
                        end
                    end
                    RETRY_WAIT: begin
                        running<=0; ready<=0; node_fill<=0;
                        if (wait_ms==0) begin wait_ms<=0; state<=START_WAIT; end
                        else if (ms_tick) wait_ms<=wait_ms-1'b1;
                    end
                    default: state<=DISABLED;
                endcase
            end
        end
    end
    rplidar_c1_sectors u_sectors (
        .clk(clk),.reset_n(reset_n),.ms_tick(ms_tick),.active(running && ready && scan_enable),
        .point_valid(point_valid),.angle_q6(angle_q6),.distance_mm(distance_mm),
        .scan_start(scan_start),.forward_angle_q6(forward_angle_q6),
        .front_mm(front_mm),.right_mm(right_mm),.back_mm(back_mm),.left_mm(left_mm),
        .sector_valid(sector_valid),.scan_complete(scan_complete)
    );
endmodule

// 16x fractional receive enable, three center samples per bit, two FF input
// synchronizer, start qualification and mandatory high stop-bit majority.
// Reset phase on a start so 50 MHz/460800 non-integer ratio does not drift.
module rplidar_c1_uart_rx #(
    parameter CLK_HZ=50000000, parameter BAUD=460800
)(
    input wire clk, reset_n, rx,
    output reg [7:0] data,
    output reg valid, frame_error
);
    localparam [1:0] IDLE=0, START=1, DATA=2, STOP=3;
    reg rx_meta, rx_sync;
    reg [1:0] state;
    localparam PHASE_WIDTH=$clog2(CLK_HZ);
    localparam [PHASE_WIDTH:0] CLK_LIMIT=CLK_HZ, PHASE_STEP=BAUD*16;
    reg [PHASE_WIDTH-1:0] phase;
    wire [PHASE_WIDTH:0] phase_sum={1'b0,phase}+PHASE_STEP;
    wire sample_tick=phase_sum>=CLK_LIMIT;
    reg [3:0] sub;
    reg [2:0] bit_index;
    reg [7:0] shift;
    reg sample_a, sample_b;
    wire majority=(sample_a && sample_b) || (sample_a && rx_sync) || (sample_b && rx_sync);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta<=1; rx_sync<=1; state<=IDLE; phase<=0; sub<=0;
            bit_index<=0; shift<=0; sample_a<=1; sample_b<=1;
            data<=0; valid<=0; frame_error<=0;
        end else begin
            rx_meta<=rx; rx_sync<=rx_meta; valid<=0; frame_error<=0;
            if (state==IDLE) begin
                phase<=0;
                if (!rx_sync) begin state<=START; sub<=1; end
            end else begin
                phase<=sample_tick ? phase_sum-CLK_LIMIT : phase_sum;
                if (sample_tick) begin
                    sub<=sub+1'b1;
                    if (sub==7) sample_a<=rx_sync;
                    if (sub==8) sample_b<=rx_sync;
                    case (state)
                        START: begin
                            if (sub==9 && majority) state<=IDLE;
                            else if (sub==15) begin state<=DATA; bit_index<=0; end
                        end
                        DATA: begin
                            if (sub==9) shift[bit_index]<=majority;
                            if (sub==15) begin
                                if (bit_index==7) state<=STOP;
                                else bit_index<=bit_index+1'b1;
                            end
                        end
                        STOP: if (sub==9) begin
                            if (majority) begin data<=shift; valid<=1; end
                            else frame_error<=1;
                            state<=IDLE;
                        end
                        default: state<=IDLE;
                    endcase
                end
            end
        end
    end
endmodule

module rplidar_c1_uart_tx #(
    parameter CLK_HZ=50000000, parameter BAUD=460800
)(
    input wire clk, reset_n, start,
    input wire [7:0] data,
    output reg busy, tx
);
    localparam PHASE_WIDTH=$clog2(CLK_HZ);
    localparam [PHASE_WIDTH:0] CLK_LIMIT=CLK_HZ, PHASE_STEP=BAUD;
    reg [PHASE_WIDTH-1:0] phase;
    wire [PHASE_WIDTH:0] phase_sum={1'b0,phase}+PHASE_STEP;
    wire bit_tick=phase_sum>=CLK_LIMIT;
    reg [9:0] shift;
    reg [3:0] bit_index;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            phase<=0; shift<=10'h3FF; bit_index<=0; busy<=0; tx<=1;
        end else if (!busy) begin
            tx<=1; phase<=0;
            if (start) begin
                shift<={1'b1,data,1'b0}; bit_index<=0; busy<=1; tx<=0;
            end
        end else begin
            phase<=bit_tick ? phase_sum-CLK_LIMIT : phase_sum;
            if (bit_tick) begin
                if (bit_index==9) begin busy<=0; tx<=1; end
                else begin
                    shift<={1'b1,shift[9:1]}; tx<=shift[1]; bit_index<=bit_index+1'b1;
                end
            end
        end
    end
endmodule

// Whole-revolution obstacle minima. Raw sensor angles increase CLOCKWISE.
// forward_angle_q6 is the raw angle looking straight ahead after installation.
// Front=[315,360)+[0,45), right=[45,135), back=[135,225), left=[225,315).
// sector_valid={back,left,right,front}; FFFF means no valid hit / UNKNOWN.
// These polar minima are not a collision envelope or proof of clear travel.
module rplidar_c1_sectors (
    input wire clk, reset_n, ms_tick, active,
    input wire point_valid, scan_start,
    input wire [14:0] angle_q6, forward_angle_q6,
    input wire [15:0] distance_mm,
    output reg [15:0] front_mm, right_mm, back_mm, left_mm,
    output reg [3:0] sector_valid,
    output reg scan_complete
);
    reg scan_seen, pending_valid, pending_start;
    reg [14:0] pending_angle, previous_forward;
    reg [15:0] pending_distance;
    reg [15:0] min_front, min_right, min_back, min_left;
    reg [3:0] current_valid;
    reg [9:0] scan_age_ms;
    wire [14:0] forward_safe=forward_angle_q6<23040 ? forward_angle_q6 : 15'd0;
    wire [15:0] relative_sum={1'b0,angle_q6}+16'd23040-{1'b0,forward_safe};
    wire [14:0] relative_angle=relative_sum>=23040 ? relative_sum-16'd23040 : relative_sum;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            scan_seen<=0; pending_valid<=0; pending_start<=0;
            pending_angle<=0; pending_distance<=0; previous_forward<=0;
            min_front<=16'hFFFF; min_right<=16'hFFFF;
            min_back<=16'hFFFF; min_left<=16'hFFFF; current_valid<=0;
            front_mm<=16'hFFFF; right_mm<=16'hFFFF;
            back_mm<=16'hFFFF; left_mm<=16'hFFFF;
            sector_valid<=0; scan_complete<=0; scan_age_ms<=0;
        end else begin
            scan_complete<=0;
            pending_valid<=point_valid; pending_start<=scan_start;
            pending_angle<=relative_angle; pending_distance<=distance_mm;
            previous_forward<=forward_safe;
            if (!active || previous_forward!=forward_safe) begin
                scan_seen<=0; pending_valid<=0; pending_start<=0;
                min_front<=16'hFFFF; min_right<=16'hFFFF;
                min_back<=16'hFFFF; min_left<=16'hFFFF; current_valid<=0;
                front_mm<=16'hFFFF; right_mm<=16'hFFFF;
                back_mm<=16'hFFFF; left_mm<=16'hFFFF; sector_valid<=0;
                scan_age_ms<=0;
            end else begin
                if (pending_start) scan_age_ms<=0;
                else if (ms_tick && scan_age_ms<500) scan_age_ms<=scan_age_ms+1'b1;
                if (scan_age_ms>=499 && !pending_start) begin
                    // Bytes alone must not keep an old obstacle scan fresh.
                    scan_seen<=0; sector_valid<=0;
                    front_mm<=16'hFFFF; right_mm<=16'hFFFF;
                    back_mm<=16'hFFFF; left_mm<=16'hFFFF;
                    min_front<=16'hFFFF; min_right<=16'hFFFF;
                    min_back<=16'hFFFF; min_left<=16'hFFFF; current_valid<=0;
                end else begin
                    if (pending_start) begin
                        scan_seen<=1;
                        if (scan_seen) begin
                            front_mm<=min_front; right_mm<=min_right;
                            back_mm<=min_back; left_mm<=min_left;
                            sector_valid<=current_valid; scan_complete<=1;
                        end
                        min_front<=16'hFFFF; min_right<=16'hFFFF;
                        min_back<=16'hFFFF; min_left<=16'hFFFF; current_valid<=0;
                    end
                    if (pending_valid && (scan_seen || pending_start)) begin
                        if (pending_angle<2880 || pending_angle>=20160) begin
                            if (pending_start || pending_distance<min_front) min_front<=pending_distance;
                            current_valid<=pending_start ? 4'b0001 : (current_valid|4'b0001);
                        end else if (pending_angle<8640) begin
                            if (pending_start || pending_distance<min_right) min_right<=pending_distance;
                            current_valid<=pending_start ? 4'b0010 : (current_valid|4'b0010);
                        end else if (pending_angle<14400) begin
                            if (pending_start || pending_distance<min_back) min_back<=pending_distance;
                            current_valid<=pending_start ? 4'b1000 : (current_valid|4'b1000);
                        end else begin
                            if (pending_start || pending_distance<min_left) min_left<=pending_distance;
                            current_valid<=pending_start ? 4'b0100 : (current_valid|4'b0100);
                        end
                    end
                end
            end
        end
    end
endmodule

// Body-frame collision envelope. Independent of mapper HOLD/freeze and RAM.
// One sequential CORDIC, constant divider and shared squarer serve every ray;
// no parallel trig table, frame buffer or wide combinational division.
// +X is vehicle front, +Y left. Fixed body returns are counted, never used to
// ray-clear occluded space. The 10mm contact flag is a virtual proximity event,
// not proof of a physical impact (there is no bumper sensor).
module lidar_body_guard(
    input wire clk,reset_n,tick_10ms,active,
    input wire point_valid,scan_start,
    input wire [14:0] angle_q6,forward_angle_q6,
    input wire [15:0] distance_mm,
    input wire signed [15:0] offset_x_mm,offset_y_mm,
    input wire [15:0] body_front_mm,body_rear_mm,body_left_mm,body_right_mm,
    input wire [15:0] drive_margin_mm,turn_margin_mm,contact_margin_mm,
    output wire ready,front_stop,rear_stop,front_slow,turn_stop,virtual_contact,
    output reg [3:0] coverage,
    // Optical-origin ranges, after the SAME measured-body SELF rejection as
    // mapping/collision checks. A raw polar minimum can otherwise display a
    // wheel/chassis return as an obstacle (notably the offset sensor's LEFT).
    output reg [15:0] filtered_front_mm,filtered_right_mm,filtered_rear_mm,filtered_left_mm,
    output reg [15:0] self_points,
    output reg [15:0] rotation_radius_mm,
    output reg signed [15:0] blocking_x_mm,blocking_y_mm,
    output reg map_point_valid,
    output reg signed [15:0] body_point_x,body_point_y,
    output reg [14:0] map_point_angle,
    output reg [15:0] map_point_distance
);
    localparam [5:0] CFG_X=0,CFG_Y=1,CFG_SUM=2,ROOT_TEST=3,ROOT_FINAL=4,
        CFG_DONE=5,IDLE=6,NORM=7,ROT_INIT=8,ROTATE=9,DIV_INIT=10,
        DIV_X=11,DIV_Y=12,HIT=13,RAD_X=14,RAD_Y=15,RAD_DONE=16,MUL_WAIT=17,
        ROT_SHIFT=18,ROT_APPLY=19;
    reg [5:0] state,after_multiply;
    reg [15:0] square_operand;
    reg [31:0] square_result,diagonal_squared,root_squared,point_x_squared;
    // One actual multiplication site; all three uses share its input register.
    always @(posedge clk) square_result<=square_operand*square_operand;
    reg [15:0] root,root_bit,root_trial;
    reg [31:0] rotation_squared;
    reg [158:0] previous_config;
    wire [158:0] current_config={forward_angle_q6,offset_x_mm,offset_y_mm,
        body_front_mm,body_rear_mm,body_left_mm,body_right_mm,
        drive_margin_mm,turn_margin_mm,contact_margin_mm};
    wire config_ok=forward_angle_q6<23040 &&
        offset_x_mm>=-1000 && offset_x_mm<=1000 && offset_y_mm>=-1000 && offset_y_mm<=1000 &&
        body_front_mm>=1 && body_front_mm<=1000 && body_rear_mm>=1 && body_rear_mm<=1000 &&
        body_left_mm>=1 && body_left_mm<=1000 && body_right_mm>=1 && body_right_mm<=1000 &&
        drive_margin_mm<=500 && turn_margin_mm<=500 && contact_margin_mm<=500;
    reg config_ready,scan_seen,scan_good,work_bad;
    reg [5:0] scan_age;
    reg work_front,work_rear,work_slow,work_turn,work_contact;
    reg published_front,published_rear,published_slow,published_turn,published_contact;
    reg [15:0] work_self;
    reg signed [15:0] work_blocking_x,work_blocking_y;
    reg [17:0] work_blocking_metric;
    reg [3:0] work_coverage;
    reg [15:0] work_range_front,work_range_right,work_range_rear,work_range_left;
    assign ready=active && config_ready && config_ok && scan_good && !work_bad && scan_age<30;
    assign front_stop=published_front || work_front;
    assign rear_stop=published_rear || work_rear;
    assign front_slow=published_slow || work_slow;
    assign turn_stop=published_turn || work_turn;
    assign virtual_contact=published_contact || work_contact;
    reg [15:0] held_distance;
    reg [14:0] held_angle;
    reg signed [16:0] theta;
    reg signed [25:0] cx,cy;
    reg signed [16:0] cz;
    reg [3:0] iteration;
    reg signed [25:0] shifted_x,shifted_y;
    reg [3:0] shift_left;
    function signed [16:0] atan_q6;
        input [3:0] i;
        begin case(i)
            0:atan_q6=2880;1:atan_q6=1700;2:atan_q6=898;3:atan_q6=456;
            4:atan_q6=229;5:atan_q6=115;6:atan_q6=57;7:atan_q6=29;
            8:atan_q6=14;9:atan_q6=7;10:atan_q6=4;default:atan_q6=2;
        endcase end
    endfunction
    reg sign_x,sign_y;
    reg [25:0] dividend,quotient;
    reg [11:0] remainder;
    reg [4:0] divide_bit;
    reg [15:0] magnitude_x;
    wire [12:0] trial_remainder={1'b0,remainder[10:0],dividend[25]};
    wire divide_ge=trial_remainder>=13'd1686;
    wire [25:0] next_quotient={quotient[24:0],divide_ge};
    reg signed [16:0] px,py;
    wire [16:0] abs_x=px[16] ? -px : px;
    wire [16:0] abs_y=py[16] ? -py : py;
    wire signed [17:0] x=$signed({px[16],px}),y=$signed({py[16],py});
    wire signed [17:0] f=$signed({2'b00,body_front_mm}),b=$signed({2'b00,body_rear_mm});
    wire signed [17:0] l=$signed({2'b00,body_left_mm}),r=$signed({2'b00,body_right_mm});
    wire signed [17:0] dm=$signed({2'b00,drive_margin_mm}),cm=$signed({2'b00,contact_margin_mm});
    wire inside_body=x<=f && x>=-b && y<=l && y>=-r;
    wire in_drive_width=y<=l+dm && y>=-r-dm;
    wire within_contact=x<=f+cm && x>=-b-cm && y<=l+cm && y>=-r-cm;
    wire [17:0] blocking_metric={1'b0,abs_x}+{1'b0,abs_y};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=CFG_X;after_multiply<=CFG_X;square_operand<=0;diagonal_squared<=0;root_squared<=0;
            point_x_squared<=0;root<=0;root_bit<=0;root_trial<=0;rotation_squared<=0;rotation_radius_mm<=0;
            previous_config<=0;config_ready<=0;scan_seen<=0;scan_good<=0;work_bad<=0;scan_age<=63;
            work_front<=0;work_rear<=0;work_slow<=0;work_turn<=0;work_contact<=0;
            published_front<=0;published_rear<=0;published_slow<=0;published_turn<=0;published_contact<=0;
            self_points<=0;work_self<=0;held_distance<=0;theta<=0;cx<=0;cy<=0;cz<=0;iteration<=0;
            work_coverage<=0;coverage<=0;
            work_range_front<=16'hffff;work_range_right<=16'hffff;work_range_rear<=16'hffff;work_range_left<=16'hffff;
            filtered_front_mm<=16'hffff;filtered_right_mm<=16'hffff;filtered_rear_mm<=16'hffff;filtered_left_mm<=16'hffff;
            blocking_x_mm<=0;blocking_y_mm<=0;work_blocking_x<=0;work_blocking_y<=0;work_blocking_metric<=18'h3ffff;
            held_angle<=0;map_point_valid<=0;map_point_angle<=0;map_point_distance<=0;body_point_x<=0;body_point_y<=0;
            shifted_x<=0;shifted_y<=0;shift_left<=0;
            sign_x<=0;sign_y<=0;dividend<=0;quotient<=0;remainder<=0;divide_bit<=0;magnitude_x<=0;px<=0;py<=0;
        end else begin
            map_point_valid<=0;
            if(tick_10ms && scan_age!=63)scan_age<=scan_age+1'b1;
            if(!active || !config_ok || current_config!=previous_config)begin
                previous_config<=current_config;state<=CFG_X;config_ready<=0;
                scan_seen<=0;scan_good<=0;work_bad<=0;scan_age<=63;
                work_front<=0;work_rear<=0;work_slow<=0;work_turn<=0;work_contact<=0;work_self<=0;
                published_front<=0;published_rear<=0;published_slow<=0;published_turn<=0;published_contact<=0;self_points<=0;
                work_coverage<=0;coverage<=0;
                work_range_front<=16'hffff;work_range_right<=16'hffff;work_range_rear<=16'hffff;work_range_left<=16'hffff;
                filtered_front_mm<=16'hffff;filtered_right_mm<=16'hffff;filtered_rear_mm<=16'hffff;filtered_left_mm<=16'hffff;
                blocking_x_mm<=0;blocking_y_mm<=0;work_blocking_x<=0;work_blocking_y<=0;work_blocking_metric<=18'h3ffff;
            end else if(scan_start && config_ready)begin
                // The boundary point is omitted. Never mix a partly projected
                // old ray into a new scan; overrun invalidates that whole scan.
                scan_seen<=1;scan_good<=scan_seen && !work_bad && state==IDLE;
                if(scan_seen && !work_bad && state==IDLE)begin
                    published_front<=work_front;published_rear<=work_rear;
                    published_slow<=work_slow;published_turn<=work_turn;published_contact<=work_contact;
                    self_points<=work_self;scan_age<=0;
                    coverage<=work_coverage;
                    filtered_front_mm<=work_range_front;filtered_right_mm<=work_range_right;
                    filtered_rear_mm<=work_range_rear;filtered_left_mm<=work_range_left;
                    blocking_x_mm<=work_blocking_x;blocking_y_mm<=work_blocking_y;
                end
                work_front<=0;work_rear<=0;work_slow<=0;work_turn<=0;work_contact<=0;work_self<=0;
                work_bad<=0;state<=IDLE;
                work_coverage<=0;
                work_range_front<=16'hffff;work_range_right<=16'hffff;work_range_rear<=16'hffff;work_range_left<=16'hffff;
                work_blocking_x<=0;work_blocking_y<=0;work_blocking_metric<=18'h3ffff;
            end else begin
                if(point_valid && config_ready && scan_seen && state!=IDLE)work_bad<=1;
                case(state)
                    CFG_X:begin
                        square_operand<=body_front_mm>body_rear_mm ? body_front_mm : body_rear_mm;
                        state<=MUL_WAIT;after_multiply<=CFG_Y;
                    end
                    CFG_Y:begin
                        diagonal_squared<=square_result;
                        square_operand<=body_left_mm>body_right_mm ? body_left_mm : body_right_mm;
                        state<=MUL_WAIT;after_multiply<=CFG_SUM;
                    end
                    CFG_SUM:begin
                        diagonal_squared<=diagonal_squared+square_result;root<=0;root_squared<=0;
                        root_bit<=1024;root_trial<=1024;square_operand<=1024;
                        state<=MUL_WAIT;after_multiply<=ROOT_TEST;
                    end
                    ROOT_TEST:begin
                        if(square_result<=diagonal_squared)begin root<=root_trial;root_squared<=square_result;end
                        if(root_bit==1)state<=ROOT_FINAL;
                        else begin
                            root_bit<=root_bit>>1;
                            root_trial<=(square_result<=diagonal_squared ? root_trial : root) | (root_bit>>1);
                            square_operand<=(square_result<=diagonal_squared ? root_trial : root) | (root_bit>>1);
                            state<=MUL_WAIT;after_multiply<=ROOT_TEST;
                        end
                    end
                    ROOT_FINAL:begin
                        rotation_radius_mm<=root+(root_squared<diagonal_squared ? 16'd1 : 16'd0)+turn_margin_mm;
                        square_operand<=root+(root_squared<diagonal_squared ? 16'd1 : 16'd0)+turn_margin_mm;
                        state<=MUL_WAIT;after_multiply<=CFG_DONE;
                    end
                    CFG_DONE:begin rotation_squared<=square_result;config_ready<=1;state<=IDLE;end
                    MUL_WAIT:state<=after_multiply;
                    IDLE:if(point_valid && scan_seen)begin
                        theta<=$signed({2'b00,forward_angle_q6})-$signed({2'b00,angle_q6});
                        held_distance<=distance_mm;state<=NORM;
                        held_angle<=angle_q6;
                    end
                    NORM:begin
                        if(theta>=11520)theta<=theta-23040;
                        else if(theta< -11520)theta<=theta+23040;
                        state<=ROT_INIT;
                    end
                    ROT_INIT:begin
                        cy<=0;iteration<=0;
                        if(theta>5760)begin cx<=-$signed({held_distance,10'd0});cz<=theta-11520;end
                        else if(theta< -5760)begin cx<=-$signed({held_distance,10'd0});cz<=theta+11520;end
                        else begin cx<=$signed({held_distance,10'd0});cz<=theta;end
                        state<=ROTATE;
                    end
                    ROTATE:begin
                        shifted_x<=cx;shifted_y<=cy;shift_left<=iteration;state<=ROT_SHIFT;
                    end
                    ROT_SHIFT:begin
                        if(shift_left!=0)begin shifted_x<=shifted_x>>>1;shifted_y<=shifted_y>>>1;shift_left<=shift_left-1'b1;end
                        else state<=ROT_APPLY;
                    end
                    ROT_APPLY:begin
                        if(cz>=0)begin cx<=cx-shifted_y;cy<=cy+shifted_x;cz<=cz-atan_q6(iteration);end
                        else begin cx<=cx+shifted_y;cy<=cy-shifted_x;cz<=cz+atan_q6(iteration);end
                        if(iteration==11)state<=DIV_INIT;else iteration<=iteration+1'b1;
                    end
                    DIV_INIT:begin
                        sign_x<=cx[25];sign_y<=cy[25];
                        dividend<=(cx[25] ? -cx : cx)+26'd843;quotient<=0;remainder<=0;divide_bit<=25;state<=DIV_X;
                    end
                    DIV_X,DIV_Y:begin
                        dividend<=dividend<<1;quotient<=next_quotient;
                        remainder<=divide_ge ? trial_remainder-13'd1686 : trial_remainder;
                        if(divide_bit!=0)divide_bit<=divide_bit-1'b1;
                        else if(state==DIV_X)begin
                            magnitude_x<=next_quotient[15:0];
                            dividend<=(cy[25] ? -cy : cy)+26'd843;quotient<=0;remainder<=0;divide_bit<=25;state<=DIV_Y;
                        end else begin
                            px<=$signed({offset_x_mm[15],offset_x_mm})+
                                (sign_x ? -$signed({1'b0,magnitude_x}) : $signed({1'b0,magnitude_x}));
                            py<=$signed({offset_y_mm[15],offset_y_mm})+
                                (sign_y ? -$signed({1'b0,next_quotient[15:0]}) : $signed({1'b0,next_quotient[15:0]}));
                            state<=HIT;
                        end
                    end
                    HIT:begin
                        if(inside_body)begin if(work_self!=65535)work_self<=work_self+1'b1;state<=IDLE;end
                        else begin
                            map_point_valid<=1;map_point_angle<=held_angle;map_point_distance<=held_distance;
                            body_point_x<=px[15:0];body_point_y<=py[15:0];
                            if(theta>=-2880 && theta<=2880)begin
                                work_coverage[0]<=1;
                                if(held_distance<work_range_front)work_range_front<=held_distance;
                            end else if(theta< -2880 && theta>=-8640)begin
                                work_coverage[1]<=1;
                                if(held_distance<work_range_right)work_range_right<=held_distance;
                            end else if(theta>2880 && theta<=8640)begin
                                work_coverage[2]<=1;
                                if(held_distance<work_range_left)work_range_left<=held_distance;
                            end else begin
                                work_coverage[3]<=1;
                                if(held_distance<work_range_rear)work_range_rear<=held_distance;
                            end
                            if(within_contact)work_contact<=1;
                            if((within_contact || (in_drive_width && x>=-b-dm && x<=f+dm)) && blocking_metric<work_blocking_metric)begin
                                work_blocking_x<=px[15:0];work_blocking_y<=py[15:0];work_blocking_metric<=blocking_metric;
                            end
                            if(in_drive_width && x>=-b && x<=f+dm)work_front<=1;
                            if(in_drive_width && x<=f && x>=-b-dm)work_rear<=1;
                            if(in_drive_width && x>=-b && x<=f+dm+100)work_slow<=1;
                            if(abs_x<={1'b0,rotation_radius_mm} && abs_y<={1'b0,rotation_radius_mm})begin
                                square_operand<=abs_x[15:0];state<=MUL_WAIT;after_multiply<=RAD_Y;
                            end else state<=IDLE;
                        end
                    end
                    RAD_Y:begin point_x_squared<=square_result;square_operand<=abs_y[15:0];state<=MUL_WAIT;after_multiply<=RAD_DONE;end
                    RAD_DONE:begin
                        if({1'b0,point_x_squared}+{1'b0,square_result}<={1'b0,rotation_squared})begin
                            work_turn<=1;
                            if(blocking_metric<work_blocking_metric)begin
                                work_blocking_x<=px[15:0];work_blocking_y<=py[15:0];work_blocking_metric<=blocking_metric;
                            end
                        end
                        state<=IDLE;
                    end
                    default:begin state<=CFG_X;config_ready<=0;scan_good<=0;end
                endcase
            end
        end
    end
endmodule
