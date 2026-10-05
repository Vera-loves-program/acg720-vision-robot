// DET1 is a display-only 32-byte payload. No motor/ROI command is generated.
// Mailbox bytes are in network order: payload byte 0 occupies bits 255:248.
module udp_detection_feedback (
    input wire clk, reset_n,
    input wire payload_valid,
    input wire [7:0] payload_byte,
    input wire [15:0] payload_length,
    input wire packet_done, packet_error,
    input wire display_ack,
    output reg display_request = 0,
    output reg [255:0] display_data = 0
);
    reg [255:0] work = 0;
    reg [5:0] count = 0;
    reg [15:0] crc = 16'hffff;
    reg [2:0] ack_sync = 0;
    reg [15:0] last_sequence = 0;
    reg have_sequence = 0;
    // Allow a restarted host to establish a new sequence after two seconds.
    // GMII receive clock is 125 MHz in this project.
    reg [27:0] idle_cycles = 28'd250000000;
    wire [15:0] sequence_delta = work[191:176] - last_sequence;
    wire new_sequence = !have_sequence || idle_cycles==28'd250000000 ||
        (sequence_delta!=0 && !sequence_delta[15]);
    wire has_box = work[208];
    wire flags_ok = work[215:208]==8'd0 || work[215:208]==8'd2 || work[215:208]==8'd3;
    wire box_ok = has_box ?
        (work[143:128]<work[111:96] && work[111:96]<=16'd400 &&
         work[127:112]<work[95:80] && work[95:80]<=16'd240) :
        (work[207:192]==0 && work[143:80]==0);
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] data;
        reg [15:0] value;
        integer i;
        begin
            value=previous ^ {data,8'd0};
            for (i=0; i<8; i=i+1)
                value=value[15] ? (value<<1)^16'h1021 : value<<1;
            crc_byte=value;
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            work<=0; count<=0; crc<=16'hffff; ack_sync<=0;
            display_request<=0; display_data<=0;
            have_sequence<=0; last_sequence<=0; idle_cycles<=28'd250000000;
        end else begin
            ack_sync<={ack_sync[1:0],display_ack};
            if (idle_cycles<28'd250000000) idle_cycles<=idle_cycles+1'b1;
            if (payload_valid && count<6'd32) begin
                work<={work[247:0],payload_byte};
                if (count<6'd30) crc<=crc_byte(crc,payload_byte);
                count<=count+1'b1;
            end
            if (packet_done) begin
                if (!packet_error && payload_length==16'd32 && count==6'd32 &&
                    work[255:224]==32'h44455431 && work[223:216]==8'd1 &&
                    flags_ok && box_ok && work[207:192]<=16'd1000 &&
                    work[175:160]==16'd400 && work[159:144]==16'd240 &&
                    work[79:64]<=16'd1500 && crc==work[15:0] && new_sequence &&
                    display_request==ack_sync[2]) begin
                    // Hold data until LCD has copied it at a frame boundary.
                    display_data<=work;
                    display_request<=~display_request;
                    have_sequence<=1; last_sequence<=work[191:176]; idle_cycles<=0;
                end
                work<=0; count<=0; crc<=16'hffff;
            end
        end
    end
endmodule
