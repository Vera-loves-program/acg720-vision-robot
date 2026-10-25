//32-byte metadata receiver. DET1 display is optional; robot packets always
//retain their CRC, byte order and held toggle/ACK mailbox. Local camera video
//does not use this module. A validation clock reuses the payload work bank.
module udp_detection_feedback #(parameter integer ENABLE_DISPLAY=1)(
    input wire clk,reset_n,payload_valid,
    input wire [7:0] payload_byte,
    input wire [15:0] payload_length,
    input wire packet_done,packet_error,display_ack,
    output wire display_request,
    output wire [255:0] display_data,
    input wire robot_ack,
    output reg robot_request=0,
    output reg [255:0] robot_data=0
);
    reg [255:0] work=0;
    reg [5:0] count=0;
    reg [15:0] crc=16'hffff;
    reg [2:0] robot_ack_sync=0;
    reg packet_pending=0;
    reg check_length=0,check_count=0,check_clean=0,check_version=0,check_crc=0;
    reg check_robot_magic=0,robot_free_at_done=0;
    wire stage_capture=packet_done && !packet_pending;
    wire packet_common_ok=packet_pending && check_length && check_count && check_clean && check_version && check_crc;
    wire commit_robot=packet_common_ok && check_robot_magic && robot_free_at_done && robot_request==robot_ack_sync[2];
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] data;
        reg [23:0] combined;
        begin
            // Exact GF(2) expansion of eight MSB-first CCITT-1021 steps.
            // Each constant mask selects at most seven input bits. Explicit
            // parity permits a balanced XOR tree instead of eight dependent
            // conditional polynomial steps in the125MHz receive domain.
            // Framing, initialFFFF and network-order CRC bytes are unchanged.
            combined={previous,data};
            crc_byte[0]=^(combined & 24'h110011);
            crc_byte[1]=^(combined & 24'h220022);
            crc_byte[2]=^(combined & 24'h440044);
            crc_byte[3]=^(combined & 24'h880088);
            crc_byte[4]=^(combined & 24'h100010);
            crc_byte[5]=^(combined & 24'h310031);
            crc_byte[6]=^(combined & 24'h620062);
            crc_byte[7]=^(combined & 24'hc400c4);
            crc_byte[8]=^(combined & 24'h880188);
            crc_byte[9]=^(combined & 24'h100210);
            crc_byte[10]=^(combined & 24'h200420);
            crc_byte[11]=^(combined & 24'h400840);
            crc_byte[12]=^(combined & 24'h911091);
            crc_byte[13]=^(combined & 24'h222022);
            crc_byte[14]=^(combined & 24'h444044);
            crc_byte[15]=^(combined & 24'h888088);
        end
    endfunction
    // Hold work through the validation clock, then reuse that same bank.
    // The vendor RX asserts done in PKT_CHECK after RX_CRC, followed by IDLE
    // and new Ethernet/IP/UDP headers. Even a hypothetical new payload byte
    // on our commit edge is captured as byte1 rather than silently dropped.
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            work<=0;count<=0;crc<=16'hffff;robot_ack_sync<=0;
            robot_request<=0;robot_data<=0;packet_pending<=0;
            check_length<=0;check_count<=0;check_clean<=0;check_version<=0;check_crc<=0;
            check_robot_magic<=0;robot_free_at_done<=0;
        end else begin
            robot_ack_sync<={robot_ack_sync[1:0],robot_ack};
            packet_pending<=0;
            if(packet_pending)begin
                if(payload_valid)begin
                    work<={248'd0,payload_byte};count<=1;crc<=crc_byte(16'hffff,payload_byte);
                end else begin work<=0;count<=0;crc<=16'hffff;end
            end else if(payload_valid && count<32)begin
                work<={work[247:0],payload_byte};count<=count+1'b1;
                if(count<30)crc<=crc_byte(crc,payload_byte);
            end
            if(stage_capture)begin
                packet_pending<=1;
                check_length<=payload_length==32;check_count<=count==32;
                check_clean<=!packet_error;check_version<=work[223:216]==1;
                check_crc<=crc==work[15:0];
                check_robot_magic<=work[255:224]==32'h464f4c31 || work[255:224]==32'h54475431 ||
                    work[255:224]==32'h56454331 || work[255:224]==32'h474f4c31;
                robot_free_at_done<=robot_request==robot_ack_sync[2];
            end
            if(commit_robot)begin robot_data<=work;robot_request<=~robot_request;end
        end
    end

    generate if(ENABLE_DISPLAY!=0)begin:legacy_display
        reg request_q=0;
        reg [255:0] data_q=0;
        assign display_request=request_q;
        assign display_data=data_q;
        reg [2:0] ack_sync=0;
        reg [15:0] last_sequence=0;
        reg have_sequence=0;
        reg [27:0] idle_cycles=28'd250000000;
        reg idle_expired=1;
        reg idle_carry7=0,idle_carry14=0,idle_carry21=0;
        reg check_det_magic=0,check_flags=0,check_has_box=0;
        reg check_x_order=0,check_x_bound=0,check_y_order=0,check_y_bound=0,check_empty=0;
        reg check_score=0,check_width=0,check_height=0,check_age=0,check_sequence=0;
        reg display_free_at_done=0;
        wire [15:0] sequence_delta=work[191:176]-last_sequence;
        wire new_sequence=!have_sequence || idle_expired || (sequence_delta!=0 && !sequence_delta[15]);
        wire commit_display=packet_common_ok && check_det_magic && check_flags &&
            (check_has_box ? check_x_order && check_x_bound && check_y_order && check_y_bound : check_empty) &&
            check_score && check_width && check_height && check_age && check_sequence &&
            display_free_at_done && request_q==ack_sync[2];
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin
                request_q<=0;data_q<=0;ack_sync<=0;last_sequence<=0;have_sequence<=0;
                idle_cycles<=28'd250000000;idle_expired<=1;idle_carry7<=0;idle_carry14<=0;idle_carry21<=0;
                check_det_magic<=0;check_flags<=0;check_has_box<=0;
                check_x_order<=0;check_x_bound<=0;check_y_order<=0;check_y_bound<=0;check_empty<=0;
                check_score<=0;check_width<=0;check_height<=0;check_age<=0;check_sequence<=0;display_free_at_done<=0;
            end else begin
                ack_sync<={ack_sync[1:0],display_ack};
                if(!idle_expired)begin
                    idle_cycles[6:0]<=idle_cycles[6:0]+7'd1;
                    idle_cycles[13:7]<=idle_cycles[13:7]+{6'd0,idle_carry7};
                    idle_cycles[20:14]<=idle_cycles[20:14]+{6'd0,idle_carry14};
                    idle_cycles[27:21]<=idle_cycles[27:21]+{6'd0,idle_carry21};
                    idle_carry7<=idle_cycles[6:0]==7'd126;
                    idle_carry14<=idle_cycles[13:0]==14'd16382;
                    idle_carry21<=idle_cycles[20:0]==21'd2097150;
                    if(idle_cycles==28'd249999999)idle_expired<=1;
                end
                if(stage_capture)begin
                    check_det_magic<=work[255:224]==32'h44455431;
                    check_flags<=work[215:208]==0 || work[215:208]==2 || work[215:208]==3;
                    check_has_box<=work[208];
                    check_x_order<=work[143:128]<work[111:96];check_x_bound<=work[111:96]<=400;
                    check_y_order<=work[127:112]<work[95:80];check_y_bound<=work[95:80]<=240;
                    check_empty<=work[207:192]==0 && work[143:80]==0;
                    check_score<=work[207:192]<=1000;
                    check_width<=work[175:160]==400;check_height<=work[159:144]==240;
                    check_age<=work[79:64]<=1500;check_sequence<=new_sequence;
                    display_free_at_done<=request_q==ack_sync[2];
                end
                if(commit_display)begin
                    data_q<=work;request_q<=~request_q;
                    have_sequence<=1;last_sequence<=work[191:176];
                    // Publication follows the original reset edge by one
                    // clock: loading1 retains its exact restart timeout.
                    idle_cycles<=28'd1;idle_expired<=0;
                    idle_carry7<=0;idle_carry14<=0;idle_carry21<=0;
                end
            end
        end
    end else begin:core_no_detection_display
        assign display_request=1'b0;
        assign display_data=256'd0;
    end endgenerate
endmodule
