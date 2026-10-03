// Arbitrate the unchanged row FIFO and a 32-byte UI snapshot onto one UDP TX.
// Video requests are latched, have priority, and receive ONLY their own done.
// Snapshot mailbox: producer holds data until this clock domain acknowledges.
module ui_video_tx_mux (
    input wire clk, reset_n,
    input wire video_start,
    input wire [7:0] video_data,
    output wire video_read,
    output wire video_done,
    input wire mailbox_request,
    input wire [255:0] mailbox_data,
    output reg mailbox_ack,
    output reg tx_start,
    output wire [15:0] tx_length,
    output wire [7:0] tx_data,
    input wire tx_read, tx_done
);
    reg [2:0] request_sync;
    reg [255:0] latest_snapshot, sending_snapshot;
    reg snapshot_pending, video_pending;
    reg busy, sending_ui;
    reg [4:0] byte_index;
    // Leave more than the Ethernet minimum interpacket gap after TX completion.
    reg [4:0] gap_count;
    wire mailbox_new = request_sync[2] != mailbox_ack;
    assign tx_length = sending_ui ? 16'd32 : 16'd802;
    assign tx_data = sending_ui ?
        sending_snapshot[255-(byte_index*8) -: 8] : video_data;
    assign video_read = busy && !sending_ui && tx_read;
    assign video_done = busy && !sending_ui && tx_done;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            request_sync<=0; mailbox_ack<=0;
            latest_snapshot<=0; sending_snapshot<=0;
            snapshot_pending<=0; video_pending<=0;
            busy<=0; sending_ui<=0; tx_start<=0;
            byte_index<=0; gap_count<=0;
        end else begin
            request_sync<={request_sync[1:0],mailbox_request};
            tx_start<=0;
            if (gap_count!=0) gap_count<=gap_count-1'b1;
            if (video_start) video_pending<=1;
            if (mailbox_new) begin
                latest_snapshot<=mailbox_data;
                mailbox_ack<=request_sync[2];
                snapshot_pending<=1;
            end
            if (busy) begin
                if (sending_ui && tx_read) byte_index<=byte_index+1'b1;
                if (tx_done) begin
                    busy<=0;
                    gap_count<=5'd24;
                end
            end else if (gap_count==0) begin
                if (video_pending || video_start) begin
                    sending_ui<=0; busy<=1; tx_start<=1;
                    video_pending<=0;
                end else if (snapshot_pending) begin
                    // If a newer mailbox arrived on this edge, leave it pending.
                    sending_snapshot<=latest_snapshot;
                    snapshot_pending<=mailbox_new;
                    sending_ui<=1; busy<=1; tx_start<=1;
                    byte_index<=0;
                end
            end
        end
    end
endmodule
