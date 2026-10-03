// Eight-byte UDP payload: "VRB1", opcode, value, sequence, XOR(bytes 0..6).
// Commit only after the Ethernet receiver reports a CRC-valid packet.
module udp_command_parser (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       payload_valid,
    input  wire [7:0] payload_byte,
    input  wire [15:0] payload_length,
    input  wire       packet_done,
    input  wire       packet_error,
    output reg        command_toggle = 1'b0,
    output reg  [7:0] command_code = 8'd0,
    output reg  [7:0] command_value = 8'd0,
    output reg        valid_command_seen = 1'b0
);
    reg [3:0] index = 4'd0;
    reg [31:0] magic = 32'd0;
    reg [7:0] opcode = 8'd0;
    reg [7:0] value = 8'd0;
    reg [7:0] checksum = 8'd0;
    reg [7:0] received_checksum = 8'd0;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            index <= 0; magic <= 0; opcode <= 0; value <= 0;
            checksum <= 0; received_checksum <= 0;
            command_toggle <= 0; command_code <= 0; command_value <= 0;
            valid_command_seen <= 0;
        end else begin
            if (payload_valid && index < 4'd8) begin
                case (index)
                    4'd0: magic[31:24] <= payload_byte;
                    4'd1: magic[23:16] <= payload_byte;
                    4'd2: magic[15:8]  <= payload_byte;
                    4'd3: magic[7:0]   <= payload_byte;
                    4'd4: opcode <= payload_byte;
                    4'd5: value <= payload_byte;
                    4'd7: received_checksum <= payload_byte;
                    default: ;
                endcase
                if (index < 4'd7) checksum <= checksum ^ payload_byte;
                index <= index + 1'b1;
            end
            if (packet_done) begin
                if (!packet_error && payload_length == 16'd8 &&
                    index == 4'd8 && magic == 32'h56524231 &&
                    checksum == received_checksum &&
                    (opcode == 8'd1 || opcode == 8'd2 ||
                     opcode == 8'd3 || opcode == 8'd4)) begin
                    command_code <= opcode;
                    command_value <= value;
                    command_toggle <= ~command_toggle;
                    valid_command_seen <= 1'b1;
                end
                index <= 0;
                magic <= 0;
                checksum <= 0;
            end
        end
    end
endmodule
