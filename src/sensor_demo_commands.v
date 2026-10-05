// HC-06 command receiver: 50 MHz, 115200 baud, 8N1. All commands are single
// ASCII bytes; CR/LF and unknown bytes are ignored. This is telemetry control
// only. It contains no motor-enable, motor-start, or motion-command logic.
//
// P = ping/acknowledgement only; F = 20 Hz; S = 5 Hz; N = default 10 Hz.
// Accepted commands update command_count/last_command; the next numeric CSV
// report is the acknowledgement. last_command is the ASCII byte as an integer.
// rx_errors counts false starts and bad stop bits, never unknown text bytes.
module sensor_demo_commands #(
    parameter CLK_HZ = 50000000,
    parameter BAUD = 115200
)(
    input wire clk,
    input wire reset_n,
    input wire rx,
    output reg [23:0] period_cycles,
    output reg [31:0] command_count,
    output reg [7:0] last_command,
    output reg [31:0] rx_errors
);
    localparam BIT_CLOCKS = (CLK_HZ + BAUD/2) / BAUD;
    localparam HALF_BIT_CLOCKS = BIT_CLOCKS / 2;
    localparam PERIOD_20_HZ = CLK_HZ / 20;
    localparam PERIOD_5_HZ = CLK_HZ / 5;
    localparam PERIOD_10_HZ = CLK_HZ / 10;
    localparam WAIT_IDLE = 0, WAIT_START = 1, VERIFY_START = 2,
        RECEIVE_BITS = 3, VERIFY_STOP = 4;

    // Only the second synchronizer stage is sampled by the UART. The delayed
    // copy detects the start edge after the input has entered this clock domain.
    reg rx_meta, rx_sync, rx_previous;
    reg [2:0] state;
    reg [31:0] bit_clock_count;
    reg [2:0] bit_index;
    reg [7:0] received;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta <= 1;
            rx_sync <= 1;
            rx_previous <= 1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            rx_previous <= rx_sync;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            period_cycles <= PERIOD_10_HZ;
            command_count <= 0;
            last_command <= 0;
            rx_errors <= 0;
            state <= WAIT_IDLE;
            bit_clock_count <= 0;
            bit_index <= 0;
            received <= 0;
        end else begin
            case (state)
                WAIT_IDLE: begin
                    // At boot and after a framing error, wait for one whole
                    // high bit before arming. A sustained low break therefore
                    // causes one error, rather than a sequence of fake bytes.
                    if (!rx_sync)
                        bit_clock_count <= 0;
                    else if (bit_clock_count == BIT_CLOCKS - 1) begin
                        bit_clock_count <= 0;
                        state <= WAIT_START;
                    end else begin
                        bit_clock_count <= bit_clock_count + 1'b1;
                    end
                end
                WAIT_START: begin
                    if (rx_previous && !rx_sync) begin
                        bit_clock_count <= 0;
                        state <= VERIFY_START;
                    end
                end
                VERIFY_START: begin
                    // Reject a short falling-edge glitch. For a real start,
                    // the next sample is data bit 0, one full bit later.
                    if (bit_clock_count == HALF_BIT_CLOCKS - 1) begin
                        bit_clock_count <= 0;
                        if (!rx_sync) begin
                            bit_index <= 0;
                            received <= 0;
                            state <= RECEIVE_BITS;
                        end else begin
                            rx_errors <= rx_errors + 1'b1;
                            state <= WAIT_IDLE;
                        end
                    end else begin
                        bit_clock_count <= bit_clock_count + 1'b1;
                    end
                end
                RECEIVE_BITS: begin
                    if (bit_clock_count == BIT_CLOCKS - 1) begin
                        bit_clock_count <= 0;
                        received[bit_index] <= rx_sync; // LSB first.
                        if (bit_index == 7)
                            state <= VERIFY_STOP;
                        else
                            bit_index <= bit_index + 1'b1;
                    end else begin
                        bit_clock_count <= bit_clock_count + 1'b1;
                    end
                end
                VERIFY_STOP: begin
                    if (bit_clock_count == BIT_CLOCKS - 1) begin
                        bit_clock_count <= 0;
                        if (rx_sync) begin
                            // There is still half a stop bit remaining. Go
                            // straight to edge detection to accept consecutive
                            // correctly framed bytes without requiring a gap.
                            state <= WAIT_START;
                            case (received)
                                8'h50: begin // P
                                    command_count <= command_count + 1'b1;
                                    last_command <= received;
                                end
                                8'h46: begin // F
                                    period_cycles <= PERIOD_20_HZ;
                                    command_count <= command_count + 1'b1;
                                    last_command <= received;
                                end
                                8'h53: begin // S
                                    period_cycles <= PERIOD_5_HZ;
                                    command_count <= command_count + 1'b1;
                                    last_command <= received;
                                end
                                8'h4e: begin // N
                                    period_cycles <= PERIOD_10_HZ;
                                    command_count <= command_count + 1'b1;
                                    last_command <= received;
                                end
                                default: begin end
                            endcase
                        end else begin
                            // No command can take effect if the stop bit is low.
                            rx_errors <= rx_errors + 1'b1;
                            state <= WAIT_IDLE;
                        end
                    end else begin
                        bit_clock_count <= bit_clock_count + 1'b1;
                    end
                end
                default: begin
                    state <= WAIT_IDLE;
                    bit_clock_count <= 0;
                end
            endcase
        end
    end
endmodule
