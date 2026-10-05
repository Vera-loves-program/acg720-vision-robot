// Small single-master register I2C engine for the temporary sensor demo.
// SCL/SDA are OPEN DRAIN: external pull-ups must go to 3.3 V, never 5 V.
// A read is START/address-W/register/RESTART/address-R/data/NACK/STOP.
// read_data[7:0] is the first byte. Only 1..12 byte reads and one byte writes
// are supported. start is a one-clock pulse; requests while busy are ignored.
// error: 00 success, 01 slave NACK, 02 transaction/stretch timeout,
//        03 bus remained stuck after nine recovery clocks, 04 bad read count.
// A future shared TOF bus must use ONE arbitrated instance of this master.
module sensor_i2c_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000
)(
    input wire clk,
    input wire reset_n,
    input wire start,
    input wire [6:0] address,
    input wire [7:0] register_address,
    input wire read_not_write,
    input wire [7:0] write_data,
    input wire [3:0] read_count,
    inout wire scl,
    inout wire sda,
    output reg busy,
    output reg done,
    output reg [7:0] error,
    output reg [95:0] read_data
);
    localparam integer HALF_CYCLES = CLK_HZ / (2 * I2C_HZ);
    localparam integer TIMEOUT_CYCLES = CLK_HZ / 50; // 20 ms, total request
    localparam [4:0] IDLE = 0, BUS_WAIT = 1, START_HOLD = 2,
        TX_LOW = 3, TX_HIGH = 4, TX_ACK_LOW = 5, TX_ACK_HIGH = 6,
        RESTART_LOW = 7, RESTART_HIGH = 8, RX_LOW = 9, RX_HIGH = 10,
        RX_ACK_LOW = 11, RX_ACK_HIGH = 12, STOP_LOW = 13,
        STOP_HIGH = 14, STOP_RELEASE = 15, RECOVER_LOW = 16,
        RECOVER_HIGH = 17;
    localparam [1:0] SEND_ADDRESS_W = 0, SEND_REGISTER = 1,
        SEND_WRITE_DATA = 2, SEND_ADDRESS_R = 3;

    reg [4:0] state;
    reg scl_low, sda_low;
    reg [1:0] scl_sync, sda_sync;
    reg [31:0] phase_count, watchdog;
    reg [6:0] latched_address;
    reg [7:0] latched_register, latched_write, tx_byte, rx_byte;
    reg latched_read;
    reg [3:0] latched_count, byte_index, recovery_count;
    reg [2:0] bit_index;
    reg [1:0] send_role;
    reg recovery_attempted, recovering;
    wire phase_done = (phase_count == HALF_CYCLES - 1);

    assign scl = scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low ? 1'b0 : 1'bz;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            busy <= 1'b0;
            done <= 1'b0;
            error <= 8'h00;
            read_data <= 96'd0;
            phase_count <= 0;
            watchdog <= 0;
            latched_address <= 0;
            latched_register <= 0;
            latched_write <= 0;
            latched_read <= 0;
            latched_count <= 0;
            tx_byte <= 0;
            rx_byte <= 0;
            byte_index <= 0;
            bit_index <= 7;
            send_role <= SEND_ADDRESS_W;
            recovery_count <= 0;
            recovery_attempted <= 0;
            recovering <= 0;
        end else begin
            done <= 1'b0;
            if (!phase_done)
                phase_count <= phase_count + 1'b1;
            if (busy)
                watchdog <= watchdog + 1'b1;

            // A slave that holds SCL low cannot stall the rest of the FPGA.
            // Release the pins on timeout. The next request first checks the
            // bus and, if needed, clocks it free before sending any address.
            if (busy && watchdog >= TIMEOUT_CYCLES - 1) begin
                state <= IDLE;
                scl_low <= 0;
                sda_low <= 0;
                busy <= 0;
                done <= 1;
                error <= 8'h02;
                phase_count <= 0;
            end else begin
                case (state)
                    IDLE: begin
                        scl_low <= 0;
                        sda_low <= 0;
                        phase_count <= 0;
                        if (start) begin
                            error <= 0;
                            read_data <= 0;
                            watchdog <= 0;
                            recovery_attempted <= 0;
                            recovering <= 0;
                            latched_address <= address;
                            latched_register <= register_address;
                            latched_write <= write_data;
                            latched_read <= read_not_write;
                            latched_count <= read_count;
                            tx_byte <= {address, 1'b0};
                            send_role <= SEND_ADDRESS_W;
                            if (read_not_write &&
                                (read_count == 0 || read_count > 12)) begin
                                done <= 1;
                                error <= 8'h04;
                            end else begin
                                busy <= 1;
                                state <= BUS_WAIT;
                            end
                        end
                    end
                    BUS_WAIT: begin
                        scl_low <= 0;
                        sda_low <= 0;
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            if (sda_sync[1]) begin
                                sda_low <= 1;
                                state <= START_HOLD;
                            end else if (!recovery_attempted) begin
                                recovery_attempted <= 1;
                                recovering <= 1;
                                recovery_count <= 0;
                                scl_low <= 1;
                                state <= RECOVER_LOW;
                            end else begin
                                error <= 8'h03;
                                busy <= 0;
                                done <= 1;
                                state <= IDLE;
                            end
                        end
                    end
                    START_HOLD: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            bit_index <= 7;
                            scl_low <= 1;
                            sda_low <= !tx_byte[7];
                            state <= TX_LOW;
                        end
                    end
                    TX_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= TX_HIGH;
                        end
                    end
                    TX_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 1;
                            if (bit_index == 0) begin
                                sda_low <= 0;
                                state <= TX_ACK_LOW;
                            end else begin
                                bit_index <= bit_index - 1'b1;
                                sda_low <= !tx_byte[bit_index - 1'b1];
                                state <= TX_LOW;
                            end
                        end
                    end
                    TX_ACK_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= TX_ACK_HIGH;
                        end
                    end
                    TX_ACK_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 1;
                            bit_index <= 7;
                            if (sda_sync[1]) begin
                                error <= 8'h01;
                                sda_low <= 1;
                                state <= STOP_LOW;
                            end else begin
                                case (send_role)
                                    SEND_ADDRESS_W: begin
                                        tx_byte <= latched_register;
                                        sda_low <= !latched_register[7];
                                        send_role <= SEND_REGISTER;
                                        state <= TX_LOW;
                                    end
                                    SEND_REGISTER: begin
                                        if (latched_read) begin
                                            sda_low <= 0;
                                            state <= RESTART_LOW;
                                        end else begin
                                            tx_byte <= latched_write;
                                            sda_low <= !latched_write[7];
                                            send_role <= SEND_WRITE_DATA;
                                            state <= TX_LOW;
                                        end
                                    end
                                    SEND_WRITE_DATA: begin
                                        sda_low <= 1;
                                        state <= STOP_LOW;
                                    end
                                    SEND_ADDRESS_R: begin
                                        sda_low <= 0;
                                        byte_index <= 0;
                                        rx_byte <= 0;
                                        state <= RX_LOW;
                                    end
                                endcase
                            end
                        end
                    end
                    RESTART_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= RESTART_HIGH;
                        end
                    end
                    RESTART_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            sda_low <= 1;
                            tx_byte <= {latched_address, 1'b1};
                            send_role <= SEND_ADDRESS_R;
                            state <= START_HOLD;
                        end
                    end
                    RX_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= RX_HIGH;
                        end
                    end
                    RX_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 1;
                            rx_byte <= {rx_byte[6:0], sda_sync[1]};
                            if (bit_index == 0) begin
                                read_data[byte_index * 8 +: 8] <=
                                    {rx_byte[6:0], sda_sync[1]};
                                // ACK every byte except the final one.
                                sda_low <= (byte_index != latched_count - 1'b1);
                                state <= RX_ACK_LOW;
                            end else begin
                                bit_index <= bit_index - 1'b1;
                                state <= RX_LOW;
                            end
                        end
                    end
                    RX_ACK_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= RX_ACK_HIGH;
                        end
                    end
                    RX_ACK_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 1;
                            if (byte_index == latched_count - 1'b1) begin
                                sda_low <= 1;
                                state <= STOP_LOW;
                            end else begin
                                sda_low <= 0;
                                byte_index <= byte_index + 1'b1;
                                bit_index <= 7;
                                rx_byte <= 0;
                                state <= RX_LOW;
                            end
                        end
                    end
                    STOP_LOW: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= STOP_HIGH;
                        end
                    end
                    STOP_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            sda_low <= 0;
                            state <= STOP_RELEASE;
                        end
                    end
                    STOP_RELEASE: begin
                        if (phase_done) begin
                            phase_count <= 0;
                            if (recovering && scl_sync[1] && sda_sync[1]) begin
                                recovering <= 0;
                                state <= BUS_WAIT;
                            end else begin
                                if (!scl_sync[1] || !sda_sync[1])
                                    error <= 8'h03;
                                busy <= 0;
                                done <= 1;
                                state <= IDLE;
                            end
                        end
                    end
                    RECOVER_LOW: begin
                        sda_low <= 0;
                        if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 0;
                            state <= RECOVER_HIGH;
                        end
                    end
                    RECOVER_HIGH: begin
                        if (!scl_sync[1]) begin
                            phase_count <= 0;
                        end else if (phase_done) begin
                            phase_count <= 0;
                            scl_low <= 1;
                            if (recovery_count == 8) begin
                                sda_low <= 1;
                                state <= STOP_LOW;
                            end else begin
                                recovery_count <= recovery_count + 1'b1;
                                state <= RECOVER_LOW;
                            end
                        end
                    end
                    default: begin
                        state <= IDLE;
                        scl_low <= 0;
                        sda_low <= 0;
                        busy <= 0;
                        done <= 1;
                        error <= 8'h03;
                    end
                endcase
            end
        end
    end
endmodule
