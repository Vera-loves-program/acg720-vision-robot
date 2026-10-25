// Bounded BQ34Z100-G1 register master. start is accepted only while idle.
// Read: START/0xAA/pointer/RESTART/0xAB/read 1..32 bytes/NACK/STOP.
// Write: START/0xAA/pointer/write 1..2 bytes/STOP. Every written byte must ACK.
// The first received byte is read_data[7:0]; the first written byte is
// write_data[7:0]. Byte ordering of data-flash fields is the caller's concern.
// SCL and SDA are strictly open drain; external pull-ups are required.
// TI SLUSBZ5D 6.14: STOP-to-START >=66 us. SLUUBW5 Appendix A describes
// series of 2..20 ms stretches; the limits below are configurable host policy,
// NOT a TI guaranteed maximum. Clock-high timing starts after actual SCL rises.
// error: 00 OK, 01 NACK, 02 SCL stretch timeout, 03 stuck SDA/recovery failure,
//        04 invalid read/write count, 05 total transaction timeout.
// Sources: https://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf
//          https://www.ti.com/lit/ug/sluubw5/sluubw5.pdf
module bq34z100_i2c_master #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=100000,
    parameter integer STRETCH_TIMEOUT_MS=50,
    parameter integer TRANSACTION_TIMEOUT_MS=2000
)(
    input wire clk, reset_n, start,
    input wire [7:0] register_address,
    input wire read_not_write,
    input wire [5:0] read_count,
    input wire [1:0] write_count,
    input wire [15:0] write_data,
    inout wire scl, sda,
    output reg busy, done,
    output reg [7:0] error,
    output reg [255:0] read_data
);
    localparam integer HALF_CYCLES=(CLK_HZ+2*I2C_HZ-1)/(2*I2C_HZ);
    localparam integer US_CYCLES=(CLK_HZ+999999)/1000000;
    localparam integer BUS_FREE_CYCLES=US_CYCLES*66;
    localparam integer STRETCH_CYCLES=(CLK_HZ/1000)*STRETCH_TIMEOUT_MS;
    localparam integer TRANSACTION_CYCLES=(CLK_HZ/1000)*TRANSACTION_TIMEOUT_MS;
    localparam [4:0] IDLE=0, BUS_WAIT=1, START_HOLD=2,
        TX_LOW=3, TX_HIGH=4, TX_ACK_LOW=5, TX_ACK_HIGH=6,
        RESTART_LOW=7, RESTART_HIGH=8, RX_LOW=9, RX_HIGH=10,
        RX_ACK_LOW=11, RX_ACK_HIGH=12, STOP_LOW=13,
        STOP_HIGH=14, STOP_RELEASE=15, RECOVER_LOW=16, RECOVER_HIGH=17;
    localparam [1:0] ADDRESS_W=0, REGISTER_POINTER=1, ADDRESS_R=2, WRITE_DATA=3;
    reg [4:0] state;
    reg scl_low, sda_low;
    reg [1:0] scl_sync, sda_sync;
    reg [31:0] phase_count, stretch_count, transaction_count, bus_free_count;
    reg [7:0] held_register, tx_byte, rx_byte;
    reg [5:0] held_count;
    reg [1:0] held_write_count, send_role;
    reg [15:0] held_write_data;
    reg held_read, write_index;
    reg [4:0] byte_index;
    reg [2:0] bit_index;
    reg [3:0] recovery_count;
    reg recovering, recovery_attempted;
    wire phase_done=phase_count>=HALF_CYCLES-1;

    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin scl_sync<=2'b11;sda_sync<=2'b11;end
        else begin scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=0;error<=0;read_data<=0;
            phase_count<=0;stretch_count<=0;transaction_count<=0;bus_free_count<=0;
            held_register<=0;tx_byte<=8'haa;rx_byte<=0;held_count<=0;send_role<=ADDRESS_W;
            held_write_count<=0;held_write_data<=0;held_read<=1;write_index<=0;
            byte_index<=0;bit_index<=7;recovery_count<=0;recovering<=0;recovery_attempted<=0;
        end else begin
            done<=0;
            if (phase_count<HALF_CYCLES-1) phase_count<=phase_count+1'b1;
            if (!scl_low && !sda_low && scl_sync[1] && sda_sync[1]) begin
                if (bus_free_count<BUS_FREE_CYCLES) bus_free_count<=bus_free_count+1'b1;
            end else bus_free_count<=0;
            if (busy) transaction_count<=transaction_count+1'b1;
            if (busy && !scl_low && !scl_sync[1]) stretch_count<=stretch_count+1'b1;
            else stretch_count<=0;

            // Every timeout releases both wires. A later request checks/recoveries
            // the physical bus again rather than treating stale data as valid.
            if (busy && transaction_count>=TRANSACTION_CYCLES-1) begin
                state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=8'h05;phase_count<=0;
            end else if (busy && !scl_low && !scl_sync[1] && stretch_count>=STRETCH_CYCLES-1) begin
                state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=8'h02;phase_count<=0;
            end else case (state)
                IDLE:begin
                    scl_low<=0;sda_low<=0;phase_count<=0;
                    if (start) begin
                        error<=0;read_data<=0;transaction_count<=0;stretch_count<=0;
                        held_register<=register_address;held_count<=read_count;
                        held_read<=read_not_write;held_write_count<=write_count;held_write_data<=write_data;write_index<=0;
                        tx_byte<=8'haa;send_role<=ADDRESS_W;bit_index<=7;byte_index<=0;
                        recovering<=0;recovery_attempted<=0;recovery_count<=0;
                        if ((read_not_write && (read_count==0 || read_count>32)) ||
                            (!read_not_write && (write_count==0 || write_count>2))) begin done<=1;error<=8'h04;end
                        else begin busy<=1;state<=BUS_WAIT;end
                    end
                end
                BUS_WAIT:begin
                    phase_count<=0;
                    if (scl_sync[1]) begin
                        if (!sda_sync[1]) begin
                            if (recovery_attempted) begin
                                busy<=0;done<=1;error<=8'h03;state<=IDLE;
                            end else begin
                                recovery_attempted<=1;recovering<=1;recovery_count<=0;
                                scl_low<=1;sda_low<=0;state<=RECOVER_LOW;
                            end
                        end else if (bus_free_count>=BUS_FREE_CYCLES) begin
                            sda_low<=1;state<=START_HOLD;
                        end
                    end
                end
                START_HOLD:if (phase_done) begin
                    phase_count<=0;scl_low<=1;sda_low<=!tx_byte[7];state<=TX_LOW;
                end
                TX_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=TX_HIGH;end
                TX_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin
                        phase_count<=0;scl_low<=1;
                        if (bit_index==0) begin sda_low<=0;state<=TX_ACK_LOW;end
                        else begin bit_index<=bit_index-1'b1;sda_low<=!tx_byte[bit_index-1'b1];state<=TX_LOW;end
                    end
                end
                TX_ACK_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=TX_ACK_HIGH;end
                TX_ACK_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin
                        phase_count<=0;scl_low<=1;
                        if (sda_sync[1]) begin error<=8'h01;sda_low<=1;state<=STOP_LOW;end
                        else case (send_role)
                            ADDRESS_W:begin
                                tx_byte<=held_register;send_role<=REGISTER_POINTER;bit_index<=7;
                                sda_low<=!held_register[7];state<=TX_LOW;
                            end
                            REGISTER_POINTER:begin
                                bit_index<=7;
                                if (held_read) begin
                                    tx_byte<=8'hab;send_role<=ADDRESS_R;sda_low<=0;state<=RESTART_LOW;
                                end else begin
                                    tx_byte<=held_write_data[7:0];send_role<=WRITE_DATA;
                                    sda_low<=!held_write_data[7];state<=TX_LOW;
                                end
                            end
                            ADDRESS_R:begin sda_low<=0;rx_byte<=0;bit_index<=7;state<=RX_LOW;end
                            WRITE_DATA:begin
                                if (!write_index && held_write_count==2) begin
                                    write_index<=1;tx_byte<=held_write_data[15:8];bit_index<=7;
                                    sda_low<=!held_write_data[15];state<=TX_LOW;
                                end else begin sda_low<=1;state<=STOP_LOW;end
                            end
                        endcase
                    end
                end
                RESTART_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=RESTART_HIGH;end
                RESTART_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin phase_count<=0;sda_low<=1;state<=START_HOLD;end
                end
                RX_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=RX_HIGH;end
                RX_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin
                        phase_count<=0;scl_low<=1;rx_byte[bit_index]<=sda_sync[1];
                        if (bit_index==0) begin
                            read_data[byte_index*8 +:8]<={rx_byte[7:1],sda_sync[1]};
                            sda_low<=({1'b0,byte_index}+6'd1)<held_count;state<=RX_ACK_LOW;
                        end else begin bit_index<=bit_index-1'b1;state<=RX_LOW;end
                    end
                end
                RX_ACK_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=RX_ACK_HIGH;end
                RX_ACK_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin
                        phase_count<=0;scl_low<=1;
                        if (({1'b0,byte_index}+6'd1)<held_count) begin
                            byte_index<=byte_index+1'b1;rx_byte<=0;bit_index<=7;sda_low<=0;state<=RX_LOW;
                        end else begin sda_low<=1;state<=STOP_LOW;end
                    end
                end
                STOP_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=STOP_HIGH;end
                STOP_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin phase_count<=0;sda_low<=0;state<=STOP_RELEASE;end
                end
                STOP_RELEASE:begin
                    if (phase_done && scl_sync[1] && !sda_sync[1]) begin
                        state<=IDLE;busy<=0;done<=1;error<=8'h03;
                    end else if (bus_free_count>=BUS_FREE_CYCLES) begin
                        phase_count<=0;
                        if (recovering) begin recovering<=0;state<=BUS_WAIT;end
                        else begin state<=IDLE;busy<=0;done<=1;end
                    end
                end
                RECOVER_LOW:if (phase_done) begin phase_count<=0;scl_low<=0;state<=RECOVER_HIGH;end
                RECOVER_HIGH:begin
                    if (!scl_sync[1]) phase_count<=0;
                    else if (phase_done) begin
                        phase_count<=0;scl_low<=1;
                        if (recovery_count==8) begin sda_low<=1;state<=STOP_LOW;end
                        else begin recovery_count<=recovery_count+1'b1;state<=RECOVER_LOW;end
                    end
                end
                default:begin state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=8'h03;end
            endcase
        end
    end
endmodule
