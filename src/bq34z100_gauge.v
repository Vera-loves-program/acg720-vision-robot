// One BQ34Z100-G1 on its own physical I2C bus, fixed seven-bit address 0x55.
// Instantiate twice for two independent same-address gauges. Standard polling
// commands only: SOC 0x02 is ONE byte (0x03 is MaxError); voltage 0x08,
// AverageCurrent 0x0a, temperature 0x0c, flags 0x0e are little-endian words.
// Raw gauge units: %, mV, signed mA, 0.1 K. SOC/current accuracy and scaling
// require the gauge's separately verified hardware configuration/calibration.
// Communication validity does not certify battery calibration or SOC accuracy.
// Explicit begin_setup pauses polling and gives the board setup FSM exclusive
// ownership of this bus. No flash writes occur during ordinary polling/boot.
// Every successful complete poll commits all fields together. Previous values
// remain on error but valid is cleared. valid also expires after STALE_MS;
// a poll that itself spans STALE_MS cannot publish a fresh snapshot.
// error_code: 00 OK, 01..05 master errors, 10 SOC outside 0..100,
//             11 stale/overlong poll, 80 waiting for first complete poll.
// TI SLUSBZ5D table 6.3 requires >=250 ms power-up communication delay;
// SLUUBW5 table 2-1 / sections 2.1.2, 2.1.6..2.1.9 define these commands.
// https://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf
// https://www.ti.com/lit/ug/sluubw5/sluubw5.pdf
module bq34z100_gauge #(
    parameter integer CLK_HZ=50000000,
    parameter integer I2C_HZ=100000,
    parameter integer POLL_HZ=1,
    parameter integer STARTUP_DELAY_MS=300,
    parameter integer STALE_MS=3000,
    parameter integer STRETCH_TIMEOUT_MS=50,
    parameter integer TRANSACTION_TIMEOUT_MS=2000,
    // The normal robot only reads already commissioned gauges. Keep the
    // flash/audit programmer available to the separate commissioning build,
    // but remove its state, RAM and routing entirely from the running robot.
    parameter integer ENABLE_SETUP=1
)(
    input wire clk, reset_n,
    input wire begin_setup,
    inout wire scl, sda,
    output reg valid,
    output reg [7:0] soc,
    output reg [15:0] voltage_mv,
    output reg signed [15:0] current_ma,
    output reg [15:0] temperature_dK,
    output reg [15:0] internal_temperature_dK,pack_configuration,control_status,
    output reg [15:0] flags,
    output reg [7:0] error_code,
    output reg [31:0] sample_count,
    output wire setup_busy,
    output reg configured,
    output wire [7:0] setup_error,setup_phase,
    output wire [15:0] debug_df_pack,debug_divider,debug_control,debug_voltage,debug_firmware,
    output wire [7:0] debug_cells,debug_failure_phase,
    output wire debug_valid
);
    localparam integer POLL_CYCLES=CLK_HZ/POLL_HZ;
    localparam integer STARTUP_CYCLES=(CLK_HZ/1000)*STARTUP_DELAY_MS;
    localparam integer STALE_CYCLES=(CLK_HZ/1000)*STALE_MS;
    localparam [2:0] BOOT=0, IDLE=1, ISSUE=2, WAIT_READ=3,CONTROL_DELAY=4;
    reg [2:0] state;
    reg [3:0] field;
    reg [31:0] startup_count, poll_count, poll_age, fresh_age;
    reg [31:0] control_delay_count;
    reg i2c_start;
    reg [7:0] register_address;
    reg [5:0] read_count;
    reg poll_read;
    wire i2c_busy, i2c_done;
    wire [7:0] i2c_error;
    wire [255:0] block_data;
    wire [15:0] read_data=block_data[15:0];
    reg setup_pending,setup_start;
    wire cfg_busy,cfg_done,cfg_start,cfg_read,cfg_restore_required;
    wire [7:0] cfg_register;
    wire [5:0] cfg_read_count;
    wire [1:0] cfg_write_count;
    wire [15:0] cfg_write_data;
    wire [7:0] cfg_error;
    reg [7:0] runtime_error;
    assign setup_error=cfg_error!=0 ? cfg_error : runtime_error;
    assign setup_busy=setup_pending || setup_start || cfg_busy;
    reg [7:0] candidate_soc;
    reg [15:0] candidate_voltage,candidate_current,candidate_temperature,candidate_internal,candidate_flags,candidate_pack;
    generate if(ENABLE_SETUP!=0)begin:board_commissioning
    bq34z100_board_setup #(.CLK_HZ(CLK_HZ)) u_setup (
        .clk(clk),.reset_n(reset_n),.begin_setup(setup_start),
        .busy(cfg_busy),.done(cfg_done),.error_code(cfg_error),.phase(setup_phase),
        .restore_required(cfg_restore_required),
        .last_df_pack(debug_df_pack),.last_divider(debug_divider),.last_cells(debug_cells),
        .last_control(debug_control),.last_voltage(debug_voltage),.firmware_version(debug_firmware),
        .failure_phase(debug_failure_phase),.diagnostic_valid(debug_valid),
        .request_start(cfg_start),.request_register(cfg_register),.request_read(cfg_read),
        .request_read_count(cfg_read_count),.request_write_count(cfg_write_count),.request_write_data(cfg_write_data),
        .master_busy(i2c_busy),.master_done(i2c_done),.master_error(i2c_error),.master_read_data(block_data)
    );
    end else begin:polling_only
        assign cfg_busy=1'b0;
        assign cfg_done=1'b0;
        assign cfg_start=1'b0;
        assign cfg_read=1'b1;
        assign cfg_restore_required=1'b0;
        assign cfg_register=8'd0;
        assign cfg_read_count=6'd0;
        assign cfg_write_count=2'd0;
        assign cfg_write_data=16'd0;
        assign cfg_error=8'd0;
        assign setup_phase=8'd0;
        assign debug_df_pack=16'd0;
        assign debug_divider=16'd0;
        assign debug_control=16'd0;
        assign debug_voltage=16'd0;
        assign debug_firmware=16'd0;
        assign debug_cells=8'd0;
        assign debug_failure_phase=8'd0;
        assign debug_valid=1'b0;
    end endgenerate
    bq34z100_i2c_master #(
        .CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),
        .STRETCH_TIMEOUT_MS(STRETCH_TIMEOUT_MS),.TRANSACTION_TIMEOUT_MS(TRANSACTION_TIMEOUT_MS)
    ) u_i2c (
        .clk(clk),.reset_n(reset_n),.start(cfg_busy ? cfg_start : i2c_start),
        .register_address(cfg_busy ? cfg_register : register_address),
        .read_not_write(cfg_busy ? cfg_read : poll_read),.read_count(cfg_busy ? cfg_read_count : read_count),
        .write_count(cfg_busy ? cfg_write_count : 2'd2),.write_data(cfg_busy ? cfg_write_data : 16'h0000),
        .scl(scl),.sda(sda),.busy(i2c_busy),.done(i2c_done),.error(i2c_error),.read_data(block_data)
    );
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=BOOT;field<=0;startup_count<=0;poll_count<=0;poll_age<=0;fresh_age<=STALE_CYCLES;control_delay_count<=0;
            i2c_start<=0;register_address<=8'h02;read_count<=1;poll_read<=1;
            setup_pending<=0;setup_start<=0;configured<=0;runtime_error<=0;
            candidate_soc<=0;candidate_voltage<=0;candidate_current<=0;candidate_temperature<=0;
            candidate_internal<=0;candidate_flags<=0;candidate_pack<=0;
            valid<=0;soc<=0;voltage_mv<=0;current_ma<=0;temperature_dK<=0;flags<=0;
            internal_temperature_dK<=0;pack_configuration<=0;control_status<=0;
            error_code<=8'h80;sample_count<=0;
        end else begin
            i2c_start<=0;setup_start<=0;
            if (begin_setup && !setup_busy) begin
                if (ENABLE_SETUP!=0) begin setup_pending<=1;runtime_error<=0;end
            end
            if (cfg_done) begin
                configured<=cfg_error==0;valid<=0;fresh_age<=STALE_CYCLES;
                poll_count<=POLL_CYCLES-1;state<=IDLE;
            end
            if (state!=BOOT && poll_count<POLL_CYCLES-1) poll_count<=poll_count+1'b1;
            if ((state==ISSUE || state==WAIT_READ || state==CONTROL_DELAY) && poll_age<STALE_CYCLES) poll_age<=poll_age+1'b1;
            if (fresh_age<STALE_CYCLES) fresh_age<=fresh_age+1'b1;
            if (valid && fresh_age>=STALE_CYCLES-1) begin
                valid<=0;error_code<=8'h11;
                if (configured) begin configured<=0;runtime_error<=8'hec;end
            end
            if (cfg_busy || setup_start) begin
                valid<=0;state<=IDLE;
            end else case (state)
                BOOT:begin
                    if (startup_count>=STARTUP_CYCLES-1) begin state<=IDLE;poll_count<=POLL_CYCLES-1;end
                    else startup_count<=startup_count+1'b1;
                end
                IDLE:if (setup_pending && !i2c_busy) begin
                    setup_pending<=0;setup_start<=1;configured<=0;valid<=0;
                end else if (!cfg_done && poll_count>=POLL_CYCLES-1 && !i2c_busy) begin
                    poll_count<=0;poll_age<=0;field<=0;state<=ISSUE;
                end
                ISSUE:if (setup_pending) state<=IDLE;
                else if (!i2c_busy) begin
                    poll_read<=1;
                    case (field)
                        0:begin register_address<=8'h02;read_count<=1;end
                        1:begin register_address<=8'h08;read_count<=2;end
                        2:begin register_address<=8'h0a;read_count<=2;end
                        3:begin register_address<=8'h0c;read_count<=2;end
                        4:begin register_address<=8'h0e;read_count<=2;end
                        5:begin register_address<=8'h2a;read_count<=2;end
                        6:begin register_address<=8'h3a;read_count<=2;end
                        7:begin register_address<=8'h00;read_count<=2;poll_read<=0;end
                        default:begin register_address<=8'h00;read_count<=2;end
                    endcase
                    i2c_start<=1;state<=WAIT_READ;
                end
                WAIT_READ:if (i2c_done) begin
                    if (i2c_error!=0) begin
                        valid<=0;error_code<=i2c_error;state<=IDLE;
                        if (configured) begin configured<=0;runtime_error<=8'hec;end
                    end
                    else if (poll_age>=STALE_CYCLES-1) begin
                        valid<=0;error_code<=8'h11;state<=IDLE;
                        if (configured) begin configured<=0;runtime_error<=8'hec;end
                    end
                    else if (field==0 && read_data[7:0]>100) begin
                        valid<=0;error_code<=8'h10;state<=IDLE;
                        if (configured) begin configured<=0;runtime_error<=8'hec;end
                    end
                    else if (field==7) begin
                        // Host margin after the Control subcommand write;
                        // this is not a manufacturer maximum-response claim.
                        control_delay_count<=0;field<=8;state<=CONTROL_DELAY;
                    end
                    else if (field==8) begin
                        soc<=candidate_soc;voltage_mv<=candidate_voltage;current_ma<=candidate_current;
                        temperature_dK<=candidate_temperature;flags<=candidate_flags;
                        internal_temperature_dK<=candidate_internal;pack_configuration<=candidate_pack;control_status<=read_data;
                        // Setup success is revoked when subsequent fresh data
                        // contradicts the board configuration. Keep raw values
                        // visible; a successful I2C read is not a sane 3S voltage.
                        if (configured) begin
                            if (candidate_voltage<8000 || candidate_voltage>14000) begin
                                configured<=0;runtime_error<=8'hea;
                            end else if ((candidate_pack&16'h0801)!=16'h0801 || read_data[12]) begin
                                configured<=0;runtime_error<=8'heb;
                            end
                        end
                        valid<=1;error_code<=0;sample_count<=sample_count+1'b1;fresh_age<=0;state<=IDLE;
                    end else begin
                        case (field)
                            0:candidate_soc<=read_data[7:0];
                            1:candidate_voltage<=read_data;
                            2:candidate_current<=read_data;
                            3:candidate_temperature<=read_data;
                            4:candidate_flags<=read_data;
                            5:candidate_internal<=read_data;
                            6:candidate_pack<=read_data;
                            default:begin end
                        endcase
                        field<=field+1'b1;state<=ISSUE;
                    end
                end
                CONTROL_DELAY:if (control_delay_count>=(CLK_HZ/1000)*10-1) state<=ISSUE;
                    else control_delay_count<=control_delay_count+1'b1;
                default:begin state<=BOOT;startup_count<=0;valid<=0;error_code<=8'h80;end
            endcase
        end
    end
endmodule
