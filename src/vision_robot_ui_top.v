// ACG720 GW5AT-60B: local OV5640/DDR3/LCD and FPGA robot commissioning.
// Chapter 45 supplies camera/DDR, chapter 59 supplies PHY/UDP transport.
module vision_robot_ui_top #(
    // Default: C1/PID/navigation pages, local camera, no UDP camera stream.
    // ACTUATOR_TEST=1 selects the retained wheel/servo commissioning page.
    // ROBOT_MODE=0 and ACTUATOR_TEST=0 select the legacy vision/PID UI.
    parameter integer ACTUATOR_TEST=0,
    parameter integer ROBOT_MODE=1,
    parameter integer SEND_CAMERA_UDP=0
) (
    input  wire        clk50M,
    input  wire        reset_n,       // S0 / F15, active low
    input  wire        button4_n,     // S4 / B21 stops armed motors; otherwise toggles filter
    output wire [15:0] TFT_rgb,
    output wire        TFT_hs,
    output wire        TFT_vs,
    output wire        TFT_clk,
    output wire        TFT_de,
    output wire        TFT_pwm,
    output wire [7:0]  led,
    // 2026-10-08 fabricated P7 map; independent 50MHz peripheral controls.
    inout  wire        sensor_scl,
    inout  wire        sensor_sda,
    output wire        ultrasonic_trig,
    input  wire        ultrasonic_echo,
    output wire        bluetooth_tx,
    input  wire        bluetooth_rx,
    // Expansion-board PIN38/P7-38=E18 TX -> C1 RX; PIN40/P7-40=F20 RX <- C1 TX.
    output wire        lidar_tx,
    input wire         lidar_rx,
    input  wire        encoder1_a, encoder1_b, encoder2_a, encoder2_b,
    output wire        motor_pwma, motor_ain1, motor_ain2,
    output wire        motor_pwmb, motor_bin1, motor_bin2,
    output wire [3:0]  laser_shut,
    inout  wire        bq_a_scl, bq_a_sda, bq_b_scl, bq_b_sda,
    output wire        servo_0, servo_1,
    output wire        step_0, dir_0, step_1, dir_1,
    // Fixed onboard RTC/EEPROM I2C bus and 74HC595 seven-segment display.
    inout  wire        IIC_scl,
    inout  wire        IIC_sda,
    output wire        SEG7_DIO,
    output wire        SEG7_RCLK,
    output wire        SEG7_SCLK,
    output wire        rgmii_gtxc,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_txen,
    input  wire        rgmii_rx_clk,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rxdv,
    output wire        eth_mdc,
    inout  wire        eth_mdio,
    output wire        eth_rst_n,

    // ACG720 schematic: FPC35/36/38/39 -> Y21/T20/T21/P20.
    inout  wire        TP_scl,
    inout  wire        TP_sda,
    inout  wire        TP_int,
    output wire        TP_rst_n,

    output wire        camera_sclk,
    inout  wire        camera_sdat,
    input  wire        camera_vsync,
    input  wire        camera_href,
    input  wire        camera_pclk,
    output wire        camera_xclk,
    input  wire [7:0]  camera_data,
    output wire        camera_rst_n,
    output wire        camera_pwdn,

    output wire [13:0] O_ddr_addr,
    output wire [2:0]  O_ddr_ba,
    output wire        O_ddr_cs_n,
    output wire        O_ddr_ras_n,
    output wire        O_ddr_cas_n,
    output wire        O_ddr_we_n,
    output wire        O_ddr_clk,
    output wire        O_ddr_clk_n,
    output wire        O_ddr_cke,
    output wire        O_ddr_odt,
    output wire        O_ddr_reset_n,
    output wire [1:0]  O_ddr_dqm,
    inout  wire [15:0] IO_ddr_dq,
    inout  wire [1:0]  IO_ddr_dqs,
    inout  wire [1:0]  IO_ddr_dqs_n
);
    localparam [27:0] IMAGE_PIXELS = 28'd384000; // 800 * 480
    localparam [7:0] DDR_BURST = 8'd200;         // 800 / 4, vendor geometry
    localparam ROBOT_ACTIVE=ROBOT_MODE!=0 && ACTUATOR_TEST==0;

    // Board startup follows the LCD project that already worked physically.
    reg reset_meta = 1'b0, reset_sync = 1'b0;
    reg [22:0] startup_count = 0;
    reg ready = 1'b0;
    always @(posedge clk50M) begin
        reset_meta <= reset_n;
        reset_sync <= reset_meta;
        if (!reset_sync) begin
            startup_count <= 0;
            ready <= 1'b0;
        end else if (!ready) begin
            if (startup_count == 23'd4999999) ready <= 1'b1;
            else startup_count <= startup_count + 1'b1;
        end
    end

    // Same 50 MHz clock; UART commands control telemetry and explicit RTC setting.
    wire imu_demo_valid, us_demo_valid;
    wire [7:0] imu_demo_who, imu_demo_error, us_demo_error;
    wire [31:0] imu_demo_samples, us_demo_samples;
    wire signed [15:0] imu_gx, imu_gy, imu_gz, imu_ax, imu_ay, imu_az;
    wire [15:0] us_distance_mm;
    wire [23:0] demo_report_period;
    wire [31:0] demo_command_count, demo_rx_errors;
    wire [7:0] demo_last_command;
    wire [7:0] demo_rx_byte;
    wire demo_rx_byte_valid;
    wire [1023:0] demo_display_text;
    wire demo_display_text_valid;
    wire imu_text_byte_valid,imu_text_frame_commit,imu_text_byte_ready;
    wire [7:0] imu_text_byte_data,imu_text_byte_index,imu_lcd_character;
    wire imu_lcd_read_en;
    wire [31:0] bluetooth_tx_lines;
    wire touch_stop_pulse;
    wire pid_page, pid_action_valid, pid_emergency_stop;
    wire [7:0] pid_action;
    wire motor_uart_action_valid, motor_uart_emergency, motor_uart_heartbeat;
    wire [7:0] motor_uart_action;
    wire signed [31:0] motor_uart_argument;
    wire [31:0] motor_parse_errors;
    wire motor_armed, motor_running, motor_mode_position, motor_position_unlocked, motor_link_fault;
    wire motor_remote_owner;
    wire vofa_cmd_valid,vofa_cmd_ready,vofa_rsp_valid,vofa_rsp_ready;
    wire [1:0] vofa_cmd_wheel,vofa_rsp_wheel;
    wire [15:0] vofa_cmd_seq,vofa_rsp_seq;
    wire [3:0] vofa_cmd_op,vofa_rsp_op;
    wire [3:0] vofa_cmd_field,vofa_rsp_field;
    wire signed [31:0] vofa_cmd_value,vofa_rsp_value;
    wire [7:0] vofa_rsp_status;
    wire vofa_action_valid,vofa_live_update,vofa_heartbeat,vofa_takeover,vofa_emergency;
    wire [7:0] vofa_action;
    wire signed [31:0] vofa_argument;
    wire [31:0] vofa_parse_errors,vofa_tx_lines,legacy_bluetooth_lines;
    wire legacy_bluetooth_tx,vofa_tx;
    wire [113:0] imu_processed_axes;
    wire imu_processed_valid;
    wire [127:0] vofa_sample_values;
    wire vofa_turn_request,robot_turn_ready,robot_turn_active,robot_turn_complete_valid,robot_turn_complete_ready;
    wire robot_turn_report_busy;
    wire signed [15:0] vofa_turn_degrees;
    wire vofa_task_valid,vofa_task_ready,vofa_task_rsp_valid;
    wire [3:0] vofa_task_action;
    wire signed [31:0] vofa_task_value,vofa_task_rsp_value;
    wire [7:0] vofa_task_rsp_status;
    wire vofa_tx_byte_valid,vofa_tx_byte_ready,vofa_tx_busy;
    wire [7:0] vofa_tx_byte;
    wire robot_local_drive;
    wire motor_clear_fault, motor_clear_stop, motor_zero_position;
    wire motor_selected_wheel, motor_edit_position;
    wire [3:0] motor_selected_field;
    wire [1:0] motor_step_select;
    wire motor_invert_left, motor_invert_right;
    wire signed [15:0] motor_target_speed_left, motor_target_speed_right;
    wire signed [31:0] motor_target_position_left, motor_target_position_right;
    wire [15:0] motor_skp_l, motor_ski_l, motor_skd_l, motor_skp_r, motor_ski_r, motor_skd_r;
    wire [15:0] motor_pkp_l, motor_pki_l, motor_pkd_l, motor_pkp_r, motor_pki_r, motor_pkd_r;
    wire [9:0] motor_max_duty, motor_min_duty;
    wire [15:0] motor_speed_db, motor_position_db;
    wire signed [31:0] motor_selected_value;
    wire [31:0] motor_selected_step;
    wire signed [31:0] motor_position_left, motor_position_right;
    wire signed [31:0] motor_speed_left, motor_speed_right, motor_speed_raw_left, motor_speed_raw_right;
    wire signed [15:0] motor_effective_left, motor_effective_right, motor_duty_left, motor_duty_right;
    wire [7:0] motor_fault_left, motor_fault_right;
    wire [15:0] motor_invalid_left, motor_invalid_right;
    wire motor_tick;
    wire [2047:0] motor_text;
    wire motor_text_valid;
    wire motor_byte_valid,motor_byte_ready;
    wire [7:0] motor_byte_data,motor_byte_index;
    wire test_emergency_stop, test_clear_stop;
    wire test_pwma,test_ain1,test_ain2,test_pwmb,test_bin1,test_bin2;
    wire pid_pwma,pid_ain1,pid_ain2,pid_pwmb,pid_bin1,pid_bin2;
    wire [2047:0] test_status_text;
    wire [15:0] test_status_flags;
    wire test_status_valid;
    wire [2:0] robot_page;
    wire robot_pid_action_valid,robot_emergency_stop,robot_clear_stop,robot_drive_enable,robot_armed;
    wire follow_mailbox_request,follow_mailbox_ack,follow_pan_pwm,follow_tilt_pwm,test_servo0,test_servo1;
    wire [255:0] follow_mailbox_data,follow_telemetry,follow_laser_telemetry;
    wire pi_shutdown_requested,fpga_software_stopped;
    assign servo_0=ACTUATOR_TEST!=0 ? test_servo0 : ROBOT_ACTIVE ? follow_tilt_pwm : 1'b0;
    assign servo_1=ACTUATOR_TEST!=0 ? test_servo1 : ROBOT_ACTIVE ? follow_pan_pwm : 1'b0;
    wire [7:0] robot_pid_action;
    wire signed [15:0] robot_target_left,robot_target_right;
    wire [2047:0] robot_text;
    wire [31:0] robot_flags;
    wire robot_text_valid,robot_lidar_valid;
    wire robot_byte_valid,robot_byte_ready;
    wire [7:0] robot_byte_data,robot_byte_index;
    wire [2:0] robot_subpage;
    wire signed [31:0] robot_pid_argument;
    wire motor_drive_invert_left,motor_drive_invert_right;
    wire robot_calibration_enable;
    wire signed [15:0] robot_calibration_duty_left,robot_calibration_duty_right;
    wire motor_lcd_byte_ready,robot_lcd_byte_ready;
    wire [7:0] motor_lcd_character,robot_lcd_character,lcd_text_read_index;
    wire motor_lcd_read_en,robot_lcd_read_en;
    wire [31:0] robot_lcd_meta;
    wire [15:0] robot_lidar_front;
    wire [7:0] robot_lidar_error;
    wire [31:0] robot_lidar_scans;
    wire robot_status_ack,map_read_en,map_read_valid;
    wire [6:0] map_read_x,map_read_y;
    wire [1:0] map_read_cell;
    wire [39:0] map_car_model;
    reg [2079:0] robot_status_mailbox;
    reg robot_status_request;
    reg [2:0] robot_status_ack_sync;
    always @(posedge clk50M or negedge ready)begin
        if(!ready)begin robot_status_mailbox<=0;robot_status_request<=0;robot_status_ack_sync<=0;end
        else begin
            robot_status_ack_sync<={robot_status_ack_sync[1:0],robot_status_ack};
            if(!ROBOT_ACTIVE && robot_text_valid && robot_status_request==robot_status_ack_sync[2])begin
                robot_status_mailbox<={robot_flags,robot_text};robot_status_request<=~robot_status_request;
            end
        end
    end
    wire bq_info;
    reg [2064:0] test_status_mailbox;
    reg test_status_request;
    reg [2:0] test_status_ack_sync;
    wire test_status_ack;
    wire bq_valid_a,bq_valid_b;
    wire [7:0] bq_soc_a,bq_soc_b,bq_error_a,bq_error_b;
    wire [15:0] bq_voltage_a,bq_voltage_b;
    wire signed [15:0] bq_current_a,bq_current_b;
    wire [15:0] bq_temperature_a,bq_temperature_b,bq_flags_a,bq_flags_b;
    wire [15:0] bq_internal_a,bq_internal_b,bq_pack_a,bq_pack_b;
    wire bq_setup_pulse,bq_setup_busy_a,bq_setup_busy_b,bq_configured_a,bq_configured_b;
    wire [7:0] bq_setup_error_a,bq_setup_error_b,bq_setup_phase_a,bq_setup_phase_b;
    wire [15:0] bq_control_a,bq_control_b,bq_df_pack_a,bq_df_pack_b,bq_divider_a,bq_divider_b;
    wire [15:0] bq_debug_control_a,bq_debug_control_b,bq_debug_voltage_a,bq_debug_voltage_b,bq_firmware_a,bq_firmware_b;
    wire [7:0] bq_cells_a,bq_cells_b,bq_failure_a,bq_failure_b;
    wire bq_debug_valid_a,bq_debug_valid_b;
    wire [31:0] bq_samples_a,bq_samples_b;
    wire [1919:0] battery_text;
    wire [1:0] battery_valid_flags;
    wire battery_text_valid,battery_status_ack;
    reg [1921:0] battery_status_mailbox;
    reg battery_status_request;
    reg [2:0] battery_status_ack_sync;
    always @(posedge clk50M or negedge ready) begin
        if (!ready) begin
            battery_status_mailbox<=0;battery_status_request<=0;battery_status_ack_sync<=0;
        end else begin
            battery_status_ack_sync<={battery_status_ack_sync[1:0],battery_status_ack};
            if (battery_text_valid && battery_status_request==battery_status_ack_sync[2]) begin
                battery_status_mailbox<={battery_valid_flags,battery_text};
                battery_status_request<=~battery_status_request;
            end
        end
    end
    bq34z100_gauge #(.ENABLE_SETUP(!ROBOT_ACTIVE)) u_bq_a (
        .clk(clk50M),.reset_n(ready),.scl(bq_a_scl),.sda(bq_a_sda),
        .begin_setup(bq_setup_pulse),.setup_busy(bq_setup_busy_a),.configured(bq_configured_a),
        .setup_error(bq_setup_error_a),.setup_phase(bq_setup_phase_a),
        .control_status(bq_control_a),.debug_df_pack(bq_df_pack_a),.debug_divider(bq_divider_a),.debug_cells(bq_cells_a),
        .debug_control(bq_debug_control_a),.debug_voltage(bq_debug_voltage_a),.debug_firmware(bq_firmware_a),
        .debug_failure_phase(bq_failure_a),.debug_valid(bq_debug_valid_a),
        .valid(bq_valid_a),.soc(bq_soc_a),.voltage_mv(bq_voltage_a),.current_ma(bq_current_a),
        .temperature_dK(bq_temperature_a),.internal_temperature_dK(bq_internal_a),.pack_configuration(bq_pack_a),
        .flags(bq_flags_a),.error_code(bq_error_a),.sample_count(bq_samples_a)
    );
    bq34z100_gauge #(.ENABLE_SETUP(!ROBOT_ACTIVE)) u_bq_b (
        .clk(clk50M),.reset_n(ready),.scl(bq_b_scl),.sda(bq_b_sda),
        .begin_setup(bq_setup_pulse),.setup_busy(bq_setup_busy_b),.configured(bq_configured_b),
        .setup_error(bq_setup_error_b),.setup_phase(bq_setup_phase_b),
        .control_status(bq_control_b),.debug_df_pack(bq_df_pack_b),.debug_divider(bq_divider_b),.debug_cells(bq_cells_b),
        .debug_control(bq_debug_control_b),.debug_voltage(bq_debug_voltage_b),.debug_firmware(bq_firmware_b),
        .debug_failure_phase(bq_failure_b),.debug_valid(bq_debug_valid_b),
        .valid(bq_valid_b),.soc(bq_soc_b),.voltage_mv(bq_voltage_b),.current_ma(bq_current_b),
        .temperature_dK(bq_temperature_b),.internal_temperature_dK(bq_internal_b),.pack_configuration(bq_pack_b),
        .flags(bq_flags_b),.error_code(bq_error_b),.sample_count(bq_samples_b)
    );
    bq34z100_status_text #(.MINIMAL(ROBOT_ACTIVE)) u_bq_text (
        .clk(clk50M),.reset_n(ready),.valid_a(bq_valid_a),.valid_b(bq_valid_b),
        .soc_a(bq_soc_a),.soc_b(bq_soc_b),.error_a(bq_error_a),.error_b(bq_error_b),
        .temperature_a(bq_temperature_a),.temperature_b(bq_temperature_b),
        .internal_a(bq_internal_a),.internal_b(bq_internal_b),.pack_a(bq_pack_a),.pack_b(bq_pack_b),
        .setup_busy_a(bq_setup_busy_a),.setup_busy_b(bq_setup_busy_b),
        .configured_a(bq_configured_a),.configured_b(bq_configured_b),
        .setup_error_a(bq_setup_error_a),.setup_error_b(bq_setup_error_b),
        .setup_phase_a(bq_setup_phase_a),.setup_phase_b(bq_setup_phase_b),
        .control_a(bq_control_a),.control_b(bq_control_b),.df_pack_a(bq_df_pack_a),.df_pack_b(bq_df_pack_b),
        .divider_a(bq_divider_a),.divider_b(bq_divider_b),.firmware_a(bq_firmware_a),.firmware_b(bq_firmware_b),
        .setup_voltage_a(bq_debug_voltage_a),.setup_voltage_b(bq_debug_voltage_b),
        .cells_a(bq_cells_a),.cells_b(bq_cells_b),.failure_a(bq_failure_a),.failure_b(bq_failure_b),
        .debug_valid_a(bq_debug_valid_a),.debug_valid_b(bq_debug_valid_b),
        .voltage_a(bq_voltage_a),.voltage_b(bq_voltage_b),.text_rows(battery_text),
        .valid_flags(battery_valid_flags),.text_valid(battery_text_valid)
    );
    always @(posedge clk50M or negedge ready) begin
        if (!ready) begin
            test_status_mailbox<=0;test_status_request<=0;test_status_ack_sync<=0;
        end else begin
            test_status_ack_sync<={test_status_ack_sync[1:0],test_status_ack};
            if (test_status_valid && test_status_request==test_status_ack_sync[2]) begin
                test_status_mailbox<={bq_info,test_status_flags,test_status_text};
                test_status_request<=~test_status_request;
            end
        end
    end
    assign motor_pwma=ACTUATOR_TEST ? test_pwma : pid_pwma;
    assign motor_ain1=ACTUATOR_TEST ? test_ain1 : pid_ain1;
    assign motor_ain2=ACTUATOR_TEST ? test_ain2 : pid_ain2;
    assign motor_pwmb=ACTUATOR_TEST ? test_pwmb : pid_pwmb;
    assign motor_bin1=ACTUATOR_TEST ? test_bin1 : pid_bin1;
    assign motor_bin2=ACTUATOR_TEST ? test_bin2 : pid_bin2;
    // STBY is physically tied high on the submitted PCB: IN00/PWM1 coast STOP
    // is a software gate, not a power disconnect or configuration-time guarantee.
    wire motor_safety_stop=stop_latched || !reset_n || pid_emergency_stop || motor_uart_emergency || vofa_emergency ||
        robot_emergency_stop || (ROBOT_ACTIVE && !button4_n);
    assign robot_local_drive=robot_drive_enable && !(motor_remote_owner && motor_armed && motor_running);
    wire motor_request_now=!ACTUATOR_TEST && ((motor_armed && motor_running) ||
        (ROBOT_ACTIVE && robot_local_drive));
    wire selected_mode_now=robot_local_drive ? 1'b0 : motor_mode_position;
    reg selected_request_q,selected_owner_q,selected_mode_q;
    reg signed [15:0] selected_speed_left_q,selected_speed_right_q;
    // One coherent target pair breaks the measured PID-gain readiness ->
    // navigation ownership -> axis abs/clamp path. A new owner/mode waits one
    // 20ns clock for its own pair; RAW STOP/fault still disables the bridge.
    always @(posedge clk50M or negedge ready)begin
        if(!ready)begin
            selected_request_q<=0;selected_owner_q<=0;selected_mode_q<=0;
            selected_speed_left_q<=0;selected_speed_right_q<=0;
        end else begin
            selected_request_q<=motor_request_now;
            selected_owner_q<=robot_local_drive;selected_mode_q<=selected_mode_now;
            selected_speed_left_q<=robot_local_drive ? robot_target_left : motor_target_speed_left;
            selected_speed_right_q<=robot_local_drive ? robot_target_right : motor_target_speed_right;
        end
    end
    wire selected_snapshot_ready=selected_request_q && selected_owner_q==robot_local_drive && selected_mode_q==selected_mode_now;
    // Unimplemented peripherals remain inactive; module SHUT is only pulled low.
    assign laser_shut=4'b0000;
    assign step_0=1'b0; assign step_1=1'b0;
    assign dir_0=1'b0; assign dir_1=1'b0;
    // Robot tuning now has one framed command decoder: VOFA. Retain the old
    // @ command set only in the legacy profile, rather than route two parsers
    // and both action/argument buses to the same settings owner. The shared
    // UART receiver and VOFA's immediate '!' emergency path remain active.
    generate if(ROBOT_ACTIVE)begin:vofa_only_commands
        assign motor_uart_action_valid=1'b0;
        assign motor_uart_action=8'd0;
        assign motor_uart_argument=32'sd0;
        assign motor_uart_emergency=1'b0;
        assign motor_uart_heartbeat=1'b0;
        assign motor_parse_errors=32'd0;
    end else begin:legacy_motor_commands
        motor_uart_commands u_motor_commands (
            .clk(clk50M), .reset_n(ready), .byte_valid(demo_rx_byte_valid), .byte_data(demo_rx_byte),
            .action_valid(motor_uart_action_valid), .action(motor_uart_action), .argument(motor_uart_argument),
            .emergency_stop_pulse(motor_uart_emergency), .heartbeat(motor_uart_heartbeat), .parse_errors(motor_parse_errors)
        );
    end endgenerate
    wire settings_ui_valid=ROBOT_ACTIVE ? robot_pid_action_valid : pid_action_valid;
    wire [7:0] settings_ui_action=ROBOT_ACTIVE ? robot_pid_action : pid_action;
    wire settings_ui_stop=settings_ui_valid && (settings_ui_action==10 || settings_ui_action==11);
    motor_tuning_control u_motor_settings (
        .clk(clk50M), .reset_n(ready && !ACTUATOR_TEST), .safety_stop(motor_safety_stop),
        .fault_left(motor_fault_left), .fault_right(motor_fault_right),
        .ui_action_valid(settings_ui_valid && (vofa_cmd_ready || settings_ui_stop)),
        .ui_action(settings_ui_action),
        .ui_argument(ROBOT_ACTIVE ? robot_pid_argument : 32'sd0),
        .uart_action_valid(vofa_action_valid || (vofa_cmd_ready && !vofa_cmd_valid && motor_uart_action_valid && (!ROBOT_ACTIVE || (robot_page==1 && !robot_calibration_enable)))),
        // Core mode already compiles out the legacy command producer. Do not
        // put fault-qualified VALID back into this32bit argument data path.
        .uart_action(ROBOT_ACTIVE ? vofa_action : (vofa_action_valid ? vofa_action : motor_uart_action)),
        .uart_argument(ROBOT_ACTIVE ? vofa_argument : (vofa_action_valid ? vofa_argument : motor_uart_argument)),
        .uart_heartbeat(motor_uart_heartbeat || vofa_heartbeat),
        .uart_live_update(vofa_action_valid && vofa_live_update),
        .armed(motor_armed), .running(motor_running), .mode_position(motor_mode_position),
        .position_unlocked(motor_position_unlocked), .link_fault(motor_link_fault),.remote_owner(motor_remote_owner),
        .clear_fault(motor_clear_fault), .clear_stop(motor_clear_stop), .zero_position(motor_zero_position),
        .selected_wheel(motor_selected_wheel), .edit_position(motor_edit_position),
        .selected_field(motor_selected_field), .step_select(motor_step_select),
        .encoder_invert_left(motor_invert_left), .encoder_invert_right(motor_invert_right),
        .drive_invert_left(motor_drive_invert_left),.drive_invert_right(motor_drive_invert_right),
        .target_speed_left(motor_target_speed_left), .target_speed_right(motor_target_speed_right),
        .target_position_left(motor_target_position_left), .target_position_right(motor_target_position_right),
        .speed_kp_left(motor_skp_l), .speed_ki_left(motor_ski_l), .speed_kd_left(motor_skd_l),
        .speed_kp_right(motor_skp_r), .speed_ki_right(motor_ski_r), .speed_kd_right(motor_skd_r),
        .position_kp_left(motor_pkp_l), .position_ki_left(motor_pki_l), .position_kd_left(motor_pkd_l),
        .position_kp_right(motor_pkp_r), .position_ki_right(motor_pki_r), .position_kd_right(motor_pkd_r),
        .max_duty_permille(motor_max_duty), .min_duty_permille(motor_min_duty),
        .speed_deadband(motor_speed_db), .position_deadband(motor_position_db),
        .selected_value(motor_selected_value), .selected_step(motor_selected_step)
    );
    motor_drive_control u_motors (
        .clk(clk50M), .reset_n(ready), .safety_stop(motor_safety_stop || motor_link_fault),
        .run_enable(motor_request_now && (!ROBOT_ACTIVE || selected_snapshot_ready)), .clear_fault(motor_clear_fault),
        .zero_position(motor_zero_position), .encoder_invert_left(motor_invert_left),
        .encoder_invert_right(motor_invert_right), .mode_position(ROBOT_ACTIVE ? selected_mode_q : selected_mode_now),
        .drive_invert_left(motor_drive_invert_left),.drive_invert_right(motor_drive_invert_right),
        .calibration_enable(ROBOT_ACTIVE && robot_calibration_enable),
        .calibration_duty_left(robot_calibration_duty_left),.calibration_duty_right(robot_calibration_duty_right),
        .encoder1_a(encoder1_a), .encoder1_b(encoder1_b), .encoder2_a(encoder2_a), .encoder2_b(encoder2_b),
        .target_speed_left(ROBOT_ACTIVE ? selected_speed_left_q : (robot_local_drive ? robot_target_left : motor_target_speed_left)),
        .target_speed_right(ROBOT_ACTIVE ? selected_speed_right_q : (robot_local_drive ? robot_target_right : motor_target_speed_right)),
        .target_position_left(motor_target_position_left), .target_position_right(motor_target_position_right),
        .speed_kp_left(motor_skp_l), .speed_ki_left(motor_ski_l), .speed_kd_left(motor_skd_l),
        .speed_kp_right(motor_skp_r), .speed_ki_right(motor_ski_r), .speed_kd_right(motor_skd_r),
        .position_kp_left(motor_pkp_l), .position_ki_left(motor_pki_l), .position_kd_left(motor_pkd_l),
        .position_kp_right(motor_pkp_r), .position_ki_right(motor_pki_r), .position_kd_right(motor_pkd_r),
        .speed_deadband(motor_speed_db), .position_deadband(motor_position_db),
        .max_duty_permille(motor_max_duty), .min_duty_permille(motor_min_duty),
        .pwma(pid_pwma), .ain1(pid_ain1), .ain2(pid_ain2),
        .pwmb(pid_pwmb), .bin1(pid_bin1), .bin2(pid_bin2),
        .position_left(motor_position_left), .position_right(motor_position_right),
        .speed_left(motor_speed_left), .speed_right(motor_speed_right),
        .speed_raw_left(motor_speed_raw_left), .speed_raw_right(motor_speed_raw_right),
        .target_effective_left(motor_effective_left), .target_effective_right(motor_effective_right),
        .duty_left(motor_duty_left), .duty_right(motor_duty_right),
        .fault_left(motor_fault_left), .fault_right(motor_fault_right),
        .invalid_left(motor_invalid_left), .invalid_right(motor_invalid_right), .tick_10ms(motor_tick)
    );
    generate if(!ROBOT_ACTIVE)begin:legacy_motor_report
    motor_status_text #(.COMPACT_ROBOT(0)) u_motor_text (
        .clk(clk50M), .reset_n(ready), .armed(motor_armed || robot_armed), .running(motor_running || robot_local_drive),
        .safety_stop(motor_safety_stop), .link_fault(motor_link_fault),
        .mode_position(motor_mode_position), .position_unlocked(motor_position_unlocked),
        .selected_wheel(motor_selected_wheel), .edit_position(motor_edit_position),
        .selected_field(motor_selected_field), .selected_value(motor_selected_value), .selected_step(motor_selected_step),
        .target_effective_left(motor_effective_left), .target_effective_right(motor_effective_right),
        .speed_left(motor_speed_left), .speed_right(motor_speed_right),
        .position_left(motor_position_left), .position_right(motor_position_right),
        .duty_left(motor_duty_left), .duty_right(motor_duty_right),
        .fault_left(motor_fault_left), .fault_right(motor_fault_right),
        .invalid_left(motor_invalid_left), .invalid_right(motor_invalid_right),
        .text_rows(motor_text), .text_valid(motor_text_valid),
        .text_byte_valid(motor_byte_valid),.text_byte_data(motor_byte_data),.text_byte_index(motor_byte_index),
        // The TEST report uses six IMU rows and wheel speeds, not PID text.
        // Drain this optional producer even when its UART store is in reset.
        .text_byte_ready(ACTUATOR_TEST ? 1'b1 :
            (motor_byte_ready && (!ROBOT_ACTIVE || motor_lcd_byte_ready)))
    );
    end else begin:core_no_motor_dashboard
        assign motor_text=2048'd0;assign motor_text_valid=1'b0;
        assign motor_byte_valid=1'b0;assign motor_byte_data=8'd0;assign motor_byte_index=8'd0;
    end endgenerate
    sensor_demo_commands u_demo_commands (
        .clk(clk50M), .reset_n(ready), .rx(bluetooth_rx),
        .period_cycles(demo_report_period), .command_count(demo_command_count),
        .last_command(demo_last_command), .rx_errors(demo_rx_errors),
        .rx_byte(demo_rx_byte), .rx_byte_valid(demo_rx_byte_valid)
    );
    lsm6dsv16x_demo u_imu_demo (
        .clk(clk50M), .reset_n(ready), .scl(sensor_scl), .sda(sensor_sda),
        .valid(imu_demo_valid), .who_am_i(imu_demo_who), .error_code(imu_demo_error),
        .sample_count(imu_demo_samples), .gx(imu_gx), .gy(imu_gy), .gz(imu_gz),
        .ax(imu_ax), .ay(imu_ay), .az(imu_az)
    );
    // Ultrasound removed from the active build; pins retained for PCB continuity.
    assign ultrasonic_trig=1'b0;
    assign us_demo_valid=ROBOT_ACTIVE && robot_lidar_valid;
    assign us_demo_error=robot_lidar_error;
    assign us_demo_samples=robot_lidar_scans;
    assign us_distance_mm=robot_lidar_front;
    generate if(!ROBOT_ACTIVE)begin:legacy_sensor_display
    sensor_demo_uart #(.ACTUATOR_TEST(ACTUATOR_TEST),.ROBOT_MODE(ROBOT_ACTIVE),.VOFA_MODE(ROBOT_ACTIVE)) u_sensor_demo_uart (
        .clk(clk50M), .reset_n(ready),
        .imu_valid(imu_demo_valid), .imu_who(imu_demo_who), .imu_error(imu_demo_error),
        .imu_samples(imu_demo_samples), .gx(imu_gx), .gy(imu_gy), .gz(imu_gz),
        .ax(imu_ax), .ay(imu_ay), .az(imu_az), .us_valid(us_demo_valid),
        .us_error(us_demo_error), .us_samples(us_demo_samples),
        .distance_mm(us_distance_mm), .period_cycles(demo_report_period),
        .command_count(demo_command_count), .last_command(demo_last_command),
        .rx_errors(demo_rx_errors), .tx(legacy_bluetooth_tx),
        .tx_line_count(legacy_bluetooth_lines),
        .processed_axes(imu_processed_axes),.processed_imu_valid(imu_processed_valid),
        .wheel_speed_left(motor_speed_left),.wheel_speed_right(motor_speed_right),
        .motor_byte_valid(!ACTUATOR_TEST && motor_byte_valid && (!ROBOT_ACTIVE || motor_lcd_byte_ready)),
        .motor_byte_data(motor_byte_data),.motor_byte_index(motor_byte_index),.motor_byte_ready(motor_byte_ready),
        .robot_byte_valid(ROBOT_ACTIVE && robot_byte_valid && robot_lcd_byte_ready),
        .robot_byte_data(robot_byte_data),.robot_byte_index(robot_byte_index),.robot_byte_ready(robot_byte_ready),
        .display_text(demo_display_text), .display_text_valid(demo_display_text_valid)
    );
        assign imu_text_byte_valid=1'b0;assign imu_text_frame_commit=1'b0;
        assign imu_text_byte_data=8'd0;assign imu_text_byte_index=8'd0;
    end else begin:core_imu_byte_display
        sensor_demo_text_compact u_imu_text(
            .clk(clk50M),.reset_n(ready),.imu_valid(imu_demo_valid && imu_demo_error==0),
            .gx(imu_gx),.gy(imu_gy),.gz(imu_gz),.ax(imu_ax),.ay(imu_ay),.az(imu_az),
            .text_valid(imu_text_frame_commit),.text_byte_valid(imu_text_byte_valid),
            .text_byte_data(imu_text_byte_data),.text_byte_index(imu_text_byte_index),.text_byte_ready(imu_text_byte_ready)
        );
        assign legacy_bluetooth_tx=1'b1;assign legacy_bluetooth_lines=32'd0;
        assign imu_processed_axes=114'd0;assign imu_processed_valid=1'b0;
        assign motor_byte_ready=1'b1;assign robot_byte_ready=1'b1;
        assign demo_display_text=1024'd0;assign demo_display_text_valid=1'b0;
    end endgenerate

    // Same physical Bluetooth UART, now FireWater samples plus command ACKs.
    // No text frames are spliced together and no floating-point hardware is used.
    assign bluetooth_tx=ROBOT_ACTIVE ? vofa_tx : legacy_bluetooth_tx;
    assign bluetooth_tx_lines=ROBOT_ACTIVE ? vofa_tx_lines : legacy_bluetooth_lines;
    assign vofa_sample_values[0*32+:32]=(robot_turn_active || robot_local_drive) ? {{16{robot_target_left[15]}},robot_target_left} :
        (motor_armed && motor_running) ? {{16{motor_target_speed_left[15]}},motor_target_speed_left} : 32'sd0;
    assign vofa_sample_values[1*32+:32]=motor_speed_left;
    assign vofa_sample_values[2*32+:32]=(robot_turn_active || robot_local_drive) ? {{16{robot_target_right[15]}},robot_target_right} :
        (motor_armed && motor_running) ? {{16{motor_target_speed_right[15]}},motor_target_speed_right} : 32'sd0;
    assign vofa_sample_values[3*32+:32]=motor_speed_right;
    // Only the four wheel-speed curves are streamed continuously at20Hz.
    robot_vofa_uart #(.CHANNELS(4)) u_vofa(
        .clk(clk50M),.reset_n(ready && ROBOT_ACTIVE),.byte_valid(demo_rx_byte_valid),.byte_data(demo_rx_byte),
        .emergency_stop_pulse(vofa_emergency),.cmd_valid(vofa_cmd_valid),.cmd_ready(vofa_cmd_ready),
        .cmd_seq(vofa_cmd_seq),.cmd_op(vofa_cmd_op),.cmd_wheel(vofa_cmd_wheel),.cmd_field(vofa_cmd_field),.cmd_value(vofa_cmd_value),
        .rsp_valid(vofa_rsp_valid),.rsp_ready(vofa_rsp_ready),.rsp_seq(vofa_rsp_seq),.rsp_op(vofa_rsp_op),
        .rsp_wheel(vofa_rsp_wheel),.rsp_field(vofa_rsp_field),.rsp_status(vofa_rsp_status),.rsp_value(vofa_rsp_value),
        .sample_values(vofa_sample_values),.tx_byte_valid(vofa_tx_byte_valid),.tx_byte_ready(vofa_tx_byte_ready),
        .turn_complete_valid(robot_turn_complete_valid),.turn_complete_ready(robot_turn_complete_ready),.turn_complete_busy(robot_turn_report_busy),
        .tx_byte_data(vofa_tx_byte),.tx_line_count(vofa_tx_lines),.parse_errors(vofa_parse_errors)
    );
    assign vofa_tx_byte_ready=!vofa_tx_busy;
    sensor_demo_uart_tx_8n1 u_vofa_tx(
        .clk(clk50M),.reset_n(ready && ROBOT_ACTIVE),.start(vofa_tx_byte_valid && vofa_tx_byte_ready),
        .data(vofa_tx_byte),.busy(vofa_tx_busy),.tx(vofa_tx)
    );
    motor_vofa_settings_adapter u_vofa_settings(
        .clk(clk50M),.reset_n(ready && ROBOT_ACTIVE),.safety_stop(motor_safety_stop),
        .cmd_valid(vofa_cmd_valid),.cmd_ready(vofa_cmd_ready),.cmd_seq(vofa_cmd_seq),.cmd_op(vofa_cmd_op),
        .cmd_wheel(vofa_cmd_wheel),.cmd_field(vofa_cmd_field),.cmd_value(vofa_cmd_value),
        .rsp_valid(vofa_rsp_valid),.rsp_ready(vofa_rsp_ready),.rsp_seq(vofa_rsp_seq),.rsp_op(vofa_rsp_op),
        .rsp_wheel(vofa_rsp_wheel),.rsp_field(vofa_rsp_field),.rsp_status(vofa_rsp_status),.rsp_value(vofa_rsp_value),
        .core_action_valid(vofa_action_valid),.core_action(vofa_action),.core_argument(vofa_argument),
        .core_live_update(vofa_live_update),.heartbeat(vofa_heartbeat),.takeover(vofa_takeover),
        .turn_request(vofa_turn_request),.turn_degrees(vofa_turn_degrees),.turn_ready(robot_turn_ready),.turn_active(robot_turn_active),
        .task_valid(vofa_task_valid),.task_action(vofa_task_action),.task_value(vofa_task_value),.task_ready(vofa_task_ready),
        .task_rsp_valid(vofa_task_rsp_valid),.task_rsp_status(vofa_task_rsp_status),.task_rsp_value(vofa_task_rsp_value),
        .core_stop_override(settings_ui_stop),
        .core_armed(motor_armed),.core_running(motor_running),.core_mode_position(motor_mode_position),
        .core_link_fault(motor_link_fault),.core_fault_left(motor_fault_left),.core_fault_right(motor_fault_right),
        .core_selected_value(motor_selected_value),.core_min_duty(motor_min_duty),.core_max_duty(motor_max_duty)
    );

    // Battery-backed RTC clock; no clock setting on reset or FPGA power-up.
    wire rtc_set_valid, rtc_set_busy, rtc_valid;
    wire [55:0] rtc_set_calendar;
    wire [23:0] rtc_time;
    wire [7:0] rtc_error;
    generate if(!ROBOT_ACTIVE)begin:legacy_clock_display
    uart_rtc_set_parser u_rtc_set_parser (
        .clk(clk50M), .reset_n(ready), .byte_valid(demo_rx_byte_valid),
        .byte_data(demo_rx_byte), .set_busy(rtc_set_busy),
        .set_valid(rtc_set_valid), .set_calendar(rtc_set_calendar)
    );
    bm8563_clock u_rtc_clock (
        .clk(clk50M), .reset_n(ready), .scl(IIC_scl), .sda(IIC_sda),
        .set_valid(rtc_set_valid), .set_calendar(rtc_set_calendar),
        .set_busy(rtc_set_busy), .valid(rtc_valid),
        .time_bcd(rtc_time), .error_code(rtc_error)
    );
    reg [25:0] seg_boot_count;
    always @(posedge clk50M or negedge ready) begin
        if (!ready) seg_boot_count <= 0;
        else if (seg_boot_count < 26'd50000000)
            seg_boot_count <= seg_boot_count + 1'b1;
    end
    wire [31:0] clock_digits = (seg_boot_count < 26'd50000000) ? 32'h88888888 :
        rtc_set_busy ? 32'hffffffff :
        rtc_valid ? {rtc_time[23:16], 4'hf, rtc_time[15:8], 4'hf, rtc_time[7:0]} :
        rtc_error[5] ? {4'he, 20'h00000, rtc_error} : 32'hffffffff;
    seg7_clock_display u_seg7_clock (
        .clk(clk50M), .reset_n(ready), .digits(clock_digits),
        .dio(SEG7_DIO), .rclk(SEG7_RCLK), .sclk(SEG7_SCLK)
    );
    end else begin:core_no_clock_demo
        assign IIC_scl=1'bz;assign IIC_sda=1'bz;
        assign SEG7_DIO=1'b0;assign SEG7_RCLK=1'b0;assign SEG7_SCLK=1'b0;
        assign rtc_set_valid=1'b0;assign rtc_set_busy=1'b0;assign rtc_valid=1'b0;
        assign rtc_set_calendar=56'd0;assign rtc_time=24'd0;assign rtc_error=8'd0;
    end endgenerate

    // S4 is debounced. Motor stop has priority; idle presses toggle the filter.
    reg button_meta = 1'b1, button_sync = 1'b1;
    reg button_stable = 1'b1;
    reg [19:0] button_count = 0;
    reg button_press = 1'b0;
    always @(posedge clk50M) begin
        button_meta <= button4_n;
        button_sync <= button_meta;
        button_press <= 1'b0;
        if (!reset_sync) begin
            button_stable <= 1'b1;
            button_count <= 0;
        end else if (button_sync == button_stable) begin
            button_count <= 0;
        end else if (button_count == 20'd999999) begin
            button_stable <= button_sync;
            button_count <= 0;
            if (!button_sync) button_press <= 1'b1;
        end else button_count <= button_count+1'b1;
    end

    wire pixel_clk, pixel_pll_locked;
    Gowin_PLL_45 u_pixel_pll (
        .clkin(clk50M), .clkout0(pixel_clk), .lock(pixel_pll_locked),
        .mdclk(clk50M), .reset(~reset_sync)
    );
    wire pixel_reset_n = ready && pixel_pll_locked;
    reg [1:0] pixel_run_pipe = 0;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) pixel_run_pipe <= 0;
        else pixel_run_pipe <= {pixel_run_pipe[0], 1'b1};
    end
    wire frame_restart;

    // Chapter 45's camera PLL output was 25 MHz despite its loc_clk24m name.
    // A board-clock divide-by-two yields that same OV5640 XCLK.
    reg camera_clk25 = 1'b0;
    always @(posedge clk50M or negedge reset_sync) begin
        if (!reset_sync) camera_clk25 <= 1'b0;
        else camera_clk25 <= ~camera_clk25;
    end
    assign camera_xclk = camera_clk25;
    wire camera_init_done;
    camera_init #(
        .SYS_CLOCK(50_000_000), .SCL_CLOCK(400_000),
        .CAMERA_TYPE("ov5640"), .IMAGE_TYPE(0),
        .IMAGE_WIDTH(800), .IMAGE_HEIGHT(480),
        .IMAGE_FLIP_EN(0), .IMAGE_MIRROR_EN(0)
    ) u_camera_init (
        .Clk(clk50M), .Rst_n(ready), .Init_Done(camera_init_done),
        .camera_rst_n(camera_rst_n), .camera_pwdn(camera_pwdn),
        .i2c_sclk(camera_sclk), .i2c_sdat(camera_sdat)
    );

    // The camera data clock and DDR user clock are independent of LCD PCLK.
    // Release capture only after SCCB setup and DDR calibration finish.
    wire init_calib_complete;
    wire camera_capture_reset_n = ready && camera_init_done && init_calib_complete;
    reg [1:0] camera_run_pipe = 0;
    always @(posedge camera_pclk or negedge camera_capture_reset_n) begin
        if (!camera_capture_reset_n) camera_run_pipe <= 0;
        else camera_run_pipe <= {camera_run_pipe[0], 1'b1};
    end
    wire camera_frame_start;
    wire camera_pixel_write;
    wire [15:0] camera_pixel_write_data;
    wire [10:0] camera_x, camera_y;
    DVP_Capture u_dvp (
        .Rst_n(camera_run_pipe[1]), .PCLK(camera_pclk),
        .Vsync(camera_vsync), .Href(camera_href), .Data(camera_data),
        .ImageState(camera_frame_start), .DataValid(camera_pixel_write),
        .DataPixel(camera_pixel_write_data),
        .DataHs(), .DataVs(), .Xaddr(camera_x), .Yaddr(camera_y)
    );
    reg filter_requested = 1'b1;
    wire debug_requested;
    reg stop_latched = 1'b0;
    wire gaussian_valid;
    wire [15:0] gaussian_pixel;
    wire [10:0] gaussian_x, gaussian_y;
    wire gaussian_frame_start;
    wire gaussian_active;
    reg [1:0] filter_camera_sync = 2'b11;
    always @(posedge camera_pclk or negedge camera_capture_reset_n) begin
        if (!camera_capture_reset_n) filter_camera_sync <= 2'b11;
        else filter_camera_sync <= {filter_camera_sync[0],filter_requested};
    end
    generate if(!ROBOT_ACTIVE)begin:legacy_camera_filter
    gaussian3x3_rgb565 u_gauss (
        .clk(camera_pclk), .reset_n(camera_run_pipe[1]),
        .frame_start(camera_frame_start),
        .enable_filter(filter_camera_sync[1]),
        .in_valid(camera_pixel_write), .in_pixel(camera_pixel_write_data),
        .in_x(camera_x), .in_y(camera_y),
        .out_valid(gaussian_valid), .out_pixel(gaussian_pixel),
        .out_x(gaussian_x), .out_y(gaussian_y),
        .out_frame_start(gaussian_frame_start),
        .active_filter(gaussian_active)
    );
    end else begin:core_raw_camera
        assign gaussian_valid=camera_pixel_write;assign gaussian_pixel=camera_pixel_write_data;
        assign gaussian_x=camera_x;assign gaussian_y=camera_y;
        assign gaussian_frame_start=camera_frame_start;assign gaussian_active=1'b0;
    end endgenerate
    reg camera_frame_seen = 1'b0;
    always @(posedge camera_pclk or negedge camera_capture_reset_n) begin
        if (!camera_capture_reset_n) camera_frame_seen <= 1'b0;
        else if (camera_frame_start) camera_frame_seen <= 1'b1;
    end
    reg [1:0] frame_seen_pixel = 0;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) frame_seen_pixel <= 0;
        else frame_seen_pixel <= {frame_seen_pixel[0], camera_frame_seen};
    end

    // DDR initialization owns mDRP first. Hand it to pll_mDRP_intf only
    // AFTER initialization has reported LOCK. Keep ownership latched until
    // S0 reset: a temporary LOCK drop must not restart PLL_INIT mid-command.
    wire ddr_pll_locked, ddr_clk400, pll_stop;
    wire mdrp_inc;
    wire [1:0] mdrp_op;
    wire [7:0] mdrp_wdata, mdrp_rdata;
    reg ddr_lock_meta = 0, ddr_lock_sync = 0;
    reg pll_stop_d = 0, mdrp_wr = 0;
    reg ddr_mdrp_user_mode = 1'b0;
    always @(posedge clk50M) begin
        if (!reset_sync) begin
            ddr_lock_meta <= 1'b0;
            ddr_lock_sync <= 1'b0;
            pll_stop_d <= 1'b0;
            mdrp_wr <= 1'b0;
            ddr_mdrp_user_mode <= 1'b0;
        end else begin
            ddr_lock_meta <= ddr_pll_locked;
            ddr_lock_sync <= ddr_lock_meta;
            pll_stop_d <= pll_stop;
            if (!ddr_lock_sync || !ddr_mdrp_user_mode) mdrp_wr <= 1'b0;
            else mdrp_wr <= pll_stop ^ pll_stop_d;
            if (ddr_lock_sync) ddr_mdrp_user_mode <= 1'b1;
        end
    end
    pll_mDRP_intf u_ddr_phase_control (
        .clk(clk50M), .rst_n(reset_sync && ddr_mdrp_user_mode),
        .pll_lock(ddr_lock_sync),
        .wr(mdrp_wr), .mdrp_inc(mdrp_inc), .mdrp_op(mdrp_op),
        .mdrp_wdata(mdrp_wdata), .mdrp_rdata(mdrp_rdata)
    );
    ddr_pll u_ddr_pll (
        .clkin(clk50M), .clkout0(), .clkout1(), .clkout2(ddr_clk400),
        .lock(ddr_pll_locked), .mdopc(mdrp_op), .mdainc(mdrp_inc),
        .mdwdi(mdrp_wdata), .mdrdo(mdrp_rdata),
        .pll_init_bypass(ddr_mdrp_user_mode),
        .mdclk(clk50M), .reset(~reset_sync)
    );

    wire [15:0] camera_pixel_read;
    wire rdfifo_empty;
    wire camera_pop;
    wire [15:0] rgb_out;
    wire frame_heartbeat;
    wire camera_underflow;
    wire [8:0] lcd_view_step;
    wire [9:0] lcd_view_left;
    wire [8:0] lcd_view_top;
    wire [27:0] lcd_ddr_read_base;
    // The base changes at the LCD boundary. Delay rd_load to allow the
    // stable address bus to reach the existing DDR adapter before its flush.
    reg [15:0] rd_restart_delay=0;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) rd_restart_delay<=0;
        else rd_restart_delay<={rd_restart_delay[14:0],frame_restart};
    end
    // Actual frame-latched geometry travels back as a coherent toggle bundle;
    // touch selection must not use a request which is still waiting to display.
    reg [31:0] lcd_view_mailbox=32'h08000000;
    reg lcd_frame_zoom_mode=0;
    reg lcd_view_request=0;
    reg [2:0] lcd_view_ack_sync=0;
    wire lcd_view_ack;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) begin
            lcd_view_mailbox<=32'h08000000; lcd_view_request<=0; lcd_view_ack_sync<=0; lcd_frame_zoom_mode<=0;
        end else begin
            lcd_view_ack_sync<={lcd_view_ack_sync[1:0],lcd_view_ack};
            if (frame_restart) lcd_frame_zoom_mode<=ui_lcd_data[28];
            if (rd_restart_delay[0] && lcd_view_request==lcd_view_ack_sync[2]) begin
                lcd_view_mailbox<={3'd0,lcd_frame_zoom_mode,lcd_view_step,lcd_view_top,lcd_view_left};
                lcd_view_request<=~lcd_view_request;
            end
        end
    end
    reg [2:0] lcd_view_request_sync=0;
    reg lcd_view_seen=0;
    reg [31:0] lcd_view_data=32'h08000000;
    assign lcd_view_ack=lcd_view_seen;
    always @(posedge clk50M or negedge ready) begin
        if (!ready) begin lcd_view_request_sync<=0; lcd_view_seen<=0; lcd_view_data<=32'h08000000; end
        else begin
            lcd_view_request_sync<={lcd_view_request_sync[1:0],lcd_view_request};
            if (lcd_view_request_sync[2]!=lcd_view_seen) begin
                lcd_view_seen<=lcd_view_request_sync[2]; lcd_view_data<=lcd_view_mailbox;
            end
        end
    end
    reg [1:0] gaussian_pixel_sync = 2'b11;
    reg [1:0] debug_pixel_sync = 2'b00;
    reg [1:0] stop_pixel_sync = 2'b00;
    reg [1:0] camera_pixel_sync = 2'b00;
    reg [1:0] ddr_pixel_sync = 2'b00;
    reg [1:0] ddr_pll_pixel_sync = 2'b00;
    reg [1:0] net_pixel_sync = 2'b00;
    wire net_ready;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) begin
            gaussian_pixel_sync<=2'b11; debug_pixel_sync<=0;
            stop_pixel_sync<=0; camera_pixel_sync<=0;
            ddr_pixel_sync<=0; ddr_pll_pixel_sync<=0; net_pixel_sync<=0;
        end else begin
            gaussian_pixel_sync<={gaussian_pixel_sync[0],gaussian_active};
            debug_pixel_sync<={debug_pixel_sync[0],debug_requested};
            stop_pixel_sync<={stop_pixel_sync[0],stop_latched};
            camera_pixel_sync<={camera_pixel_sync[0],camera_init_done};
            ddr_pixel_sync<={ddr_pixel_sync[0],init_calib_complete};
            ddr_pll_pixel_sync<={ddr_pll_pixel_sync[0],ddr_pll_locked};
            net_pixel_sync<={net_pixel_sync[0],net_ready};
        end
    end
    wire detection_request, detection_ack;
    wire [255:0] detection_mailbox;
    wire [1023:0] sensor_text_mailbox;
    wire sensor_text_request;
    wire sensor_text_ack;
    generate if(!ROBOT_ACTIVE)begin:legacy_sensor_mailbox
    reg [1023:0] held_sensor_text;
    reg held_sensor_request;
    reg [2:0] sensor_text_ack_sync;
    assign sensor_text_mailbox=held_sensor_text;
    assign sensor_text_request=held_sensor_request;
    always @(posedge clk50M or negedge ready) begin
        if (!ready) begin
            held_sensor_text<=0; held_sensor_request<=0; sensor_text_ack_sync<=0;
        end else begin
            sensor_text_ack_sync<={sensor_text_ack_sync[1:0],sensor_text_ack};
            if (demo_display_text_valid && held_sensor_request==sensor_text_ack_sync[2]) begin
                held_sensor_text<=demo_display_text;
                held_sensor_request<=~held_sensor_request;
            end
        end
    end
    end else begin:core_no_parallel_sensor_mailbox
        assign sensor_text_mailbox=1024'd0;assign sensor_text_request=1'b0;
    end endgenerate
    reg [2047:0] pid_text_mailbox;
    reg pid_text_request;
    wire pid_text_ack;
    reg [2:0] pid_text_ack_sync;
    always @(posedge clk50M or negedge ready) begin
        if (!ready) begin pid_text_mailbox<=0; pid_text_request<=0; pid_text_ack_sync<=0; end
        else begin
            pid_text_ack_sync<={pid_text_ack_sync[1:0],pid_text_ack};
            if (!ROBOT_ACTIVE && motor_text_valid && pid_text_request==pid_text_ack_sync[2]) begin
                pid_text_mailbox<=motor_text; pid_text_request<=~pid_text_request;
            end
        end
    end
    // Each snapshot owns two physically separate RAM banks. The formatter
    // byte is accepted by UART and LCD together; LCD ready is independent of
    // valid/UART ready, so this join has no combinational handshake loop.
    // A PLL reset resets BOTH RAM ownership domains; a raster pause only
    // gates reads and never discards the active-bank/ACK relationship.
    lcd_text_byte_snapshot u_motor_lcd_text (
        .write_clk(clk50M),.reset_n(pixel_reset_n && ROBOT_ACTIVE),
        .byte_valid(ROBOT_ACTIVE && motor_byte_valid && motor_byte_ready),
        .byte_data(motor_byte_data),.byte_index(motor_byte_index),.byte_ready(motor_lcd_byte_ready),
        .frame_commit(motor_text_valid),.frame_meta(32'd0),
        .read_clk(pixel_clk),.read_run(pixel_run_pipe[1]),.frame_restart(frame_restart),
        .read_en(motor_lcd_read_en),.read_index(lcd_text_read_index),
        .read_byte(motor_lcd_character),.read_meta(),.read_seen()
    );
    lcd_text_byte_snapshot u_robot_lcd_text (
        .write_clk(clk50M),.reset_n(pixel_reset_n && ROBOT_ACTIVE),
        .byte_valid(ROBOT_ACTIVE && robot_byte_valid && robot_byte_ready),
        .byte_data(robot_byte_data),.byte_index(robot_byte_index),.byte_ready(robot_lcd_byte_ready),
        .frame_commit(robot_text_valid),.frame_meta(robot_flags),
        .read_clk(pixel_clk),.read_run(pixel_run_pipe[1]),.frame_restart(frame_restart),
        .read_en(robot_lcd_read_en),.read_index(lcd_text_read_index),
        .read_byte(robot_lcd_character),.read_meta(robot_lcd_meta),.read_seen()
    );
    lcd_text_byte_snapshot u_imu_lcd_text (
        .write_clk(clk50M),.reset_n(pixel_reset_n && ROBOT_ACTIVE),
        .byte_valid(ROBOT_ACTIVE && imu_text_byte_valid),
        .byte_data(imu_text_byte_data),.byte_index(imu_text_byte_index),.byte_ready(imu_text_byte_ready),
        .frame_commit(imu_text_frame_commit),.frame_meta(32'd0),
        .read_clk(pixel_clk),.read_run(pixel_run_pipe[1]),.frame_restart(frame_restart),
        .read_en(imu_lcd_read_en),.read_index(lcd_text_read_index),
        .read_byte(imu_lcd_character),.read_meta(),.read_seen()
    );
    lcd1024_interaction_ui #(.BYTE_TEXT_MODE(ROBOT_ACTIVE)) u_lcd (
        .clk(pixel_clk), .run(pixel_run_pipe[1]),
        .robot_byte_character(robot_lcd_character),.pid_byte_character(motor_lcd_character),
        .sensor_byte_character(imu_lcd_character),.sensor_byte_read_en(imu_lcd_read_en),
        .robot_byte_meta(robot_lcd_meta),.robot_byte_read_en(robot_lcd_read_en),
        .pid_byte_read_en(motor_lcd_read_en),.byte_text_read_index(lcd_text_read_index),
        .camera_pixel(camera_pixel_read),
        .camera_pixel_valid(!rdfifo_empty && frame_seen_pixel[1]),
        .camera_ready(camera_pixel_sync[1]),
        .ddr_ready(ddr_pixel_sync[1]),
        .ddr_pll_ready(ddr_pll_pixel_sync[1]),
        .net_ready(net_pixel_sync[1]),
        .filter_active(gaussian_pixel_sync[1]),
        .debug_mode(ui_lcd_data[209]),
        .touch_ready(ui_lcd_data[211]),
        .touch_error(ui_lcd_data[107:104]),
        .touch_count(ui_lcd_data[193:192]),
        .selection_valid(ui_lcd_data[210]),
        .selection_x(ui_lcd_data[137:128]),
        .selection_y(ui_lcd_data[121:112]),
        .zoom_mode(ui_lcd_data[28]),
        .zoom_step(ui_lcd_data[27:19]),
        .zoom_left(ui_lcd_data[9:0]), .zoom_top(ui_lcd_data[18:10]),
        .focus_x(ui_lcd_focus[18:9]), .focus_y(ui_lcd_focus[8:0]),
        .stop_latched(stop_pixel_sync[1]),
        .detection_request(detection_request), .detection_data(detection_mailbox),
        .detection_ack(detection_ack),
        .sensor_text_request(sensor_text_request), .sensor_text_data(sensor_text_mailbox),
        .sensor_text_ack(sensor_text_ack),
        .pid_page(ROBOT_ACTIVE ? robot_page==1 : pid_page), .pid_text_request(pid_text_request),
        .pid_text_data(pid_text_mailbox), .pid_text_ack(pid_text_ack),
        .actuator_test(ACTUATOR_TEST!=0), .test_status_request(test_status_request),
        .test_status_data(test_status_mailbox), .test_status_ack(test_status_ack),
        .battery_status_request(battery_status_request),.battery_status_data(battery_status_mailbox),
        .battery_status_ack(battery_status_ack),
        .robot_control(ROBOT_ACTIVE),.robot_status_request(robot_status_request),.robot_status_data(robot_status_mailbox),
        .robot_status_ack(robot_status_ack),.map_cell(map_read_cell),.map_read_valid(map_read_valid),
        .map_car_model(map_car_model),
        .map_read_en(map_read_en),.map_read_x(map_read_x),.map_read_y(map_read_y),
        .camera_pop(camera_pop), .frame_restart(frame_restart),
        .display_step(lcd_view_step), .display_left(lcd_view_left),
        .display_top(lcd_view_top), .ddr_read_base(lcd_ddr_read_base),
        .rgb(rgb_out), .de(TFT_de), .hs(TFT_hs), .vs(TFT_vs),
        .frame_heartbeat(frame_heartbeat), .camera_underflow(camera_underflow)
    );
    // No combinational gate follows the RGB output register; doing so broke
    // the 16 physical RGB setup paths in the earlier 1024x600 project.
    assign TFT_rgb = rgb_out;
    assign TFT_clk = pixel_clk;
    assign TFT_pwm = ready;

    ddr3_ctrl_2port u_framebuffer (
        .clk(clk50M), .pll_stop(pll_stop), .pll_lock(ddr_pll_locked),
        .clk_200m(ddr_clk400), .sys_rst_n(ready && ddr_mdrp_user_mode),
        .init_calib_complete(init_calib_complete),
        .rd_load(rd_restart_delay[15]), .wr_load(gaussian_frame_start),
        .app_addr_rd_min(lcd_ddr_read_base), .app_addr_rd_max(IMAGE_PIXELS),
        .rd_bust_len(DDR_BURST),
        .app_addr_wr_min(28'd0), .app_addr_wr_max(IMAGE_PIXELS),
        .wr_bust_len(DDR_BURST),
        .wr_clk(camera_pclk), .wfifo_wren(gaussian_valid),
        .wfifo_din(gaussian_pixel), .wrfifo_full(),
        .rd_clk(pixel_clk), .rfifo_rden(camera_pop),
        .rdfifo_empty(rdfifo_empty), .rfifo_dout(camera_pixel_read),
        .ddr3_dq(IO_ddr_dq), .ddr3_dqs_n(IO_ddr_dqs_n),
        .ddr3_dqs_p(IO_ddr_dqs), .ddr3_addr(O_ddr_addr),
        .ddr3_ba(O_ddr_ba), .ddr3_ras_n(O_ddr_ras_n),
        .ddr3_cas_n(O_ddr_cas_n), .ddr3_we_n(O_ddr_we_n),
        .ddr3_reset_n(O_ddr_reset_n), .ddr3_ck_p(O_ddr_clk),
        .ddr3_ck_n(O_ddr_clk_n), .ddr3_cke(O_ddr_cke),
        .ddr3_cs_n(O_ddr_cs_n), .ddr3_dm(O_ddr_dqm),
        .ddr3_odt(O_ddr_odt)
    );

    // The chapter-59 PLL/PHY/UDP path runs independently of the LCD clock.
    // FPGA = 192.168.10.2, PC = 192.168.10.3, subnet = /24.
    localparam [47:0] FPGA_MAC = 48'h02_ac_72_00_00_02;
    wire net_clk125, net_pll_locked, phy_init_done;
    Gowin_PLL u_net_pll (
        .clkin(clk50M), .clkout0(), .clkout1(net_clk125), .clkout2(),
        .lock(net_pll_locked), .mdclk(clk50M), .reset(~reset_sync)
    );
    phy_reg_config u_phy (
        .Clk(clk50M), .Rst_n(ready && net_pll_locked),
        .Phy_rst_n(eth_rst_n), .Rd_Data(),
        .Phy_init_done(phy_init_done), .mdc(eth_mdc), .mdio(eth_mdio)
    );
    assign net_ready = net_pll_locked && phy_init_done;

    wire [15:0] net_word;
    wire net_word_valid;
    wire net_frame_sent;
    video_400x240_packetizer u_video_packets (
        .clk(camera_pclk), .reset_n(camera_run_pipe[1]),
        .frame_start(gaussian_frame_start),
        .pixel_valid(SEND_CAMERA_UDP ? gaussian_valid : 1'b0), .pixel(gaussian_pixel),
        .x(gaussian_x), .y(gaussian_y),
        .word_out(net_word), .word_valid(net_word_valid),
        .frame_sent(net_frame_sent)
    );
    wire net_tx_start, net_tx_done, net_fifo_read;
    wire video_tx_done, udp_tx_start, udp_payload_read;
    wire [15:0] udp_tx_length;
    wire [7:0] udp_payload_data;
    wire [7:0] net_fifo_byte;
    generate if(SEND_CAMERA_UDP!=0)begin:camera_udp_fifo
    eth_tx_ctrl #(.PAYLOAD_DATA_BYTE(2), .PAYLOAD_LENGTH(401))
    u_net_tx_fifo (
        .reset_p(~net_pll_locked), .clk(camera_pclk),
        .data_i({net_word[7:0],net_word[15:8]}),
        .data_valid_i(net_word_valid),
        .eth_txfifo_rd_clk(net_clk125),
        .tx_en_pulse(net_tx_start), .tx_done(video_tx_done),
        .eth_txfifo_rden(net_fifo_read),
        .eth_txfifo_dout(net_fifo_byte)
    );
    end else begin:no_camera_udp_fifo
        assign net_tx_start=1'b0;assign net_fifo_byte=8'd0;
    end endgenerate
    wire [7:0] gmii_txd;
    wire gmii_tx_clk, gmii_txen;
    eth_udp_tx_gmii u_udp_tx (
        .clk125m(net_clk125), .reset_p(~net_pll_locked),
        .tx_en_pulse(udp_tx_start), .tx_done(net_tx_done),
        .dst_mac(48'hff_ff_ff_ff_ff_ff), .src_mac(FPGA_MAC),
        .dst_ip(32'hc0_a8_0a_03), .src_ip(32'hc0_a8_0a_02),
        .dst_port(16'd6102), .src_port(16'd5000),
        .data_length(udp_tx_length),
        .payload_req_o(udp_payload_read), .payload_dat_i(udp_payload_data),
        .gmii_tx_clk(gmii_tx_clk), .gmii_txen(gmii_txen),
        .gmii_txd(gmii_txd)
    );
    gmii_to_rgmii u_rgmii_tx (
        .reset_n(net_pll_locked),
        .gmii_tx_clk(gmii_tx_clk), .gmii_txd(gmii_txd),
        .gmii_txen(gmii_txen), .gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_gtxc),
        .rgmii_txd(rgmii_txd), .rgmii_txen(rgmii_txen)
    );

    // RX accepts 192.168.10.2 and the directed broadcast .255 on UDP 5001.
    // Broadcast commands need no FPGA-side ARP implementation.
    wire gmii_rx_clk, gmii_rxdv, gmii_rxer;
    wire [7:0] gmii_rxd;
    rgmii_to_gmii u_rgmii_rx (
        .reset(~net_pll_locked),
        .rgmii_rx_clk(rgmii_rx_clk), .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv),
        .gmii_rx_clk(gmii_rx_clk), .gmii_rxdv(gmii_rxdv),
        .gmii_rxd(gmii_rxd), .gmii_rxer(gmii_rxer)
    );
    wire rx_payload_valid, rx_packet_done, rx_packet_error;
    wire [7:0] rx_payload_byte;
    wire [15:0] rx_payload_length;
    eth_udp_rx_gmii u_udp_rx (
        .reset_p(~net_pll_locked),
        .local_mac(FPGA_MAC), .local_ip(32'hc0_a8_0a_02),
        .local_port(16'd5001),
        .clk125m_o(), .exter_mac(), .exter_ip(), .exter_port(),
        .rx_data_length(rx_payload_length), .data_overflow_i(1'b0),
        .payload_valid_o(rx_payload_valid), .payload_dat_o(rx_payload_byte),
        .one_pkt_done(rx_packet_done), .pkt_error(rx_packet_error),
        .debug_crc_check(),
        .gmii_rx_clk(gmii_rx_clk), .gmii_rxdv(gmii_rxdv),
        .gmii_rxd(gmii_rxd)
    );
    udp_detection_feedback #(.ENABLE_DISPLAY(!ROBOT_ACTIVE)) u_detection_feedback (
        .robot_ack(follow_mailbox_ack),.robot_request(follow_mailbox_request),.robot_data(follow_mailbox_data),
        .clk(gmii_rx_clk), .reset_n(net_pll_locked && pixel_reset_n),
        .payload_valid(rx_payload_valid), .payload_byte(rx_payload_byte),
        .payload_length(rx_payload_length),
        .packet_done(rx_packet_done), .packet_error(rx_packet_error),
        .display_ack(detection_ack), .display_request(detection_request),
        .display_data(detection_mailbox)
    );
    wire command_toggle, valid_command_seen;
    wire [7:0] command_code, command_value;
    udp_ui_command_parser u_command_parser (
        .clk(gmii_rx_clk), .reset_n(net_pll_locked),
        .payload_valid(rx_payload_valid), .payload_byte(rx_payload_byte),
        .payload_length(rx_payload_length),
        .packet_done(rx_packet_done), .packet_error(rx_packet_error),
        .command_toggle(command_toggle), .command_code(command_code),
        .command_value(command_value),
        .valid_command_seen(valid_command_seen)
    );
    reg [2:0] command_sync = 3'b000;
    always @(posedge clk50M) begin
        command_sync <= {command_sync[1:0],command_toggle};
        if (!reset_sync) begin
            filter_requested <= 1'b1;
            stop_latched <= 1'b0;
        end else if (touch_stop_pulse || pid_emergency_stop || motor_uart_emergency || vofa_emergency || test_emergency_stop ||
                     robot_emergency_stop || (button_press && (ROBOT_ACTIVE || ACTUATOR_TEST || motor_armed || motor_running)) ||
                     (command_sync[2] != command_sync[1] && command_code==8'd3)) begin
            stop_latched <= 1'b1; // takes priority over host clear on this clock
        end else if (motor_clear_stop || test_clear_stop || robot_clear_stop) begin
            stop_latched <= 1'b0;
        end else if (command_sync[2] != command_sync[1]) begin
            case (command_code)
                8'd1: filter_requested <= command_value[0];
                8'd2: ; // absolute debug request is handled by gestures
                8'd3: stop_latched <= 1'b1;
                8'd4: stop_latched <= 1'b0;
                default: ;
            endcase
        end else if (button_press) begin
            filter_requested <= ~filter_requested;
        end
    end


    // GT911 never gates camera, DDR or Ethernet initialization.
    wire touch_snapshot, touch_snapshot_cancel, touch_ack, touch_identified, touch_ready;
    wire [1:0] touch_count;
    wire [10:0] touch_x0, touch_y0, touch_x1, touch_y1;
    wire [3:0] touch_id0, touch_id1, touch_error;
    wire [31:0] touch_product;
    wire [15:0] touch_resolution_x, touch_resolution_y;
    gt911_touch u_touch (
        .clk(clk50M), .reset_n(ready),
        .tp_scl(TP_scl), .tp_sda(TP_sda), .tp_int(TP_int),
        .tp_reset_n(TP_rst_n), .snapshot_valid(touch_snapshot),
        .snapshot_cancel(touch_snapshot_cancel),
        .contact_count(touch_count), .x0(touch_x0), .y0(touch_y0),
        .x1(touch_x1), .y1(touch_y1), .id0(touch_id0), .id1(touch_id1),
        .ack_seen(touch_ack), .identified(touch_identified),
        .ready(touch_ready), .error_code(touch_error),
        .product_id(touch_product),
        .resolution_x(touch_resolution_x), .resolution_y(touch_resolution_y)
    );
    robot_subsystem u_robot(
        .follow_request(follow_mailbox_request),.follow_data(follow_mailbox_data),.follow_ack(follow_mailbox_ack),
        .follow_telemetry(follow_telemetry),.follow_laser_telemetry(follow_laser_telemetry),.follow_pan_pwm(follow_pan_pwm),.follow_tilt_pwm(follow_tilt_pwm),
        .battery_a_low(bq_valid_a && bq_error_a==0 && bq_voltage_a>8000 && bq_soc_a<=17),
        .shutdown_requested(pi_shutdown_requested),.software_stopped(fpga_software_stopped),
        .clk(clk50M),.reset_n(ready && ROBOT_ACTIVE),.pixel_clk(pixel_clk),.safety_stop(motor_safety_stop),
        .remote_takeover(vofa_takeover),.remote_active((motor_remote_owner && motor_armed && motor_running) || robot_turn_active),
        .remote_turn_request(vofa_turn_request),.remote_turn_degrees(vofa_turn_degrees),
        .task_valid(vofa_task_valid),.task_action(vofa_task_action),.task_value(vofa_task_value),.task_ready(vofa_task_ready),
        .task_rsp_valid(vofa_task_rsp_valid),.task_rsp_status(vofa_task_rsp_status),.task_rsp_value(vofa_task_rsp_value),
        .remote_turn_ready(robot_turn_ready),.remote_turn_active(robot_turn_active),
        .turn_complete_valid(robot_turn_complete_valid),.turn_complete_ready(robot_turn_complete_ready),
        .turn_report_busy(robot_turn_report_busy),
        .motor_fault_left(motor_fault_left),.motor_fault_right(motor_fault_right),
        .motor_armed(motor_armed),.motor_running(motor_running),
        .motor_mode_position(motor_mode_position),
        .motor_link_fault(motor_link_fault),
        .motor_target_left(motor_target_speed_left),.motor_target_right(motor_target_speed_right),
        .speed_kp_left(motor_skp_l),.speed_kp_right(motor_skp_r),
        .speed_ki_left(motor_ski_l),.speed_ki_right(motor_ski_r),
        .speed_kd_left(motor_skd_l),.speed_kd_right(motor_skd_r),
        .motor_position_target_left(motor_target_position_left),.motor_position_target_right(motor_target_position_right),
        .position_kp_left(motor_pkp_l),.position_kp_right(motor_pkp_r),
        .position_ki_left(motor_pki_l),.position_ki_right(motor_pki_r),
        .position_kd_left(motor_pkd_l),.position_kd_right(motor_pkd_r),
        .motor_selected_wheel(motor_selected_wheel),.motor_edit_position(motor_edit_position),
        .encoder_invert_left(motor_invert_left),.encoder_invert_right(motor_invert_right),
        .drive_invert_left(motor_drive_invert_left),.drive_invert_right(motor_drive_invert_right),
        .duty_left(motor_duty_left),.duty_right(motor_duty_right),
        .gains_ready((motor_skp_l!=0 || motor_ski_l!=0) && (motor_skp_r!=0 || motor_ski_r!=0)),
        .tick_10ms(motor_tick),.encoder_zeroed(motor_zero_position),
        .position_left(motor_position_left),.position_right(motor_position_right),
        .speed_left(motor_speed_left),.speed_right(motor_speed_right),.gyro_z(imu_gz),
        .gyro_valid(imu_demo_valid),.gyro_samples(imu_demo_samples),
        .touch_ready(touch_ready),.snapshot_valid(touch_snapshot),.snapshot_cancel(touch_snapshot_cancel),
        .contact_count(touch_count),.x0(touch_x0),.y0(touch_y0),.x1(touch_x1),.y1(touch_y1),.id0(touch_id0),
        .lidar_rx(lidar_rx),.lidar_tx(lidar_tx),.page(robot_page),.subpage(robot_subpage),.pid_action_valid(robot_pid_action_valid),
        .pid_action(robot_pid_action),.pid_argument(robot_pid_argument),
        .calibration_enable(robot_calibration_enable),
        .calibration_duty_left(robot_calibration_duty_left),.calibration_duty_right(robot_calibration_duty_right),
        .emergency_stop_pulse(robot_emergency_stop),.clear_stop_pulse(robot_clear_stop),
        .drive_enable(robot_drive_enable),.armed(robot_armed),.target_left(robot_target_left),.target_right(robot_target_right),
        .status_text(robot_text),.status_flags(robot_flags),.status_valid(robot_text_valid),
        .status_byte_valid(robot_byte_valid),.status_byte_data(robot_byte_data),.status_byte_index(robot_byte_index),
        .status_byte_ready(ROBOT_ACTIVE ? (robot_byte_ready && robot_lcd_byte_ready) : 1'b1),
        .lidar_front_valid(robot_lidar_valid),.lidar_front_mm(robot_lidar_front),
        .lidar_error(robot_lidar_error),.lidar_scans(robot_lidar_scans),
        .map_rd_en(map_read_en),.map_rd_x(map_read_x),.map_rd_y(map_read_y),.map_rd_cell(map_read_cell),.map_rd_valid(map_read_valid),
        .map_car_model(map_car_model)
    );
    wire ui_command_event = command_sync[2] != command_sync[1];
    actuator_test_control u_actuator_test (
        .bq_info(bq_info),
        .bq_setup_busy(bq_setup_busy_a || bq_setup_busy_b),.bq_setup_pulse(bq_setup_pulse),
        .clk(clk50M),.reset_n(ready && ACTUATOR_TEST!=0),
        .safety_stop(motor_safety_stop || !button4_n),
        .touch_ready(touch_ready),.snapshot_valid(touch_snapshot),
        .snapshot_cancel(touch_snapshot_cancel),.contact_count(touch_count),
        .x0(touch_x0),.y0(touch_y0),.x1(touch_x1),.y1(touch_y1),.id0(touch_id0),
        .rx_byte_valid(demo_rx_byte_valid),.tx_line_count(bluetooth_tx_lines),
        .speed_text(motor_text),.speed_text_valid(motor_text_valid),
        .emergency_stop_pulse(test_emergency_stop),
        .clear_stop_pulse(test_clear_stop),
        .pwma(test_pwma),.ain1(test_ain1),.ain2(test_ain2),
        .pwmb(test_pwmb),.bin1(test_bin1),.bin2(test_bin2),
        .servo0(test_servo0),.servo1(test_servo1),.status_text(test_status_text),
        .status_flags(test_status_flags),.status_valid(test_status_valid)
    );
    wire host_debug_toggle = ui_command_event && command_code==8'd2 &&
                             command_value[0]!=debug_requested;
    wire host_clear = ui_command_event && command_code==8'd5;
    wire host_zoom_reset = ui_command_event && command_code==8'd6;
    wire selection_valid, selection_toggle, capture_toggle, clear_toggle;
    wire [9:0] selection_x, selection_y;
    wire zoom_mode;
    wire [8:0] zoom_step, zoom_top, focus_y;
    wire [9:0] zoom_left, focus_x;
    wire zoom_toggle;
    wire [3:0] ui_click_event;
    pid_touch_controls u_pid_touch (
        .clk(clk50M), .reset_n(ready && !ACTUATOR_TEST && !ROBOT_ACTIVE), .touch_ready(touch_ready),
        .snapshot_valid(touch_snapshot), .snapshot_cancel(touch_snapshot_cancel),
        .contact_count(touch_count), .x0(touch_x0), .y0(touch_y0), .x1(touch_x1), .y1(touch_y1),
        .id0(touch_id0), .id1(touch_id1), .pid_page(pid_page),
        .action_valid(pid_action_valid), .action(pid_action), .emergency_stop_pulse(pid_emergency_stop)
    );
    touch_gestures u_gestures (
        .clk(clk50M), .reset_n(ready), .touch_ready(touch_ready),
        .snapshot_valid(touch_snapshot), .snapshot_cancel(touch_snapshot_cancel || pid_page || ACTUATOR_TEST || ROBOT_ACTIVE),
        .contact_count((pid_page || ACTUATOR_TEST || ROBOT_ACTIVE) ? 2'd0 : touch_count),
        .x0(touch_x0), .y0(touch_y0), .x1(touch_x1), .y1(touch_y1),
        .id0(touch_id0), .id1(touch_id1),
        .view_step(lcd_view_data[27:19]),
        .view_left(lcd_view_data[9:0]), .view_top(lcd_view_data[18:10]),
        .debug_toggle_pulse(host_debug_toggle), .clear_pulse(host_clear),
        .zoom_reset_pulse(host_zoom_reset),
        .debug_mode(debug_requested), .selection_valid(selection_valid),
        .selection_x(selection_x), .selection_y(selection_y),
        .selection_toggle(selection_toggle), .zoom_mode(zoom_mode),
        .zoom_step(zoom_step), .zoom_left(zoom_left), .zoom_top(zoom_top),
        .focus_x(focus_x), .focus_y(focus_y),
        .zoom_toggle(zoom_toggle), .capture_toggle(capture_toggle),
        .clear_toggle(clear_toggle), .click_event(ui_click_event),
        .stop_pulse(touch_stop_pulse)
    );

    // Event counters survive dropped telemetry packets. They are requests,
    // not acknowledgements of host photo saving or tracking completion.
    reg selection_seen=0, capture_seen=0, clear_seen=0;
    reg [15:0] selection_count=0, capture_count=0, clear_count=0;
    reg [1:0] actual_filter_sync=2'b11;
    reg [22:0] ui_tick_divider=0;
    reg [31:0] ui_ticks=0, ui_sequence=0;
    reg [255:0] ui_mailbox_data=0;
    reg ui_mailbox_request=0;
    // LCD is local: it must keep responding even if the network clock fails.
    reg [255:0] ui_lcd_mailbox_data=0;
    reg [18:0] ui_lcd_focus_mailbox={10'd400,9'd240};
    reg ui_lcd_mailbox_request=0;
    reg [19:0] ui_lcd_tick_divider=0;
    reg [2:0] ui_lcd_ack_sync=0;
    wire ui_lcd_ack;
    wire ui_mailbox_ack;
    reg [2:0] ui_ack_sync=0;
    wire [7:0] ui_flags = {2'b00,stop_latched,touch_identified,
                         touch_ready,selection_valid,debug_requested,
                         actual_filter_sync[1]};
    wire [255:0] ui_snapshot_now = {
        32'h56554931,8'd2,ui_flags,{7'd0,zoom_step!=9'd256},
        {6'd0,touch_count},selection_count,capture_count,
        clear_count,{6'd0,selection_x},{6'd0,selection_y},
        {4'd0,touch_error},8'd0,ui_sequence,ui_ticks,
        3'd0,zoom_mode,zoom_step,zoom_top,zoom_left
    };
    // LCD gets requests; host observers get the last coherently observed
    // frame-latched geometry. Neither observation is a video-frame ACK.
    wire [255:0] ui_observed_now = {
        ui_snapshot_now[255:208],7'd0,lcd_view_data[27:19]!=9'd256,
        ui_snapshot_now[199:32],lcd_view_data
    };
    always @(posedge clk50M) begin
        ui_ack_sync<={ui_ack_sync[1:0],ui_mailbox_ack};
        ui_lcd_ack_sync<={ui_lcd_ack_sync[1:0],ui_lcd_ack};
        actual_filter_sync<={actual_filter_sync[0],gaussian_active};
        if (!reset_sync) begin
            selection_seen<=0; capture_seen<=0; clear_seen<=0;
            selection_count<=0; capture_count<=0; clear_count<=0;
            actual_filter_sync<=2'b11; ui_tick_divider<=0;
            ui_ticks<=0; ui_sequence<=0; ui_mailbox_data<=0;
            ui_mailbox_request<=0; ui_ack_sync<=0;
            ui_lcd_mailbox_data<=0; ui_lcd_mailbox_request<=0;
            ui_lcd_focus_mailbox<={10'd400,9'd240}; ui_lcd_tick_divider<=0; ui_lcd_ack_sync<=0;
        end else begin
            selection_seen<=selection_toggle;
            capture_seen<=capture_toggle;
            clear_seen<=clear_toggle;
            if (selection_seen!=selection_toggle) selection_count<=selection_count+1'b1;
            if (capture_seen!=capture_toggle) capture_count<=capture_count+1'b1;
            if (clear_seen!=clear_toggle) clear_count<=clear_count+1'b1;
            // Local UI updates about 60 Hz, independently of 10 Hz UDP status.
            // Hold the whole bundle unchanged until the destination ACKs it.
            if (ui_lcd_tick_divider==20'd833332) begin
                ui_lcd_tick_divider<=0;
                if (ui_lcd_mailbox_request==ui_lcd_ack_sync[2]) begin
                    ui_lcd_mailbox_data<=ui_snapshot_now;
                    ui_lcd_focus_mailbox<={focus_x,focus_y};
                    ui_lcd_mailbox_request<=~ui_lcd_mailbox_request;
                end
            end else ui_lcd_tick_divider<=ui_lcd_tick_divider+1'b1;
            if (ui_tick_divider==(ROBOT_ACTIVE ? 23'd2499999 : 23'd4999999)) begin
                ui_tick_divider<=0;
                ui_ticks<=ui_ticks+1'b1;
                if (ui_mailbox_request==ui_ack_sync[2]) begin
                    ui_mailbox_data<=ROBOT_ACTIVE ? (ui_sequence[0] ? follow_laser_telemetry : follow_telemetry) : ui_observed_now;
                    ui_sequence<=ui_sequence+1'b1;
                    ui_mailbox_request<=~ui_mailbox_request;
                end
            end else ui_tick_divider<=ui_tick_divider+1'b1;
        end
    end
    ui_video_tx_mux u_ui_tx_mux (
        .clk(net_clk125), .reset_n(net_pll_locked),
        .video_start(net_tx_start), .video_data(net_fifo_byte),
        .video_read(net_fifo_read), .video_done(video_tx_done),
        .mailbox_request(ui_mailbox_request), .mailbox_data(ui_mailbox_data),
        .mailbox_ack(ui_mailbox_ack), .tx_start(udp_tx_start),
        .tx_length(udp_tx_length), .tx_data(udp_payload_data),
        .tx_read(udp_payload_read), .tx_done(net_tx_done)
    );

    // Handshaked bundle: request crosses three flops, then data is sampled.
    reg [2:0] ui_lcd_request_sync=0;
    reg ui_lcd_request_seen=0;
    reg [255:0] ui_lcd_data=0;
    reg [18:0] ui_lcd_focus={10'd400,9'd240};
    assign ui_lcd_ack=ui_lcd_request_seen;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) begin
            ui_lcd_request_sync<=0; ui_lcd_request_seen<=0;
            ui_lcd_data<={224'd0,32'h08000000}; ui_lcd_focus<={10'd400,9'd240};
        end else begin
            ui_lcd_request_sync<={ui_lcd_request_sync[1:0],ui_lcd_mailbox_request};
            if (ui_lcd_request_sync[2]!=ui_lcd_request_seen) begin
                ui_lcd_request_seen<=ui_lcd_request_sync[2];
                ui_lcd_data<=ui_lcd_mailbox_data;
                ui_lcd_focus<=ui_lcd_focus_mailbox;
            end
        end
    end

    reg [24:0] alive_count = 0;
    reg alive = 0;
    always @(posedge clk50M) begin
        if (alive_count == 25'd24999999) begin
            alive_count <= 0;
            alive <= ~alive;
        end else alive_count <= alive_count+1'b1;
    end
    reg frame_meta = 0, frame_sync = 0;
    reg ddr_ready_meta = 0, ddr_ready_sync = 0;
    reg cam_seen_meta = 0, cam_seen_sync = 0;
    reg underflow_meta = 0, underflow_sync = 0;
    always @(posedge clk50M) begin
        frame_meta <= frame_heartbeat;
        frame_sync <= frame_meta;
        ddr_ready_meta <= init_calib_complete;
        ddr_ready_sync <= ddr_ready_meta;
        cam_seen_meta <= camera_frame_seen;
        cam_seen_sync <= cam_seen_meta;
        underflow_meta <= camera_underflow;
        underflow_sync <= underflow_meta;
    end
    assign led[0] = alive;            // 0.5 s toggle: 50 MHz clock alive
    assign led[1] = ready;            // board startup completed
    assign led[2] = frame_sync;       // LCD scan heartbeat
    assign led[3] = camera_init_done;// OV5640 SCCB register writes completed
    assign led[4] = ddr_ready_sync;   // DDR3 calibrated
    assign led[5] = net_ready;        // Ethernet PHY configured (not link detect)
    assign led[6] = cam_seen_sync;    // OV5640 frame sync was observed
    assign led[7] = underflow_sync;   // LCD video FIFO ran dry this frame
endmodule
