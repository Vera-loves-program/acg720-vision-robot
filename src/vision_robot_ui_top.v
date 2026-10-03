// ACG720 GW5AT-60B: live OV5640 -> real Gaussian -> DDR3/LCD + UDP.
// Chapter 45 supplies camera/DDR, chapter 59 supplies PHY/UDP transport.
module vision_robot_ui_top (
    input  wire        clk50M,
    input  wire        reset_n,       // S0 / F15, active low
    input  wire        button4_n,     // S4 / B21 toggles Gaussian filter
    output wire [15:0] TFT_rgb,
    output wire        TFT_hs,
    output wire        TFT_vs,
    output wire        TFT_clk,
    output wire        TFT_de,
    output wire        TFT_pwm,
    output wire [7:0]  led,
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

    // S4 is debounced. A press toggles the filter, initially ON.
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
    lcd1024_interaction_ui u_lcd (
        .clk(pixel_clk), .run(pixel_run_pipe[1]),
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
        .selection_valid(ui_lcd_data[210]),
        .selection_x(ui_lcd_data[137:128]),
        .selection_y(ui_lcd_data[121:112]),
        .zoom_level(ui_lcd_data[201:200]),
        .stop_latched(stop_pixel_sync[1]),
        .camera_pop(camera_pop), .frame_restart(frame_restart),
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
        .rd_load(frame_restart), .wr_load(gaussian_frame_start),
        .app_addr_rd_min(28'd0), .app_addr_rd_max(IMAGE_PIXELS),
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
        .pixel_valid(gaussian_valid), .pixel(gaussian_pixel),
        .x(gaussian_x), .y(gaussian_y),
        .word_out(net_word), .word_valid(net_word_valid),
        .frame_sent(net_frame_sent)
    );
    wire net_tx_start, net_tx_done, net_fifo_read;
    wire video_tx_done, udp_tx_start, udp_payload_read;
    wire [15:0] udp_tx_length;
    wire [7:0] udp_payload_data;
    wire [7:0] net_fifo_byte;
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
    wire ui_command_event = command_sync[2] != command_sync[1];
    wire host_debug_toggle = ui_command_event && command_code==8'd2 &&
                             command_value[0]!=debug_requested;
    wire host_clear = ui_command_event && command_code==8'd5;
    wire host_zoom_reset = ui_command_event && command_code==8'd6;
    wire selection_valid, selection_toggle, capture_toggle, clear_toggle;
    wire [9:0] selection_x, selection_y;
    wire [1:0] zoom_level;
    wire zoom_toggle;
    wire [3:0] ui_click_event;
    touch_gestures u_gestures (
        .clk(clk50M), .reset_n(ready), .touch_ready(touch_ready),
        .snapshot_valid(touch_snapshot), .snapshot_cancel(touch_snapshot_cancel),
        .contact_count(touch_count),
        .x0(touch_x0), .y0(touch_y0), .x1(touch_x1), .y1(touch_y1),
        .id0(touch_id0), .id1(touch_id1),
        .debug_toggle_pulse(host_debug_toggle), .clear_pulse(host_clear),
        .zoom_reset_pulse(host_zoom_reset),
        .debug_mode(debug_requested), .selection_valid(selection_valid),
        .selection_x(selection_x), .selection_y(selection_y),
        .selection_toggle(selection_toggle), .zoom_level(zoom_level),
        .zoom_toggle(zoom_toggle), .capture_toggle(capture_toggle),
        .clear_toggle(clear_toggle), .click_event(ui_click_event)
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
    reg ui_lcd_mailbox_request=0;
    wire ui_mailbox_ack;
    reg [2:0] ui_ack_sync=0;
    wire [7:0] ui_flags = {2'b00,stop_latched,touch_identified,
                         touch_ready,selection_valid,debug_requested,
                         actual_filter_sync[1]};
    wire [255:0] ui_snapshot_now = {
        32'h56554931,8'd1,ui_flags,{6'd0,zoom_level},
        {6'd0,touch_count},selection_count,capture_count,
        clear_count,{6'd0,selection_x},{6'd0,selection_y},
        {4'd0,touch_error},8'd0,ui_sequence,ui_ticks,32'd0
    };
    always @(posedge clk50M) begin
        ui_ack_sync<={ui_ack_sync[1:0],ui_mailbox_ack};
        actual_filter_sync<={actual_filter_sync[0],gaussian_active};
        if (!reset_sync) begin
            selection_seen<=0; capture_seen<=0; clear_seen<=0;
            selection_count<=0; capture_count<=0; clear_count<=0;
            actual_filter_sync<=2'b11; ui_tick_divider<=0;
            ui_ticks<=0; ui_sequence<=0; ui_mailbox_data<=0;
            ui_mailbox_request<=0; ui_ack_sync<=0;
            ui_lcd_mailbox_data<=0; ui_lcd_mailbox_request<=0;
        end else begin
            selection_seen<=selection_toggle;
            capture_seen<=capture_toggle;
            clear_seen<=clear_toggle;
            if (selection_seen!=selection_toggle) selection_count<=selection_count+1'b1;
            if (capture_seen!=capture_toggle) capture_count<=capture_count+1'b1;
            if (clear_seen!=clear_toggle) clear_count<=clear_count+1'b1;
            if (ui_tick_divider==23'd4999999) begin
                ui_tick_divider<=0;
                ui_ticks<=ui_ticks+1'b1;
                ui_lcd_mailbox_data<=ui_snapshot_now;
                ui_lcd_mailbox_request<=~ui_lcd_mailbox_request;
                if (ui_mailbox_request==ui_ack_sync[2]) begin
                    ui_mailbox_data<=ui_snapshot_now;
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

    // The mailbox changes at most once per 100 ms; data is stable before
    // the three-flop request reaches the LCD domain. Snapshot whole bundle.
    reg [2:0] ui_lcd_request_sync=0;
    reg ui_lcd_request_seen=0;
    reg [255:0] ui_lcd_data=0;
    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) begin
            ui_lcd_request_sync<=0; ui_lcd_request_seen<=0; ui_lcd_data<=0;
        end else begin
            ui_lcd_request_sync<={ui_lcd_request_sync[1:0],ui_lcd_mailbox_request};
            if (ui_lcd_request_sync[2]!=ui_lcd_request_seen) begin
                ui_lcd_request_seen<=ui_lcd_request_sync[2];
                ui_lcd_data<=ui_lcd_mailbox_data;
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
