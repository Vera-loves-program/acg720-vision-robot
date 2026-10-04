// LCD-only, nearest-neighbour crop zoom of the original 800x480 RGB565 frame.
// Q8 source step 256 = 1x, 128 = 2x, 64 = 4x. The 1x FIFO path is unchanged.
//
// The DDR reader must start each LCD frame at ddr_read_base, then stream full
// 800-pixel source rows in their original order. The integrating top delays
// rd_load after frame_restart so the new base is stable before the FIFO flush.
// Camera capture, DDR writes, PHY/PLL, burst length and UDP video are untouched.
//
// Two full source-row buffers allow arbitrary horizontal origin and source
// step without a whole-frame BSRAM cache. Loader stays at most one source row
// ahead of the displayed row, preserving a row while it is repeated on LCD.
// Geometry is latched only at frame_restart. Source coordinates are aligned
// with the synchronous RAM result for ROI overlays in original camera space.
//
// video_fetch is one pixel clock before video_area, with x=0..799/y=0..479.
// Never add an extra delay to the 1x FWFT pixel or to the fetch coordinates.
module lcd_camera_zoom #(
    // Covers delayed rd_load, completion of the current 200-beat read burst,
    // the existing 1024 DDR UI quiet clocks and 32-cycle FIFO reset interval.
    parameter integer RESTART_GUARD_CYCLES = 2048
) (
    input  wire        clk,
    input  wire        run,
    input  wire        frame_restart,
    input  wire [8:0]  step_request,
    input  wire [9:0]  left_request,
    input  wire [8:0]  top_request,
    input  wire        video_area,
    input  wire        video_fetch,
    input  wire [9:0]  video_fetch_x,
    input  wire [8:0]  video_fetch_y,
    input  wire [15:0] camera_pixel,
    input  wire        camera_pixel_valid,
    output wire        camera_pop,
    output wire [15:0] display_pixel,
    output wire        display_valid,
    output wire [9:0]  display_source_x,
    output wire [8:0]  display_source_y,
    output reg         zoom_active = 1'b0,
    output wire        zoom_ready,
    output reg         frame_underflow = 1'b0,
    output reg  [27:0] ddr_read_base = 28'd0,
    output reg  [8:0]  active_step = 9'd256,
    output reg  [9:0]  active_left = 10'd0,
    output reg  [8:0]  active_top = 9'd0
);
    // Constant products use shifts/adds and are consumed only at frame start.
    // Clamp the last sampled source pixel to the 800x480 frame, even if the
    // control-side bundle is malformed. 1x always restores the full frame.
    wire [8:0] bounded_step = step_request < 9'd64 ? 9'd64 :
                              step_request > 9'd256 ? 9'd256 : step_request;
    wire [17:0] step_wide = {9'd0, bounded_step};
    wire [17:0] last_x_product = (step_wide << 9) + (step_wide << 8) +
                                 (step_wide << 5) - step_wide; // 799 * step
    wire [17:0] last_y_product = (step_wide << 9) - (step_wide << 5) -
                                 step_wide;                 // 479 * step
    wire [18:0] footprint_x_product = ({1'b0,step_wide} << 9) +
        ({1'b0,step_wide} << 8) + ({1'b0,step_wide} << 5) + 19'd255;
    wire [17:0] footprint_y_product = (step_wide << 9) - (step_wide << 5) + 18'd255;
    wire [9:0] max_left = 10'd800 - footprint_x_product[17:8];
    wire [8:0] max_top = 9'd480 - footprint_y_product[16:8];
    wire [9:0] bounded_left = bounded_step == 9'd256 ? 10'd0 :
                             left_request > max_left ? max_left : left_request;
    wire [8:0] bounded_top = bounded_step == 9'd256 ? 9'd0 :
                            top_request > max_top ? max_top : top_request;
    wire [27:0] top_wide = {19'd0, bounded_top};
    wire [27:0] requested_read_base = (top_wide << 9) + (top_wide << 8) +
                                    (top_wide << 5);          // top * 800

    // Do not reset the RAM arrays or their data outputs: infer BSRAM.
    // Every valid source row writes entries 0..799; the unused tail is not read.
    (* ram_style = "block" *) reg [15:0] line0 [0:1023];
    (* ram_style = "block" *) reg [15:0] line1 [0:1023];
    reg [15:0] line0_read, line1_read;
    reg read_bank = 1'b0;
    reg read_valid = 1'b0;
    reg [9:0] read_source_x = 10'd0;
    reg [8:0] read_source_y = 9'd0;

    reg [10:0] restart_guard = 0;
    reg [9:0] source_x = 0;
    reg [8:0] source_y = 0;
    reg [8:0] source_last_y = 9'd479;
    reg source_done = 1'b0;
    reg [8:0] presented_row = 0;
    reg line0_valid = 1'b0, line1_valid = 1'b0;
    reg [8:0] line0_tag = 0, line1_tag = 0;
    reg first_line_ready = 1'b0;

    // DDA accumulators avoid a variable multiplier/divider in the 45 MHz
    // pixel-to-RAM-address path. x advances every fetch; y only at row start.
    reg [17:0] x_phase = 0;
    reg [16:0] y_phase = 0;
    wire [17:0] fetch_x_phase = video_fetch_x == 10'd0 ? 18'd0 : x_phase;
    wire [16:0] fetch_y_phase = video_fetch_y == 9'd0 ? 17'd0 :
        video_fetch_x == 10'd0 ? y_phase + {8'd0, active_step} : y_phase;
    wire [9:0] fetch_source_x = active_left + fetch_x_phase[17:8];
    wire [8:0] fetch_source_y = active_top + fetch_y_phase[16:8];
    wire load_permitted = {1'b0, source_y} <= {1'b0, presented_row} + 10'd1;
    wire zoom_pop = run && zoom_active && !frame_restart && restart_guard == 0 &&
                    !source_done && !frame_underflow && load_permitted &&
                    camera_pixel_valid;
    wire ram_read = run && zoom_active && video_fetch &&
                    !frame_restart && !frame_underflow;
    wire fetch_line_valid = fetch_source_y[0]
        ? line1_valid && line1_tag == fetch_source_y
        : line0_valid && line0_tag == fetch_source_y;

    assign camera_pop = zoom_active ? zoom_pop :
        run && video_area && camera_pixel_valid && !frame_underflow;
    assign display_pixel = zoom_active
        ? (read_bank ? line1_read : line0_read) : camera_pixel;
    assign display_valid = run && !frame_underflow &&
        (zoom_active ? read_valid : camera_pixel_valid);
    assign display_source_x = zoom_active ? read_source_x : video_fetch_x - 10'd1;
    assign display_source_y = zoom_active ? read_source_y : video_fetch_y;
    assign zoom_ready = run && !frame_underflow &&
        (!zoom_active || first_line_ready);

    always @(posedge clk) begin
        if (zoom_pop && !source_y[0]) line0[source_x] <= camera_pixel;
        if (ram_read && !fetch_source_y[0]) line0_read <= line0[fetch_source_x];
    end
    always @(posedge clk) begin
        if (zoom_pop && source_y[0]) line1[source_x] <= camera_pixel;
        if (ram_read && fetch_source_y[0]) line1_read <= line1[fetch_source_x];
    end

    always @(posedge clk or negedge run) begin
        if (!run) begin
            zoom_active <= 1'b0;
            frame_underflow <= 1'b0;
            ddr_read_base <= 28'd0;
            active_step <= 9'd256; active_left <= 0; active_top <= 0;
            restart_guard <= 0;
            source_x <= 0; source_y <= 0; source_last_y <= 9'd479;
            source_done <= 1'b0; presented_row <= 0;
            line0_valid <= 1'b0; line1_valid <= 1'b0;
            line0_tag <= 0; line1_tag <= 0;
            read_bank <= 1'b0; read_valid <= 1'b0;
            read_source_x <= 0; read_source_y <= 0;
            first_line_ready <= 1'b0;
            x_phase <= 0; y_phase <= 0;
        end else if (frame_restart) begin
            zoom_active <= bounded_step != 9'd256;
            ddr_read_base <= requested_read_base;
            active_step <= bounded_step;
            active_left <= bounded_left; active_top <= bounded_top;
            frame_underflow <= 1'b0;
            restart_guard <= RESTART_GUARD_CYCLES - 1;
            source_x <= 0; source_y <= bounded_top;
            source_last_y <= bounded_top + last_y_product[16:8];
            source_done <= 1'b0; presented_row <= bounded_top;
            line0_valid <= 1'b0; line1_valid <= 1'b0;
            line0_tag <= 0; line1_tag <= 0;
            read_bank <= 1'b0; read_valid <= 1'b0;
            read_source_x <= 0; read_source_y <= 0;
            first_line_ready <= 1'b0;
            x_phase <= 0; y_phase <= 0;
        end else begin
            if (restart_guard != 0) restart_guard <= restart_guard - 1'b1;
            if (video_area && !display_valid) frame_underflow <= 1'b1;

            read_valid <= ram_read && fetch_line_valid;
            if (ram_read) begin
                x_phase <= fetch_x_phase + {9'd0, active_step};
                if (video_fetch_x == 10'd0) begin
                    y_phase <= fetch_y_phase;
                    presented_row <= fetch_source_y;
                end
                read_bank <= fetch_source_y[0];
                read_source_x <= fetch_source_x;
                read_source_y <= fetch_source_y;
            end

            // Count only successful FWFT FIFO pops. All 800 source columns
            // are loaded, including margins outside the selected crop.
            if (zoom_pop) begin
                if (source_x == 10'd0) begin
                    if (source_y[0]) line1_valid <= 1'b0;
                    else line0_valid <= 1'b0;
                end
                if (source_x == 10'd799) begin
                    source_x <= 0;
                    if (source_y[0]) begin
                        line1_tag <= source_y;
                        line1_valid <= 1'b1;
                    end else begin
                        line0_tag <= source_y;
                        line0_valid <= 1'b1;
                    end
                    if (source_y == active_top) first_line_ready <= 1'b1;
                    if (source_y == source_last_y) source_done <= 1'b1;
                    else source_y <= source_y + 1'b1;
                end else source_x <= source_x + 1'b1;
            end
        end
    end
endmodule
