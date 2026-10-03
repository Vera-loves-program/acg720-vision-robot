// LCD-only 1x / 2x zoom for the chapter-45 sequential DDR read FIFO.
//
// 1x preserves the original camera_pop/display timing. At 2x the central
// source rectangle [200,600) x [120,360) is enlarged to 800x480 by nearest
// neighbour replication. Two 400-pixel line buffers are sufficient; the
// camera, Gaussian pipeline, DDR writer and 400x240 UDP stream are unchanged.
//
// The renderer supplies video_fetch one pixel clock BEFORE video_area:
//   video area : ax = 16..815, ay = 56..535
//   video fetch: ax = 15..814, ay = 56..535
//   fetch_x    : ax - 15, fetch_y : ay - 56
// The synchronous RAM result therefore belongs to the renderer's current
// video_area pixel. Do not delay 1x RGB or add another delay to fetch_x/y.
//
// A missing required line blanks the rest of that LCD frame. It is never
// replaced with stale buffer contents or a shifted camera image. Recovery
// and requested zoom changes happen only at frame_restart.
module lcd_camera_zoom #(
    // The existing adapter finishes its current 200-beat burst, waits 1024
    // DDR UI cycles, then flushes for 32 cycles on rd_load. At 100 MHz DDR
    // UI / 45 MHz LCD, 2048 LCD clocks leave a conservative flush margin.
    // With the existing y=56 video origin, approximately 106336 LCD clocks
    // precede the first video pixel: 2048 + 96000 skipped source pixels +
    // 600 first-row pixels leave about 7688 clocks for FIFO stalls.
    parameter integer RESTART_GUARD_CYCLES = 2048
) (
    input  wire        clk,
    input  wire        run,
    input  wire        frame_restart,
    input  wire [1:0]  zoom_request,     // 0=1x; any nonzero request=2x
    input  wire        video_area,
    input  wire        video_fetch,
    input  wire [9:0]  video_fetch_x,    // 0..799, one clock ahead of display
    input  wire [8:0]  video_fetch_y,    // 0..479
    input  wire [15:0] camera_pixel,     // existing FIFO first-word output
    input  wire        camera_pixel_valid,
    output wire        camera_pop,
    output wire [15:0] display_pixel,
    output wire        display_valid,
    output reg         zoom_active = 1'b0,
    output wire        zoom_ready,
    output reg         frame_underflow = 1'b0
);
    // No reset/initialisation of either memory array: infer synchronous
    // simple dual-port BSRAM rather than thousands of resettable registers.
    // Power-of-two allocation makes the RAM addresses explicit; only the
    // first 400 entries in each bank are used.
    (* ram_style = "block" *) reg [15:0] line0 [0:511];
    (* ram_style = "block" *) reg [15:0] line1 [0:511];
    reg [15:0] line0_read, line1_read;
    reg read_bank = 1'b0;
    reg read_valid = 1'b0;

    reg [10:0] restart_guard = 0;
    reg [9:0] source_x = 0;
    reg [8:0] source_y = 0;
    reg source_done = 1'b0;
    reg [7:0] presented_row = 0;
    reg line0_valid = 1'b0, line1_valid = 1'b0;
    reg [7:0] line0_tag = 0, line1_tag = 0;
    reg first_line_ready = 1'b0;

    wire [7:0] source_crop_row = source_y - 9'd120;
    wire [7:0] fetch_crop_row = video_fetch_y[8:1];
    wire [8:0] source_write_address = source_x - 10'd200;
    wire [8:0] read_address = video_fetch_x[9:1];
    wire source_crop = source_y >= 9'd120 && source_y < 9'd360;
    wire source_crop_pixel = source_crop &&
                             source_x >= 10'd200 && source_x < 10'd600;
    // Prefill rows 0 and 1, then keep at most one crop row ahead. This leaves
    // the currently displayed bank unchanged throughout both repeated rows.
    wire load_permitted = source_y < 9'd120 ||
                          {1'b0, source_crop_row} <=
                          ({1'b0, presented_row} + 9'd1);
    wire zoom_pop = run && zoom_active && !frame_restart &&
                    restart_guard == 0 && !source_done &&
                    !frame_underflow && load_permitted && camera_pixel_valid;
    wire source_write = zoom_pop && source_crop_pixel;
    wire ram_read = run && zoom_active && video_fetch &&
                    !frame_restart && !frame_underflow;
    wire fetch_line_valid = video_fetch_y[1]
        ? (line1_valid && line1_tag == fetch_crop_row)
        : (line0_valid && line0_tag == fetch_crop_row);

    // The ordinary view consumes exactly the same FIFO pixels at exactly
    // the same LCD coordinates as before this module was added.
    assign camera_pop = zoom_active ? zoom_pop :
        (run && video_area && camera_pixel_valid && !frame_underflow);
    assign display_pixel = zoom_active
        ? (read_bank ? line1_read : line0_read) : camera_pixel;
    assign display_valid = run && !frame_underflow &&
        (zoom_active ? read_valid : camera_pixel_valid);
    assign zoom_ready = run && !frame_underflow &&
        (!zoom_active || first_line_ready);

    // Registered BSRAM reads are intentionally not reset. read_valid guards
    // all data until the correct tagged line has been written in this frame.
    always @(posedge clk) begin
        if (source_write && !source_y[0])
            line0[source_write_address] <= camera_pixel;
        if (ram_read && !video_fetch_y[1])
            line0_read <= line0[read_address];
    end
    always @(posedge clk) begin
        if (source_write && source_y[0])
            line1[source_write_address] <= camera_pixel;
        if (ram_read && video_fetch_y[1])
            line1_read <= line1[read_address];
    end

    always @(posedge clk or negedge run) begin
        if (!run) begin
            zoom_active <= 1'b0;
            frame_underflow <= 1'b0;
            restart_guard <= 0;
            source_x <= 0; source_y <= 0; source_done <= 1'b0;
            presented_row <= 0;
            line0_valid <= 1'b0; line1_valid <= 1'b0;
            line0_tag <= 0; line1_tag <= 0;
            read_bank <= 1'b0; read_valid <= 1'b0;
            first_line_ready <= 1'b0;
        end else if (frame_restart) begin
            zoom_active <= zoom_request != 2'd0;
            frame_underflow <= 1'b0;
            restart_guard <= RESTART_GUARD_CYCLES - 1;
            source_x <= 0; source_y <= 0; source_done <= 1'b0;
            presented_row <= 0;
            line0_valid <= 1'b0; line1_valid <= 1'b0;
            line0_tag <= 0; line1_tag <= 0;
            read_bank <= 1'b0; read_valid <= 1'b0;
            first_line_ready <= 1'b0;
        end else begin
            if (restart_guard != 0)
                restart_guard <= restart_guard - 1'b1;

            if (video_area && !display_valid)
                frame_underflow <= 1'b1;

            read_valid <= ram_read && fetch_line_valid;
            if (ram_read) begin
                read_bank <= video_fetch_y[1];
                if (video_fetch_x == 10'd0)
                    presented_row <= fetch_crop_row;
            end

            // Counts advance only on successful FIFO pops, including the
            // discarded top rows and each source line's left/right margins.
            if (zoom_pop) begin
                if (source_x == 10'd799) begin
                    source_x <= 0;
                    if (source_y == 9'd359)
                        source_done <= 1'b1;
                    else source_y <= source_y + 1'b1;
                end else source_x <= source_x + 1'b1;

                if (source_crop && source_x == 10'd200) begin
                    if (source_y[0]) line1_valid <= 1'b0;
                    else line0_valid <= 1'b0;
                end
                if (source_crop && source_x == 10'd599) begin
                    if (source_y[0]) begin
                        line1_tag <= source_crop_row;
                        line1_valid <= 1'b1;
                    end else begin
                        line0_tag <= source_crop_row;
                        line0_valid <= 1'b1;
                    end
                    if (source_y == 9'd120)
                        first_line_ready <= 1'b1;
                end
            end
        end
    end
endmodule
