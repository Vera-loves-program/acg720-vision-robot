// Position-servo pulses: 50 Hz, 1 us resolution, frame-boundary updates.
// pulse_us is a COMMAND, not a measurement of shaft angle/current.
// First enable starts from 1500 us: the shaft's initial position is unknown.
// Keep the original pulse timing for this commissioning build. Coarse/fine
// changes belong to the controller. Disable does not disconnect servo power.
module servo_pulse_output #(
    parameter integer CLK_HZ=50000000,
    parameter integer MIN_US=500,
    parameter integer MAX_US=2500
)(
    input wire clk, reset_n, enable,
    input wire [11:0] pulse_us,
    output reg [11:0] applied_us,
    output wire active,
    output wire pwm
);
    localparam integer US_CYCLES=CLK_HZ/1000000;
    localparam integer FRAME_CYCLES=US_CYCLES*20000;
    reg [31:0] frame_count;
    reg frame_enabled;
    wire [11:0] bounded_us=pulse_us<MIN_US ? MIN_US : pulse_us>MAX_US ? MAX_US : pulse_us;
    assign active=reset_n && enable && frame_enabled;
    assign pwm=active && frame_count<applied_us*US_CYCLES;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            frame_count<=0;frame_enabled<=0;applied_us<=1500;
        end else begin
            if (frame_count==FRAME_CYCLES-1) begin
                frame_count<=0;
                frame_enabled<=enable;
                if (enable) begin
                    if (applied_us<bounded_us) applied_us<=applied_us+1'b1;
                    else if (applied_us>bounded_us) applied_us<=applied_us-1'b1;
                end
            end else frame_count<=frame_count+1'b1;
            if (!enable) frame_enabled<=0;
        end
    end
endmodule
