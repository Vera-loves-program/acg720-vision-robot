// Shared unsigned restoring divider: one quotient bit per clk, 32 iterations.
// start is accepted only while !busy. Operands are latched on acceptance.
// done is a one-cycle pulse; outputs remain stable until another completion.
// Division by zero completes immediately with quotient=ffffffff and remainder
// equal to numerator. Callers use fixed nonzero divisors and bound operands.
// Deliberately no HDL / or %: low-rate UI arithmetic must not infer a long
// single-cycle combinational divider on the 50 MHz actuator/control clock.
module unsigned_divmod (
    input wire clk,reset_n,start,
    input wire [31:0] numerator,denominator,
    output reg busy,done,
    output reg [31:0] quotient,remainder
);
    reg [31:0] dividend_bits,divisor,quotient_work;
    reg [32:0] remainder_work;
    reg [4:0] bit_count;
    wire [32:0] shifted={remainder_work[31:0],dividend_bits[31]};
    wire [32:0] trial=shifted-{1'b0,divisor};
    wire accept_subtraction=!trial[32];
    wire [32:0] next_remainder=accept_subtraction ? trial : shifted;
    wire [31:0] next_quotient={quotient_work[30:0],accept_subtraction};
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy<=0;done<=0;quotient<=0;remainder<=0;
            dividend_bits<=0;divisor<=0;quotient_work<=0;remainder_work<=0;bit_count<=0;
        end else begin
            done<=0;
            if (!busy) begin
                if (start) begin
                    if (denominator==0) begin done<=1;quotient<=32'hffffffff;remainder<=numerator;end
                    else begin
                        dividend_bits<=numerator;divisor<=denominator;
                        quotient_work<=0;remainder_work<=0;bit_count<=0;busy<=1;
                    end
                end
            end else begin
                dividend_bits<={dividend_bits[30:0],1'b0};
                quotient_work<=next_quotient;remainder_work<=next_remainder;
                if (bit_count==31) begin
                    quotient<=next_quotient;remainder<=next_remainder[31:0];busy<=0;done<=1;
                end else bit_count<=bit_count+1'b1;
            end
        end
    end
endmodule
