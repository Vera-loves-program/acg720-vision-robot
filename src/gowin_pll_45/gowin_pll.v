// Chapter 28 GW5AT60B wrapper and PLL_INIT, retuned for ATK-7016 at 45 MHz.
module Gowin_PLL_45(
    clkin,
    clkout0,
    lock,
    mdclk,
    reset
);


input clkin;
output clkout0;
output lock;
input mdclk;
input reset;
wire [7:0] mdrdo;
wire [1:0] wMdOpc;
wire wMdAInc;
wire [7:0] wMdDIn;
wire [7:0] wMdQOut;
wire pll_lock;
wire pll_rst;


    Gowin_PLL_45_MOD u_pll(
        .clkout0(clkout0),
        .lock(pll_lock),
        .mdrdo(wMdQOut),
        .clkin(clkin),
        .reset(pll_rst),
        .mdclk(mdclk),
        .mdopc(wMdOpc),
        .mdainc(wMdAInc),
        .mdwdi(wMdDIn)
    );


    PLL_INIT u_pll_init(
        .I_RST(reset),
        .O_RST(pll_rst),
        .I_LOCK(pll_lock),
        .O_LOCK(lock),
        .I_MD_CLK(mdclk),
        .O_MD_INC(wMdAInc),
        .O_MD_OPC(wMdOpc),
        .O_MD_WR_DATA(wMdDIn),
        .I_MD_RD_DATA(wMdQOut),
        .PLL_INIT_BYPASS(1'b0),
        .MDRDO(mdrdo),
        .MDOPC(2'b00),
        .MDAINC(1'b0),
        .MDWDI(8'h0)
    );
    defparam u_pll_init.CLK_PERIOD = 20;
    // VCO/input = MDIV * FBDIV / IDIV = 36 / 2 = 18.
    // PLL_INIT selects its analog initialization range from this ratio.
    defparam u_pll_init.MULTI_FAC = 18;


endmodule
