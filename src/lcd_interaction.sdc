create_clock -name clk50M -period 20.000 [get_ports {clk50M}]
create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
create_generated_clock -name lcd_pclk -source [get_ports {clk50M}] -multiply_by 9 -divide_by 10 [get_ports {TFT_clk}]
# DDR PLL: 50 MHz * MDIV(16) / IDIV(1) / ODIV2(2) = 400 MHz.
# The DDR3 IP is configured for controller:memory = 1:4, hence 100 MHz UI.
# Explicitly describe the divider which 1.9.11.03 reports as TA1132; this
# changes timing constraints only, not the PLL/IP configuration or clocks.
create_generated_clock -name ddr_memory_clk -source [get_ports {clk50M}] -master_clock clk50M -multiply_by 8 [get_pins {u_ddr_pll/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name ddr_ui_clk -source [get_pins {u_ddr_pll/u_pll/PLLA_inst/CLKOUT2}] -master_clock ddr_memory_clk -divide_by 4 [get_pins {u_framebuffer/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
# 45 MHz LCD clock and physical output timing follow the working project.
# The 2 ns output margin is provisional, as panel timing data was not supplied.
set_output_delay -clock lcd_pclk -clock_fall -max 2.000 [get_ports {TFT_rgb[*] TFT_de TFT_hs TFT_vs}]
set_output_delay -clock lcd_pclk -clock_fall -min -2.000 [get_ports {TFT_rgb[*] TFT_de TFT_hs TFT_vs}]

# UI snapshot is a held-data handshake from 50 MHz to the 125 MHz TX domain.
# ui_video_tx_mux samples mailbox_data only AFTER its three request flops,
# and the producer keeps the bundle unchanged until the ACK returns through
# three 50 MHz flops. It is not sampled on the coincident launch clock edge.
# Bound this DATA bundle to 16 ns (< three 8 ns destination cycles), and omit
# its inapplicable same-edge hold check. Keep setup/max-delay analysis active.
# Scope is only this reviewed mailbox: no whole clock group or DDR exception.
set_max_delay -from [get_regs {ui_mailbox_data*}] -to [get_regs {u_ui_tx_mux/latest_snapshot*}] 16.000
set_false_path -from [get_regs {ui_mailbox_data*}] -to [get_regs {u_ui_tx_mux/latest_snapshot*}] -hold
