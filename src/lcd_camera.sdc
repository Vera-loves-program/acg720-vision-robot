create_clock -name clk50M -period 20.000 [get_ports {clk50M}]
create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
create_generated_clock -name lcd_pclk -source [get_ports {clk50M}] -multiply_by 9 -divide_by 10 [get_ports {TFT_clk}]
# 45 MHz LCD clock and physical output timing follow the working project.
# The 2 ns output margin is provisional, as panel timing data was not supplied.
set_output_delay -clock lcd_pclk -clock_fall -max 2.000 [get_ports {TFT_rgb[*] TFT_de TFT_hs TFT_vs}]
set_output_delay -clock lcd_pclk -clock_fall -min -2.000 [get_ports {TFT_rgb[*] TFT_de TFT_hs TFT_vs}]
