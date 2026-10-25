"""Check SDC ratios against the existing DDR PLL/IP configuration.

Read-only source/config checks. No HDL or Gowin tools are executed. In
particular, this cannot prove that get_pins resolves in a placed netlist.
"""
from configparser import ConfigParser
from fractions import Fraction
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SDC = (ROOT / 'src/lcd_interaction.sdc').read_text(encoding='utf-8')
PLLMOD = (ROOT / 'src/vendor/ddr_pll/ddr_pll_mod.v').read_text(encoding='utf-8')
PLL = (ROOT / 'src/vendor/ddr_pll/ddr_pll.v').read_text(encoding='utf-8')
TOP = (ROOT / 'src/vision_robot_ui_top.v').read_text(encoding='utf-8')
BRIDGE = (ROOT / 'src/vendor/ddr3_ctrl_2port/ddr3_ctrl_2port.v').read_text(encoding='utf-8')
CONFIG = ConfigParser()
CONFIG.read(ROOT / 'src/vendor/ddr3_memory_interface/ddr3_memory_interface.ipc')


def pll_parameter(name):
    match = re.search(r'defparam\s+PLLA_inst\.' + name + r'\s*=\s*"?(\d+)"?\s*;', PLLMOD)
    assert match, name
    return int(match[1])


base = Fraction(pll_parameter('FCLKIN'))
memory = base * pll_parameter('MDIV_SEL') / pll_parameter('IDIV_SEL') / pll_parameter('ODIV2_SEL')
assert pll_parameter('MDIV_FRAC_SEL') == pll_parameter('ODIV0_FRAC_SEL') == 0
controller_ratio, memory_ratio = map(int, CONFIG['Config']['CLK_RATIO'].split(':'))
ui = memory * controller_ratio / memory_ratio
assert memory == int(CONFIG['Config']['MEMORY_CLK']) == 400
assert controller_ratio == 1 and memory_ratio == 4 and ui == 100

# Resolve the source chain using its actual instance/port connections, rather
# than treating the historical clk_200m port name as a clock specification.
for text, pattern in (
    (TOP, r'ddr_pll\s+u_ddr_pll\s*\([^;]*\.clkin\(clk50M\)[^;]*\.clkout2\(ddr_clk400\)'),
    (PLL, r'ddr_pll_MOD\s+u_pll\s*\([^;]*\.clkout2\(clkout2\)'),
    (PLLMOD, r'PLLA\s+PLLA_inst\s*\([^;]*\.CLKOUT2\(clkout2\)'),
    (TOP, r'ddr3_ctrl_2port\s+u_framebuffer\s*\([^;]*\.clk_200m\(ddr_clk400\)'),
    (BRIDGE, r'DDR3_Memory_Interface_Top\s+DDR3_Memory_Interface_Top\s*\([^;]*\.memory_clk\(clk_200m\)'),
):
    assert re.search(pattern, text, re.S), pattern

commands = [line.strip() for line in SDC.splitlines()
            if line.strip() and not line.lstrip().startswith('#')]
source_pin = 'u_ddr_pll/u_pll/PLLA_inst/CLKOUT2'
ui_pin = 'u_framebuffer/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT'
memory_command = (
    'create_generated_clock -name ddr_memory_clk -source [get_ports {clk50M}] '
    '-master_clock clk50M -multiply_by 8 [get_pins {' + source_pin + '}]'
)
ui_command = (
    'create_generated_clock -name ddr_ui_clk -source [get_pins {' + source_pin + '}] '
    '-master_clock ddr_memory_clk -divide_by 4 [get_pins {' + ui_pin + '}]'
)
assert commands.count(memory_command) == commands.count(ui_command) == 1
assert commands.index(memory_command) < commands.index(ui_command)
assert base * 8 == memory and memory / 4 == ui
assert 'create_clock -name clk50M -period 20.000 [get_ports {clk50M}]' in commands
assert 'create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]' in commands
assert 'create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]' in commands
assert 'create_generated_clock -name lcd_pclk -source [get_ports {clk50M}] -multiply_by 9 -divide_by 10 [get_ports {TFT_clk}]' in commands
assert not any('set_clock_groups' in line for line in commands)
assert sum(line.startswith('set_false_path') for line in commands) == 1
assert all('ddr' not in line.lower() for line in commands if line.startswith('set_false_path'))

print('PASS: actual PLL/IP ratios 50 MHz -> 400 MHz -> 100 MHz; source instance/port chain')
print('PASS: explicit memory/UI generated clocks; original non-DDR clocks and timing analysis retained')
print('NO HDL/Gowin execution. Next P&R must verify pin resolution, TA1132 removal and real timing.')
