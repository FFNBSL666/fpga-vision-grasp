## XC7Z020CLG400-2 + baseboard V2.3.2, core schematic 2V5.
## CAMERA: OpenMV P1(TX) -> BASEBOARD J3 pin1 -> B35_L20_P -> K14.
## GROUND: OpenMV GND -> BASEBOARD J3 pin37. Pin38 is +3.3V, NEVER ground.
## Camera uses separate USB power. Bank35 VCCO=3.3V per populated 0-ohm links.
create_clock -name clk_50m -period 20.000 [get_ports clk_50m]
set_property PACKAGE_PIN U18 [get_ports clk_50m]
set_property PACKAGE_PIN N16 [get_ports rst_key_n]
set_property PACKAGE_PIN L14 [get_ports enable_sw]
set_property PACKAGE_PIN K16 [get_ports estop_sw]
set_property PACKAGE_PIN K14 [get_ports camera_uart_rx]
set_property PACKAGE_PIN J15 [get_ports servo_uart_tx]
set_property PACKAGE_PIN H15 [get_ports busy_led]
set_property PACKAGE_PIN L15 [get_ports fault_led]
set_property PACKAGE_PIN N15 [get_ports locked_led]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_50m rst_key_n enable_sw estop_sw camera_uart_rx servo_uart_tx busy_led fault_led locked_led}]
set_property PULLUP true [get_ports {rst_key_n enable_sw estop_sw camera_uart_rx}]
set_property SLEW SLOW [get_ports {servo_uart_tx busy_led fault_led locked_led}]
set_false_path -from [get_ports {rst_key_n enable_sw estop_sw camera_uart_rx}]
