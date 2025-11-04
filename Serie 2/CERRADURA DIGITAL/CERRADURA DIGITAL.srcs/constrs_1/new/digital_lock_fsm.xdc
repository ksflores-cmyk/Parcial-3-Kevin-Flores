## =========================================================
## Constraints para digital_lock_fsm en Basys 3 (ejemplo)
## =========================================================

## Reloj de 100 MHz
set_property PACKAGE_PIN W5 [get_ports {clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk}]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {clk}]

## Reset (BTNC)
set_property PACKAGE_PIN U18 [get_ports {rst}]
set_property IOSTANDARD LVCMOS33 [get_ports {rst}]

## Botón capturar (BTNU)
set_property PACKAGE_PIN T18 [get_ports {btn_capturar}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_capturar}]

## Botón confirmar (BTNL)
set_property PACKAGE_PIN W19 [get_ports {btn_confirmar}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_confirmar}]

## Botón limpiar (BTNR)
set_property PACKAGE_PIN T17 [get_ports {btn_limpiar}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_limpiar}]

## Dígitos de entrada desde switches SW0..SW3
set_property PACKAGE_PIN V17 [get_ports {digit_in[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digit_in[0]}]

set_property PACKAGE_PIN V16 [get_ports {digit_in[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digit_in[1]}]

set_property PACKAGE_PIN W16 [get_ports {digit_in[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digit_in[2]}]

set_property PACKAGE_PIN W17 [get_ports {digit_in[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digit_in[3]}]

## =========================================================
## Si quieres ver algo en LEDs, hazlo en el top, por ejemplo:
##   assign led[1:0] = intentos_usados;
##   assign led[3:2] = bloqueo_restante;
##   ...
## y luego los constraints típicos:
## set_property PACKAGE_PIN U16 [get_ports {led[0]}]
## set_property PACKAGE_PIN E19 [get_ports {led[1]}]
## ...
## =========================================================
