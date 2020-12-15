set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) top_astria
set ::env(STD_CELL_LIBRARY) sky130_fd_sc_hd
set ::env(SYNTH_READ_BLACKBOX_LIB) 1

set ::env(VERILOG_FILES) "\
	$script_dir/../../verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/top_astria.v"

set ::env(CLOCK_PORT) ""
set ::env(CLOCK_NET) "stoch_adc_comp.clk"
set ::env(CLOCK_PERIOD) "15"

set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 800 800"
set ::env(DESIGN_IS_CORE) 0
set ::env(GLB_RT_ALLOW_CONGESTION) 1
set ::env(DIODE_INSERTION_STRATEGY) 3
set ::env(GLB_RT_MAXLAYER) 5

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(PL_BASIC_PLACEMENT) 1
set ::env(PL_TARGET_DENSITY) 0.4