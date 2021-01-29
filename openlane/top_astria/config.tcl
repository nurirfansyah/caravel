set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) top_astria
set ::env(STD_CELL_LIBRARY) sky130_fd_sc_hd
set ::env(SYNTH_READ_BLACKBOX_LIB) 1

set ::env(VERILOG_FILES) "\
	$script_dir/../../verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/top_astria.v"

set ::env(CLOCK_PORT) ""
set ::env(CLOCK_NET) "stoch_adc_comp.clk"
set ::env(CLOCK_PERIOD) "20"

set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 1400 1200"
set ::env(GLB_RT_OBS) "met5 0 0 1400 1200"
set ::env(DESIGN_IS_CORE) 0
set ::env(GLB_RT_ALLOW_CONGESTION) 1
set ::env(DIODE_INSERTION_STRATEGY) 1
set ::env(GLB_RT_MAXLAYER) 5
set ::env(PL_OPENPHYSYN_OPTIMIZATIONS) 1

set ::env(VDD_NETS) [list {vccd1} {vccd2} {vdda1} {vdda2}]
set ::env(GND_NETS) [list {vssd1} {vssd2} {vssa1} {vssa2}]

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(PL_BASIC_PLACEMENT) 1
#set ::env(PL_TARGET_DENSITY) 0.15

# If you're going to use multiple power domains, then keep this disabled.
set ::env(RUN_CVC) 0