# Builds an isolated candidate only. No hardware target is opened.
set root [file normalize [file join [file dirname [info script]] ..]]
set project_file [file join $root vivado v4.xpr]
proc all_verilog {folder} {
    set result [glob -nocomplain [file join $folder *.v]]
    foreach p [glob -nocomplain -types d [file join $folder *]] {set result [concat $result [all_verilog $p]]}
    return $result
}
if {[file exists $project_file]} {open_project $project_file} else {
    create_project v4 [file join $root vivado] -part xc7z020clg400-2
}
add_files -norecurse [all_verilog [file join $root rtl]]
add_files -norecurse [file join $root rtl integration confirmed_grasp_profile.vh]
set_property include_dirs [list [file join $root rtl integration]] [get_filesets sources_1]
add_files -fileset constrs_1 -norecurse [file join $root constraints board_template.xdc]
set_property top fpga_board_top [get_filesets sources_1]
set_property generic {CONFIRMED_GRASP_ENABLE=1 POINTING_GRASP_ENABLE=1 POINTING_PLACE_ENABLE=1 ACTUATOR_OUTPUT_ENABLE=1 LEGACY_DEMO_ENABLE=0 MODEL_DIAGNOSTIC_ENABLE=0 ENABLE_MODEL_ILA=0} [get_filesets sources_1]
file mkdir [file join $root ip]
if {[llength [get_ips -quiet confirmed_grasp_monitor]]==0} {
    create_ip -name vio -vendor xilinx.com -library ip -version 3.0 -module_name confirmed_grasp_monitor -dir [file join $root ip]
}
set props [list CONFIG.C_NUM_PROBE_IN 20 CONFIG.C_NUM_PROBE_OUT 1 CONFIG.C_PROBE_OUT0_WIDTH 1 CONFIG.C_PROBE_OUT0_INIT_VAL 0x0]
set n 0
foreach width {4 1 1 8 32 32 50 1 8 16 1 1 2 16 16 32 16 16 3 32} {
    lappend props CONFIG.C_PROBE_IN${n}_WIDTH $width
    incr n
}
set_property -dict $props [get_ips confirmed_grasp_monitor]
generate_target all [get_ips confirmed_grasp_monitor]
create_ip_run [get_ips confirmed_grasp_monitor]
set_property STEPS.SYNTH_DESIGN.TCL.PRE [file join $root tcl prepare_roms.tcl] [get_runs synth_1]
update_compile_order -fileset sources_1
launch_runs synth_1 -jobs 2
wait_on_run synth_1
if {![string match {*Complete*} [get_property STATUS [get_runs synth_1]]]} {error "ISOLATED_SYNTHESIS_FAILED"}
launch_runs impl_1 -to_step write_bitstream -jobs 2
wait_on_run impl_1
if {![string match {*Complete*} [get_property STATUS [get_runs impl_1]]]} {error "ISOLATED_IMPLEMENTATION_FAILED"}
open_run impl_1
set out [file join $root release]
report_timing_summary -file [file join $out timing.rpt]
report_drc -file [file join $out drc.rpt]
report_utilization -file [file join $out utilization.rpt]
report_io -file [file join $out io.rpt]
set setup [get_timing_paths -delay_type max -max_paths 1]
set hold [get_timing_paths -delay_type min -max_paths 1]
if {[llength $setup]==0 || [llength $hold]==0 || [get_property SLACK $setup]<0 || [get_property SLACK $hold]<0} {error "ISOLATED_ROUTED_TIMING_FAILED"}
if {[llength [get_drc_violations -quiet -filter {SEVERITY == Error}]]>0} {error "ISOLATED_DRC_FAILED"}
foreach {port pin} {clk_50m U18 rst_key_n N16 enable_sw L14 estop_sw K16 camera_uart_rx K14 servo_uart_tx J15 busy_led H15 fault_led L15 locked_led N15} {
    if {[get_property PACKAGE_PIN [get_ports $port]] ne $pin} {error "ISOLATED_PIN_MISMATCH: $port"}
}
if {[llength [get_cells -hier -filter {NAME =~ *confirmed*system/core/model/core/*}]]==0} {error "REAL_MODEL_MISSING"}
write_debug_probes -force [file join $out v4.ltx]
write_checkpoint -force [file join $out v4.dcp]
file copy -force [file join [get_property DIRECTORY [get_runs impl_1]] fpga_board_top.bit] [file join $out v4.bit]
set f [open [file join $out build_metrics.json] w]
puts $f [format {{"status":"PASS","part":"xc7z020clg400-2","wns_ns":%s,"whs_ns":%s,"drc_errors":0,"model_execution":"FPGA","move_ms":5000,"hardware_programmed":false,"flash_written":false}} [get_property SLACK $setup] [get_property SLACK $hold]]
close $f
close_project
puts "PASS: V4_RADIUS_3CM_READY_NOT_PROGRAMMED"
