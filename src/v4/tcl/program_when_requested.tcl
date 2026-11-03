# Explicit execution programs the candidate and starts its automatic initial pose.
# This script is prepared for the next requested board trial; it is not a build hook.
if {[lsearch -exact $argv user-authorized]<0} {error "WAITING_FOR_USER_NOTIFICATION: NO_PROGRAMMING_PERFORMED"}
set root [file normalize [file join [file dirname [info script]] ..]]
exec C:/Users/admin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe -B [file join $root scripts verify_delivery.py]
open_hw
connect_hw_server -url localhost:3121
set target [get_hw_targets -quiet localhost:3121/xilinx_tcf/Digilent/210512180081]
if {[llength $target]!=1} {error "Recorded FPGA cable not found"}
current_hw_target $target
set_property PARAM.FREQUENCY 1000000 $target
open_hw_target
set devices [get_hw_devices -quiet -filter {PART =~ "xc7z020*"}]
if {[llength $devices]!=1} {error "Expected one XC7Z020 device"}
set dev [lindex $devices 0]
set_property PROGRAM.FILE [file join $root release v4.bit] $dev
set_property PROBES.FILE [file join $root release v4.ltx] $dev
set_property FULL_PROBES.FILE [file join $root release v4.ltx] $dev
program_hw_devices $dev
refresh_hw_device $dev
if {[get_property REGISTER.CONFIG_STATUS.BIT14_DONE_PIN $dev]!=1 || [get_property REGISTER.CONFIG_STATUS.BIT00_CRC_ERROR $dev]!=0} {error "CONFIGURATION_DONE_CRC_FAILED"}
puts "PROGRAMMED: automatic base-neutral and upper-arm initial pose; no Flash write"
source [file join $root tcl monitor.tcl]
puts [candidate_status]
