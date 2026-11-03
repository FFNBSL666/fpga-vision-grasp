# Read-only runtime monitor. Command counters are not servo feedback.
source [file join [file dirname [info script]] probe_names.tcl]
set candidate_device [lindex [get_hw_devices -quiet -filter {PART =~ "xc7z020*"}] 0]
set candidate_vio [get_hw_vios -of_objects $candidate_device]
if {[llength $candidate_vio]!=1} {error "Expected the isolated candidate VIO"}
proc candidate_value {number} {
    global candidate_vio candidate_probe_names
    set expected [dict get $candidate_probe_names probe_in${number}]
    set matches {}
    foreach p [get_hw_probes -of_objects $candidate_vio] {
        set name [get_property NAME $p]
        if {[lindex [split $name {[}] 0] eq $expected} {lappend matches $p}
    }
    if {[llength $matches]!=1} {error "Missing monitor probe $number"}
    set raw [get_property INPUT_VALUE [lindex $matches 0]]
    if {![regexp {^[0-9a-fA-F]+$} $raw]} {error "Unknown monitor value"}
    return [expr "0x$raw"]
}
proc candidate_status {} {
    global candidate_vio
    refresh_hw_vio $candidate_vio
    set result [format {{"state":%d,"fault":%d,"error_code":%d,"upper_groups":%d,"completed_cycles":%d,"frame_id":%d,"source_pixel":%d,"destination_pixel":%d,"selection_phase":%d,"base_command_p":%d,"base_command_count":%d,"source_base_p":%d,"destination_base_p":%d}} [candidate_value 0] [candidate_value 2] [candidate_value 3] [candidate_value 4] [candidate_value 5] [candidate_value 8] [candidate_value 9] [candidate_value 13] [candidate_value 12] [candidate_value 14] [candidate_value 15] [candidate_value 16] [candidate_value 17]]
    return "[string range $result 0 end-1],\"active_joint\":[candidate_value 18],\"actuator_commands\":[candidate_value 19]\}"
}
