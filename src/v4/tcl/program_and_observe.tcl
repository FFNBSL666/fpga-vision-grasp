set root [file normalize [file join [file dirname [info script]] ..]]
source [file join $root tcl program_when_requested.tcl]
set samples {}
set deadline [expr {[clock milliseconds]+45000}]
while {[clock milliseconds]<$deadline} {
    set snapshot [candidate_status]
    if {[candidate_value 2]} {error "CANDIDATE_INITIALIZATION_FAULT: $snapshot"}
    if {[candidate_value 0]==4 && [candidate_value 4]>=1} {break}
    after 300
}
if {[candidate_value 0]!=4} {error "CANDIDATE_INITIAL_POSE_NOT_REACHED: $snapshot"}
for {set i 0} {$i<5} {incr i} {
    set snapshot [candidate_status]
    # A stale GUI/JTAG client can corrupt readback. Never certify impossible
    # counters as motion feedback or print a success marker from such data.
    if {[candidate_value 0]>12 || [candidate_value 3]>18 ||
        [candidate_value 12]>2 || [candidate_value 18]>7 ||
        [candidate_value 14]<500 || [candidate_value 14]>2500 ||
        [candidate_value 4]>11 || [candidate_value 5]>1 ||
        [candidate_value 15]>4 || [candidate_value 19]>10000} {
        error "PROGRAMMED_BUT_LIVE_READBACK_INCONSISTENT: $snapshot; close the stale GUI hardware target before reading again"
    }
    lappend samples $snapshot
    after 600
}
set f [open [file join $root reports programmed_live_samples.json] w]
puts $f "\[[join $samples ,]\]"
close $f
puts "PASS: ISOLATED_CANDIDATE_PROGRAMMED_INITIAL_POSE_AND_LIVE_STATUS_RECORDED"
puts $snapshot
