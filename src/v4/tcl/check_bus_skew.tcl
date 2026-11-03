set root [file normalize [file join [file dirname [info script]] ..]]
open_checkpoint [file join $root release vision_base_command_fix.dcp]
if {[llength [get_cells -hier -filter {NAME =~ *path_lookup* && PRIMITIVE_TYPE =~ BMEM.*}]]==0} {error "VERTICAL_PATH_ROM_MISSING"}
if {[llength [get_cells -hier -filter {NAME =~ *base_link*sender*}]]==0} {error "DEDICATED_BASE_LINK_MISSING"}
report_bus_skew -file [file join $root release bus_skew.rpt]
close_design
