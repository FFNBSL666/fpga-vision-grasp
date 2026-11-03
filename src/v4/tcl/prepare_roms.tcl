set root [file normalize [file join [file dirname [info script]] ..]]
foreach f [glob [file join $root model_candidates native_qnn_v3_20261024 qnn_*.mem]] {
    file copy -force $f [file join [pwd] [file tail $f]]
}
foreach name {workspace_pose_lut.mem placement_pose_lut.mem pixel_radius_lut.mem vertical_path_lut.mem} {
    file copy -force [file join $root config $name] [file join [pwd] $name]
}
puts "ISOLATED_QNN_AND_POSE_ROMS_READY: [pwd]"
