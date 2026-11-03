$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
$taskWork=Join-Path $taskRoot 'work\coordinate_test'
Push-Location -LiteralPath $taskWork
try {
    $taskSources=@(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'rtl\model') -Filter '*.v' | ForEach-Object FullName)
    $taskSources+=@(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'rtl\camera') -Filter '*.v' | ForEach-Object FullName)
    $taskSources+=@('common/uart_rx.v','common/uart_tx.v','servo/servo_ascii_encoder.v','servo/servo_group_encoder.v','servo/base_servo_link.v','servo/servo_command_master.v','vision/observation_frame_gate.v','vision/pointing_target_selector.v','vision/pixel_pose_lookup.v','vision/vertical_path_lookup.v','vision/pick_place_target_mapper.v','integration/confirmed_grasp_core.v') | ForEach-Object {Join-Path (Join-Path $taskRoot 'rtl') $_}
    & 'F:\Vivado\2018.3\bin\xvlog.bat' -i (Join-Path $taskRoot 'rtl\integration') @taskSources (Join-Path $taskRoot 'sim\tb_coordinate_base.v')
    if($LASTEXITCODE -ne 0){throw 'Coordinate test compilation failed'}
    & 'F:\Vivado\2018.3\bin\xelab.bat' tb_coordinate_base -s coordinate_base_test
    if($LASTEXITCODE -ne 0){throw 'Coordinate test elaboration failed'}
    & 'F:\Vivado\2018.3\bin\xsim.bat' coordinate_base_test -runall -log right_to_left.log
    if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath 'right_to_left.log' -SimpleMatch 'PASS: COORDINATES_' -Quiet)){throw 'Right-to-left coordinate test failed'}
    & 'F:\Vivado\2018.3\bin\xsim.bat' coordinate_base_test -testplusarg left_to_right -runall -log left_to_right.log
    if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath 'left_to_right.log' -SimpleMatch 'PASS: COORDINATES_' -Quiet)){throw 'Left-to-right coordinate test failed'}
    & 'F:\Vivado\2018.3\bin\xsim.bat' coordinate_base_test -testplusarg same_bearing -runall -log same_bearing.log
    if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath 'same_bearing.log' -SimpleMatch 'PASS: COORDINATES_' -Quiet)){throw 'Forced base command at equal bearing test failed'}
} finally {Pop-Location}
