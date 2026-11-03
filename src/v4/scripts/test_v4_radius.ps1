$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
$taskWork=Join-Path $taskRoot 'work\coordinate_test'
Push-Location -LiteralPath $taskWork
try {
    & 'F:\Vivado\2018.3\bin\xvlog.bat' (Join-Path $taskRoot 'rtl\vision\pixel_pose_lookup.v') (Join-Path $taskRoot 'rtl\vision\pick_place_target_mapper.v') (Join-Path $taskRoot 'sim\tb_v4_radius_selection.v')
    if($LASTEXITCODE -ne 0){throw 'V4 radius compilation failed'}
    & 'F:\Vivado\2018.3\bin\xelab.bat' tb_v4_radius_selection -s v4_radius_test
    if($LASTEXITCODE -ne 0){throw 'V4 radius elaboration failed'}
    & 'F:\Vivado\2018.3\bin\xsim.bat' v4_radius_test -runall -log v4_radius.log
    if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath 'v4_radius.log' -SimpleMatch 'PASS: V4_RADIUS_30MM_' -Quiet)){throw 'V4 radius regression failed'}
} finally {Pop-Location}
