<#
simulation/windows/check_mp_plugin.ps1 -- PURPOSE: compile the ASTRA Mission Planner plugin (mission_planner_plugin\astra_gcs.cs)
OUTSIDE Mission Planner, against Mission Planner's own program files, so a mistake shows up here with a line number
instead of the plugin silently not appearing (Mission Planner skips a plugin that does not compile). It uses the C#
compiler that comes with Windows (.NET Framework 4, C# 5), which is stricter than Mission Planner's, so code that passes
here also compiles inside Mission Planner. Nothing is installed or changed; the test build goes to the temp folder.

Run:  powershell -NoProfile -ExecutionPolicy Bypass -File simulation\windows\check_mp_plugin.ps1 [-MissionPlannerDir "C:\...\Mission Planner"]
Exit: 0 compiles; 1 compile errors (printed); 2 Mission Planner or the compiler not found.
#>
param([string]$MissionPlannerDir = 'C:\Program Files (x86)\Mission Planner')

$source = Join-Path $PSScriptRoot 'mission_planner_plugin\astra_gcs.cs'
$csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (-not (Test-Path $csc)) { $csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe' }
if (-not (Test-Path $csc)) { Write-Host 'No C# compiler found (.NET Framework 4 csc.exe).'; exit 2 }
if (-not (Test-Path (Join-Path $MissionPlannerDir 'MissionPlanner.exe'))) { Write-Host "Mission Planner not found in $MissionPlannerDir"; exit 2 }

$fromMissionPlanner = 'MissionPlanner.exe', 'MissionPlanner.ArduPilot.dll', 'MissionPlanner.Utilities.dll', 'MissionPlanner.Controls.dll',
                      'MissionPlanner.Comms.dll', 'MAVLink.dll', 'GMap.NET.Core.dll', 'GMap.NET.WindowsForms.dll', 'GMap.NET.Drawing.dll',
                      'MissionPlanner.Drawing.dll', 'Interfaces.dll', 'OpenTK.dll', 'OpenTK.GLControl.dll', 'System.ValueTuple.dll', 'System.Memory.dll'
$references = @()
foreach ($name in $fromMissionPlanner) {
    $path = Join-Path $MissionPlannerDir $name
    if (Test-Path $path) { $references += "/reference:`"$path`"" } else { Write-Host "note: $name not found (skipped)" }
}
$framework = Split-Path $csc
foreach ($name in 'System.dll', 'System.Core.dll', 'System.Drawing.dll', 'System.Windows.Forms.dll', 'netstandard.dll') {
    $path = Join-Path $framework $name
    if (-not (Test-Path $path)) { $path = Join-Path (Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319') $name }
    if (Test-Path $path) { $references += "/reference:`"$path`"" }
}
$out = Join-Path $env:TEMP 'astra_gcs_check.dll'
$arguments = @('/nologo', '/target:library', '/platform:anycpu', '/warn:4', "/out:`"$out`"") + $references + @("`"$source`"")
$output = & $csc @arguments 2>&1
$errors = @($output | Where-Object { $_ -match ': error ' })
$warnings = @($output | Where-Object { $_ -match ': warning ' })
$output | Where-Object { $_ -match ': (error|warning) ' } | ForEach-Object { ($_ -replace [regex]::Escape($PSScriptRoot + '\'), '') } | Select-Object -First 40
if ($errors.Count -gt 0 -or $LASTEXITCODE -ne 0) {
    if ($errors.Count -eq 0) { $output | Select-Object -First 20 }
    Write-Host "FAILED: $($errors.Count) error(s), $($warnings.Count) warning(s)"
    exit 1
}
Remove-Item $out -ErrorAction SilentlyContinue
Write-Host "OK: the plugin compiles against Mission Planner in $MissionPlannerDir ($($warnings.Count) warning(s))"
exit 0
