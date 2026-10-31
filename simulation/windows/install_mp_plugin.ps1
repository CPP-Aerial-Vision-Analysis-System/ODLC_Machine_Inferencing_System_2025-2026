<#
simulation/windows/install_mp_plugin.ps1 -- PURPOSE: put the ASTRA ground-station plugin into Mission Planner (or take it
out again). The plugin is ONE C# file, mission_planner_plugin\astra_gcs.cs; Mission Planner compiles every .cs file in its
"plugins" folder when it starts, so installing means copying that file there and restarting Mission Planner. After that
Mission Planner shows the ASTRA tab on its DATA screen and the drone camera in the HUD (see the file's own header).

Run (Windows PowerShell):
    powershell -NoProfile -ExecutionPolicy Bypass -File simulation\windows\install_mp_plugin.ps1            install / update
    powershell -NoProfile -ExecutionPolicy Bypass -File simulation\windows\install_mp_plugin.ps1 -Remove    uninstall
    ... -MissionPlannerDir "E:\Mission Planner"     when Mission Planner is installed somewhere else
mission_planner.sh start (and the old panel's button) installs/updates it automatically before opening Mission Planner.
Exit: 0 done; 1 Mission Planner not found; 2 the plugins folder is not writable (run once as administrator, or copy by hand).
#>
param([string]$MissionPlannerDir = '', [switch]$Remove)

$source = Join-Path $PSScriptRoot 'mission_planner_plugin\astra_gcs.cs'
$candidates = @($MissionPlannerDir, $env:MISSION_PLANNER_DIR, 'C:\Program Files (x86)\Mission Planner', 'C:\Program Files\Mission Planner') | Where-Object { $_ }
$dir = $candidates | Where-Object { Test-Path (Join-Path $_ 'MissionPlanner.exe') } | Select-Object -First 1
if (-not $dir) { Write-Host "Mission Planner not found (looked in: $($candidates -join '; ')). Give -MissionPlannerDir."; exit 1 }
$target = Join-Path $dir 'plugins\astra_gcs.cs'

try {
    if ($Remove) {
        if (Test-Path $target) { Remove-Item $target -Force; Write-Host "Removed $target (restart Mission Planner)." } else { Write-Host 'Not installed.' }
        exit 0
    }
    New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
    $same = (Test-Path $target) -and ((Get-FileHash $target).Hash -eq (Get-FileHash $source).Hash)
    if ($same) { Write-Host "Already up to date: $target" }
    else { Copy-Item $source $target -Force; Write-Host "Installed: $target" }
} catch {
    Write-Host "Could not write to $(Split-Path $target): $($_.Exception.Message). Run this once from an administrator PowerShell, or copy $source there by hand."
    exit 2
}
if (Get-Process MissionPlanner -ErrorAction SilentlyContinue) {
    if (-not $same) { Write-Host 'Mission Planner is open: close and reopen it to load the new version.' }
} else {
    Write-Host 'Start Mission Planner: the ASTRA tab is on the DATA screen (bottom left), the camera shows in the HUD when the simulator runs.'
}
exit 0
