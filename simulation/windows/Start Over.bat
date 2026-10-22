@echo off
rem ===========================================================================
rem  Start Over.bat  --  double-click: put the running demo back to the start and fly it again
rem
rem  PURPOSE: runs, inside the Ubuntu-22.04 WSL distro (Linux-side repo copy):
rem      bash simulation/scripts/start_over.sh --fly --follow
rem  Gazebo, RViz and the demo panel STAY OPEN. A flying drone lands first; the old photos go into the old
rem  run folder; a fresh drone with BOTH payloads stands on the start point; the autopilot and the team's
rem  flight code restart; the mission is uploaded; then the drone flies the search again (about 2-3 minutes
rem  of preparation). This window then shows every mission phase until the drone has landed.
rem  If no simulator is running, it simply starts a fresh demo.
rem  Same as clicking START OVER twice in the demo panel, then UPLOAD MISSION + START.
rem  This window stays open at the end.
rem ===========================================================================
setlocal
title ASTRA - Start Over
for %%I in ("%~dp0..\..") do set "ASTRA_WINDOWS_REPO=%%~fI"
set "WSLENV=ASTRA_WINDOWS_REPO/u:%WSLENV%"

echo.
echo   ASTRA - START OVER (the windows stay open)
echo   ------------------------------------------
echo   Putting everything back to the start, then flying the search again ...
echo.
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/ODLC_Machine_Inferencing_System_2025-2026 || { echo 'No Linux copy of the repo yet: double-click Rebuild.bat first.'; exit 1; }; bash simulation/scripts/start_over.sh --fly --follow"
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" goto failed
echo   Done. Run it again any time with this file, or with START OVER in the demo panel.
goto end
:failed
echo   START OVER did not finish (code %RC%): read the messages above.
echo   If it keeps failing: Stop Simulation.bat, then Start Search Demo.bat.
:end
echo.
pause
