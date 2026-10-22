@echo off
rem ===========================================================================
rem  Rebuild.bat  --  double-click after changing code: copy the repo to Linux and build it
rem
rem  PURPOSE: runs simulation/setup/03_build.sh inside the Ubuntu-22.04 WSL distro, started from THIS
rem  Windows folder of the repo. It mirrors the repo to the Linux-side copy
rem  ~/ODLC_Machine_Inferencing_System_2025-2026 (photos and build results there are kept) and builds
rem  everything with colcon: the team's packages and the simulation packages. The other launchers in
rem  this folder then use the new build. A few minutes the first time, about 1-2 minutes after that.
rem  Close the simulator first with Stop Simulation.bat: a running demo keeps using the old build.
rem  This window stays open at the end.
rem ===========================================================================
setlocal
title ASTRA - Rebuild
for %%I in ("%~dp0..\..") do set "ASTRA_WINDOWS_REPO=%%~fI"

echo.
echo   ASTRA - copying the repo to Ubuntu and building it
echo   ---------------------------------------------------
echo   From: %ASTRA_WINDOWS_REPO%
echo   To:   ~/ODLC_Machine_Inferencing_System_2025-2026 inside Ubuntu-22.04
echo.
wsl.exe -d Ubuntu-22.04 --cd "%ASTRA_WINDOWS_REPO%" -- bash -lc "bash simulation/setup/03_build.sh"
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" goto failed
echo   Build finished. Start the demo with Start Search Demo.bat or Start for Mission Planner.bat.
goto end
:failed
echo   The build FAILED (code %RC%): read the messages above.
echo   The full log is in ~/.astra_sim/logs/build_*.log inside Ubuntu.
:end
echo.
pause
