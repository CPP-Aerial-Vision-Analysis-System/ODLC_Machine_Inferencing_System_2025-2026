@echo off
rem ===========================================================================
rem  Stop Simulation.bat  --  double-click: close the simulator and everything it started
rem
rem  PURPOSE: runs, inside the Ubuntu-22.04 WSL distro (Linux-side repo copy):
rem      bash simulation/scripts/stop_sim.sh
rem  It stops ONLY the simulator's own programs: Gazebo and its window, ArduCopter SITL, MAVProxy, the camera
rem  emulator, RViz, the demo panel, and the team's flight stack that the demo or START OVER started.
rem  It finds them by the simulator's session tag, so nothing else on the PC is touched. Mission Planner
rem  stays open: close it yourself, or with CLOSE MISSION PLANNER in the demo panel before stopping.
rem  This window stays open at the end.
rem ===========================================================================
setlocal
title ASTRA - Stop Simulation
echo.
echo   ASTRA - stopping the simulator ...
echo.
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/ODLC_Machine_Inferencing_System_2025-2026 || { echo 'No Linux copy of the repo yet: nothing to stop.'; exit 0; }; bash simulation/scripts/stop_sim.sh"
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" goto failed
echo   The simulator is stopped.
goto end
:failed
echo   Something is still running (see above). It is safe to run this file once more.
:end
echo.
pause
