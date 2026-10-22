@echo off
rem ===========================================================================
rem  Start Search Demo.bat  --  double-click: the full search demo, kept open
rem
rem  PURPOSE: starts the whole ASTRA search demo inside the Ubuntu-22.04 WSL distro, from the Linux-side
rem  copy of the repo (~/ODLC_Machine_Inferencing_System_2025-2026, made and updated by Rebuild.bat):
rem      bash simulation/scripts/demo.sh runway --keep
rem  Gazebo, RViz and the demo panel open, the team's real flight code flies the search, finds the hiker
rem  and the tent, drops both payloads and lands. Then EVERYTHING STAYS OPEN: START OVER in the demo
rem  panel, or Start Over.bat, flies it again; Stop Simulation.bat closes it all.
rem  It reuses the LAST build, so it starts quickly. After changing code, double-click Rebuild.bat first.
rem  This window shows the progress and stays open at the end.
rem ===========================================================================
setlocal
title ASTRA - Search Demo
rem The Windows folder of this repo (this file sits in simulation\windows inside it), so messages can name
rem Windows paths, e.g. the mission file for Mission Planner. WSLENV hands the variable to Ubuntu.
for %%I in ("%~dp0..\..") do set "ASTRA_WINDOWS_REPO=%%~fI"
set "WSLENV=ASTRA_WINDOWS_REPO/u:%WSLENV%"

echo.
echo   ASTRA SEARCH DEMO - Gazebo simulator + the team's real flight code
echo   -------------------------------------------------------------------
echo   Starting inside Ubuntu-22.04 (WSL). The first start takes 1-2 minutes.
echo   Gazebo, RViz and the demo panel will open; the drone then flies the search by itself.
echo   Mission Planner can watch: connection type UDP, click CONNECT, port 14550.
echo.
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/ODLC_Machine_Inferencing_System_2025-2026 || { echo 'No Linux copy of the repo yet: double-click Rebuild.bat first.'; exit 1; }; bash simulation/scripts/demo.sh runway --keep"
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" goto failed
echo   Finished. Gazebo, RViz and the demo panel STAY OPEN.
echo   Fly it again:  START OVER in the demo panel (click it twice), or double-click Start Over.bat
echo   Close it all:  double-click Stop Simulation.bat
goto end
:failed
echo   Something went wrong (code %RC%): read the messages above.
echo   Common fixes: Stop Simulation.bat and try again; Rebuild.bat if the code changed.
:end
echo.
pause
