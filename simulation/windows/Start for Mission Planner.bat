@echo off
rem ===========================================================================
rem  Start for Mission Planner.bat  --  double-click: set up the whole run, then YOU fly it in Mission Planner
rem
rem  PURPOSE: the competition procedure in the simulator. Starts everything inside the Ubuntu-22.04 WSL
rem  distro, from the Linux-side repo copy ~/ODLC_Machine_Inferencing_System_2025-2026 (last build):
rem      bash simulation/scripts/demo.sh runway --manual
rem  That is Gazebo, RViz, the demo panel, ArduCopter SITL, the SIYI camera emulator, the team's real flight
rem  stack (MAVROS, video_cam, new_od, the photo trigger, waypoint_manager, main_controller) and the MAVLink
rem  copy for Mission Planner on Windows (UDP 14550). Then it WAITS: nothing is uploaded, armed or started
rem  automatically. This window prints the Mission Planner steps (connect, Load WP File, Write WPs, Arm,
rem  Auto) and then every mission phase as it happens. Everything stays open after the landing;
rem  START OVER in the demo panel, or Start Over.bat, puts everything back for another run.
rem  This window stays open at the end.
rem ===========================================================================
setlocal
title ASTRA - Fly it from Mission Planner
for %%I in ("%~dp0..\..") do set "ASTRA_WINDOWS_REPO=%%~fI"
set "WSLENV=ASTRA_WINDOWS_REPO/u:%WSLENV%"

echo.
echo   ASTRA - FLY IT FROM MISSION PLANNER (the competition procedure)
echo   ----------------------------------------------------------------
echo   Starting the simulator and the team's flight code (1-3 minutes). Then it is YOUR turn:
echo   the exact Mission Planner steps appear below when everything is ready.
echo   The mission file to load in Mission Planner (FLIGHT PLAN, then LOAD WP FILE):
echo     %ASTRA_WINDOWS_REPO%\ros2_ws\src\simulation\sim_worlds\missions\runway_survey.waypoints
echo.
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/ODLC_Machine_Inferencing_System_2025-2026 || { echo 'No Linux copy of the repo yet: double-click Rebuild.bat first.'; exit 1; }; bash simulation/scripts/demo.sh runway --manual"
set "RC=%ERRORLEVEL%"
echo.
if not "%RC%"=="0" goto failed
echo   The flight is over and everything STAYS OPEN.
echo   Another run: START OVER in the demo panel (click twice) or Start Over.bat,
echo   then repeat the Mission Planner steps (Load WP File, Write WPs, Arm, Auto).
echo   Close it all: Stop Simulation.bat
goto end
:failed
echo   Something went wrong (code %RC%): read the messages above.
echo   Common fix: Stop Simulation.bat, then try again.
:end
echo.
pause
