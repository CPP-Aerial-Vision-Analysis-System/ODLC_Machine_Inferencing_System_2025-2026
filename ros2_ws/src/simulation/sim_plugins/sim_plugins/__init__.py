"""
sim_plugins -- PURPOSE: plug-and-play "slots" for the ASTRA Gazebo simulator.

A slot is one replaceable piece of the simulation. Each slot has a written contract (exact topics,
services, message types, QoS, files and ports) and the team's current component as the default:

    autopilot      ArduCopter SITL 4.6.3 (~/sitl/arducopter); any version, a source build, or external
    camera         siyi_emulator (SIYI SDK + media web API at 192.168.144.25); or your own
    detector       the team's detection/new_od; or truth_detector (answer key); or your own
    mission_brain  the team's main/main_controller.py; or your own
    world          runway | suas_field | suas_field_real | any .sdf + config .yaml
    extras         any extra launch files / nodes / programs started with the stack (a logger ...)
    network        where MAVProxy sends MAVLink, where the camera emulator listens (Docker setups)

A "profile" (simulation/profiles/*.yaml) says which component fills each slot. The modules:

    schema        every profile key with its default (= today's behaviour) and allowed values
    profiles      load + merge (extends) + validate a profile, with friendly error messages
    params        read ArduPilot parameter files (Mission Planner .param, QGC .params) and keep only
                  what makes sense in the simulator
    autopilot     turn the autopilot slot into a SITL binary, parameter files and a run folder
    plan          turn a profile into launch arguments and a printable plan (demo.sh / start_sim.sh)
    contracts     the written contract of every slot (also printed by `profile_tool contract`)
    check_slot    the conformance checker: inspects the live ROS graph (ros2 run sim_plugins check_slot)
    siyi_probe    talks to a SIYI-compatible camera (UDP SDK + HTTP media API) for the camera check
    tracker_filter  a copy of the team's tracker.launch.xml without the nodes a profile replaces
    launch_support  helpers the sim_bringup launch files use to apply a profile
    doc_blocks    the generated parts of docs/simulation/PLUGINS.md (profile reference, slot contracts)
    profile_tool  the command line: validate, plan, shell, resolve, list, contract, docs

Everything except check_slot's ROS part and launch_support's launch actions is plain Python and is
unit-tested with pytest (no ROS, no Gazebo).
"""

__all__ = ['schema', 'profiles', 'params', 'autopilot', 'plan', 'contracts', 'tracker_filter', 'launch_support',
           'check_slot', 'siyi_probe', 'doc_blocks', 'profile_tool', 'paths']
