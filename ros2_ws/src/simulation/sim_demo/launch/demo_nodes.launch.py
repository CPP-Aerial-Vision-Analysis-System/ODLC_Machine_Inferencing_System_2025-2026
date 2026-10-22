"""
sim_demo/launch/demo_nodes.launch.py -- PURPOSE: start the demo tools of the ASTRA simulator
in one go: the demo panel window, the RViz markers, the run scorer and (only when asked) the
truth_detector stand-in. sim_bringup's full demo launch includes this file; it also works on
its own next to an already running simulation.

Arguments (all optional):
    world:=runway            world name (sim_worlds/config/<world>.yaml)
    detector:=real           real = the team's new_od runs elsewhere (nothing started here);
                             truth = start truth_detector (answer-key stand-in, clearly labelled)
    panel:=true              open the demo window
    markers:=true            publish /sim/demo/markers for RViz
    score:=true              record the run and write the score report
    photo_dir:=''            team photo folder ('' = <ws>/src/video_cam/mapping_photos)
    worlds_dir:=''           sim_worlds folder ('' = installed sim_worlds package)
    run_dir:=''              where the score report goes ('' = ~/astra_sim_runs/<world>_<time>)
    mavlink_url:=udpin:127.0.0.1:14553   MAVProxy output the panel's mission buttons use
    mission_link:=auto       auto / mavlink / mavros
    auto_start:=false        true = the panel presses UPLOAD MISSION + START by itself
    geolocation:=filename    truth_detector: filename (like new_od) / projected
    frame_id:=world          RViz fixed frame of the markers
    use_sim_time:=false

Example:  ros2 launch sim_demo demo_nodes.launch.py world:=runway detector:=truth auto_start:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGS = [
    ('world', 'runway', 'world name (sim_worlds/config/<world>.yaml)'),
    ('detector', 'real', 'real = team new_od (started elsewhere); truth = start truth_detector'),
    ('panel', 'true', 'open the demo panel window'),
    ('markers', 'true', 'publish RViz markers on /sim/demo/markers'),
    ('score', 'true', 'record the run and write the score report'),
    ('photo_dir', '', "team photo folder ('' = <ws>/src/video_cam/mapping_photos)"),
    ('worlds_dir', '', "sim_worlds folder ('' = installed package)"),
    ('run_dir', '', "score report folder ('' = ~/astra_sim_runs/<world>_<time>)"),
    ('mavlink_url', 'udpin:127.0.0.1:14553', 'pymavlink URL of the panel mission buttons'),
    ('mission_link', 'auto', 'auto / mavlink / mavros'),
    ('auto_start', 'false', 'press UPLOAD MISSION + START automatically'),
    ('geolocation', 'filename', 'truth_detector lat/lon: filename (like new_od) or projected'),
    ('frame_id', 'world', 'RViz fixed frame of the markers'),
    ('use_sim_time', 'false', 'use the /clock of the simulator'),
]


def generate_launch_description():
    """Build the launch description (see the module docstring)."""
    lc = {name: LaunchConfiguration(name) for name, _, _ in ARGS}
    common = {'world': lc['world'], 'worlds_dir': lc['worlds_dir'], 'photo_dir': lc['photo_dir'],
              'use_sim_time': lc['use_sim_time']}
    actions = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
    actions += [
        Node(package='sim_demo', executable='demo_panel', name='demo_panel', output='screen',
             condition=IfCondition(lc['panel']),
             parameters=[dict(common, detector=lc['detector'], mavlink_url=lc['mavlink_url'],
                              mission_link=lc['mission_link'], auto_start=lc['auto_start'])]),
        Node(package='sim_demo', executable='demo_markers', name='demo_markers', output='screen',
             condition=IfCondition(lc['markers']),
             parameters=[dict(common, frame_id=lc['frame_id'])]),
        Node(package='sim_demo', executable='truth_detector', name='truth_detector',
             output='screen',
             condition=IfCondition(PythonExpression(["'", lc['detector'], "' == 'truth'"])),
             parameters=[dict(common, geolocation=lc['geolocation'])]),
        Node(package='sim_demo', executable='score_run', name='score_run', output='screen',
             condition=IfCondition(lc['score']),
             parameters=[dict(common, run_dir=lc['run_dir'], detector=lc['detector'])]),
    ]
    return LaunchDescription(actions)
