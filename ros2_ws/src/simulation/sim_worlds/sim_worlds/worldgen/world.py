"""
sim_worlds/worldgen/world.py -- PURPOSE: the parts of a world file that are the same for every world: the
header (physics, systems, GPS origin, sky, sun), the DEMO GUI layout, and the closing tags.

Simulation contract followed here (see the bringup README):
  * physics step 1 ms, real-time factor 1 (ArduPilot runs in lock-step with Gazebo),
  * systems Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), Imu, NavSat,
  * <spherical_coordinates> EARTH_WGS84, ENU, heading 0, equal to SITL's --home so GPS and Gazebo agree,
  * GUI built for the demo: the 3D view (ogre2) with a good starting camera, CameraTracking (the bringup calls
    the /gui/follow service with "iris" so the view follows the drone), world control, world stats and a big
    "Drone camera (live)" picture of /sim/camera/video/image (the drone's point of view).
"""

import math

from sim_worlds.worldgen.sdf import xml_comment

#: gz topic of the live gimbal camera (sim_models contract)
CAMERA_TOPIC = '/sim/camera/video/image'


def look_at(eye, target):
    """GUI camera pose (x, y, z, roll, pitch, yaw) at `eye` looking at `target` (Gazebo cameras look along +x,
    positive pitch looks down)."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    return (eye[0], eye[1], eye[2], 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx))


def _floating(width=5, height=5):
    return f'''        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">{width}</property>
          <property key="height" type="double">{height}</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>'''


def gui_xml(camera_pose, follow_target='iris', follow_offset=(-12.0, 0.0, 6.0), follow_pgain=0.02,
            camera_topic=CAMERA_TOPIC, panel_size=(640, 400)):
    """The <gui> block of a world.

    camera_pose: (x, y, z, roll, pitch, yaw) of the 3D view before following starts (see look_at()).
    follow_target / follow_offset / follow_pgain: CameraTracking settings; the offset is in the drone's own
    frame (-12 m behind, 6 m above). Following starts when the bringup calls /gui/follow (gz-gui 8 does not
    start it from the config alone).
    panel_size: (width, height) pixels of the "Drone camera (live)" picture. It is a FLOATING card anchored to
    the top-right corner of the 3D view, because gz-gui 8 sizes a DOCKED card to the plugin's minimum width
    (250 px for ImageDisplay) and offers no way to make it bigger from the config; you can still drag it,
    resize it or dock it with its title-bar buttons.
    """
    cam = ' '.join(f'{v:.3f}' for v in camera_pose)
    off = ' '.join(f'{v:g}' for v in follow_offset)
    helpers = [('GzSceneManager', 'Scene Manager'), ('InteractiveViewControl', 'Interactive view control'),
               ('EntityContextMenuPlugin', 'Entity context menu'), ('SelectEntities', 'Select Entities'),
               ('MarkerManager', 'Marker manager'), ('VisualizationCapabilities', 'Visualization Capabilities')]
    out = [f'''    <gui fullscreen="0">
      <!-- 3D view: starts with an overview of the field; once the drone exists the bringup makes it follow -->
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.45 0.45 0.45</ambient_light>
        <background_color>0.62 0.74 0.90</background_color>
        <camera_pose>{cam}</camera_pose>
        <camera_clip>
          <near>0.25</near>
          <far>6000</far>
        </camera_clip>
      </plugin>
''']
    for fname, title in helpers:
        out.append(f'      <plugin filename="{fname}" name="{title}">\n{_floating()}\n      </plugin>\n')
    out.append(f'''      <!-- follow the drone: the bringup calls the gz service /gui/follow (request gz.msgs.StringMsg
           data "{follow_target}", reply gz.msgs.Boolean); the offset and gain below are used then -->
      <plugin filename="CameraTracking" name="Camera Tracking">
{_floating()}
        <follow_target>{follow_target}</follow_target>
        <follow_offset>{off}</follow_offset>
        <follow_pgain>{follow_pgain:g}</follow_pgain>
      </plugin>

      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>

      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>

      <!-- THE DRONE'S POINT OF VIEW: live picture of the gimbal camera (1280 x 720, 25 Hz) -->
      <plugin filename="ImageDisplay" name="Drone camera (live)">
        <gz-gui>
          <title>Drone camera (live)</title>
          <property type="bool" key="showTitleBar">true</property>
          <property type="bool" key="resizable">true</property>
          <property type="double" key="width">{panel_size[0]}</property>
          <property type="double" key="height">{panel_size[1]}</property>
          <property type="double" key="z">2</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
        </gz-gui>
        <topic>{camera_topic}</topic>
        <topic_picker>false</topic_picker>
      </plugin>
    </gui>
''')
    return ''.join(out)


def header(world_name, home, purpose_lines, gui):
    """Start of the world file up to (and including) the sun.

    world_name: <world name>. home: geo.Home (the GPS origin). purpose_lines: list of comment lines that
    explain the file (the PURPOSE paragraph). gui: text from gui_xml().
    """
    comment = '\n'.join(f'  {xml_comment(ln)}' if ln else '' for ln in purpose_lines)
    return f'''<?xml version="1.0" ?>
<!--
{comment}
-->
<sdf version="1.9">
  <world name="{world_name}">
{gui}
    <!-- 1 ms physics steps at real-time factor 1: ArduPilot SITL runs in lock-step with these steps -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <!-- GPS origin = world (0, 0, 0) = the centre of the take-off pad at ground level.
         SITL gets the SAME numbers as its home option: {home.sitl_string()} -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{home.lat:.7f}</latitude_deg>
      <longitude_deg>{home.lon:.7f}</longitude_deg>
      <elevation>{home.alt:.1f}</elevation>
      <heading_deg>{home.heading:g}</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.46 0.47 0.49 1</ambient>
      <background>0.62 0.74 0.90 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <!-- mid-September late-morning sun from the south-east (about 50 degrees up) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <visualize>false</visualize>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 0.96 0.88 1</diffuse>
      <specular>0.35 0.33 0.30 1</specular>
      <direction>-0.321 0.557 -0.766</direction>
    </light>
'''


def footer():
    """Closing tags of a world file."""
    return '  </world>\n</sdf>\n'
