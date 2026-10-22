"""
siyi_emulator -- PURPOSE: a protocol-level emulator of the SIYI gimbal camera (ZR10 / A8 mini) that
last year's video_cam driver talks to: SIYI SDK on UDP 37260, media web server on TCP 82, gimbal and
photo camera in Gazebo. Start it with ``ros2 run siyi_emulator siyi_emulator``; see README.md.

Modules: sdk_protocol (frames, CRC), camera_state (models, zoom), gimbal_control (stabilisation, IK),
command_handler (what each command does/answers), sd_card + media_server (photos over HTTP),
photo (frame -> JPEG), gz_link (Gazebo), emulator (everything together), node (ROS 2 wrapper),
sdk_client (tiny client + CLI ``siyi_client``), netutil (which address to listen on).
"""

__version__ = "1.0.0"
