"""
sim_worlds -- PURPOSE: the Python helpers that come with the simulated worlds (the runway test field and the
SUAS 2026 competition field).

Other packages may import these small, dependency-light modules (they only need PyYAML):
  sim_worlds.geo           GPS latitude/longitude  <->  local Gazebo metres (ArduPilot's own formulas + WGS84)
  sim_worlds.survey        camera footprint maths and lawn-mower survey lanes
  sim_worlds.mission       QGC WPL 110 mission files (read/write) in the exact shape last year's code expects
  sim_worlds.world_config  find and read config/<world>.yaml (world file, SITL home, spawn, mission, truth)
  sim_worlds.truth         read and check truth/<world>.yaml (where the hiker and the tent really are)

The sub-package sim_worlds.worldgen (numpy + Pillow) is only needed to RE-GENERATE the worlds.
"""

__all__ = ['geo', 'survey', 'mission', 'world_config', 'truth']
