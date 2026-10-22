"""
sim_worlds/realfield -- PURPOSE: the parts of the REAL-TERRAIN SUAS 2026 field generator ("suas_field_real"):
a Gazebo world built from real, public data of the competition site (Skyway36 droneport, Tulsa OK) instead of
a flat procedural field. The command-line front end is scripts/build_real_field.py; the full story (data
sources, licences, how to regenerate, how to change the targets) is docs/simulation/WORLDS.md.

The generator has two stages, so the team never needs GIS libraries just to USE or re-target the world:

  stage 1 "prepare"  (needs a GIS Python: rasterio, pyproj, laspy + lazrs, numpy, scipy, Pillow)
      raw downloads (fetch_real_field_data.sh)  ->  prepared grids in the world frame
      prepare.py      USGS 3DEP lidar DTM + point cloud, NAIP 2023 photo, OpenStreetMap, building footprints
      trees_detect.py tree tops / heights / crowns from the lidar, checked against the 2023 photo
  stage 2 "build"    (numpy + Pillow + PyYAML only: works in the team's ROS 2 Humble machine and Docker)
      prepared grids  ->  Gazebo assets committed to the package (models/suas_field_real_*)
      rtin.py, glb.py, terrain.py   terrain tiles (fine near the search areas, coarse far away) + collision
      texture.py      ground pictures per tile (NAIP colour, season grade, CC0 detail near the search areas)
      trees.py        real trees as merged low-poly models + closed-forest canopy sheets
      structures.py   buildings from footprints, the pad, flight-line canopies, runway markers
      sampler.py      "how high is the ground here?" read back from the committed terrain meshes
  "targets"          (numpy + PyYAML only, from the COMMITTED files): hiker, tent, debris, missions, truth
      placed on the real ground with a seed, exactly like the other worlds (worldgen.builder.WorldBuilder)

site.py holds every number that describes the real place (coordinates from the RoboNation SUAS 2026 Team
Handbook and its Appendix A, OpenStreetMap, the site-facts research) and the world frame.
"""
