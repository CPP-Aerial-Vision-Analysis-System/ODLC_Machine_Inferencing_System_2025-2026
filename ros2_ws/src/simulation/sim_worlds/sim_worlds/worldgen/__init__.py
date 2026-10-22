"""
sim_worlds/worldgen -- PURPOSE: the building blocks the world generators (scripts/build_runway.py and
scripts/build_suas_field.py) use to write Gazebo worlds. Needs numpy and Pillow (only to RE-GENERATE worlds).

  sdf.py         turn "parts" (one shape + colour, or one mesh) into SDF <model> text; rotation helpers
  assets.py      the realistic Gazebo Fuel models we use (pinned versions, exact placement, bounding boxes)
  primitives.py  objects built from simple shapes (debris, bushes, tent, fallback mannequin, vehicles, people)
  meshes.py      tiny OBJ mesh writers (ground grid, decal quad, dome tent, creek ribbon, ...)
  textures.py    procedural textures painted with numpy (grass, dry/wet patches, dirt, tent fabric, pad, runway)
  layout.py      a "Placer" that finds free spots so objects never overlap
  world.py       world header (physics, systems, GPS origin, sun), the demo GUI, and the config/truth writers
"""
