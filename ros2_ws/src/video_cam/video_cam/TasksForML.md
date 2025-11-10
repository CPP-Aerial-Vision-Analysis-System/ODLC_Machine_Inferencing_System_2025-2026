# How to create our images to train on

## Example Blender Python Script

This script:

Clears the scene

Loads the models

Randomly places objects

Simulates a drone camera at a fixed altitude

Renders images and produces YOLO .txt annotation files


###   Workflow Overview

    Import your mannequin and tent 3D models into Blender (.obj, .fbx, or .glb formats).

    Randomly place the objects on a textured ground surface.

    Randomize position, scale, rotation, lighting, and partial occlusions.

    Position the camera at an overhead angle to simulate drone imaging.

    Render images.

    Generate YOLO-compatible annotation text files for each frame.

```bash
import bpy
import random
import math
import os

OUTPUT_DIR = "/mnt/data/synthetic_dataset"
NUM_IMAGES = 200
MIN_OBJECTS = 2
MAX_OBJECTS = 6
GROUND_SIZE = 50
CAMERA_HEIGHT = 45  # 150ft in meters
FOV = 81 * (math.pi / 180)

os.makedirs(OUTPUT_DIR, exist_ok=True)

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

bpy.ops.mesh.primitive_plane_add(size=GROUND_SIZE, location=(0,0,0))
ground = bpy.context.active_object

def load_model(path):
    bpy.ops.import_scene.obj(filepath=path)
    return bpy.context.selected_objects[0]

tent_model = load_model("/path/to/tent.obj")
mannequin_model = load_model("/path/to/mannequin.obj")

bpy.ops.object.camera_add(location=(0, 0, CAMERA_HEIGHT))
camera = bpy.context.active_object
camera.data.type = 'PERSP'
camera.data.lens_unit = 'FOV'
camera.data.angle = FOV
bpy.context.scene.camera = camera

def place_random_object(base_model):
    obj = base_model.copy()
    bpy.context.collection.objects.link(obj)
    obj.location.x = random.uniform(-20, 20)
    obj.location.y = random.uniform(-20, 20)
    obj.location.z = 0
    obj.rotation_euler[2] = random.uniform(0, math.pi * 2)
    scale = random.uniform(0.85, 1.2)
    obj.scale = (scale, scale, scale)
    return obj

for i in range(NUM_IMAGES):
    for obj in bpy.context.scene.objects:
        if obj not in (camera, ground, tent_model, mannequin_model):
            bpy.data.objects.remove(obj, do_unlink=True)

    object_count = random.randint(MIN_OBJECTS, MAX_OBJECTS)
    placed_objects = []

    for _ in range(object_count):
        if random.random() < 0.5:
            placed_objects.append(place_random_object(tent_model))
        else:
            placed_objects.append(place_random_object(mannequin_model))

    camera.location.x = random.uniform(-5, 5)
    camera.location.y = random.uniform(-5, 5)

    img_path = f"{OUTPUT_DIR}/image_{i:04d}.png"
    bpy.context.scene.render.filepath = img_path
    bpy.ops.render.render(write_still=True)

    label_path = f"{OUTPUT_DIR}/image_{i:04d}.txt"
    with open(label_path, "w") as f:
        for obj in placed_objects:
            bbox = [camera.matrix_world @ v.co for v in obj.bound_box]
            xs = [p.x for p in bbox]
            ys = [p.y for p in bbox]
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)

            x_center = (xmin + xmax) / 2 / GROUND_SIZE
            y_center = (ymin + ymax) / 2 / GROUND_SIZE
            w = (xmax - xmin) / GROUND_SIZE
            h = (ymax - ymin) / GROUND_SIZE

            cls = 0 if obj == tent_model else 1
            f.write(f"{cls} {x_center} {y_center} {w} {h}\n")

```

## Create images for negatives

Blender Python script that generates negative samples — images with no mannequins or tents, but containing confusing background objects such as rocks, bushes, trees, tarps, debris, or any distractor models you choose.

These negative images will receive empty YOLO label files, which is exactly what you want for training a detector that must learn not to trigger on background clutter.

### Instructions

    Place your distractor models (e.g., tree, bush, rock, crate, random shapes) into a folder.

    Use .obj or .fbx format.

    Update the file paths in the script.

    Run in Blender’s Scripting panel.

```bash
import bpy
import random
import math
import os
from glob import glob

# =======================
# CONFIGURATION
# =======================
OUTPUT_DIR = "/mnt/data/negatives_dataset"
DISTRACTOR_MODELS_DIR = "/mnt/data/distractors"  # folder containing .obj or .fbx models
NUM_IMAGES = 200
MIN_DISTRACTORS = 1
MAX_DISTRACTORS = 6
GROUND_SIZE = 50
CAMERA_HEIGHT = 45  # meters equivalent
HORIZONTAL_FOV_DEG = 81

# =======================
# SETUP DIRECTORIES
# =======================
os.makedirs(OUTPUT_DIR, exist_ok=True)
model_paths = glob(os.path.join(DISTRACTOR_MODELS_DIR, "*.obj")) + \
              glob(os.path.join(DISTRACTOR_MODELS_DIR, "*.fbx"))

if len(model_paths) == 0:
    raise RuntimeError("No distractor models found in directory.")

# =======================
# CLEAR SCENE
# =======================
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# =======================
# GROUND PLANE
# =======================
bpy.ops.mesh.primitive_plane_add(size=GROUND_SIZE, location=(0, 0, 0))
ground = bpy.context.active_object

# =======================
# LOAD MODEL FUNCTION
# =======================
def load_model(path):
    if path.endswith(".obj"):
        bpy.ops.import_scene.obj(filepath=path)
    else:
        bpy.ops.import_scene.fbx(filepath=path)
    return bpy.context.selected_objects[0]

# Pre-load models (so we don’t re-import each time)
loaded_models = [load_model(p) for p in model_paths]
for model in loaded_models:
    model.hide_viewport = True
    model.hide_render = True

# =======================
# CAMERA SETUP
# =======================
bpy.ops.object.camera_add(location=(0, 0, CAMERA_HEIGHT))
camera = bpy.context.active_object
camera.data.lens_unit = 'FOV'
camera.data.angle = math.radians(HORIZONTAL_FOV_DEG)
bpy.context.scene.camera = camera

# =======================
# OBJECT PLACEMENT
# =======================
def place_random_distractor(base_model):
    obj = base_model.copy()
    bpy.context.collection.objects.link(obj)

    obj.location.x = random.uniform(-20, 20)
    obj.location.y = random.uniform(-20, 20)
    obj.location.z = 0

    obj.rotation_euler[2] = random.uniform(0, math.pi * 2)

    scale = random.uniform(0.6, 1.6)
    obj.scale = (scale, scale, scale)

    obj.hide_viewport = False
    obj.hide_render = False
    return obj

# =======================
# GENERATION LOOP
# =======================
for i in range(NUM_IMAGES):
    # Remove previously placed distractors
    for obj in bpy.context.scene.objects:
        if obj not in (camera, ground) and obj not in loaded_models:
            bpy.data.objects.remove(obj, do_unlink=True)

    distractor_count = random.randint(MIN_DISTRACTORS, MAX_DISTRACTORS)
    for _ in range(distractor_count):
        place_random_distractor(random.choice(loaded_models))

    camera.location.x = random.uniform(-5, 5)
    camera.location.y = random.uniform(-5, 5)

    # Render image
    img_path = f"{OUTPUT_DIR}/negative_{i:04d}.png"
    bpy.context.scene.render.filepath = img_path
    bpy.ops.render.render(write_still=True)

    # Create empty YOLO label file
    label_path = f"{OUTPUT_DIR}/negative_{i:04d}.txt"
    open(label_path, "w").close()

```