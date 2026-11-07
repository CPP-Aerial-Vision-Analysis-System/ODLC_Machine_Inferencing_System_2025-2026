<!-- PURPOSE: explain what the simulator's image detector really is, why it mistakes trees for hikers and tents (measured on the real-terrain flights of 2026-11-07), what the opt-in physical size filter does, and give the exact steps to connect your own detection model and check it. -->

# The image detection model: what it is, why trees fool it, how to connect yours

## What detects in the simulator today

The team's detector node `new_od` (package `detection`) cuts every photo into 640 px tiles, runs a YOLO model on each tile (SAHI) and
publishes one `/image_detection` message per photo. The weights the simulator uses by default are
`~/.astra_sim/models/yolo26m.pt`, downloaded by `03_build.sh` from Ultralytics. **Those are the generic COCO weights: 80 everyday classes
(person, car, umbrella, kite, potted plant ...), not trained on aerial photos, mannequins or tents.** The team code turns them into targets
with a small table (`detection/detection_processor.py`, `CLASS_MAP`):

| The model says | The team code calls it |
|---|---|
| `person`, `mannequin`, `doll` | person (class id 0) |
| `umbrella`, `kite` | **tent** (class id 1): a guess that a tent seen from above looks like an umbrella or a kite |
| anything else | object (ignored by the mission brain) |

You can see this yourself: `python3 -m sim_plugins.model_check ~/.astra_sim/models/yolo26m.pt --load` ends with
`classes the team code uses: person <- person; tent <- umbrella, kite`.

## Why trees were taken for hikers and tents (measured)

On 2026-11-07 two real-terrain flights (110 photos each) were taken apart: every detection box was put on the ground (photo position at the
shutter from the telemetry, camera height over the real ground, 81 degree field of view) and matched with what stands there in the world.
The mapping is accurate to about 1 m (the true hiker, seen at 56 % in one photo, landed 0.9 m from its true position).

| What the detector boxed | "person" detections | "tent" detections | Size of those boxes (long side) |
|---|---|---|---|
| tree crowns | 40-43 % | 46-55 % | 6-8 m (up to 20 m) |
| bare ground / grass (bushes, tall grass, shadows) | 51-54 % | 39-44 % | 5-8 m |
| debris, props, vehicles | 0 | 5-8 % | 1-4 m |
| **the real hiker** | **5-6 % (7-8 boxes)** | 0.4 % | **1.1 m** |
| **the real tent** | 0 | **0 boxes** | 2.4 m |

* A **person** box on a tree is typically 8 m long; a hiker (the mannequin) is about 1.1 m. The model only sees "something dark and roundish
  with texture", which in COCO is often a person, and it is *more confident about a cedar (0.90) than about the real hiker (0.56 to 0.82)*.
* The mission brain acts on the **single most confident** detection per class. So the drone dropped the "person" payload 14.6 m (flight 2)
  and 45 m (flight 3) from the hiker: it was sent to a tree.
* The real **tent was never detected as a tent**: there is no tent class in COCO and an open blue dome does not look enough like an umbrella
  or a kite from above, while round tree crowns do. The "tent found" lines in the score reports were false alarms (127 m and 75 m off).
* So the model is not "bad": it is **the wrong model for this job**. It is a stand-in that lets the whole mission run end to end. Judging the
  team's detection quality needs the team's own trained weights (next section), and the simulator is the place to prove them.
* The simulator's trees are also simplified (3D crowns textured from the photo). Rounder, smoother crowns look more like umbrellas, so the
  crown shape was made more irregular on 2026-11-07; real trees still look like trees to a trained model, which is why a trained model is the fix.

## The opt-in physical size filter (a team-code change, off by default)

A hiker is about 1-2 m long, a pop-up tent about 2.4 m. The size of a pixel on the ground follows from the camera's field of view and
height, both saved in each photo's JSON sidecar. `detection/size_filter.py` turns every box into metres and drops a "person" outside
0.3-3.0 m and a "tent" outside 1.0-4.5 m. It changes which detections the mission acts on, so it is **off unless you turn it on**:

```bash
ASTRA_PHYSICAL_SIZE_FILTER=1 bash simulation/scripts/demo.sh suas_field_real      # simulator, this run only
# or as ROS parameters of new_od: physical_size_filter:=true  ground_height_m:=0.0  person_size_m:="[0.3, 3.0]"  tent_size_m:="[1.0, 4.5]"
```

What it would have done to the two flights (offline, from their logged detections): "person" detections **115 -> 21** and **151 -> 29**, every
real hiker detection kept (7 of 7 and 8 of 8); in flight 3 the most confident remaining "person" is the hiker (0.82) instead of a tree (0.90).
In flight 2 a grass patch (0.76) still edges the hiker (0.71). It cannot help the tent: the real tent was not detected at all.
**Flown (flight 4, 2026-11-07, same world, filter on):** the hiker pick was **7.7 m** from the hiker (14.6 m and 45 m without the filter); "person"
false alarms per 108 photos fell from 87 to **7**, person precision from 0.24 to **0.67**, tent false alarms from 916 to 158 (tent precision is still
0.06). The price: person recall fell from 0.82 to 0.61 (some true hiker boxes fall outside 0.3-3.0 m when the ground is higher than `ground_height_m`
assumes, so a lower limit or a ground height helps). The tent is still never found: the weights have no tent class.
`rel_alt_m` is the height above the take-off point; on sloped ground set `ground_height_m` (the real field rises 2-12 m across the search areas).

## Connect your own detection model (exact steps)

Use this to test the team's trained weights in the simulator (the same file goes on the Jetson).

1. **Copy the weights into Ubuntu**, a folder without spaces, never into the repo (model files are not committed):

   ```bash
   mkdir -p ~/.astra_sim/models && cp "/mnt/d/<where it is>/my_detector.pt" ~/.astra_sim/models/
   ```

2. **Check the class names.** The team code only acts on the names in the table above. A model whose class is literally `tent` is
   **ignored** (it becomes "object"). Either export the model with the class named `umbrella` or `kite`, or add your names to `CLASS_MAP` in
   `ros2_ws/src/detection/detection/detection_processor.py` (a team-code change: test it on the real drone too). For people use `person`,
   `mannequin` or `doll`.
3. **Check the file and load it once** (5-15 s):

   ```bash
   python3 -m sim_plugins.model_check ~/.astra_sim/models/my_detector.pt --load
   ```

   Expect ` OK model my_detector.pt: pytorch, ... MB, sha256 ...` and ` .. loaded in ... s on cuda:0 (...): N class(es), ... ms per 640 px tile`
   and the line `classes the team code uses: person <- ...; tent <- ...`. ` XX` lines are errors and say the fix. A TensorRT `.engine` file
   works only on the Jetson; on the PC use the `.pt`.
4. **Fly with it:**

   ```bash
   bash simulation/scripts/demo.sh suas_field_real --model ~/.astra_sim/models/my_detector.pt
   ```

   The model name, size and SHA-256 are written to `~/.astra_sim/runs/<run>/run_meta/detector.json`; the score report shows precision and recall
   per class. To keep the choice, copy `simulation/profiles/custom_model.yaml`, change `model_path`, and run `demo.sh --profile my_model.yaml`.
5. **Go back to the stock weights** by running the same command without `--model`.
6. **Another detector program** (not YOLO, or your own node): set `detector.kind: custom` in a profile; the contract (topics, message, health
   service) is in [PLUGINS.md](PLUGINS.md#detector) and the template in `ros2_ws/src/simulation/sim_plugin_examples/example_detector`.
   `bash simulation/scripts/demo.sh runway --detector truth` replaces detection by the answer key (for mission-logic tests only).

More swaps (ArduPilot, Jetson code): [SWAPPING.md](SWAPPING.md).

## How to read a run (find out what your model really did)

* `~/.astra_sim/runs/<run>/score_report.md`: targets found, drop errors, detections per class with precision and recall.
* `~/.astra_sim/runs/<run>/detections/sahi_detected_*.jpg`: every photo with the boxes drawn, to look at what was boxed.
* `~/.astra_sim/runs/<run>/csv/detections.csv`: every detection (class, confidence, box in pixels, photo position).
* The scorer's "false alarm" flag is coarse; the table above came from placing each box on the ground and matching it with the world.
