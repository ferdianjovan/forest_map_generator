# forest_map_generator (ROS2 Jazzy + Gazebo Harmonic)

A ROS 2 package for generating forest simulation environments in Gazebo Harmonic, including
terrain heightmaps, procedural tree placement, and fire/smoke generation.

<p align="center">
  <img src="docs/images/gazebo_overview.png" width="850">
</p>

---


## Table of Contents

- [Overview](#overview)
- [Repository Layout](#repository-layout)
- [Key Components](#key-components)
  - [1. ForestMapGenerator (ROS 2 Node)](#1-forestmapgenerator-ros-2-node)
  - [2. TerrainHelper (Terrain Abstraction Layer)](#2-terrainhelper-terrain-abstraction-layer)
  - [3. TreeGenerator](#3-treegenerator)
  - [4. FireGenerator (Fire Model + Smoke Particle Emitters)](#4-firegenerator-fire-model--smoke-particle-emitters)
  - [5. update_heightmap script (Heightmap → Terrain SDF Update)](#5-update_heightmap-script-heightmap--terrain-sdf-update)
  - [6. ply_to_gazebo_textured pipeline (PLY → Gazebo Tree Model)](#6-ply_to_gazebo_textured-pipeline-ply--gazebo-tree-model)

---

## Overview

This project provides a complete pipeline to build a forest scene for Gazebo from:
1) a terrain heightmap (PNG) used by the `terrain` model, and  
2) tree assets (either built-in models such as `oak_tree` / `pine_tree`, or textured meshes generated from point clouds), and
3) fire assets represented by visible `fire_model` instances and `fog_generator` smoke particle emitters.

Please install python package `wmm-calculator` to avoid getting an error.

Generated worlds also include a standard set of Gazebo system plugins every time the generator writes the output `.world` file. 
This is intentional: the generated world is expected to be ready for terrain physics, scene publication, particle emitters, and common simulated sensors without requiring the user to manually add systems later. 
The inserted systems currently include physics, user commands, scene broadcaster, particle emitter, IMU, air pressure, magnetometer, NavSat, and the Gazebo sensors system using the `ogre2` render engine. 
These plugins are loaded regardless of whether a particular generated scene currently contains fires, smoke, or sensor-equipped vehicles; do not use their presence alone as evidence that those entities were generated. 
If the base world is customized, avoid duplicating the same system plugin declarations unless that duplication is deliberate.

The generated forest poses use Gazebo world-frame coordinates derived from the terrain heightmap, not arbitrary map coordinates. 
`TerrainHelper` reads the terrain model's heightmap `<size>` and `<pos>` values from `models/terrain/model.sdf`, samples the heightmap pixels, and converts accepted tree/fire pixels into local Gazebo `(x, y, z)` poses. 
The `latitude`, `longitude`, and `altitude` values in `configs/map_configuration.yaml` are then used to write `<spherical_coordinates>` and `<magnetic_field>` metadata into the world. 
It is preferable for those geographic values to be representative of, or based on, the same real terrain / heightmap region used to generate the forest world. 
The local placement logic will still work if they do not match, but GPS, magnetic-field, and world-geography assumptions will no longer describe the visible terrain consistently.

The core workflow is:

1) **Acquire a terrain heightmap**
   - Select a geographic region of interest and export a digital elevation model (DEM) as a heightmap.
   - Public DEM-to-heightmap services can be used to obtain real-world terrain heightmaps for almost any location worldwide (e.g., https://dx3377.com/dem/heightmap).
   - Export/download the terrain as a heightmap image and ensure it meets the Gazebo format requirements below.

2) **Prepare a Gazebo-compatible heightmap**
   - Convert or export the heightmap as a single-channel (grayscale) PNG.
   - Ensure the heightmap is square and follows Gazebo heightmap requirements (`N × N`, where `N = 2^k + 1`).
   - Use 8-bit grayscale (`0–255`) unless a higher bit-depth workflow is explicitly required.

3) **Update the terrain model**
   - `scripts/update_heightmap/main.py` updates `models/terrain/model.sdf`, including:
     - heightmap URI
     - heightmap size and vertical scale
     - texture blending parameters
     - terrain pose
   - The script also copies the heightmap PNG into:
     - `models/terrain/materials/textures/`
   - This ensures the heightmap is discoverable by Gazebo at runtime.

4) **Generate the forest world**
   - `forest_map_generator/forest_map_generator.py` (ROS 2 node) generates a new `.world` file by inserting:
     - randomly placed tree `<include>` blocks (slope-aware and minimum-distance constrained)
     - randomly placed fire/smoke blocks using `fire_model` and `fog_generator`
   - Trees and fires are evaluated directly on the heightmap using shared terrain logic.

5) **Create Gazebo-ready tree models from point clouds (optional)**
   - `scripts/ply_to_gazebo_textured/main.py` converts colored `.ply` point clouds into Gazebo-ready tree models under `models/<tree_name>/`, including:
     - `meshes/tree_mesh.dae` (visual mesh)
     - `meshes/tree_collision.stl` (collision mesh)
     - `meshes/<tree_name>_albedo.png` (texture baked via Blender)

This pipeline enables reproducible construction of large-scale forest environments in Gazebo from either synthetic or real-world terrain data.


The package is structured as an `ament_python` ROS 2 package and is intended for research workflows where repeatable generation of natural outdoor scenes is required.

---

## Repository Layout


```text
forest_map_generator/
├── forest_map_generator/
│   └── forest_map_generator.py        # ROS 2 node (tree & fire generation)
│
├── scripts/
│   ├── update_heightmap/
│   │   └── main.py                    # Heightmap → terrain SDF update
│   │
│   └── ply_to_gazebo_textured/
│       ├── main.py                    # PLY → Gazebo tree model pipeline
│       └── bake_vcol_to_texture.py    # Blender texture baking (called by main.py)
│
├── models/
│   ├── terrain/                       # Heightmap-based terrain model
│   ├── fog_generator/                 # Smoke/fog particle textures and color ranges
│   ├── fire_model/                    # Visual fire model placed inside smoke areas
│   ├── oak_tree/                      # Predefined tree model
│   ├── pine_tree/                     # Predefined tree model
│   └── tree*/                         # Auto-generated tree instances
│
├── worlds/
│   └── world.world                    # Base Gazebo world
│
├── launch/
│   ├── gazebo.launch.py
│   └── tree_generator.launch.py
│
└── docs/
    └── images/
        └── gazebo_overview.png

```

---

## Key Components

- 'ForestMapGenerator (ROS 2 node)': generates a new world file by injecting trees, fire models, and smoke particle emitters into a base world.

- 'TerrainHelper': shared utility class for heightmap loading, pixel–world coordinate conversion, and terrain slope computation.

- 'TreeGenerator': slope-aware random tree placement on the heightmap, built on top of TerrainHelper.

- 'FireGenerator': random fire placement on the heightmap, built on top of TerrainHelper. Each fire combines a smoke particle emitter (`fog_generator`) with one or more visible flame models (`fire_model`).

- 'update_heightmap script': updates terrain SDF parameters and ensures the heightmap image is placed in the correct model path for Gazebo.

- 'ply_to_gazebo_textured pipeline': converts colored point clouds into textured Gazebo-ready meshes using Open3D and Blender texture baking.

### 1. ForestMapGenerator (ROS 2 Node)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**

Primary ROS 2 node that procedurally generates a forest simulation world by injecting trees, fire models, and smoke particle emitters into a base Gazebo world.
The node samples valid placements directly on the terrain heightmap, converts heightmap pixels into world-frame poses, and writes a new .world file under worlds/.

**Execution Flow**
1. Load the terrain heightmap and compute local slope information
2. Sample valid tree positions subject to slope and distance constraints
3. Sample valid fire positions subject to minimum distance and optional dirt-layer constraints
4. Convert heightmap pixels to world-frame poses
5. Inject generated tree instances, fire model includes, smoke particle emitters, and required Gazebo particle plugins into a new Gazebo world file

**Launch Command**
```text
ros2 launch forest_map_generator tree_generator.launch.py
```

The node writes a generated world file to the package worlds/ directory (see output_world_file below).

**Parameters**

| Parameter | Type | Description                                                                                                                                                                                                                     |
|----------|------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `num_trees` | `int` | Number of trees to generate and inject into the world.                                                                                                                                                                          |
| `tree_types` | `list[string]` | List of Gazebo model names available under `models/` (e.g., `tree1`–`tree14`). A random type is selected per placement.                                                                                                         |
| `min_tree_distance` | `float` | Minimum allowed distance (meters) between any two trees.                                                                                                                                                                        |
| `max_slope` | `float` | Maximum allowed slope (degrees) for valid placements. Trees are rejected on steep terrain.                                                                                                                                      |
| `output_world_file` | `string` | Output world filename written to `worlds/` (e.g., `world_with_trees.world`).                                                                                                                                                    |
| `plant_tree_above_dirt` | `bool`         | Only allow to put trees in the second blend layer or above (1st Layer Dirt, 2nd Layer Grass, and 3rd Highest Layer Fungi). This corresponds to the `--blend1_min` + `--blend1_fade` height from the **update_heightmap script** |
| `num_fires` | `int` | Number of fire/smoke areas to generate and inject into the world. |
| `min_fire_distance` | `float` | Minimum spacing constraint between generated fire/smoke placement centers. |
| `plant_fire_above_dirt` | `bool` | Only allow fires in the second blend layer or above, using the same dirt-layer threshold logic as `plant_tree_above_dirt`. |
| `min_fire_size` | `float` | Minimum square smoke emitter size. Default: `2.0`. Values are clamped to the supported fire-size range. |
| `max_fire_size` | `float` | Maximum square smoke emitter size. Default: `10.0`. Values are clamped to the supported fire-size range. |

Note: Several of the parameters above are automatically printed during heightmap loading and SDF update for verification and reproducibility.  
These outputs will be explained in detail in a later section.

**Output**
```text
worlds/<output_world_file>
```

What is written into the world

- A list of Gazebo `<include>` blocks, one per generated tree instance

- A list of Gazebo `<include>` blocks, one or more per generated fire area, for `fire_model`

- A list of Gazebo `fire_smoke_<id>` particle emitter models for smoke/fog

- Randomized yaw for visual diversity on trees and fire models

- Tree placement is slope-aware and respects minimum spacing constraints

- Fire placement respects minimum spacing and optional dirt-layer constraints, but does not reject steep slopes

**Assumptions**
- The terrain model and heightmap are pre-loaded in Gazebo
- All tree models listed in `tree_types` exist under `models/`
- `fog_generator` and `fire_model` exist under `models/` and are installed by the package

**Example Launch Parameters**
```text
Node(
    package="forest_map_generator",
    executable="forest_map_generator",
    name="forest_map_generator",
    output="screen",
    parameters=[
        {
            "num_trees": 200,
            "tree_types": [
                "tree1","tree2","tree3","tree4","tree5","tree6","tree7",
                "tree8","tree9","tree10","tree11","tree12","tree13","tree14",
            ],
            "min_tree_distance": 5.0,
            "max_slope": 30.0,
            "plant_tree_above_dirt": True,
            "num_fires": 5,
            "min_fire_distance": 5.0,
            "plant_fire_above_dirt": False,
            "min_fire_size": 2.0,
            "max_fire_size": 10.0,
            "output_world_file": "world_with_trees.world",
        }
    ],
)
```

**Reproducibility**  
For fixed parameters and heightmap input, the generation process is stochastic due to randomized tree placement, tree orientation, tree type selection, fire placement, smoke size, smoke color range, and fire model placement.  
A fixed random seed is planned to be introduced to enable reproducible map generation for benchmarking and evaluation.

### 2. TerrainHelper (Terrain Abstraction Layer)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**

`TerrainHelper` is a shared utility class that encapsulates all terrain-related operations, providing a consistent abstraction over the heightmap-based terrain model used in Gazebo.

It serves as the geometric and physical foundation for both tree and fire generation by:

- loading and validating the terrain heightmap,

- computing local terrain slope,

- converting between heightmap pixel coordinates and world-frame coordinates.

By centralizing these operations, `TerrainHelper` ensures that terrain assumptions (scale, orientation, slope limits) remain consistent across different procedural components.

**Responsibilities**

- **Heightmap loading**
  - Loads grayscale PNG heightmaps from  
    `models/terrain/heightmaps/<heightmap_file>`
  - Converts pixel values into floating-point elevation data
  - Reports image dimensions and value range for verification

- **Slope computation**
  - Estimates local terrain slope using finite differences on the heightmap
  - Computes slope angle in degrees from heightmap gradients
  - Enforces a maximum allowable slope (`max_slope`) for placement validity

- **Coordinate conversion**
  - Maps heightmap pixel coordinates `(px, py)` to Gazebo world coordinates `(x, y, z)`. `(px, py)` are usually the same value as `(x, y)`.
  - Converts world-frame coordinates back to heightmap pixels
  - Maintains a consistent terrain reference frame shared by all generators

**Methods**

| Method | Description |
|-------|-------------|
| `load_heightmap()` | Loads the grayscale heightmap image from disk and caches it as a NumPy array for reuse. |
| `calculate_scope(px, py)` | Computes the local terrain slope angle (degrees) at the specified heightmap pixel using finite differences. |
| `pixel_to_world(px, py)` | Converts heightmap pixel coordinates to Gazebo world-frame coordinates `(x, y, z)` using terrain scale parameters. |
| `world_to_pixel(x, y)` | Converts Gazebo world-frame `(x, y)` coordinates back to heightmap pixel indices. |

**Design Notes**

- Terrain dimensions and scaling: `terrain_size_x`, `terrain_size_y`, `terrain_size_z` are taken from the model.sdf model for the terrain (saved under models/terrain/materials in the shared--install--folder, or under the same folder structure in the package). These dimensions are then compared with the .png file described inside the model.sdf. It is usually that the x and y dimensions are the same, while the z dimensions can be different. Please refer to **update_heightmap script** to know the z-dimension calculation.
- Tree slope checks and fire ground-height lookup rely on the same terrain conversion logic.
- Boundary regions of the heightmap are conservatively rejected to avoid invalid gradient estimates.

**Consumers**

`TerrainHelper` is inherited by:

- `TreeGenerator` — for slope-aware tree placement and pixel-to-world coordinate conversion
- `FireGenerator` — for fire/smoke placement, fire model ground alignment, and particle emitter pose generation

This design avoids duplicated terrain logic and ensures that all procedural elements are generated under identical terrain constraints.

**Assumptions**

- The heightmap is a single-channel (grayscale) PNG compatible with Gazebo heightmap terrain models.
- Heightmap resolution matches `terrain_size_x × terrain_size_y` described at the model.sdf (with the .png file described inside the model.sdf).
- The terrain model is centered at the world origin with symmetric extents.


### 3. TreeGenerator

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**
TreeGenerator is responsible for procedural tree placement on the terrain heightmap and generating the corresponding Gazebo `<include>` blocks that will be injected into the output .world file. It inherits TerrainHelper so that placement validation and pixel-to-world coordinate conversion are consistent with the shared terrain model assumptions.

**Inputs (ROS 2 Parameters)**

| Parameter | Type           | Description                                                                                                                                                                                                                    |
|----------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `num_trees` | `int`          | Number of tree instances to place.                                                                                                                                                                                             |
| `tree_types` | `list[string]` | List of Gazebo model names under models/ (e.g., oak_tree, pine_tree). A random type is selected per placement.                                                                                                                 |
| `min_tree_distance` | `float`        | Minimum spacing constraint between any two placed trees.                                                                                                                                                                       |
| `max_slope` | `float`        | Maximum allowable terrain slope in degrees. Candidate points with slope >= max_slope will be rejected.                                                                                                                         |
| `plant_tree_above_dirt` | `bool`         | Only allow to put trees in the second blend layer or above (1st Layer Dirt, 2nd Layer Grass, and 3rd Highest Layer Fungi. This corresponds to the `--blend1_min` + `--blend1_fade` height from the **update_heightmap script** |
**Execution Flow**
1) Load the heightmap as a grayscale image and convert it to float32 (taken this from the url available at the models/terrain/materials/model.sdf in the share folder)
2) Randomly sample candidate pixel coordinates (px, py)
3) Validate each candidate using boundary margin checks, local slope check, and minimum distance constraint to previously placed trees
4) Convert accepted pixel coordinates into world coordinates (x, y, z)
5) Generate one Gazebo `<include>` block per accepted placement
6) Return the full list of accepted trees and the concatenated XML snippet

**Key Methods**

| Method | Description |
|------|-------------|
| `generate_trees()` | Main placement loop. Randomly samples pixels and accepts valid tree positions until num_trees is reached or attempts exceed the limit. |
| `is_valid_tree_position(px, py, trees)` | Validity check for a candidate pixel: border constraints, slope check, and distance check against existing trees. |
| `create_tree_include_xml(tree_type, world_x, world_y, world_z, tree_id)` | Creates one `<include>` block for the selected tree type at a computed world pose, with randomized yaw. |
| `generate_trees_xml(trees)` | Converts all accepted tree placements into a single XML string to inject into the output world. |

**Output**
```text
worlds/<output_world_file>
```

What is written into the world:
- A list of Gazebo `<include>` blocks, one per generated tree instance
- Each instance includes a randomized yaw for visual diversity
- Tree placement is slope-aware and respects minimum spacing constraints

**Customization Guide (How to Define Your Own Tree Placement Logic)**
To customize the tree generation logic, the recommended approach is to modify or extend:

1) **Candidate sampling strategy**
- Edit: TreeGenerator.generate_trees()
- Example: restrict sampling to a region, or apply weighted sampling

2) **Validity constraints**
- Edit: TreeGenerator.is_valid_tree_position(px, py, trees)
- Example: add elevation limits, biome masks, or per-type slope limits

3) **Model selection / pose formatting**
- Edit: TreeGenerator.create_tree_include_xml(...)
- Example: weighted tree type sampling, fixed yaw, custom naming rules


### 4. FireGenerator (Fire Model + Smoke Particle Emitters)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**
FireGenerator is responsible for procedural fire placement on the terrain heightmap and generating the Gazebo XML used to represent each fire. It inherits `TerrainHelper` so that fire and smoke placement uses the same heightmap loading, pixel-to-world conversion, and terrain height lookup as tree placement.

Each generated fire area is represented by two parts:

- `fog_generator`: a Gazebo particle emitter that renders smoke/fog using `fog.png` as the particle texture.
- `fire_model`: one or more visual flame models placed randomly inside the smoke area.

The fire placement logic does not check slope. Fires can be generated on any terrain slope, but the `fire_model` pose is corrected using the terrain height at its own `(x, y)` position so the visible flame sits above the local ground instead of reusing the smoke center altitude.

**Inputs (ROS 2 Parameters)**

| Parameter | Type | Description |
|----------|------|-------------|
| `num_fires` | `int` | Number of fire/smoke areas to place. Default: `5`. |
| `min_fire_distance` | `float` | Minimum spacing constraint between generated fire/smoke placement centers. |
| `plant_fire_above_dirt` | `bool` | If `true`, fires are only placed in the second blend layer or above. This uses the same dirt threshold derived from `--blend1_min` + `--blend1_fade` in the **update_heightmap script**. Default: `False`. |
| `min_fire_size` | `float` | Minimum square smoke emitter size used for `<size>x x 0</size>`. Default: `2.0`. The supported lower limit is `1.0`. |
| `max_fire_size` | `float` | Maximum square smoke emitter size used for `<size>x x 0</size>`. Default: `10.0`. The supported upper limit is `50.0`. |

**Execution Flow**
1) Load the heightmap through `TerrainHelper`
2) Randomly sample candidate pixel coordinates `(px, py)`
3) Reject candidates outside the heightmap boundary margin
4) Reject candidates that violate `plant_fire_above_dirt`, if enabled
5) Reject candidates that violate `min_fire_distance`
6) Convert accepted fire centers to world coordinates `(x, y, z)`
7) Randomly choose a square smoke size between `min_fire_size` and `max_fire_size`
8) Randomly choose either `fogcolors.png` or `smokecolors.png` for the particle color range
9) Generate one smoke particle emitter named `fire_smoke_<id>`
10) Generate one or more `fire_model_<fire_id>_<model_id>` includes randomly inside the smoke area
11) Inject the Gazebo particle emitter plugin into the output world when fire XML is generated

**Smoke Particle Emitter**

Each smoke area is written as a Gazebo `<model>` with a `<particle_emitter name="emitter" type="box">`. The smoke area is square:

```xml
<size>{fire_size} {fire_size} 0</size>
```

The particle material uses:

```xml
<albedo_map>model://fog_generator/materials/textures/fog.png</albedo_map>
```

The color range image is selected randomly per fire from:

```text
models/fog_generator/materials/textures/fogcolors.png
models/fog_generator/materials/textures/smokecolors.png
```

The generated world uses the resolved installed model path for the color range image because Gazebo's particle emitter can fail to resolve `model://` paths for `<color_range_image>` even when the same path works for material textures.

**Size-Dependent Particle Parameters**

`min_fire_size` and `max_fire_size` define the random smoke size range. The selected `fire_size` is clamped to the supported range `1.0` to `50.0`, and the following particle parameters are interpolated linearly from small fires to large fires:

| Particle Parameter | Value at size `1.0` | Value at size `50.0` |
|--------------------|---------------------|----------------------|
| `lifetime` | `5.0` | `20.0` |
| `min_velocity` | `0.1` | `1.0` |
| `max_velocity` | `0.2` | `3.0` |
| `scale_rate` | `0.3` | `1.0` |
| `rate` | `10.0` | `100.0` |
| `particle_scatter_ratio` | `0.2` | `1.0` |

This makes larger fire areas emit longer-lived, faster, denser, and more widely scattered smoke while keeping small fires visually restrained.

**Fire Model Placement**

For each smoke area, FireGenerator also inserts one or more:

```xml
<include>
    <name>fire_model_<fire_id>_<model_id></name>
    <uri>model://fire_model</uri>
    <pose>x y z 0 0 yaw</pose>
</include>
```

The number of visible `fire_model` instances scales with smoke size:

- smoke size below or equal to `10`: one fire model
- smoke size from `10` to `20`: two fire models
- every additional `10` units adds one more fire model

Each `fire_model` is placed randomly inside the square smoke area. A `10` unit padding is used where the smoke area is large enough so that fire models are not placed at the smoke boundary. For smaller smoke areas that cannot fit a full `10` unit padding, the padding is reduced proportionally.

Each fire model's `z` value is computed from its own random `(x, y)` position by converting back to heightmap pixels and querying the local terrain height with `pixel_to_world()`. A small vertical offset is then applied so the visible fire sits above the ground.

**World Plugins**

When fires are generated, the node injects the required Gazebo systems into the output world if the particle emitter system is not already present:

```xml
<plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"></plugin>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

The generated XML also includes the standard physics, user command, and scene broadcaster systems used by the simulation world.

**Key Methods**

| Method | Description |
|------|-------------|
| `generate_fires()` | Main placement loop. Randomly samples pixels and accepts valid fire positions until `num_fires` is reached or attempts exceed the limit. |
| `is_valid_fire_position(px, py, fires)` | Validity check for a candidate fire center: border constraints, optional dirt-layer check, and distance check against existing fires. |
| `get_fire_size_range()` | Sorts and clamps `min_fire_size` / `max_fire_size` to the supported fire-size range. |
| `calculate_particle_parameters(fire_size)` | Computes lifetime, velocity, scale, rate, and scatter values by linear interpolation from size `1.0` to size `50.0`. |
| `create_fire_model_includes_xml(world_x, world_y, fire_id, fire_size)` | Creates the `fire_model` include blocks randomly inside the smoke area and corrects each model's height using local terrain data. |
| `create_fire_particle_emitter_xml(...)` | Creates the smoke particle emitter XML and combines it with the fire model includes for one generated fire area. |
| `generate_fires_xml(fires)` | Converts all accepted fire placements into XML to inject into the output world. |

**Output**

What is written into the world:

- One `fire_smoke_<id>` particle emitter model per accepted fire
- One or more `fire_model_<fire_id>_<model_id>` includes inside each smoke area
- Required Gazebo particle emitter plugin XML, if needed
- Randomized smoke color ranges using `fogcolors.png` or `smokecolors.png`
- Randomized fire model yaw for visual variation

**Assumptions**

- `models/fog_generator/` is installed with `fog.png`, `fogcolors.png`, and `smokecolors.png`.
- `models/fire_model/` is installed with `model.sdf`, its meshes, and required texture files.
- The terrain model and heightmap are available so FireGenerator can compute local ground height for each fire model.

### 5. update_heightmap script (Heightmap → Terrain SDF Update)

**Location**
```text
scripts/update_heightmap/main.py
```

**Role**
This script updates the terrain model SDF (`models/terrain/model.sdf`) to reference a new heightmap, and ensures the heightmap image is copied into the Gazebo-discoverable model path under `models/terrain/materials/textures/`.

It edits the `<heightmap>` block(s) in the SDF, updating:
- `<uri>`: points to the heightmap under `model://terrain/materials/textures/`
- `<size>`: sets `(width, height, height_range)`
- `<blend>`: ensures exactly two blend entries and updates their `min_height` and `fade_dist`
- `<pos>`: sets terrain heightmap pose offset

**CLI Arguments**
- `--heightmap`: input heightmap PNG path
- `--height_range`: vertical scale used in `<size>` (meters)
- `--blend1_min` / `--blend1_fade`: texture blend layer 1 parameters
- `--blend2_min` / `--blend2_fade`: texture blend layer 2 parameters
- `--pos_x` / `--pos_y` / `--pos_z`: heightmap pose offset written into `<pos>`
- `--terrain_sdf`: target terrain SDF path (default: `models/terrain/model.sdf`)
- `--target_dir`: copy destination for heightmap (default: `models/terrain/materials/textures/`)
- `--dry_run`: print computed values without modifying files

**Execution Flow**
1) Resolve package paths and defaults
2) Validate heightmap file exists and read image dimensions `(w, h)`
3) Construct SDF fields:
   - `uri = model://terrain/materials/textures/<heightmap_name>`
   - `size = "w h height_range"`
   - `pos = "pos_x pos_y pos_z"`
4) Parse the SDF and locate all `<heightmap>` blocks
5) For each `<heightmap>`:
   - set `<uri>`, `<size>`, `<pos>`
   - enforce exactly two `<blend>` entries and update blend parameters
6) If not `--dry_run`:
   - create a timestamped backup of the original SDF
   - copy the heightmap PNG into `target_dir`
   - write the updated SDF back to disk

**Outputs**
- Heightmap copied to:
```text
models/terrain/materials/textures/<heightmap_name>.png
```

- Terrain SDF updated:
```text
models/terrain/model.sdf
```

- Backup created:
```text
models/terrain/model.sdf.bak_<YYYYMMDD_HHMMSS>
```

**Notes**
- The script edits every `<heightmap>` block found in the SDF (supports multiple occurrences).
- The script prints the final computed `uri`, `size`, `pos`, and copy destination for verification before writing.
- Make sure to recompile this package with ROS to ensure the heightmap (and/or other resources in terrain folder) is copied to the share directory (i.e. install/forest_map_generator/share/forest_map_generator/models/terrain)

### 6. ply_to_gazebo_textured pipeline (PLY → Gazebo Tree Model)

**Location**
```text
scripts/ply_to_gazebo_textured/
├── main.py
└── bake_vcol_to_texture.py
```

**Role**
This pipeline converts colored point clouds (`.ply`) into Gazebo-ready tree models under `models/<tree_name>/`. It produces a textured visual mesh (`.dae` + albedo `.png`) and a simplified collision mesh (`.stl`).

`main.py` handles point cloud processing, mesh reconstruction, simplification, and collision generation. Texture baking and Collada export are delegated to Blender via `bake_vcol_to_texture.py`.

**Inputs**
- Point clouds placed under:
```text
scripts/ply_to_gazebo_textured/trees_ply/*.ply
```

- Requires `open3d` and a working Blender CLI (`blender` in `PATH` or configured via `BLENDER_BIN`).

**Execution Flow**
1) Load `.ply` point cloud using `open3d`
2) Normalize geometry:
   - center `XY` at the cloud centroid
   - shift `Z` so the base is at `Z=0`
3) Downsample to `TARGET_POINTS` if needed
4) Estimate normals and run Poisson surface reconstruction
5) Remove low-density vertices and clean mesh topology
6) Transfer vertex colors from point cloud to mesh (kNN weighted average), or paint a uniform fallback color
7) Generate visual mesh:
   - simplify to `VISUAL_MAX_TRIANGLES`
   - export an intermediate colored visual `.ply`
8) Run Blender texture baking:
   - bake vertex colors into a texture image (`*_albedo.png`)
   - generate UVs if missing
   - export textured Collada (`tree_mesh.dae`)
9) Generate collision mesh:
   - simplify to `COLLISION_MAX_TRIANGLES`
   - export collision geometry as `tree_collision.stl`

**Outputs**
For each input `<tree_name>.ply`, the pipeline creates:
```text
models/<tree_name>/
└── meshes/
    ├── tree_mesh.dae
    ├── tree_collision.stl
    ├── <tree_name>_albedo.png
    └── <tree_name>_visual_colored.ply
```

**Key Scripts**
- `main.py`
  - reconstructs meshes from point clouds (`Poisson`)
  - applies decimation for visual and collision meshes
  - calls Blender for UV + texture baking
  - writes outputs into `models/<tree_name>/meshes/`

- `bake_vcol_to_texture.py`
  - imports the intermediate colored mesh (`.ply`)
  - creates UVs if missing (`uv.smart_project`)
  - builds a node graph that reads vertex color and bakes it to an image via `EMIT`
  - saves the baked texture (`.png`) and exports Collada (`.dae`)

**Notes**
- The visual mesh is intended for rendering (`tree_mesh.dae` + `*_albedo.png`).
- The collision mesh is intentionally simplified for simulation performance (`tree_collision.stl`).
- Blender is invoked in headless mode (`-b`) and called from `main.py` via `subprocess`.


---
