# CLAUDE.md

This file gives Claude Code guidance for working in this repository.

## What this is

`simulation` is a ROS 2 (Jazzy) `ament_python` package from the DHBW Smart Rollerz team. It simulates their 1:10-scale autonomous model car "Smarty" in Gazebo (gz-sim, Harmonic) for the CAUDRI challenge (https://caudri-challenge.de/, formerly Carolo-Cup). It is meant to be a **drop-in replacement for the real car**: the rest of the Smarty stack (camera_preprocessing, lane_detection_ai, pathplanning, querregelung, ...) runs against it unchanged, using the same ROS topics.

The package lives in `src/` of a "smarty workspace" (see `DHBW-Smart-Rollerz/smarty_workspace`). It needs `smarty_utils` (`dependencies.repos`, branch `jazzy`) for `setup_utils.include_directory` and `timing`. The active branch line is `jazzy*`, which CI runs on.

## Commands

Run these from the workspace root. The `make` targets come from the smarty workspace, not from this repo.

```bash
sudo apt install libcairo2-dev && make install_python   # pip deps from requirements.txt
make build PKG=simulation                               # colcon build of this package only
make clean && make build                                # if the build breaks

ros2 launch simulation simulation.launch.py             # Gazebo + bridge + nodes + RViz
ros2 launch simulation simulation.launch.py keyboard_control:=true rviz:=false
ros2 launch simulation simulation.launch.py bridge_config_file:=<share>/config/ros_gz_bridge_no_camera.yaml  # faster, no camera
```

Launch arguments are `rviz`, `keyboard_control`, `params_file`, `bridge_config_file`, `road_render_mode` (`cached`|`legacy`), `entity_spawn_mode` (`service`|`legacy`) and `spawn_concurrency` (default 4).

Tests:
```bash
colcon test --packages-select simulation && colcon test-result --verbose
pytest test/test_road_render_cache.py                   # pure Python, no ROS needed
pytest test/test_world_spawner_spawn.py                 # needs a sourced ROS env (imports rclpy msgs)
python -m simulation.groundtruth.road.sections.test     # unittest suite for road geometry
```

Lint and format: `pre-commit run --all-files`, which runs ruff-check --fix, ruff-format, the standard hooks and ament_uncrustify. `.pre-commit-config.yaml` refers to a `ruff.toml` that is **not in the repo**, so ruff uses its defaults. `test/` also contains the ament `flake8`, `pep257` and `copyright` tests, which run under `colcon test`.

Windows note: this repo is checked out on Windows here, but building and running it needs Linux with ROS 2 Jazzy and Gazebo, for example native Ubuntu 24.04 or WSL2. Pure-Python parts (the `groundtruth` geometry and road code, `render_cache`) can be edited and unit-tested without ROS if `shapely`, `numpy`, `scipy`, `pycairo` and `pyquaternion` are installed.

## Architecture

### Runtime data flow

```
Smarty stack ──/control/steering_angle/target (Int16)──┐
             ──/control/velocity/target (Float32)──────┤
                                                       ▼
                                          car_control_bridge (200 Hz)
                                                       │ /smarty/cmd_vel (Twist)
                                                       ▼
                    ros_gz_bridge (config/ros_gz_bridge.yaml) ⇄ Gazebo world "smartrollerz"
                                                       │
   /camera/image/raw, /camera/camera/info, /sensor/imu, /tf, /tf_static, /joint_states, /clock
   /smarty/odometry/ackermann ──► car_sensor_bridge ──► /sensor/velocity (Float32)
   /gazebo/keypress (Int32)   ──► keyboard_control  ──► the /control/... targets
```

Nodes are in `simulation/` and registered in `setup.py` `entry_points`. Parameters are in `config/ros_params.yaml`.

- `car_control_bridge.py` latches the last steering and velocity targets and republishes them as a `Twist` at a fixed rate: `linear.x = velocity` and `angular.z = steering_angle`.
- `car_sensor_bridge.py` takes `twist.linear.x` from the Ackermann odometry and publishes it as `/sensor/velocity`.
- `robot_description_publisher.py` reads `models/smarty/model.sdf`, rewrites `package://` to `package://simulation/` so RViz can resolve meshes, and publishes the result once, transient-local, on `/robot_description`.
- `keyboard_control.py` turns Gazebo GUI key codes (W/A/S/D and the arrow keys) into control targets. It conflicts with the autonomous controllers if both run.
- `world_spawner.py` builds the track at startup (see below). The launch file runs it with the interpreter from `$PYTHON_EXECUTABLE`, default `/usr/bin/python3`.

### Gazebo models

- `models/world.sdf` defines the world `smartrollerz`, with a 1 ms physics step, two directional lights and a ground plane. It includes `models/smarty` and adds the PosePublisher, OdometryPublisher and **AckermannSteering** plugins (wheel_base 0.26, wheel_separation 0.17, wheel_radius 0.035).
- `models/smarty/model.sdf` holds the chassis, the steering and wheel joints (steering limit ±0.6 rad), a 200 Hz IMU, and a 2064×1544 wide-angle camera at 30 Hz on a pole.
- `models/meshes/` holds the traffic sign meshes (`*_sign.dae`, with `.blend` sources and `.png` textures) and the car meshes (`smarty.stl`, `mr_fast.stl`). Sign mesh names must match `SignTuple(mesh=...)` in `traffic_sign.py`.
- The launch file sets `GZ_SIM_RESOURCE_PATH` to the package share dir, so `package://models/...` URIs in SDF resolve relative to it.

### Road generation (`simulation/groundtruth/`)

This is a Python DSL for describing a CAUDRI-style track. It appears to be ported from the KITcar gazebo simulation; some docstrings still use `simulation.utils.*` import paths.

- `geometry/` contains `Point`, `Vector`, `Line`, `Polygon`, `Pose` and `Transform`. These are wrappers around shapely and numpy.
- `road/road.py` defines `Road`, which is a list of `RoadSection`s. `Road.append()` chains each section to the end pose of the previous one. `load(path, seed="smartrollerz")` seeds `random` and then **executes** the road file with importlib. The file must define a module-level `road`. `render_to_file()` uses cairo to draw every line and surface marking into one PNG at 512 px/m on a black background.
- `road/sections/` contains the section types: `StraightRoad`, `Left/RightCircularArc`, `Quad/CubicBezier`, `Intersection`, `ParkingArea`/`ParkingLot`/`ParkingSpot`, `ZebraCrossing`, `BlockedArea`, `TrafficIsland` and `CustomSection`. It also contains the elements a section can carry: `StaticObstacle`, `ParkingObstacle`, `TrafficSign`, `SurfaceMarking*` and `SpeedLimit` (added with `add_speed_limit`; a negative speed ends the zone). Every subclass must declare `TYPE`. The import order in `sections/__init__.py` matters because changing it causes circular imports.
- `road/config.py` sets lane width (`road_width = 0.4` m, middle line to edge line) and the sign and marking distance intervals taken from the competition rules.
- `road/renderer/` produces the PNG drawing (`surface_markings.py`, `utils.py`) and the SDF strings for signs (`traffic_sign.py`) and obstacles (`obstacle.py`).
- `road/render_cache.py` skips re-rendering when nothing changed. Its fingerprint is a SHA-256 over every `*.py` file in the `simulation` package and in the road directory, plus the seed. The fingerprint is stored in `.<name>.render-cache.json` next to the PNG. Roads with `use_seed=False` are never cached.

When `world_spawner` starts, it loads the road, renders or reuses `<road>.png` **inside the installed share dir** next to `road.py`, and spawns a flat plane textured with that PNG. It then spawns signs (named `<mesh>_<idx>`) and obstacles (named `obstacle_<idx>`). In `service` mode the requests are queued and sent in batches of `spawn_concurrency` to `/world/smartrollerz/create`, which the launch file exposes through the `parameter_bridge`. In `legacy` mode it starts one `gz_spawn_model` launch per entity, which is slow. `PERF ...` log lines record timing; the current branch is working on startup time.

### Adding or changing a track

1. Create `models/roads/<name>/road.py` that builds `road = Road()` and appends sections. Use `models/roads/default_road/road.py` as the template.
2. Set `world_spawner.ros__parameters.road: "models/roads/<name>"` in `config/ros_params.yaml`, or pass a different `params_file`.
3. Rebuild, because `models/` is installed into share. With `--symlink-install`, files written at runtime land in the source tree.

## Gotchas

- **Steering units:** the stack sends the steering angle as an `Int16`, which `keyboard_control` treats as degrees. `car_control_bridge` puts it unchanged into `Twist.angular.z`, and the gz AckermannSteering plugin reads that field as a yaw rate in rad/s. Simulated steering therefore does not match the real car's scale. The git log also says the Ackermann behaviour is only "partially corrected". Keep this in mind for controller tuning.
- `world_spawner.spawn_road()` builds the texture URI with `dir.split("/simulation/")[2]`. This relies on the install path looking like `.../install/simulation/share/simulation/models/...` and breaks with other layouts or on Windows paths.
- The world name `smartrollerz` is hard-coded in `world_spawner.py` (marked TODO), in `world.sdf`, in the launch file's service bridge argument and in the bridge YAML (`/world/smartrollerz/...`). Change all of them together.
- Two bridge configs exist, `ros_gz_bridge.yaml` and `ros_gz_bridge_no_camera.yaml`, and they differ only in the camera topics. Keep the two files in sync.
- Default topic names in the nodes' `declare_parameters` sometimes differ from `ros_params.yaml`; for example, the steering default is `/control/steering/target`. The YAML is what actually runs.
- The simulation can run well below real time. The real-time factor is shown in the bottom right of the Gazebo GUI, as black text on the black road. About 30% is the usable minimum. Everything uses `/clock`, so set `use_sim_time` on nodes that need it.

## Conventions

- Each node is an `rclpy.node.Node` subclass with `load_ros_params()` (declare and read all parameters), `init_publisher_and_subscriber()`, and a `main()` that spins and shuts down cleanly. Follow this pattern for new nodes and register them in `setup.py` `console_scripts`. Add them to `launch/simulation.launch.py` with `parameters=[params_file]`.
- Use Google-style docstrings (`Args:` / `Returns:`) and dataclasses throughout `groundtruth`.
- Add new Python dependencies to both `requirements.txt` and, where it applies, `package.xml`.
- The license is MIT. Mesh licenses are in `models/meshes/LICENSE`.
