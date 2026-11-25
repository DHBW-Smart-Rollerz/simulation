# Simulation performance measurement

This guide separates slow startup from slow simulation and defines a repeatable
baseline before attempting optimizations. A single run is not enough to identify
a bottleneck: keep the launch options, road, hardware, and other running
applications constant, then change one factor at a time.

## What to measure

| Metric | Definition | Why it matters |
|---|---|---|
| Startup to ROS-ready | Wall time from launching to the required ROS nodes being present and `/clock` advancing | Measures process, middleware, and world startup |
| Startup to scene-ready | Wall time until the road, signs, and obstacles are present in Gazebo | Includes road rendering and model insertion |
| Real-time factor (RTF) | Change in simulation time divided by elapsed wall time; Gazebo also publishes `real_time_factor` in its world statistics | `1.0` means real time; lower values mean the simulation is falling behind |
| Sensor rate | Observed rate of `/camera/image/raw` and `/sensor/imu` | Shows whether simulated sensor output keeps up with its configured rate |
| CPU and memory | Per-process CPU and resident memory for Gazebo server, Gazebo GUI, `parameter_bridge`, RViz, and `world_spawner` | Identifies which process consumes resources |
| Startup phase duration | Time spent loading/rendering the road and inserting each entity | Separates file/rendering cost from Gazebo service/model-insertion cost |

The world requests a 1 ms physics step and an RTF target of `1.0`
([`models/world.sdf`](models/world.sdf)). The vehicle model configures a
2064 × 1544 camera at 30 Hz and an IMU at 200 Hz
([`models/smarty/model.sdf`](models/smarty/model.sdf)). Treat those as
configured targets, not proof that the rates are achieved. The package README
currently describes 30% real time as a minimum usable level; the performance
goal should still be to approach RTF `1.0`.

## Repeatable baseline procedure

1. Record the date, CPU/GPU model, memory, OS, ROS 2 and Gazebo versions,
   display resolution, selected road, launch command, and whether the rest of
   the Smarty stack is running. Close unrelated CPU/GPU-heavy applications.
2. Build and source the workspace, then use the normal simulation launch:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch simulation simulation.launch.py
   ```

3. Record a monotonic start immediately before launch and use the same
   monotonic clock for readiness events (for example, with a small launch-event
   recorder). Define ROS ready as the expected simulation nodes being present
   and `/clock` advancing.
   Define scene ready as ROS ready plus the road, all expected signs and
   obstacles appearing in Gazebo. Record both times; do not use the first
   process-start message as the full startup time.
4. After the scene is ready, allow at least 30 seconds for warm-up. Measure RTF
   over a 60-second steady-state window from
   `/world/smartrollerz/stats`:

   ```bash
   gz topic -e -t /world/smartrollerz/stats
   ```

   Record the mean, median, minimum, and 95th percentile of
   `real_time_factor`. Also compute the window RTF as
   `(end sim_time - start sim_time) / elapsed wall seconds`. The window value
   is a useful check against brief spikes in individual statistics messages.
5. In separate repeat runs, sample the sensor rates and per-process resource
   use. The probes add some overhead, so do not run them during the primary RTF
   window:

   ```bash
   ros2 topic hz /camera/image/raw
   ros2 topic hz /sensor/imu
   pidstat -u -r -p ALL 1
   ```

   Record the observed rate and CPU/memory by process. If possible, use a
   60-second resource sample and retain the process IDs and sample times.
6. Repeat at least five times from a fresh launch. Report the median and range
   for startup times and RTF; keep the first run separate if it includes cold
   cache or one-time asset loading. Save launch logs and raw measurements with
   the summary.
7. Compare controlled variants, changing one factor per set of runs:
   simulation only with defaults; `rviz:=false`; and simulation plus the
   same selected downstream software stack. The current launch has no
   Gazebo-GUI-off or sensor-disable argument, so do not claim those components
   have been isolated unless a separate, documented test configuration is
   used.

For each change, compare the median against the same-condition baseline:

```text
change (%) = 100 * (variant median - baseline median) / baseline median
```

For RTF, a positive change is better; for startup duration and CPU use, a
negative change is better. Keep the hardware and all unmodified settings the
same, and verify that the scene and required sensor topics still work before
accepting a performance improvement.

## Likely areas to investigate

These are testable hypotheses, not established causes:

- `world_spawner` renders the road texture at startup and then inserts the road,
  signs, and obstacles serially. Each insertion starts and runs a separate
  `ros_gz_sim` launch service
  ([`simulation/world_spawner.py`](simulation/world_spawner.py)). Measure
  road load/render time separately from each insertion request and completion.
- The camera is high resolution and runs at 30 Hz; image transport through
  Gazebo, `ros_gz_bridge`, and any downstream perception nodes may be a
  substantial CPU and memory-bandwidth cost. Measure the simulation-only
  configuration before adding the rest of the software stack.
- Gazebo's GUI and RViz both render the scene. The launch defaults to opening
  RViz ([`launch/simulation.launch.py`](launch/simulation.launch.py)); comparing
  `rviz:=false` helps isolate RViz, but does not disable Gazebo's GUI.
- The 1 ms physics step implies up to 1000 simulation iterations per simulated
  second at RTF `1.0`. Measure the server process and RTF before changing
  physics settings, because changing the step can affect simulation fidelity.

Monotonic timing logs cover road loading/rendering and entity-spawn requests.
The `entity_spawn_mode` launch argument selects the new `service` path
(default) or the original `legacy` path, which starts one launch process per
entity. The service path sends requests through one persistent client in
batches, with bounded parallelism controlled by `spawn_concurrency` (default
4). Each request is checked for a response and a successful spawn.

Compare spawn strategies with the same road and render-cache state:

```bash
ros2 launch simulation simulation.launch.py rviz:=false entity_spawn_mode:=legacy
ros2 launch simulation simulation.launch.py rviz:=false entity_spawn_mode:=service
```

The service mode changes request submission from one process and wait per
entity to at most four concurrent requests. Gazebo may still serialize internal
insertion work, so the improvement must be measured rather than assumed. The
timing logged for each service-spawned entity is elapsed time from its batch
submission until its response, not an isolated Gazebo processing duration.

Road texture rendering can be compared using the `road_render_mode` launch
argument. `cached` (default) reuses the rendered PNG when its recorded source
fingerprint and image size still match; `legacy` always renders as before.
Compare warm launches with the same road using:

```bash
ros2 launch simulation simulation.launch.py rviz:=false road_render_mode:=legacy
ros2 launch simulation simulation.launch.py rviz:=false road_render_mode:=cached
```

The first cached launch creates the cache and still performs a full render;
subsequent unchanged launches are the warm-cache measurement. Changing
simulation or road Python sources invalidates the cache. Unseeded roads are
always rendered to preserve their per-launch random layout.

The launch accepts `bridge_config_file`, defaulting to the existing normal
bridge configuration. The measurement-only
[`config/ros_gz_bridge_no_camera.yaml`](config/ros_gz_bridge_no_camera.yaml)
omits only the camera image and camera-info ROS bridges; Gazebo's camera
sensor remains enabled. Physics settings are unchanged.

## Initial observation (2026-11-04)

This initial exploratory observation is superseded by the controlled
instrumented measurements below. It is retained only to explain why the
original incomplete RTF sample is not used as a baseline:

Run setup: Linux host, ROS 2 Jazzy, Gazebo Sim 8, 12 logical CPUs and about
30 GiB RAM; simulation launch with the default Gazebo GUI and RViz, without the
downstream Smarty software stack. GPU/display details and unrelated host load
were not controlled.

| Measurement | Observation | Limitation |
|---|---|---|
| Launch to first road spawn request | About 6.5 s | The launch log timestamp is used as start; the message is emitted before the request completes |
| Launch to last entity spawn request | About 12.0 s | This is not a verified scene-ready time; there is no spawn-completion log |
| Warm RTF sample | 35-second sample, first 5 seconds discarded: mean `0.678`, minimum `0.143`, maximum `1.374` | Only one sample; values varied widely and the scene was not verified complete |
| Gazebo model list | Listed `ground_plane` and `smarty` only | The expected road/sign/obstacle models were not present in this query, despite spawn-request log messages |
| Sensor rate | Initial camera probe reported roughly 4.8 Hz while the world was starting | Too early and too short to represent steady state |

The launch log showed `world_spawner` initialized at about 1.3 seconds,
requested the road at about 6.5 seconds, and requested its last obstacle at
about 12.0 seconds. The incomplete scene query invalidated this first run;
subsequent instrumented runs verified the expected road entities after
spawning. Its RTF and startup observations are not used in the current results.

The first attempt to launch from the VS Code Snap environment failed because
Gazebo and RViz loaded an incompatible Snap `libpthread`; retrying with a clean
inherited environment allowed both GUIs to start. During shutdown, Gazebo also
reported an OGRE renderer segmentation fault. Record and investigate these
runtime issues separately from steady-state performance.

## Current instrumented measurements (2026-11-04)

Run setup: Linux, ROS 2 Jazzy, Gazebo Sim 8, 12 logical CPUs and about 30 GiB
RAM. The default road was used without the downstream Smarty stack. RViz was
disabled in all but the first run; Gazebo's GUI remained enabled. Host load,
GUI rendering, and cold-cache effects were not controlled, so ranges are more
informative than a single result.

### Startup phase timings

| Run | RViz | Camera bridge | Road load | Road render | Sum of 11 spawn calls | Full spawn phase |
|---|---:|---:|---:|---:|---:|---:|
| Initial default | on | on | 0.092 s | 4.873 s | 9.438 s | 14.349 s |
| Initial control | off | on | 0.117 s | 8.904 s | 7.711 s | 16.660 s |
| Initial comparison | off | off | 0.163 s | 8.497 s | 6.221 s | 14.761 s |
| Repeat control | off | on | 0.161 s | 10.832 s | 6.480 s | 17.364 s |
| Repeat comparison | off | off | 0.136 s | 7.160 s | 7.487 s | 14.680 s |

The full spawn phase includes texture rendering and sequential model
insertions. Road loading took only 0.09–0.16 s. Rendering took 4.9–10.8 s,
and 11 separate spawn calls took 6.2–9.4 s combined. The slowest single sign
insertion was about 5.0 s in the initial run; most calls in the other runs
took roughly 0.4–1.3 s. Expected road entities were present after spawning.
The legacy measurements establish the pre-change baseline for serial entity
insertion. The follow-up service-mode comparison is below.

### Road-render cache comparison

The original path remains available as `road_render_mode:=legacy`; the default
is now `cached`. In full `rviz:=false` launch runs on the same host, the
always-render run took 9.838 s in the road-render phase and 19.124 s in the
full entity-spawn phase. Three warm-cache runs logged render phases of 0.008 s,
0.010 s, and 0.006 s, with full phases of 6.123 s, 6.565 s, and 6.391 s.
The cache-hit log was true in all optimized runs, and all 11 spawn calls
completed. The warm-run median render phase was 0.008 s; the median full
entity-spawn phase was 6.391 s.

The directly attributable improvement is the road-render phase: about
9.83 seconds saved in the 9.838-second legacy measurement, or approximately
99.9%. Full-phase times also decreased in these runs, but entity-spawn call
durations varied between runs; do not attribute that entire difference to
render caching. A separate direct check after the final cache change measured
3.756 s to rebuild the cache and 0.005 s for a warm lookup; an earlier direct
comparison measured 4.185 s for the legacy path. The first cached render still
takes the full render cost to create the cache.

These are phase timings from inside `world_spawner`, not a fully instrumented
`ros2 launch`-to-ROS-ready wall-clock time. Measure process startup and
readiness separately using the procedure above before claiming an
end-to-end startup duration.

### Entity-spawn comparison

The service mode waits for the road spawn to succeed before submitting signs
and obstacles, then submits those independent entities in batches of up to
four. A same-session warm-cache comparison with RViz disabled completed all
11 entity spawns in both modes:

| Mode | Render | Entity spawning | Full spawn phase |
|---|---:|---:|---:|
| Legacy per-entity launch | 0.009 s | 6.189 s | 6.198 s |
| Persistent service, concurrency 4 | 0.006 s | 0.128 s | 0.134 s |

This single-run comparison suggests roughly 97.9% lower entity-spawn time
(about 48 times faster). The service measurement includes response time and
the runs were not repeated as a statistical benchmark; repeat both modes
several times before treating the percentage as stable. The legacy
implementation remains selectable with `entity_spawn_mode:=legacy`.

### Steady-state runtime comparison

All samples were taken with RViz disabled after scene creation. Gazebo's
camera sensor remained enabled in the bridge-off profile, which omitted only
the camera image and camera-info ROS bridges. RTF was captured for about 40
seconds; CPU was sampled separately with `pidstat`. CPU percentages are
per-core style, so values above 100% represent multiple logical CPUs.

| Profile | Run | RTF samples | Mean | Median | P95 | `parameter_bridge` CPU |
|---|---:|---:|---:|---:|---:|---:|
| Camera bridges on | 1 | 363 | 0.778 | 0.818 | 1.155 | 111.2% |
| Camera bridges on | 2 | 383 | 0.852 | 0.998 | 1.102 | 115.72% |
| Camera bridges off | 1 | 365 | 0.968 | 1.000 | 1.208 | 39.32% |
| Camera bridges off | 2 | 415 | 0.938 | 0.999 | 1.116 | 33.38% |

Removing the image bridges reduced `parameter_bridge` CPU by about 72–82
percentage points in these paired observations, while leaving the simulated
camera enabled. Mean RTF was higher in both bridge-off runs, but the per-run
median was almost unchanged in the repeat pair and varies between runs. This
is strong evidence that image bridging is a major bridge CPU cost and a
credible contributor to lost average RTF; it does not yet prove that it is the
largest overall RTF bottleneck. Gazebo also remained CPU-intensive without
the image bridge (one repeat measured about 113% server CPU and 133% GUI CPU),
so physics/rendering may independently limit real-time performance.

The camera is configured at 2064 × 1544 and 30 Hz. A raw image message was
about 9.56 MB; a short bandwidth probe reported roughly 24–44 MB/s and should
be treated as approximate. Short ROS rate probes observed about 16–25 Hz,
below the configured target. Probe subscriber overhead and host load mean
these are not yet a final sensor-throughput baseline.

### Fix proposal (not implemented)

Prioritize controlled experiments before changing functionality:

1. **Runtime, image transport:** Test lazy/on-demand image bridging, or make
   the image bridge independently optional, to avoid image conversion and ROS
   transport when no ROS consumer needs it. Preserve current camera topic
   behavior when perception is running; compare RTF, bridge CPU, message rate,
   and downstream perception output.
2. **Runtime, sensor cost:** Separately test camera resolution/frame-rate
   reductions or compressed transport only if perception fidelity allows it.
   The bridge-off test did not remove camera rendering, so it does not quantify
   the camera sensor's Gazebo-side cost.
3. **Startup, road rendering:** Profile the road renderer and test reusing its
   generated texture when road inputs are unchanged. Cache invalidation must
   follow road assets/configuration so changed roads still regenerate.
4. **Startup, entity insertion:** Split launch setup time from Gazebo insertion
   time, then test a persistent spawn client or batch insertion instead of
   starting a `LaunchService` for each entity. Verify every entity and the
   scene-ready condition after any change.
5. **Remaining runtime load:** Compare Gazebo GUI on/off and measure server
   physics/rendering load separately. Do not change the 1 ms physics step
   until its effect on simulation fidelity and sensor behavior is tested.

**Conclusion:** Road rendering and serial insertion are the largest measured
startup phases. Image bridging is the clearest isolated runtime CPU cost, but
at least five fresh runs per profile under controlled host load are still
needed to determine its stable RTF effect and whether it is the biggest
overall bottleneck. The next measurement should also add split spawn timing
and a monotonic launch-to-ready wall time.
