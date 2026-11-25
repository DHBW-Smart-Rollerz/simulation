# Simulation performance

Use the [laptop baseline report](BASELINE_LAPTOP.md) for the pre-branch
revision, hardware and power settings, measured results, raw evidence, and
instructions for repeating the benchmark. This guide describes the existing
optimizations, earlier exploratory measurements, and further experiments.

## Comparing changes

Keep the hardware, power profile, software versions, road, display, launch
options, background workload, probes, and camera settings the same. Separate
cold-start and warm-cache measurements. Run the same workload at least five
times per variant, alternate baseline and candidate runs where practical, and
report each result plus the median and range.

Measure monotonic wall time to both ROS readiness and verified scene
completion, then real-time factor (RTF) over a steady-state window. In separate
runs, measure per-process CPU and memory, camera and IMU receive rates, frame
drops, and downstream perception performance. Probes and subscribers can
affect the workload. Check scene entities, camera output, and relevant
perception results before accepting a speedup.

The vehicle configures a 2064 × 1544 camera at 30 Hz and an IMU at 200 Hz;
the world requests a 1 ms physics step and RTF 1.0. These are simulation-time
targets, not proof of delivered wall-clock rates. Preserve camera resolution
and frame rate by default. Investigate reductions separately only if
calibration, detection quality, and latency are also evaluated.

Use the same-condition median for comparisons:

```text
change (%) = 100 * (candidate median - baseline median) / baseline median
```

Higher RTF is better; lower startup time and resource usage are generally
better. Agree on acceptance thresholds before testing. The pre-branch commit
is the end-to-end baseline; the legacy modes below are useful diagnostics but
do not replace measuring that commit.

## Startup paths and diagnostic options

The current `world_spawner` defaults to a source-aware road texture cache and
a persistent Gazebo spawn-service client. The first cache miss renders the road;
valid warm hits reuse the image. Source changes invalidate the cache, and
unseeded roads are always rendered. The service path waits for the road to
spawn, then submits signs and obstacles in bounded batches; each response is
checked. The default concurrency is four, and Gazebo may still serialize
insertion internally.

The original paths remain selectable for phase-level comparisons:

```bash
# Original rendering and serial, process-per-entity spawning
ros2 launch simulation simulation.launch.py rviz:=false \
  road_render_mode:=legacy entity_spawn_mode:=legacy

# Compare spawning with the same warm road cache
ros2 launch simulation simulation.launch.py rviz:=false \
  entity_spawn_mode:=legacy
ros2 launch simulation simulation.launch.py rviz:=false \
  entity_spawn_mode:=service spawn_concurrency:=4

# Compare rendering with the same spawn mode
ros2 launch simulation simulation.launch.py rviz:=false \
  road_render_mode:=legacy
ros2 launch simulation simulation.launch.py rviz:=false \
  road_render_mode:=cached
```

Record cache state and the `PERF` phase logs. A service-spawn entity timing
includes elapsed time from batch submission to its response; it is not an
isolated Gazebo processing duration. Test other concurrency values only with
the same scene and correctness checks.

For diagnosing image transport, `bridge_config_file` can select
[`ros_gz_bridge_no_camera.yaml`](config/ros_gz_bridge_no_camera.yaml), which
omits the camera image and camera-info ROS bridges but leaves the Gazebo
sensor enabled. This profile is **not functionally equivalent** to running
perception and does not measure the cost of camera rendering. `rviz:=false`
disables RViz, not the Gazebo GUI.

## Exploratory measurements: 4 November 2026

These are historical phase and diagnostic observations, not the controlled
five-run pre-branch laptop baseline. The runs used Linux, ROS 2 Jazzy, Gazebo
Sim 8, 12 logical CPUs, about 30 GiB RAM, and the default road without the
downstream stack. Gazebo GUI was enabled; RViz was disabled except in the
first startup row. GPU/display details, host load, and cold-cache effects were
not controlled. Do not treat these measurements as an end-to-end before/after
comparison or claim stable percentage savings from single runs.

### Startup phases

| Run | RViz | Camera bridge | Road load | Road render | Sum of 11 spawn calls | Full spawn phase |
|---|---:|---:|---:|---:|---:|---:|
| Initial default | on | on | 0.092 s | 4.873 s | 9.438 s | 14.349 s |
| Initial control | off | on | 0.117 s | 8.904 s | 7.711 s | 16.660 s |
| Initial comparison | off | off | 0.163 s | 8.497 s | 6.221 s | 14.761 s |
| Repeat control | off | on | 0.161 s | 10.832 s | 6.480 s | 17.364 s |
| Repeat comparison | off | off | 0.136 s | 7.160 s | 7.487 s | 14.680 s |

These sequential-spawn phase timings include road rendering but are not
launch-to-ready times. Expected entities were verified after spawning.

In another `rviz:=false` comparison, always-render took 9.838 s; three warm
cache hits took 0.008, 0.010, and 0.006 s. Their full spawn phases were 6.123,
6.565, and 6.391 s; the separate legacy full phase was 19.124 s. Spawn timing
varied, so the full-phase difference cannot all be attributed to caching.
A direct check measured 3.756 s to rebuild a cache and 0.005 s for a warm
lookup, compared with an earlier 4.185 s legacy render.

One same-session warm-cache comparison completed all 11 expected spawns:

| Spawn mode | Render | Entity spawning | Full spawn phase |
|---|---:|---:|---:|
| Legacy per-entity launch | 0.009 s | 6.189 s | 6.198 s |
| Persistent service, concurrency 4 | 0.006 s | 0.128 s | 0.134 s |

The service result includes response times. Repeat both modes before drawing
conclusions about typical savings. The 19.124 s legacy and 0.134 s optimized
full-phase observations came from separate, uncontrolled runs.

An earlier pilot was rejected because the scene query found only the ground
plane and vehicle. Its 35-second RTF sample had mean 0.678, minimum 0.143,
and maximum 1.374; its approximate 4.8 Hz camera probe was taken during
startup. Neither is used as a baseline.

### Image-bridge diagnostic

These 40-second RTF samples were taken with RViz off after scene creation;
CPU was sampled separately with `pidstat`. The bridge-off profile omitted
camera image and camera-info ROS bridges, not the Gazebo camera sensor.

| Profile | Run | RTF samples | Mean | Median | P95 | Bridge CPU |
|---|---:|---:|---:|---:|---:|---:|
| Camera bridges on | 1 | 363 | 0.778 | 0.818 | 1.155 | 111.2% |
| Camera bridges on | 2 | 383 | 0.852 | 0.998 | 1.102 | 115.72% |
| Camera bridges off | 1 | 365 | 0.968 | 1.000 | 1.208 | 39.32% |
| Camera bridges off | 2 | 415 | 0.938 | 0.999 | 1.116 | 33.38% |

CPU percentages are per logical core. Image bridging is a substantial bridge
CPU cost in these observations, but the samples do not establish it as the
largest overall RTF bottleneck. A raw image was about 9.56 MB; short probes
reported approximately 24–44 MB/s and 16–25 Hz. Subscriber overhead and host
load prevent treating these as a stable throughput baseline.

## Next experiments

1. Compare the cache and spawn client against the pre-branch revision with
   both cold and warm starts. Verify every scene entity and road texture.
2. Investigate image conversion and transport at the current resolution and
   rate, including lazy/on-demand bridging or compression where supported.
   Preserve camera topics and perception behavior when consumers are active.
3. Measure the complete downstream workload, including processing latency and
   perception quality. Confirm timestamp propagation before using message
   headers to calculate latency.
4. Compare GUI and physics costs separately. Do not change the physics step
   without validating simulation fidelity; test lower camera resolution or
   frame rate only as an explicit quality/performance trade-off.
