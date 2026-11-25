# Simulation startup optimization: architecture and measured impact

**Measurement date:** 2026-11-04  
**Purpose:** Presentation-ready summary of the startup changes, architecture, and
observed timings.

## Summary

Two startup costs were addressed: rendering the road texture on every launch
and starting a separate ROS launch process for every Gazebo entity.

- A source-aware road texture cache reduces a warm render from **9.838 s to
  0.006–0.010 s** in the measured runs.
- A persistent Gazebo spawn-service client submits independent entities in
  batches of up to four. In a same-session comparison, the entity-spawn phase
  decreased from **6.189 s to 0.128 s**.
- In the measured full `world_spawner` phase, the legacy combination took
  **19.124 s**, while the latest cached/service run took **0.134 s**. That is
  an observed difference of **18.990 s (about 99.3%)**. These two runs were not
  a controlled paired benchmark, so this combined figure is indicative, not a
  guaranteed or statistically established saving.
- The previous paths remain available for comparison:
  `road_render_mode:=legacy` and `entity_spawn_mode:=legacy`.

## Architecture change

### Before

```text
WorldSpawner
  ├─ Render road texture on every launch
  ├─ Start ROS launch for road ───────── wait for completion
  ├─ Start ROS launch for sign 1 ──────── wait for completion
  ├─ Start ROS launch for sign 2 ──────── wait for completion
  ├─ ... one launch and wait per entity
  └─ Start ROS launch for each obstacle ─ wait for completion
```

Each entity went through a separate `ros_gz_sim` launch path. The next entity
was not requested until that path had finished.

### After

```text
WorldSpawner
  ├─ Reuse road texture if source fingerprint and image are valid
  ├─ Send road through persistent /world/smartrollerz/create service
  ├─ Wait for road success
  └─ Submit signs and obstacles in bounded batches (default: 4 concurrent)
       └─ Check every service response; fail visibly on errors/timeouts
```

The road is kept as a first step so later scene objects are only spawned after
the road succeeds. Signs and obstacles are submitted in batches because they
are independent of each other. The legacy process-per-entity implementation is
still selectable.

## Measured times

### Road texture rendering

| Path | Render phase | Result |
|---|---:|---|
| Always render (legacy) | 9.838 s | Baseline run |
| Warm cache | 0.008 s, 0.010 s, 0.006 s | Median: 0.008 s |
| Difference in the direct phase comparison | About 9.83 s saved | Approximately 99.9% lower render time |

The cache skips rasterization only when the source fingerprint and generated
image are still valid. A cache miss or source change still renders the road.

### Entity insertion with the same warm-cache setup

| Spawning architecture | Render | Entity spawning | Full `world_spawner` phase |
|---|---:|---:|---:|
| Legacy: separate launch per entity | 0.009 s | 6.189 s | 6.198 s |
| Service: persistent client, concurrency 4 | 0.006 s | 0.128 s | 0.134 s |
| Observed reduction | — | **6.061 s (about 97.9%)** | **6.064 s (about 97.8%)** |

All 11 expected entities completed in both runs. The measured service-mode
phase includes the road spawn and service responses for the remaining scene
objects.

### Combined result observed

| Measured path | Full phase |
|---|---:|
| Earlier legacy rendering + sequential spawning run | 19.124 s |
| Latest warm-cache + service spawning run | 0.134 s |
| Observed difference | **18.990 s (about 99.3%)** |

This combined comparison shows the scale of the improvement, but it combines
measurements from separate runs. Use the render-only comparison and the
same-cache spawn comparison above when attributing savings to each change.
These timings cover work inside `world_spawner`; they are not total
process-launch-to-ROS-ready measurements.

## How to compare the implementations

Keep the road, GUI setting, host load, and cache state the same. Run each mode
several times, record the `PERF` lines, and compare medians.

```bash
# Original per-entity launch architecture
ros2 launch simulation simulation.launch.py rviz:=false entity_spawn_mode:=legacy

# Persistent service with bounded concurrency (default is 4)
ros2 launch simulation simulation.launch.py rviz:=false entity_spawn_mode:=service spawn_concurrency:=4

# Also force the original always-render road path
ros2 launch simulation simulation.launch.py rviz:=false road_render_mode:=legacy entity_spawn_mode:=legacy
```

## Interpretation and next step

The results support replacing repeated launch-process startup and synchronous
per-entity waits with the persistent service path for this scene. The
service-mode result is currently one run per mode, so repeat runs before
presenting the percentage as a stable benchmark. For larger scenes, test a few
concurrency values (for example 1, 2, 4, and 8); higher concurrency may not
help if Gazebo serializes the underlying insertion work.
