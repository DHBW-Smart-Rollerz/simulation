# Pre-branch simulation baseline: laptop

This report records a baseline for startup and steady-state simulation on one
laptop. It is not a measurement of the physical car or the complete perception
and control stack. The purpose is to provide a reproducible reference for
future runtime and startup changes and their evaluation in the thesis.

## Code and configuration

| Component | Revision or setting |
|---|---|
| Simulation baseline | `e783ce86964166c76be6308a97493de1f45b3358`, the merge base of `runtime-investigation` and `jazzy` when measured |
| Workspace | `ce2ab4f19cb3bc65f03852768fb1f6d3ef2d2385` |
| `smarty_utils` | `c8ad231c280c3f96f1a2b7671813e81d2df57b8e`, clean |
| `camera_preprocessing` | `37dfed02972e381478a3ae34afafdc8fec5ba358`, clean but not launched |
| Build | Isolated simulation worktree and `colcon` overlay, `--symlink-install`, `RelWithDebInfo`; existing workspace overlay supplied support packages |
| Launch | `ros2 launch simulation simulation.launch.py rviz:=false` |
| Scenario | Default road, Gazebo GUI enabled, RViz disabled, no downstream Smarty stack |
| Camera | Simulated wide-angle camera configured for 2064 × 1544 at 30 Hz; default camera bridge enabled |
| Physics | 1 ms maximum step, target real-time factor 1.0 |
| Middleware isolation | ROS domain 127; a separate Gazebo partition for each run |

Configured camera frequency is per simulated second, not proof of achieved
wall-clock delivery. The original source was built separately and selected by
its overlay; selecting `legacy` modes on the branch is useful for diagnostic
comparisons but is not a substitute for this pre-branch revision.

## Laptop and software

| Setting | Observed value |
|---|---|
| Computer | Framework Laptop 13 (AMD Ryzen 7040 Series), product version A5 |
| CPU | AMD Ryzen 5 7640U, 6 cores / 12 logical CPUs |
| GPU | Integrated AMD Radeon 760M, `amdgpu` kernel driver |
| Memory | 30.66 GiB reported by the OS; 8 GiB swap configured |
| OS | Ubuntu 24.04.5 LTS, Linux `7.0.0-43-generic`, x86-64 |
| Display | Built-in display, 1920 × 1280 at approximately 119.92 Hz; Wayland desktop |
| Power | AC connected, battery charging; charge level is recorded per run |
| Power profile | `performance` |
| CPU policy | `amd-pstate-epp`, `performance` governor and energy preference; boost enabled; policy limits approximately 1.116–4.972 GHz |
| GPU power policy | `auto` |
| Mesa | 25.2.8 |
| ROS / Gazebo | ROS 2 Jazzy; Gazebo Sim 8.15.0; `ros_gz_bridge` 1.0.24 |
| Python environment | Python 3.12.15; NumPy 2.5.4; Pycairo 1.29.2; setuptools 73.0.1 |

These are observed settings, not evidence that CPU frequency, temperature,
graphics workload, or other processes were constant during the experiments.
The actual GPU renderer was not independently verified. Load averages are
recorded in each raw result. Preserve the same display and power settings for
comparison and record temperatures, background workload, renderer, package
versions, and any deviations.

## Method

Each trial started the original simulation in a fresh process. The measurement
observer used a monotonic wall clock:

1. **Start** immediately before starting `ros2 launch`.
2. **ROS ready** when the five expected simulation nodes are discoverable and
   `/clock` has advanced between two received messages.
3. **Scene ready** when ROS is ready and `gz model --list` reports the
   ground plane, vehicle, road, eight signs, and two obstacles. The model
   query is polled; its completion time is an upper-bound observation, not the
   exact insertion instant.
4. Wait 30 seconds after scene readiness, then collect Gazebo world-statistics
   messages for 60 seconds. Calculate window RTF as change in reported
   simulation time divided by elapsed monotonic wall time between the first
   and last valid samples. Record the individual reported RTF values too.
5. Shut down the launch and its remaining child processes, wait 30 seconds,
   and repeat. Each run uses its own Gazebo partition.

The observer and model-list polling add some overhead; keep the same probe and
polling procedure when measuring changes. There was no camera subscriber during
the primary RTF sample: the enabled, non-lazy bridge still converts/publishes
images. This simulation-only workload does not establish perception throughput
or full-system readiness.

## Results: 25 November 2026

These are five consecutive warm-system runs, after exploratory launches, not
cold-boot measurements. All runs verified the 13 expected models. The raw
per-run metadata and approximately 60 seconds of statistics samples are in
[`benchmarks/laptop-2026-11-25/`](benchmarks/laptop-2026-11-25/).

| Run | ROS ready (s) | Scene ready (s) | Window RTF | Battery at start |
|---:|---:|---:|---:|---:|
| 1 | 3.629 | 16.586 | 0.98730 | 48% |
| 2 | 2.805 | 14.619 | 0.99604 | 49% |
| 3 | 3.690 | 16.328 | 0.99306 | 50% |
| 4 | 3.716 | 14.986 | 0.99083 | 51% |
| 5 | 3.709 | 14.606 | 0.99596 | 52% |
| **Median** | **3.690** | **14.986** | **0.99306** | — |
| **Range** | **2.805–3.716** | **14.606–16.586** | **0.98730–0.99604** | — |

Window RTF represents simulated seconds per wall second: the median is about
99.3% of real time for this particular simulation-only workload. It does not
mean that the camera delivered 30 images per wall second. The one-minute
system load average before launch varied from 3.12 to 6.12, and the laptop was
charging throughout. Background load, temperatures, and cache state were not
fully controlled; these results are a measured reference, not a controlled
before/after comparison or a confidence interval.

The test ran from a clean desktop environment rather than inheriting VS Code's
Snap-specific GTK libraries: an initial smoke test failed to start the Gazebo
GUI because of an incompatible Snap `libpthread`. A second smoke test confirmed
that display authorization must also be retained. The final runs used the
normal authenticated desktop display and excluded the Snap-specific library
environment. An attempted repeat was rejected because Gazebo processes from a
smoke test had survived shutdown; the five reported runs use unique partitions
and verified child-process cleanup. Neither failed attempt is included above.
All five runs logged Gazebo failing to exit within five seconds of the
requested SIGINT and being escalated to SIGTERM after sampling. The remaining
tracked children were subsequently stopped; this is a shutdown issue, not a
startup or RTF timing. Sanitized launch logs are retained with the raw results;
local paths and the machine-identifying log directory were replaced.

### Separate camera and resource diagnostic

One additional run used the same scene and warm-up, but added a
`ros2 topic hz /camera/image/raw --wall-time --window 10000` subscriber and
`pidstat -u -r` at one-second intervals during the 60-second stats window. Its
window RTF was 0.98941. The last camera-rate report was **13.742 received
frames/s**, with 802 samples in the rolling window. This is the rate at the
probe subscriber, not a direct measurement of the publisher; image copying,
subscriber overhead, QoS, and load can affect it. It is not a five-run
camera-throughput baseline.

| Process | Mean CPU, per logical core | Mean RSS |
|---|---:|---:|
| Gazebo server | 126.05% | 402,084 KiB |
| Gazebo GUI | 103.37% | 506,012 KiB |
| ROS–Gazebo bridge | 87.85% | 83,152 KiB |
| Camera-rate probe | 33.22% | 92,009 KiB |

CPU values above 100% mean more than one logical CPU; RSS is resident memory
averaged by `pidstat`, not peak memory. The camera subscriber was present
throughout this diagnostic window, unlike the five primary runs, so these
resource numbers must not be presented as the primary runs' resource use.
The [camera-rate samples](benchmarks/laptop-2026-11-25/diagnostic-camera-rate.txt),
[process samples](benchmarks/laptop-2026-11-25/diagnostic-pidstat.txt), and
[RTF samples](benchmarks/laptop-2026-11-25/diagnostic-run.json) are retained.
The machine-identifying `pidstat` header and camera tool's shutdown traceback
were omitted; the measurement rows remain. To repeat this diagnostic, run the
two probes concurrently after scene readiness and the same 30-second warm-up,
selecting the Gazebo server, GUI, bridge, and receiver process IDs explicitly.

## Reproducing the measurement

Use the same laptop, software, display, power profile, road, and support-package
revisions. Work in a normal desktop terminal with display authorization, not a
Snap-injected shell. The commands below assume the current workspace is the
working directory and the `default` Python environment contains the existing
workspace dependencies. Use a new, empty benchmark directory; do not overwrite
previous results.

```bash
BASE="$HOME/smarty-benchmark-baseline"
git -C simulation worktree add --detach "$BASE/simulation" \
  e783ce86964166c76be6308a97493de1f45b3358
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python -m colcon --log-base "$BASE/log" build \
  --base-paths "$BASE/simulation" \
  --build-base "$BASE/build" --install-base "$BASE/install" \
  --symlink-install --packages-select simulation \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source "$BASE/install/local_setup.bash"

# Both paths must point into the separate baseline overlay.
ros2 pkg prefix simulation
python -c 'import simulation; print(simulation.__file__)'

export ROS_DOMAIN_ID=127
export PYTHON_EXECUTABLE="$(command -v python)"
python simulation/benchmarks/measure_baseline.py \
  --runs 5 --output "$BASE/results"
```

The [measurement script](benchmarks/measure_baseline.py) writes individual
JSON files and launch logs. It hard-codes this road's expected model names and
the laptop's power-supply paths; adapt and document those for other scenarios
or hardware. It waits for tracked child processes to exit before proceeding.
Review logs for launch/shutdown errors as well as the JSON status. Repeat the
same measurement on the candidate build in a separate overlay and output
directory, never using this baseline's generated files as candidate results.
The checked-in script was refactored for linting, type checks, and shutdown
handling after collecting the five results; its measurement procedure is
unchanged.

## Interpretation and follow-up

The primary objective is to retain the configured camera resolution and frame
rate. Investigate these hypotheses one at a time:

1. Measure the existing road-render cache and persistent spawn client against
   this original commit, separating cold and warm starts and checking that
   every scene entity and road texture is correct. Phase timings in
   [performance guide](PERFORMANCE.md#exploratory-measurements-4-november-2026)
   are exploratory and do not establish end-to-end startup savings.
2. Investigate image conversion and transport costs at the existing camera
   settings. The bridge-off experiment in [PERFORMANCE.md](PERFORMANCE.md)
   is diagnostic only: it removes camera output from ROS and therefore is not
   functionally equivalent to a perception workload.
3. Measure a separate workload with the actual downstream stack and subscribers,
   including achieved camera and processed-image rates, frame drops, resource
   usage, latency, and perception/control quality. Check timestamp propagation
   before using message headers for end-to-end latency: the current
   preprocessing publisher does not copy the input image header.
4. Only as a controlled trade-off experiment, test lower resolution and frame
   rate separately. Update and verify camera calibration and perception
   compatibility, and compare task quality and latency, not just CPU/RTF.

For any proposed improvement, repeat the same number of trials on this laptop
with the same versions, display, power, scene, warm-up, and probes. Alternate
baseline and candidate runs where practical, report individual values and
median/range, and preserve failed runs with reasons. Compare startup using
`100 × (candidate − baseline) / baseline`; use the same formula for RTF but
interpret a positive RTF change as an improvement. Agree on acceptance
thresholds with the project lead before testing; do not infer them from these
measurements.

This baseline does not measure cold boot, physical camera/driver startup,
autonomous driving readiness, full-stack performance, or failure recovery.
Those require separate, explicitly defined scenarios and, for the car, safe
stationary testing.
