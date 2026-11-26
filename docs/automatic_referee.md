# Automatic Referee: Feasibility Study

This document evaluates whether an **automatic referee** can score simulated runs according to the **CAuDri-Challenge Regulations 2026** (source snapshot `regulations-2026-05-06`, chapter 7 *Dynamic Events*). The referee would give and take points the same way the official referees do on the real track. The three dynamic events are evaluated separately:

- Free Drive Course (Reg. 7.3, `freedrive_course.tex`)
- Obstacle Evasion Course (Reg. 7.4, `obstacle_course.tex`)
- Navigation Course (Reg. 7.5, `navigation_course.tex`)

For every course this document covers what can be automated, how hard it is, which problems to expect and how much simulation performance it costs. It also compares several implementation approaches.

It builds on [regulations_2026_gap_analysis.md](regulations_2026_gap_analysis.md). IDs such as `DYN-02`, `VEH-01` or `REF-05` refer to items in that document.

---

## Table of Contents

1. [Summary](#1-summary)
2. [What the referee can rely on today](#2-what-the-referee-can-rely-on-today)
3. [Implementation choices](#3-implementation-choices)
4. [Shared building blocks](#4-shared-building-blocks)
5. [Free Drive Course](#5-free-drive-course)
6. [Obstacle Evasion Course](#6-obstacle-evasion-course)
7. [Navigation Course](#7-navigation-course)
8. [Cross-cutting problems and risks](#8-cross-cutting-problems-and-risks)
9. [Performance impact on the simulation](#9-performance-impact-on-the-simulation)
10. [Suggested implementation order](#10-suggested-implementation-order)
11. [Open questions](#11-open-questions)

**Difficulty scale used below**

| Rating | Meaning |
|---|---|
| **Easy** | Direct geometry check on ground-truth data. No interpretation of the rules needed. |
| **Medium** | Needs a small state machine (approach → act → leave) and/or one or two thresholds the rules do not define. |
| **Hard** | Needs complex scenario logic, causal reasoning ("was this caused by …?") or several undefined thresholds. |
| **Blocked** | Cannot be implemented until a simulation feature exists (named in the cell). |
| **Manual** | Not observable in the simulation. Must be entered by hand or ignored. |

**Effort** follows the gap analysis: **S** < 1 day, **M** 1–3 days, **L** > 3 days.

---

## 1. Summary

**An automatic referee is possible and worth building.** Unlike the human referees, the simulation knows the exact pose of the car, every lane line, every stop line and every obstacle. Most rules that are hard to judge by eye (for example "≤ 150 mm before the stop line" or "two or more wheels across the marking") become simple geometry checks.

The limits do not come from the referee itself. They come from **missing simulation features** (dynamic obstacles, pedestrians, landmarks, light and RC-mode interfaces) and from **rules that the regulations leave open** (for example what counts as a "faulty" brake light).

| Course | Feasible? | Share of scoring rules automatable **today** | Share automatable **after the listed sim features exist** | Referee effort | Main blockers |
|---|---|---|---|---|---|
| **Free Drive** | **Yes, nearly fully** | 3 of 4 penalties, distance, timer | 4 of 4 penalties | **M** (incl. shared foundation) | Brake-light signal from the stack (VEH-01). Multipliers are manual. |
| **Obstacle Evasion** | **Yes, partially now, mostly later** | About half the element tables (static obstacles, intersections without traffic, barred area without oncoming traffic, crosswalk without pedestrian, no-passing line crossing, speed limit, sign collisions, lane departure) | Nearly all. Some decisions (RC-mode cause, attempt validity) stay **"needs review"** flags. | **L** (2–4 weeks) | Dynamic obstacles and pedestrians (DYN-02 … DYN-07); indicators and brake lights (VEH-01); RC mode (VEH-02) |
| **Navigation** | **Yes, but only after track work** | Distance, timer, RC count; lane departure only on single-loop tracks | All rules | **M** for the referee itself, **L** including the road graph | Landmarks (ELEM-08); the road DSL is a single chain and cannot describe tracks without a designated route; mapping/navigation phase reset (EVT-05) |

**Recommended architecture** (details in [section 3](#3-implementation-choices)): an **online ROS 2 referee node** that evaluates the run live. Its scoring logic sits in a **ROS-free core** that the node feeds sample by sample, so the logic can be unit-tested without Gazebo. Scoring recorded runs (rosbags) is **not a requirement**. The car pose and speed come from `/smarty/odometry`. The track geometry (lines, stop lines, obstacles, signs, parking spots) comes from the `Road` object, which the node loads with the same seed as the spawner. Distances to lines and obstacles are computed from these two sources ([section 3.3](#33-where-the-referee-gets-its-data)). The runtime cost is **well below 5 % of one CPU core**, which is negligible next to camera rendering and physics ([section 9](#9-performance-impact-on-the-simulation)).

---

## 2. What the referee can rely on today

The referee should only use **ground truth**, never the stack's own perception. The table lists every input it needs and whether the simulation provides it today.

| Information | Source today | Status | Notes |
|---|---|---|---|
| Car pose (x, y, yaw) and speed | `/smarty/odometry` (`nav_msgs/Odometry`, OdometryPublisher, world frame, about 50 Hz by default, no noise) | ✅ | Already bridged. It is configured **BIDIRECTIONAL** in [ros_gz_bridge.yaml](../config/ros_gz_bridge.yaml), so the referee must never publish on it. |
| Link poses (wheels, chassis) | `/tf` from the PosePublisher | ✅ but expensive | `update_frequency` is not set in [world.sdf](../models/world.sdf#L96-L106), so it publishes **every physics step (1 kHz)**. Avoid it in Python; compute the wheel positions from the car pose instead (see below). |
| Wheel contact points | Derived from the car pose and [model.sdf](../models/smarty/model.sdf) | ✅ | In the model frame: front wheels at x = +0.140, rear wheels at x = −0.115, y = ±0.0845 (wheel width 0.028). Steering moves the front contact points by only a few mm. |
| Car footprint (front bumper) | Chassis collision box 0.370 × 0.125 m, centred on the model origin; wheels stick out to y = ±0.0985 | ⚠️ | The front edge (x = +0.185) is needed for the "≤ 150 mm before the stop line" rule. Check it against the real Smarty dimensions; the visual mesh may be longer than the collision box. |
| Road geometry (lanes, lines, line types) | `Road` object: `section.middle_line`, `left_line`, `right_line`, `lines` (`MarkedLine` with style) | ✅ | Road coordinates are **identical to world coordinates**: the road plane is spawned centred at the origin and the PNG is rendered symmetrically ([world_spawner.py:103-164](../simulation/world_spawner.py#L103-L164)). |
| Designated route and arc length | Concatenated `middle_line`s of `road.sections` with `prev_length` | ⚠️ | The section chain **is** the designated route. A crossing appears **twice** in the chain (two overlaid `Intersection` sections, see [default_road](../models/roads/default_road/road.py)), so projecting onto the nearest section is ambiguous there (see [4.2](#42-route-tracker)). |
| Stop and give-way lines | `Intersection.surface_markings` (`SurfaceMarkingRect` with kind) | ⚠️ | Lines are missing for several turning cases (FIX-02, FIX-03). The referee can only evaluate lines that exist. |
| Intersection type and mandatory turn | `Intersection.rule`, `Intersection.turn` | ✅ | Enough to pick the correct scoring table. |
| Static obstacles | `section.obstacles` (`StaticObstacle.frame` polygon); spawned as `obstacle_<idx>` | ⚠️ | Spawned as **non-static 1 kg bodies** (FIX-10). After a collision the real position differs from the `Road` data. |
| Traffic signs | `section.traffic_signs` (`TrafficSign.frame`, `kind`); spawned as `<mesh>_<idx>` | ✅ | Needed for "collision with road sign". |
| Parking spots | `ParkingArea` → `ParkingLot` → `ParkingSpot` (`frame` polygon, `kind` FREE / OCCUPIED / BLOCKED) | ✅ | Spot validity is directly available. |
| Barred area, crosswalk | `BlockedArea.frame`, `ZebraCrossing.frame` | ✅ | Barred areas only in the right lane today (FIX-06). |
| Speed-limit zones | `section.speed_limits` (`SpeedLimit.arc_length`, `speed`; negative = end of zone) | ✅ | Scale 1:10: sign value in km/h ÷ 10 → m/s ÷ 3.6. |
| No-passing zones | Implicit: `middle_line_marking` ∈ {double solid, solid on the ego side} | ⚠️ | No explicit element or signs (ELEM-04), but the line style is enough for the referee. |
| Dynamic obstacles, pedestrians | – | ❌ | DYN-02 … DYN-07. |
| Landmarks | – | ❌ | ELEM-08. |
| Start line, start box, start signal | Start line only inside `ParkingArea`; no start box or start signal | ⚠️ / ❌ | The referee can treat arc length 0 of the chain as the start line. A start signal is needed for the timer (EVT-01, EVT-02). |
| Brake light, turn indicators | – | ❌ | The stack's light-command topic must be mirrored (VEH-01). |
| RC-mode state | – | ❌ | VEH-02. `keyboard_control` publishes directly to the control topics today and is not a real RC mode. |
| Simulation time | `/clock` | ✅ | All referee timing **must** use sim time. The sim can run at ~30 % real time. |

**Existing code to reuse:** the unmerged `origin/feature/labeling` branch contains `groundtruth_labeler.py`. Its `_get_car_arc_length()` already projects the car onto the section chain **with temporal continuity**, which solves the overlaid-intersection ambiguity. Its `_precompute_road_points()` samples the road into NumPy arrays. Both can seed the referee's route tracker.

---

## 3. Implementation choices

### 3.1 Architecture options

#### Option A: Online ROS 2 Python node

A new node `referee.py` (registered in `setup.py`, started from the launch file) subscribes to `/smarty/odometry`, `/clock` and the light, RC and actor topics. It loads the same road as `world_spawner` and evaluates live.

- **Pros:** Fits the existing package (Python, `ament_python`, the node conventions in `CLAUDE.md`). Live feedback in RViz and the terminal. Can publish events that other nodes (scenario controllers) react to. Easy for the team to maintain.
- **Cons:** Correctness depends on not dropping messages. A callback that falls behind loses samples unless the QoS depth is large enough. It is hard to unit-test if logic and ROS plumbing are mixed (Option D fixes this).
- **Effort:** Medium. **Performance:** low (see [section 9](#9-performance-impact-on-the-simulation)).

#### Option B: Gazebo system plugin (C++)

A gz-sim system (`ISystemPostUpdate`) runs inside the simulation loop and reads entity poses and contacts directly from the ECM at every physics step.

- **Pros:** Evaluates every 1 ms step, perfectly synchronous with physics, independent of ROS transport, and contacts are available natively.
- **Cons:** Needs a separate `ament_cmake` package next to this `ament_python` package. The track geometry lives in Python, so it would have to be exported (for example as JSON) and re-implemented with a C++ geometry library. Slow referee code **directly lowers the real-time factor** because it blocks the physics loop. The team's code base is Python. The 1 kHz resolution is not needed: no rule is time-critical below about 20 ms.
- **Effort:** High. **Performance:** low if well written, but any cost is paid inside the physics loop.

#### Option C (not pursued): Offline evaluation from a rosbag

Record the inputs during the run and score them afterwards by replaying the bag. This would allow re-scoring old runs, but **that is not a requirement**. It also gives no live feedback, so it is not pursued. The design below does not prevent adding it later: a bag reader would only have to feed the same samples into the core.

#### Option D: Online ROS node with a ROS-free core (recommended)

This is Option A with the scoring logic separated from the ROS plumbing:

1. **`simulation/referee_node.py`**: the ROS 2 node, and the **only** part that talks to ROS. It loads the road, subscribes to the input topics, turns every message into a small typed sample and passes it to the core. It publishes the results (`/referee/events`, RViz markers) and writes the report at the end of the run. Section [3.3](#33-where-the-referee-gets-its-data) describes exactly what it reads.
2. **`simulation/referee/core/`**: plain Python with no `rclpy` import. It receives the track model once and then `update(sample)` calls in timestamp order (`VehicleState`, later `ActorState`, `SignalState`, `PhaseEvent`). It computes all distances and evaluations and returns `RefereeEvent`s and a final `ScoreReport`.

- **Pros:** Live feedback, and it fits the package conventions. The core can be unit-tested with **synthetic trajectories** (for example "drive straight, stop 120 mm before the line for 3.2 s"), without ROS or Gazebo and even on Windows, like the existing `groundtruth` tests. Point values live in a per-year YAML file.
- **Cons:** It needs a clean sample interface up front. It is slightly more code than Option A.
- **Effort:** Medium (only marginally more than A).

#### Option E (rejected): Vision-based referee

Mount a virtual top-down camera in Gazebo and detect lane departures and stops with computer vision, as a human referee would.

- **Rejected because:** It throws away exact ground truth, adds a heavy render load (a second camera), and introduces detection errors into the scoring. It would only make sense to *test the referees*, not the car.

### 3.2 Comparison

| Criterion | A: Online node | B: gz plugin | **D: Node + ROS-free core** |
|---|---|---|---|
| Fits the codebase (Python, ament_python) | ✅ | ❌ | ✅ |
| Live feedback (terminal, RViz) | ✅ | ⚠️ (via gz topics) | ✅ |
| No dropped samples | ⚠️ (QoS depth) | ✅ | ⚠️ (QoS depth) |
| Unit-testable without Gazebo | ⚠️ | ❌ | ✅ |
| Runtime cost during simulation | Low | Low, but inside the physics loop | Low |
| Implementation effort | M | L | M |

**Recommendation: Option D.** Build the core together with its unit tests first, then the node around it.

### 3.3 Where the referee gets its data

The core does not fetch anything itself. The referee node collects two kinds of data and hands them to the core. Everything else the rules need (where the wheels are, distances to lines and obstacles, speed and deceleration) is **computed** from these two sources.

```
 Startup (once)                               Every odometry message (~50 Hz)
 ──────────────                               ───────────────────────────────
 ros_params.yaml ─► road path + seed          /smarty/odometry (nav_msgs/Odometry)
        │                                       header.stamp           → t (sim time)
        ▼                                       pose.position.x/y      → x, y
 road.load(path, seed)                          pose.orientation       → yaw
 (same call as world_spawner)                   twist.linear.x         → v
        │                                               │
        ▼                                               ▼
 TrackModel: route, lane lines, stop lines,    VehicleState(t, x, y, yaw, v)
 obstacles, signs, spots, zones, speed limits           │
        │                                               │
        └──────────────► referee core ◄─────────────────┘
                              │  computes wheel points, footprint,
                              │  arc length, distances, acceleration
                              ▼
                RefereeEvents → /referee/events, RViz, report
```

#### Static data: the track (loaded once at startup)

The node reads the same road parameter as `world_spawner` (`world_spawner.ros__parameters.road` in [ros_params.yaml](../config/ros_params.yaml), or its own copy of it). It then calls `road.load(path, seed)`, exactly as [world_spawner.py:48](../simulation/world_spawner.py#L48) does. Because `load()` seeds `random` first, the referee gets **the same geometry** that was spawned in Gazebo, and road coordinates equal world coordinates (section 2). From the `Road` object the core builds its track model once:

| Needed for | Taken from the `Road` object |
|---|---|
| Lanes, line positions and line types | `section.middle_line`, `left_line`, `right_line`, `section.lines` (`MarkedLine` with style) |
| Route and arc length | Section order in `road.sections`, `section.prev_length`, `middle_line.length` |
| Stop and give-way lines | `Intersection.surface_markings` (kind and polygon) |
| Intersection rule and turn | `Intersection.rule`, `Intersection.turn` |
| Obstacles | `section.obstacles` (`frame` polygon, `center`, size) |
| Signs | `section.traffic_signs` (`frame`, `kind`) |
| Parking spots | `ParkingArea` → lots → `ParkingSpot.frame` and `kind` |
| Barred areas, crosswalks | `BlockedArea.frame`, `ZebraCrossing.frame` |
| Speed limits | `section.speed_limits` (`arc_length`, `speed`) |

These objects already wrap shapely geometry, so the core can measure distances on them directly.

#### Live data: the car (every message)

| Value | Topic and field | Notes |
|---|---|---|
| Time `t` | `header.stamp` of each message | Sim time. Set `use_sim_time: true` on the node; never use the callback's wall-clock time. |
| Position `x, y` | `/smarty/odometry` → `pose.pose.position` | World frame, ground truth without noise, published by the gz OdometryPublisher at about 50 Hz. The rate can be raised with `<odom_publish_frequency>` in [world.sdf](../models/world.sdf#L110-L116). |
| Heading `yaw` | `/smarty/odometry` → `pose.pose.orientation` | Convert the quaternion to yaw. |
| Speed `v` | `/smarty/odometry` → `twist.twist.linear.x` | Forward speed in the car frame. `/sensor/velocity` would also work, but it comes from a different message and has its own timestamp. Using one message keeps pose and speed consistent. |

The topic already exists in the bridge, so no new bridge entry is needed. It is configured as BIDIRECTIONAL, so the referee must only subscribe, never publish.

#### Values the core computes itself

| Value | How it is computed |
|---|---|
| Wheel contact points | Car pose combined with the fixed wheel offsets from [model.sdf](../models/smarty/model.sdf): front (+0.140, ±0.0845), rear (−0.115, ±0.0845). |
| Footprint and front bumper | Rectangle around the car pose (about 0.370 × 0.197 m, to be checked against the real car); front bumper = the point 0.185 m ahead of the model origin. |
| Position along the route (arc length `s`) | Project the car position onto the current section's `middle_line` (route tracker, 4.2). |
| Distance of each wheel to a line (lane check) | Signed lateral distance of the wheel point from the section's `middle_line` (shapely `project` / `distance`), compared with the lane width. |
| Distance to a stop or give-way line | Front bumper arc length minus the arc length of the line's near edge. |
| Distance to an obstacle or sign | `footprint.distance(obstacle.frame)`. A value of 0 means collision. For following distances, the gap in `s` between the bumpers. |
| Inside a parking spot or barred area | Point-in-polygon of the four wheel points against `spot.frame` / `area.frame`. |
| Acceleration | Filtered derivative of `v` over time (needed for the brake-light rule). |
| Standstill duration | Time span during which `|v|` stays below a threshold. |

#### Data that will be added later

These inputs do not exist yet. Each will be one more subscription in the node and one more sample type for the core:

| Data | Source (once it exists) |
|---|---|
| Brake light, indicators, hazard | The stack's light-command topic (VEH-01) |
| RC-mode state | The RC-mode mux (VEH-02) |
| Dynamic obstacle and pedestrian poses and states | The actor controller node (DYN-02 … DYN-07). This is preferably the controller's own topic, because it also knows the actor's intent (waiting, crossing, has priority). The alternative is live model poses from Gazebo. |
| Start signal and phases | Start scheduler (EVT-02), or a `/referee/start` service until then |
| Landmark positions | Part of the `Road` once ELEM-08 exists, so static data like the rest of the track |

### 3.4 Choices for the core sub-problems

Several sub-problems have more than one reasonable solution. The recommended choice is listed first.

#### a) Car pose source

| Choice | Pros | Cons |
|---|---|---|
| **`/smarty/odometry` (50 Hz) plus fixed wheel offsets** (recommended) | Already bridged, cheap, one message per sample. Pose and twist in one message. | 50 Hz = 4 cm of travel per sample at 2 m/s. Enough for every rule; the rate can be raised with `<odom_publish_frequency>`. |
| `/tf` link poses (1 kHz) | Exact wheel positions including steering. | 1 kHz `TFMessage` deserialisation in Python costs noticeable CPU and adds no value. |
| Direct gz-transport Python bindings (`gz.transport13`, for example `/world/smartrollerz/pose/info`) | No ROS bridge needed; also gives **live poses of all models** (pushed obstacles, actors). | Extra dependency and a second transport layer in the node. Keep it as an option for actor poses. |

#### b) Route tracking (where on the designated route is the car?)

| Choice | Pros | Cons |
|---|---|---|
| **Projection onto section middle lines with temporal continuity** (recommended; exists in `feature/labeling`) | Gives arc length, section and lateral offset in one step. Handles overlaid intersections by preferring the candidate closest to the previous arc length. | Needs a reset strategy after teleports (RC repositioning, start box). |
| Section-transition state machine (enter or leave section bounding boxes) | Robust against jumps. | Coarse and gives no lateral offset. Section bounding boxes overlap at intersections. |

#### c) Lane check ("crossing the lane markings with two or more wheels")

| Choice | Pros | Cons |
|---|---|---|
| **Frenet lateral offset per wheel** (recommended) | For each wheel, `d = signed distance to the current section's middle line`. The wheel is in the right lane if `−(lane_width + tol) ≤ d ≤ +tol`. Cheap (4 projections per sample), and it uses the route tracker that is needed anyway. | Inside intersections and parking areas the "lane" is not a simple corridor, so dedicated zones are needed there. |
| Shapely lane polygons and point-in-polygon with an `STRtree` index | Exact and works for any shape (intersections, parking lots). | More preprocessing (building lane polygons per section, handling overlaps at intersections). |
| Rasterised lane-ID map (render a label image with cairo, reusing `render_to_file`, then look up pixels at the wheel positions) | Fastest lookup, O(1) per wheel, and reuses the renderer. | Overlapping intersections need multiple labels per pixel. About 2 mm resolution at 512 px/m. Larger memory footprint. Not needed at 50 Hz. |

A combination works well: use Frenet offsets on normal sections and polygons for special zones (intersection box, parking lot, barred area).

#### d) Collision detection (obstacles, signs, pedestrians)

| Choice | Pros | Cons |
|---|---|---|
| **2D footprint overlap** (car rectangle against obstacle/sign footprint, shapely) (recommended) | No change to the SDFs. Testable without Gazebo. Obstacles are ≥ 100 mm high and the chassis bottom is at ~52 mm, so a 2D overlap is effectively a collision. | Uses the *planned* obstacle position from the `Road`. If an obstacle is pushed (FIX-10), later laps are wrong until obstacles are made static or live poses are used. |
| Gazebo contact sensor (`gz-sim-contact-system` plus `<sensor type="contact">`) | Reports real physical contacts, including pushed objects. | Needs a world plugin, a bridge entry and a sensor per car link. The chassis box (0.125 m wide) does **not** cover the wheels (0.197 m), so sensors on the wheels must filter ground contacts. Online only. |

Recommendation: use footprint overlap first. Add a contact sensor later as a cross-check if the footprint gives false positives.

#### e) Vehicle signals (brake light, indicators, RC mode)

| Choice | Pros | Cons |
|---|---|---|
| **Subscribe to the stack's real light and RC topics** (recommended) | The simulation stays a drop-in replacement, and the referee scores what the real car would show. | The topic names and types are not known yet (gap analysis, open question 1). In the simulation they are *commanded* states: the blink pattern (≤ 2 Hz, 50 %, start "on") is generated by firmware on the real car and cannot be checked unless the stack sends each on/off edge. |
| Sim-only `/referee/signals` topic that the stack must publish additionally | Simple to define. | Breaks the drop-in principle and can diverge from the real behaviour. |

#### f) Point tables

| Choice | Pros | Cons |
|---|---|---|
| **YAML per regulation year** (`config/referee_2026.yaml`: points, caps, thresholds, time limits) with the logic in code (recommended) | Rule updates usually change numbers, not logic. Thresholds that the rules leave open are visible and documented in one place. | The logic still has to change when a rule changes qualitatively. |
| Fully declarative rule engine | Maximum flexibility. | Over-engineering: most rules need custom geometry anyway. |

---

## 4. Shared building blocks

These components are needed by all three courses. Together they form the **foundation** (about **M–L** in total).

| # | Building block | What it does | Difficulty | Effort |
|---|---|---|---|---|
| 4.1 | **Track ground-truth adapter** | Loads the road with `road.load(path, seed)`, exactly like `world_spawner`. Builds the referee's track model: route polyline, per-section lane data, element list with zones and polygons. Verifies consistency with the spawned world using the existing render-cache fingerprint. | Easy | S–M |
| 4.2 | <a id="42-route-tracker"></a>**Route tracker** | Arc length `s`, current section, lap counter, per-sample lateral offset. Handles overlaid intersections (continuity) and repositioning after RC mode (re-acquire within 1 m). | Medium | M |
| 4.3 | **Vehicle model** | Footprint rectangle, 4 wheel contact points, front bumper point, from the car pose and the model constants. | Easy | S |
| 4.4 | **Lane monitor** | Wheels outside the right lane per sample. **Episodes** (start or end of a departure) with debounce, so one departure is one penalty. Exemption hooks for overtaking, barred areas and parking. | Medium | M |
| 4.5 | **Standstill detector** | `|v| < v_eps` for a minimum time. Measures the standstill duration in sim time. Used for stop lines, crosswalks, parking and the "no stop" checks. | Easy | S |
| 4.6 | **Event timeline and timers** | Sim-time timers (2 min / 5 min / 3 + 3 min). Start trigger (`/referee/start` service, or the start-box gate once EVT-01/02 exist). Penalties only after the car **has passed the starting line** (Reg. 7.1). | Easy | S |
| 4.7 | **Signal monitor** | Normalises brake, indicator, hazard and RC states with timestamps. | Easy (once the topics are known) | S |
| 4.8 | **Report and visualisation** | `/referee/events` (live). RViz markers for zones and the current element. JSON or Markdown report with seed, track, events, penalties and score (REF-12). | Easy | S–M |
| 4.9 | **Manual inputs** | Parameters for things the sim cannot observe: second attempt, WiFi enabled, base points, the commission's minimum distance. | Easy | S |

**Geometry interpretation that must be fixed once for all courses** (see [section 11](#11-open-questions)): when does a wheel count as "across" a marking? Is it when the tyre touches the line, when the tyre centre is over it, or when the whole tyre is beyond the line's outer edge? The referee should expose this as a single parameter `marking_crossing_mode`, defaulting to "tyre fully beyond the line", which is what referees can reliably see on a real track.

---

## 5. Free Drive Course

**Task (Reg. 7.3):** Cover the greatest possible distance within **2 minutes** on an obstacle-free track. Intersections are crossed straight and signs are ignored. Score = (distance − penalties) × multiplier, then scaled relative to the best team.

### 5.1 Scoring rules and how to automate them

| Rule | Points | How the referee detects it | Difficulty |
|---|---|---|---|
| **Covered distance** | Base of the score | Forward progress of the route arc length `s` (sum of positive Δs across laps) between the start signal and +120 s of sim time. An odometer (integrated path length) is the alternative, but it rewards zig-zagging. Arc-length progress is closer to how referees measure distance on the track. | Easy |
| **Time limit 2 min** | – | Sim-time timer from the start signal. | Easy |
| **Leaving the right lane** | −5 m, ∞ | Lane monitor (4.4): an episode starts when ≥ 2 wheels are outside the right lane and ends when ≤ 1 wheel is outside, with debounce. In Free Drive **all** centre lines are treated as dashed, but leaving the lane is still penalised. Parking zones count as normal road. | Easy–Medium |
| **Activation of RC mode** | −5 m, ∞ | Count rising edges of the RC state. | Easy once VEH-02 exists, otherwise **Blocked** |
| **Faulty brake-light activation** | −2.5 m, max 3 | Compare the brake-light state with the ground-truth longitudinal deceleration. "Expected on" = deceleration above `a_brake` for longer than `t_min`. "Faulty" = expected but off, or on while not decelerating, with a grace period. Max 3 caps the effect of a bad definition. | Code: Easy. Definition: **Hard** (thresholds not in the rules). **Blocked** by VEH-01. |
| **Driving in the wrong direction at an intersection** | −5 m, ∞ | When the car leaves an intersection's box, compare the exit arm with the expected arm (straight ahead). Count one penalty per intersection pass. | Medium |
| **Canceled or second attempt** | ×(−0.3) | Not observable. | **Manual** |
| **WiFi enabled** | ×(−0.5) | Not observable. | **Manual** |
| **Relative scoring vs. best team** | – | Cannot be done from a single run. Report the absolute value; compare it between stack versions or against a stored reference run. | n/a |

### 5.2 Problems to expect

- **Track shape:** in Free Drive every intersection is crossed **straight**. The section chain is the designated route, so Free Drive tracks must be built with `Intersection(turn=STRAIGHT)` only (for example figure-eight layouts). The `default_road` uses `turn=RIGHT` and cannot be used for Free Drive scoring without a separate "Free Drive route".
- **Wrong turn and the arc length:** if the car turns at an intersection, it ends up on another part of the route. The arc length jumps, and the continuity logic in the route tracker would at first reject the jump. The tracker must detect this case (car far from the expected section, close to another one), apply the 5 m penalty and **re-anchor**. The rules do not say how distance counts afterwards; follow the route from the new position.
- **Steering scale:** `CLAUDE.md` notes that the simulated steering does not match the real car (degrees fed into a yaw-rate field). Lane-departure counts in the simulation will therefore not match the real car until the vehicle model is fixed (FIX-13). This is not a referee problem, but it limits how meaningful the scores are.
- **Brake light:** "faulty activation" is the only rule here that needs interpretation. It also needs an interface that does not exist yet.

### 5.3 Verdict

**Very feasible.** Free Drive is the best first target: one timer, one progress measure and four penalty types, three of them pure geometry. With the shared foundation in place, the Free Drive scoring itself is **S** of extra effort. It can be fully automated except for the brake-light definition and the manual multipliers.

---

## 6. Obstacle Evasion Course

**Task (Reg. 7.4):** **3 laps within 5 minutes.** Every track element passed is evaluated as positive, neutral or negative and **evaluated once per lap**. Fixed base points plus extra base points per successful lap, plus the element points, plus the additional penalties. The attempt is only valid after **at least one full lap**.

This course is by far the hardest to referee automatically. It has the most rules, most rules are state machines rather than single checks, and many rules involve moving actors that do not exist yet.

### 6.1 Lap, time and base points

| Rule | How | Difficulty |
|---|---|---|
| 5 min limit, end after 3 laps | Sim-time timer; route tracker lap counter | Easy |
| **Successful lap** = start line passed "without skipping any elements or sections (i.e. by leaving the right lane)" | Lap valid if `s` increased monotonically through every section **and** every element of the lap has an evaluation record. A big jump in `s` (shortcut) invalidates the lap. | Medium |
| Base points, extra base points per lap | Parameters (the commission defines them per track). | Manual (parameter) |
| Attempt valid only after ≥ 1 full lap; "the commission will decide" if the car left the track | Automatically valid if ≥ 1 valid lap. Otherwise produce a **"needs review"** flag instead of a decision. | Medium (cannot be fully automated) |
| Each element evaluated once per lap | Evaluation records keyed by `(element_id, lap)`. Elements are created from the track model, so every lap has a fixed list. | Easy |
| Skipped element → neutral | An element whose window was not passed in sequence gets "neutral (skipped)". | Easy |

### 6.2 Element evaluations

Each element is a small state machine with an **activation window** on the route (for example from 1.5 m before to 2.5 m after the element). The window is opened and closed by the route tracker.

| Element (points +/0/−) | Detection approach | Difficulty | Dependencies / problems |
|---|---|---|---|
| **Static obstacle** (+10 / 0 / −10) | *Positive:* the car's rear passes the obstacle's rear arc length without footprint overlap, then the car is **back in the right lane within 2 m** (measured along `s` from the obstacle's rear). *Neutral:* merge distance > 2 m. *Negative:* footprint overlap. | Medium | Obstacles that sit in the left lane or beside the track need a decision: are they evaluated at all? (open question). The planned obstacle pose is wrong after a push (FIX-10). |
| **Dynamic obstacle** (+15 / 0 / −10) | Like the static obstacle, but against the live actor pose: the relative arc length changes from "behind" to "ahead". *Neutral* also if passed inside an intersection (the crossing happens while the car is inside an intersection box) or if never passed. | Medium | **Blocked** by DYN-02 (actor + pose topic). |
| **Obstacle at an intersection** (+10 / 0 / −10) | When the car reaches the intersection, check whether the actor is inside the **Abb. 6 area** and has priority. *Positive* if the car did not enter the intersection box before the actor had completely left it. *Neutral* if the car entered early. *Negative* on collision. | **Hard** | **Blocked** by DYN-03. The exact Abb. 6 area must be modelled. The scenario controller and the referee must agree on who had priority, so the controller should publish its priority decision as an event the referee consumes. |
| **Oncoming obstacle at a barred area** (+5 / 0 / −10) | *Positive* if the car stayed in its lane until the oncoming actor passed the barred area. Exception: the car had already "started a passing manoeuvre" (≥ 2 wheels in the left lane) before the actor approached. *Neutral* if the car went into the left lane while the actor had right of way. | **Hard** | **Blocked** by DYN-04. "Approaching" and "already started" need thresholds. The actor's evasive manoeuvre (Reg. 7.4) must not be scored as a collision. |
| **Drive straight, stop / give-way intersection** (+5 / 0 / −5) | Standstill detector inside the stop zone. *Positive:* front bumper **not across** the line, **≤ 150 mm** before it, stop **≥ 3 s** (stop line) or **≥ 1 s** (give-way line), then exit straight. *Neutral:* too far away, too short, or turned. *Negative:* no standstill at all at a **stop** line. | Medium | Missing lines in several configurations (FIX-02, FIX-03). The stop and line-crossing measures depend on the footprint's front edge (section 2). Which edge of the 36–40 mm line counts? |
| **Drive straight, priority-to-right** (+10 / 0 / –) | As above, with the 1 s give-way rule. No negative case. | Medium | – |
| **Mandatory turn, stop / give-way** (+15 / 0 / −5) | As above, plus *correct exit arm* = the arm defined by `Intersection.turn`. A wrong turn **includes driving straight** wrongly. | Medium | Same as above. |
| **Mandatory turn, priority-to-right** (+15 / 0 / –) | As above. | Medium | Give-way lines missing for turning cases today (FIX-02). |
| **Parking** (+20 / 0 / −10) | *Attempt start:* the car leaves the right lane towards a parking lot inside a `ParkingArea`. *Valid:* full stop, then **all four indicators** flash at least once. At the first flash, **all four wheel points inside one `FREE` spot polygon**. *Neutral:* not inside, no attempt, or more than one attempt per lap. *Negative:* footprint overlap with a parking obstacle (also invalidates the attempt). Also: the indicator side at the start of the manoeuvre, the speed in the lot ≤ the adjacent road's limit, and leaving the lot's outer boundary counts as a lane departure. | **Hard** | **Blocked** by VEH-01 (hazard and indicator states). "Start of a parking attempt" is not defined precisely. Parallel spots are bounded by obstacles, so their usable length depends on obstacle placement (ELEM-05). |
| **No-passing zone** (+5 / 0 / −10) | Zone = sections whose centre line is solid on the ego side. *Negative:* > 1 wheel across the solid centre line. *Neutral:* distance to the dynamic obstacle < 300 mm (bumper to bumper along `s`). *Positive* otherwise. | Easy (line part) / Medium (distance part) | Distance part **blocked** by DYN-05. Without an obstacle, is the zone still worth +5? (open question). An overtaking manoeuvre started **before** the zone may be completed. |
| **Barred area** (+10 / 0 / –) | Footprint (or wheels) overlap with `BlockedArea.frame` → neutral; otherwise positive. Leaving the right lane while bypassing is **exempt** from the lane penalty, but the car must be back within 2 m. | Easy | Only right-lane barred areas exist today (FIX-06). |
| **Crosswalk with pedestrian** (+15 / 0 / −10) | Stop ≤ 150 mm before the crosswalk for ≥ 3 s. Do not move on until the pedestrian state is `cleared`. Collision with the pedestrian → −10. | Medium | **Blocked** by ELEM-07 and DYN-07. The pedestrian must only start after the car has stopped, which is scenario logic, not referee logic. |
| **Crosswalk without pedestrian** (+5 / 0 / –) | *Positive* if no standstill inside the crosswalk window. | Easy | – |

### 6.3 Additional penalties

| Violation | Points, cap | How | Difficulty |
|---|---|---|---|
| Second attempt | −0.5 × base | Not observable | **Manual** |
| Active WiFi connection | −0.5 × base | Not observable | **Manual** |
| **Exceeding the speed limit** | −5, ∞ | Zones from `SpeedLimit` (scale 1:10). One violation episode = speed > limit + tolerance for ≥ `t_min`; a new episode only after the speed drops below the limit. Also applies in parking lots (adjacent road's limit). | Code: Easy. Definition: **Medium** (tolerance and episode definition are not in the rules). |
| **Activation of RC mode** | −5, ∞ | Rising edges of the RC state. | Easy, **Blocked** by VEH-02 |
| **Leaving the right lane** | −2, max 10 | Lane monitor **with exemptions**: overtaking an obstacle (window from the start of the lane change until ≤ 2 m after the obstacle), bypassing a barred area, and the parking manoeuvre. Outside these windows: penalty. | **Medium–Hard**: the exemption windows decide everything. Too generous and real departures are missed; too strict and legitimate overtakes are penalised. |
| **Collision with a road sign** | −5, ∞ | Footprint overlap with sign collision boxes (episode-based: one contact = one penalty). | Easy |
| **Falsely using turn indicators** | −2, max 10 | Expected indicator events come from the referee's own manoeuvre detection: lane change out (left), lane change back (right), mandatory turns (turn side), parking start (spot side), parking end (all). A violation is a missing, wrong-side or unmotivated indicator. | **Hard**: needs timing windows (how early before a lane change?) and a decision on whether "missing" counts as "falsely using". **Blocked** by VEH-01. |

### 6.4 RC-mode rule for elements

Reg. 7.4 (RC-Mode): an element is evaluated **from the behaviour before RC mode** was activated. After RC mode, the element is only evaluated further if the activation was **not caused by incorrect behaviour related to that element**.

- The first half is easy to automate: freeze the element's state at the RC rising edge.
- The second half needs **causal attribution**. A heuristic works for most cases: if the element already has a failure recorded, or the car was inside the element's window with the lane monitor active, then attribute the activation to the element. Otherwise continue evaluating. The referee should mark every element touched by an RC activation as **"needs review"** instead of pretending to be certain.
- Difficulty: **Hard**. Effort: **M**.

### 6.5 Problems to expect

1. **Most points depend on things that do not exist yet.** Dynamic obstacles (+15), right of way at intersections (+10), oncoming traffic (+5), pedestrians (+15) and parking (+20, needs indicators) are the highest-value elements, and all are blocked. Without them the referee only covers the static part of the course.
2. **Referee and scenario controller must share the truth.** For right-of-way situations, the controller decides when the actor drives. The referee has to know the same facts: was the actor in the Abb. 6 area, did it have priority, had it cleared? Duplicating this logic in two places will drift. The controller should publish structured events (`actor_entered_priority_area`, `actor_cleared_intersection`) that the referee consumes.
3. **Per-lap resets.** Elements are evaluated again every lap, so actors (pedestrian, dynamic obstacles) must be reset before the car arrives on the next lap. Otherwise laps 2 and 3 are not comparable (EVT-04).
4. **Interpretation gaps** that a human referee resolves on the spot: "successfully passed", "started a passing manoeuvre", "parking attempt", "falsely using indicators", "faulty brake light". Each needs an explicit threshold in the YAML. The referee should report its thresholds in the result so a score is never mistaken for an official one.
5. **Overlaid intersections:** each physical crossing appears twice in the chain (once per pass). Each pass is a separate element evaluation, which matches the rules ("each element once per lap" applies per pass of the designated route). The route tracker must never confuse the two passes.
6. **Elements are only as correct as the track.** Missing stop lines (FIX-02/03), barred areas only on the right (FIX-06), and crosswalk dimensions (FIX-05) make some tables impossible to evaluate as written. Fix the track generator first (gap analysis phase 1).
7. **"Commission decides" cases** (validity after leaving the track, RC misuse, disqualification) can never be decided automatically. Output flags, not verdicts.

### 6.6 Verdict

**Feasible, but the largest effort, and only partially useful until dynamic actors and the light/RC interfaces exist.**

- **Phase 1, automatable today (M–L):** laps, timer, static obstacles, the four intersection tables without traffic, barred area, crosswalk without pedestrian, no-passing line crossing, speed limits, sign collisions, lane departure with exemptions.
- **Phase 2, after VEH-01/02 (M):** parking, indicators, RC-mode rules.
- **Phase 3, after DYN-02 … DYN-07 (M–L):** dynamic obstacles, intersections with traffic, oncoming traffic, crosswalk with pedestrian, no-passing following distance.

Total referee effort for this course: **L (about 2–4 weeks)**, excluding the actors themselves.

---

## 7. Navigation Course

**Task (Reg. 7.5):** a 3-minute **mapping phase**, then the car is placed back in the start box, then a 3-minute **navigation phase**. In the navigation phase the car visits **landmarks 0, 1, 2, …** in ascending order over the **shortest distance**. There are no obstacles or pedestrians. Signs and mandatory turns are ignored, and a **designated route is not guaranteed**.

- Primary rank: the number of landmarks successfully visited.
- Secondary: `Score = BasePoints · Multiplier · MinDistance / (Distance + ΣPenalties)`.

### 7.1 Scoring rules and how to automate them

| Rule | How | Difficulty |
|---|---|---|
| **Phases and timers** (3 min + 3 min; navigation ends early when all landmarks are visited) | Phase events from a start scheduler (EVT-02) or a referee service. Scoring only in the navigation phase. | Easy |
| **Landmark successfully visited** (fully passed both signs; all lower IDs visited before; out-of-order passes do not count; passes in RC mode do not count) | Each landmark gets a **gate line** between its two signs across the road. "Fully passed" = the whole footprint has crossed the gate (the rear edge is past it), in **either** direction. Keep a `next_expected_id`; a crossing counts only if `id == next_expected_id` and RC is off. | Easy, but **Blocked** by ELEM-08 (landmarks do not exist) |
| **Distance** | Odometer: integrated 2D path length from ground truth during the navigation phase. Use a small standstill threshold so jitter does not add distance. Here (unlike Free Drive) the real driven distance is what the rules mean. | Easy |
| **Leaving the right lane** (+1 m, ∞) | "Right lane" depends on the **direction of travel** on **any** road. Compare the car heading with the section direction: if aligned, the right lane is `d ∈ [−w, 0]`, else `d ∈ [0, w]`. Inside intersections every turn is allowed, so check only the intersection box and the entry and exit lanes. | **Medium** (on a single chain), **Hard** on tracks with branches (see 7.2) |
| **Activation of RC mode** (+1 m, ∞) | Rising edges. | Easy, **Blocked** by VEH-02 |
| **Multiplier** (second attempt −0.3, WiFi −0.5) | Not observable. | **Manual** |
| **Minimum distance** (defined by the commission) | Option 1: a parameter (simple, matches the rules). Option 2: compute it as the shortest route through landmarks 0 → N on a **road graph** (Dijkstra between consecutive gates, respecting the driving direction). | Parameter: Easy. Computed: **Medium**, needs the road graph. |
| **Score formula and ranking** | Direct formula. Report landmarks first, score second. | Easy |

### 7.2 Problems to expect

1. **The road DSL cannot describe typical Navigation tracks.** `Road` is a **single chain** of sections that follows one route. Reg. 7.5 says a designated route is **not guaranteed**, and Abb. 25 shows a network with branches. Roads that are not part of the single loop (for example an extra connection between two intersections, or a dead end at a T-intersection) have **no geometry** in the current model. Without that geometry the referee cannot check lanes there. This is primarily a track-generation problem: the DSL needs multiple chains or a branch mechanism. It then needs a **road graph** builder that merges overlaid intersections into one node and splits chains at intersections. Effort: **L**.
2. **Landmarks are missing** (ELEM-08). This blocks the core scoring rule.
3. **Mapping → navigation reset.** The car must be put back into the start box **without** restarting the simulation or the stack (EVT-05, a teleport service). The referee must reset its route tracker on that teleport and not count the jump as distance.
4. **Direction-aware lane check at intersections.** Every turn is allowed, so a lane polygon for each possible turn path (or a relaxed rule inside the box) is needed.
5. **MinDistance consistency.** If the referee computes `MinDistance` itself, the result depends on modelling choices (centre of the right lane? which turn radius at intersections?). Teams comparing against the commission's number need the same convention. The safest default is a parameter, with the computed value as a suggestion.

### 7.3 Verdict

**Feasible, and the scoring logic is the simplest of the three courses**: few rules, mostly counting and an odometer. The problems are in the prerequisites: landmarks, the start-box reset, and above all a road model that can represent a road **network** instead of a single loop. On a single-loop track with landmarks the referee is **M** of effort. Full support for arbitrary navigation tracks is **L**, mostly for the road graph.

---

## 8. Cross-cutting problems and risks

| # | Problem | Impact | Mitigation |
|---|---|---|---|
| 1 | **Rules leave thresholds open** (marking crossing, standstill speed, stop-line edge, speed tolerance, brake-light deceleration, indicator timing, "attempt" definitions) | Scores depend on choices that real referees make by eye. | One YAML with every threshold. Print the thresholds in every report. Agree on them in the team. Ask the commission where possible. |
| 2 | **Seed determinism** | The referee and `world_spawner` must build the **identical** road. Roads with `use_seed=False` produce different geometry in every process. | Load the road with the same seed. Compare the render-cache fingerprint. Refuse to score `use_seed=False` roads, or let the spawner publish the generated geometry (latched topic / JSON file). |
| 3 | **Sim time vs. wall time** | The RTF can be ~30 %. Wall-clock timers would make the 2 min limit about 6 min of sim time, and stop durations would be wrong. | Use only `/clock` and message header stamps. Never use `time.time()` in the core. |
| 4 | **Dropped samples (online)** | Missed samples could hide a short lane departure or a stop. | Process samples in timestamp order with a sufficient QoS depth (for example 100). Log a warning when a gap between stamps is larger than two publish periods. |
| 5 | **Teleports** (RC repositioning, mapping → navigation, manual reset in Gazebo) | Jumps in arc length or distance; false lap counts. | Teleport detection (position jump > x m within one sample) → re-acquire the route position and flag it in the report. |
| 6 | **Vehicle model fidelity** | Steering scale (CLAUDE.md gotcha) and model constants (FIX-13) differ from the real car, so simulated scores do not predict real scores. | Out of scope for the referee, but document it next to every score. |
| 7 | **Footprint accuracy** | The 150 mm stop rule and collisions depend on the real front-bumper and width values. | Measure the real Smarty and put the dimensions in the referee config, not hard-coded. |
| 8 | **Unknown stack interfaces** (lights, RC mode, buttons) | Several rules are blocked. | Clarify the topics with the stack owners (gap analysis, open question 1). |
| 9 | **"Commission decides" rules** | Cannot be automated. | Flags in the report ("needs review"), never silent decisions. |
| 10 | **Rule changes every year** | Logic written for 2026 goes stale. | Point tables per year in YAML. Unit tests per rule with synthetic trajectories. |
| 11 | **False confidence** | An automatic score looks official. | Label the output as an *estimate* under stated assumptions. Keep a per-event log so every point can be traced to a timestamp. |

---

## 9. Performance impact on the simulation

**Short answer: the referee does not draw meaningful performance from the simulation if it uses the 50 Hz odometry instead of `/tf` and does not render anything.**

The estimates below are orders of magnitude and should be confirmed with the existing `PERF` logging approach once a prototype exists.

| Component | Cost estimate | Comment |
|---|---|---|
| Camera rendering and bridging (existing) | **Dominant** | A 2064 × 1544 wide-angle camera at 30 Hz is ~9.5 MB per raw RGB frame, ~290 MB/s through `ros_gz_bridge`. |
| Physics at 1 ms steps (existing) | High | Sets the RTF together with the camera. |
| Referee core per sample | ≈ 0.1–1 ms in Python | 4 wheel projections onto the current section line, a few point-in-polygon tests for **nearby** elements only (spatial index or the route tracker's current window), and small state machines. |
| Referee at 50 Hz | **≈ 0.5–5 % of one core** | rclpy callback overhead is of the same order as the evaluation itself. |
| Subscribing to `/tf` at 1 kHz instead | Noticeably more, about 5–15 % of a core | Python deserialisation of ~10 link poses per step. **Avoid it**: the wheel positions follow from the odometry pose. |
| Contact sensors (optional) | Small | Physics computes contacts anyway; the contact system only publishes them. Only one sensor on the car, not one per obstacle. |
| RViz markers | Small | Publish static zones once (transient-local); publish dynamic markers at ≤ 5 Hz. |
| gz plugin (Option B) | Small, but **inside the physics loop** | Any slow path directly lowers the RTF. |

**Low RTF helps the referee.** All decisions use message timestamps, so a slow simulation does not reduce accuracy. It only gives the referee more wall time per simulated second.

**Indirect cost:** dynamic actors (DYN-02) and their controllers will cost more than the referee, especially if they move through physics instead of pose updates. The gap analysis already recommends pose-driven actors.

---

## 10. Suggested implementation order

| Step | Content | Prerequisites | Effort |
|---|---|---|---|
| 1 | **Core foundation**: track adapter, route tracker (port from `feature/labeling`), vehicle model, lane monitor, standstill detector, sim-time timers, YAML thresholds, JSON report, unit tests with synthetic trajectories | – | M–L |
| 2 | **Free Drive scoring** (distance, lane departure, wrong direction, RC count) and a Free Drive example track with straight intersections only | Step 1 | S |
| 3 | **Live ROS node**, `/referee/events`, RViz markers, start service | Step 1 | S–M |
| 4 | **Obstacle Course, static part**: laps, static obstacles, intersection tables, barred area, crosswalk without pedestrian, no-passing line, speed limits, sign collisions, lane exemptions | Step 1; FIX-01 … FIX-06 recommended | M–L |
| 5 | **Signals**: brake light (Free Drive), indicators, parking, RC-mode rules | VEH-01, VEH-02 | M |
| 6 | **Obstacle Course, dynamic part**: dynamic obstacles, right of way, oncoming traffic, pedestrian crosswalk, following distance | DYN-02 … DYN-07 | M–L |
| 7 | **Navigation Course**: landmarks, gate crossings, odometer, phases, direction-aware lane check, MinDistance as a parameter | ELEM-08, EVT-05 | M |
| 8 | **Road graph** for branched navigation tracks and computed MinDistance | DSL extension (multiple chains / branches) | L |

Steps 1–4 give a useful regression tool (Free Drive plus the static Obstacle Course) after roughly **2–3 weeks** of work.

---

## 11. Open questions

These should be decided by the team, and where marked, asked of the commission, before the referee is implemented:

1. **Marking crossing:** when is a wheel "across" a lane marking? Touching the line, its centre over the line, or fully beyond it? *(commission)*
2. **Stop-line reference:** is the 150 mm distance measured to the near edge of the 36–40 mm line? Does "not crossing" mean the front bumper must not reach the near edge? *(commission)*
3. **Standstill definition:** which speed counts as stopped (for example < 0.01 m/s), and for how long is a "stop" without the required duration still a stop (relevant for −5 vs 0 at stop lines)?
4. **Obstacles not in the ego lane:** are static obstacles in the left lane or beside the track evaluated (+10) at all? *(commission)*
5. **No-passing zone without an obstacle:** is it still evaluated (+5)? *(commission)*
6. **Indicator rules:** how early before a lane change must the indicator be on, and does a *missing* indicator count as "falsely using"? *(commission)*
7. **Brake light:** which deceleration counts as "active braking"? *(commission)*
8. **Speed-limit violations:** tolerance and episode definition (one violation per zone, per second, or per exceedance)?
9. **Navigation penalties in the mapping phase:** do lane departures or RC activations during mapping count? The scoring section only mentions the navigation phase. *(commission)*
10. **Interfaces:** which topics and message types does the Smarty stack use for brake light, indicators, hazard and RC mode? (Same as open question 1 in the gap analysis.)
11. **Base points and minimum distance:** should the referee use fixed per-track parameters, or compute an estimate from the track?
