# CAuDri Regulations 2026: Simulation Gap Analysis

This document compares the **CAuDri-Challenge Regulations 2026** (source snapshot `regulations-2026-05-06`) with the current state of this simulation package (branch `check-ruleset`, based on `jazzy` @ `e792652`). It lists what is **missing**, **incomplete**, or **not compliant with the rules** and should be added or fixed, so the simulation can serve as a realistic training and test environment for all three dynamic events.

Every item has an ID (for example `FIX-03` or `ELEM-07`) so it can be used directly as an issue or ticket title.

---

## Table of Contents

1. [How to read this document](#1-how-to-read-this-document)
2. [Executive summary](#2-executive-summary)
3. [Coverage matrix](#3-coverage-matrix)
4. [Bugs and rule deviations in existing features (FIX)](#4-bugs-and-rule-deviations-in-existing-features-fix)
5. [General track layout and lane markings (TRK)](#5-general-track-layout-and-lane-markings-trk)
6. [Track elements (ELEM)](#6-track-elements-elem)
7. [Obstacles and other traffic participants (DYN)](#7-obstacles-and-other-traffic-participants-dyn)
8. [Vehicle-side features: lights, RC mode, buttons (VEH)](#8-vehicle-side-features-lights-rc-mode-buttons-veh)
9. [Competition procedure: start box, start scheduling, events (EVT)](#9-competition-procedure-start-box-start-scheduling-events-evt)
10. [Automatic referee and scoring (REF)](#10-automatic-referee-and-scoring-ref)
11. [Track validation and tooling (TOOL)](#11-track-validation-and-tooling-tool)
12. [Suggested roadmap](#12-suggested-roadmap)
13. [Open questions](#13-open-questions)
14. [Appendix: key dimensions from the regulations](#14-appendix-key-dimensions-from-the-regulations)

---

## 1. How to read this document

**Status symbols**

| Symbol | Meaning |
|---|---|
| ✅ | Present and compliant with the rules |
| ⚠️ | Present, but incomplete or deviating from the rules |
| ❌ | Missing |

**Priority**

| Priority | Meaning |
|---|---|
| **P1** | Needed to practise core scoring situations of the Obstacle Evasion Course or Free Drive, or a bug that makes generated tracks non-compliant. |
| **P2** | Needed for full event coverage, including the Navigation Course and the procedure around the start. |
| **P3** | Realism and robustness improvements, or rules that become mandatory only in 2027. |

**Effort** (rough): **S** < 1 day, **M** 1–3 days, **L** > 3 days.

**Regulation references** use the chapter and section numbering of the compiled regulations PDF (for example *Reg. 5.3 Intersection*). Appendix figures are referenced by their file name (for example *Abb. 6* = `Abb_6_intersection_give_way.pdf`).

---

## 2. Executive summary

The simulation already has a solid **static road-generation DSL** (straights, arcs, Bézier curves, 4-way intersections, parking areas, crosswalks, barred areas, speed limits) plus the matching sign meshes. Compared with the 2026 rules, it is missing the following:

1. **Everything that moves.** There are no dynamic obstacles and no pedestrians. These are central to the Obstacle Evasion Course: dynamic obstacle passing, right of way at intersections, oncoming traffic at barred areas, following in no-passing zones, and crosswalks with pedestrians.
2. **Several track elements:** landmarks (the Navigation Course cannot be practised at all without them), the start box with its gate, T-intersections, side-road junctions, expressway and no-passing-zone sections, slopes, and automatic guide signs at sharp turns.
3. **Variability required by the rules:** variable lane width (350–450 mm), partially missing markings (≤ 1 m), artifacts outside the track, and rounded intersection corners.
4. **Vehicle-side features the referees score:** brake lights, turn indicators, the RC-mode light and RC-mode behaviour, and event-selection buttons. None of these are represented in the simulation.
5. **An automatic referee.** There is no lap, time, penalty, or per-element evaluation. With one, the simulation could report a score per event, as the real referees would.
6. **Rule deviations in existing code.** Sign distances at intersections are out of range because of a formula bug, several marking dimensions are wrong, give-way and stop lines are missing for turning intersections, and the shipped `default_road` itself breaks the "missing marking ≤ 1 m" rule.

---

## 3. Coverage matrix

| Regulation topic | Reg. section | Status | Items |
|---|---|---|---|
| Two-lane closed loop, start line | 5.1 | ⚠️ start line only inside `ParkingArea` | TRK-08 |
| Designated route, laps | 5.1.1 | ❌ | TRK-08, REF-02 |
| Lane width 350–450 mm (variable) | 5.1.2 | ⚠️ fixed at about 380 mm | TRK-01 |
| Center line: dashed / double solid / mixed | 5.1.2 | ✅ | – |
| Side road junction (100/50 mm dashed edge) | 5.1.2 | ❌ | TRK-02 |
| Missing markings ≤ 1000 mm | 5.1.2 | ⚠️ whole sections only | TRK-03, FIX-12 |
| Slopes ≤ 10 % with signs | 5.1 | ❌ signs exist, no geometry | TRK-04 |
| Artifacts outside the track | 5.1 | ❌ | TRK-05 |
| Traffic sign positions (75–125 mm lateral) | 5.1.3 | ✅ / ⚠️ intersection distances wrong | FIX-01 |
| Guide signs for sharp turns | 5.1.3 | ❌ meshes exist, no placement | TRK-06 |
| Static obstacles | 5.2.1 | ⚠️ no validation, can be pushed | DYN-01 |
| Dynamic obstacles (≤ 0.6 m/s) | 5.2.2 | ❌ | DYN-02 … DYN-06 |
| Intersection 4-way, 70–90° | 5.3 | ✅ | – |
| Intersection 3-way (T) | 5.3 | ❌ `closing` is ignored | ELEM-01 |
| Rounded corners r ≈ 100 mm | 5.3 | ❌ | ELEM-02 |
| Stop / give-way / priority-to-right markings | 5.3 | ⚠️ missing for turning cases | FIX-02, FIX-03 |
| Mandatory turn (sign, arrow, turn lines) | 5.3 | ⚠️ arrow too short | FIX-04 |
| Parallel and perpendicular parking | 5.4 | ⚠️ no rule-compliant layout helpers or validation | ELEM-05, FIX-09 |
| No-passing zone | 5.5 | ⚠️ markings and signs exist, no section or helper | ELEM-04 |
| Barred area | 5.6 | ⚠️ stripe width wrong, right lane only | FIX-06, ELEM-06 |
| Crosswalk | 5.7 | ⚠️ length and signs deviate | FIX-05 |
| Pedestrians and waiting areas | 5.7 | ❌ | ELEM-07, DYN-07 |
| Two-lane expressway | 5.8 | ❌ signs exist, no section | ELEM-03 |
| Speed limit zones | 5.9 | ⚠️ marking dimensions and font | FIX-07 |
| Landmarks (QR, IDs 0–9) | 5.10 | ❌ | ELEM-08 |
| Start box, gate, QR code | 6.2.3 | ❌ | EVT-01 |
| Start scheduling system | 6.2.2 | ❌ | EVT-02 |
| Vehicle lights and indicators | 4.1.7 | ❌ | VEH-01 |
| RC mode | 4.3 | ❌ | VEH-02 |
| Event selection buttons | 4.1.9 | ❌ | VEH-03 |
| Free Drive / Obstacle / Navigation event flow | 7.3–7.5 | ❌ | EVT-03 … EVT-06 |
| Scoring and penalties | 7.3–7.5 | ❌ | REF-01 … REF-12 |

---

## 4. Bugs and rule deviations in existing features (FIX)

These are quick wins. They make tracks that are generated today non-compliant, so they should be fixed before new features are built on top of them.

### FIX-01: Interval helper formula is wrong, so intersection sign distances are out of range (P1, S)

- **Where:** [config.py:17-45](../simulation/groundtruth/road/config.py#L17-L45), used in [intersection.py:584-618](../simulation/groundtruth/road/sections/intersection.py#L584-L618)
- **Problem:** All `get_*` helpers compute `(lo + hi) * rand` instead of `lo + (hi - lo) * rand`. With the default `rand=0.5` this happens to give the midpoint, but `get_prio_sign_dist(1)` (used for **all** priority, stop and yield signs) returns **0.8 m** instead of 0.5 m. On top of that, `cp_sign_*` adds a hard-coded `0.1` ("TODO remove magic number", [intersection.py:194](../simulation/groundtruth/road/sections/intersection.py#L194)). As a result, intersection priority signs end up about **0.9 m** before the intersection. The rules require **300–500 mm** (Abb. 5, 12, 14–16, 26).
- **To do:**
  - Fix the interpolation formula.
  - Draw distances from the seeded `random` within the regulation interval, so tracks vary but stay reproducible with the seed.
  - Remove the magic `0.1` and use the real sign footprint.
  - Add a unit test that asserts every generated sign distance is inside its interval.

### FIX-02: Priority-to-right with a mandatory turn has no give-way lines (P1, S)

- **Where:** [intersection.py:674-700](../simulation/groundtruth/road/sections/intersection.py#L674-L700)
- **Problem:** Give-way lines on all entries are only drawn for `rule == EQUAL and turn == STRAIGHT`. Reg. 5.3 states that at priority-to-right intersections **all entries are marked with a give-way line**, independent of the turn direction. The scoring table *Mandatory Turn Priority-to-Right Intersection* relies on these lines.
- **To do:** Draw give-way lines on all (3 or 4) entries for `EQUAL`, whatever the turn direction.

### FIX-03: Priority intersections with a turn have no stop or give-way lines on the crossing road (P1, S)

- **Where:** same block as FIX-02.
- **Problem:** For `PRIORITY_YIELD` and `PRIORITY_STOP`, the west and east lines are only added when `turn == STRAIGHT`. In Abb. 16 (mandatory turn, ego has right of way) the crossing road still carries its stop signs and lines.
- **To do:** Add the crossing-road lines independent of `turn`. Compare the result with Abb. 14, 15, 16 and 26.

### FIX-04: Turn-arrow surface marking has the wrong size (P2, S)

- **Where:** [config.py:4-5](../simulation/groundtruth/road/config.py#L4-L5) and `models/meshes/surface_marking_turn_{left,right}.png` (72 × 500 px at 1 px = 1 mm)
- **Problem:** Abb. 17 (*Turn left ahead*) specifies **70 mm × 600 mm**. The simulation uses 72 mm × 500 mm.
- **To do:**
  - Regenerate both PNGs at 70 × 600 and update `TURN_SF_MARK_*`.
  - Check the left/right swap in [surface_markings.py:35-41](../simulation/groundtruth/road/renderer/surface_markings.py#L35-L41) (`"left" if kind != LEFT_TURN_MARKING else "right"`). It may be intended to compensate for the mirrored y axis, but it needs a visual check.
  - Check the 50–250 mm distance to the stop line: `get_surface_mark_dist()` is affected by FIX-01.

### FIX-05: Crosswalk length and signage deviate from the rules (P1, S)

- **Where:** [zebra_crossing.py:25-44](../simulation/groundtruth/road/sections/zebra_crossing.py#L25-L44)
- **Problem:**
  - The default `length` is **0.45 m**, but the stripes must be **400 mm** long (Reg. 5.7).
  - Only one crosswalk sign is spawned, for the ego direction. Abb. 9 shows a sign for **each** direction, 300–500 mm before the crosswalk.
- **To do:**
  - Set the default `length=0.4`.
  - Add the second sign for the opposite direction (left side, rotated by π).
  - Randomise the sign distance in 0.3–0.5 m.

### FIX-06: Barred area stripes are too thin and only the right lane is supported (P1, S)

- **Where:** [blocked_area.py](../simulation/groundtruth/road/sections/blocked_area.py) and [surface_markings.py:166-186](../simulation/groundtruth/road/renderer/surface_markings.py#L166-L186) / [surface_markings.py:331](../simulation/groundtruth/road/renderer/surface_markings.py#L331)
- **Problem:**
  - The diagonal stripes are drawn **20 mm** wide. The rules require **36–40 mm**.
  - The outline width is undefined: it inherits whatever line width the cairo context had before. The rules require 18–20 mm.
  - The polygon is always attached to the right lane boundary. Reg. 5.6 also allows barred areas **attached to the left lane boundary** (Reg. 7.4: "Barred areas located in the left lane can be passed without changing lanes").
  - The maximum length of **2000 mm** (measured along the outer marking) is not checked.
- **To do:**
  - Set the stripe width to 0.038 and set the outline width explicitly to 0.02.
  - Add a `side` parameter (`RIGHT`/`LEFT`).
  - Assert `length ≤ 2.0`.
  - Keep the 60° opening angle, the 27° stripe angle and the 150 mm gap, which are already correct.

### FIX-07: Speed-limit surface markings have the wrong dimensions and an unbundled font (P2, M)

- **Where:** [speed_limit.py:28-40](../simulation/groundtruth/road/sections/speed_limit.py#L28-L40) and [surface_markings.py:91-131](../simulation/groundtruth/road/renderer/surface_markings.py#L91-L131)
- **Problem:**
  - Abb. 17 specifies the **start marking** at about **400 mm long × 150 mm wide** (two digits, each 60 mm wide) and the **end marking** at **300 mm × 200 mm** with a cross. The simulation uses 400 × 250 for both.
  - The font `OpenDinSchriftenEngshrift` is not shipped with the repo. cairo silently falls back to a default font, so the digits do not look like the competition markings.
  - Abb. 17 also states that **all digits 0–9** can appear. Only the tens values 10–90 have meshes, which is sufficient for the signs.
- **To do:**
  - Bundle a DIN 1451 Engschrift-like font (with a compatible licence), or render the digits from vector paths.
  - Implement separate start and end dimensions.
  - Add a test that renders every limit.

### FIX-08: Start line only exists inside `ParkingArea`, and its width is ignored (P2, S)

- **Where:** [parking_area.py:233-265](../simulation/groundtruth/road/sections/parking_area.py#L233-L265) and [surface_markings.py:134-163](../simulation/groundtruth/road/renderer/surface_markings.py#L134-L163)
- **Problem:**
  - `draw_start_lane` always draws 3 rows × 20 mm = **60 mm** and ignores `start_line_length`. The rules require about **50 mm**.
  - The start line can only be created through `ParkingArea(start_line=True)`.
- **To do:** Add a standalone start-line element that can be attached to any straight section, and honour its length. See also TRK-08 and EVT-01.

### FIX-09: Parking sign position is not tied to the first parking area (P2, S)

- **Where:** [parking_area.py:275-283](../simulation/groundtruth/road/sections/parking_area.py#L275-L283)
- **Problem:** The `P` sign is always placed at `arc_length=0` of the section. Reg. 5.4 requires it **300–500 mm before the start of the first parking area**. The actual distance currently depends on `lot.start`.
- **To do:** Place the sign at `min(lot.start) - U(0.3, 0.5)`, or warn when `lot.start` leaves too little room.

### FIX-10: Static obstacles are not fixed and are not validated (P2, S)

- **Where:** [renderer/obstacle.py](../simulation/groundtruth/road/renderer/obstacle.py) and [sections/obstacle.py](../simulation/groundtruth/road/sections/obstacle.py)
- **Problem:**
  - Obstacles are dynamic bodies with a mass of 1 kg, so the car can push them away. The rules say obstacles "can be fixed to the ground".
  - Sizes are not validated: width 100–400 mm, height 100–240 mm, length ≥ 100 mm (Abb. 20).
- **To do:**
  - Add a `fixed: bool = True` option and emit `<static>true</static>` when it is set.
  - Validate the dimensions.
  - Use a matte white cardboard-like material.

### FIX-11: `Intersection.closing` is accepted but has no effect (P1, M)

- **Where:** [intersection.py:91-105](../simulation/groundtruth/road/sections/intersection.py#L91-L105)
- **Problem:** The docstring promises T-intersections, but `_closing` is never used. The rules allow intersections with three entries (Reg. 5.3, Abb. 26). See ELEM-01.

### FIX-12: The shipped `default_road` itself violates the rules (P2, S)

- **Where:** [default_road/road.py:47-65](../models/roads/default_road/road.py#L47-L65)
- **Problem:**
  - The right marking is missing over a full 90° arc with r = 2 m (about 3.1 m), and the left marking over about 2.4 m. The rules allow at most **1000 mm** of missing marking.
  - The parallel parking spots do not increase in length and do not end with a spot ≥ 700 mm.
- **To do:** Fix the example road once TRK-03 and ELEM-05 exist. Add the validator from TOOL-01 to CI so example roads stay compliant.

### FIX-13: Vehicle model parameters are inconsistent (P3, S)

- **Where:** [world.sdf:127-129](../models/world.sdf#L127-L129) and `models/smarty/model.sdf`
- **Problem:**
  - The AckermannSteering plugin uses `wheel_base 0.26` and `wheel_radius 0.035`. The model's joint positions give a wheelbase of 0.255 m and a wheel radius of 0.0345 m.
  - The steering-unit issue described in `CLAUDE.md` remains: an `Int16` in degrees is fed into a yaw-rate field.
- **Rule check:** The vehicle complies with Reg. 4.1.4: wheelbase ≥ 200 mm, track width ≥ 160 mm, width ≤ 300 mm, height ≤ 300 mm. The highest point is the camera at about 0.27 m.
- **To do:** Align the plugin with the model, or with measured values from the real car.

---

## 5. General track layout and lane markings (TRK)

### TRK-01: Variable lane width (350–450 mm) (P2, M)

- **Rule (Reg. 5.1.2):** Each lane is 350–450 mm wide, measured between the inside edges of the markings. The width may vary along the track, but the **outer markings must transition smoothly**.
- **Current state:** `Config.road_width = 0.4` is a global constant (middle line to the edge-line centre), which gives an inner lane width of about 380 mm everywhere.
- **To do:**
  - Add an optional per-section `lane_width` (or a start/end pair with smooth interpolation).
  - Make `left_line`, `right_line` and all element placements use the section's width instead of the global constant.
  - Make sure transitions between sections stay continuous: no lateral jumps in the outer lines.

### TRK-02: Side road junction marking (P2, S)

- **Rule (Reg. 5.1.2):** On straight sections, the outer boundary can be marked as a side-road junction: **100 mm dashes with 50 mm gaps**, at most **960 mm** long. It must still be treated as a solid line. There are no other markings.
- **Current state:** Not available. `DASHED_LINE_MARKING` always uses 200/200 for the outer lines.
- **To do:**
  - Add a new marking style `SIDE_JUNCTION_LINE_MARKING`, rendered with `dash_length=0.1` and `dash_gap=0.05`.
  - Add a helper `StraightRoad(..., side_junction=(start, length, side))` that splits the boundary line.
  - Assert `length ≤ 0.96` and that the section is straight.

### TRK-03: Partially missing markings (≤ 1000 mm) (P1, M)

- **Rule (Reg. 5.1.2):**
  - Any lane marking can be missing at **arbitrary locations** for at most **1000 mm**.
  - Outside intersections, no more than **two** markings are missing at the same time.
  - Turn lines inside intersections are **never** missing.
  - Parking zones have **no** missing markings.
- **Current state:** A line can only be missing for a **whole section** (`MISSING_LINE_MARKING`).
- **To do:**
  - Add a per-line list of gaps `[(start_arc_length, length), ...]` to `RoadSection`, and honour it in the renderer (draw sub-lines) and in `MarkedLine`.
  - Add an optional random generator, for example `road.add_random_marking_gaps(n, max_len=1.0, seed=...)`, that respects the "max 2 simultaneously" and "not in parking or turn lines" constraints.

### TRK-04: Slopes (uphill and downhill up to 10 %) (P3, L)

- **Rule (Reg. 5.1):**
  - Parts of the track can have slopes of up to 10 %.
  - Each slope is announced by an uphill or downhill sign placed ≥ 1000 mm before the change in slope.
  - No track elements are located on slopes.
- **Current state:** The sign meshes `uphill_grade_sign` and `downhill_grade_sign` (`RAMP_START`/`RAMP_END`) exist. The road is a single flat textured `<plane>` ([world_spawner.py:103-158](../simulation/world_spawner.py#L103-L158)), so slopes are impossible.
- **To do:**
  - Add a `Ramp` or `Slope` section with a height profile (flat, then up, plateau, down, flat). Keep transitions smooth so the car does not hit an edge.
  - Generate a road mesh, or a separate ramp model textured with the same PNG region, instead of a flat plane. A heightmap is an alternative.
  - Automatically place the grade signs at ≥ 1.0 m before each slope change.
  - Validation: no elements and no intersections on the sloped part.
- **Why it matters:** IMU pitch, camera perspective change (lane detection and the bird's-eye transform), and longitudinal control (holding speed uphill).

### TRK-05: Artifacts outside the track (P3, M)

- **Rule (Reg. 5.1):** The area outside the track is undefined. **Artifacts** (physical objects, remains of old lane markings) can be present, at least **100 mm** away from valid lane markings.
- **Current state:** The outside area is plain black.
- **To do:**
  - Add optional decoys rendered into the PNG: random partial white lines and old faded markings.
  - Add optional decoy 3D objects (boxes, cables, chairs, people's feet) at ≥ 100 mm from the track.
  - Make the floor outside the track configurable (grey hall floor or carpet texture) instead of pure black.
- **Why it matters:** Robustness of lane detection against false positives.

### TRK-06: Automatic guide signs for sharp turns (P1, S)

- **Rule (Reg. 5.1.3):** Curves with a radius **< 1200 mm** that follow a straight section **≥ 3 m** long are marked with:
  - a first (large) sign about **1500 mm before** the turn,
  - a second (large) sign at the **beginning** of the turn,
  - **small** signs repeated about every **400 mm** until the **apex**.
- **Current state:** The meshes `SHARP_TURN_LEFT/RIGHT(_SMALL)` exist, but nothing places them. They are not used anywhere in the code.
- **To do:**
  - Add a post-processing step in `Road` (for example `road.add_guide_signs()`, called automatically or explicitly) that scans for this pattern and places the signs on the outer side of the curve.
  - Also apply it to curves inside expressways (Reg. 5.8).

### TRK-07: Track surface and lighting realism (P3, S–M)

- **Rule (Reg. 5.1):** The track surface is "of a dark shade close to black" and the markings are white.
- **Current state:** Pure `rgb(0,0,0)` background ([road.py:143](../simulation/groundtruth/road/road.py#L143)) and fixed directional lighting.
- **To do:**
  - Use a slightly grey, noisy surface and slightly off-white, worn markings.
  - Make the lighting configurable (hall lights, glare and reflections on the surface, shadows) and use it for domain randomisation.

### TRK-08: Starting line, designated route and laps (P1, M)

- **Rule (Reg. 5.1, 5.1.1):**
  - A checkered starting line (about 50 mm) marks the beginning of the track.
  - The designated route starts at the starting line and follows all mandatory turns. Following it completes one lap and returns to the starting line.
- **Current state:** There is no explicit concept of a route or laps. `Road.close_loop()` exists, but the "closed loop" is only geometric.
- **To do:**
  - Add a standalone `StartLine` element (FIX-08).
  - Have `Road` expose the **designated route** as a polyline of the right-lane centre, including turn paths through intersections. The automatic referee needs it for lane checks, lap counting and the merge distance.
  - Validate that the route is closed. This does not apply to Navigation Course tracks, where a designated route is *not guaranteed* (Reg. 7.5).

---

## 6. Track elements (ELEM)

### ELEM-01: T-intersections (3 entries) (P1, M)

- **Rule (Reg. 5.3, Abb. 26):** Intersections have 3 or 4 entries and exits. Abb. 26 shows a T-intersection with a mandatory turn and give-way line.
- **To do:**
  - Implement `closing` (FIX-11): skip the line set of the closed arm, draw a continuous boundary line across it, and drop the signs and markings of that arm.
  - Validate that the ego `turn` does not point into the closed arm.
  - Add tests for every combination of `closing`, `turn` and `rule`. The hypothesis test already samples `closing` but does not pass it to the constructor ([intersection_test.py:47](../simulation/groundtruth/road/sections/test/intersection_test.py#L47)).

### ELEM-02: Rounded intersection corners (r ≈ 100 mm) (P2, S)

- **Rule (Reg. 5.3):** The lane boundaries at the corners can be connected by rounded transitions with a radius of about 100 mm.
- **Current state:** The corners are sharp: the outer lines simply end.
- **To do:** Add an optional `corner_radius` (default 0.1) and replace the corner end points with arcs that connect the neighbouring boundary lines.

### ELEM-03: Two-lane expressway section (P2, S)

- **Rule (Reg. 5.8):**
  - Mostly straight, planar, **≥ 5 m** long, one lane per direction.
  - No sharp turns; distinctive curves get guide signs.
  - Start and end signs.
  - The ego has right of way: no stop lines and **no obstacles**.
- **Current state:** The `EXPRESSWAY_START/END` sign meshes exist, but there is no section and the signs are not used.
- **To do:**
  - Add an `Expressway` composite section (or a `road.add_expressway(start_section, end_section)` helper) that places the start and end signs.
  - Validate length ≥ 5 m, no obstacles, no intersections, no stop lines, and minimum radius ≥ 1.2 m.
- **Why it matters:** It allows high-speed testing in the Free Drive Course.

### ELEM-04: No-passing zone section (P1, S)

- **Rule (Reg. 5.5, 7.4):**
  - Indicated by the start and end signs plus a solid center line on the driver's side: double solid, or mixed with the solid line on the driver's side.
  - Marking types persist for ≥ 1000 mm.
  - **No static obstacles** in the zone; at most one dynamic obstacle, driving in the right lane.
  - No other element for **2000 mm** after the zone.
- **Current state:** `DOUBLE_SOLID` and the mixed line styles plus the `NO_OVERTAKING_START/END` meshes exist, but no helper ties them together, and the signs are never placed.
- **To do:**
  - Add a `NoPassingZone` wrapper (or a `RoadSection.no_passing=True` flag) that sets the center-line style and places the start and end signs. For mixed lines, the solid side must face the ego lane.
  - Validate the constraints listed above.
  - Add an optional slot for a dynamic obstacle (DYN-05).

### ELEM-05: Rule-compliant parking zones (P1, M)

- **Rule (Reg. 5.4, Abb. 1–3):**
  - **Zone:** planar, straight, dashed center line, no missing markings, no other elements. The parking sign is placed 300–500 mm before the first area (FIX-09).
  - **Parallel (right side):**
    - Depth 300 mm, plus an extra solid line.
    - Spots **increase in length** when approaching from the sign; the last spot is **≥ 700 mm**.
    - Gaps < 400 mm between obstacles can exist, but they are not valid spots.
    - Obstacles have a lateral gap of **20–200 mm** to the right lane marking.
    - Spots can be marked as "no parking" (X).
  - **Perpendicular (left side):**
    - Uniform spots, 350 mm wide × 500 mm deep.
    - Blocked by obstacles or an X.
    - Obstacles sit 20–100 mm from the solid left lane marking.
  - **Opening angle** of the lot outline: **45°–60°** (the code fixes it at 60°, which is allowed, but it should be configurable within the range).
  - Abb. 1 also says that at least one perpendicular and one parallel area should lie **within 20 m** of the start line.
  - Obstacles are never in a driving lane.
- **Current state:** `ParkingArea`, `ParkingLot` and `ParkingSpot` support the geometry but enforce none of these constraints. Free spots on the right side have no separating lines, which matches the rules.
- **To do:**
  - Add a generator `ParkingArea.random_compliant(seed, ...)` that creates rule-conformant layouts with obstacle placement and size randomisation.
  - Add validation for all constraints above.
  - Expose the spot list as ground truth (for parking evaluation, REF-07).

### ELEM-06: Barred area in the left lane, plus oncoming-traffic scenario (P1, S)

- See FIX-06 for the geometry.
- Add an optional dynamic oncoming obstacle (DYN-04).
- Note the rule (Reg. 5.6): at most **one** dynamic obstacle per barred area.

### ELEM-07: Crosswalk pedestrian waiting areas and pedestrian model (P1, M)

- **Rule (Reg. 5.7, Abb. 9, Abb. 21):**
  - Each crosswalk has two waiting areas, one on each roadside: **200 mm** wide (beyond the edge line) × **450 mm** long (along the road, around the crosswalk).
  - Up to **one** pedestrian per crosswalk, on the left or right side.
  - **Pedestrian model:** a white cardboard box, 100 mm wide × 150 mm high, with a black pictograph (stick figure about 120 × 75 mm) on the sides facing the road.
- **To do:**
  - Add a `Pedestrian` element and a `ZebraCrossing(pedestrian=Pedestrian(side=LEFT|RIGHT, ...))` option.
  - Model: a box with a textured pictograph on the faces seen by an approaching vehicle.
  - Spawn it in the waiting area.
  - Crossing behaviour is described in DYN-07.

### ELEM-08: Landmarks (QR-code signs) (P2, M)

- **Rule (Reg. 5.10, Abb. 24):**
  - At most **10** landmarks with IDs **0–9**, assigned in ascending order starting at 0.
  - The sign has the same shape and size as a speed-limit sign and carries a QR code with a unique identifier.
  - Each landmark is **two signs with the same ID, one on each side of the track**, each facing the vehicle in the right lane of its direction, so it can be detected from both directions.
  - Landmarks are placed wherever they are not obstructed by other elements or obstacles.
- **Current state:** Not available.
- **To do:**
  - Add a `Landmark(id, arc_length)` element that adds the pair of signs automatically.
  - Generate the QR textures. Use exactly the payloads from Abb. 24 if they can be decoded; otherwise use the plain ID. Generate them at build time or ship the PNGs, and create one `landmark_<id>_sign.dae` mesh from the speed-limit sign template.
  - Validate the count (≤ 10), unique and contiguous IDs, and that the position is unobstructed.
  - Expose the landmark positions as ground truth for the referee (REF-10).
  - Requires the Python package `qrcode` (add it to `requirements.txt`) if textures are generated.

### ELEM-09: Pedestrian island (`TrafficIsland`): keep, low priority (P3, –)

- The 2026 regulation text no longer describes pedestrian islands. The "pedestrian island" sign is still shown in Abb. 18, and `Abb_10/11_pedestrian_island*.pdf` exist in the sources but are not referenced in the text.
- **To do:** Keep `TrafficIsland` as is. Do not prioritise further work unless the commission confirms that islands are part of the next competition (see Open Questions).

---

## 7. Obstacles and other traffic participants (DYN)

None of this exists yet. Dynamic actors are the **largest single gap**: the Obstacle Evasion Course awards 15 points per dynamic overtake and 10 points per right-of-way situation, and several elements (barred area with oncoming traffic, no-passing zone, crosswalk with pedestrian) are only meaningful when something moves.

### DYN-01: Static obstacle improvements (P2, S)

- FIX-10: make them fixed and validate the sizes.
- Allow placement **outside the track** (Reg. 5.2: these are not artifacts, so the 100 mm rule does not apply).
- Validate a minimum spacing of **1000 mm** between any two obstacles along the track, and that they never block both lanes.

### DYN-02: Dynamic obstacle core (P1, L)

- **Rule (Reg. 5.2.2):**
  - A white box (same size range as static obstacles) that moves along the track at up to **0.6 m/s**, keeping to the **right lane**.
  - It does not change lanes and does not overtake.
  - It can **stop temporarily**.
  - At least one dynamic obstacle is present in the Obstacle Course.
- **Design proposal:**
  - **Model:** a box model that is kinematic, or has a large mass and no gravity issues. Move it by pose updates, using the gz `set_pose` service or the gz *TrajectoryFollower* or *Actor* scripted trajectory, instead of physics forces. This keeps it robust at low real-time factors.
  - **Controller node** `dynamic_obstacle_controller.py` (new):
    - Takes a path: the right-lane centre of a chosen sequence of sections, or the reverse direction for oncoming traffic.
    - Takes a speed profile: constant, stop-and-go, or triggered.
    - Takes trigger conditions, for example "start when the ego is within X m" or "stop for t s at position s".
    - Publishes its ground-truth pose for the referee.
    - All behaviour uses `/clock`, so it stays consistent at low RTF.
  - **Road DSL:** `section.add_dynamic_obstacle(start_arc_length, speed, behaviour=..., direction=SAME|ONCOMING, ...)`.
- **Collision:** the obstacle needs a collision box and a contact sensor so the referee can detect hits (REF-04).

### DYN-03: Dynamic obstacles at intersections (P1, M)

- **Rule (Reg. 5.3, 7.4, Abb. 6):**
  - At most one dynamic obstacle per intersection.
  - It approaches from a **different direction than the ego**, goes straight, left or right, and **obeys the intersection's priority rules** (priority to the right, give-way, stop).
  - The ego must respect the obstacle's right of way if it is inside the **defined area** of Abb. 6 (a zone on the obstacle's approach lane next to the intersection, dimensioned at about 200 mm + 250 mm).
  - If the ego must yield, it waits until the obstacle has **completely cleared** the intersection.
- **To do:**
  - The obstacle controller needs intersection-aware behaviour: approach, stop at its own stop or give-way line if required, wait, then cross along the turn path.
  - The trigger should be synchronised with the ego arrival, so right-of-way situations actually occur.
  - Expose the "defined area" polygon as ground truth for the referee.

### DYN-04: Oncoming obstacle at a barred area (P1, M)

- **Rule (Reg. 7.4, Barred Area):**
  - If a dynamic obstacle approaches from the opposite direction, the ego must yield and wait.
  - If the ego has already started passing, the obstacle **stops and waits at ≥ 500 mm** from the barred area.
  - If the ego fails to yield and both meet in the free lane, the obstacle performs an **evasive manoeuvre** to avoid a collision or deadlock.
- **To do:**
  - Add a scenario script with three branches: (a) obstacle has priority → it drives through; (b) ego already committed → the obstacle stops at 0.5 m; (c) conflict → the obstacle evades, for example by pulling to the side and stopping.

### DYN-05: Dynamic obstacle in a no-passing zone (P1, S)

- **Rule (Reg. 7.4):** The ego must follow at **≥ 300 mm** until the end of the zone.
- **To do:** Spawn a slow obstacle that drives through the whole no-passing zone and continues behind it, and expose its pose to the referee.

### DYN-06: Overtakable dynamic obstacle on regular road (P1, S)

- **Rule (Reg. 7.4):**
  - It may be passed anywhere **except** in intersections, no-passing zones, barred areas and crosswalks.
  - After passing, the ego must merge back within 2 m.
- **To do:** A slow obstacle (for example 0.2–0.4 m/s) on a long straight. Optionally add stop-and-go behaviour to test following and stopping.

### DYN-07: Pedestrian behaviour at crosswalks (P1, M)

- **Rule (Reg. 7.4, Crosswalk):**
  - If a pedestrian is in a waiting area, the ego must stop in front of the crosswalk, with the same rules as at intersections: ≤ 150 mm, not crossing the line.
  - The pedestrian **starts crossing only after the ego has stopped**, crosses in front of the vehicle, then the ego may continue.
  - With no pedestrian, the ego may slow down but **must not stop**.
- **To do:**
  - A pedestrian controller: wait in the area, detect the ego standstill near the crosswalk (from ground-truth odometry), then move the box across the crosswalk at a realistic speed (for example 0.1–0.2 m/s) into the opposite waiting area.
  - Expose the pedestrian state (`waiting`, `crossing`, `cleared`) to the referee.

---

## 8. Vehicle-side features: lights, RC mode, buttons (VEH)

The simulation is meant to be a drop-in replacement for the real car, so everything the stack *commands* on the real car (lights, mode changes) should be **accepted, visualised and logged** by the simulation. The referees score these features.

### VEH-01: Lights and indicators (P1, M)

- **Rules (Reg. 4.1.7):**
  - **Brake lights:** 3 red lights at the rear. They must signal active braking. In Free Drive, faulty activation costs 2.5 m, at most 3 times.
  - **Direction indicators:** 4 yellow or orange lights, one per corner. They flash at **≤ 2 Hz, 50 % duty cycle, initial state on** when overtaking, turning or parking. Parking requires flashing **all** indicators at least once. False use costs −2 per occurrence, at most 10.
  - **RC-mode light:** a blue light at the highest point, flashing at **1 Hz, 50 %, starting on**.
  - **Headlights (2 white) and taillights (2 red):** not required in 2026, **mandatory from 2027**. Brake lights may double as taillights at reduced intensity.
- **To do:**
  - Subscribe to the same light-command topics the real car uses (see Open Questions).
  - Add light links to `models/smarty/model.sdf` and switch them with emissive visuals or gz light entities (for example through the gz `LightConfig` or `VisualConfig` services, or a small gz system plugin).
  - Publish a **normalised light state** for the referee (REF-08): `brake`, `indicator_left`, `indicator_right`, `hazard`, `rc_light`, with timestamps. The referee then checks the flashing frequency and the initial state.

### VEH-02: RC-mode emulation (P2, M)

- **Rules (Reg. 4.3, 7.1):**
  - Switching to RC mode must **instantly stop** the car without further steering.
  - The car must then stay **stationary for ≥ 1 s** before remote control or switching back is possible.
  - Maximum speed in RC mode is **0.3 m/s** during events.
  - The blue light flashes.
  - An **acoustic signal ≤ 0.5 s** on each switch is mandatory from 2027.
  - It is **not allowed inside the start box**: doing so cancels the attempt.
  - The car must be repositioned within **1 m** of where it left the track before switching back to autonomous mode.
- **Current state:** `keyboard_control` publishes directly to the control topics and conflicts with the autonomous stack.
- **To do:**
  - Turn `keyboard_control` (or a new `rc_mode_mux` node) into a **mux** between autonomous and RC commands: a toggle key, a forced 1 s standstill, a 0.3 m/s cap, and RC-light flashing.
  - Publish the RC-mode state on the topic the real car uses, so the stack reacts as it would on the real car (see Open Questions).
  - Optional: a "reposition" service that teleports the car back onto the route within 1 m of the departure point.
  - Count activations for the referee (REF-09).

### VEH-03: Event selection and mapping-end buttons (P2, S)

- **Rules (Reg. 4.1.9, 7.5):** The vehicle has distinct buttons to start the event modes. In the Navigation Course an additional button can signal the end of the mapping phase.
- **To do:**
  - Emulate the buttons by publishing the same messages the real hardware publishes. This could be a ROS service, a Gazebo GUI plugin with buttons, or keyboard shortcuts.
  - Document the mapping in the README.

---

## 9. Competition procedure: start box, start scheduling, events (EVT)

### EVT-01: Start box with gate (P1, M)

- **Rules (Reg. 6.2.3, Abb. 1, Abb. 23):**
  - The start box is separated from the track by **physical barriers** and its width is 350–450 mm.
  - At its front is an **openable gate** marked with a **stop sign (150 mm)** and a **QR / matrix barcode (150 mm)**, side by side with a gap of **≥ 50 mm**, mounted **100–200 mm** above the ground.
  - The gate opens at "green" and **stays open for 30 s**.
  - The start-box exit can be separated from the track by a **solid white line**, which may be crossed.
  - In Abb. 1, the start box joins the right lane shortly before the starting line.
- **Current state:** The car spawns at the world origin `<pose>0 0 0 0 0 0</pose>` ([world.sdf:87-89](../models/world.sdf#L87-L89)), on the track.
- **To do:**
  - Add a `StartBox` element: a lane stub with markings, wall models, and a gate model with the sign texture and QR texture (decode or reproduce the QR from Abb. 23).
  - Animate the gate opening (prismatic or revolute joint, or pose change) on command.
  - Make the car spawn pose configurable (`world_spawner` parameter, or derived from the `StartBox` element), so that the car starts **inside** the box.
- **Why it matters:** The real car must detect the gate opening, probably through the camera or QR code, to start. This cannot be tested today.

### EVT-02: Start scheduling system (P2, S)

- **Rule (Reg. 6.2.2):** Red (standby) → Yellow (preparation, max 5 min) → **Green** (gate opens, timing starts) → **Red** (stop). At the final red the vehicle must **come to an immediate, complete stop and stay stationary**.
- **To do:**
  - Add a `start_scheduler` node that publishes the phase and a countdown, opens the gate at green, and starts the referee timer.
  - At the end of the event it signals red. The referee checks that the vehicle stops.
  - Provide a CLI or service to trigger the phases manually, or automatically per event.

### EVT-03: Free Drive Course scenario (P1, S)

- **Rule (Reg. 7.3):**
  - Rural road with long straights, tight turns and intersections.
  - **Not present:** obstacles, barred areas, crosswalks, landmarks.
  - Intersections are crossed straight; signs and center-line types can be ignored; parking zones are ordinary road.
  - **2-minute** time limit; the score is the distance covered.
- **To do:**
  - Add an event preset (`event:=free_drive`) that loads a Free Drive track and configures the referee: timer 120 s, distance scoring, and penalties *leave lane* 5 m, *RC* 5 m, *brake light* 2.5 m (at most 3), *wrong direction at intersection* 5 m.
  - Add an example track `models/roads/free_drive_example/`.

### EVT-04: Obstacle Evasion Course scenario (P1, M)

- **Rule (Reg. 7.4):** All elements except landmarks; **3 laps within 5 minutes**; per-element scoring; one parking attempt per lap.
- **To do:**
  - Add an event preset (`event:=obstacle`).
  - Add an example track modelled on **Abb. 22** (example circuit), including a parking zone near the start (Abb. 1), dynamic obstacles, and a pedestrian.
  - Add per-lap actor reset logic, so each lap can present the scenario again or a variant of it.

### EVT-05: Navigation Course scenario: mapping and navigation phases (P2, M)

- **Rule (Reg. 7.5):**
  - **Mapping phase:** 3 minutes of free driving.
  - Then the car is **placed back into the start box** without software changes, and the **Navigation phase** of 3 minutes begins.
  - The car must visit landmarks **0, 1, 2, …** in order over the shortest distance.
  - All Obstacle Course elements can appear, **but no obstacles or pedestrians**. Speed limits can apply.
  - A designated route is **not guaranteed**.
- **To do:**
  - Add an event preset (`event:=navigation`).
  - Add a service that **teleports the car back into the start box without restarting** the simulation or the stack, so the map is kept in memory.
  - Signal the phase transitions through the start scheduler (EVT-02).
  - Add an example track modelled on **Abb. 25**, with landmarks.
  - The second attempt always starts with mapping again, and mapping data must be discarded. The simulation only needs to provide the reset; discarding the data is the stack's job.

### EVT-06: Scenario configuration and randomisation (P2, M)

- **To do:**
  - Add an `event` launch argument and per-event parameter files.
  - Add seed-based randomisation of: obstacle positions and sizes, dynamic obstacle triggers, pedestrian presence and side, parking occupancy, marking gaps, lane width, and sign distances inside their intervals.
  - Store the seed in the referee report so every run can be reproduced.

---

## 10. Automatic referee and scoring (REF)

A new `referee` node (or a set of nodes) that uses **ground truth** (Gazebo poses, contacts, the track model and actor states) to evaluate a run the way the official referees would, and to produce a score report. This turns the simulation into a regression-test tool: run a scenario, then compare scores between stack versions.

### REF-01: Ground-truth interfaces (P1, M)

- The ego pose and the 4 wheel contact points, taken from Gazebo poses. `/tf` already carries link poses through the PosePublisher.
- Track geometry: lane polygons, line polylines with type, element zones, stop and give-way lines, crosswalk areas, parking-spot polygons, landmark positions, speed-limit intervals, and the designated route (TRK-08). These come from the `Road` object.
- Actor states: dynamic obstacles and pedestrians.
- **Note:** branch `origin/feature/labeling` contains an unmerged `groundtruth_labeler.py` (about 630 lines, "lane labeling"). Check whether it can serve as a base for the ground-truth export.

### REF-02: Laps, time limits and distance (P1, S)

- Count laps when the start line is crossed along the designated route.
- Timers: Free Drive 2:00; Obstacle Course 5:00 and 3 laps; Navigation 3:00 + 3:00.
- Distance travelled, from integrated ground-truth odometry (Free Drive score, Navigation denominator).
- Penalties only start once the vehicle has left the start box **and passed the starting line** (Reg. 7.1).

### REF-03: Lane departure (P1, M)

- **Rule:** Crossing the lane markings with **two or more wheels** is a penalty: Free Drive 5 m, Navigation 1 m, Obstacle Course −2 (at most 10).
- Detect it from the wheel contact points against the right-lane polygon.
- Exceptions: allowed overtaking, the barred-area bypass, and parking manoeuvres (Reg. 7.4).
- Also detect skipped elements and sections, which make a lap invalid.

### REF-04: Collisions (P1, S)

- Add contact sensors on obstacles, signs and pedestrians, or ego-chassis contacts filtered by the other entity's name.
- Penalties:
  - Obstacle: −10 element evaluation.
  - Road sign: −5 each.
  - Pedestrian: −10.
  - Parking obstacle: invalidates the parking attempt and gives −10.

### REF-05: Intersection evaluation (P1, M)

- Stop line: standstill **≥ 3 s**. Give-way line: **≥ 1 s**. In both cases the vehicle front must **not cross** the line and must be **≤ 150 mm** from it.
- Correct direction: straight if there is no mandatory turn; otherwise the turn shown by the sign and arrow.
- Right of way of a dynamic obstacle inside the Abb. 6 area.
- Scores per table (+5 / +10 / +15 / 0 / −5):
  - *Drive straight stop/give-way*
  - *Drive straight priority-to-right*
  - *Mandatory turn stop/give-way*
  - *Mandatory turn priority-to-right*
  - *Obstacles at intersections*

### REF-06: Crosswalk evaluation (P1, S)

- With a pedestrian: stop ≥ 3 s, ≤ 150 mm before the crosswalk, wait until the pedestrian has cleared → +15. Collision → −10.
- Without a pedestrian: **no** full stop → +5, otherwise 0.

### REF-07: Parking evaluation (P1, M)

- Full stop with **all wheels inside a valid spot**, and **all indicators** flashing at least once. The position is checked at the first flash.
- The start of the manoeuvre must be signalled with the indicator on the spot's side.
- One attempt per lap. A second attempt is penalised (0 points: "more than one parking manoeuvre").
- The car must leave the parking area back into the right lane. Leaving the outer parking boundary counts as a lane departure.
- Speed in the parking lot must not exceed the speed allowed on the adjacent road.
- Score +20 / 0 / −10.

### REF-08: Indicator and brake-light evaluation (P1, M)

- Requires VEH-01.
- **Indicators:** expected on lane changes (overtaking, barred area), turns and parking. Check frequency ≤ 2 Hz, 50 % duty cycle and initial state on. False use costs −2 (at most 10).
- **Brake light:** expected whenever the vehicle is actively decelerating. A "faulty activation" (lit without braking, or not lit while braking) costs 2.5 m in Free Drive, at most 3 times. The deceleration threshold has to be defined (see Open Questions).

### REF-09: Overtaking, no-passing, barred area and RC mode (P1, M)

- **Static obstacle passed:** +10. **Dynamic obstacle passed:** +15. Merging distance > 2 m gives 0. Passing inside an intersection gives 0.
- **No-passing zone:** the solid line was not crossed **and** the distance to the obstacle stayed ≥ 300 mm → +5. Distance < 300 mm → 0. Solid line crossed with more than 1 wheel → −10.
- **Barred area:** not entered → +10. Oncoming obstacle: right of way respected → +5.
- **RC mode:** −5 (Obstacle Course), 5 m (Free Drive), 1 m (Navigation), per activation.

### REF-10: Speed limits (P2, S)

- Speed limits are signed in km/h and must be scaled 1:10, so a 30 km/h sign means 3 km/h ≈ **0.833 m/s** for the car.
- Each violation costs −5 in the Obstacle Course.
- The "violation" logic still needs to be defined: for example, an episode above the limit by more than a tolerance (see Open Questions).

### REF-11: Navigation Course evaluation (P2, S)

- A landmark counts as **successfully visited** once the car has *fully passed* both of its signs **and** all lower IDs were visited before. Visits out of order do not count; visits in RC mode do not count.
- Primary rank: the number of landmarks successfully visited.
- Score = `BasePoints · Multiplier · MinDistance / (Distance + ΣPenalties)`. `MinDistance` is computed from the track graph as the shortest path through landmarks 0 → N.

### REF-12: Output and visualisation (P2, S)

- Publish live events on a topic (`/referee/events`) and RViz markers (zones, lines, the current element, the next landmark).
- Write an end-of-run report as JSON or Markdown with: seed, track, event, laps, per-element evaluations, penalties, multipliers and the final score.
- Optional: a simple Gazebo GUI overlay showing the time, laps and score.

---

## 11. Track validation and tooling (TOOL)

### TOOL-01: Road validator ("rule linter") (P1, M)

A pure-Python checker that runs on `Road` objects. It can run in CI and as a CLI, for example `python -m simulation.groundtruth.road.validate models/roads/<name>`. It checks at least:

| Check | Rule |
|---|---|
| Minimum inner radius of the sharpest turn ≥ 1000 mm (so middle-line radius ≥ 1.0 m + lane width) | 5.1 |
| Distance between track elements ≥ 1000 mm, no overlap | 5.1 |
| Distance between neighbouring roads ≥ 50 mm, between the outer edges of the markings | 5.1 |
| Lane width 350–450 mm, with smooth transitions | 5.1.2 |
| Marking type persists ≥ 1000 mm | 5.1.2 |
| Missing markings ≤ 1000 mm; ≤ 2 at the same time outside intersections; none in parking zones or turn lines | 5.1.2, 5.3, 5.4 |
| Side junction ≤ 960 mm, on a straight | 5.1.2 |
| Sign lateral offset 75–125 mm; distances in range | 5.1.3, App. |
| Guide signs present where required | 5.1.3 |
| No elements on slopes; grade signs ≥ 1000 mm before | 5.1 |
| Obstacles: sizes, spacing ≥ 1000 mm, never both lanes blocked, none in parking lanes, expressway or no-passing zone | 5.2, 5.4, 5.5, 5.8 |
| Intersection angle 70°–90°, 3 or 4 arms, ≤ 2 stop lines | 5.3 |
| Parking constraints (ELEM-05) | 5.4 |
| No other element within 2000 mm after a no-passing zone | 5.5 |
| Barred area length ≤ 2000 mm, on a straight | 5.6 |
| Expressway length ≥ 5 m, no obstacles, no stop lines | 5.8 |
| Speed-limit markings do not overlap other markings; signs side by side or ≥ 500 mm apart | 5.9 |
| Landmarks ≤ 10, IDs contiguous from 0, two signs each, unobstructed | 5.10 |
| Designated route is closed (except for Navigation tracks) | 5.1.1 |

**Note:** The intersection hypothesis test samples angles up to 110° ([intersection_test.py:36](../simulation/groundtruth/road/sections/test/intersection_test.py#L36)). The rules allow 70°–90°.

### TOOL-02: Track preview and documentation (P3, S)

- Add a CLI that renders a road to PNG with sign, obstacle and landmark overlays, without starting Gazebo. This makes it quick to review new tracks.
- Add a README section that lists every DSL element and its parameters, linked to the rule section it implements.

### TOOL-03: Sign mesh dimension check (P3, S)

- Verify the existing sign meshes against Abb. 18 and Abb. 19: plate sizes on the 50 mm grid, and the plate's lower edge about **150 mm** above the ground. Also check that the collision boxes in `SignTuple` match the meshes.

---

## 12. Suggested roadmap

| Phase | Goal | Items |
|---|---|---|
| **1. Correctness** (about 1 week) | Generated tracks comply with the rules | FIX-01 … FIX-12, TRK-06, TOOL-01 (basic checks) |
| **2. Static completeness** (about 2 weeks) | Every static element from Reg. 5 can be built | TRK-01, TRK-02, TRK-03, TRK-08, ELEM-01 … ELEM-05, ELEM-07 (static pedestrian), ELEM-08, EVT-01 |
| **3. Dynamic actors** (about 2–3 weeks) | Obstacle Course scenarios become meaningful | DYN-01 … DYN-07, VEH-01 |
| **4. Procedure and referee** (about 2–3 weeks) | Score a full run automatically | EVT-02 … EVT-06, REF-01 … REF-12, VEH-02, VEH-03 |
| **5. Realism** (ongoing) | Robustness and 2027 readiness | TRK-04, TRK-05, TRK-07, headlights, taillights and speaker (VEH-01/02), TOOL-02, TOOL-03, FIX-13 |

Dependencies to keep in mind:

- **REF-\*** depends on REF-01 (ground truth) and TRK-08 (designated route).
- **REF-08** depends on VEH-01.
- **DYN-03/04/05** depend on DYN-02.
- **EVT-05** depends on ELEM-08 and EVT-01.

---

## 13. Open questions

These points cannot be decided from the regulations or the code alone. Clarify them with the team or, where marked, with the commission.

1. **Interfaces of the real car:** Which ROS topics and message types does the Smarty stack use for (a) light commands (brake, indicators, RC light), (b) RC-mode state, (c) event-selection and mapping-end buttons, (d) start-gate detection? The simulation should mirror these exactly (VEH-01 … VEH-03, EVT-01).
2. **Target competition:** The 2026 competition (9 May 2026) is over. Should the simulation target the 2026 rules as documented here, or should items marked "mandatory from 2027" (headlights, taillights, RC-mode speaker) already be treated as P1?
3. **Pedestrian islands:** They are not described in the 2026 text, but their figures and sign still exist in the sources. Keep `TrafficIsland` as low priority, or remove it? (Commission question.)
4. **Referee thresholds that the rules leave open:** speed-limit tolerance and the definition of a "violation episode" (REF-10); the deceleration threshold for a "faulty brake light" (REF-08); the exact geometry of the Abb. 6 right-of-way area (REF-05).
5. **Dynamic actor implementation:** Pose-driven kinematic models through a ROS node (portable, simple) or a custom gz system plugin in C++ (more accurate, but adds a C++ build to an `ament_python` package)? The recommendation is to start pose-driven.
6. **Start-gate QR code content:** Does the stack decode the gate's matrix barcode (Abb. 23) or only detect the gate opening? If it decodes it, the exact payload must be reproduced.
7. **Landmark QR payloads:** Does the stack expect the exact payloads printed in Abb. 24, or only the numeric IDs?

---

## 14. Appendix: key dimensions from the regulations

All values are in mm unless noted. The figure tolerance is ±5 %.

| Item | Value | Source |
|---|---|---|
| Lane width (inner edges) | 350–450 | 5.1.2 |
| Line width | 18–20 | 5.1 |
| Center dashed | 200 stroke / 200 gap | 5.1.2 |
| Double solid | 2 lines, about 20 apart, total 56–60 | 5.1.2 |
| Side junction | 100 dash / 50 gap, ≤ 960 long | 5.1.2 |
| Missing marking | ≤ 1000 | 5.1.2 |
| Start line | checkered, about 50 wide | 5.1 |
| Min. inner radius | 1000 | 5.1 |
| Element spacing | ≥ 1000 | 5.1 |
| Neighbouring roads | ≥ 50 apart | 5.1 |
| Artifacts | ≥ 100 from markings | 5.1 |
| Slope | ≤ 10 %, sign ≥ 1000 before | 5.1 |
| Sign lateral offset | 75–125 from the right lane edge | 5.1.3, Abb. 19 |
| Sign plate lower edge | about 150 above ground | Abb. 19 |
| Guide signs | radius < 1200 after a straight ≥ 3 m; first sign 1500 before, second at the start, small signs every 400 to the apex | 5.1.3 |
| Obstacle | W 100–400, H 100–240, L ≥ 100; spacing ≥ 1000 | 5.2, Abb. 20 |
| Dynamic obstacle speed | ≤ 0.6 m/s | 5.2.2 |
| Intersection angle | 70°–90°, 3 or 4 arms, corner radius about 100 | 5.3 |
| Stop line | solid, 36–40 wide | 5.3 |
| Give-way line | 80 dash / 60 gap, 36–40 wide | 5.3 |
| Intersection signs | 300–500 before (priority, stop, yield); mandatory-turn sign 150–250; turn arrow 50–250 before the line | Abb. 5, 14 |
| Turn arrow marking | 70 × 600 | Abb. 17 |
| Stop / give-way behaviour | stop ≥ 3 s / ≥ 1 s, front ≤ 150 from the line, not crossing it | 7.4 |
| Parking sign | 300–500 before the first area | 5.4 |
| Parallel parking | depth 300; last spot ≥ 700; obstacles 20–200 from the lane marking; gaps < 400 are not spots | 5.4, Abb. 2 |
| Perpendicular parking | spot 350 × 500; obstacles 20–100 from the left lane marking | 5.4, Abb. 3 |
| Parking lot outline angle | 45°–60° | Abb. 1 |
| Parking areas | at least one of each within 20 m of the start | Abb. 1 |
| Barred area | ≤ 2000 long; outline 18–20; stripes 36–40 wide, 150 gap, 27°; flanks 60°; sign 300–500 before | 5.6, Abb. 8 |
| Crosswalk | stripes 36–40 wide, 400 long, 40 gap; sign 300–500 before | 5.7, Abb. 9 |
| Pedestrian waiting area | 200 × 450 per side | Abb. 9 |
| Pedestrian | box 100 wide × 150 high; pictograph about 120 × 75 | Abb. 21 |
| Expressway | ≥ 5 m, no obstacles, no stop lines | 5.8 |
| Speed-limit start marking | about 400 long × 150 wide (digits 60 wide each) | Abb. 17 |
| Speed-limit end marking | about 300 long × 200 wide, crossed out | Abb. 17 |
| Speed scale | km/h ÷ 10 (for example 30 km/h → 0.833 m/s) | 5.9 |
| Landmarks | ≤ 10, IDs 0–9, a pair per landmark (both sides), speed-limit sign shape | 5.10, Abb. 24 |
| Start box | width 350–450; gate stays open 30 s | 6.2.3, Abb. 1 |
| Gate marking | stop sign 150 + QR 150, gap ≥ 50, mounted 100–200 above ground | Abb. 23 |
| No-passing following distance | ≥ 300 | 7.4 |
| Merge back after a pass | ≤ 2 m | 7.4 |
| Barred-area oncoming obstacle wait | ≥ 500 from the barred area | 7.4 |
| Vehicle | wheelbase ≥ 200, track ≥ 160, width ≤ 300, height ≤ 300 | 4.1.4 |
| Indicators | ≤ 2 Hz, 50 %, start on | 4.1.7 |
| RC light | 1 Hz, 50 %, start on | 4.3 |
| RC mode | instant stop, ≥ 1 s standstill, ≤ 0.3 m/s | 4.3 |
| Time limits | Free Drive 2 min; Obstacle Course 5 min / 3 laps; Navigation 3 min + 3 min | 7.3–7.5 |
