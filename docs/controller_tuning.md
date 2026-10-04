# Controller tuning log

A running record of changes to the controller configurations in `config/`,
why each was made, and the evidence behind it. Newest entries at the bottom.
Each entry lists the symptom, the diagnosis, the change, the measured effect
and the alternatives that were tried and rejected.

Unless stated otherwise, measurements come from headless single runs of
`scripts/run_benchmark.py` on the default coverage path (3 swaths × 5 m,
0.75 m spacing, U-turn radius 0.375 m, 17.35 m total), ROS 2 Humble,
Nav2 1.1.20.

---

## 2026-10-04 — MPPI: stall in U-turns

**Config:** `config/nav2_mppi_fair.yaml`

### Symptom

MPPI aborted every run at the start of the second U-turn with
`Failed to make progress`, after 11.5 of 17.35 m. On both turns the robot
slowed from 0.83 m/s to below 0.1 m/s on entry. In the second turn it then
reversed, turned the wrong way (left instead of right) and stopped until the
progress checker gave up. RPP finished the same path.

### Diagnosis

Obstacles were ruled out first: the local costmap was entirely free at the
moment of the stall and the lidar returned no hits.

The cause is how Humble's MPPI decides it is near the goal. Its critics call
`withinPositionGoalTolerance()`, which measures the distance from the robot to
the **last point of the local path**, not to the real end of the path. The
local path only reaches `prune_distance` (1.5 m) ahead along the path
(`PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame`).

- On a straight swath that point is about 1.5 m away. `GoalCritic` and
  `PathFollowCritic` used `threshold_to_consider: 1.4`, a margin of only
  0.1 m.
- In a U-turn the path folds back. The point 1.5 m ahead along the path lands
  on the neighbouring swath, as close as the swath spacing (0.75 m) in a
  straight line. That is below 1.4 m, so `GoalCritic` switches on and
  `PathFollowCritic` switches off: the controller behaves as if it were
  arriving at the goal. It slows down and settles in a local minimum, while
  the "goal" it is aiming at slides along as the robot moves.

Newer Nav2 releases (Jazzy) compare against the real goal pose instead, so
this is specific to Humble with folded coverage paths.

### Change

| Parameter | Before | After |
|---|---:|---:|
| `GoalCritic.threshold_to_consider` | 1.4 | 0.5 |
| `PathFollowCritic.threshold_to_consider` | 1.4 | 0.5 |

Rule: these thresholds must stay below the swath spacing (0.75 m). If the
swath spacing or the turn geometry changes, revisit them.

### Effect

| | Before | After (run 1) | After (run 2) |
|---|---:|---:|---:|
| Result | aborted at 11.5 m | succeeded | succeeded |
| Completion time [s] | — | 32.9 | 33.3 |

The robot still spent about 10 s getting through the second turn after this
change; the next entry fixed that.

### Tried and rejected

| Variant | Result |
|---|---|
| `vx_min: 0.0` (no reversing) | Still aborted; pivoted the wrong way and oscillated in place |
| `time_steps: 56` (2.8 s horizon instead of 1.2 s) | Still aborted |
| Threshold fix + `time_steps: 56` | Succeeded but slower (46.5 s) |
| Threshold fix + `vx_min: 0.0` | Succeeded, 32.4 s, no clear gain |

---

## 2026-10-04 — MPPI: lateral offset on the swaths

**Config:** `config/nav2_mppi_fair.yaml`

### Symptom

At the start of the first swath MPPI steered about 4° left, drifted 4.7 cm
off the line, and then drove parallel to the reference about 3 cm to the
side for the rest of the swath. RPP and DWB stayed within 1 mm of the line
over the same first metres, so robot physics (casters, wheel friction) was
ruled out.

### Diagnosis

`PathAlignCritic`, the critic that penalises lateral distance from the path,
was weak in two ways:

- **Off at the start.** `offset_from_furthest` counts path poses, and the
  coverage path has one every 0.05 m. With a value of 20 the critic stayed
  disabled for the first metre, so the initial heading error, a product of
  MPPI's random sampling, went uncorrected.
- **Too light.** At `cost_weight: 2.0` (Nav2 default 14.0) a few centimetres
  of offset cost less than the sampling noise and `TwirlingCritic`'s penalty
  on the yaw rate needed to correct it. MPPI has no integral action, so
  driving parallel to the line was an acceptable optimum.

`PathAngleCritic` could not help: it only acts when the heading error exceeds
`max_angle_to_furthest` (1.2 rad, about 69°).

### Change

| Parameter | Before | After |
|---|---:|---:|
| `PathAlignCritic.cost_weight` | 2.0 | 14.0 (Nav2 default) |
| `PathAlignCritic.offset_from_furthest` | 20 | 5 |

### Effect

Both columns include the threshold fix from the previous entry.

| | Before | After (run 1) | After (run 2) |
|---|---:|---:|---:|
| Result | succeeded | succeeded | succeeded |
| Completion time [s] | 33.3 | 30.0 | 30.0 |
| CTE RMS, swaths [cm] | 6.7 | 0.7 | 0.7 |
| CTE max, swaths [cm] | 21.3 | 2.3 | 2.3 |
| CTE RMS, turns [cm] | 21.2 | 1.4 | 1.4 |
| CTE max, turns [cm] | 37.5 | 3.4 | 3.3 |
| Yaw jerk RMS (cmd) [rad/s³] | 2.6 | 3.1 | 3.0 |
| Coverage [%] | 93.1 | 99.4 | 99.4 |

The initial lateral drift dropped from 4.7 cm to under 1 cm. The cost is a
small rise in commanded yaw jerk.

### Tried and rejected

| Variant | Result |
|---|---|
| `wz_std: 0.4` (Nav2 default, less yaw noise) | Initial drift unchanged (4.7 cm); aborted in the second turn |

---

## Open notes

- **Benchmark table is stale.** `results/benchmark/summary_table.md` predates
  both MPPI fixes (MPPI shows as aborted at 8.2 s). Re-run the 3×3 benchmark
  before quoting numbers.
- **Fairness.** MPPI has now been tuned; RPP and DWB are still at their
  initial settings. Either state this in the thesis or give the other two
  controllers an equivalent tuning pass.
- **MPPI weights versus Nav2 defaults.** Several weights are still well below
  the defaults (e.g. `GoalCritic` 2.0 vs 5.0, `PathFollowCritic` 3.0 vs 5.0).
  Worth reviewing if MPPI is tuned further.
- **`PathAngleCritic.mode` is ignored on Humble.** That parameter only exists
  from Iron onward; Humble's equivalent is `forward_preference` (default
  `true`). The line has no effect on current behaviour.
- **`metrics_recorder` label.** Run without `-p controller:=<name>`, it writes
  `rpp` into `summary.csv` whatever controller is running.
  `run_benchmark.py` passes the label correctly.
