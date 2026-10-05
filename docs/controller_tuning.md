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

## 2026-10-04 — RPP: corner cutting and slow recovery after U-turns

**Config:** `config/nav2_rpp_fair.yaml`

### Symptom

RPP completed the path (18.6 s) but tracked it poorly: swath CTE RMS
12.7 cm, turns 20.9 cm, coverage 91 %. The first swath was perfect (0 cm).
The robot started each U-turn about 1 m early, cut inside it, sometimes
stopped to rotate in place mid-turn, then overshot the next swath by up to
26 cm and took several metres to settle.

### Diagnosis

Two causes, both tied to the tight 0.375 m U-turn radius:

- **Lookahead too long.** At 0.7–1.5 m (1.5 m at full speed) the carrot was
  already on the next swath before the robot reached the turn, so pure
  pursuit cut the corner. A long lookahead also means a low corrective gain,
  hence the slow recovery after the turn.
- **Yaw-rate saturation.** The velocity_smoother caps the yaw rate at
  1.0 rad/s. Following a 0.375 m radius at 1 rad/s needs a speed of at most
  0.375 m/s, but with `regulated_linear_scaling_min_radius: 0.8` RPP only
  slowed to about 1.0 × 0.375 / 0.8 ≈ 0.47 m/s. The smoother then clipped the
  yaw rate but not the speed, so the robot turned wider than commanded. The
  rule is `min_radius >= desired_linear_vel / max_yaw_rate` (here ≥ 1.0 m).

Shortening the lookahead alone (variants R1, R3 below) made things worse:
with the yaw rate still saturating, the higher gain turned into a ±25 cm
weave after each turn. The weave was large enough to exceed
`rotate_to_heading_min_angle` (45°), so the robot kept stopping to rotate in
place. Both changes were needed together.

What remains is a ~10 cm overshoot at each turn exit. It comes mostly from
the velocity_smoother's 1.0 rad/s² yaw acceleration limit (unwinding from
1 rad/s to 0 takes a full second) and was left alone, because that limit is
shared by all three controllers.

### Change

| Parameter | Before | After |
|---|---:|---:|
| `lookahead_dist` | 1.2 | 0.7 |
| `min_lookahead_dist` | 0.7 | 0.35 |
| `max_lookahead_dist` | 1.5 | 0.7 |
| `regulated_linear_scaling_min_radius` | 0.8 | 1.5 |
| `rotate_to_heading` → `use_rotate_to_heading` | `true` (ignored) | `true` |

The last row is a rename only. Humble reads `use_rotate_to_heading`, whose
default is already `true`, so behaviour is unchanged.

### Effect

| | Before | After (run 1) | After (run 2) | After (final config) |
|---|---:|---:|---:|---:|
| Result | succeeded | succeeded | succeeded | succeeded |
| Completion time [s] | 18.6 | 22.7 | 22.7 | 22.7 |
| CTE RMS, swaths [cm] | 12.7 | 1.1 | 1.0 | 1.0 |
| CTE max, swaths [cm] | 26.5 | 5.2 | 5.2 | 5.2 |
| CTE RMS, turns [cm] | 20.9 | 4.8 | 4.8 | 4.9 |
| CTE max, turns [cm] | 37.3 | 10.5 | 10.3 | 10.6 |
| Yaw jerk RMS (cmd) [rad/s³] | 5.3 | 3.7 | 4.7 | 5.1 |
| Coverage [%] | 91.0 | 98.2 | 98.2 | 98.1 |

The cost is about 4 s of completion time, from slowing down in the turns.

### Variants tried

Lookahead given as min–max in metres.

| Variant | Lookahead | min_radius | Time [s] | Swath RMS [cm] | Turn RMS [cm] | Coverage [%] |
|---|---|---:|---:|---:|---:|---:|
| Baseline | 0.7–1.5 | 0.8 | 18.6 | 12.7 | 20.9 | 91.0 |
| R1 | 0.3–0.6 | 0.8 | 35.9 | 15.5 | 9.9 | 92.4 |
| R2 | 0.4–0.8 | 0.8 | 20.8 | 5.9 | 8.2 | 95.4 |
| R3 | 0.5 fixed | 0.8 | 39.7 | 18.2 | 10.5 | 93.6 |
| R4 | 0.4–0.8 | 1.2 | 21.6 | 2.1 | 6.7 | 97.6 |
| R5 | 0.3–0.6 | 1.2 | 22.4 | 5.0 | 3.3 | 96.0 |
| R6 | 0.4–0.8 | 1.5 | 22.4 | 1.9 | 6.5 | 97.8 |
| **R7 (adopted)** | **0.35–0.7** | **1.5** | **22.7** | **1.1** | **4.8** | **98.2** |

---

## 2026-10-04 — DWB: reversing before U-turns

**Config:** `config/nav2_dwb_fair.yaml`

### Symptom

DWB tracked the first swath almost perfectly, then about 1 m before the
first U-turn it started reversing and rocked back and forth between
x ≈ 3.3 and 3.5 m without ever turning, until the progress checker aborted.

### Diagnosis

Verified against the Humble source and the critic scores DWB publishes on
`/evaluation` (`debug_trajectory_details: true`):

- **The local goal folds back behind the robot.** `transformGlobalPlan`
  starts the local plan at the first pose within `prune_distance` of the
  robot (i.e. behind it) and ends it at the first pose more than
  `forward_prune_distance` (2.0 m, Euclidean) from the robot. GoalDist and
  GoalAlign use the end of that plan as their goal. Once the whole U-turn is
  inside the 2 m circle, the end lands on the next swath, behind the robot.
  At the stall the best-scored trajectory was reversing (vx = −0.40, total
  11.07, almost all of it GoalDist and GoalAlign) against 11.43 for the best
  forward-and-turning one. The Oscillation critic then blocked direction
  changes, so the robot rocked in place.
- **`prune_distance` must stay below `forward_prune_distance`.** Otherwise
  the plan's start pose is already beyond the end threshold and the plan
  comes out empty ("Resulting plan has 0 poses").
- **A short window brings a random freeze.** With the window below the
  0.75 m swath spacing the plan can no longer fold, but runs randomly
  freeze on the path. Scores captured during a freeze show standing still
  and moving forward tie exactly (9.36 vs 9.36: GoalDist gained exactly what
  GoalAlign lost), and DWB keeps the first-evaluated candidate, standing
  still. With the goal this close, GoalAlign's forward point (taken ahead
  of each trajectory's end) overshoots it, so moving forward costs in
  GoalAlign what it gains in GoalDist. Removing GoalAlign (variant X1)
  ended the freezes.
- **GoalDist is DWB's only progress term.** Removing GoalDist and GoalAlign
  leaves the robot standing still at the start.

### Variants tried

Failures are random, so success rates need several runs per variant.

| Variant | prune / forward_prune [m] | sim_time [s] | Other | Succeeded |
|---|---|---:|---|---:|
| Baseline | 2.0 / 2.0 (defaults) | 1.2 | | 0/1 |
| W1 | 2.0 / 2.0 | 1.2 | no GoalDist, GoalAlign | 0/1 |
| W2 | 2.0 / 0.7 | 1.2 | | 0/1 (empty plan) |
| W3 | 0.4 / 0.7 | 1.2 | | 1/1, capped at ~0.55 m/s |
| W4 (committed) | 0.4 / 0.7 | 0.7 | | 3/6 |
| W5 | 0.3 / 0.6 | 0.6 | | 4/6 |
| W6 | 0.3 / 0.6 | 0.6 | Nav2 default critic scales | 0/3 |
| W7 | 0.35 / 0.65 | 0.4 | | 0/3 |
| X3 | 0.3 / 0.6 | 0.6 | no GoalAlign | 25/26 |
| **X1 (adopted)** | **0.3 / 0.6** | **0.6** | **no GoalAlign, no Oscillation** | **12/12** |

W3 was speed-capped because a trajectory longer than the 0.7 m window
overshoots its goal, so `sim_time × max_vel_x` should not exceed
`forward_prune_distance`.

### Change

| Parameter | Before | After |
|---|---:|---:|
| `prune_distance` | 2.0 (default) | 0.3 |
| `forward_prune_distance` | 2.0 (default) | 0.6 |
| `sim_time` | 1.2 | 0.6 |
| `critics` | RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist | RotateToGoal, BaseObstacle, PathAlign, PathDist, GoalDist |

Removing GoalAlign is the main fix. An ablation that keeps the Oscillation
critic (X3) completed 25 of 26 runs, against 4 of 6 with GoalAlign still
in (W5). Oscillation is dropped as well because X3's one failure looks like
the original lock-up: in the second U-turn the robot slipped back about
4 cm and then froze in place until the progress checker aborted, the
pattern of the Oscillation critic forbidding the forward direction after a
reversal. The scores for that freeze could not be captured: 14 further X3
runs, watched for a stall, all completed. Once the plan no longer folds
back the robot has no reason to reverse, so the critic has nothing useful
to damp. X3 and X1 track equally well (X3, mean of the 11 successful
benchmark runs, which record metrics: 24.2 s, swath CTE RMS 2.6 cm, turn
CTE RMS 2.8 cm, coverage 97.5 %).

### Effect

Mean over 12 runs, range in brackets. The baseline aborted in front of
the first U-turn every time.

| | Before | After (X1, 12 runs) |
|---|---:|---:|
| Succeeded | 0/1 | 12/12 |
| Completion time [s] | — (aborted at 14.3) | 23.9 (23.5–24.4) |
| CTE RMS, swaths [cm] | — | 2.8 (2.0–3.5) |
| CTE max, swaths [cm] | — | 5.1 (3.6–6.3) |
| CTE RMS, turns [cm] | — | 2.8 (2.7–3.0) |
| CTE max, turns [cm] | — | 6.9 (6.3–7.5) |
| Yaw jerk RMS (cmd) [rad/s³] | 21.5 | 15.0 (13.1–16.7) |
| Coverage [%] | 28.4 | 97.5 (96.8–98.1) |

DWB now tracks the turns more closely than the tuned RPP (turn CTE RMS
2.8 vs 4.9 cm), but stays a few cm off on the swaths and its commanded yaw
is far less smooth (jerk RMS about 15 vs 5), since it switches between
discrete velocity samples.

---

## Open notes

- **All tuning above used perfect localization.** Every measurement in this
  log was taken with ground truth odometry, the only mode at the time. The
  launch default is now `odometry:=encoder` (wheel encoder dead reckoning),
  so to reproduce these numbers pass
  `--launch-args odometry:=ground_truth` to `run_benchmark.py`. A first look
  with encoder odometry raised swath CTE RMS from 1.0 to 2.5 cm for RPP,
  0.7 to 2.1 cm for MPPI and 2.8 to 4.6 cm for DWB. RPP completed 2 of 2
  runs, MPPI and DWB 3 of 4 each; too few runs to tell whether encoder
  odometry makes them less reliable.
- **Benchmark table is stale.** `results/benchmark/summary_table.md` predates
  the MPPI and RPP tuning (MPPI shows as aborted at 8.2 s). Re-run the 3×3
  benchmark before quoting numbers.
- **Fairness.** All three controllers have now had a tuning pass on this
  path; RPP and DWB also needed structural fixes, not just gains. State
  in the thesis which parameters differ from Nav2's defaults and why.
- **Shared velocity_smoother limits.** The 1.0 rad/s yaw-rate cap and
  1.0 rad/s² yaw-acceleration limit bound every controller in the tight
  U-turns, and cause most of RPP's remaining turn-exit overshoot. If they are
  changed, change them for all three configs and re-tune.
- **MPPI weights versus Nav2 defaults.** Several weights are still well below
  the defaults (e.g. `GoalCritic` 2.0 vs 5.0, `PathFollowCritic` 3.0 vs 5.0).
  Worth reviewing if MPPI is tuned further.
- **`PathAngleCritic.mode` is ignored on Humble.** That parameter only exists
  from Iron onward; Humble's equivalent is `forward_preference` (default
  `true`). The line has no effect on current behaviour.
- **`metrics_recorder` label.** Run without `-p controller:=<name>`, it writes
  `rpp` into `summary.csv` whatever controller is running.
  `run_benchmark.py` passes the label correctly.
