# Autonomous Lawn Mower Navigation

M.Sc. thesis project at Politecnico di Milano — Master's in Automation and Control Engineering, in collaboration with VIRES s.r.l.

## Scope

The broader thesis proposal ("Autonomous Lawn Mower Operation for Large-Scale Field Maintenance") covers a full pipeline including RTK GNSS/LiDAR localization, obstacle avoidance, and autonomous docking. This repository covers a focused subset of that work:

- Take a coverage path produced by [Fields2Cover](https://fields2cover.github.io/index.html) as input.
- Use [Nav2](https://docs.nav2.org) to follow that path as accurately as possible, evaluating and comparing different local controllers (MPPI, Regulated Pure Pursuit, DWB).
- Based on that analysis, improve/extend the navigation stack to get better path-following performance.

Obstacle avoidance and autonomous docking are **out of scope** for this part of the project.

## Running one controller and watching it in RViz

Each launch loads exactly one controller (`controller:=rpp|mppi|dwb`, one
`FollowPath` plugin in its `config/nav2_<controller>_fair.yaml`).

```bash
# terminal 1: Gazebo + Nav2 + RViz
ros2 launch mower_sim nav2_sim.launch.py controller:=rpp

# terminal 2, once Nav2 reports "Managed nodes are active": send the coverage path
ros2 run mower_sim run_mowing_path --ros-args -p use_sim_time:=true

# optional, terminal 3 (start before terminal 2): write the metrics CSVs
ros2 run mower_sim metrics_recorder --ros-args -p use_sim_time:=true -p controller:=rpp
```

RViz (`rviz/mower_view.rviz`) shows the robot model, the reference path in
yellow (`/coverage_path`) and the path the robot has actually driven in blue
(`/robot_trail`, ground truth, cleared at the start of every FollowPath goal).
`gazebo_gui:=false` skips the Gazebo window; `rviz:=false` skips RViz. The robot
is not reset between goals, so restart the launch for each new run.

## Controller benchmark

`metrics_recorder` listens to `/coverage_path`, the FollowPath action status and
the ground-truth robot pose (TF `map -> base_footprint`), and on goal completion
writes CSVs with:

- cross-track error (RMS and max), separately for swaths and U-turns. Each turn
  is widened by `turn_margin` (1 m) of path on both sides so a controller that
  cuts the corner is scored on the turn, not on the swath ends;
- completion time (FollowPath goal executing -> finished, sim time);
- yaw-rate smoothness: RMS/max angular acceleration and jerk, from `/cmd_vel`
  and from `/odom`, plus the count of samples with |jerk| above `jerk_threshold`;
- a rough coverage percentage: the share of the swath area (0.75 m strips) that
  a 0.75 m wide cutter centred on the robot swept.

Run every controller three times on the same path, headless, then build the
table and the figure:

```bash
python3 scripts/run_benchmark.py --runs 3 --output-dir results/benchmark/raw
python3 scripts/analyze_benchmark.py --raw-dir results/benchmark/raw --out-dir results/benchmark
```

Add `--launch-args use_lidar:=false` if your Gazebo build crashes loading the
ray sensor plugin (seen with RoboStack); the lidar is not used on the empty field.

## Status

Work in progress.
