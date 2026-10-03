# Autonomous Lawn Mower Navigation

M.Sc. thesis project at Politecnico di Milano — Master's in Automation and Control Engineering, in collaboration with VIRES s.r.l.

## Scope

The broader thesis proposal ("Autonomous Lawn Mower Operation for Large-Scale Field Maintenance") covers a full pipeline including RTK GNSS/LiDAR localization, obstacle avoidance, and autonomous docking. This repository covers a focused subset of that work:

- Take a coverage path produced by [Fields2Cover](https://fields2cover.github.io/index.html) as input.
- Use [Nav2](https://docs.nav2.org) to follow that path as accurately as possible, evaluating and comparing different local controllers (MPPI, Regulated Pure Pursuit, DWB).
- Based on that analysis, improve/extend the navigation stack to get better path-following performance.

Obstacle avoidance and autonomous docking are **out of scope** for this part of the project.

## Current state

- ROS 2 (Humble) + Gazebo simulation with a TurtleBot3 Burger as a stand-in platform.
- Nav2 bringup with per-controller parameter files (`config/nav2_mppi_fair.yaml`, `config/nav2_rpp_fair.yaml`, `config/nav2_dwb_fair.yaml`) for a fair, like-for-like comparison of controllers.
- `mower_sim/run_mowing_path.py` sends a fixed zigzag path via `NavigateThroughPoses` as a placeholder for a real Fields2Cover-generated path.
- Fields2Cover integration, controller benchmarking/analysis, and navigation stack improvements are still to be done.

## Status

Work in progress.
