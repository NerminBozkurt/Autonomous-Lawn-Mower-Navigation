#!/usr/bin/env python3
"""
Record how well the robot tracks /coverage_path and write the result to CSV.

Recording is tied to the FollowPath action: it starts when a goal on
follow_path starts executing and stops when that goal finishes (succeeded,
aborted or canceled), so the completion time is the controller's, not the
launch's. While recording, the node samples the robot's true pose from
/ground_truth/pose (published by ground_truth_tf_publisher, so it stays exact
even when the robot localizes from wheel odometry alone) on every /odom
message, thinned to sample_rate, and logs the yaw rate from both /cmd_vel
(what the robot was told, after Nav2's velocity smoother) and /odom (what it
did).

Samples within turn_margin metres of path from a U-turn count as turn
samples; see path_metrics.turn_zones for why.

A run may span several FollowPath goals: a switching configuration preempts
the goal at every controller change (see path_executor), which aborts the
old goal. The run therefore continues while another goal is accepted or
executing, and ends with the outcome of the last one. The active controller
(/active_controller) is logged per sample, and the controller's raw output
(/cmd_vel_nav, before the velocity smoother) feeds the transition metrics
taken around every row/turn boundary (path_metrics.transition_stats).

On finish it writes, into output_dir:
- <run_label>_trajectory.csv: one row per pose sample, with its cross-track
  error, whether it was matched to a swath or a turn zone, and the
  controller driving at the time.
- <run_label>_commands.csv: the controller's raw output (/cmd_vel_nav) and
  the smoothed command sent to the robot (/cmd_vel), with the controller
  driving at the time; for plotting what happens at a controller switch.
- <run_label>_reference.csv: the reference path, the geometric label of
  each segment and the zone it is scored in.
- summary.csv: one row per run, appended, with every metric (see
  mower_sim/path_metrics.py for definitions).
"""

import csv
import math
import os

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from mower_sim import path_metrics as pm
from mower_sim.path_segments import segments
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String

TERMINAL = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}

SUMMARY_FIELDS = [
    'run_label', 'controller', 'result', 'completion_time_s',
    'path_length_ref_m', 'path_length_robot_m', 'samples',
    'cte_rms_all', 'cte_max_all', 'cte_rms_swath', 'cte_max_swath',
    'cte_rms_turn', 'cte_max_turn',
    'cmd_ang_acc_rms', 'cmd_ang_acc_max', 'cmd_ang_jerk_rms',
    'cmd_ang_jerk_max', 'cmd_ang_jerk_events',
    'odom_ang_acc_rms', 'odom_ang_acc_max', 'odom_ang_jerk_rms',
    'odom_ang_jerk_max', 'odom_ang_jerk_events',
    'nav_ang_acc_rms', 'nav_ang_acc_max', 'nav_ang_jerk_rms',
    'nav_ang_jerk_max', 'nav_ang_jerk_events',
    'coverage_pct',
    'switches', 'switch_dv_max', 'switch_dw_max',
    'boundaries', 'trans_cte_mean', 'trans_cte_max',
    'trans_nav_dv_max', 'trans_nav_dw_max',
]


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MetricsRecorder(Node):

    def __init__(self):
        super().__init__('metrics_recorder')

        self.declare_parameter('path_topic', '/coverage_path')
        self.declare_parameter('status_topic', '/follow_path/_action/status')
        self.declare_parameter('pose_topic', '/ground_truth/pose')
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('cutting_width', 0.75)
        self.declare_parameter('min_swath_length', 1.0)
        self.declare_parameter('turn_margin', 1.0)
        # |angular jerk| above this counts as a "sudden change" (rad/s^3).
        self.declare_parameter('jerk_threshold', 10.0)
        self.declare_parameter('output_dir', 'metrics')
        self.declare_parameter('run_label', 'run')
        self.declare_parameter('controller', '')
        self.declare_parameter('exit_on_finish', True)

        p = self.get_parameter
        self.output_dir = os.path.expanduser(p('output_dir').value)
        self.run_label = p('run_label').value
        self.exit_on_finish = p('exit_on_finish').value

        self.latest_path = None
        self.ref_xy = None
        self.goal_id = None
        self.t_start = None
        self.done = False
        self.poses = []      # (t, x, y, yaw)
        self.cmd_w = []      # (t, wz)
        self.odom_w = []     # (t, wz)
        self.nav = []        # (t, v, wz, controller) of /cmd_vel_nav
        self.cmd = []        # (t, v, wz) of /cmd_vel
        self.controller = ''  # active controller, from /active_controller
        self.true_pose = None

        self.create_subscription(PoseStamped, p('pose_topic').value,
                                 self.pose_callback, 10)

        self.create_subscription(Path, p('path_topic').value,
                                 self.path_callback, 10)
        self.create_subscription(GoalStatusArray, p('status_topic').value,
                                 self.status_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 50)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 50)
        self.create_subscription(
            String, '/active_controller', self.controller_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.sample_period = 1.0 / p('sample_rate').value

        self.get_logger().info(
            f'Metrics recorder "{self.run_label}" waiting for a FollowPath '
            f'goal; output -> {self.output_dir}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def recording(self):
        return self.t_start is not None and not self.done

    def path_callback(self, msg):
        self.latest_path = msg

    def status_callback(self, msg):
        if self.done:
            return
        live = [bytes(st.goal_info.goal_id.uuid) for st in msg.status_list
                if st.status in (GoalStatus.STATUS_ACCEPTED,
                                 GoalStatus.STATUS_EXECUTING)]
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid)
            if self.goal_id is None and status.status == GoalStatus.STATUS_EXECUTING:
                # The reference is read at the end: run_mowing_path only
                # starts republishing /coverage_path after sending the goal.
                self.goal_id = gid
                self.t_start = self._now()
                self.get_logger().info('FollowPath goal executing, recording')
            elif gid == self.goal_id and status.status in TERMINAL:
                newer = [g for g in live if g != gid]
                if newer:
                    # Preempted by a controller switch: follow the new goal.
                    self.goal_id = newer[0]
                    continue
                self.finish(TERMINAL[status.status])
                return

    def pose_callback(self, msg):
        self.true_pose = msg.pose

    def nav_callback(self, msg):
        if self.recording:
            self.nav.append((self._now(), msg.linear.x, msg.angular.z,
                             self.controller))

    def controller_callback(self, msg):
        self.controller = msg.data

    def cmd_callback(self, msg):
        if self.recording:
            self.cmd_w.append((self._now(), msg.angular.z))
            self.cmd.append((self._now(), msg.linear.x, msg.angular.z))

    def odom_callback(self, msg):
        if not self.recording:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.odom_w.append((t, msg.twist.twist.angular.z))
        if self.poses and t - self.poses[-1][0] < self.sample_period - 1e-3:
            return
        self.sample_pose(t)

    def sample_pose(self, t):
        if self.true_pose is None:
            self.get_logger().warn(
                f'No pose on {self.get_parameter("pose_topic").value} yet',
                throttle_duration_sec=2.0)
            return
        pos = self.true_pose.position
        self.poses.append((t, pos.x, pos.y, _yaw(self.true_pose.orientation),
                           self.controller))

    def finish(self, result):
        self.done = True
        t_end = self._now()
        p = self.get_parameter
        if self.latest_path is None or len(self.latest_path.poses) < 2:
            self.get_logger().error(
                f'No {p("path_topic").value} received, nothing to evaluate')
            return
        self.ref_xy = [(ps.pose.position.x, ps.pose.position.y)
                       for ps in self.latest_path.poses]

        labels = pm.segment_labels(self.ref_xy, p('min_swath_length').value)
        zones = pm.turn_zones(self.ref_xy, labels, p('turn_margin').value)
        tracker = pm.CrossTrackTracker(self.ref_xy)
        rows = []
        for t, x, y, yaw, controller in self.poses:
            i, err, s = tracker.project(x, y)
            rows.append((t - self.t_start, x, y, yaw, err, s, zones[i],
                         controller))

        thr = p('jerk_threshold').value
        summary = {
            'run_label': self.run_label,
            'controller': p('controller').value,
            'result': result,
            'completion_time_s': t_end - self.t_start,
            'path_length_ref_m': pm.path_length(self.ref_xy),
            'path_length_robot_m': pm.path_length([r[1:3] for r in rows]),
            'samples': len(rows),
            'coverage_pct': pm.coverage_percent(
                self.ref_xy, labels, [r[1:3] for r in rows],
                p('cutting_width').value),
        }
        summary.update(pm.switch_stats(self.nav))
        boundaries = [s0 for _, s0, _ in
                      segments(self.ref_xy, p('min_swath_length').value)[1:]]
        summary.update(pm.transition_stats(
            [r[0] + self.t_start for r in rows], [r[5] for r in rows],
            [r[4] for r in rows], boundaries, [n[:3] for n in self.nav]))
        summary.update(pm.cross_track_stats([r[4] for r in rows],
                                            [r[6] for r in rows]))
        nav_w = [(n[0], n[2]) for n in self.nav]
        for prefix, series in (('cmd', self.cmd_w), ('odom', self.odom_w),
                               ('nav', nav_w)):
            stats = pm.angular_smoothness([s[0] for s in series],
                                          [s[1] for s in series],
                                          jerk_threshold=thr)
            summary.update({f'{prefix}_{k}': v for k, v in stats.items()})

        self.write(rows, labels, zones, summary)
        self.get_logger().info(
            f'Run {result} in {summary["completion_time_s"]:.1f} s: '
            f'CTE RMS swath {summary["cte_rms_swath"]:.3f} m, '
            f'turn {summary["cte_rms_turn"]:.3f} m, '
            f'coverage {summary["coverage_pct"]:.1f} %')

    def write(self, rows, labels, zones, summary):
        os.makedirs(self.output_dir, exist_ok=True)
        base = os.path.join(self.output_dir, self.run_label)

        with open(f'{base}_trajectory.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'cte', 's', 'segment',
                        'controller'])
            for r in rows:
                w.writerow([f'{r[0]:.3f}', f'{r[1]:.4f}', f'{r[2]:.4f}',
                            f'{r[3]:.4f}', f'{r[4]:.4f}', f'{r[5]:.3f}', r[6],
                            r[7]])

        with open(f'{base}_commands.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'source', 'v', 'w', 'controller'])
            series = [(t, 'nav', v, wz, c) for t, v, wz, c in self.nav]
            series += [(t, 'cmd', v, wz, '') for t, v, wz in self.cmd]
            for t, src, v, wz, c in sorted(series, key=lambda r: r[0]):
                w.writerow([f'{t - self.t_start:.3f}', src, f'{v:.4f}',
                            f'{wz:.4f}', c])

        with open(f'{base}_reference.csv', 'w', newline='') as f:
            w = csv.writer(f)
            # Labels belong to segments; the last point repeats its segment's.
            w.writerow(['x', 'y', 'segment', 'zone'])
            for (x, y), k, z in zip(self.ref_xy, labels + labels[-1:],
                                    zones + zones[-1:]):
                w.writerow([f'{x:.4f}', f'{y:.4f}', k, z])

        summary_file = os.path.join(self.output_dir, 'summary.csv')
        new = not os.path.exists(summary_file)
        with open(summary_file, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
            if new:
                w.writeheader()
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v)
                        for k, v in summary.items()})


def main():
    rclpy.init()
    node = MetricsRecorder()
    try:
        while rclpy.ok() and not (node.done and node.exit_on_finish):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
