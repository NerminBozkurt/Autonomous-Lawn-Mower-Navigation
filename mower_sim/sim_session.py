"""
Run the simulation on behalf of the control panel.

SimSession owns three things:
- the simulation itself, `ros2 launch mower_sim nav2_sim.launch.py`, run as
  its own process group so it can be torn down completely;
- a metrics_recorder process per FollowPath goal, writing to metrics/;
- a ROS node that sends the coverage path to controller_server's FollowPath
  action, republishes it on /coverage_path, and tracks the goal's feedback.

The panel calls launch(), start(), stop() and reset() and polls snapshot();
none of them blocks, since launching and tearing down run in a worker thread.
No tkinter here, so the session can be driven and tested without a display.
"""

import csv
import os
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from mower_sim.coverage_path import boustrophedon_path
from mower_sim.run_mowing_path import to_path_msg
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

CONTROLLERS = ('rpp', 'mppi', 'dwb')
ODOMETRY = ('encoder', 'ground_truth')

# Session states.
IDLE = 'idle'            # no simulation running
LAUNCHING = 'launching'  # simulation starting, waiting for Nav2
READY = 'ready'          # Nav2 active, robot at the start, no goal yet
RUNNING = 'running'      # FollowPath goal executing
STOPPED = 'stopped'      # goal canceled by the user, robot halted mid-path
FINISHED = 'finished'    # goal ended on its own (succeeded or aborted)
SHUTTING_DOWN = 'shutting down'

RESULTS = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


def _stop_process_group(proc):
    """SIGINT, then SIGTERM, then SIGKILL the process group of proc."""
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5),
                      (signal.SIGKILL, 5)):
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=wait)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def _kill_stray_gzserver():
    # gzserver can outlive its launch and keep the Gazebo master port, so the
    # next launch would attach its robot to the old world. Match the exact
    # process name only.
    if subprocess.run(['pkill', '-9', '-x', 'gzserver'],
                      check=False).returncode == 0:
        time.sleep(3.0)


class _PathNode(Node):
    """Sends the coverage path and follows the goal; used by SimSession."""

    def __init__(self, session):
        super().__init__('mower_control_panel')
        self.session = session
        self.path = to_path_msg(boustrophedon_path(
            num_swaths=3, swath_length=5.0, swath_spacing=0.75, step=0.05),
            'map')
        self.action = ActionClient(self, FollowPath, 'follow_path')
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        # Steady clock: /clock restarts at zero every time the simulation is
        # relaunched, and a sim-time timer would stall on that jump.
        self.create_timer(1.0, lambda: self.path_pub.publish(self.path),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))


class SimSession:

    def __init__(self, metrics_dir='metrics', log_dir=None):
        self.metrics_dir = os.path.abspath(metrics_dir)
        self.log_dir = log_dir or os.path.expanduser('~/.ros/log/mower_panel')
        os.makedirs(self.log_dir, exist_ok=True)

        self._lock = threading.Lock()
        self.state = IDLE
        self.message = 'Simulation not running.'
        self.config = None          # (controller, odometry, gazebo_gui, rviz)
        self.sim = None
        self.recorder = None
        self.run_label = None
        self.goal_handle = None
        self.goal_started = None    # wall time of the current goal's start
        self.elapsed_before = 0.0   # time spent in earlier, stopped goals
        self.distance_left = None
        self.speed = None
        self.result = None
        self.metrics = None
        self.resumed = False        # current goal continues a stopped one
        self.goal_path = None       # the path sent with the current goal

        rclpy.init()
        self.node = _PathNode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    # ------------------------------------------------------------ queries

    @property
    def path_length(self):
        poses = self.node.path.poses
        return sum(
            ((b.pose.position.x - a.pose.position.x) ** 2 +
             (b.pose.position.y - a.pose.position.y) ** 2) ** 0.5
            for a, b in zip(poses, poses[1:]))

    def _remaining_path(self, distance_left):
        """
        Return the tail of the coverage path that is distance_left long.

        A resumed goal gets only what is left of the path: sent whole, DWB
        (prune_distance 0.3 m) cannot find a robot that is metres along it.
        distance_left is FollowPath's feedback, the path length from the
        robot to the end, so it locates the robot on the path without being
        fooled by the U-turns folding the path back next to itself.
        """
        full = self.node.path
        poses = full.poses
        remaining = 0.0
        for i in range(len(poses) - 1, 0, -1):
            a, b = poses[i - 1].pose.position, poses[i].pose.position
            remaining += ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
            if remaining >= distance_left:
                break
        path = Path()
        path.header = full.header
        path.poses = poses[i - 1:]
        return path

    def snapshot(self):
        """Everything the panel shows, read consistently."""
        with self._lock:
            elapsed = self.elapsed_before
            if self.state == RUNNING and self.goal_started is not None:
                elapsed += time.monotonic() - self.goal_started
            return {
                'state': self.state,
                'message': self.message,
                'config': self.config,
                'distance_left': self.distance_left,
                'speed': self.speed,
                'elapsed': elapsed,
                'result': self.result,
                'metrics': self.metrics,
                'resumed': self.resumed,
            }

    # ------------------------------------------------------------ actions

    def launch(self, controller, odometry, gazebo_gui=True, rviz=True):
        """Start the simulation with this configuration (from IDLE)."""
        if controller not in CONTROLLERS or odometry not in ODOMETRY:
            raise ValueError(f'bad configuration {controller}, {odometry}')
        with self._lock:
            if self.state != IDLE:
                return False
            self._set(LAUNCHING, f'Starting simulation ({controller.upper()}, '
                                 f'{odometry} odometry)...')
            self.config = (controller, odometry, gazebo_gui, rviz)
        threading.Thread(target=self._launch_worker, daemon=True).start()
        return True

    def start(self):
        """Send the coverage path (READY) or resume it (STOPPED)."""
        with self._lock:
            if self.state not in (READY, STOPPED):
                return False
            # A resumed goal gets its own metrics_recorder run, which only
            # sees the part of the path driven after resuming.
            self.resumed = self.state == STOPPED
            if self.resumed and self.distance_left is not None:
                self.goal_path = self._remaining_path(self.distance_left)
            else:
                self.goal_path = self.node.path
            self._set(RUNNING, 'Sending the coverage path...')
            self.result = None
            self.metrics = None
        threading.Thread(target=self._send_goal, daemon=True).start()
        return True

    def stop(self):
        """Cancel the running goal; the robot halts, the simulation stays up."""
        with self._lock:
            handle = self.goal_handle
            if self.state != RUNNING or handle is None:
                return False
            self.message = 'Stopping the robot...'
        handle.cancel_goal_async()
        return True

    def reset(self, controller, odometry, gazebo_gui=True, rviz=True):
        """Tear the simulation down and relaunch it with this configuration."""
        with self._lock:
            if self.state in (LAUNCHING, SHUTTING_DOWN):
                return False
        threading.Thread(
            target=self._reset_worker,
            args=(controller, odometry, gazebo_gui, rviz), daemon=True).start()
        return True

    def shutdown(self, wait=True):
        """Stop the simulation (from any state but LAUNCHING)."""
        worker = threading.Thread(target=self._teardown, daemon=True)
        worker.start()
        if wait:
            worker.join()

    def close(self):
        """Shut everything down; call once when the panel exits."""
        self._teardown()
        # Stop the spin thread before destroying the node it is spinning.
        self.executor.shutdown(timeout_sec=5.0)
        self._spin.join(timeout=5.0)
        self.node.destroy_node()
        rclpy.try_shutdown()

    # ------------------------------------------------------------ workers

    def _set(self, state, message):
        self.state = state
        self.message = message

    def _launch_worker(self):
        # Nav2's bringup occasionally fails on its own (a lifecycle
        # transition times out); a fresh launch usually works.
        attempts = 3
        for attempt in range(1, attempts + 1):
            outcome = self._launch_once()
            if outcome != 'failed':
                return
            if attempt < attempts:
                sim, self.sim = self.sim, None
                _stop_process_group(sim)
                with self._lock:
                    self.message = (f'Nav2 failed to come up; retrying '
                                    f'({attempt + 1}/{attempts})...')
        self._teardown()
        with self._lock:
            self.message = ('Simulation failed to start; see '
                            f'{os.path.join(self.log_dir, "sim.log")}')

    def _launch_once(self):
        """Launch the simulation; return 'ready', 'failed' or 'cancelled'."""
        controller, odometry, gazebo_gui, rviz = self.config
        _kill_stray_gzserver()
        log_path = os.path.join(self.log_dir, 'sim.log')
        log = open(log_path, 'w')
        cmd = ['ros2', 'launch', 'mower_sim', 'nav2_sim.launch.py',
               f'controller:={controller}', f'odometry:={odometry}',
               f'gazebo_gui:={str(gazebo_gui).lower()}',
               f'rviz:={str(rviz).lower()}']
        sim = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        self.sim = sim
        # controller_server's action server comes up before Nav2 activates
        # it, and goals sent in that gap are rejected, so wait for the
        # lifecycle manager to report every node active. Either lifecycle
        # manager (map_server's or Nav2's) aborting means a failed bringup.
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            if self.sim is not sim:
                return 'cancelled'  # shut down from outside while launching
            if sim.poll() is not None:
                return 'failed'
            with open(log_path, errors='replace') as f:
                lines = [ln for ln in f if 'lifecycle_manager' in ln]
            if any('lifecycle_manager_navigation' in ln and
                   'Managed nodes are active' in ln for ln in lines):
                with self._lock:
                    self.elapsed_before = 0.0
                    self.distance_left = self.path_length
                    self.speed = 0.0
                    self._set(READY, 'Ready. Press Start to follow the path.')
                return 'ready'
            if any('Aborting bringup' in ln for ln in lines):
                return 'failed'
            time.sleep(1.0)
        return 'failed'

    def _send_goal(self):
        if not self.node.action.wait_for_server(timeout_sec=10.0):
            with self._lock:
                self._set(STOPPED, 'FollowPath action server not available.')
            return
        controller, odometry = self.config[:2]
        self.run_label = time.strftime(f'{controller}_{odometry}_%Y%m%d-%H%M%S')
        self._start_recorder(controller)

        goal = FollowPath.Goal()
        goal.path = self.goal_path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        future = self.node.action.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _start_recorder(self, controller):
        self._stop_recorder()
        log = open(os.path.join(self.log_dir, 'recorder.log'), 'w')
        self.recorder = subprocess.Popen(
            ['ros2', 'run', 'mower_sim', 'metrics_recorder', '--ros-args',
             '-p', 'use_sim_time:=true',
             '-p', f'output_dir:={self.metrics_dir}',
             '-p', f'run_label:={self.run_label}',
             '-p', f'controller:={controller}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        # Give it time to subscribe before the goal starts executing.
        time.sleep(2.0)

    def _stop_recorder(self):
        if self.recorder is not None:
            _stop_process_group(self.recorder)
            self.recorder = None

    def _on_goal_response(self, future):
        handle = future.result()
        with self._lock:
            if not handle.accepted:
                self._set(STOPPED, 'FollowPath goal rejected.')
                return
            self.goal_handle = handle
            self.goal_started = time.monotonic()
            self.message = 'Following the coverage path.'
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        with self._lock:
            self.distance_left = msg.feedback.distance_to_goal
            self.speed = msg.feedback.speed

    def _on_result(self, future):
        # Waiting for the recorder below must not block the executor.
        threading.Thread(target=self._finish_goal,
                         args=(future.result().status,), daemon=True).start()

    def _finish_goal(self, status):
        result = RESULTS.get(status, f'status {status}')
        # metrics_recorder writes its CSVs as soon as it sees the goal end.
        if self.recorder is not None:
            try:
                self.recorder.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                pass
        metrics = self._read_metrics()
        with self._lock:
            if self.goal_started is not None:
                self.elapsed_before += time.monotonic() - self.goal_started
            self.goal_started = None
            self.goal_handle = None
            self.speed = 0.0
            self.result = result
            self.metrics = metrics
            if status == GoalStatus.STATUS_CANCELED:
                self._set(STOPPED, 'Stopped. Start resumes the path; Reset '
                                   'puts the robot back at the start.')
            else:
                self._set(FINISHED, f'Path {result}. Reset to run again.')

    def _read_metrics(self):
        summary = os.path.join(self.metrics_dir, 'summary.csv')
        try:
            with open(summary, newline='') as f:
                rows = [r for r in csv.DictReader(f)
                        if r['run_label'] == self.run_label]
        except OSError:
            return None
        return rows[-1] if rows else None

    def _reset_worker(self, controller, odometry, gazebo_gui, rviz):
        self._teardown()
        self.launch(controller, odometry, gazebo_gui, rviz)

    def _teardown(self):
        with self._lock:
            if self.state == IDLE and self.sim is None:
                return
            self._set(SHUTTING_DOWN, 'Shutting the simulation down...')
            handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        self._stop_recorder()
        if self.sim is not None:
            _stop_process_group(self.sim)
            self.sim = None
        _kill_stray_gzserver()
        with self._lock:
            self.goal_handle = None
            self.goal_started = None
            self.elapsed_before = 0.0
            self.distance_left = None
            self.speed = None
            self.config = None
            self._set(IDLE, 'Simulation not running.')
