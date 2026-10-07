"""
Run the simulation on behalf of the control panel.

SimSession owns three things:
- the simulation itself, `ros2 launch mower_sim nav2_sim.launch.py`, run as
  its own process group so it can be torn down completely;
- a metrics_recorder process per run, writing to metrics/;
- a ROS node that republishes the coverage path on /coverage_path and runs
  it through a PathExecutor, which sends the FollowPath goals and, in the
  switching configuration, changes controller between rows and turns.

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
from mower_sim.path_executor import LATCHED, PathExecutor
from mower_sim.path_segments import controller_schedule, segments
from mower_sim.run_mowing_path import to_path_msg
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

CONTROLLERS = ('rpp', 'mppi', 'dwb', 'switching')
# Plugin ids of the controllers in nav2_switching.yaml.
SWITCHING_PLUGINS = ('RPP', 'MPPI', 'DWB')
ODOMETRY = ('encoder', 'ground_truth')

# Session states.
IDLE = 'idle'            # no simulation running
LAUNCHING = 'launching'  # simulation starting, waiting for Nav2
READY = 'ready'          # Nav2 active, robot at the start, no goal yet
RUNNING = 'running'      # following the path
STOPPED = 'stopped'      # run canceled by the user, robot halted mid-path
FINISHED = 'finished'    # run ended on its own (succeeded or aborted)
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
    """Publishes the coverage path and talks to FollowPath for SimSession."""

    def __init__(self, session):
        super().__init__('mower_control_panel')
        self.poses = boustrophedon_path(
            num_swaths=3, swath_length=5.0, swath_spacing=0.75, step=0.05)
        self.path = to_path_msg(self.poses, 'map')
        self.action = ActionClient(self, FollowPath, 'follow_path')
        self.active_pub = self.create_publisher(String, '/active_controller',
                                                LATCHED)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.create_subscription(Float32, '/coverage_percent',
                                 session._on_coverage, 10)
        self.create_subscription(Float32MultiArray, '/odometry_error',
                                 session._on_odometry_error, 10)
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
        # (controller, odometry, gazebo_gui, rviz, row, turn); row and turn
        # are the switching configuration's plugin ids, None otherwise.
        self.config = None
        self.sim = None
        self.recorder = None
        self.run_label = None
        self.path_executor = None
        self.goal_started = None    # wall time the current run started moving
        self.elapsed_before = 0.0   # time spent in earlier, stopped runs
        self.distance_left = None
        self.speed = None
        self.active_controller = None
        self.result = None
        self.metrics = None
        self.coverage = None        # live coverage from coverage_monitor, %
        # (position error, largest this run [m], heading error [rad]) of the
        # odometry estimate against the true pose, from robot_trail_publisher
        self.odometry_error = None
        self.resumed = False        # current run continues a stopped one

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
                'active_controller': self.active_controller,
                'elapsed': elapsed,
                'result': self.result,
                'metrics': self.metrics,
                'resumed': self.resumed,
                'coverage': self.coverage,
                'odometry_error': self.odometry_error,
            }

    # ------------------------------------------------------------ actions

    def launch(self, controller, odometry, gazebo_gui=True, rviz=True,
               row=None, turn=None):
        """
        Start the simulation with this configuration (from IDLE).

        controller 'switching' follows the rows with plugin row and the turns
        with plugin turn (each one of SWITCHING_PLUGINS).
        """
        if controller not in CONTROLLERS or odometry not in ODOMETRY:
            raise ValueError(f'bad configuration {controller}, {odometry}')
        if controller == 'switching':
            if row not in SWITCHING_PLUGINS or turn not in SWITCHING_PLUGINS:
                raise ValueError(f'bad switching pair {row}, {turn}')
            name = f'switching (rows {row}, turns {turn})'
        else:
            row = turn = None
            name = controller.upper()
        with self._lock:
            if self.state != IDLE:
                return False
            self._set(LAUNCHING, f'Starting simulation ({name}, '
                                 f'{odometry} odometry)...')
            self.config = (controller, odometry, gazebo_gui, rviz, row, turn)
        threading.Thread(target=self._launch_worker, daemon=True).start()
        return True

    def start(self):
        """Send the coverage path (READY) or resume it (STOPPED)."""
        with self._lock:
            if self.state not in (READY, STOPPED):
                return False
            # A resumed run gets its own metrics_recorder run, which only
            # sees the part of the path driven after resuming.
            self.resumed = self.state == STOPPED
            distance_left = self.distance_left if self.resumed else None
            self._set(RUNNING, 'Sending the coverage path...')
            self.result = None
            self.metrics = None
        threading.Thread(target=self._start_run, args=(distance_left,),
                         daemon=True).start()
        return True

    def stop(self):
        """Cancel the run; the robot halts, the simulation stays up."""
        with self._lock:
            run = self.path_executor
            if self.state != RUNNING or run is None:
                return False
            self.message = 'Stopping the robot...'
        run.cancel()
        return True

    def reset(self, controller, odometry, gazebo_gui=True, rviz=True,
              row=None, turn=None):
        """Tear the simulation down and relaunch it with this configuration."""
        with self._lock:
            if self.state in (LAUNCHING, SHUTTING_DOWN):
                return False
        threading.Thread(
            target=self._reset_worker,
            args=(controller, odometry, gazebo_gui, rviz, row, turn),
            daemon=True).start()
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
        controller, odometry, gazebo_gui, rviz = self.config[:4]
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

    def _schedule(self):
        controller, _, _, _, row, turn = self.config
        if controller == 'switching':
            xy = [(x, y) for x, y, _ in self.node.poses]
            return controller_schedule(segments(xy), row, turn)
        # The single-controller configs name their only plugin FollowPath.
        return [(0.0, 'FollowPath')]

    def _start_run(self, distance_left):
        if not self.node.action.wait_for_server(timeout_sec=10.0):
            with self._lock:
                self._set(STOPPED, 'FollowPath action server not available.')
            return
        controller, odometry, _, _, row, turn = self.config
        label = (f'switching-{row}-{turn}'.lower() if controller == 'switching'
                 else controller)
        self.run_label = time.strftime(f'{label}_{odometry}_%Y%m%d-%H%M%S')
        self._start_recorder(label)

        run = PathExecutor(
            self.node, self.node.action, self.node.path, self._schedule(),
            on_feedback=self._on_feedback, on_done=self._on_done,
            on_rejected=self._on_rejected, active_pub=self.node.active_pub)
        with self._lock:
            self.path_executor = run
            self.message = 'Following the coverage path.'
        run.start(distance_left)

    def _start_recorder(self, label):
        self._stop_recorder()
        log = open(os.path.join(self.log_dir, 'recorder.log'), 'w')
        self.recorder = subprocess.Popen(
            ['ros2', 'run', 'mower_sim', 'metrics_recorder', '--ros-args',
             '-p', 'use_sim_time:=true',
             '-p', f'output_dir:={self.metrics_dir}',
             '-p', f'run_label:={self.run_label}',
             '-p', f'controller:={label}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        # Give it time to subscribe before the goal starts executing.
        time.sleep(2.0)

    def _stop_recorder(self):
        if self.recorder is not None:
            _stop_process_group(self.recorder)
            self.recorder = None

    def _on_rejected(self):
        with self._lock:
            self.path_executor = None
            self._set(STOPPED, 'FollowPath goal rejected.')

    def _on_coverage(self, msg):
        with self._lock:
            if self.state not in (IDLE, LAUNCHING, SHUTTING_DOWN):
                self.coverage = msg.data

    def _on_odometry_error(self, msg):
        with self._lock:
            if self.state not in (IDLE, LAUNCHING, SHUTTING_DOWN) \
                    and len(msg.data) == 3:
                self.odometry_error = tuple(msg.data)

    def _on_feedback(self, distance_left, speed, controller):
        with self._lock:
            if self.goal_started is None:
                self.goal_started = time.monotonic()
            self.distance_left = distance_left
            self.speed = speed
            self.active_controller = controller

    def _on_done(self, status):
        # Waiting for the recorder below must not block the executor.
        threading.Thread(target=self._finish_run, args=(status,),
                         daemon=True).start()

    def _finish_run(self, status):
        result = RESULTS.get(status, f'status {status}')
        # metrics_recorder writes its CSVs as soon as it sees the run end.
        if self.recorder is not None:
            try:
                self.recorder.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                pass
        metrics = self._read_metrics()
        with self._lock:
            if self.state in (SHUTTING_DOWN, IDLE):
                return  # canceled by a teardown, which sets the state itself
            if self.goal_started is not None:
                self.elapsed_before += time.monotonic() - self.goal_started
            self.goal_started = None
            self.path_executor = None
            self.speed = 0.0
            self.active_controller = None
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

    def _reset_worker(self, controller, odometry, gazebo_gui, rviz, row, turn):
        self._teardown()
        self.launch(controller, odometry, gazebo_gui, rviz, row, turn)

    def _teardown(self):
        with self._lock:
            if self.state == IDLE and self.sim is None:
                return
            self._set(SHUTTING_DOWN, 'Shutting the simulation down...')
            run = self.path_executor
        if run is not None:
            run.cancel()
        self._stop_recorder()
        if self.sim is not None:
            _stop_process_group(self.sim)
            self.sim = None
        _kill_stray_gzserver()
        with self._lock:
            self.path_executor = None
            self.goal_started = None
            self.elapsed_before = 0.0
            self.distance_left = None
            self.speed = None
            self.active_controller = None
            self.coverage = None
            self.odometry_error = None
            self.config = None
            self._set(IDLE, 'Simulation not running.')
