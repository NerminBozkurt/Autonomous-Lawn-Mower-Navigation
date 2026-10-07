"""
Follow a coverage path with FollowPath, switching controllers along the way.

This is what Nav2's ControllerSelector BT node does, done in Python: when the
robot passes a point where the schedule (path_segments.controller_schedule)
names another controller, the running FollowPath goal is preempted by a new
one carrying that controller_id. controller_server then swaps controllers
without stopping the robot (ControllerServer::updateGlobalPath).

Two things differ from the BT. The new goal carries only the rest of the
path, from the robot onwards: all three controllers lose the robot if handed
the whole path halfway along it (DWB prunes only the first prune_distance of
a new plan; RPP and MPPI search only the first few metres for the robot).
And Humble's BT navigator cannot follow a precomputed coverage path at all.

Progress comes from FollowPath's feedback, distance_to_goal: every goal ends
where the full path ends, so s = total length - distance_to_goal is the
robot's arc length along the full path whichever tail it is following.

The active controller is published on /active_controller (std_msgs/String,
transient local) for the metrics recorder.
"""

from action_msgs.msg import GoalStatus
from mower_sim.path_segments import cumulative_lengths, tail_start
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PathExecutor:
    """Runs one pass over a path; create a new one for every run."""

    def __init__(self, node, action_client, path, schedule,
                 goal_checker_id='general_goal_checker', on_feedback=None,
                 on_done=None, on_rejected=None, active_pub=None):
        """
        Set up a run; nothing is sent until start().

        path is the full nav_msgs/Path and schedule a list of
        (arc length, controller_id) starting at 0. Callbacks, all called
        from the node's executor: on_feedback(distance_left, speed,
        controller), on_done(status), on_rejected().
        """
        self.node = node
        self.action = action_client
        self.path = path
        self.xy = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        self.total = cumulative_lengths(self.xy)[-1]
        self.schedule = schedule
        self.goal_checker_id = goal_checker_id
        self.on_feedback = on_feedback
        self.on_done = on_done
        self.on_rejected = on_rejected
        self.active_pub = active_pub or node.create_publisher(
            String, '/active_controller', LATCHED)

        self.generation = 0      # bumps with every goal sent
        self.handle = None       # handle of the newest accepted goal
        self.pending = False     # a goal is sent but not yet accepted
        self.next_switch = None  # index into schedule of the next switch
        self.controller = None
        self.canceled = False
        self.finished = False

    def _controller_at(self, s):
        index = 0
        for i, (start, _) in enumerate(self.schedule):
            if start <= s + 1e-9:
                index = i
        return index

    def start(self, distance_left=None):
        """Send the first goal: the whole path, or its last distance_left."""
        if distance_left is None:
            distance_left = self.total
        index = self._controller_at(self.total - distance_left)
        self._send(distance_left, index)

    def cancel(self):
        """Stop the robot; on_done reports STATUS_CANCELED."""
        self.canceled = True
        if self.handle is not None:
            self.handle.cancel_goal_async()

    def _send(self, distance_left, index):
        self.generation += 1
        generation = self.generation
        self.controller = self.schedule[index][1]
        self.next_switch = index + 1 if index + 1 < len(self.schedule) else None
        self.pending = True

        path = Path()
        path.header = self.path.header
        path.poses = self.path.poses[tail_start(self.xy, distance_left):]
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller
        goal.goal_checker_id = self.goal_checker_id

        self.active_pub.publish(String(data=self.controller))
        future = self.action.send_goal_async(
            goal, feedback_callback=lambda msg: self._feedback(generation, msg))
        future.add_done_callback(lambda f: self._response(generation, f))

    def _response(self, generation, future):
        handle = future.result()
        if generation != self.generation:
            return
        self.pending = False
        if not handle.accepted:
            if self.on_rejected:
                self.on_rejected()
            return
        self.handle = handle
        if self.canceled:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self._result(generation, f))

    def _feedback(self, generation, msg):
        if generation != self.generation or self.finished:
            return
        distance_left = msg.feedback.distance_to_goal
        if self.on_feedback:
            self.on_feedback(distance_left, msg.feedback.speed,
                             self.controller)
        if self.pending or self.canceled or self.next_switch is None:
            return
        s = self.total - distance_left
        if s >= self.schedule[self.next_switch][0]:
            self._send(distance_left, self._controller_at(s))

    def _result(self, generation, future):
        # Goals superseded by a newer one end ABORTED; that is the switch,
        # not the run's outcome.
        if generation != self.generation:
            return
        self.finished = True
        status = future.result().status
        if self.canceled and status != GoalStatus.STATUS_SUCCEEDED:
            status = GoalStatus.STATUS_CANCELED
        if self.on_done:
            self.on_done(status)
