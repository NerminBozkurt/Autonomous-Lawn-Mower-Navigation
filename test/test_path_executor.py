"""PathExecutor's switching logic, against a fake FollowPath action client."""

import types

from action_msgs.msg import GoalStatus
from mower_sim.coverage_path import boustrophedon_path
from mower_sim.path_executor import PathExecutor
from mower_sim.path_segments import (controller_schedule, cumulative_lengths,
                                     segments)
from mower_sim.run_mowing_path import to_path_msg
import pytest


class _Future:
    def __init__(self, value=None):
        self.value = value
        self.callbacks = []

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def resolve(self, value):
        self.value = value
        for cb in self.callbacks:
            cb(self)


class _Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = _Future()
        self.canceled = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.canceled = True

    def finish(self, status):
        self.result_future.resolve(types.SimpleNamespace(status=status))


class _FakeAction:
    """Records goals; the test accepts them and feeds feedback by hand."""

    def __init__(self):
        self.goals = []   # (goal, feedback_callback, response future)

    def send_goal_async(self, goal, feedback_callback):
        future = _Future()
        self.goals.append((goal, feedback_callback, future))
        return future

    def accept(self, i=-1):
        handle = _Handle()
        self.goals[i][2].resolve(handle)
        return handle

    def feedback(self, distance_left, i=-1):
        fb = types.SimpleNamespace(distance_to_goal=distance_left, speed=0.5)
        self.goals[i][1](types.SimpleNamespace(feedback=fb))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


def _setup(row='RPP', turn='MPPI'):
    poses = boustrophedon_path(3, 5.0, 0.75, step=0.05)
    path = to_path_msg(poses, 'map')
    xy = [(x, y) for x, y, _ in poses]
    total = cumulative_lengths(xy)[-1]
    schedule = controller_schedule(segments(xy), row, turn)
    action, pub, done = _FakeAction(), _Pub(), []
    ex = PathExecutor(None, action, path, schedule, on_done=done.append,
                      active_pub=pub)
    return ex, action, pub, done, total


def _length(goal):
    pts = [(p.pose.position.x, p.pose.position.y) for p in goal.path.poses]
    return cumulative_lengths(pts)[-1]


def test_first_goal_is_the_whole_path_with_the_row_controller():
    ex, action, pub, _, total = _setup()
    ex.start()
    goal = action.goals[0][0]
    assert goal.controller_id == 'RPP'
    assert _length(goal) == pytest.approx(total)
    assert pub.sent == ['RPP']


def test_switches_to_the_turn_controller_with_the_rest_of_the_path():
    ex, action, pub, _, total = _setup()
    ex.start()
    action.accept()
    action.feedback(total - 4.0)      # 4 m along the first row: no switch
    assert len(action.goals) == 1
    action.feedback(total - 5.02)     # just past the first turn's start
    assert len(action.goals) == 2
    goal = action.goals[1][0]
    assert goal.controller_id == 'MPPI'
    assert _length(goal) == pytest.approx(total - 5.02, abs=0.06)
    assert pub.sent == ['RPP', 'MPPI']


def test_no_second_switch_while_the_first_is_unanswered():
    ex, action, _, _, total = _setup()
    ex.start()
    action.accept()
    action.feedback(total - 5.02)
    action.feedback(total - 6.3, i=0)   # late feedback from the old goal
    assert len(action.goals) == 2


def test_preempted_goal_ending_aborted_is_not_the_outcome():
    ex, action, _, done, total = _setup()
    ex.start()
    first = action.accept()
    action.feedback(total - 5.02)
    second = action.accept()
    first.finish(GoalStatus.STATUS_ABORTED)   # what a preemption does
    assert done == []
    second.finish(GoalStatus.STATUS_SUCCEEDED)
    assert done == [GoalStatus.STATUS_SUCCEEDED]


def test_whole_run_switches_four_times():
    ex, action, pub, done, total = _setup()
    ex.start()
    action.accept()
    d = total
    while d > 0.3:
        d -= 0.05
        before = len(action.goals)
        action.feedback(d)
        if len(action.goals) > before:
            action.accept()
    assert pub.sent == ['RPP', 'MPPI', 'RPP', 'MPPI', 'RPP']


def test_single_controller_never_preempts():
    ex, action, pub, _, total = _setup(row='DWB', turn='DWB')
    ex.start()
    action.accept()
    for d in (total - 5.5, total - 10.0, 1.0):
        action.feedback(d)
    assert len(action.goals) == 1
    assert pub.sent == ['DWB']


def test_resume_starts_with_the_controller_for_that_point():
    ex, action, _, _, total = _setup()
    ex.start(distance_left=total - 5.5)   # robot inside the first turn
    goal = action.goals[0][0]
    assert goal.controller_id == 'MPPI'
    assert _length(goal) == pytest.approx(total - 5.5, abs=0.06)


def test_cancel_reports_canceled():
    ex, action, _, done, _ = _setup()
    ex.start()
    handle = action.accept()
    ex.cancel()
    assert handle.canceled
    handle.finish(GoalStatus.STATUS_ABORTED)
    assert done == [GoalStatus.STATUS_CANCELED]
