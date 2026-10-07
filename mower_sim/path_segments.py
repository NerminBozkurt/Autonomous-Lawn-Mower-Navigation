"""
Split a coverage path into rows and turns and schedule a controller for each.

A switching configuration follows the rows (swaths) with one controller and
the headland turns with another. This module decides where along the path
each controller takes over; path_executor carries it out by preempting the
FollowPath goal at those points.

Positions along the path are arc lengths s from the path's start. A turn
spans [s0, s1]; with turn_lead and turn_lag the turn controller takes over
turn_lead metres before the turn and hands back turn_lag metres after it.

Pure Python, no ROS, so it can be unit tested.
"""

import math

from mower_sim import path_metrics as pm

ROW = 'row'
TURN = 'turn'


def cumulative_lengths(xy):
    """Arc length from the start to every pose of the path."""
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s


def segments(xy, min_swath_length=1.0):
    """
    Return the path as a list of (kind, s_start, s_end), kind ROW or TURN.

    Rows are the straight swaths, as labelled by path_metrics.segment_labels;
    everything between them is a turn.
    """
    labels = pm.segment_labels(xy, min_swath_length)
    if not labels:
        return []
    s = cumulative_lengths(xy)
    out = []
    for i, label in enumerate(labels):
        kind = ROW if label == pm.SWATH else TURN
        if out and out[-1][0] == kind:
            out[-1] = (kind, out[-1][1], s[i + 1])
        else:
            out.append((kind, s[i], s[i + 1]))
    return out


def controller_schedule(segs, row_controller, turn_controller,
                        turn_lead=0.0, turn_lag=0.0):
    """
    Return [(s, controller)]: from arc length s on, use that controller.

    The first entry is always at s = 0. With row_controller ==
    turn_controller the schedule has that one entry, i.e. no switching.
    """
    if not segs:
        return [(0.0, row_controller)]
    end = segs[-1][2]
    # Turn intervals, widened by lead and lag and merged where they meet.
    turns = []
    for kind, s0, s1 in segs:
        if kind != TURN:
            continue
        a, b = max(0.0, s0 - turn_lead), min(end, s1 + turn_lag)
        if turns and a <= turns[-1][1]:
            turns[-1] = (turns[-1][0], max(turns[-1][1], b))
        else:
            turns.append((a, b))

    schedule = [(0.0, row_controller)]
    for a, b in turns:
        schedule.append((a, turn_controller))
        if b < end:
            schedule.append((b, row_controller))
    # Drop entries that do not change the controller, and zero-length ones.
    out = []
    for s, c in schedule:
        if out and out[-1][0] == s:
            out[-1] = (s, c)
        elif not out or out[-1][1] != c:
            out.append((s, c))
    return out


def tail_start(xy, distance_left):
    """
    Index of the first pose of the path's tail that is distance_left long.

    FollowPath's feedback reports distance_to_goal, the path length from the
    robot to the end. Cutting the path there locates the robot on it without
    being fooled by U-turns folding the path back next to itself.
    """
    remaining = 0.0
    for i in range(len(xy) - 1, 0, -1):
        (x0, y0), (x1, y1) = xy[i - 1], xy[i]
        remaining += math.hypot(x1 - x0, y1 - y0)
        if remaining >= distance_left:
            return i - 1
    return 0
