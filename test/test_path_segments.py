import math

from mower_sim.coverage_path import boustrophedon_path
from mower_sim.path_segments import (controller_schedule, cumulative_lengths,
                                     ROW, segments, tail_start, TURN)
import pytest

TURN_LEN = math.pi * 0.375  # semicircle of the default 0.75 m spacing


def _xy():
    return [(x, y) for x, y, _ in boustrophedon_path(3, 5.0, 0.75, step=0.05)]


def test_rows_and_turns_alternate():
    segs = segments(_xy())
    assert [k for k, _, _ in segs] == [ROW, TURN, ROW, TURN, ROW]
    lengths = [s1 - s0 for _, s0, s1 in segs]
    assert lengths[0] == pytest.approx(5.0)
    assert lengths[1] == pytest.approx(TURN_LEN, abs=0.01)
    assert segs[-1][2] == pytest.approx(cumulative_lengths(_xy())[-1])


def test_single_controller_never_switches():
    assert controller_schedule(segments(_xy()), 'RPP', 'RPP') == [(0.0, 'RPP')]


def test_switching_schedule():
    sched = controller_schedule(segments(_xy()), 'RPP', 'MPPI')
    assert [c for _, c in sched] == ['RPP', 'MPPI', 'RPP', 'MPPI', 'RPP']
    assert sched[1][0] == pytest.approx(5.0)
    assert sched[2][0] == pytest.approx(5.0 + TURN_LEN, abs=0.01)


def test_lead_and_lag_widen_the_turns():
    sched = controller_schedule(segments(_xy()), 'RPP', 'MPPI',
                                turn_lead=0.5, turn_lag=0.3)
    assert sched[1][0] == pytest.approx(4.5)
    assert sched[2][0] == pytest.approx(5.0 + TURN_LEN + 0.3, abs=0.01)


def test_overlapping_turn_windows_merge():
    # Lead and lag longer than half the middle row swallow it: the turn
    # controller runs from 3 m before the first turn to 3 m after the last.
    sched = controller_schedule(segments(_xy()), 'RPP', 'MPPI',
                                turn_lead=3.0, turn_lag=3.0)
    assert [c for _, c in sched] == ['RPP', 'MPPI', 'RPP']
    assert sched[1][0] == pytest.approx(2.0)
    assert sched[2][0] == pytest.approx(2 * 5.0 + 2 * TURN_LEN + 3.0,
                                        abs=0.01)


def test_tail_start():
    xy = _xy()
    s = cumulative_lengths(xy)
    total = s[-1]
    assert tail_start(xy, total) == 0
    i = tail_start(xy, total - 4.6)   # robot 4.6 m along the first row
    assert xy[i][0] == pytest.approx(4.6, abs=0.05)
    assert xy[i][1] == pytest.approx(0.0)
    assert total - s[i] >= total - 4.6 - 1e-9
