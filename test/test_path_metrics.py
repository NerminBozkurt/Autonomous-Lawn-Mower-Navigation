import math

from mower_sim import path_metrics as pm
from mower_sim.coverage_path import boustrophedon_path
import pytest

CUTTING_WIDTH = 0.75


@pytest.fixture
def ref():
    return [(x, y) for x, y, _ in
            boustrophedon_path(3, 5.0, CUTTING_WIDTH, step=0.05)]


def test_labels_split_swaths_from_turns(ref):
    labels = pm.segment_labels(ref)
    assert len(labels) == len(ref) - 1
    # Three 5 m swaths of 100 segments each; the rest are the two U-turns.
    assert labels.count(pm.SWATH) == 300
    assert labels[:100] == [pm.SWATH] * 100
    assert labels[100] == pm.TURN
    assert labels[-100:] == [pm.SWATH] * 100


def test_short_straight_inside_turn_is_not_a_swath():
    ref = [(x, y) for x, y, _ in
           boustrophedon_path(2, 5.0, CUTTING_WIDTH, turn_radius=0.2,
                              step=0.05)]
    labels = pm.segment_labels(ref)
    assert labels.count(pm.SWATH) == 200


def test_turn_zones_extend_turns_by_margin(ref):
    labels = pm.segment_labels(ref)
    zones = pm.turn_zones(ref, labels, margin=1.0)
    # 1 m either side of each turn moves 20 swath segments per turn side.
    assert zones.count(pm.SWATH) == 300 - 4 * 20
    assert zones[:80] == [pm.SWATH] * 80
    assert zones[80] == pm.TURN
    assert pm.turn_zones(ref, labels, margin=0.0) == labels


def test_cross_track_sign_and_magnitude(ref):
    tracker = pm.CrossTrackTracker(ref)
    i, err, s = tracker.project(2.0, 0.1)
    assert err == pytest.approx(0.1)
    assert s == pytest.approx(2.0)
    _, err, _ = tracker.project(2.5, -0.2)
    assert err == pytest.approx(-0.2)


def test_tracker_does_not_jump_to_neighbouring_swath(ref):
    tracker = pm.CrossTrackTracker(ref)
    tracker.project(4.0, 0.0)
    # Closer to swath 2 (y = 0.75) than to swath 1, but the robot is still
    # progressing along swath 1: the windowed search keeps it there.
    _, err, s = tracker.project(4.2, 0.4)
    assert s < 5.0
    assert err == pytest.approx(0.4)


def test_tracker_follows_the_turn_arc(ref):
    tracker = pm.CrossTrackTracker(ref)
    for x in (3.0, 4.0, 5.0):
        tracker.project(x, 0.0)
    r = CUTTING_WIDTH / 2.0
    # Apex of the first U-turn, 5 cm outside the arc.
    i, err, _ = tracker.project(5.0 + r + 0.05, r)
    assert pm.segment_labels(ref)[i] == pm.TURN
    assert err == pytest.approx(-0.05, abs=1e-3)


def test_cross_track_stats_split_by_kind():
    stats = pm.cross_track_stats([0.1, -0.1, 0.3, -0.4],
                                 [pm.SWATH, pm.SWATH, pm.TURN, pm.TURN])
    assert stats['cte_rms_swath'] == pytest.approx(0.1)
    assert stats['cte_max_swath'] == pytest.approx(0.1)
    assert stats['cte_rms_turn'] == pytest.approx(math.sqrt(0.125))
    assert stats['cte_max_turn'] == pytest.approx(0.4)
    assert stats['cte_max_all'] == pytest.approx(0.4)
    assert math.isnan(pm.cross_track_stats([], [])['cte_rms_all'])


def test_constant_turn_rate_has_no_jerk():
    t = [i * 0.033 for i in range(200)]
    stats = pm.angular_smoothness(t, [0.5] * 200, jerk_threshold=1.0)
    assert stats['ang_acc_rms'] == pytest.approx(0.0, abs=1e-9)
    assert stats['ang_jerk_max'] == pytest.approx(0.0, abs=1e-9)
    assert stats['ang_jerk_events'] == 0


def test_linear_ramp_has_constant_acceleration_and_no_jerk():
    t = [i * 0.01 for i in range(500)]
    w = [0.2 * ti for ti in t]
    stats = pm.angular_smoothness(t, w)
    assert stats['ang_acc_rms'] == pytest.approx(0.2, rel=1e-6)
    assert stats['ang_jerk_rms'] == pytest.approx(0.0, abs=1e-6)


def test_step_in_turn_rate_counts_as_jerk_event():
    t = [i * 0.05 for i in range(100)]
    w = [0.0] * 50 + [1.0] * 50
    stats = pm.angular_smoothness(t, w, rate=20.0, jerk_threshold=10.0)
    # One 1 rad/s step at 20 Hz: acc pulse of 20 rad/s^2, jerk +-400.
    assert stats['ang_acc_max'] == pytest.approx(20.0)
    assert stats['ang_jerk_max'] == pytest.approx(400.0)
    assert stats['ang_jerk_events'] == 2


def test_perfect_tracking_covers_everything(ref):
    labels = pm.segment_labels(ref)
    pct = pm.coverage_percent(ref, labels, ref, CUTTING_WIDTH,
                              resolution=0.02)
    assert pct > 99.0


def test_offset_tracking_leaves_strip_uncut(ref):
    labels = pm.segment_labels(ref)
    # Every pass 0.2 m off to the right: each 0.75 m strip loses ~0.2 m,
    # except where the next pass overlaps it.
    shifted = [(x, y - 0.2) for x, y in ref]
    pct = pm.coverage_percent(ref, labels, shifted, CUTTING_WIDTH)
    assert 80.0 < pct < 95.0


def test_missing_a_swath_drops_a_third(ref):
    labels = pm.segment_labels(ref)
    first_two = ref[:len(ref) - 100]
    pct = pm.coverage_percent(ref, labels, first_two, CUTTING_WIDTH)
    # Slightly over two thirds: the disc-shaped cutter at the end of the
    # second U-turn already reaches into the start of the third swath.
    assert 200.0 / 3.0 < pct < 200.0 / 3.0 + 3.0


def test_path_length(ref):
    assert pm.path_length(ref) == pytest.approx(
        15.0 + 2 * math.pi * CUTTING_WIDTH / 2.0, rel=1e-3)


def test_transition_stats_window_and_steps():
    # Robot at 1 m/s along s; boundary at s = 5 m, crossed at t = 5 s.
    times = [0.1 * i for i in range(100)]
    arc = list(times)
    cte = [0.3 if 4.5 <= t <= 7.0 else 0.01 for t in times]
    cte[10] = 0.9   # far outside the window, must be ignored
    nav = [(0.1 * i, 1.0, 0.0) for i in range(100)]
    nav[55] = (5.5, 1.0, 0.8)   # a 0.8 rad/s step right after the boundary
    out = pm.transition_stats(times, arc, cte, [5.0], nav)
    assert out['boundaries'] == 1
    assert out['trans_cte_max'] == pytest.approx(0.3)
    assert out['trans_nav_dw_max'] == pytest.approx(0.8)
    assert out['trans_nav_dv_max'] == pytest.approx(0.0)


def test_transition_stats_skips_boundaries_never_reached():
    times = [0.1 * i for i in range(30)]
    out = pm.transition_stats(times, times, [0.0] * 30, [1.0, 50.0], [])
    assert out['boundaries'] == 1
    assert math.isnan(out['trans_nav_dw_max'])


def test_switch_stats():
    nav = [(0.1 * i, 1.0, 0.0, 'RPP') for i in range(20)]
    nav += [(2.0 + 0.1 * i, 0.1 * i, 0.05 * i, 'MPPI') for i in range(20)]
    out = pm.switch_stats(nav)
    assert out['switches'] == 1
    assert out['switch_dv_max'] == pytest.approx(1.0)     # 1.0 -> 0.0
    assert out['switch_dw_max'] == pytest.approx(0.05)


def test_switch_stats_without_switches():
    out = pm.switch_stats([(0.1 * i, 1.0, 0.0, 'RPP') for i in range(20)])
    assert out['switches'] == 0
    assert math.isnan(out['switch_dv_max'])
