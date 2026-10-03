import math

from mower_sim.coverage_path import boustrophedon_path
import pytest

CUTTING_WIDTH = 0.75
STEP = 0.05


def _angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _swath_ys(poses, length):
    """Distinct y values of poses lying on a straight swath (heading +/-x)."""
    ys = set()
    for x, y, yaw in poses:
        on_swath = 0.0 <= x <= length and (
            _angle_diff(yaw, 0.0) < 1e-9 or _angle_diff(yaw, math.pi) < 1e-9)
        if on_swath:
            ys.add(round(y, 6))
    return sorted(ys)


@pytest.fixture
def path():
    return boustrophedon_path(3, 5.0, CUTTING_WIDTH, step=STEP)


def test_starts_at_origin_and_ends_after_last_swath(path):
    assert path[0] == (0.0, 0.0, 0.0)
    x, y, yaw = path[-1]
    assert x == pytest.approx(5.0)
    assert y == pytest.approx(2 * CUTTING_WIDTH)
    assert _angle_diff(yaw, 0.0) < 1e-9


def test_points_are_dense_and_continuous(path):
    for (x0, y0, _), (x1, y1, _) in zip(path, path[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        assert 0.0 < d <= STEP + 1e-9


def test_yaw_follows_direction_of_travel(path):
    for (x0, y0, yaw0), (x1, y1, yaw1) in zip(path, path[1:]):
        travel = math.atan2(y1 - y0, x1 - x0)
        # On an arc the chord direction is the mean of the two tangents.
        mean = math.atan2(math.sin(yaw0) + math.sin(yaw1),
                          math.cos(yaw0) + math.cos(yaw1))
        assert _angle_diff(travel, mean) < 1e-6


def test_swaths_are_one_cutting_width_apart_so_no_uncut_strip(path):
    ys = _swath_ys(path, 5.0)
    assert ys == pytest.approx([0.0, CUTTING_WIDTH, 2 * CUTTING_WIDTH])
    for lo, hi in zip(ys, ys[1:]):
        assert hi - lo <= CUTTING_WIDTH + 1e-9


def test_turns_alternate_ends_and_stay_within_turn_radius(path):
    radius = CUTTING_WIDTH / 2.0
    xs = [p[0] for p in path]
    assert max(xs) == pytest.approx(5.0 + radius, abs=1e-6)
    assert min(xs) == pytest.approx(-radius, abs=1e-6)


def test_smaller_radius_inserts_straight_between_arcs():
    poses = boustrophedon_path(2, 5.0, CUTTING_WIDTH, turn_radius=0.2,
                               step=STEP)
    on_connector = [p for p in poses
                    if p[0] == pytest.approx(5.2)
                    and _angle_diff(p[2], math.pi / 2.0) < 1e-9]
    ys = [p[1] for p in on_connector]
    assert min(ys) == pytest.approx(0.2)
    assert max(ys) == pytest.approx(CUTTING_WIDTH - 0.2)


def test_single_swath_is_one_straight_line():
    poses = boustrophedon_path(1, 2.0, CUTTING_WIDTH, step=0.5)
    assert poses == pytest.approx(
        [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0),
         (1.5, 0.0, 0.0), (2.0, 0.0, 0.0)])


@pytest.mark.parametrize('kwargs', [
    {'num_swaths': 0},
    {'swath_length': 0.0},
    {'swath_spacing': -1.0},
    {'step': 0.0},
    {'turn_radius': 0.0},
    {'turn_radius': CUTTING_WIDTH},
])
def test_rejects_invalid_arguments(kwargs):
    args = {'num_swaths': 3, 'swath_length': 5.0,
            'swath_spacing': CUTTING_WIDTH}
    args.update(kwargs)
    with pytest.raises(ValueError):
        boustrophedon_path(**args)
