import math

from mower_sim import path_metrics as pm
from mower_sim.coverage_grid import CoverageGrid
from mower_sim.coverage_path import boustrophedon_path
import pytest


def _path_xy():
    return [(x, y) for x, y, _ in boustrophedon_path(3, 5.0, 0.75, step=0.05)]


def test_field_is_the_rectangle_the_swaths_tile():
    grid = CoverageGrid(_path_xy(), cutting_width=0.75)
    xs = [p[0] for p in grid.boundary]
    ys = [p[1] for p in grid.boundary]
    assert min(xs) == pytest.approx(0.0)
    assert max(xs) == pytest.approx(5.0)
    assert min(ys) == pytest.approx(-0.375)
    assert max(ys) == pytest.approx(1.875)


def test_nothing_swept_is_zero_percent():
    assert CoverageGrid(_path_xy()).percent == 0.0


def test_driving_the_path_covers_the_field():
    ref = _path_xy()
    grid = CoverageGrid(ref, cutting_width=0.75)
    for x, y in ref:
        grid.add_pose(x, y)
    assert grid.percent > 99.0


def test_matches_the_end_of_run_metric():
    # A trajectory 0.2 m off to the side leaves a strip unmowed; the live
    # grid and metrics_recorder's coverage_percent must agree on how much.
    ref = _path_xy()
    traj = [(x, y + 0.2) for x, y in ref]
    grid = CoverageGrid(ref, cutting_width=0.75)
    for x, y in traj:
        grid.add_pose(x, y)
    expected = pm.coverage_percent(ref, pm.segment_labels(ref), traj, 0.75)
    assert 80.0 < expected < 99.0
    assert grid.percent == pytest.approx(expected, abs=1.0)


def test_polygon_field():
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    grid = CoverageGrid(polygon=square, cutting_width=1.0)
    for i in range(41):
        grid.add_pose(0.1 * i, 2.0)   # one 1 m wide pass across the middle
    assert grid.percent == pytest.approx(25.0, abs=1.0)


def test_clear():
    grid = CoverageGrid(_path_xy())
    grid.add_pose(1.0, 0.0)
    grid.add_pose(3.0, 0.0)
    assert grid.percent > 0.0
    grid.clear()
    assert grid.percent == 0.0


def test_swept_cells_for_drawing():
    grid = CoverageGrid(_path_xy(), cutting_width=0.75)
    grid.add_pose(1.0, 0.0)
    grid.add_pose(2.0, 0.0)
    xs, ys, size = grid.swept_cells(0.05)
    assert size == pytest.approx(0.05)
    assert len(xs) > 0
    # Every drawn cell is near the swept band.
    assert all(0.5 < x < 2.5 and abs(y) < 0.45 for x, y in zip(xs, ys))
    assert not math.isnan(grid.percent)
