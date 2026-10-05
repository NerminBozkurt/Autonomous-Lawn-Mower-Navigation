"""
Incremental coverage of a field by a mower, on a raster grid.

The field to mow is either an explicit polygon or, by default, derived from
the coverage path the same way path_metrics.coverage_percent does it: one
cutting width wide strip centred on every swath, headland turns excluded.
For the boustrophedon paths of coverage_path.py those strips tile a
rectangle, which is then the field boundary.

The cutter is a disc of cutting_width diameter centred on the robot frame,
again as in path_metrics, so the live percentage converges to the figure
metrics_recorder reports at the end of a run.

Pure numpy, no ROS, so it can be unit tested.
"""

import math

from mower_sim import path_metrics as pm
import numpy as np


def _inside_polygon(px, py, polygon):
    """Even-odd rule: True for the grid points inside the polygon."""
    inside = np.zeros(px.shape, dtype=bool)
    n = len(polygon)
    for i in range(n):
        (x0, y0), (x1, y1) = polygon[i], polygon[(i + 1) % n]
        if y0 == y1:
            continue
        crosses = (py >= min(y0, y1)) & (py < max(y0, y1))
        x_at = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (px < x_at)
    return inside


class CoverageGrid:
    """Field mask plus the cells swept so far, on one grid."""

    def __init__(self, ref_xy=None, cutting_width=0.75, polygon=None,
                 resolution=0.025, margin=1.0):
        """
        Build the grid around the field.

        Give either polygon, a list of (x, y) field corners, or ref_xy, the
        coverage path whose swaths define the field. margin leaves room
        around the field for the headland turns, so they are drawn too.
        """
        self.half = cutting_width / 2.0
        self.resolution = resolution
        if polygon:
            self.boundary = [tuple(map(float, p)) for p in polygon]
            pts = np.asarray(self.boundary, dtype=float)
        else:
            ref = np.asarray(ref_xy, dtype=float)
            labels = pm.segment_labels(ref)
            swath = [i for i, k in enumerate(labels) if k == pm.SWATH]
            if not swath:
                raise ValueError('coverage path has no swaths')
            pts = ref[np.unique(np.concatenate([swath, np.add(swath, 1)]))]
            (x0, y0), (x1, y1) = pts.min(axis=0), pts.max(axis=0)
            # The strips extend half a cutting width across the swaths only;
            # for axis-aligned swaths that is the rectangle below.
            horizontal = (x1 - x0) >= (y1 - y0)
            dx, dy = (0.0, self.half) if horizontal else (self.half, 0.0)
            self.boundary = [(x0 - dx, y0 - dy), (x1 + dx, y0 - dy),
                             (x1 + dx, y1 + dy), (x0 - dx, y1 + dy)]

        lo = pts.min(axis=0) - self.half - margin
        hi = pts.max(axis=0) + self.half + margin
        self.xs = np.arange(lo[0], hi[0], resolution) + resolution / 2.0
        self.ys = np.arange(lo[1], hi[1], resolution) + resolution / 2.0
        self.px, self.py = np.meshgrid(self.xs, self.ys)

        if polygon:
            self.target = _inside_polygon(self.px, self.py, self.boundary)
        else:
            self.target = np.zeros(self.px.shape, dtype=bool)
            for i in swath:
                m = pm._strip_mask(self.px, self.py, ref[i], ref[i + 1],
                                   self.half, caps=False)
                if m is not None:
                    self.target |= m
        self.swept = np.zeros(self.px.shape, dtype=bool)
        self.last = None

    def add_pose(self, x, y):
        """Sweep the cutter from the previous pose to (x, y)."""
        if self.last is None:
            self.last = (x, y)
            self._sweep(self.last, self.last)
            return
        # Thin to the grid resolution, as coverage_percent does.
        if math.hypot(x - self.last[0], y - self.last[1]) < self.resolution:
            return
        self._sweep(self.last, (x, y))
        self.last = (x, y)

    def _sweep(self, a, b):
        c0, c1 = np.searchsorted(self.xs, [min(a[0], b[0]) - self.half,
                                           max(a[0], b[0]) + self.half])
        r0, r1 = np.searchsorted(self.ys, [min(a[1], b[1]) - self.half,
                                           max(a[1], b[1]) + self.half])
        if c0 >= c1 or r0 >= r1:
            return
        self.swept[r0:r1, c0:c1] |= pm._strip_mask(
            self.px[r0:r1, c0:c1], self.py[r0:r1, c0:c1], a, b, self.half,
            caps=True)

    def clear(self):
        self.swept[:] = False
        self.last = None

    @property
    def percent(self):
        total = int(self.target.sum())
        if not total:
            return float('nan')
        return 100.0 * int((self.target & self.swept).sum()) / total

    def swept_cells(self, cell=0.05):
        """
        Centres of the swept area on a coarser grid of size cell.

        For drawing: a coarse cell counts as swept if any fine cell in it is.
        """
        k = max(1, int(round(cell / self.resolution)))
        rows, cols = self.swept.shape
        r, c = rows - rows % k, cols - cols % k
        coarse = self.swept[:r, :c].reshape(r // k, k, c // k, k).any(axis=(1, 3))
        iy, ix = np.nonzero(coarse)
        x0 = self.xs[0] - self.resolution / 2.0
        y0 = self.ys[0] - self.resolution / 2.0
        size = k * self.resolution
        return (x0 + (ix + 0.5) * size, y0 + (iy + 0.5) * size, size)
