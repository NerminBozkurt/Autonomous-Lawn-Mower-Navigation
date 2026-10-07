"""
Path-following metrics for the controller benchmark.

Kept free of ROS so it can be unit tested and reused offline: the
metrics_recorder node only collects samples and hands them to these functions.

Conventions:
- The reference is a dense polyline of (x, y) points, as published on
  /coverage_path. Segment i joins point i and point i + 1.
- Cross-track error is the signed perpendicular distance from the robot to
  the reference, positive when the robot is to the LEFT of the direction of
  travel.
- Every reference segment is labelled 'swath' or 'turn'. A swath is a
  straight run at least min_swath_length long; everything else (the arcs and
  any short straight connecting them) is part of a turn. This only looks at
  the geometry, so it works for any boustrophedon path, not just ours.
"""

import math

import numpy as np

SWATH = 'swath'
TURN = 'turn'


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def segment_labels(ref_xy, min_swath_length=1.0, heading_tol=1e-3):
    """Label each segment of the reference polyline 'swath' or 'turn'."""
    ref = np.asarray(ref_xy, dtype=float)
    if len(ref) < 2:
        return []
    d = np.diff(ref, axis=0)
    headings = np.arctan2(d[:, 1], d[:, 0])
    lengths = np.hypot(d[:, 0], d[:, 1])

    labels = [TURN] * len(d)
    start = 0
    for i in range(1, len(d) + 1):
        if i < len(d) and abs(_wrap(headings[i] - headings[start])) < heading_tol:
            continue
        # Close the straight run [start, i).
        if lengths[start:i].sum() >= min_swath_length - 1e-9:
            labels[start:i] = [SWATH] * (i - start)
        start = i
    return labels


def turn_zones(ref_xy, labels, margin):
    """
    Widen every turn by `margin` metres of path on both sides.

    A controller that cuts the corner never gets near the turn arc, so by
    nearest point its whole manoeuvre would be scored against the ends of
    the swaths. Treating the last and first `margin` metres of each swath as
    part of the turn keeps entry and exit transients out of the swath
    statistics, which then measure steady straight-line tracking.
    """
    ref = np.asarray(ref_xy, dtype=float)
    if not labels:
        return []
    seg_len = np.hypot(*np.diff(ref, axis=0).T)
    mid = np.cumsum(seg_len) - seg_len / 2.0
    turn_mid = mid[np.asarray(labels) == TURN]
    if margin <= 0.0 or turn_mid.size == 0:
        return list(labels)
    out = []
    for k, m in zip(labels, mid):
        near = np.min(np.abs(turn_mid - m)) <= margin
        out.append(TURN if k == TURN or near else SWATH)
    return out


class CrossTrackTracker:
    """
    Project robot positions onto the reference, one sample at a time.

    The nearest segment is searched only in a window around the previous
    match, so the projection follows the robot along the path and cannot
    jump to an adjacent swath (they are only one cutting width apart). The
    first call, and any call after the robot has strayed beyond the window,
    falls back to a global search.
    """

    def __init__(self, ref_xy, window=2.0):
        ref = np.asarray(ref_xy, dtype=float)
        if len(ref) < 2:
            raise ValueError('reference path needs at least two points')
        self._a = ref[:-1]
        d = np.diff(ref, axis=0)
        self._len = np.hypot(d[:, 0], d[:, 1])
        self._len_sq = np.maximum(self._len ** 2, 1e-12)
        self._d = d
        self._s0 = np.concatenate(([0.0], np.cumsum(self._len)[:-1]))
        self._window = window
        self._last = None

    def _closest(self, x, y, lo, hi):
        a = self._a[lo:hi]
        d = self._d[lo:hi]
        t = ((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / self._len_sq[lo:hi]
        t = np.clip(t, 0.0, 1.0)
        px = a[:, 0] + t * d[:, 0]
        py = a[:, 1] + t * d[:, 1]
        dist = np.hypot(x - px, y - py)
        k = int(np.argmin(dist))
        return lo + k, float(t[k]), float(dist[k])

    def project(self, x, y):
        """Return (segment_index, signed_cross_track_error, arc_length)."""
        n = len(self._a)
        if self._last is None:
            i, t, dist = self._closest(x, y, 0, n)
        else:
            s = self._s0[self._last]
            lo = int(np.searchsorted(self._s0, s - self._window, side='left'))
            hi = int(np.searchsorted(self._s0, s + self._window, side='right'))
            i, t, dist = self._closest(x, y, max(lo, 0), max(hi, lo + 1))
            if dist > self._window:
                i, t, dist = self._closest(x, y, 0, n)
        self._last = i
        dx, dy = self._d[i]
        cross = dx * (y - self._a[i, 1]) - dy * (x - self._a[i, 0])
        sign = 1.0 if cross >= 0.0 else -1.0
        return i, sign * dist, float(self._s0[i] + t * self._len[i])


def _rms(values):
    v = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(v ** 2))) if v.size else float('nan')


def _max_abs(values):
    v = np.asarray(values, dtype=float)
    return float(np.max(np.abs(v))) if v.size else float('nan')


def cross_track_stats(errors, kinds):
    """RMS and max |error| over all samples, swath samples and turn samples."""
    errors = np.asarray(errors, dtype=float)
    kinds = np.asarray(kinds)
    out = {}
    for name, mask in (('all', np.ones(len(errors), dtype=bool)),
                       (SWATH, kinds == SWATH),
                       (TURN, kinds == TURN)):
        out[f'cte_rms_{name}'] = _rms(errors[mask])
        out[f'cte_max_{name}'] = _max_abs(errors[mask])
    return out


def angular_smoothness(times, omegas, rate=20.0, jerk_threshold=None):
    """
    Angular acceleration and jerk statistics of a yaw-rate signal.

    The signal is resampled onto a uniform grid at `rate` Hz before
    differentiating, so irregular message timing does not show up as jerk.
    With jerk_threshold set, also count the samples whose |jerk| exceeds it
    (the "sudden changes").
    """
    t = np.asarray(times, dtype=float)
    w = np.asarray(omegas, dtype=float)
    nan = float('nan')
    out = {'ang_acc_rms': nan, 'ang_acc_max': nan,
           'ang_jerk_rms': nan, 'ang_jerk_max': nan}
    if jerk_threshold is not None:
        out['ang_jerk_events'] = 0
    if len(t) < 4 or t[-1] - t[0] < 3.0 / rate:
        return out
    order = np.argsort(t, kind='stable')
    t, w = t[order], w[order]
    grid = np.arange(t[0], t[-1], 1.0 / rate)
    wu = np.interp(grid, t, w)
    acc = np.diff(wu) * rate
    jerk = np.diff(acc) * rate
    out.update(ang_acc_rms=_rms(acc), ang_acc_max=_max_abs(acc),
               ang_jerk_rms=_rms(jerk), ang_jerk_max=_max_abs(jerk))
    if jerk_threshold is not None:
        out['ang_jerk_events'] = int(np.sum(np.abs(jerk) > jerk_threshold))
    return out


def transition_stats(times, arc, cte, boundaries, nav, pre=1.0, post=3.0):
    """
    Tracking and command behaviour around each row/turn boundary.

    times, arc and cte are the pose samples (time, arc length along the
    reference, cross-track error); boundaries the arc lengths where a row
    meets a turn; nav the controller's raw output as (time, v, w) samples,
    i.e. /cmd_vel_nav, before the velocity smoother hides any discontinuity.

    A boundary is crossed at the first sample with arc >= boundary; its
    window runs from pre seconds before to post seconds after. Per window:
    the largest |cross-track error| and the largest step in v and in w
    between consecutive nav samples. Returned: the number of boundaries
    crossed, the mean and max of the per-window |cte| maxima, and the max
    of the v and w steps over all windows. Every configuration is scored at
    the same places, whether or not it switches controller there.
    """
    t = np.asarray(times, dtype=float)
    s = np.asarray(arc, dtype=float)
    e = np.abs(np.asarray(cte, dtype=float))
    nan = float('nan')
    out = {'boundaries': 0, 'trans_cte_mean': nan, 'trans_cte_max': nan,
           'trans_nav_dv_max': nan, 'trans_nav_dw_max': nan}
    if len(t) == 0:
        return out
    nav = np.asarray(nav, dtype=float).reshape(-1, 3)
    cte_max, dv_max, dw_max = [], [], []
    for b in boundaries:
        crossed = np.nonzero(s >= b)[0]
        if crossed.size == 0:
            continue
        tb = t[crossed[0]]
        in_window = (t >= tb - pre) & (t <= tb + post)
        cte_max.append(float(e[in_window].max()))
        w = nav[(nav[:, 0] >= tb - pre) & (nav[:, 0] <= tb + post)]
        if len(w) >= 2:
            dv_max.append(float(np.abs(np.diff(w[:, 1])).max()))
            dw_max.append(float(np.abs(np.diff(w[:, 2])).max()))
    out['boundaries'] = len(cte_max)
    if cte_max:
        out['trans_cte_mean'] = float(np.mean(cte_max))
        out['trans_cte_max'] = float(np.max(cte_max))
    if dv_max:
        out['trans_nav_dv_max'] = float(np.max(dv_max))
        out['trans_nav_dw_max'] = float(np.max(dw_max))
    return out


def switch_stats(nav, before=0.1, after=0.5):
    """
    Command discontinuity at every controller switch.

    nav is the controller's raw output as (time, v, w, controller) samples.
    A switch is a sample whose controller differs from the previous one; its
    window runs from `before` seconds before to `after` seconds after it, to
    catch both the step at the handover and the new controller's first few
    commands. Returned: the number of switches and the largest step in v and
    in w between consecutive samples in any window. Unlike transition_stats,
    this is measured where the switch actually happens, wherever turn_lead
    and turn_lag put it.
    """
    nan = float('nan')
    out = {'switches': 0, 'switch_dv_max': nan, 'switch_dw_max': nan}
    if len(nav) < 2:
        return out
    t = np.array([n[0] for n in nav], dtype=float)
    v = np.array([n[1] for n in nav], dtype=float)
    w = np.array([n[2] for n in nav], dtype=float)
    c = [n[3] for n in nav]
    dv, dw = np.abs(np.diff(v)), np.abs(np.diff(w))
    dv_max, dw_max = [], []
    for i in range(1, len(c)):
        if c[i] == c[i - 1] or not c[i - 1]:
            continue
        # Steps j -> j + 1 with both samples inside the window.
        inside = (t[:-1] >= t[i] - before) & (t[1:] <= t[i] + after)
        dv_max.append(float(dv[inside].max()))
        dw_max.append(float(dw[inside].max()))
    out['switches'] = len(dv_max)
    if dv_max:
        out['switch_dv_max'] = max(dv_max)
        out['switch_dw_max'] = max(dw_max)
    return out


def _strip_mask(px, py, a, b, half_width, caps):
    """Cells within half_width of segment a-b; with caps=False, a flat strip."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return np.hypot(px - a[0], py - a[1]) <= half_width if caps else None
    t = ((px - a[0]) * dx + (py - a[1]) * dy) / len_sq
    if caps:
        tc = np.clip(t, 0.0, 1.0)
        return np.hypot(px - (a[0] + tc * dx), py - (a[1] + tc * dy)) <= half_width
    perp = np.abs((px - a[0]) * dy - (py - a[1]) * dx) / math.sqrt(len_sq)
    return (t >= 0.0) & (t <= 1.0) & (perp <= half_width)


def coverage_percent(ref_xy, labels, traj_xy, cutting_width, resolution=0.02):
    """
    Rough share of the intended mowing area the robot actually swept.

    Target area: a strip one cutting width wide centred on every swath
    segment of the reference (headland turns are not part of the lawn being
    measured). Swept area: every point within cutting_width / 2 of the
    robot's trajectory, i.e. the cutter treated as a disc of that diameter
    centred on the robot frame. Both are rasterised at `resolution`; the
    result is |target & swept| / |target| in percent.
    """
    ref = np.asarray(ref_xy, dtype=float)
    traj = np.asarray(traj_xy, dtype=float)
    half = cutting_width / 2.0
    swath = [i for i, k in enumerate(labels) if k == SWATH]
    if not swath or len(traj) == 0:
        return float('nan')

    pts = ref[np.unique(np.concatenate([swath, np.add(swath, 1)]))]
    x0, y0 = pts.min(axis=0) - half - resolution
    x1, y1 = pts.max(axis=0) + half + resolution
    xs = np.arange(x0, x1, resolution) + resolution / 2.0
    ys = np.arange(y0, y1, resolution) + resolution / 2.0
    px, py = np.meshgrid(xs, ys)

    target = np.zeros(px.shape, dtype=bool)
    for i in swath:
        m = _strip_mask(px, py, ref[i], ref[i + 1], half, caps=False)
        if m is not None:
            target |= m

    # Thin the trajectory to roughly the grid resolution; capsules between
    # consecutive kept points then cover the whole swept band.
    keep = [traj[0]]
    for p in traj[1:]:
        if math.hypot(p[0] - keep[-1][0], p[1] - keep[-1][1]) >= resolution:
            keep.append(p)
    if len(keep) == 1:
        keep.append(keep[0])

    swept = np.zeros(px.shape, dtype=bool)
    for a, b in zip(keep, keep[1:]):
        lo_x, hi_x = min(a[0], b[0]) - half, max(a[0], b[0]) + half
        lo_y, hi_y = min(a[1], b[1]) - half, max(a[1], b[1]) + half
        c0, c1 = np.searchsorted(xs, [lo_x, hi_x])
        r0, r1 = np.searchsorted(ys, [lo_y, hi_y])
        if c0 >= c1 or r0 >= r1:
            continue
        sub = _strip_mask(px[r0:r1, c0:c1], py[r0:r1, c0:c1], a, b, half,
                          caps=True)
        swept[r0:r1, c0:c1] |= sub

    total = int(target.sum())
    return 100.0 * int((target & swept).sum()) / total if total else float('nan')


def path_length(xy):
    p = np.asarray(xy, dtype=float)
    if len(p) < 2:
        return 0.0
    return float(np.hypot(*np.diff(p, axis=0).T).sum())
