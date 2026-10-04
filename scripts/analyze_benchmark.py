#!/usr/bin/env python3
"""
Turn the metrics_recorder output of a benchmark into a table and a figure.

Reads <raw-dir>/summary.csv plus the per-run trajectory/reference CSVs and
writes into <out-dir>:
- summary_table.csv / summary_table.md: mean and standard deviation of each
  metric per controller, over all runs.
- trajectories.png: the reference path with one trace per controller (the
  run with the median overall cross-track RMS), plus a zoom on a U-turn.

    python3 scripts/analyze_benchmark.py --raw-dir results/benchmark/raw \\
        --out-dir results/benchmark
"""

import argparse
import csv
import os
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.axes_grid1.inset_locator import mark_inset  # noqa: E402

ORDER = ['rpp', 'mppi', 'dwb']
NAMES = {'rpp': 'RPP', 'mppi': 'MPPI', 'dwb': 'DWB'}
# Categorical slots 1-3 of the reference palette, which stay distinguishable
# for colour-blind readers even when all three overlap.
COLORS = {'rpp': '#2a78d6', 'mppi': '#eb6834', 'dwb': '#1baf7a'}
REF_COLOR = '#8a8984'
INK = '#0b0b0b'
INK_2 = '#52514e'
GRID = '#e4e3df'

# (column, header, unit, decimals, scale)
TABLE = [
    ('completion_time_s', 'Completion time', 's', 1, 1.0),
    ('cte_rms_swath', 'CTE RMS, swaths', 'cm', 1, 100.0),
    ('cte_max_swath', 'CTE max, swaths', 'cm', 1, 100.0),
    ('cte_rms_turn', 'CTE RMS, turns', 'cm', 1, 100.0),
    ('cte_max_turn', 'CTE max, turns', 'cm', 1, 100.0),
    ('cmd_ang_jerk_rms', 'Yaw jerk RMS (cmd)', 'rad/s³', 1, 1.0),
    ('cmd_ang_jerk_events', 'Yaw jerk events (cmd)', '#', 0, 1.0),
    ('odom_ang_jerk_rms', 'Yaw jerk RMS (odom)', 'rad/s³', 1, 1.0),
    ('coverage_pct', 'Coverage', '%', 1, 1.0),
]


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def fmt(values, decimals):
    if not values:
        return 'n/a'
    mean = statistics.fmean(values)
    if len(values) < 2:
        return f'{mean:.{decimals}f}'
    return f'{mean:.{decimals}f} ± {statistics.stdev(values):.{decimals}f}'


def build_table(rows, out_dir):
    controllers = [c for c in ORDER if any(r['controller'] == c for r in rows)]
    per = {c: [r for r in rows if r['controller'] == c] for c in controllers}

    def values(c, col, scale):
        out = []
        for r in per[c]:
            v = float(r[col])
            if v == v:  # skip NaN
                out.append(v * scale)
        return out

    with open(os.path.join(out_dir, 'summary_table.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['controller', 'runs', 'succeeded'] +
                   [f'{col}_{s}' for col, *_ in TABLE for s in ('mean', 'std')])
        for c in controllers:
            row = [c, len(per[c]),
                   sum(r['result'] == 'succeeded' for r in per[c])]
            for col, _, _, dec, scale in TABLE:
                v = values(c, col, scale)
                row += [f'{statistics.fmean(v):.{dec + 2}f}' if v else '',
                        f'{statistics.stdev(v):.{dec + 2}f}' if len(v) > 1 else '']
            w.writerow(row)

    lines = ['| Metric | ' + ' | '.join(NAMES[c] for c in controllers) + ' |',
             '|---|' + '---:|' * len(controllers)]
    lines.append('| Runs (succeeded) | ' + ' | '.join(
        f'{len(per[c])} ({sum(r["result"] == "succeeded" for r in per[c])})'
        for c in controllers) + ' |')
    for col, header, unit, dec, scale in TABLE:
        cells = []
        for c in controllers:
            if col != 'completion_time_s':
                cells.append(fmt(values(c, col, scale), dec))
                continue
            # A completion time only means something for runs that completed.
            done = [float(r[col]) for r in per[c] if r['result'] == 'succeeded']
            failed = [float(r[col]) for r in per[c] if r['result'] != 'succeeded']
            cell = fmt(done, dec) if done else '—'
            if failed:
                cell += f' (aborted at {fmt(failed, dec)})'
            cells.append(cell)
        lines.append(f'| {header} [{unit}] | ' + ' | '.join(cells) + ' |')
    table = '\n'.join(lines) + '\n'
    with open(os.path.join(out_dir, 'summary_table.md'), 'w') as f:
        f.write(table)
    return table


def representative_runs(rows):
    """Per controller, the run with the median overall cross-track RMS."""
    picks = {}
    for c in ORDER:
        runs = sorted((r for r in rows if r['controller'] == c),
                      key=lambda r: float(r['cte_rms_all']))
        if runs:
            picks[c] = runs[(len(runs) - 1) // 2]['run_label']
    return picks


def load_xy(path, x='x', y='y'):
    rows = read_csv(path)
    return [float(r[x]) for r in rows], [float(r[y]) for r in rows]


def style_axes(ax):
    ax.set_facecolor('white')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.set_aspect('equal')


def plot(rows, raw_dir, out_dir, zoom):
    picks = representative_runs(rows)
    ref_x, ref_y = load_xy(os.path.join(raw_dir,
                                        f'{next(iter(picks.values()))}_reference.csv'))
    traces = {c: load_xy(os.path.join(raw_dir, f'{label}_trajectory.csv'))
              for c, label in picks.items()}
    result = {r['run_label']: r['result'] for r in rows}

    fig = plt.figure(figsize=(11, 4.6), facecolor='white')
    ax = fig.add_axes([0.06, 0.20, 0.57, 0.62])
    axz = fig.add_axes([0.69, 0.20, 0.29, 0.62])

    for a in (ax, axz):
        style_axes(a)
        a.plot(ref_x, ref_y, color=REF_COLOR, linewidth=2.0, linestyle=(0, (4, 3)),
               label='Reference path', zorder=2)
        for c, (x, y) in traces.items():
            ok = result[picks[c]] == 'succeeded'
            a.plot(x, y, color=COLORS[c], linewidth=2.0, zorder=3,
                   label=f'{NAMES[c]} ({picks[c].split("_")[-1]}'
                         f'{"" if ok else ", aborted ×"})')
            if not ok:
                a.plot(x[-1], y[-1], marker='X', markersize=10, color=COLORS[c],
                       markeredgecolor='white', markeredgewidth=1.5, zorder=4)

    x0, x1, y0, y1 = zoom
    axz.set_xlim(x0, x1)
    axz.set_ylim(y0, y1)
    axz.set_title('U-turn, zoomed', loc='left', fontsize=10, color=INK_2)
    mark_inset(ax, axz, loc1=2, loc2=3, fc='none', ec=INK_2, lw=0.8,
               linestyle=':')

    ax.set_xlabel('x [m]', color=INK_2, fontsize=9)
    ax.set_ylabel('y [m]', color=INK_2, fontsize=9)
    axz.set_xlabel('x [m]', color=INK_2, fontsize=9)
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, -0.16), ncol=4,
              frameon=False, fontsize=9, labelcolor=INK)
    fig.text(0.06, 0.94, 'Controllers tracking the same coverage path',
             fontsize=13, color=INK, weight='bold')
    fig.text(0.06, 0.885, 'Ground-truth robot position; for each controller '
             'the run with the median cross-track RMS of its runs. '
             '× marks where FollowPath aborted.',
             fontsize=9.5, color=INK_2)
    path = os.path.join(out_dir, 'trajectories.png')
    fig.savefig(path, dpi=160, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--raw-dir', default='results/benchmark/raw')
    parser.add_argument('--out-dir', default='results/benchmark')
    parser.add_argument('--zoom', nargs=4, type=float,
                        default=[3.9, 5.6, -0.35, 1.1],
                        metavar=('X0', 'X1', 'Y0', 'Y1'),
                        help='zoom window, default: the first U-turn')
    args = parser.parse_args()

    rows = read_csv(os.path.join(args.raw_dir, 'summary.csv'))
    os.makedirs(args.out_dir, exist_ok=True)
    print(build_table(rows, args.out_dir))
    print('figure:', plot(rows, args.raw_dir, args.out_dir, args.zoom))


if __name__ == '__main__':
    main()
