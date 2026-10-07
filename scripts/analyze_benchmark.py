#!/usr/bin/env python3
"""
Turn the metrics_recorder output of a benchmark into a table and a figure.

Reads <raw-dir>/summary.csv plus the per-run trajectory/reference CSVs and
writes into <out-dir>:
- summary_table.csv / summary_table.md: mean and standard deviation of each
  metric per controller, over all runs.
- trajectories.png: the reference path with one trace per controller (the
  run with the median overall cross-track RMS), plus a zoom on a U-turn.
- transitions.png: the controller's raw output (v and w, before the velocity
  smoother) around the first row-to-turn boundary, same runs; this is where
  a switching configuration hands over and its transients show. Written
  when the runs logged their commands (<run>_commands.csv).

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

ORDER = ['rpp', 'mppi', 'dwb', 'rpp-only', 'mppi-only', 'dwb-only',
         'switching-RPP-MPPI', 'switching-RPP-MPPI:0.5:0.5',
         'switching-RPP-DWB']
NAMES = {'rpp': 'RPP', 'mppi': 'MPPI', 'dwb': 'DWB',
         'rpp-only': 'RPP only', 'mppi-only': 'MPPI only',
         'dwb-only': 'DWB only',
         'switching-RPP-MPPI': 'Switching RPP/MPPI',
         'switching-RPP-MPPI:0.5:0.5': 'Switching RPP/MPPI, early',
         'switching-RPP-DWB': 'Switching RPP/DWB'}
# Categorical slots of the reference palette, in its fixed order. A
# controller keeps its colour whether it runs alone from its own config or
# from the switching one; the switching configurations take slots 4 and 5.
PALETTE = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300',
           '#4a3aa7', '#e34948']
COLORS = {'rpp': PALETTE[0], 'mppi': PALETTE[1], 'dwb': PALETTE[2],
          'rpp-only': PALETTE[0], 'mppi-only': PALETTE[1],
          'dwb-only': PALETTE[2], 'switching-RPP-MPPI': PALETTE[3],
          'switching-RPP-MPPI:0.5:0.5': PALETTE[4],
          'switching-RPP-DWB': PALETTE[5]}
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
    ('switches', 'Controller switches', '#', 0, 1.0),
    ('switch_dv_max', 'Largest v step at a switch (raw)', 'm/s', 2, 1.0),
    ('switch_dw_max', 'Largest w step at a switch (raw)', 'rad/s', 2, 1.0),
    ('trans_cte_max', 'CTE max around row/turn boundaries', 'cm', 1, 100.0),
    ('nav_ang_jerk_rms', 'Yaw jerk RMS (raw controller output)', 'rad/s³', 1,
     1.0),
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
    controllers = configurations(rows)
    per = {c: [r for r in rows if r['controller'] == c] for c in controllers}

    def values(c, col, scale):
        out = []
        for r in per[c]:
            if not r.get(col):  # column missing from older runs
                continue
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

    lines = ['| Metric | ' + ' | '.join(name(c) for c in controllers) + ' |',
             '|---|' + '---:|' * len(controllers)]
    lines.append('| Runs (succeeded) | ' + ' | '.join(
        f'{len(per[c])} ({sum(r["result"] == "succeeded" for r in per[c])})'
        for c in controllers) + ' |')
    for col, header, unit, dec, scale in TABLE:
        if not any(values(c, col, scale) for c in controllers):
            continue  # a metric these runs did not record
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


def configurations(rows):
    """Return the configurations present in rows, known ones first."""
    seen = []
    for r in rows:
        if r['controller'] not in seen:
            seen.append(r['controller'])
    return ([c for c in ORDER if c in seen] +
            [c for c in seen if c not in ORDER])


def name(c):
    return NAMES.get(c, c)


def color(c, rows):
    if c in COLORS:
        return COLORS[c]
    # Unknown configurations take the palette slots no known one uses.
    free = [p for p in PALETTE if p not in
            {COLORS.get(k) for k in configurations(rows)}]
    unknown = [k for k in configurations(rows) if k not in COLORS]
    return free[unknown.index(c) % len(free)]


def representative_runs(rows):
    """Per controller, the run with the median overall cross-track RMS."""
    picks = {}
    for c in configurations(rows):
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

    fig = plt.figure(figsize=(11, 5.2), facecolor='white')
    ax = fig.add_axes([0.06, 0.30, 0.57, 0.52])
    axz = fig.add_axes([0.69, 0.30, 0.29, 0.52])

    for a in (ax, axz):
        style_axes(a)
        a.plot(ref_x, ref_y, color=REF_COLOR, linewidth=2.0, linestyle=(0, (4, 3)),
               label='Reference path', zorder=2)
        for c, (x, y) in traces.items():
            ok = result[picks[c]] == 'succeeded'
            a.plot(x, y, color=color(c, rows), linewidth=2.0, zorder=3,
                   label=f'{name(c)} ({picks[c].split("_")[-1]}'
                         f'{"" if ok else ", aborted ×"})')
            if not ok:
                a.plot(x[-1], y[-1], marker='X', markersize=10,
                       color=color(c, rows),
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
    # Below both panels, clear of the zoom's connector lines.
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower left', bbox_to_anchor=(0.06, 0.0),
               ncol=3, frameon=False, fontsize=9, labelcolor=INK)
    fig.text(0.06, 0.94, 'Controllers tracking the same coverage path',
             fontsize=13, color=INK, weight='bold')
    fig.text(0.06, 0.885, 'Ground-truth robot position; for each controller '
             'the run with the median cross-track RMS of its runs. '
             '× marks where FollowPath aborted. "Early": handover 0.5 m '
             'before and after each turn.',
             fontsize=9.5, color=INK_2)
    path = os.path.join(out_dir, 'trajectories.png')
    fig.savefig(path, dpi=160, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)
    return path


def first_boundary(reference_rows):
    """Arc length where the reference's first row ends and a turn begins."""
    s = 0.0
    for a, b in zip(reference_rows, reference_rows[1:]):
        if a['segment'] != 'swath':
            return s
        s += ((float(b['x']) - float(a['x'])) ** 2 +
              (float(b['y']) - float(a['y'])) ** 2) ** 0.5
    return None


def plot_transitions(rows, raw_dir, out_dir, before=2.0, after=4.0):
    picks = representative_runs(rows)
    curves = {}
    for c, label in picks.items():
        commands = os.path.join(raw_dir, f'{label}_commands.csv')
        if not os.path.exists(commands):
            continue
        s_b = first_boundary(read_csv(os.path.join(raw_dir,
                                                   f'{label}_reference.csv')))
        crossed = [float(r['t']) for r in
                   read_csv(os.path.join(raw_dir, f'{label}_trajectory.csv'))
                   if float(r['s']) >= s_b]
        if s_b is None or not crossed:
            continue
        t_b = crossed[0]
        nav = [r for r in read_csv(commands) if r['source'] == 'nav'
               and -before <= float(r['t']) - t_b <= after]
        curves[c] = nav, t_b
    if not curves:
        return None

    fig = plt.figure(figsize=(11, 6.2), facecolor='white')
    axv = fig.add_axes([0.07, 0.56, 0.9, 0.29])
    axw = fig.add_axes([0.07, 0.19, 0.9, 0.29], sharex=axv)
    for ax, key, label in ((axv, 'v', 'v [m/s]'), (axw, 'w', 'ω [rad/s]')):
        style_axes(ax)
        ax.set_aspect('auto')
        ax.axvline(0.0, color=INK_2, linewidth=1.0, linestyle=':', zorder=1)
        ax.set_ylabel(label, color=INK_2, fontsize=9)
        for c, (nav, t_b) in curves.items():
            t = [float(r['t']) - t_b for r in nav]
            y = [float(r[key]) for r in nav]
            ax.plot(t, y, color=color(c, rows), linewidth=2.0, zorder=3,
                    label=name(c) if ax is axv else None)
            # Mark the handovers of a switching run.
            for i in range(1, len(nav)):
                if nav[i]['controller'] != nav[i - 1]['controller']:
                    ax.plot(t[i], y[i], marker='o', markersize=8,
                            color=color(c, rows), markeredgecolor='white',
                            markeredgewidth=2.0, zorder=4)
    axv.tick_params(labelbottom=False)
    axw.set_xlabel('time from reaching the first turn [s]', color=INK_2,
                   fontsize=9)
    axv.text(0.02, 1.0, 'row → turn boundary', transform=axv.get_xaxis_transform(),
             fontsize=8.5, color=INK_2, va='bottom')
    axv.legend(loc='upper left', bbox_to_anchor=(0.0, -1.55), ncol=3,
               frameon=False, fontsize=9, labelcolor=INK)
    fig.text(0.07, 0.94, 'Controller output entering the first U-turn',
             fontsize=13, color=INK, weight='bold')
    fig.text(0.07, 0.895, 'Raw FollowPath output (/cmd_vel_nav, before the '
             'velocity smoother), same runs as trajectories.png. '
             'Dots mark a controller handover.', fontsize=9.5, color=INK_2)
    path = os.path.join(out_dir, 'transitions.png')
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
    transitions = plot_transitions(rows, args.raw_dir, args.out_dir)
    if transitions:
        print('figure:', transitions)


if __name__ == '__main__':
    main()
