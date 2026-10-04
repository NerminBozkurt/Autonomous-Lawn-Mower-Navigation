#!/usr/bin/env python3
"""
Run every controller over the same coverage path several times.

Each run is a fresh, headless simulation:
  1. ros2 launch mower_sim nav2_sim.launch.py controller:=<c> headless:=true
  2. ros2 run mower_sim metrics_recorder   (writes the CSVs)
  3. ros2 run mower_sim run_mowing_path    (sends the path to FollowPath)
The run ends when the recorder exits (goal finished) or on timeout, then the
whole launch is torn down so the next run starts from the same state.

Needs a sourced ROS 2 workspace with mower_sim built. Results land in
--output-dir; summarise them with scripts/analyze_benchmark.py.

    python3 scripts/run_benchmark.py --runs 3 --output-dir results/benchmark/raw
"""

import argparse
import os
import signal
import subprocess
import time


def _kill_group(proc, sig, wait):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=wait)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def _stop(proc):
    _kill_group(proc, signal.SIGINT, 15)
    _kill_group(proc, signal.SIGTERM, 5)
    _kill_group(proc, signal.SIGKILL, 5)


def _kill_stray_gzserver():
    # gzserver can outlive its launch. A leftover one keeps the Gazebo
    # master port, so the next run's gzserver would quietly attach the new
    # robot to the old world. Match the exact process name: -f would also
    # hit any shell whose command line merely mentions gzserver.
    if subprocess.run(['pkill', '-9', '-x', 'gzserver'], check=False).returncode == 0:
        time.sleep(3.0)


def run_once(controller, label, out_dir, timeout, log_dir, launch_args):
    def spawn(cmd, name):
        log = open(os.path.join(log_dir, f'{label}_{name}.log'), 'w')
        return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    _kill_stray_gzserver()
    sim = spawn(['ros2', 'launch', 'mower_sim', 'nav2_sim.launch.py',
                 f'controller:={controller}', 'headless:=true',
                 *launch_args], 'sim')
    recorder = spawn(['ros2', 'run', 'mower_sim', 'metrics_recorder',
                      '--ros-args',
                      '-p', 'use_sim_time:=true',
                      '-p', f'output_dir:={out_dir}',
                      '-p', f'run_label:={label}',
                      '-p', f'controller:={controller}'], 'recorder')
    start = time.monotonic()
    # controller_server's action server appears before the node is
    # activated, and a goal sent in that gap is rejected; wait for Nav2's
    # lifecycle manager to report everything active first.
    sim_log = os.path.join(log_dir, f'{label}_sim.log')
    client = None
    active = False
    while time.monotonic() - start < timeout and sim.poll() is None:
        with open(sim_log, errors='replace') as f:
            text = f.read()
        if 'lifecycle_manager_navigation' in text:
            lines = [ln for ln in text.splitlines()
                     if 'lifecycle_manager_navigation' in ln]
            if any('Managed nodes are active' in ln for ln in lines):
                active = True
                break
            # A lifecycle transition timed out: this bringup is dead.
            if any('Aborting bringup' in ln for ln in lines):
                break
        time.sleep(1.0)
    if not active:
        print(f'[benchmark] {label}: Nav2 never became active', flush=True)
        for proc in (recorder, sim):
            _stop(proc)
        _kill_stray_gzserver()
        return None, time.monotonic() - start
    if sim.poll() is None and time.monotonic() - start < timeout:
        client = spawn(['ros2', 'run', 'mower_sim', 'run_mowing_path',
                        '--ros-args', '-p', 'use_sim_time:=true'], 'client')

    ok = client is not None
    if ok:
        try:
            recorder.wait(timeout=max(1.0, timeout - (time.monotonic() - start)))
        except subprocess.TimeoutExpired:
            ok = False
    for proc in (client, recorder, sim):
        if proc is not None:
            _stop(proc)
    _kill_stray_gzserver()
    return ok, time.monotonic() - start


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--controllers', nargs='+',
                        default=['rpp', 'mppi', 'dwb'])
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--output-dir', default='results/benchmark/raw')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='wall-clock seconds per run, startup included')
    parser.add_argument('--retries', type=int, default=2,
                        help='restarts allowed when Nav2 fails to come up; '
                             'a run that starts is never repeated')
    parser.add_argument('--launch-args', nargs='*', default=[],
                        help='extra name:=value arguments for nav2_sim.launch.py')
    args = parser.parse_args()

    out_dir = os.path.abspath(args.output_dir)
    log_dir = os.path.join(out_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    for run in range(1, args.runs + 1):
        for controller in args.controllers:
            label = f'{controller}_run{run}'
            print(f'[benchmark] {label} ...', flush=True)
            for _ in range(args.retries + 1):
                ok, wall = run_once(controller, label, out_dir, args.timeout,
                                    log_dir, args.launch_args)
                if ok is not None:
                    break
            status = {True: 'done', False: 'TIMED OUT (no summary row written)',
                      None: 'FAILED to start'}[ok]
            print(f'[benchmark] {label} {status} after {wall:.0f} s',
                  flush=True)


if __name__ == '__main__':
    main()
