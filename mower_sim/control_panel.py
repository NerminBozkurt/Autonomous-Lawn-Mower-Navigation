#!/usr/bin/env python3
"""
Control panel for the mower simulation.

Pick the controller (RPP, MPPI, DWB) and the odometry source (wheel encoders
or ground truth), then:
- Start launches the simulation if it is not running and sends the coverage
  path; after Stop it resumes the path from where the robot is.
- Stop cancels the path; the robot halts and the simulation stays up.
- Reset shuts the simulation down and relaunches it with the current
  selection, so the robot is back at the start. Changing the controller or
  the odometry while the simulation runs takes effect on Reset.

Each run is recorded by metrics_recorder into metrics/ and its headline
numbers are shown when it ends. All the work happens in SimSession; this
module only draws the window.

    ros2 run mower_sim control_panel
"""

import threading
import tkinter as tk
from tkinter import ttk

from mower_sim import sim_session as ss

POLL_MS = 200

CONTROLLER_LABELS = (('rpp', 'Regulated Pure Pursuit (RPP)'),
                     ('mppi', 'Model Predictive Path Integral (MPPI)'),
                     ('dwb', 'Dynamic Window (DWB)'))
ODOMETRY_LABELS = (('encoder', 'Wheel encoders (dead reckoning)'),
                   ('ground_truth', 'Ground truth (perfect localization)'))


def _fmt(value, unit, digits=2):
    return '—' if value is None else f'{value:.{digits}f} {unit}'


class ControlPanel:

    def __init__(self, root, session):
        self.root = root
        self.session = session
        self.start_when_ready = False
        self.closing = False

        root.title('Mower control panel')
        root.resizable(False, False)
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        frame = ttk.Frame(root, padding=12)
        frame.grid(sticky='nsew')

        self.controller = tk.StringVar(value='rpp')
        self.odometry = tk.StringVar(value='encoder')
        self.gazebo_gui = tk.BooleanVar(value=True)
        self.rviz = tk.BooleanVar(value=True)

        config = ttk.LabelFrame(frame, text='Configuration', padding=8)
        config.grid(row=0, column=0, sticky='ew')
        ttk.Label(config, text='Controller').grid(row=0, column=0, sticky='w')
        for i, (value, label) in enumerate(CONTROLLER_LABELS):
            ttk.Radiobutton(config, text=label, value=value,
                            variable=self.controller).grid(
                row=1 + i, column=0, sticky='w', padx=(12, 0))
        ttk.Label(config, text='Odometry').grid(row=4, column=0, sticky='w',
                                                pady=(8, 0))
        for i, (value, label) in enumerate(ODOMETRY_LABELS):
            ttk.Radiobutton(config, text=label, value=value,
                            variable=self.odometry).grid(
                row=5 + i, column=0, sticky='w', padx=(12, 0))
        windows = ttk.Frame(config)
        windows.grid(row=7, column=0, sticky='w', pady=(8, 0))
        ttk.Label(windows, text='Windows').pack(side='left')
        ttk.Checkbutton(windows, text='Gazebo',
                        variable=self.gazebo_gui).pack(side='left', padx=8)
        ttk.Checkbutton(windows, text='RViz', variable=self.rviz).pack(
            side='left')
        self.pending = ttk.Label(config, foreground='#a15c00')
        self.pending.grid(row=8, column=0, sticky='w', pady=(6, 0))

        buttons = ttk.Frame(frame, padding=(0, 10))
        buttons.grid(row=1, column=0, sticky='ew')
        self.start_btn = ttk.Button(buttons, text='Start', command=self.on_start)
        self.stop_btn = ttk.Button(buttons, text='Stop', command=self.on_stop)
        self.reset_btn = ttk.Button(buttons, text='Reset', command=self.on_reset)
        for b in (self.start_btn, self.stop_btn, self.reset_btn):
            b.pack(side='left', expand=True, fill='x', padx=2)

        status = ttk.LabelFrame(frame, text='Status', padding=8)
        status.grid(row=2, column=0, sticky='ew')
        self.status_vars = {}
        for row, name in enumerate(('State', 'Running', 'Remaining',
                                    'Speed', 'Elapsed')):
            ttk.Label(status, text=name).grid(row=row, column=0, sticky='w')
            var = tk.StringVar(value='—')
            ttk.Label(status, textvariable=var).grid(row=row, column=1,
                                                     sticky='w', padx=(12, 0))
            self.status_vars[name] = var
        self.progress = ttk.Progressbar(status, length=320, maximum=100.0)
        self.progress.grid(row=5, column=0, columnspan=2, sticky='ew',
                           pady=(6, 0))
        self.message = ttk.Label(status, wraplength=320, justify='left')
        self.message.grid(row=6, column=0, columnspan=2, sticky='w',
                          pady=(6, 0))

        results = ttk.LabelFrame(frame, text='Last run', padding=8)
        results.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        self.result_vars = {}
        for row, name in enumerate(('Result', 'Completion time',
                                    'Cross-track RMS (swaths)',
                                    'Cross-track RMS (turns)', 'Coverage')):
            ttk.Label(results, text=name).grid(row=row, column=0, sticky='w')
            var = tk.StringVar(value='—')
            ttk.Label(results, textvariable=var, wraplength=180,
                      justify='left').grid(row=row, column=1, sticky='w',
                                           padx=(12, 0))
            self.result_vars[name] = var

        self.refresh()

    # ------------------------------------------------------------ helpers

    def selection(self):
        return (self.controller.get(), self.odometry.get(),
                self.gazebo_gui.get(), self.rviz.get())

    # ------------------------------------------------------------ buttons

    def on_start(self):
        state = self.session.snapshot()['state']
        if state == ss.IDLE:
            self.start_when_ready = True
            self.session.launch(*self.selection())
        else:
            self.session.start()

    def on_stop(self):
        if self.start_when_ready:
            self.start_when_ready = False
            return
        self.session.stop()

    def on_reset(self):
        self.start_when_ready = False
        self.session.reset(*self.selection())

    def on_close(self):
        if self.closing:
            return
        self.closing = True
        for b in (self.start_btn, self.stop_btn, self.reset_btn):
            b.state(['disabled'])
        self.message.configure(text='Shutting the simulation down...')
        worker = threading.Thread(target=self.session.close, daemon=True)
        worker.start()

        def wait():
            if worker.is_alive():
                self.root.after(POLL_MS, wait)
            else:
                self.root.destroy()
        wait()

    # ------------------------------------------------------------ polling

    def refresh(self):
        if self.closing:
            return
        snap = self.session.snapshot()
        state = snap['state']
        if state == ss.READY and self.start_when_ready:
            self.start_when_ready = False
            self.session.start()
        elif state == ss.IDLE and self.start_when_ready \
                and 'failed' in snap['message']:
            self.start_when_ready = False

        busy = state in (ss.LAUNCHING, ss.SHUTTING_DOWN)
        self._enable(self.start_btn, state in (ss.IDLE, ss.READY, ss.STOPPED)
                     and not (state == ss.IDLE and self.start_when_ready))
        self._enable(self.stop_btn, state == ss.RUNNING or
                     (state == ss.LAUNCHING and self.start_when_ready))
        self._enable(self.reset_btn, not busy)
        self.start_btn.configure(text='Resume' if state == ss.STOPPED
                                 else 'Start')

        config = snap['config']
        if config is None:
            running = '—'
            self.pending.configure(text='')
        else:
            controller, odometry = config[:2]
            running = f'{controller.upper()}, {odometry.replace("_", " ")} odometry'
            changed = tuple(config) != self.selection()
            self.pending.configure(
                text='Selection differs from the running simulation; '
                     'press Reset to apply it.' if changed else '')

        total = self.session.path_length
        left = snap['distance_left']
        self.status_vars['State'].set(state.capitalize())
        self.status_vars['Running'].set(running)
        self.status_vars['Remaining'].set(
            '—' if left is None else f'{left:.2f} m of {total:.2f} m')
        self.status_vars['Speed'].set(_fmt(snap['speed'], 'm/s'))
        self.status_vars['Elapsed'].set(
            _fmt(snap['elapsed'], 's', 1) if config else '—')
        self.progress['value'] = 0.0 if left is None else \
            max(0.0, min(100.0, 100.0 * (1.0 - left / total)))
        self.message.configure(text=snap['message'])

        self.show_metrics(snap['result'], snap['metrics'], snap['resumed'])
        self.root.after(POLL_MS, self.refresh)

    def show_metrics(self, result, metrics, resumed):
        if result is None:
            return
        v = self.result_vars
        v['Result'].set(f'{result} (resumed: figures cover only the part '
                        'after Resume)' if resumed else result)
        if metrics is None:
            for name in list(v)[1:]:
                v[name].set('—')
            return

        def num(key, scale=1.0, unit='', digits=1):
            try:
                return f'{float(metrics[key]) * scale:.{digits}f} {unit}'.strip()
            except (KeyError, ValueError):
                return '—'
        v['Completion time'].set(num('completion_time_s', unit='s'))
        v['Cross-track RMS (swaths)'].set(num('cte_rms_swath', 100, 'cm'))
        v['Cross-track RMS (turns)'].set(num('cte_rms_turn', 100, 'cm'))
        v['Coverage'].set(num('coverage_pct', unit='%'))

    @staticmethod
    def _enable(button, enabled):
        button.state(['!disabled'] if enabled else ['disabled'])


def main():
    session = ss.SimSession()
    root = tk.Tk()
    ControlPanel(root, session)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        session.close()


if __name__ == '__main__':
    main()
