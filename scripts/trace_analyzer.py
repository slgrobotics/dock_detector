#!/usr/bin/env python3
"""
Plot the approach, yaw alignment, and insertion from a docking trace.

The plot shows the robot path by phase, distance to the dock, and yaw error.
When insertion status lines are present, it also plots forward travel and its cap.
Scroll to zoom the entire figure, left-drag to pan it, and press R to fit it to the window.
The interactive viewer requires Qt with SVG support.

     ~/robot_ws/src/dock_detector/scripts/trace_analyzer.py "/path/to/trace.log"

Use --output dock-approach.png to save the plot without opening a window.

The script requires Matplotlib and expects one docking run per input file.

To prepare the log file, run the robot and copy the portion of the terminal output where the docking_server prints appear. The log should contain lines like:

[component_...] [INFO] [...] [docking_server]: DOCK: approach id=v_bay frame=odom scan_age_ms=100.0 robot=(0.275,5.596) detected=(0.041,6.243) filtered=(0.041,6.243) threshold_cm=2.50 threshold_yaw_deg=5.00
[component_...] [INFO] [...] [docking_server]:       xy_cm=68.83 yaw_deg=-1.39 filtered_xy_cm=68.83 dx_cm=+23.43 dy_cm=-64.72 filtered_yaw_deg=-1.39
[component_...] [INFO] [...] [docking_server]: DOCK: approach id=v_bay frame=odom scan_age_ms=100.0 robot=(0.276,5.611) detected=(0.041,6.248) filtered=(0.041,6.244) threshold_cm=2.50 threshold_yaw_deg=5.00
[component_...] [INFO] [...] [docking_server]:       xy_cm=67.82 yaw_deg=-1.45 filtered_xy_cm=67.44 dx_cm=+23.49 dy_cm=-63.62 filtered_yaw_deg=-1.45

"""

import argparse
from dataclasses import dataclass
from importlib import import_module
from io import BytesIO
from itertools import groupby
from pathlib import Path
import re
import signal


NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
POINT = rf'\(({NUMBER}),\s*({NUMBER})\)'
POSE = re.compile(
    rf'\[({NUMBER})\]\s+\[docking_server\]:\s+'
    rf'DOCK: (approach|arrival) id=(\S+) frame=(\S+) .*?'
    rf'robot={POINT} detected={POINT} filtered={POINT} threshold_cm=({NUMBER})'
    rf' threshold_yaw_deg=({NUMBER})(?: phase=(\S+))?'
)
ERROR = re.compile(
    rf'\[docking_server\]:\s+xy_cm=({NUMBER}) yaw_deg=({NUMBER}) '
    rf'filtered_xy_cm=({NUMBER})'
    rf' dx_cm={NUMBER} dy_cm={NUMBER} '
    rf'filtered_yaw_deg=({NUMBER})'
)

INSERTION = re.compile(
    rf'\[({NUMBER})\]\s+\[docking_alignment\]:\s+'
    rf'ALIGN: insertion_(?:advance|wait_charge|charging) id=(\S+) .*?'
    rf'travel_cm=({NUMBER}) max_travel_cm=({NUMBER})'
)
STAGES = {
    'approach': ('Approach', '#1468a0'),
    'alignment': ('Yaw alignment', '#7453a6'),
    'insertion': ('Insertion', '#b34435'),
}


@dataclass
class Sample:
    time: float
    phase: str
    dock: str
    frame: str
    robot: tuple[float, float]
    detected: tuple[float, float]
    filtered: tuple[float, float]
    threshold: float
    distance: float
    yaw: float
    filtered_distance: float
    filtered_yaw: float
    yaw_threshold: float
    stage: str = 'approach'


def read_samples(lines):
    samples = []
    pose = None
    for line in lines:
        match = POSE.search(line)
        if match:
            pose = match
            continue
        error = ERROR.search(line)
        if pose is None or error is None:
            continue
        time, phase, dock, frame, *coordinates, threshold, yaw_limit, stage = pose.groups()
        rx, ry, dx, dy, fx, fy = map(float, coordinates)
        distance, yaw, filtered_distance, filtered_yaw = map(float, error.groups())
        samples.append(Sample(
            float(time), phase, dock, frame, (rx, ry), (dx, dy), (fx, fy),
            float(threshold), distance, yaw, filtered_distance, filtered_yaw, float(yaw_limit), stage or 'approach',
        ))
        pose = None
    # Older wrappers logged Nav2's position handoff as arrival with yaw disabled.
    # Require the next pose to be from the same run's wrapper before relabeling it.
    for index, (sample, following) in enumerate(zip(samples, samples[1:])):
        if (sample.phase == 'arrival' and sample.stage == 'approach'
                and sample.yaw_threshold == 180 and following.stage == 'alignment'
                and (sample.dock, sample.frame) == (following.dock, following.frame)):
            sample.phase = 'approach'
            sample.stage = 'position_reached'
            for approach in reversed(samples[:index + 1]):
                if (approach.stage not in ('approach', 'position_reached')
                        or approach.phase == 'arrival' or approach.yaw_threshold != 180):
                    break
                approach.yaw_threshold = following.yaw_threshold
    return samples


def read_insertion_travel(lines, first):
    travel = []
    for line in lines:
        match = INSERTION.search(line)
        if match:
            time, dock, distance, limit = match.groups()
            if dock == first.dock and float(time) >= first.time:
                travel.append((float(time) - first.time, float(distance), float(limit)))
    return travel


def plot(samples, plt, insertion_travel=()):
    first, last = samples[0], samples[-1]
    elapsed = [sample.time - first.time for sample in samples]

    figure = plt.figure(figsize=(10, 6), constrained_layout=True)
    rows = 3 if insertion_travel else 2
    grid = figure.add_gridspec(rows, 2, width_ratios=[1, 1.4])
    path = figure.add_subplot(grid[:, 0])
    distance = figure.add_subplot(grid[0, 1])
    yaw = figure.add_subplot(grid[1, 1], sharex=distance)
    time_axes = [distance, yaw]
    if insertion_travel:
        travel = figure.add_subplot(grid[2, 1], sharex=distance)
        time_axes.append(travel)
    figure.suptitle(f'Docking trace · {first.dock}', fontsize=17, fontweight='bold')

    blue, orange, green, purple = '#1468a0', '#d77e29', '#00846f', '#7453a6'
    for axes in (path, *time_axes):
        axes.grid(alpha=0.22)
        axes.set_axisbelow(True)
        for side in ('top', 'right'):
            axes.spines[side].set_visible(False)

    seen = set()
    stage_at = [sample.stage if sample.stage in STAGES else 'approach' for sample in samples]
    for stage, indices in groupby(range(len(samples)), key=stage_at.__getitem__):
        indices = list(indices)
        start, end = indices[0], indices[-1]
        label, color = STAGES[stage]
        points = samples[max(0, start - 1):end + 1]
        path.plot(*zip(*(sample.robot for sample in points)), '.-',
                  color=color, markersize=3, label=label if stage not in seen else None)
        seen.add(stage)
        end_time = elapsed[min(end + 1, len(elapsed) - 1)]
        if stage == 'insertion' and insertion_travel:
            end_time = max(end_time, insertion_travel[-1][0])
        if stage != 'approach':
            for axes in time_axes:
                axes.axvspan(elapsed[start], end_time, color=color, alpha=0.10, linewidth=0)
                axes.axvline(elapsed[start], color=color, alpha=0.55, linewidth=1)

    handoff = next((elapsed[i] for i, sample in enumerate(samples)
                    if sample.stage == 'position_reached'), None)
    if handoff is not None:
        for axes in time_axes:
            axes.axvline(handoff, color=blue, alpha=0.65, linestyle=':', linewidth=1,
                         label='Nav2 handoff' if axes is distance else None)
    path.plot(*zip(*(sample.detected for sample in samples)), '.',
              color=orange, markersize=3, label='Detected dock')
    path.plot(*zip(*(sample.filtered for sample in samples)), '.-',
              color=green, markersize=3, label='Filtered dock')
    end_label = 'Arrival' if last.phase == 'arrival' else 'Last sample'
    for sample, label, offset in [(first, 'Start', (-6, 7)), (last, end_label, (9, -10))]:
        color = STAGES[sample.stage if sample.stage in STAGES else 'approach'][1]
        path.scatter(*sample.robot, color=color, s=36, zorder=4)
        path.annotate(label, sample.robot, xytext=offset, textcoords='offset points',
                      ha='right' if sample is first else 'left')
    path.set(xlabel=f'{first.frame} x (m)', ylabel=f'{first.frame} y (m)',
             title='Robot path and dock estimates', aspect='equal')
    path.margins(x=0.2, y=0.08)
    path.legend(loc='best', fontsize=8)

    distance.plot(elapsed, [sample.distance for sample in samples],
                  color=orange, label='Detected dock')
    distance.plot(elapsed, [sample.filtered_distance for sample in samples],
                  color=green, label='Filtered dock')
    distance.step(elapsed, [sample.threshold for sample in samples], where='post',
                  color='0.4', linestyle='--', linewidth=1,
                  label='Arrival threshold')
    distance.set(ylabel='Distance (cm)', title='Distance to dock')
    distance.legend(fontsize=8)
    distance.tick_params(labelbottom=False)

    yaw.axhline(0, color='0.4', linestyle='--', linewidth=1)
    yaw.plot(elapsed, [sample.yaw for sample in samples], color=purple, label='Detected dock')
    yaw.plot(elapsed, [sample.filtered_yaw for sample in samples],
             color=green, label='Filtered dock')
    limits = [sample.yaw_threshold for sample in samples]
    yaw.step(elapsed, limits, where='post', color='0.4', linestyle='--',
             linewidth=1, label='Arrival tolerance')
    yaw.step(elapsed, [-limit for limit in limits], where='post',
             color='0.4', linestyle='--', linewidth=1)
    yaw.legend(fontsize=8)
    yaw.set(ylabel='Yaw error (°)', title='Logged yaw error')
    if insertion_travel:
        times, travel_cm, caps = zip(*insertion_travel)
        travel.plot(times, travel_cm, '.-', color=STAGES['insertion'][1], markersize=3,
                    label='Forward travel')
        travel.step(times, caps, where='post', color='0.4', linestyle='--', linewidth=1,
                    label='Travel cap')
        travel.set(ylabel='Travel (cm)', title='Insertion travel from odometry')
        travel.legend(fontsize=8)
        yaw.tick_params(labelbottom=False)
    time_axes[-1].set_xlabel('Elapsed time (s)')
    return figure


def figure_window(svg):
    from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets
    QtSvg = import_module(f'{QtCore.__name__.split(".")[0]}.QtSvg')

    class Canvas(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.renderer = QtSvg.QSvgRenderer(QtCore.QByteArray(svg), self)

        def paintEvent(self, event):
            painter = QtGui.QPainter(self)
            try:
                painter.fillRect(self.rect(), QtCore.Qt.GlobalColor.white)
                self.renderer.render(painter)
            finally:
                painter.end()

    class FigureWindow(QtWidgets.QScrollArea):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('Docking trace · Scroll to zoom, drag to pan, R to fit')
            self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            self.canvas = Canvas()
            self.base_size = self.canvas.renderer.defaultSize()
            self.setWidget(self.canvas)
            self.canvas.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
            self.canvas.installEventFilter(self)
            self.viewport().installEventFilter(self)
            self.scale = 1.0
            self.drag_position = None
            self.drag_scroll = None
            desktop = self.screen().availableGeometry()
            self.resize(min(3 * self.base_size.width(), round(.95 * desktop.width())),
                        min(3 * self.base_size.height(), round(.90 * desktop.height())))
            QtCore.QTimer.singleShot(0, self.fit)

        def set_scale(self, scale):
            self.scale = scale
            self.canvas.setFixedSize(max(1, round(self.base_size.width() * scale)),
                                     max(1, round(self.base_size.height() * scale)))

        def fit(self):
            size = self.maximumViewportSize()
            self.set_scale(max(.1, min((size.width() - 2) / self.base_size.width(),
                                       (size.height() - 2) / self.base_size.height())))

        def keyPressEvent(self, event):
            if event.key() == QtCore.Qt.Key.Key_R:
                self.fit()
                return
            super().keyPressEvent(event)

        def eventFilter(self, watched, event):
            kind = event.type()
            types = QtCore.QEvent.Type
            horizontal, vertical = self.horizontalScrollBar(), self.verticalScrollBar()
            if kind == types.Wheel and event.angleDelta().y():
                point = event.globalPosition().toPoint() if hasattr(event, 'globalPosition') else event.globalPos()
                before = self.canvas.mapFromGlobal(point)
                scale = min(12.0, max(.1, self.scale * 1.2 ** (event.angleDelta().y() / 120)))
                ratio = scale / self.scale
                self.set_scale(scale)
                after = self.canvas.mapFromGlobal(point)
                horizontal.setValue(horizontal.value() + round(before.x() * ratio - after.x()))
                vertical.setValue(vertical.value() + round(before.y() * ratio - after.y()))
                return True
            if kind in (types.MouseButtonPress, types.MouseMove, types.MouseButtonRelease):
                point = event.globalPosition().toPoint() if hasattr(event, 'globalPosition') else event.globalPos()
                if kind == types.MouseButtonPress and event.button() == QtCore.Qt.MouseButton.LeftButton:
                    self.setFocus()
                    self.drag_position = point
                    self.drag_scroll = (horizontal.value(), vertical.value())
                    self.canvas.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
                    return True
                if kind == types.MouseMove and self.drag_position is not None:
                    delta = point - self.drag_position
                    horizontal.setValue(self.drag_scroll[0] - delta.x())
                    vertical.setValue(self.drag_scroll[1] - delta.y())
                    return True
                if kind == types.MouseButtonRelease and event.button() == QtCore.Qt.MouseButton.LeftButton:
                    self.drag_position = None
                    self.canvas.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
                    return True
            return super().eventFilter(watched, event)

    return FigureWindow()


def show_figure(figure):
    from matplotlib.backends.qt_compat import QtCore, QtWidgets

    svg = BytesIO()
    figure.savefig(svg, format='svg')
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = figure_window(svg.getvalue())
    window.show()
    window.setFocus()

    # Let Python handle Ctrl+C while Qt's event loop is idle.
    timer = QtCore.QTimer(window)
    timer.timeout.connect(lambda: None)
    timer.start(100)
    previous = signal.signal(signal.SIGINT, lambda *_: app.exit(130))
    try:
        return app.exec() if hasattr(app, 'exec') else app.exec_()
    finally:
        signal.signal(signal.SIGINT, previous)
        window.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path, help='docking trace log')
    parser.add_argument('-o', '--output', type=Path,
                        help='save the plot to a file instead of opening a window')
    args = parser.parse_args()

    try:
        lines = args.log.read_text(encoding='utf-8', errors='replace').splitlines()
        samples = read_samples(lines)
    except OSError as exc:
        parser.error(str(exc))
    if not samples:
        parser.error('no complete samples found (expected threshold_yaw_deg on the pose line and filtered_yaw_deg on the error line)')
    if len({(sample.dock, sample.frame) for sample in samples}) > 1:
        parser.error('the log must contain samples for a single dock and coordinate frame')
    if any(sample.phase == 'arrival' for sample in samples[:-1]):
        parser.error('the log contains more than one approach; provide one run at a time')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error('Matplotlib is required (Ubuntu package: python3-matplotlib)')

    figure = plot(samples, plt, read_insertion_travel(lines, samples[0]))
    if args.output:
        try:
            figure.savefig(args.output, dpi=180)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
    else:
        try:
            status = show_figure(figure)
        except ImportError:
            parser.error('interactive display requires Qt with SVG support; use --output to export')
        finally:
            plt.close(figure)
        if status:
            raise SystemExit(status)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
