#!/usr/bin/env python3
"""Plot a docking_server approach trace from a log file."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re


NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
POINT = rf'\(({NUMBER}),\s*({NUMBER})\)'
POSE = re.compile(
    rf'\[({NUMBER})\]\s+\[docking_server\]:\s+'
    rf'DOCK: (approach|arrival) id=(\S+) frame=(\S+) .*?'
    rf'robot={POINT} detected={POINT} filtered={POINT} threshold_cm=({NUMBER})'
)
ERROR = re.compile(
    rf'\[docking_server\]:\s+xy_cm=({NUMBER}) yaw_deg=({NUMBER}) '
    rf'filtered_xy_cm=({NUMBER})'
)


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
        time, phase, dock, frame, *coordinates, threshold = pose.groups()
        rx, ry, dx, dy, fx, fy = map(float, coordinates)
        distance, yaw, filtered_distance = map(float, error.groups())
        samples.append(Sample(
            float(time), phase, dock, frame, (rx, ry), (dx, dy), (fx, fy),
            float(threshold), distance, yaw, filtered_distance,
        ))
        pose = None
    return samples


def plot(samples, plt):
    first, last = samples[0], samples[-1]
    elapsed = [sample.time - first.time for sample in samples]

    figure = plt.figure(figsize=(10, 6), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=[1, 1.4])
    path = figure.add_subplot(grid[:, 0])
    distance = figure.add_subplot(grid[0, 1])
    yaw = figure.add_subplot(grid[1, 1], sharex=distance)
    figure.suptitle(f'Dock approach · {first.dock}', fontsize=17, fontweight='bold')

    blue, orange, green, purple = '#1468a0', '#d77e29', '#00846f', '#7453a6'
    for axes in (path, distance, yaw):
        axes.grid(alpha=0.22)
        axes.set_axisbelow(True)
        for side in ('top', 'right'):
            axes.spines[side].set_visible(False)

    path.plot(*zip(*(sample.robot for sample in samples)), '.-',
              color=blue, markersize=3, label='Robot path')
    path.plot(*zip(*(sample.detected for sample in samples)), '.',
              color=orange, markersize=3, label='Detected dock')
    path.plot(*zip(*(sample.filtered for sample in samples)), '.-',
              color=green, markersize=3, label='Filtered dock')
    end_label = 'Arrival' if last.phase == 'arrival' else 'Last sample'
    for sample, label, offset in [(first, 'Start', (-6, 7)), (last, end_label, (9, -10))]:
        path.scatter(*sample.robot, color=blue, s=36, zorder=4)
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
    yaw.plot(elapsed, [sample.yaw for sample in samples], color=purple)
    yaw.set(xlabel='Elapsed time (s)', ylabel='Yaw error (°)', title='Logged yaw error')
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path, help='docking trace log')
    parser.add_argument('-o', '--output', type=Path,
                        help='save the plot to a file instead of opening a window')
    args = parser.parse_args()

    try:
        with args.log.open(encoding='utf-8', errors='replace') as lines:
            samples = read_samples(lines)
    except OSError as exc:
        parser.error(str(exc))
    if not samples:
        parser.error('no complete docking approach or arrival samples found')
    if len({(sample.dock, sample.frame) for sample in samples}) > 1:
        parser.error('the log must contain samples for a single dock and coordinate frame')
    if any(sample.phase == 'arrival' for sample in samples[:-1]):
        parser.error('the log contains more than one approach; provide one run at a time')

    try:
        import matplotlib
        if args.output:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error('Matplotlib is required (Ubuntu package: python3-matplotlib)')

    figure = plot(samples, plt)
    if args.output:
        try:
            figure.savefig(args.output, dpi=180)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
    else:
        plt.show()


if __name__ == '__main__':
    main()
