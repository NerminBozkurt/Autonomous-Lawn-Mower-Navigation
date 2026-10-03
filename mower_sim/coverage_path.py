"""
Generate a Fields2Cover-style boustrophedon coverage path.

Fields2Cover itself is not a dependency: this reproduces the shape of the path
it hands to a follower, i.e. parallel swaths spaced one cutting width apart,
joined by U-turns in the headland, densely sampled as (x, y, yaw) poses. All
three controllers are then benchmarked against this exact reference.

Pure Python on purpose, so it can be unit tested without ROS.
"""

import math


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _straight(x, y, yaw, length, step):
    n = max(1, math.ceil(length / step))
    return [
        (x + length * i / n * math.cos(yaw),
         y + length * i / n * math.sin(yaw),
         yaw)
        for i in range(1, n + 1)
    ]


def _arc(x, y, yaw, radius, sweep, step):
    """Sample a circular arc; sweep > 0 turns left (CCW), < 0 turns right."""
    n = max(1, math.ceil(abs(sweep) * radius / step))
    side = 1.0 if sweep > 0 else -1.0
    cx = x - side * radius * math.sin(yaw)
    cy = y + side * radius * math.cos(yaw)
    poses = []
    for i in range(1, n + 1):
        heading = yaw + sweep * i / n
        poses.append((cx + side * radius * math.sin(heading),
                      cy - side * radius * math.cos(heading),
                      _wrap(heading)))
    return poses


def boustrophedon_path(num_swaths, swath_length, swath_spacing,
                       turn_radius=None, step=0.05, start=(0.0, 0.0, 0.0)):
    """
    Return a list of (x, y, yaw) poses covering num_swaths parallel swaths.

    The first swath starts at `start` and runs along its yaw; later swaths
    stack to its left. Swaths are swath_spacing apart, so setting that to the
    cutting width leaves no uncut strip between passes. Each U-turn is a
    quarter arc, a straight of (swath_spacing - 2 * turn_radius) and another
    quarter arc; turn_radius defaults to swath_spacing / 2, a plain semicircle.
    """
    if num_swaths < 1:
        raise ValueError('num_swaths must be at least 1')
    if swath_length <= 0.0 or swath_spacing <= 0.0 or step <= 0.0:
        raise ValueError('swath_length, swath_spacing and step must be positive')
    radius = swath_spacing / 2.0 if turn_radius is None else turn_radius
    if radius <= 0.0:
        raise ValueError('turn_radius must be positive')
    # A larger radius needs a bulb (omega) turn, which this generator does
    # not produce.
    if radius > swath_spacing / 2.0 + 1e-9:
        raise ValueError(
            f'turn_radius {radius} exceeds swath_spacing / 2 '
            f'({swath_spacing / 2.0}); U-turn would not fit')

    poses = [(start[0], start[1], _wrap(start[2]))]
    for i in range(num_swaths):
        poses += _straight(*poses[-1], swath_length, step)
        if i == num_swaths - 1:
            break
        sweep = math.pi / 2.0 if i % 2 == 0 else -math.pi / 2.0
        poses += _arc(*poses[-1], radius, sweep, step)
        gap = swath_spacing - 2.0 * radius
        if gap > 1e-9:
            poses += _straight(*poses[-1], gap, step)
        poses += _arc(*poses[-1], radius, sweep, step)
    return poses
