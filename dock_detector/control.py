"""Pure planar controller extracted from the staging survey."""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Limits:
    speed: float = 0.035
    angular_speed: float = 0.08
    xy_tolerance: float = 0.025
    yaw_tolerance: float = math.radians(2)
    reverse: bool = True


class PoseController:
    def __init__(self, limits):
        self.limits = limits
        self.xy_reached = False

    def reset(self):
        self.xy_reached = False

    def command(self, robot, goal):
        c = self.limits
        dx, dy = goal[0]-robot[0], goal[1]-robot[1]
        distance = math.hypot(dx, dy)
        heading = wrap(goal[2]-robot[2])
        if distance <= c.xy_tolerance:
            self.xy_reached = True
        if distance > 2*c.xy_tolerance:
            self.xy_reached = False
        if self.xy_reached:
            if abs(heading) <= c.yaw_tolerance:
                return 0., 0., 'settle'
            return 0., max(-c.angular_speed, min(c.angular_speed, 1.2*heading)), 'align'
        if c.reverse:
            alpha = wrap(math.atan2(dy, dx)-(robot[2]+math.pi))
            beta = wrap(goal[2]-robot[2]-alpha)
            angular = alpha-.35*beta
            linear = -min(c.speed, max(.006, .18*distance))*max(0., math.cos(alpha))
            if abs(alpha) > .55:
                linear = 0.
        else:
            alpha = wrap(math.atan2(dy, dx)-robot[2])
            angular = alpha
            linear = min(c.speed, max(.006, .4*distance))*max(0., math.cos(alpha))
            if abs(alpha) > .45:
                linear = 0.
        return linear, max(-c.angular_speed, min(c.angular_speed, angular)), 'drive'


class GoalMemory:
    """Last fully acquired goal; unconfirmed packets cannot extend its lifetime."""
    def __init__(self):
        self.clear()

    def clear(self):
        self.goal = None
        self.wall = self.stamp_ns = None

    def accept(self, goal, wall, stamp_ns):
        self.goal, self.wall, self.stamp_ns = tuple(goal), wall, stamp_ns

    def lookup(self, now, ros_ns, timeout, grace):
        if self.goal is None:
            return None
        age = (ros_ns-self.stamp_ns)/1e9
        if not 0 <= now-self.wall <= timeout+grace or not -.1 <= age <= timeout+grace:
            self.clear()
            return None
        return self.goal


def cached_reverse_command(law, robot, goal):
    """Follow a cached goal only while reversing; no blind final alignment."""
    v, w, mode = law.command(robot, goal)
    if mode != 'drive' or v >= 0.:
        return 0., 0., 'wait_for_detection'
    return v, w, 'reverse_cached_goal'
