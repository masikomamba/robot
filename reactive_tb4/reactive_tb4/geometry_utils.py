"""Small math helpers shared by the controller and the mapper.

Nothing in this module depends on ROS, so it can be unit tested on its own.
"""

import math

#: Number of meters in one foot. The assignment is written in feet, ROS uses meters.
FEET_TO_METERS = 0.3048


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians into the range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the heading (rotation about z) encoded by a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
