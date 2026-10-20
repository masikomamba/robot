"""Perceptual processing of laser scans for the reactive behaviors.

The escape and avoid behaviors both look at the same thing: the closest
obstacle in the left half and in the right half of the area in front of the
robot. This module turns a raw scan into that summary, which is stored in a
:class:`FrontObstacleReport`. It has no ROS dependencies.
"""

from dataclasses import dataclass
import math
from typing import Sequence

from reactive_tb4.geometry_utils import normalize_angle


@dataclass(frozen=True)
class FrontObstacleReport:
    """Summary of the obstacles in front of the robot.

    Distances are measured from the edge of the robot body (not the lidar),
    so ``left_clearance == 0.3`` means an obstacle about 0.3 m past the bumper.
    ``math.inf`` means nothing was seen on that side.
    """

    left_clearance: float
    right_clearance: float
    threshold: float
    symmetry_tolerance: float

    @property
    def left_blocked(self) -> bool:
        """Return True if an obstacle is within the threshold on the left."""
        return self.left_clearance < self.threshold

    @property
    def right_blocked(self) -> bool:
        """Return True if an obstacle is within the threshold on the right."""
        return self.right_clearance < self.threshold

    @property
    def any_blocked(self) -> bool:
        """Return True if anything is within the threshold in front."""
        return self.left_blocked or self.right_blocked

    @property
    def symmetric(self) -> bool:
        """Return True if both sides are blocked by about the same amount."""
        return (self.left_blocked and self.right_blocked and
                abs(self.left_clearance - self.right_clearance) <= self.symmetry_tolerance)

    @property
    def asymmetric(self) -> bool:
        """Return True if something is close in front but not symmetrically."""
        return self.any_blocked and not self.symmetric

    @property
    def closer_side(self) -> int:
        """Return +1 if the closest obstacle is on the left, -1 if on the right."""
        return 1 if self.left_clearance < self.right_clearance else -1


def analyze_front(ranges: Sequence[float], angle_min: float, angle_increment: float,
                  range_min: float, range_max: float, laser_yaw: float,
                  robot_radius: float, front_half_angle: float, threshold: float,
                  symmetry_tolerance: float) -> FrontObstacleReport:
    """Find the closest obstacle on each side of the robot's front sector.

    Args:
        ranges: Raw ``LaserScan.ranges``.
        angle_min: Angle of the first beam in the laser frame (rad).
        angle_increment: Angle between beams (rad).
        range_min: Readings below this are invalid.
        range_max: Readings above this are invalid.
        laser_yaw: Yaw of the laser frame relative to base_link (rad). The
            TurtleBot 4 lidar is mounted rotated 90 degrees, so this matters.
        robot_radius: Distance from the robot center to its bumper (m).
        front_half_angle: Beams within +/- this angle of straight ahead count
            as "in front" (rad).
        threshold: Clearance that counts as "close" (m). One foot here.
        symmetry_tolerance: Max left/right clearance difference that still
            counts as symmetric (m).

    Returns:
        A :class:`FrontObstacleReport`. Left means positive angles (CCW).

    """
    left = math.inf
    right = math.inf
    for i, reading in enumerate(ranges):
        if not math.isfinite(reading) or reading < range_min or reading > range_max:
            continue
        # Angle of this beam measured in the robot's own frame.
        angle = normalize_angle(angle_min + i * angle_increment + laser_yaw)
        if abs(angle) > front_half_angle:
            continue
        clearance = max(0.0, reading - robot_radius)
        if angle >= 0.0:
            left = min(left, clearance)
        else:
            right = min(right, clearance)
    return FrontObstacleReport(left, right, threshold, symmetry_tolerance)
