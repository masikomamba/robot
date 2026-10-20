"""Unit tests for perception and behaviors (no ROS needed)."""

import math
import random

from reactive_tb4.behaviors import (
    AvoidAsymmetric, DriveForward, EscapeSymmetric, HaltOnBump, KeyboardTeleop,
    MotionCommand, PriorityArbiter, RandomTurn, SensorSnapshot)
from reactive_tb4.geometry_utils import FEET_TO_METERS
from reactive_tb4.perception import analyze_front, FrontObstacleReport

FT = FEET_TO_METERS
CLEAR = FrontObstacleReport(math.inf, math.inf, FT, 0.05)


def make_arbiter(seed=0):
    """Build the same behavior stack the node uses."""
    rng = random.Random(seed)
    return PriorityArbiter([
        HaltOnBump(), KeyboardTeleop(), EscapeSymmetric(1.0, rng=rng),
        AvoidAsymmetric(0.6), RandomTurn(1.0, rng=rng), DriveForward(0.2)])


def snap(front=CLEAR, bump=False, teleop=None, yaw=0.0, odo=0.0):
    """Shorthand for a sensor snapshot."""
    return SensorSnapshot(bump, teleop, front, yaw, odo)


def scan_with(points, n=360):
    """Fake 360-beam scan in the robot frame with obstacles at (deg, range)."""
    ranges = [math.inf] * n
    for deg, rng in points:
        ranges[int(round(deg)) % n] = rng
    return ranges


def test_example_from_assignment_turns_right():
    """0.7 ft ahead-left and 0.9 ft straight ahead: avoid by turning right."""
    r = 0.17
    ranges = scan_with([(20, r + 0.7 * FT), (0, r + 0.9 * FT), (359, r + 0.9 * FT)])
    front = analyze_front(ranges, 0.0, math.radians(1), 0.1, 12.0, 0.0, r,
                          math.radians(60), FT, 0.05)
    assert front.asymmetric
    name, cmd = make_arbiter().arbitrate(snap(front))
    assert name == 'avoid' and cmd.angular < 0.0


def test_wall_straight_ahead_is_symmetric():
    """A wall dead ahead gives equal clearance on both sides."""
    r = 0.17
    pts = [(d, (r + 0.25) / math.cos(math.radians(d))) for d in range(-40, 41)]
    front = analyze_front(scan_with(pts), 0.0, math.radians(1), 0.1, 12.0, 0.0, r,
                          math.radians(60), FT, 0.05)
    assert front.symmetric


def test_laser_yaw_offset_is_applied():
    """With the lidar rotated 90 deg, beam 270 deg in laser frame is dead ahead."""
    ranges = scan_with([(270, 0.3)])
    front = analyze_front(ranges, 0.0, math.radians(1), 0.1, 12.0, math.pi / 2, 0.17,
                          math.radians(60), FT, 0.05)
    assert front.any_blocked


def test_priority_order():
    """Bump beats keyboard, keyboard beats obstacles."""
    arb = make_arbiter()
    sym = FrontObstacleReport(0.1, 0.1, FT, 0.05)
    key = MotionCommand(0.1, 0.0)
    assert arb.arbitrate(snap(sym, bump=True, teleop=key))[0] == 'halt'
    assert arb.arbitrate(snap(sym, teleop=key))[0] == 'teleop'
    assert arb.arbitrate(snap(sym))[0] == 'escape'
    assert arb.arbitrate(snap(CLEAR))[0] in ('escape', 'forward')


def test_escape_is_fixed_action_pattern():
    """Escape keeps turning after the obstacle disappears, ending near 180 deg."""
    arb = make_arbiter()
    sym = FrontObstacleReport(0.1, 0.12, FT, 0.05)
    yaw = 0.0
    name, cmd = arb.arbitrate(snap(sym, yaw=yaw))
    assert name == 'escape'
    total = 0.0
    for _ in range(500):
        dyaw = cmd.angular * 0.1
        yaw += dyaw
        total += dyaw
        name, cmd = arb.arbitrate(snap(CLEAR, yaw=yaw))
        if name != 'escape':
            break
    assert math.radians(150) - 0.05 <= abs(total) <= math.radians(210) + 0.05


def test_random_turn_every_foot():
    """After one foot of travel a turn within +/-15 deg happens."""
    rng = random.Random(3)
    rt = RandomTurn(1.0, rng=rng)
    assert rt.compute(snap(odo=0.0)) is None
    assert rt.compute(snap(odo=0.2)) is None
    cmd = rt.compute(snap(odo=FT + 0.01))
    assert cmd is not None and abs(rt.last_angle) <= math.radians(15)
