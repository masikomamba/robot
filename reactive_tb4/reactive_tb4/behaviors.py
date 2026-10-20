"""Reactive behaviors and the priority arbiter that combines them.

This is the heart of the project's architecture: a priority-based
(pseudo-subsumption) design in the style of Jones, Flynn and Seiger. Each
behavior is its own class with one job. Every control cycle the
:class:`PriorityArbiter` asks the behaviors, from highest to lowest priority,
whether they want control. The first one that answers with a command wins and
everything below it is suppressed for that cycle.

Priorities (highest first), exactly as the assignment lists them:

1. :class:`HaltOnBump`
2. :class:`KeyboardTeleop`
3. :class:`EscapeSymmetric` (fixed action pattern)
4. :class:`AvoidAsymmetric` (reflex)
5. :class:`RandomTurn` (fixed action pattern)
6. :class:`DriveForward`

Nothing here imports ROS. The ROS node packs its sensor data into a
:class:`SensorSnapshot` and turns the winning :class:`MotionCommand` into a
velocity message. That keeps the behaviors easy to read and easy to test.
"""

from dataclasses import dataclass
import math
import random
from typing import List, Optional, Tuple

from reactive_tb4.geometry_utils import FEET_TO_METERS, normalize_angle
from reactive_tb4.perception import FrontObstacleReport


@dataclass(frozen=True)
class MotionCommand:
    """A velocity request: forward speed (m/s) and turn rate (rad/s, + is left)."""

    linear: float = 0.0
    angular: float = 0.0


STOP = MotionCommand(0.0, 0.0)


@dataclass(frozen=True)
class SensorSnapshot:
    """Everything the behaviors are allowed to know for one control cycle."""

    bumper_pressed: bool
    teleop_command: Optional[MotionCommand]
    front: FrontObstacleReport
    yaw: float
    forward_odometer: float


class TurnTracker:
    """Tracks an in-place turn of a set size by adding up small yaw changes.

    Adding up deltas (instead of comparing to a target heading) avoids the
    +/-180 degree wrap problem, which matters because escape turns are about
    180 degrees.
    """

    def __init__(self, turn_speed: float, min_turn_speed: float = 0.3,
                 tolerance: float = math.radians(2.0)):
        """Store the speed limits used while turning."""
        self._turn_speed = turn_speed
        self._min_turn_speed = min_turn_speed
        self._tolerance = tolerance
        self._goal = 0.0
        self._turned = 0.0
        self._last_yaw: Optional[float] = None
        self.active = False

    def start(self, amount: float, yaw: float) -> None:
        """Begin turning by ``amount`` radians (+ is left) from heading ``yaw``."""
        self._goal = amount
        self._turned = 0.0
        self._last_yaw = yaw
        self.active = True

    def cancel(self) -> None:
        """Forget the current turn."""
        self.active = False
        self._last_yaw = None

    @property
    def remaining(self) -> float:
        """Return how much of the turn is left (rad, signed)."""
        return self._goal - self._turned

    def step(self, yaw: float) -> Optional[MotionCommand]:
        """Update with the newest heading; return a command or None when done."""
        if not self.active:
            return None
        if self._last_yaw is not None:
            self._turned += normalize_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        remaining = self.remaining
        if abs(remaining) <= self._tolerance or remaining * self._goal < 0.0:
            # Reached the goal (or slightly overshot it).
            self.cancel()
            return None
        # Slow down near the end so the turn does not overshoot much.
        speed = min(self._turn_speed, max(self._min_turn_speed, 2.0 * abs(remaining)))
        return MotionCommand(0.0, math.copysign(speed, remaining))


class Behavior:
    """Base class for one reactive behavior.

    Subclasses override :meth:`compute`. Returning ``None`` means "I do not
    want control right now", which lets lower priority behaviors run.
    """

    name = 'behavior'

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Return a command if this behavior wants control, else None."""
        raise NotImplementedError

    def on_suppressed(self) -> None:
        """Handle being overridden by a higher priority behavior.

        Reflexes have no memory so the default does nothing. Fixed action
        patterns override this to abandon their current action, since the
        situation that triggered it may be gone once they get control back.
        """


class HaltOnBump(Behavior):
    """Priority 1: stop dead while any bumper reports a collision."""

    name = 'halt'

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Return STOP while a bump is detected."""
        return STOP if sensors.bumper_pressed else None


class KeyboardTeleop(Behavior):
    """Priority 2: pass through a recent keyboard command from a human."""

    name = 'teleop'

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Return the human's command if one arrived recently."""
        return sensors.teleop_command


class EscapeSymmetric(Behavior):
    """Priority 3: turn around (180 +/- 30 deg) from roughly symmetric obstacles.

    This is a fixed action pattern: once triggered it finishes the whole turn
    even if the obstacles stop looking symmetric halfway through.
    """

    name = 'escape'

    def __init__(self, turn_speed: float, spread: float = math.radians(30.0),
                 rng: Optional[random.Random] = None):
        """Set turn speed and the random spread around 180 degrees."""
        self._tracker = TurnTracker(turn_speed)
        self._spread = spread
        self._rng = rng or random.Random()

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Keep turning if already escaping, or start a new escape if triggered."""
        if not self._tracker.active and sensors.front.symmetric:
            # Turn away from whichever side is a little closer.
            direction = -float(sensors.front.closer_side)
            amount = math.pi + self._rng.uniform(-self._spread, self._spread)
            self._tracker.start(direction * amount, sensors.yaw)
        return self._tracker.step(sensors.yaw)

    def on_suppressed(self) -> None:
        """Abandon the turn if a higher priority behavior takes over."""
        self._tracker.cancel()


class AvoidAsymmetric(Behavior):
    """Priority 4: reflexively turn away from the closer of lopsided obstacles.

    Being a reflex, it only acts while the stimulus is present and stops the
    moment nothing asymmetric is within one foot.
    """

    name = 'avoid'

    def __init__(self, turn_speed: float):
        """Set how fast to turn away."""
        self._turn_speed = turn_speed

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Turn away from the closer side while an asymmetric obstacle is close."""
        if not sensors.front.asymmetric:
            return None
        return MotionCommand(0.0, -sensors.front.closer_side * self._turn_speed)


class RandomTurn(Behavior):
    """Priority 5: after each foot of forward travel, turn by U(-15, 15) degrees.

    Also a (small) fixed action pattern: once started, the turn is finished.
    """

    name = 'random_turn'

    def __init__(self, turn_speed: float, interval: float = FEET_TO_METERS,
                 max_angle: float = math.radians(15.0),
                 rng: Optional[random.Random] = None):
        """Set turn speed, travel interval (m) and max random angle (rad)."""
        self._tracker = TurnTracker(turn_speed, min_turn_speed=0.2,
                                    tolerance=math.radians(1.0))
        self._interval = interval
        self._max_angle = max_angle
        self._rng = rng or random.Random()
        self._mark: Optional[float] = None
        self.last_angle = 0.0

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Start a random turn every ``interval`` meters, then finish it."""
        if self._mark is None:
            self._mark = sensors.forward_odometer
        if not self._tracker.active:
            if sensors.forward_odometer - self._mark < self._interval:
                return None
            self.last_angle = self._rng.uniform(-self._max_angle, self._max_angle)
            self._tracker.start(self.last_angle, sensors.yaw)
        command = self._tracker.step(sensors.yaw)
        if command is None:
            # Turn finished: start counting the next foot from here.
            self._mark = sensors.forward_odometer
        return command

    def on_suppressed(self) -> None:
        """Drop the partial turn; a new one starts when control returns."""
        self._tracker.cancel()


class DriveForward(Behavior):
    """Priority 6: the default, just drive straight ahead."""

    name = 'forward'

    def __init__(self, speed: float):
        """Set the cruising speed (m/s)."""
        self._command = MotionCommand(speed, 0.0)

    def compute(self, sensors: SensorSnapshot) -> Optional[MotionCommand]:
        """Return the forward command; this bottom layer always wants control."""
        return self._command


class PriorityArbiter:
    """Pick one behavior's command per cycle based on fixed priority order."""

    def __init__(self, behaviors: List[Behavior]):
        """Take behaviors ordered from highest to lowest priority."""
        self._behaviors = list(behaviors)

    @property
    def behaviors(self) -> List[Behavior]:
        """Return the behaviors in priority order."""
        return list(self._behaviors)

    def arbitrate(self, sensors: SensorSnapshot) -> Tuple[str, MotionCommand]:
        """Return the name of the winning behavior and its command.

        Behaviors below the winner are told they were suppressed so that
        fixed action patterns can reset.
        """
        winner_index = None
        winner = ('none', STOP)
        for index, behavior in enumerate(self._behaviors):
            command = behavior.compute(sensors)
            if command is not None:
                winner_index = index
                winner = (behavior.name, command)
                break
        if winner_index is not None:
            for behavior in self._behaviors[winner_index + 1:]:
                behavior.on_suppressed()
        return winner
