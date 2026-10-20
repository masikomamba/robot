"""ROS 2 node that runs reactive behaviors on a TurtleBot 4.

This node is the "glue" between ROS and :mod:`reactive_tb4.behaviors`:

* sensor callbacks store the newest bumper, lidar, odometry and keyboard data,
* a timer (10 Hz by default) packs that data into a ``SensorSnapshot``,
* the ``PriorityArbiter`` picks one behavior's command,
* the command is published on ``cmd_vel`` as a ``TwistStamped``.

On startup the robot is sitting on its charging dock, so the node first asks
the Create 3 base to undock (a built-in action), then starts the behaviors.
"""

import math
import random
from typing import Optional

from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from reactive_tb4.behaviors import (
    AvoidAsymmetric, DriveForward, EscapeSymmetric, HaltOnBump, KeyboardTeleop,
    MotionCommand, PriorityArbiter, RandomTurn, SensorSnapshot, STOP)
from reactive_tb4.geometry_utils import yaw_from_quaternion
from reactive_tb4.perception import analyze_front


class ReactiveController(Node):
    """Collects sensor data and runs the behavior arbiter at a fixed rate."""

    # Startup phases, in order.
    WAITING = 'waiting_for_sensors'
    UNDOCKING = 'undocking'
    RUNNING = 'running'

    def __init__(self):
        """Declare parameters, build the behaviors and set up ROS interfaces."""
        super().__init__('reactive_controller')
        self._declare_parameters()
        p = self._param

        seed = p('random_seed')
        rng = random.Random(None if seed < 0 else seed)
        self._arbiter = PriorityArbiter([
            HaltOnBump(),
            KeyboardTeleop(),
            EscapeSymmetric(p('turn_speed'), math.radians(p('escape_spread_deg')), rng),
            AvoidAsymmetric(p('avoid_turn_speed')),
            RandomTurn(p('turn_speed'), p('random_turn_interval'),
                       math.radians(p('random_turn_max_deg')), rng),
            DriveForward(p('forward_speed')),
        ])

        # Latest sensor data (None until the first message arrives).
        self._scan: Optional[LaserScan] = None
        self._scan_time: Optional[Time] = None
        self._odom_time: Optional[Time] = None
        self._hazard_time: Optional[Time] = None
        self._yaw: Optional[float] = None
        self._last_xy: Optional[tuple] = None
        self._odometer = 0.0
        self._bumper_pressed = False
        self._teleop: Optional[MotionCommand] = None
        self._teleop_time: Optional[Time] = None
        self._laser_yaw: Optional[float] = None
        self._is_docked: Optional[bool] = None
        self._last_behavior = ''

        self._phase = self.WAITING
        self._start_time = self.get_clock().now()

        # Transform lookup for the lidar mounting angle.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers.
        self._cmd_pub = self.create_publisher(TwistStamped, p('cmd_vel_topic'), 10)
        self._behavior_pub = self.create_publisher(String, 'active_behavior', 10)

        # Subscribers. Sensor-data QoS (best effort) matches any publisher.
        self.create_subscription(LaserScan, p('scan_topic'), self._on_scan,
                     qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom,
                     qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, p('hazard_topic'),
                                 self._on_hazards, qos_profile_sensor_data)
        self.create_subscription(DockStatus, p('dock_status_topic'), self._on_dock_status,
                                 qos_profile_sensor_data)
        self.create_subscription(TwistStamped, p('keyboard_topic'), self._on_keyboard, 10)

        self._undock_client = ActionClient(self, Undock, 'undock')

        self._timer = self.create_timer(1.0 / p('control_rate'), self._control_step)
        self.get_logger().info('Reactive controller started, waiting for sensors.')

    # ------------------------------------------------------------------ setup
    def _declare_parameters(self) -> None:
        """Declare every tunable value with a sensible default."""
        defaults = {
            'control_rate': 10.0,           # Hz
            'forward_speed': 0.20,          # m/s
            'turn_speed': 1.0,              # rad/s, used by escape and random turn
            'avoid_turn_speed': 0.6,        # rad/s
            'robot_radius': 0.17,           # m, center of robot to bumper
            'obstacle_distance': 0.3048,    # m, one foot past the bumper
            'front_half_angle_deg': 60.0,   # beams within +/- this are "in front"
            'symmetry_tolerance': 0.05,     # m
            'escape_spread_deg': 30.0,      # escape turns 180 +/- this
            'random_turn_interval': 0.3048,  # m, one foot
            'random_turn_max_deg': 15.0,
            'teleop_timeout': 0.75,         # s a key press stays in control
            'scan_timeout': 1.0,            # s before a scan is considered stale
            'odom_timeout': 1.0,            # s before odometry is considered stale
            'hazard_timeout': 2.0,          # s before bumper/hazard data is stale
            'laser_yaw_fallback': math.pi / 2.0,  # TB4 lidar is mounted rotated 90 deg
            'tf_wait_time': 5.0,            # s to wait for TF before using fallback
            'base_frame': 'base_link',
            'keyboard_topic': 'keyboard/cmd_vel',
            'scan_topic': 'scan',
            'odom_topic': 'odom',
            'hazard_topic': 'hazard_detection',
            'dock_status_topic': 'dock_status',
            'cmd_vel_topic': 'cmd_vel',
            'undock_on_start': False,
            'undock_wait_time': 20.0,       # s to wait for the dock/undock interfaces
            'random_seed': -1,              # -1 means seed from the system clock
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name: str):
        """Shorthand for reading a parameter value."""
        return self.get_parameter(name).value

    # -------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan) -> None:
        """Store the latest lidar scan and resolve the lidar mounting angle."""
        self._scan = msg
        self._scan_time = self.get_clock().now()
        if self._laser_yaw is None:
            self._resolve_laser_yaw(msg.header.frame_id)

    def _on_odom(self, msg: Odometry) -> None:
        """Update the current yaw and accumulate forward distance traveled."""
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self._last_xy is not None:
            # Only forward motion counts toward the "every 1 ft" random turn.
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            if forward > 0.0:
                self._odometer += forward
        self._last_xy = (x, y)
        self._yaw = yaw
        self._odom_time = self.get_clock().now()

    def _on_hazards(self, msg: HazardDetectionVector) -> None:
        """Record whether the bumper is currently pressed."""
        self._bumper_pressed = any(d.type == HazardDetection.BUMP for d in msg.detections)
        self._hazard_time = self.get_clock().now()

    def _on_dock_status(self, msg: DockStatus) -> None:
        """Record whether the robot is currently on its dock."""
        self._is_docked = msg.is_docked

    def _on_keyboard(self, msg: TwistStamped) -> None:
        """Store the latest teleop command and when it arrived."""
        self._teleop = MotionCommand(msg.twist.linear.x, msg.twist.angular.z)
        self._teleop_time = self.get_clock().now()

    # ---------------------------------------------------------------- helpers
    def _resolve_laser_yaw(self, scan_frame: str) -> None:
        """Look up how the lidar is rotated relative to the robot body."""
        try:
            tf = self._tf_buffer.lookup_transform(self._param('base_frame'), scan_frame, Time())
            r = tf.transform.rotation
            self._laser_yaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)
            self.get_logger().info(
                f'Lidar yaw from TF: {math.degrees(self._laser_yaw):.1f} deg')
        except TransformException:
            waited = self._seconds_since(self._start_time)
            if waited > self._param('tf_wait_time'):
                self._laser_yaw = self._param('laser_yaw_fallback')
                self.get_logger().warn(
                    f'No TF for {scan_frame}, using fallback lidar yaw '
                    f'{math.degrees(self._laser_yaw):.1f} deg')

    def _seconds_since(self, stamp: Time) -> float:
        """Return elapsed time, in seconds, since ``stamp``."""
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _recent_teleop(self) -> Optional[MotionCommand]:
        """Return the latest teleop command, or None if it has timed out."""
        if self._teleop_time is None:
            return None
        if self._seconds_since(self._teleop_time) > self._param('teleop_timeout'):
            return None
        return self._teleop

    def _publish(self, command: MotionCommand, behavior: str) -> None:
        """Publish a velocity command and the name of the behavior that chose it."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._param('base_frame')
        msg.twist.linear.x = float(command.linear)
        msg.twist.angular.z = float(command.angular)
        self._cmd_pub.publish(msg)
        if behavior != self._last_behavior:
            self.get_logger().info(f'Active behavior: {behavior}')
            self._last_behavior = behavior
        self._behavior_pub.publish(String(data=behavior))

    # ------------------------------------------------------------ main loop
    def _control_step(self) -> None:
        """Run once per control period."""
        if self._phase == self.WAITING:
            self._handle_startup()
            if self._phase == self.WAITING:
                self._publish(STOP, 'waiting_for_safe_start')
            return
        if self._phase == self.UNDOCKING:
            return  # The Create 3 drives itself while undocking.

        scan_fresh = (self._scan_time is not None and
                      self._seconds_since(self._scan_time) < self._param('scan_timeout'))
        if not scan_fresh:
            self._publish(STOP, 'no_scan')
            return
        odom_fresh = (self._odom_time is not None and
                      self._seconds_since(self._odom_time) < self._param('odom_timeout'))
        if not odom_fresh:
            self._publish(STOP, 'no_odom')
            return
        hazard_fresh = (self._hazard_time is not None and
                        self._seconds_since(self._hazard_time) <
                        self._param('hazard_timeout'))
        if not hazard_fresh:
            self._publish(STOP, 'no_hazard_data')
            return

        scan = self._scan
        front = analyze_front(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, self._laser_yaw, self._param('robot_radius'),
            math.radians(self._param('front_half_angle_deg')),
            self._param('obstacle_distance'), self._param('symmetry_tolerance'))
        snapshot = SensorSnapshot(
            bumper_pressed=self._bumper_pressed,
            teleop_command=self._recent_teleop(),
            front=front,
            yaw=self._yaw,
            forward_odometer=self._odometer)
        name, command = self._arbiter.arbitrate(snapshot)
        self._publish(command, name)

    def _handle_startup(self) -> None:
        """Wait for sensors, then undock if needed, then start running."""
        if self._scan is None or self._yaw is None or self._laser_yaw is None:
            return
        if not self._param('undock_on_start'):
            if self._is_docked is None:
                return
            if self._is_docked:
                return
            self._begin_running('robot reports it is undocked')
            return
        ready = self._is_docked is not None and self._undock_client.server_is_ready()
        if not ready:
            if self._seconds_since(self._start_time) > self._param('undock_wait_time'):
                self.get_logger().error(
                    'Dock status/undock action unavailable; remaining stopped. '
                    'Verify the robot interfaces and launch only after dock status is valid.')
                self._start_time = self.get_clock().now()
            return
        if not self._is_docked:
            self._begin_running('robot is not docked')
            return
        self._phase = self.UNDOCKING
        self.get_logger().info('Robot is docked, sending undock goal.')
        future = self._undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._on_undock_accepted)

    def _on_undock_accepted(self, future) -> None:
        """Handle the undock goal response, then wait for the undock to finish."""
        handle = future.result()
        if not handle.accepted:
            self._begin_running('undock goal rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda _: self._begin_running('undock finished'))

    def _begin_running(self, reason: str) -> None:
        """Switch to the running phase and log why."""
        self._phase = self.RUNNING
        self.get_logger().info(f'Starting reactive behaviors ({reason}).')

    def stop_robot(self) -> None:
        """Send one zero-velocity command, used on shutdown."""
        self._publish(STOP, 'shutdown')


def main(args=None):
    """Entry point for ``ros2 run reactive_tb4 reactive_controller``."""
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
