"""ROS 2 node that builds an occupancy grid while the robot wanders.

Mapping runs in parallel with the behaviors (it never competes for the
motors), so it lives in its own node. For each laser scan it looks up where
the lidar is in the odom frame, traces the beams into a
:class:`reactive_tb4.occupancy_grid.LogOddsGrid`, and once a second publishes
the result as a ``nav_msgs/OccupancyGrid`` on ``map`` for RViz.
"""

from array import array
import math

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from reactive_tb4.geometry_utils import yaw_from_quaternion
from reactive_tb4.occupancy_grid import LogOddsGrid, pose_compose


class OccupancyMapper(Node):
    """Turns laser scans plus odometry into an occupancy grid."""

    def __init__(self):
        """Declare parameters, allocate the grid and set up ROS interfaces."""
        super().__init__('occupancy_mapper')
        defaults = {
            'map_frame': 'odom',
            'base_frame': 'base_link',
            'resolution': 0.05,       # m per cell
            'width': 16.0,            # m
            'height': 16.0,           # m
            'origin_x': -8.0,         # m, lower-left corner in the odom frame
            'origin_y': -8.0,
            'max_trace': 4.0,         # m, how far free space is traced
            'publish_period': 1.0,    # s
            'scan_topic': 'scan',
            'odom_topic': 'odom',
            'laser_yaw_fallback': math.pi / 2.0,
            'laser_x_fallback': -0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = self._param

        self._grid = LogOddsGrid(p('width'), p('height'), p('resolution'),
                                 p('origin_x'), p('origin_y'))
        self._pose = None                 # (x, y, yaw) of base_link in odom
        self._laser_offset = None         # (x, y, yaw) of lidar in base_link
        self._tf_failures = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom,
                     qos_profile_sensor_data)
        self.create_subscription(LaserScan, p('scan_topic'), self._on_scan,
                     qos_profile_sensor_data)
        self._timer = self.create_timer(p('publish_period'), self._publish_map)
        self.get_logger().info(
            f'Mapper ready: {self._grid.width} x {self._grid.height} cells '
            f'at {p("resolution")} m.')

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _lookup_laser_offset(self, scan_frame: str) -> None:
        """Find where the lidar sits on the robot (only needed once)."""
        try:
            tf = self._tf_buffer.lookup_transform(self._param('base_frame'), scan_frame, Time())
            t, r = tf.transform.translation, tf.transform.rotation
            self._laser_offset = (t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))
        except TransformException:
            self._tf_failures += 1
            if self._tf_failures > 50:
                self._laser_offset = (self._param('laser_x_fallback'), 0.0,
                                      self._param('laser_yaw_fallback'))
                self.get_logger().warn('No lidar TF found, using fallback offset.')

    def _on_scan(self, msg: LaserScan) -> None:
        if self._laser_offset is None:
            self._lookup_laser_offset(msg.header.frame_id)
        if self._pose is None or self._laser_offset is None:
            return
        sx, sy, syaw = pose_compose(*self._pose, *self._laser_offset)
        self._grid.update_from_scan(sx, sy, syaw, msg.ranges, msg.angle_min,
                                    msg.angle_increment, msg.range_min, msg.range_max,
                                    self._param('max_trace'))

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._param('map_frame')
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(self._grid.resolution)
        msg.info.width = self._grid.width
        msg.info.height = self._grid.height
        msg.info.origin.position.x = float(self._grid.origin_x)
        msg.info.origin.position.y = float(self._grid.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', self._grid.to_occupancy_values().tobytes())
        self._map_pub.publish(msg)


def main(args=None):
    """Entry point for ``ros2 run reactive_tb4 occupancy_mapper``."""
    rclpy.init(args=args)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
