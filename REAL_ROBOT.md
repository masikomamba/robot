# Running on a physical TurtleBot 4

This package can connect to a real TurtleBot 4, but it is **experimental reactive behavior software, not a safety-rated controller**. It has not been validated on your particular robot. Run it only in a clear, controlled area with a person at the physical emergency stop. Start at low speed and supervise continuously.

The robot's official TurtleBot 4 bringup must already be running. This package does not start or replace that bringup. It does not start Gazebo in the real-robot launch.

## Compatibility and prerequisites

- Physical TurtleBot 4 with its normal robot software running. The launch is intended for the TurtleBot 4 Jazzy software stack. Jazzy uses stamped velocity commands in the current TurtleBot 4 stack; this node publishes `geometry_msgs/msg/TwistStamped`.
- A ROS 2 Jazzy computer on the same ROS 2 network as the robot. For best control reliability, run the package on the robot's Raspberry Pi, or use a reliable wired/nearby network and verify ROS discovery first.
- The robot must expose compatible `scan`, `odom`, `hazard_detection`, `dock_status`, and `cmd_vel` interfaces. Names are relative so a robot namespace can be applied. Confirm topic types and namespace on your specific software image before moving the robot.
- Do not run another controller, navigation stack, teleop publisher, or test command that is also commanding the robot on `cmd_vel`.

## Before connecting the behavior controller

1. Make sure the robot is on the floor, in a large clear area, with a person holding or immediately able to reach its physical emergency stop. Remove nearby people, pets, cables, stairs, and fragile objects. Do not rely on the lidar or software stop as an emergency stop.
2. Set up ROS networking according to the TurtleBot 4 manual. On both computers, use the same `ROS_DOMAIN_ID` and a compatible RMW/DDS configuration. The TurtleBot 4 uses a Create 3 republisher; use the topics exposed in the robot's main namespace, not the hidden `_do_not_use` namespace.
3. Source ROS and the workspace, then check the robot interfaces **before launching this controller**:

   ```bash
   ros2 topic list
   ros2 topic type /scan
   ros2 topic type /odom
   ros2 topic type /hazard_detection
   ros2 topic type /dock_status
   ros2 topic type /cmd_vel
   ros2 topic info /cmd_vel -v
   ros2 action list -t
   ```

   Expected types are `sensor_msgs/msg/LaserScan` for scan, `nav_msgs/msg/Odometry` for odometry, `irobot_create_msgs/msg/HazardDetectionVector` for hazards, `irobot_create_msgs/msg/DockStatus` for dock status, and `geometry_msgs/msg/TwistStamped` for velocity commands. If the Create 3 hazard topic is missing, verify the TurtleBot 4 Create 3 republisher configuration exposes it; the controller intentionally remains stopped without fresh hazard data. If names or types differ, do not start the controller until they have been verified and the configuration or code has been adapted. A ROS topic name alone does not establish message-type compatibility.
4. Ensure the robot is **undocked** before running. The real-robot launch deliberately disables automatic undocking and will wait for a valid dock status reporting that the robot is undocked. It will keep publishing zero velocity while it waits. If the dock status topic is unavailable, diagnose it; do not bypass this check just to make the node start.
5. Inspect and, if needed, reduce `forward_speed`, `turn_speed`, `avoid_turn_speed`, and `obstacle_distance` in `reactive_tb4/config/reactive_params.yaml`. The default is a conservative forward speed but has not been safety-tested on hardware.

## Build and start

Build the package in a ROS 2 Jazzy workspace on the robot or a network-connected Ubuntu 24.04 computer:

```bash
cd ~/ros2_ws
colcon build --packages-select reactive_tb4
source install/setup.bash
```

After confirming the robot has been manually undocked and ROS discovery works, start the hardware-connected nodes:

```bash
ros2 launch reactive_tb4 real_robot.launch.py
```

RViz is disabled by default. To enable it, add `rviz:=true`. If the robot publishes under a namespace, pass it, for example `robot_namespace:=my_robot` (without a leading slash):

```bash
ros2 launch reactive_tb4 real_robot.launch.py robot_namespace:=my_robot rviz:=true
```

For a robot with non-default topic names, set the corresponding `scan_topic`, `odom_topic`, `hazard_topic`, `dock_status_topic`, and `cmd_vel_topic` parameters in `config/reactive_params.yaml`. Mapper input topics use `scan_topic` and `odom_topic` under the `occupancy_mapper` section. Keep names relative when using a namespace. Rebuild/source the workspace after changing installed package files, or use a ROS parameter override file.

## Supervised first test

- Before movement, confirm live scan/odometry, correct lidar orientation in the logs, correct dock status, and zero command while waiting.
- For the first movement test, keep the robot on the floor in a large empty area, reduce speeds, and have a person immediately at the physical emergency stop. Confirm forward direction and obstacle response at low speed before any longer run.
- Stop the launch with Ctrl+C and verify the base comes to a stop. Also test the physical emergency stop independently before operation.
- Do not run unattended. This project is not navigation, SLAM, collision certification, or a replacement for the Create 3's built-in safety features. Lidar coverage is limited to a front sector; the mapper is odometry-based and can drift.

## Simulation remains separate

The existing simulation entry point remains available:

```bash
ros2 launch reactive_tb4 reactive_tb4.launch.py
```

For controller and mapper nodes connected to a robot, use `real_robot.launch.py` instead; do not use the simulation launch or `use_sim_time:=true` on hardware.
