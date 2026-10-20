# Project 1: Reactive TurtleBot 4 (ROS 2 Jazzy + Gazebo Harmonic)

A simulated TurtleBot 4 wanders a 15 ft x 10 ft room and a backwards-L hallway using a
priority-based (pseudo-subsumption) reactive architecture, while building an occupancy grid.

For a physical TurtleBot 4, see [REAL_ROBOT.md](REAL_ROBOT.md). The real-robot launch
connects to an already-running robot; it does not start Gazebo or robot bringup. It is
experimental software and is not a safety-rated controller.

Behaviors, highest priority first:

1. Halt on bumper collision
2. Keyboard teleop
3. Escape symmetric obstacles within 1 ft (fixed action pattern, turn 180 +/- 30 deg)
4. Avoid asymmetric obstacles within 1 ft (reflex)
5. Random turn, uniform in +/-15 deg, after every 1 ft of forward travel
6. Drive forward

Plus: occupancy grid mapping on `/map` (separate node, runs in parallel).

## Layout

```
reactive_tb4/
  config/reactive_params.yaml    tunable parameters
  launch/reactive_tb4.launch.py  runs everything (sim + controller + mapper + RViz)
  launch/real_robot.launch.py    controller + mapper for an already-running robot
  launch/room_hall_sim.launch.py Gazebo + TurtleBot 4 in our world only
  reactive_tb4/behaviors.py      the six behaviors + PriorityArbiter (no ROS)
  reactive_tb4/perception.py     lidar -> left/right front clearance (no ROS)
  reactive_tb4/occupancy_grid.py log-odds grid + ray tracing (no ROS)
  reactive_tb4/behavior_controller.py  ROS node for the behaviors
  reactive_tb4/occupancy_mapper.py     ROS node for mapping
  rviz/reactive_tb4.rviz         top-down view of map + scan + robot
  test/                          unit tests + ament_flake8 / ament_pep257
  worlds/room_hall.sdf           the room and hallway
tools/gen_world.py               regenerates room_hall.sdf from the foot measurements
```

## Running on a CSN Linux box

Assumes ROS 2 Jazzy and the TurtleBot 4 simulator are installed. Pick your own `ROS_DOMAIN_ID`.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
mkdir -p ~/ros2_ws/src
cp -r ~/Downloads/project_1_ML-main/reactive_tb4 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select reactive_tb4
source install/setup.bash
ros2 launch reactive_tb4 reactive_tb4.launch.py
```

Keyboard control, in a second terminal (hold i / j / l / , to drive):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=keyboard/cmd_vel
```

Watch which behavior is in control:

```bash
ros2 topic echo /active_behavior
```

## Running the tests

```bash
cd ~/ros2_ws
colcon test --packages-select reactive_tb4
colcon test-result --verbose
```

## Useful launch options

```bash
ros2 launch reactive_tb4 reactive_tb4.launch.py rviz:=false
ros2 launch reactive_tb4 reactive_tb4.launch.py x:=1.0 y:=1.0 yaw:=1.57
ros2 launch reactive_tb4 reactive_tb4.launch.py sim:=false
```

For hardware, follow [REAL_ROBOT.md](REAL_ROBOT.md) and use
`ros2 launch reactive_tb4 real_robot.launch.py` instead.
