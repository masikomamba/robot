# Running this on a Windows 11 PC (WSL2 + Ubuntu 24.04)

ROS 2 Jazzy needs Ubuntu 24.04. On Windows the easiest way is WSL2, which also shows Linux GUI
apps (Gazebo, RViz) as normal windows.

## 1. Install Ubuntu 24.04 in WSL2

Open PowerShell as Administrator:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot if asked, then open "Ubuntu 24.04" from the Start menu and make a username/password.
Also update your NVIDIA driver from the NVIDIA app so WSL gets GPU acceleration.

## 2. Install ROS 2 Jazzy (inside the Ubuntu terminal)

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

## 3. Install the TurtleBot 4 simulator and teleop

```bash
sudo apt install -y ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes ros-jazzy-teleop-twist-keyboard
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Sanity check that the stock sim works (Gazebo should open with a robot in a warehouse):

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

Ctrl+C it once you see the robot.

## 4. Build and run the project

Your Windows Downloads folder is at `/mnt/c/Users/<YourWindowsName>/Downloads` inside WSL.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
unzip /mnt/c/Users/<YourWindowsName>/Downloads/project_1_ML-main.zip
mv project_1_ML-main/reactive_tb4 .
cd ~/ros2_ws
colcon build --packages-select reactive_tb4
source install/setup.bash
ros2 launch reactive_tb4 reactive_tb4.launch.py
```

Second Ubuntu tab for the keyboard:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=keyboard/cmd_vel
```

## Troubleshooting

Gazebo window is black or crashes on start:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch reactive_tb4 reactive_tb4.launch.py
```

Robot never moves: check the controller log says "Starting reactive behaviors", then check the
lidar and bumper topics are alive:

```bash
ros2 topic hz /scan
ros2 topic echo /hazard_detection --once
ros2 topic echo /active_behavior
```

Robot turns toward walls instead of away: the lidar angle came out wrong. The log prints
"Lidar yaw from TF"; it should be about 90 deg.
