# TurtleBot3 ROS 2 Jazzy Environment Setup

This document captures the complete workflow for installing ROS 2 Jazzy, configuring the TurtleBot3 simulator stack, and validating the sensor outputs required for dataset recording. All commands assume Ubuntu 24.04 (Noble) with sudo privileges.

> **Note:** The default container image used for this repository cannot reach the upstream apt mirrors (403 from proxy), so the actual installation cannot be performed automatically here. Run the commands below on a machine with working apt access.

## 1. Install ROS 2 Jazzy desktop + development tools

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo \"$VERSION_CODENAME\") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-rosdep
sudo rosdep init
rosdep update
```

## 2. TurtleBot3 packages + Gazebo assets

```bash
sudo apt install -y ros-jazzy-turtlebot3* ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros-control ros-jazzy-turtlebot3-simulations
```

The `ros-jazzy-turtlebot3-simulations` metapackage provides Gazebo/Humble-compatible models, including multi-sensor URDFs.

## 3. Environment variables and workspace overlay

Add the following to `~/.bashrc` (or the shell profile that `colcon` uses):

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DISTRO=jazzy
```

If you use this repository’s workspace, also source it after building:

```bash
source /workspace/SLAM/ros2_ws/install/setup.bash
```

## 4. Launch the TurtleBot3 multi-sensor Gazebo world

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=true
```

The waffle_pi model enables the 3D LiDAR and RGB-D camera. For 2D/3D LiDAR at the same time, use the `turtlebot3_multi_sensor.launch.py` file from the simulations package or load the URDF overlay in RViz.

### Verify TF tree and sensor topics

1. **TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```
   or stream:
   ```bash
   ros2 run rqt_tf_tree rqt_tf_tree
   ```
2. **Sensor topics:**
   ```bash
   ros2 topic list | grep -E '/scan|points|camera'
   ros2 topic echo /scan --once
   ros2 topic echo /camera/depth/points --once
   ```

You should see `/scan` (2D LiDAR), `/points` (3D LiDAR), `/camera/color/image_raw`, `/camera/depth/image_raw`, and `/camera/depth/points`.

## 5. Launch files in this repository

After building the workspace with `colcon build`, you can launch the recorder stack that subscribes to the required topics and writes synchronized rosbag files:

```bash
source /workspace/SLAM/ros2_ws/install/setup.bash
ros2 launch tb3_data_tools record_tb3_dataset.launch.py bag_path:=/data/tb3_runs/run1
```

See [bagging_workflow.md](bagging_workflow.md) for details.
