# SLAM Workspace Utilities

This repository documents how to spin up a TurtleBot3 ROS 2 Jazzy simulator and provides a ROS 2 package (`tb3_data_tools`) for recording synchronized datasets from the robot’s primary perception topics.

## Contents

| Path | Description |
| --- | --- |
| `docs/turtlebot3_setup.md` | Step-by-step guide for installing ROS 2 Jazzy, TurtleBot3 assets, and launching Gazebo with the correct sensors. |
| `docs/bagging_workflow.md` | Instructions for running the dataset bagging workflow. |
| `ros2_ws/src/tb3_data_tools` | ROS 2 Python package with recorder nodes and launch files. |
| `scripts/` | Helper scripts for installing dependencies and launching simulations. |

## Quick start

1. Follow the installation steps in `docs/turtlebot3_setup.md`.
2. Build the workspace:
   ```bash
   cd /workspace/SLAM/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Launch the TurtleBot3 Gazebo world:
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
4. Start the recorder:
   ```bash
   ros2 launch tb3_data_tools record_tb3_dataset.launch.py bag_path:=/tmp/tb3_run1
   ```

See the docs folder for validation steps (TF tree, topic verification, rosbag playback).
