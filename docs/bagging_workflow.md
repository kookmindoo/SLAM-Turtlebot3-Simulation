# TurtleBot3 Bagging Workflow

The `tb3_data_tools` ROS 2 package provides a reusable node and launch description that records the TurtleBot3 perception topics into a rosbag2 dataset using `rosbag2_py`. The workflow synchronizes color, depth, 2D LiDAR, 3D LiDAR, and depth point cloud streams.

## 1. Build the workspace

```bash
cd /workspace/SLAM/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Launch the Gazebo simulator

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=true
```

Ensure the following topics are active:

```bash
ros2 topic list | grep -E '/scan|points|camera'
```

## 3. Record synchronized data

Use the provided launch file to start the recorder node with configurable parameters.

```bash
ros2 launch tb3_data_tools record_tb3_dataset.launch.py \
  bag_path:=/data/tb3_runs/run1 \
  storage_id:=sqlite3 \
  use_sim_time:=true
```

### What the launch file does

1. Starts the `tb3_multi_topic_recorder` node.
2. Reads `config/recorder_topics.yaml` to map topics and message types.
3. Uses `rosbag2_py.SequentialWriter` to serialize each subscribed topic into a single bag.
4. Tracks the latest timestamp from each stream and publishes a `/tb3_data_tools/recorder_state` message whenever all topics fall within the configured `sync_slop` window, which provides synchronization metadata for QA.

### Customizing topics or QoS

Edit `ros2_ws/src/tb3_data_tools/config/recorder_topics.yaml` to add/remove topics or tweak QoS overrides. The node reads this file on startup via ROS parameters.

### Playing back the dataset

```bash
ros2 bag play /data/tb3_runs/run1
```

### Metadata and QA

The node publishes `/tb3_data_tools/recorder_state` diagnostic messages and writes `bag_path/metadata.yaml` with topic counts. Inspect `ros2 topic echo /tb3_data_tools/recorder_state` to confirm all sensors are being recorded.
