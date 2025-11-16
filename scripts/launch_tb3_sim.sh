#!/usr/bin/env bash
set -euo pipefail

: "${TURTLEBOT3_MODEL:=waffle_pi}"
source /opt/ros/jazzy/setup.bash

echo "Launching TurtleBot3 Gazebo world with model ${TURTLEBOT3_MODEL}"
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=true
