#!/usr/bin/env bash
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Please run as root (sudo)." >&2
  exit 1
fi

apt-get update
apt-get install -y software-properties-common curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sh -c "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo \$VERSION_CODENAME) main\" > /etc/apt/sources.list.d/ros2.list"
apt-get update
apt-get install -y ros-jazzy-desktop ros-dev-tools python3-rosdep2 \
  ros-jazzy-turtlebot3* ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros-control ros-jazzy-turtlebot3-simulations

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep not installed correctly" >&2
  exit 1
fi

rosdep init || true
rosdep update

echo "export TURTLEBOT3_MODEL=waffle_pi" >> /etc/profile.d/turtlebot3.sh
echo "source /opt/ros/jazzy/setup.bash" >> /etc/profile.d/turtlebot3.sh

cat <<PROFILE
Installation complete.
Open a new shell or run:
  source /opt/ros/jazzy/setup.bash
  export TURTLEBOT3_MODEL=waffle_pi
PROFILE
