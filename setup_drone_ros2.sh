#!/bin/bash
# AVIANS ROS2 Environment Setup Script

export DRONE_WS="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
cd "$DRONE_WS"

# Source conda
source ~/miniconda3/etc/profile.d/conda.sh
conda activate airship_ros2

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "🚁 AVIANS ROS2 environment activated!"
echo "📍 Current directory: $(pwd)"
echo "🐍 Python: $(which python3)"
echo "🤖 ROS2: $ROS_DISTRO"

# Make this available as a command
alias setup_drone_ros2="source $DRONE_WS/setup_drone_ros2.sh"
