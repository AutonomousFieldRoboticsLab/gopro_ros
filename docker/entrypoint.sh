#!/bin/bash
set -e

# Source ROS and the gopro_ros workspace, then run the given command
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/gopro_ws/install/setup.bash"

exec "$@"
