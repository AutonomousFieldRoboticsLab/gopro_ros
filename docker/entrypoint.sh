#!/bin/bash
set -e

# Source ROS and the gopro_ros workspace, then run the given command. The arguments are cleared
# while sourcing, since the ROS 1 setup scripts would otherwise try to parse them.
command=("$@")
set --
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/gopro_ws/install/setup.bash"

exec "${command[@]}"
