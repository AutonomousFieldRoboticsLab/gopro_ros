#!/bin/bash
# Validation test for the gopro_ros Docker images (ROS 1 or ROS 2).
#
# Runs both executables on an empty input folder: they must start, read all their launch
# parameters, write a valid empty output and exit cleanly.
set -euo pipefail

empty_dir=$(mktemp -d)
out_dir=$(mktemp -d)

if [ "${ROS_VERSION}" = "1" ]; then
  roslaunch gopro_ros gopro_to_rosbag.launch \
    gopro_folder:="${empty_dir}" multiple_files:=true rosbag:="${out_dir}/gopro.bag"
  test -f "${out_dir}/gopro.bag"

  roslaunch gopro_ros gopro_to_asl.launch \
    gopro_folder:="${empty_dir}" multiple_files:=true asl_dir:="${out_dir}/asl"
else
  for storage in mcap db3; do
    ros2 launch gopro_ros gopro_to_rosbag.launch.py \
      gopro_folder:="${empty_dir}" multiple_files:=true storage_id:=."${storage}" \
      rosbag:="${out_dir}/gopro_${storage}"
    test -f "${out_dir}/gopro_${storage}/metadata.yaml"
    ls "${out_dir}/gopro_${storage}"/*."${storage}" > /dev/null
  done

  ros2 launch gopro_ros gopro_to_asl.launch.py \
    gopro_folder:="${empty_dir}" multiple_files:=true asl_dir:="${out_dir}/asl"
fi

test -f "${out_dir}/asl/mav0/cam0/data.csv"
test -f "${out_dir}/asl/mav0/imu0/data.csv"

echo "Validation test passed (ROS ${ROS_VERSION}, ${ROS_DISTRO})"
