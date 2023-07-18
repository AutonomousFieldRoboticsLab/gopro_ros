# gopro_ros

This repository contains code for parsing GoPro telemetry metadata to obtain GoPro images with synchronized IMU measurements. The GoPro visual-inertial data can then be saved in [rosbag](http://wiki.ros.org/rosbag) or [Euroc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format. Thus, effectively paving the way for visual-inertial odometry/SLAM for GoPro cameras.

This repository use [gpmf-parser](https://github.com/gopro/gpmf-parser)  from [GoPro](https://gopro.com) to extract metadata and timing information from GoPro cameras.

## Related Paper
If you find the code useful in your research, please cite our paper
```
@inproceedings{joshi_gopro_icra_2022,
  author      = {Bharat Joshi and Marios Xanthidis and Sharmin Rahman and Ioannis Rekleitis},
  title       = {High Definition, Inexpensive, Underwater Mapping},
  booktitle   = {IEEE International Conference on Robotics and Automation (ICRA)},
  year        = {2022},
  pages       = {1113-1121},
  doi         = {10.1109/ICRA46639.2022.9811695},
  abbr        = {ICRA},
  bibtex_show = {true},
  code        = {https://github.com/AutonomousFieldRoboticsLab/gopro_ros},
}
```

# Installation

Tested on Ubuntu 18.04 (ros-melodic) & 20.04 (ros-noetic). The install instructions are for Ubuntu 18.04.

## Prerequisites

- ros-melodic-desktop-full
- [OpenCV](https://github.com/opencv/opencv) >= 3.2
- [FFmpeg](http://ffmpeg.org/)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Install Dependencies

- Install ROS using [this guide](http://wiki.ros.org/ROS/Installation)

- System installation of OpenCV should work

```bash
sudo apt install libopencv-dev
```

- Install FFmpeg

```bash
sudo apt install ffmpeg
```

- Install Eigen3

```bash
sudo apt install libeigen3-dev
```

## Install gopro_ros

Before proceeding, ensure all dependencies are installed. To install gopro_ros

```bash
mkdir -p ~/gopro_ws/src
cd gopro_ws/src
git clone https://github.com/joshi-bharat/gopro_ros.git
cd ~/gopro_ws/
catkin_make
source ~/gopro_ws/devel/setup.bash
# add this to ~/.bashrc to make this permanent 
```

# Usage

GoPro splits video into smaller chunks. By splitting up the video it reduces the chance of you losing all your footage if the file gets corrupted somehow. It’s called chaptering, and the idea is that if one chapter gets corrupted the others should still be okay because they’re separate files.

## Save to rosbag

To save GoPro video with IMU measurements to rosbag:

```bash
roslaunch gopro_ros gopro_to_rosbag.launch gopro_video:=<gopro_video_file> rosbag:=<bag_file>
```

If you have multiple files from a single session, put all videos in same folder you can use the following command to concatenate into a single rosbag:

```bash
roslaunch gopro_ros gopro_to_rosbag.launch gopro_folder:=<folder_with_gopro_video_files> multiple_files:=true rosbag:=<bag_file>
```

## Save in Euroc format

To save GoPro video with IMU measurements in Euroc format:

```bash
roslaunch gopro_ros gopro_to_rosbag.launch gopro_video:=<gopro_video_file> asl_dir:=<asl_format_dir>
```

If you have multiple files from a single session, put all videos in same folder you can use the following command extract all videos in a single folder:

```bash
roslaunch gopro_ros gopro_to_rosbag.launch gopro_folder:=<folder_with_gopro_video_files> multiple_files:=true asl_dir:=<asl_format_dir>
```
# TODO:
Extraction video takes a lot of time. Implement multi-threaded.
