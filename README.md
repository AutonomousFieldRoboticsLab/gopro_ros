# GoProROS

[![ROS 1 Workflow](https://github.com/AutonomousFieldRoboticsLab/gopro_ros2/actions/workflows/build_ros1.yml/badge.svg)](https://github.com/AutonomousFieldRoboticsLab/gopro_ros2/actions/workflows/build_ros1.yml)
[![ROS 2 Workflow](https://github.com/AutonomousFieldRoboticsLab/gopro_ros2/actions/workflows/build_ros2.yml/badge.svg)](https://github.com/AutonomousFieldRoboticsLab/gopro_ros2/actions/workflows/build_ros2.yml)

Extract time-synchronized images and IMU measurements from GoPro videos and save them as a
**ROS 1 bag**, a **ROS 2 bag** (MCAP or SQLite3), or in the
[EuRoC/ASL](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format,
ready for visual-inertial odometry and SLAM.

Timing and IMU data are read from the GoPro GPMF telemetry track with GoPro's
[gpmf-parser](https://github.com/gopro/gpmf-parser); images are decoded with FFmpeg and stamped on
the same clock.

## Supported platforms

| ROS | Distro | Ubuntu | Docker image |
|---|---|---|---|
| ROS 1 | Noetic | 20.04 | `docker/Dockerfile_ros1_20_04` |
| ROS 2 | Humble | 22.04 | `docker/Dockerfile_ros2_22_04` |
| ROS 2 | Jazzy | 24.04 | `docker/Dockerfile_ros2_24_04` |

The same package builds for both ROS versions: CMake detects whether catkin or ament is sourced and
builds the matching executables (the approach used by [OpenVINS](https://github.com/rpng/open_vins)).

## Performance

Video frames are decoded on the GPU when available (NVIDIA NVDEC or VAAPI, with automatic fallback
to the CPU), and scaling, color conversion and JPEG/PNG encoding run in parallel worker threads.

| Video (1080p HEVC, 45 Mbps) | Length | CPU decoding | GPU decoding (NVDEC) |
|---|---|---|---|
| Single chapter | 11.9 min | 187 s | **31 s** |
| Two chapters combined | 14.5 min | | **42 s** |

With NVDEC, conversion runs at about 20x real time: one hour of video takes about 3 minutes. Measured
on an Intel Core Ultra 7 155H with an NVIDIA RTX 500 Ada.

## Output

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/gopro/image_raw` | `sensor_msgs/Image` | `gopro` | `bgr8`, or `mono8` with `grayscale:=true` |
| `/gopro/image_raw/compressed` | `sensor_msgs/CompressedImage` | `gopro` | JPEG, written instead of the above with `compressed_image_format:=true` |
| `/gopro/imu` | `sensor_msgs/Imu` | `body` | Accelerometer + gyroscope |
| `/gopro/magnetic_field` | `sensor_msgs/MagneticField` | `body` | Only if the camera records a magnetometer stream |

The EuRoC exporter writes `mav0/cam0/data/<timestamp>.png`, `mav0/cam0/data.csv` and
`mav0/imu0/data.csv` under `asl_dir`.

# Installation

## Docker (recommended)

[`docker-compose.yml`](docker-compose.yml) defines one service per distro: `gopro_ros_noetic`,
`gopro_ros_humble` and `gopro_ros_jazzy`. The folder in `DATA_DIR` is mounted at `/gopro_ws/data`
(default: `./data`); you can set it once in a `.env` file next to `docker-compose.yml`:

```bash
echo "DATA_DIR=/path/to/your/data" > .env
```

Build an image from the repository root:

```bash
docker compose build gopro_ros_jazzy
```

Run a conversion (ROS 2):

```bash
docker compose run --rm gopro_ros_jazzy ros2 launch gopro_ros gopro_to_rosbag.launch.py gopro_video:=/gopro_ws/data/GX010001.MP4 rosbag:=/gopro_ws/data/gopro_run
```

or with ROS 1:

```bash
docker compose run --rm gopro_ros_noetic roslaunch gopro_ros gopro_to_rosbag.launch gopro_video:=/gopro_ws/data/GX010001.MP4 rosbag:=/gopro_ws/data/gopro_run.bag
```

Run `docker compose run --rm gopro_ros_jazzy` without a command for an interactive shell. For
`display_images:=true`, allow X11 access on the host first with `xhost +local:docker`.

### GPU decoding in Docker

With an NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html),
add [`docker-compose.nvidia.yml`](docker-compose.nvidia.yml) to decode the video on the GPU, which
is several times faster. Enable it once in `.env`:

```bash
echo "COMPOSE_FILE=docker-compose.yml:docker-compose.nvidia.yml" >> .env
```

Without it, the same images decode on the CPU. The log shows which decoder is used
(`Using hardware video decoding (cuda)`).

Containers run as root by default, so output files are owned by root. To keep your own user, add
`--user $(id -u):$(id -g) -e HOME=/tmp` to `docker compose run`.

## Build from source

All dependencies (ROS packages, OpenCV, Eigen, FFmpeg headers) are declared in `package.xml` and
installed by `rosdep`.

### ROS 2 (Humble / Jazzy)

```bash
mkdir -p ~/gopro_ws/src && cd ~/gopro_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/gopro_ros2.git
cd ~/gopro_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select gopro_ros
source install/setup.bash
```

### ROS 1 (Noetic)

```bash
mkdir -p ~/gopro_ws/src && cd ~/gopro_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/gopro_ros2.git
cd ~/gopro_ws
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```

### Hardware decoding

GPU decoding works with the FFmpeg packages from Ubuntu and needs no extra build step:

- **NVIDIA (NVDEC):** the NVIDIA driver must be installed.
- **Intel / AMD (VAAPI):** a VA-API driver must be installed (`intel-media-va-driver` or
  `mesa-va-drivers`).

If neither is available, decoding falls back to the CPU. Set `hardware_decoding:=false` to always
decode on the CPU.

### CMake options

| Option | Default | Description |
|---|---|---|
| `BUILD_GOPRO_TO_ASL` | `ON` | Build the EuRoC/ASL exporter |
| `ENABLE_ROS` | `ON` | Build the ROS executables; when `OFF` (or no ROS is found), only the core library is built |

# Usage

Every executable has a ROS 2 launch file (`*.launch.py`) and a ROS 1 launch file (`*.launch`) with
the same arguments.

## ROS 2 bag

`rosbag` is the output **bag directory**. Choose the storage backend with `storage_id`:

```bash
ros2 launch gopro_ros gopro_to_rosbag.launch.py \
    gopro_video:=/path/to/GX010001.MP4 \
    rosbag:=/path/to/output/gopro_run \
    storage_id:=.mcap \
    mcap_compression:=zstd_fast
```

This creates `gopro_run/` with `metadata.yaml` and `gopro_run_0.mcap`. Use `storage_id:=.db3` for
SQLite3. The output directory must not exist yet.

## ROS 1 bag

`rosbag` is the output bag file (`.bag` is appended if missing):

```bash
roslaunch gopro_ros gopro_to_rosbag.launch \
    gopro_video:=/path/to/GX010001.MP4 \
    rosbag:=/path/to/output/gopro_run.bag
```

## EuRoC / ASL format

```bash
ros2 launch gopro_ros gopro_to_asl.launch.py \
    gopro_video:=/path/to/GX010001.MP4 \
    asl_dir:=/path/to/output/asl
```

On ROS 1, use `roslaunch gopro_ros gopro_to_asl.launch` with the same arguments.

## Chaptered recordings

GoPro splits long recordings into chapters (`GX010001.MP4`, `GX020001.MP4`, ...). To combine all
chapters of one recording into a single output, put them in a folder and pass it with
`multiple_files:=true`:

```bash
ros2 launch gopro_ros gopro_to_rosbag.launch.py \
    gopro_folder:=/path/to/chapters \
    multiple_files:=true \
    rosbag:=/path/to/output/gopro_run
```

All `.MP4` files in the folder are processed in name order, so the folder should contain the
chapters of **one** recording only. Images and IMU data continue across chapter boundaries without
gaps.

## Parameters

| Parameter | Launch default | Description |
|---|---|---|
| `gopro_video` | | Input video file |
| `gopro_folder` | | Folder with video chapters (used with `multiple_files:=true`) |
| `multiple_files` | `false` | Process all chapters in `gopro_folder` into one output |
| `rosbag` | | Output bag (`gopro_to_rosbag` only) |
| `asl_dir` | | Output directory (`gopro_to_asl` only) |
| `storage_id` | `.mcap` | ROS 2 only: `.mcap` or `.db3` |
| `mcap_compression` | `zstd_fast` | ROS 2 only: `zstd_fast`, `zstd_small` or `none` |
| `scale` | `0.5` | Image scaling factor |
| `compressed_image_format` | `true` | Write JPEG `CompressedImage` instead of raw `Image` |
| `grayscale` | `false` (`true` for ASL) | Convert images to grayscale |
| `display_images` | `false` | Show images while processing |
| `hardware_decoding` | `true` | Decode on the GPU (NVIDIA NVDEC, then VAAPI) if available, otherwise on the CPU |

# Repository layout

```
src/
  core/        GPMF (IMU, timing) and video extraction, ROS-agnostic
  utils/       Time, bag and progress helpers, measurement types, logging, thread pool;
               ROS-agnostic
  ros/         ROS 1 and ROS 2 bag writers with the same interface
  gpmf/        Vendored gpmf-parser (GoPro)
  date/        Vendored date library (Howard Hinnant)
  gopro_to_rosbag.cpp, gopro_to_asl.cpp
cmake/         ROS1.cmake, ROS2.cmake and Findffmpeg.cmake
launch/        ROS 1 (.launch) and ROS 2 (.launch.py) launch files
docker/        Dockerfiles for Noetic, Humble and Jazzy
.github/       CI workflows (build + validation test in each Docker image)
scripts/       Legacy ROS 1 helper scripts
docker-compose.yml          One service per distro
docker-compose.nvidia.yml   Optional GPU access for hardware decoding
```

Code is formatted with the repository's `.clang-format`:

```bash
clang-format -i src/core/*.?pp src/utils/*.?pp src/ros/*.?pp src/*.cpp
```

# Citation

If you find the code useful in your research, please cite our paper:

```bibtex
@inproceedings{joshi_gopro_icra_2022,
  author      = {Bharat Joshi and Marios Xanthidis and Sharmin Rahman and Ioannis Rekleitis},
  title       = {High Definition, Inexpensive, Underwater Mapping},
  booktitle   = {IEEE International Conference on Robotics and Automation (ICRA)},
  year        = {2022},
  pages       = {1113-1121},
  doi         = {10.1109/ICRA46639.2022.9811695},
}
```

# License

BSD 3-Clause, see [LICENSE](LICENSE). The vendored
[gpmf-parser](https://github.com/gopro/gpmf-parser) (`src/gpmf/`) is Apache-2.0 / MIT and the
[date](https://github.com/HowardHinnant/date) library (`src/date/`) is MIT.
