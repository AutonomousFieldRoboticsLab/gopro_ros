//
// Created by bjoshi on 10/29/20.
//

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "core/imu_extractor.hpp"
#include "core/video_extractor.hpp"
#include "utils/measurements.hpp"
#include "utils/print.hpp"

#if ROS_AVAILABLE == 1
#include <ros/ros.h>

#include "ros/ros1_bag_writer.hpp"
#elif ROS_AVAILABLE == 2
#include <rclcpp/rclcpp.hpp>

#include "ros/ros2_bag_writer.hpp"
#endif

namespace fs = std::filesystem;

using gopro_ros2::AcclMeasurement;
using gopro_ros2::GoProImuExtractor;
using gopro_ros2::GoProVideoExtractor;
using gopro_ros2::GyroMeasurement;
using gopro_ros2::MagMeasurement;
using gopro_ros2::Timestamp;

namespace {

void shutdown() {
#if ROS_AVAILABLE == 1
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif
}

void printPayloadStamps(const std::string& label,
                        const std::vector<uint64_t>& start_stamps,
                        const std::vector<uint32_t>& samples) {
  PRINT_INFO("[" << label << "] Payloads: " << start_stamps.size() << " Start stamp: "
                 << start_stamps[0] << " End stamp: " << start_stamps[samples.size() - 1]
                 << " Total Samples: " << samples.at(samples.size() - 1));
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string gopro_video;
  std::string gopro_folder;
  std::string rosbag;
  std::string storage_id;
  std::string mcap_compression;
  double scaling;
  bool compress_images;
  bool grayscale;
  bool display_images;
  bool multiple_files;

#if ROS_AVAILABLE == 1
  ros::init(argc, argv, "gopro_to_rosbag");
  ros::NodeHandle nh("~");
  nh.param<std::string>("gopro_video", gopro_video, "");
  nh.param<std::string>("gopro_folder", gopro_folder, "");
  nh.param<std::string>("rosbag", rosbag, "");
  nh.param<double>("scale", scaling, 1.0);
  nh.param<bool>("compressed_image_format", compress_images, false);
  nh.param<bool>("grayscale", grayscale, false);
  nh.param<bool>("display_images", display_images, false);
  nh.param<bool>("multiple_files", multiple_files, false);
#elif ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gopro_to_rosbag");
  gopro_video = node->declare_parameter<std::string>("gopro_video", "");
  gopro_folder = node->declare_parameter<std::string>("gopro_folder", "");
  rosbag = node->declare_parameter<std::string>("rosbag", "");
  storage_id = node->declare_parameter<std::string>("storage_id", ".mcap");
  mcap_compression = node->declare_parameter<std::string>("mcap_compression", "zstd_fast");
  scaling = node->declare_parameter<double>("scale", 1.0);
  compress_images = node->declare_parameter<bool>("compressed_image_format", false);
  grayscale = node->declare_parameter<bool>("grayscale", false);
  display_images = node->declare_parameter<bool>("display_images", false);
  multiple_files = node->declare_parameter<bool>("multiple_files", false);
#endif

  bool is_gopro_video = !gopro_video.empty();
  bool is_gopro_folder = !gopro_folder.empty();

  if (!is_gopro_video && !is_gopro_folder) {
    PRINT_ERROR("Please specify the gopro video or folder");
    shutdown();
    return 1;
  }

  if (rosbag.empty()) {
    PRINT_ERROR("No rosbag file specified");
    shutdown();
    return 1;
  }

#if ROS_AVAILABLE == 1
  gopro_ros2::ROS1BagWriter bag_writer(rosbag);
#elif ROS_AVAILABLE == 2
  gopro_ros2::ROS2BagWriter bag_writer(rosbag, storage_id, mcap_compression);
#endif

  std::vector<fs::path> video_files;

  if (is_gopro_folder && multiple_files) {
    std::copy(fs::directory_iterator(gopro_folder),
              fs::directory_iterator(),
              std::back_inserter(video_files));
    std::sort(video_files.begin(), video_files.end());
  } else {
    video_files.push_back(fs::path(gopro_video));
  }

  auto end = std::remove_if(video_files.begin(), video_files.end(), [](const fs::path& p) {
    return p.extension() != ".MP4" || fs::is_directory(p);
  });
  video_files.erase(end, video_files.end());

  std::vector<uint64_t> start_stamps;
  std::vector<uint32_t> samples;
  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;
  std::deque<MagMeasurement> magnetometer_queue;

  std::vector<uint64_t> image_stamps;

  bool has_magnetic_field_readings = false;

  // Read from each video chunk and write video to rosbag
  for (uint32_t i = 0; i < video_files.size(); i++) {
    image_stamps.clear();

    PRINT_WARNING("Opening Video File: " << video_files[i].filename().string());

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling, true);

    if (i == 0 && imu_extractor.getNumOfSamples(STR2FOURCC("MAGN"))) {
      has_magnetic_field_readings = true;
    }

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    printPayloadStamps("ACCL", start_stamps, samples);
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    printPayloadStamps("GYRO", start_stamps, samples);
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    printPayloadStamps("Image", start_stamps, samples);
    if (has_magnetic_field_readings) {
      imu_extractor.getPayloadStamps(STR2FOURCC("MAGN"), start_stamps, samples);
      printPayloadStamps("MAGN", start_stamps, samples);
    }

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    uint64_t video_end_stamp = 0;
    uint64_t magnetometer_end_stamp = 0;

    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
      video_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("CORI"), 0);
      if (has_magnetic_field_readings) {
        magnetometer_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("MAGN"), 0);
      }
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);
    imu_extractor.readMagnetometerData(magnetometer_queue, magnetometer_end_stamp);

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count) {
      PRINT_ERROR("Video and metadata frame count do not match");
      shutdown();
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      PRINT_ERROR("Video creation time does not match");
      shutdown();
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      PRINT_ERROR("ffmpeg and gpmf frame count does not match. " << image_stamps.size() << " vs "
                                                                 << ffmpeg_frame_count);
      shutdown();
    }

    video_extractor.processFrames(
        image_stamps, grayscale, display_images, [&](const cv::Mat& image, uint64_t stamp_ns) {
          bag_writer.writeImage("/gopro/image_raw", image, stamp_ns, compress_images);
        });
  }

  // Write IMU data
  PRINT_INFO("[ACCL] Payloads: " << accl_queue.size());
  PRINT_INFO("[GYRO] Payloads: " << gyro_queue.size());

  assert(accl_queue.size() == gyro_queue.size());

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    int64_t diff = accl.timestamp - gyro.timestamp;
    Timestamp stamp;

    if (std::abs(diff) > 100000) {
      // I will need to handle this case more carefully
      PRINT_WARNING(diff << " ns difference between gyro and accl");
      stamp = static_cast<Timestamp>(
          (static_cast<double>(accl.timestamp) + static_cast<double>(gyro.timestamp)) / 2.0);
    } else {
      stamp = accl.timestamp;
    }

    bag_writer.writeImu("/gopro/imu", accl, gyro, stamp);

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  // Write magnetometer data
  while (!magnetometer_queue.empty()) {
    bag_writer.writeMagneticField("/gopro/magnetic_field", magnetometer_queue.front());
    magnetometer_queue.pop_front();
  }

  shutdown();
  return 0;
}
