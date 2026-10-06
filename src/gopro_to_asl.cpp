//
// Created by bjoshi on 10/29/20.
//

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "core/imu_extractor.hpp"
#include "core/video_extractor.hpp"
#include "utils/measurements.hpp"
#include "utils/print.hpp"
#include "utils/time_utils.hpp"

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#elif ROS_AVAILABLE == 2
#include <rclcpp/rclcpp.hpp>
#endif

namespace fs = std::filesystem;

using gopro_ros::AcclMeasurement;
using gopro_ros::GoProImuExtractor;
using gopro_ros::GoProVideoExtractor;
using gopro_ros::GyroMeasurement;
using gopro_ros::Timestamp;

namespace {

void shutdown() {
#if ROS_AVAILABLE == 1
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif
}

bool rosOk() {
#if ROS_AVAILABLE == 1
  return ros::ok();
#elif ROS_AVAILABLE == 2
  return rclcpp::ok();
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
  std::string asl_dir;
  double scaling;
  bool grayscale;
  bool display_images;
  bool multiple_files;
  bool hardware_decoding;

#if ROS_AVAILABLE == 1
  ros::init(argc, argv, "gopro_to_asl");
  ros::NodeHandle nh("~");
  nh.param<std::string>("gopro_video", gopro_video, "");
  nh.param<std::string>("gopro_folder", gopro_folder, "");
  nh.param<std::string>("asl_dir", asl_dir, "");
  nh.param<double>("scale", scaling, 1.0);
  nh.param<bool>("grayscale", grayscale, false);
  nh.param<bool>("display_images", display_images, false);
  nh.param<bool>("multiple_files", multiple_files, false);
  nh.param<bool>("hardware_decoding", hardware_decoding, true);
#elif ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gopro_to_asl");
  gopro_video = node->declare_parameter<std::string>("gopro_video", "");
  gopro_folder = node->declare_parameter<std::string>("gopro_folder", "");
  asl_dir = node->declare_parameter<std::string>("asl_dir", "");
  scaling = node->declare_parameter<double>("scale", 1.0);
  grayscale = node->declare_parameter<bool>("grayscale", false);
  display_images = node->declare_parameter<bool>("display_images", false);
  multiple_files = node->declare_parameter<bool>("multiple_files", false);
  hardware_decoding = node->declare_parameter<bool>("hardware_decoding", true);
#endif

  bool is_gopro_video = !gopro_video.empty();
  bool is_gopro_folder = !gopro_folder.empty();

  if (!is_gopro_video && !is_gopro_folder) {
    PRINT_ERROR("Please specify the gopro video or folder");
    shutdown();
    return 1;
  }

  if (asl_dir.empty()) {
    PRINT_ERROR("No asl directory specified");
    shutdown();
    return 1;
  }

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

  std::string image_folder = asl_dir + "/mav0/cam0";
  PRINT_INFO("Image folder: " << image_folder);

  if (!fs::is_directory(image_folder)) {
    fs::create_directories(image_folder);
  }

  std::string image_data_folder = image_folder + "/data";
  if (!fs::is_directory(image_data_folder)) {
    fs::create_directories(image_data_folder);
  }

  std::string image_file = image_folder + "/data.csv";
  std::ofstream image_stream;
  image_stream.open(image_file);
  image_stream << std::fixed << std::setprecision(19);
  image_stream << "#timestamp [ns],filename" << std::endl;
  image_stream.close();

  std::string imu_folder = asl_dir + "/mav0/imu0";
  if (!fs::is_directory(imu_folder)) {
    fs::create_directories(imu_folder);
  }

  std::string imu_file = imu_folder + "/data.csv";

  std::vector<uint64_t> start_stamps;
  std::vector<uint32_t> samples;
  std::vector<uint64_t> image_stamps;

  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;

  for (uint32_t i = 0; i < video_files.size(); i++) {
    if (!rosOk()) break;  // Ctrl+C: skip the remaining chapters
    image_stamps.clear();

    PRINT_WARNING("Opening Video File: " << video_files[i].filename().string());

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling, false, hardware_decoding);

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    printPayloadStamps("ACCL", start_stamps, samples);
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    printPayloadStamps("GYRO", start_stamps, samples);
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    printPayloadStamps("CORI", start_stamps, samples);

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    uint64_t video_end_stamp = 0;
    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
      video_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("CORI"), 0);
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count) {
      PRINT_ERROR("Video and metadata frame count do not match");
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      PRINT_ERROR("Video creation time does not match");
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      PRINT_ERROR("ffmpeg and gpmf frame count does not match. " << image_stamps.size() << " vs "
                                                                 << ffmpeg_frame_count);
    }

    video_extractor.extractFrames(image_folder, image_stamps, grayscale, display_images, rosOk);
  }

  // Ctrl+C: stop without writing the remaining data
  if (!rosOk()) {
    PRINT_WARNING("Interrupted");
    return 0;
  }

  PRINT_INFO("[ACCL] Payloads: " << accl_queue.size());
  PRINT_INFO("[GYRO] Payloads: " << gyro_queue.size());

  std::ofstream imu_stream;
  imu_stream.open(imu_file);
  imu_stream << std::fixed << std::setprecision(19);
  imu_stream << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
             << std::endl;

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    Timestamp stamp;
    int64_t diff = accl.timestamp - gyro.timestamp;
    if (std::abs(diff) > 100000) {
      // I will need to handle this case more carefully
      PRINT_WARNING(diff << " ns difference between gyro and accl");
      stamp = static_cast<Timestamp>(
          (static_cast<double>(accl.timestamp) + static_cast<double>(gyro.timestamp)) / 2.0);
    } else {
      stamp = accl.timestamp;
    }
    imu_stream << gopro_ros::uint64ToString(stamp);

    imu_stream << "," << gyro.data.x();
    imu_stream << "," << gyro.data.y();
    imu_stream << "," << gyro.data.z();

    imu_stream << "," << accl.data.x();
    imu_stream << "," << accl.data.y();
    imu_stream << "," << accl.data.z() << std::endl;

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  imu_stream.close();

  shutdown();
  return 0;
}
