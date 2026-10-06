#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <rosbag/bag.h>

#include "utils/measurements.hpp"

namespace gopro_ros2 {

/**
 * @brief Writes GoPro images, IMU and magnetometer measurements into a ROS 1 bag.
 *
 * Has the same interface as ROS2BagWriter so the executables only differ in which one they use.
 */
class ROS1BagWriter {
public:
  /// @param bag_path output bag file; ".bag" is appended if missing
  explicit ROS1BagWriter(const std::string& bag_path);
  ~ROS1BagWriter();

  /// Writes a BGR8 or MONO8 image, optionally JPEG-compressed to `<topic>/compressed`.
  void writeImage(const std::string& topic,
                  const cv::Mat& image,
                  uint64_t stamp_ns,
                  bool compress,
                  const std::string& frame_id = "gopro");

  void writeImu(const std::string& topic,
                const AcclMeasurement& accl,
                const GyroMeasurement& gyro,
                uint64_t stamp_ns,
                const std::string& frame_id = "body");

  void writeMagneticField(const std::string& topic,
                          const MagMeasurement& mag,
                          const std::string& frame_id = "body");

private:
  rosbag::Bag bag_;
};

}  // namespace gopro_ros2
