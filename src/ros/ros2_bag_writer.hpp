#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "utils/measurements.hpp"

namespace gopro_ros2 {

/**
 * @brief Writes GoPro images, IMU and magnetometer measurements into a ROS 2 bag.
 *
 * Has the same interface as ROS1BagWriter so the executables only differ in which one they use.
 */
class ROS2BagWriter {
public:
  /**
   * @param bag_path output bag directory
   * @param storage_id storage selector (".mcap" or ".db3"); anything else falls back to ".mcap"
   * @param mcap_compression MCAP preset ("zstd_fast", "zstd_small" or "none")
   */
  ROS2BagWriter(const std::string& bag_path,
                std::string storage_id = ".mcap",
                std::string mcap_compression = "zstd_fast");

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
  rosbag2_cpp::Writer bag_;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serializer_;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer_;
  rclcpp::Serialization<sensor_msgs::msg::MagneticField> mag_serializer_;
};

}  // namespace gopro_ros2
