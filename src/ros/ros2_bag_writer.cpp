#include "ros/ros2_bag_writer.hpp"

#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rmw/rmw.h>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/header.hpp>

// cv_bridge ships cv_bridge.h up to Humble and cv_bridge.hpp from Jazzy on.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "utils/print.hpp"
#include "utils/rosbag_utils.hpp"

namespace gopro_ros2 {

namespace {

builtin_interfaces::msg::Time toRosTime(uint64_t stamp_ns) {
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = stamp_ns / 1000000000ULL;
  ros_time.nanosec = stamp_ns % 1000000000ULL;
  return ros_time;
}

}  // namespace

ROS2BagWriter::ROS2BagWriter(const std::string& bag_path,
                             std::string storage_id,
                             std::string mcap_compression) {
  if (storage_id != ".db3" && storage_id != ".mcap") {
    PRINT_WARNING("Invalid storage_id '" << storage_id << "'. Falling back to '.mcap'.");
    storage_id = ".mcap";
  }

  BagConfig cfg = inferBagConfig(bag_path, storage_id);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = cfg.uri;
  storage_options.storage_id = cfg.storage_id;

  if (cfg.storage_id == "mcap") {
    if (mcap_compression != "zstd_fast" && mcap_compression != "zstd_small" &&
        mcap_compression != "none") {
      PRINT_WARNING("Invalid mcap_compression '" << mcap_compression
                                                 << "'. Falling back to 'zstd_fast'.");
      mcap_compression = "zstd_fast";
    }
    storage_options.storage_preset_profile = mcap_compression;
  }

  rosbag2_cpp::ConverterOptions converter_options{rmw_get_serialization_format(),
                                                  rmw_get_serialization_format()};

  bag_.open(storage_options, converter_options);
}

void ROS2BagWriter::writeImage(const std::string& topic,
                               const cv::Mat& image,
                               uint64_t stamp_ns,
                               bool compress,
                               const std::string& frame_id) {
  std_msgs::msg::Header header;
  header.stamp = toRosTime(stamp_ns);
  header.frame_id = frame_id;
  const std::string encoding = image.channels() == 1 ? "mono8" : "bgr8";
  const rclcpp::Time time(header.stamp);

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  if (compress) {
    auto img_msg = cv_bridge::CvImage(header, encoding, image).toCompressedImageMsg();
    compressed_image_serializer_.serialize_message(img_msg.get(), serialized_msg.get());
    bag_.write(serialized_msg, topic + "/compressed", "sensor_msgs/msg/CompressedImage", time);
  } else {
    auto img_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    image_serializer_.serialize_message(img_msg.get(), serialized_msg.get());
    bag_.write(serialized_msg, topic, "sensor_msgs/msg/Image", time);
  }
}

void ROS2BagWriter::writeImu(const std::string& topic,
                             const AcclMeasurement& accl,
                             const GyroMeasurement& gyro,
                             uint64_t stamp_ns,
                             const std::string& frame_id) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = toRosTime(stamp_ns);
  imu_msg.header.frame_id = frame_id;
  imu_msg.linear_acceleration.x = accl.data.x();
  imu_msg.linear_acceleration.y = accl.data.y();
  imu_msg.linear_acceleration.z = accl.data.z();
  imu_msg.angular_velocity.x = gyro.data.x();
  imu_msg.angular_velocity.y = gyro.data.y();
  imu_msg.angular_velocity.z = gyro.data.z();

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  imu_serializer_.serialize_message(&imu_msg, serialized_msg.get());
  bag_.write(serialized_msg, topic, "sensor_msgs/msg/Imu", rclcpp::Time(imu_msg.header.stamp));
}

void ROS2BagWriter::writeMagneticField(const std::string& topic,
                                       const MagMeasurement& mag,
                                       const std::string& frame_id) {
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp = toRosTime(mag.timestamp);
  mag_msg.header.frame_id = frame_id;
  mag_msg.magnetic_field.x = mag.magnetic_field.x();
  mag_msg.magnetic_field.y = mag.magnetic_field.y();
  mag_msg.magnetic_field.z = mag.magnetic_field.z();

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  mag_serializer_.serialize_message(&mag_msg, serialized_msg.get());
  bag_.write(
      serialized_msg, topic, "sensor_msgs/msg/MagneticField", rclcpp::Time(mag_msg.header.stamp));
}

}  // namespace gopro_ros2
