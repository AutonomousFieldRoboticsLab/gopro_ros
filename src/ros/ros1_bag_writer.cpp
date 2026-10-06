#include "ros/ros1_bag_writer.hpp"

#include <filesystem>

#include <cv_bridge/cv_bridge.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Header.h>

namespace gopro_ros2 {

namespace {

ros::Time toRosTime(uint64_t stamp_ns) {
  return ros::Time(static_cast<uint32_t>(stamp_ns / 1000000000ULL),
                   static_cast<uint32_t>(stamp_ns % 1000000000ULL));
}

}  // namespace

ROS1BagWriter::ROS1BagWriter(const std::string& bag_path) {
  std::filesystem::path path(bag_path);
  if (path.extension() != ".bag") path += ".bag";
  bag_.open(path.string(), rosbag::bagmode::Write);
}

ROS1BagWriter::~ROS1BagWriter() { bag_.close(); }

void ROS1BagWriter::writeImage(const std::string& topic,
                               const cv::Mat& image,
                               uint64_t stamp_ns,
                               bool compress,
                               const std::string& frame_id) {
  std_msgs::Header header;
  header.stamp = toRosTime(stamp_ns);
  header.frame_id = frame_id;
  const std::string encoding = image.channels() == 1 ? "mono8" : "bgr8";

  if (compress) {
    auto img_msg = cv_bridge::CvImage(header, encoding, image).toCompressedImageMsg();
    bag_.write(topic + "/compressed", header.stamp, *img_msg);
  } else {
    auto img_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    bag_.write(topic, header.stamp, *img_msg);
  }
}

void ROS1BagWriter::writeImu(const std::string& topic,
                             const AcclMeasurement& accl,
                             const GyroMeasurement& gyro,
                             uint64_t stamp_ns,
                             const std::string& frame_id) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = toRosTime(stamp_ns);
  imu_msg.header.frame_id = frame_id;
  imu_msg.linear_acceleration.x = accl.data.x();
  imu_msg.linear_acceleration.y = accl.data.y();
  imu_msg.linear_acceleration.z = accl.data.z();
  imu_msg.angular_velocity.x = gyro.data.x();
  imu_msg.angular_velocity.y = gyro.data.y();
  imu_msg.angular_velocity.z = gyro.data.z();

  bag_.write(topic, imu_msg.header.stamp, imu_msg);
}

void ROS1BagWriter::writeMagneticField(const std::string& topic,
                                       const MagMeasurement& mag,
                                       const std::string& frame_id) {
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = toRosTime(mag.timestamp);
  mag_msg.header.frame_id = frame_id;
  mag_msg.magnetic_field.x = mag.magnetic_field.x();
  mag_msg.magnetic_field.y = mag.magnetic_field.y();
  mag_msg.magnetic_field.z = mag.magnetic_field.z();

  bag_.write(topic, mag_msg.header.stamp, mag_msg);
}

}  // namespace gopro_ros2
