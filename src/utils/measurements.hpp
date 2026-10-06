//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <cstdint>
#include <utility>

#include <Eigen/Core>

namespace gopro_ros2 {

// Inertial containers.
using Timestamp = uint64_t;
using ImuStamp = Timestamp;
// First 3 elements correspond to acceleration data [m/s^2]
// while the 3 last correspond to angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
using ImuAccl = Eigen::Matrix<double, 3, 1>;
using ImuGyro = Eigen::Matrix<double, 3, 1>;
using MagneticField = Eigen::Matrix<double, 3, 1>;

struct MagMeasurement {
  MagMeasurement() = default;
  MagMeasurement(const Timestamp& timestamp, const MagneticField& magnetic_field)
      : timestamp(timestamp), magnetic_field(magnetic_field) {}
  MagMeasurement(Timestamp&& timestamp, MagneticField&& magnetic_field)
      : timestamp(std::move(timestamp)), magnetic_field(std::move(magnetic_field)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp;
  MagneticField magnetic_field;
};

struct ImuMeasurement {
  ImuMeasurement() = default;
  ImuMeasurement(const ImuStamp& timestamp, const ImuAccGyr& imu_data)
      : timestamp(timestamp), acc_gyr(imu_data) {}
  ImuMeasurement(ImuStamp&& timestamp, ImuAccGyr&& imu_data)
      : timestamp(std::move(timestamp)), acc_gyr(std::move(imu_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp;
  ImuAccGyr acc_gyr;
};

struct AcclMeasurement {
  AcclMeasurement() = default;
  AcclMeasurement(const ImuStamp& timestamp, const ImuAccl& accl_data)
      : timestamp(timestamp), data(accl_data) {}
  AcclMeasurement(ImuStamp&& timestamp, ImuAccl&& accl_data)
      : timestamp(std::move(timestamp)), data(std::move(accl_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp;
  ImuAccl data;
};

struct GyroMeasurement {
  GyroMeasurement() = default;
  GyroMeasurement(const ImuStamp& timestamp, const ImuGyro& gyro_data)
      : timestamp(timestamp), data(gyro_data) {}
  GyroMeasurement(ImuStamp&& timestamp, ImuGyro&& gyro_data)
      : timestamp(std::move(timestamp)), data(std::move(gyro_data)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuStamp timestamp;
  ImuGyro data;
};

}  // namespace gopro_ros2
