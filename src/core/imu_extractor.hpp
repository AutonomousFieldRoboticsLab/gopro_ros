//
// Created by bjoshi on 10/29/20.
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "gpmf/GPMF_mp4reader.h"
#include "gpmf/GPMF_parser.h"

#include "utils/measurements.hpp"

namespace gopro_ros {

class GoProImuExtractor {
public:
  explicit GoProImuExtractor(const std::string& file);
  ~GoProImuExtractor();

  bool displayVideoFramerate();
  void cleanup();
  void showGpmfStructure();
  GPMF_ERR getScaledData(uint32_t fourcc, std::vector<std::vector<double>>& readings);
  int saveImuStream(const std::string& imu_file, uint64_t end_time);
  uint64_t getStamp(uint32_t fourcc);
  uint32_t getNumOfSamples(uint32_t fourcc);
  GPMF_ERR showCurrentPayload(uint32_t index);

  void getPayloadStamps(uint32_t fourcc,
                        std::vector<uint64_t>& start_stamps,
                        std::vector<uint32_t>& samples);
  void skipPayloads(uint32_t num_payloads);
  void getImageStamps(std::vector<uint64_t>& image_stamps,
                      uint64_t image_end_stamp = 0,
                      bool offset_only = false);

  void readImuData(std::deque<AcclMeasurement>& accl_queue,
                   std::deque<GyroMeasurement>& gyro_queue,
                   uint64_t accl_end_time = 0,
                   uint64_t gyro_end_time = 0);
  uint64_t getPayloadStartStamp(uint32_t fourcc, uint32_t index);
  void readMagnetometerData(std::deque<MagMeasurement>& mag_queue, uint64_t mag_end_time = 0);

  inline uint32_t getImageCount() { return frame_count_; }
  inline uint64_t getVideoCreationTime() { return movie_creation_time_; }

private:
  GPMF_stream metadata_stream_;
  GPMF_stream* ms_;
  double metadata_length_;
  size_t mp4_;
  uint32_t payloads_ = 0;
  uint32_t* payload_ = nullptr;
  size_t payload_res_ = 0;
  uint32_t payloads_skipped_ = 0;

  // Video metadata
  uint32_t frame_count_;
  float frame_rate_;
  uint64_t movie_creation_time_;
};

}  // namespace gopro_ros
