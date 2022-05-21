//
// Created by bjoshi on 10/29/20.
//

#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "GPMF_mp4reader.h"
#include "GPMF_parser.h"
#include "Utils.h"
#include "color_codes.h"

class GoProImuExtractor {
private:
  char* video;
  GPMF_stream metadata_stream;
  GPMF_stream* ms;
  double metadatalength;
  size_t mp4;
  uint32_t payloads;
  uint32_t* payload = NULL;
  size_t payloadres = 0;

  // Video Metadata
  uint32_t frame_count;
  float frame_rate;
  uint64_t movie_creation_time;

public:
  uint32_t payloads_skipped = 0;

public:
  GoProImuExtractor(const std::string file);
  ~GoProImuExtractor();

  bool display_video_framerate();
  void cleanup();
  void show_gpmf_structure();
  GPMF_ERR get_scaled_data(uint32_t fourcc, std::vector<std::vector<double>>& readings);
  int save_imu_stream(const std::string file, uint64_t end_time);
  uint64_t get_stamp(uint32_t fourcc);
  uint32_t getNumofSamples(uint32_t fourcc);
  GPMF_ERR show_current_payload(uint32_t index);

  void getPayloadStamps(uint32_t fourcc,
                        std::vector<uint64_t>& start_stamps,
                        std::vector<uint32_t>& samples);
  void skipPayloads(uint32_t last_n_payloads);
  void getImageStamps(std::vector<uint64_t>& image_stamps,
                      uint64_t image_end_stamp = 0,
                      bool offset_only = false);

  void writeImuData(rosbag::Bag& bag, uint64_t end_time, const std::string& imu_topic);
  void readImuData(std::deque<AcclMeasurement>& accl_data,
                   std::deque<GyroMeasurement>& gyro_data,
                   uint64_t accl_end_time = 0,
                   uint64_t gyro_end_time = 0);
  uint64_t getPayloadStartStamp(uint32_t fourcc, uint32_t index);
  void readImuData(std::vector<uint64_t>& image_stamps,
                   std::deque<GyroMeasurement>& gyro_data,
                   uint64_t accl_end_time = 0,
                   uint64_t gyro_end_time = 0);

  // GPMF_ERR getRawData(uint32_t fourcc, vector<vector<float>> &readings);

  inline uint32_t getImageCount() { return frame_count; }
  inline uint64_t getVideoCreationTime() { return movie_creation_time; }
};
