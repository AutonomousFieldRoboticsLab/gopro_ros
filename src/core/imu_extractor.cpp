/*! @file imu_extractor.cpp
 *
 *  @brief Extract IMU, magnetometer and image timestamps from the GPMF track of a GoPro MP4.
 *
 *  Derived from GPMF_demo.c (version 2.0.0) of gpmf-parser.
 *
 *  (C) Copyright 2017-2020 GoPro Inc (http://gopro.com/).
 *
 *  Licensed under either:
 *  - Apache License, Version 2.0, http://www.apache.org/licenses/LICENSE-2.0
 *  - MIT license, http://opensource.org/licenses/MIT
 *  at your option.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include "core/imu_extractor.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "utils/color_codes.hpp"
#include "utils/print.hpp"
#include "utils/time_utils.hpp"

extern void PrintGPMF(GPMF_stream* ms);

namespace gopro_ros {

namespace {

constexpr auto kRecurseTolerant = static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT);
constexpr auto kCurrentTolerant = static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT);

}  // namespace

GoProImuExtractor::GoProImuExtractor(const std::string& file) {
  char* video = const_cast<char*>(file.c_str());

  ms_ = &metadata_stream_;
  mp4_ = OpenMP4Source(video, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
  if (mp4_ == 0) {
    PRINT_ERROR("Could not open video file");
  }
  metadata_length_ = GetDuration(mp4_);

  if (metadata_length_ > 0.0) {
    payloads_ = GetNumberPayloads(mp4_);
    // payloads_ = payloads_ - 1;  // Discarding the last payload. Found that payload is not
    //                             // reliable as others
  }
  uint32_t fr_num, fr_dem;
  frame_count_ = GetVideoFrameRateAndCount(mp4_, &fr_num, &fr_dem);
  frame_rate_ = static_cast<float>(fr_num) / static_cast<float>(fr_dem);

  movie_creation_time_ =
      static_cast<uint64_t>((getCreationtime(mp4_) - getOffset1904()) * 1000000000);
}

bool GoProImuExtractor::displayVideoFramerate() {
  if (frame_count_) {
    printf("VIDEO FRAMERATE:\n  %.3f with %d frames\n", frame_rate_, frame_count_);
    return true;
  } else {
    return false;
  }
}

GoProImuExtractor::~GoProImuExtractor() {
  if (payload_res_) FreePayloadResource(mp4_, payload_res_);
  if (ms_) GPMF_Free(ms_);

  payload_ = nullptr;
  CloseSource(mp4_);
}

void GoProImuExtractor::cleanup() {
  if (payload_res_) FreePayloadResource(mp4_, payload_res_);
  if (ms_) GPMF_Free(ms_);

  payload_ = nullptr;
  CloseSource(mp4_);
}

void GoProImuExtractor::showGpmfStructure() {
  uint32_t payload_size;
  GPMF_ERR ret = GPMF_OK;

  // Just print the structure of first payload
  // Remaining structure should also be similar
  uint32_t index = 0;
  double in = 0.0, out = 0.0;  // times

  payload_size = GetPayloadSize(mp4_, index);
  payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
  payload_ = GetPayload(mp4_, payload_res_, index);

  if (payload_ == nullptr) cleanup();

  ret = GetPayloadTime(mp4_, index, &in, &out);
  if (ret != GPMF_OK) cleanup();

  ret = GPMF_Init(ms_, payload_, payload_size);
  if (ret != GPMF_OK) cleanup();

  printf("PAYLOAD TIME:\n  %.3f to %.3f seconds\n", in, out);
  printf("GPMF STRUCTURE:\n");
  // Output (printf) all the contained GPMF data within this payload
  ret = GPMF_Validate(ms_, GPMF_RECURSE_LEVELS);  // optional
  if (GPMF_OK != ret) {
    if (GPMF_ERROR_UNKNOWN_TYPE == ret) {
      printf("Unknown GPMF Type within, ignoring\n");
      ret = GPMF_OK;
    } else {
      printf("Invalid GPMF Structure\n");
    }
  }

  GPMF_ResetState(ms_);

  GPMF_ERR nextret;
  do {
    printf("  ");
    PrintGPMF(ms_);  // printf current GPMF KLV

    nextret = GPMF_Next(ms_, GPMF_RECURSE_LEVELS);

    // Or just use GPMF_Next(ms_, GPMF_RECURSE_LEVELS | GPMF_TOLERANT) to ignore and skip unknown
    // types
    while (nextret == GPMF_ERROR_UNKNOWN_TYPE) nextret = GPMF_Next(ms_, GPMF_RECURSE_LEVELS);
  } while (GPMF_OK == nextret);
  GPMF_ResetState(ms_);
}

/** Only supports native formats like ACCL, GYRO, GPS
 *
 * @param fourcc
 * @param readings
 * @return
 */
GPMF_ERR GoProImuExtractor::getScaledData(uint32_t fourcc,
                                          std::vector<std::vector<double>>& readings) {
  while (GPMF_OK == GPMF_FindNext(ms_, STR2FOURCC("STRM"), kRecurseTolerant)) {
    if (GPMF_OK != GPMF_FindNext(ms_, fourcc, kRecurseTolerant)) continue;

    uint32_t samples = GPMF_Repeat(ms_);
    uint32_t elements = GPMF_ElementsInStruct(ms_);
    uint32_t buffer_size = samples * elements * sizeof(double);
    double* ptr;
    double* tmp_buffer = static_cast<double*>(malloc(buffer_size));

    readings.resize(samples);

    if (tmp_buffer && samples) {
      // Use GPMF_FormattedData(ms_, tmp_buffer, buffer_size, 0, samples) to output data in
      // little-endian, but without scaling.
      if (GPMF_OK == GPMF_ScaledData(ms_, tmp_buffer, buffer_size, 0, samples, GPMF_TYPE_DOUBLE)) {
        ptr = tmp_buffer;
        for (uint32_t i = 0; i < samples; i++) {
          std::vector<double> sample(elements);
          for (uint32_t j = 0; j < elements; j++) {
            sample.at(j) = *ptr++;
          }
          readings.at(i) = sample;
        }
      }
      free(tmp_buffer);
    }
  }

  GPMF_ResetState(ms_);
  return GPMF_OK;
}

/** Returns STMP (start time of current payload)
 *
 * @param fourcc
 * @return timestamp
 */
uint64_t GoProImuExtractor::getStamp(uint32_t fourcc) {
  GPMF_stream find_stream;

  uint64_t timestamp;
  while (GPMF_OK == GPMF_FindNext(ms_, STR2FOURCC("STRM"), kRecurseTolerant)) {
    if (GPMF_OK != GPMF_FindNext(ms_, fourcc, kRecurseTolerant)) continue;

    GPMF_CopyState(ms_, &find_stream);
    if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_STAMP, kCurrentTolerant))
      timestamp = BYTESWAP64(*static_cast<uint64_t*>(GPMF_RawData(&find_stream)));
  }
  GPMF_ResetState(ms_);
  return timestamp;
}

GPMF_ERR GoProImuExtractor::showCurrentPayload(uint32_t index) {
  uint32_t payload_size;
  GPMF_ERR ret;

  payload_size = GetPayloadSize(mp4_, index);
  payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
  payload_ = GetPayload(mp4_, payload_res_, index);

  if (payload_ == nullptr) cleanup();
  ret = GPMF_Init(ms_, payload_, payload_size);
  if (ret != GPMF_OK) cleanup();

  GPMF_ERR nextret;
  do {
    printf("  ");
    PrintGPMF(ms_);  // printf current GPMF KLV

    nextret = GPMF_Next(ms_, GPMF_RECURSE_LEVELS);

    // Or just use GPMF_Next(ms_, GPMF_RECURSE_LEVELS | GPMF_TOLERANT) to ignore and skip unknown
    // types
    while (nextret == GPMF_ERROR_UNKNOWN_TYPE) nextret = GPMF_Next(ms_, GPMF_RECURSE_LEVELS);
  } while (GPMF_OK == nextret);
  GPMF_ResetState(ms_);
}

void GoProImuExtractor::getImageStamps(std::vector<uint64_t>& image_stamps,
                                       uint64_t image_end_stamp,
                                       bool offset_only) {
  std::vector<std::vector<double>> cam_orient_data;

  uint64_t current_stamp, prev_stamp;

  uint64_t total_samples = 0;

  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) cleanup();
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) cleanup();

    current_stamp = getStamp(STR2FOURCC("CORI"));

    current_stamp = current_stamp * 1000;  // us to ns

    if (index > 0) {
      uint64_t time_span = current_stamp - prev_stamp;

      if (time_span < 0) {
        PRINT_ERROR("previous timestamp should be smaller than current stamp");
        exit(1);
      }

      double step_size = static_cast<double>(time_span) / cam_orient_data.size();

      for (uint32_t i = 0; i < cam_orient_data.size(); i++) {
        uint64_t stamp = prev_stamp + static_cast<uint64_t>(i * step_size);
        if (!offset_only) stamp = movie_creation_time_ + stamp;
        image_stamps.push_back(stamp);
      }
      total_samples += cam_orient_data.size();
    }

    cam_orient_data.clear();
    getScaledData(STR2FOURCC("CORI"), cam_orient_data);

    prev_stamp = current_stamp;

    GPMF_Free(ms_);
  }

  if (image_end_stamp > 0) {
    uint64_t time_span = image_end_stamp * 1000 - current_stamp;
    double step_size = static_cast<double>(time_span) / cam_orient_data.size();

    for (uint32_t i = 0; i < cam_orient_data.size(); i++) {
      uint64_t stamp = prev_stamp + static_cast<uint64_t>(i * step_size);
      if (!offset_only) stamp = movie_creation_time_ + stamp;
      image_stamps.push_back(stamp);
    }
  }
}

int GoProImuExtractor::saveImuStream(const std::string& imu_file, uint64_t end_time) {
  std::ofstream imu_stream;
  imu_stream.open(imu_file);
  imu_stream << std::fixed << std::setprecision(19);
  imu_stream << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
             << std::endl;

  uint64_t first_frame_us, first_frame_ns;
  std::vector<std::vector<double>> accl_data;
  std::vector<std::vector<double>> gyro_data;

  uint64_t current_stamp, prev_stamp;

  std::vector<uint64_t> steps;
  uint64_t total_samples = 0;

  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) cleanup();
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) cleanup();

    if (index == 0) {
      first_frame_us = getStamp(STR2FOURCC("CORI"));
      first_frame_ns = first_frame_us * 1000;
    }

    current_stamp = getStamp(STR2FOURCC("ACCL"));
    uint64_t current_gyro_stamp = getStamp(STR2FOURCC("GYRO"));

    if (current_gyro_stamp != current_stamp) {
      int32_t diff = current_gyro_stamp - current_stamp;
      if (std::abs(diff) > 20) {
        PRINT_ERROR("[ERROR] ACCL and GYRO timestamp heavily un-synchronized ....Shutting Down!!!");
        PRINT_ERROR("Index: " << index << " accl stamp: " << current_stamp
                              << " gyro stamp: " << current_gyro_stamp);
        exit(1);
      } else {
        PRINT_WARNING("[WARN] ACCL and GYRO timestamp slightly not synchronized ....");
        PRINT_WARNING("Index: " << index << " accl stamp: " << current_stamp
                                << " gyro stamp: " << current_gyro_stamp);
      }
    }

    current_stamp = current_stamp * 1000;  // us to ns
    if (index > 0) {
      uint64_t time_span = current_stamp - prev_stamp;
      if (time_span < 0) {
        PRINT_ERROR("previous timestamp should be smaller than current stamp");
        exit(1);
      }

      uint64_t step_size = time_span / accl_data.size();
      steps.emplace_back(step_size);

      if (accl_data.size() != gyro_data.size()) {
        PRINT_ERROR("ACCL and GYRO data must be of same size");
        exit(1);
      }

      for (int i = 0; i < gyro_data.size(); ++i) {
        uint64_t s = prev_stamp + i * step_size;
        uint64_t ros_stamp = movie_creation_time_ + s - first_frame_ns;
        imu_stream << uint64ToString(ros_stamp);

        std::vector<double> gyro_sample = gyro_data.at(i);

        // Data comes in ZXY order
        imu_stream << "," << gyro_sample.at(1);
        imu_stream << "," << gyro_sample.at(2);
        imu_stream << "," << gyro_sample.at(0);

        std::vector<double> accl_sample = accl_data.at(i);
        imu_stream << "," << accl_sample.at(1);
        imu_stream << "," << accl_sample.at(2);
        imu_stream << "," << accl_sample.at(0) << std::endl;

        // Letting one extra imu stamp
        if ((ros_stamp - movie_creation_time_) > end_time) break;
      }
    }

    gyro_data.clear();
    accl_data.clear();
    getScaledData(STR2FOURCC("ACCL"), accl_data);
    getScaledData(STR2FOURCC("GYRO"), gyro_data);

    total_samples += gyro_data.size();
    prev_stamp = current_stamp;

    GPMF_Free(ms_);
  }

  PRINT_INFO(GREEN << "Wrote " << total_samples << " imu samples to file" << RESET);
  imu_stream.close();
}

uint32_t GoProImuExtractor::getNumOfSamples(uint32_t fourcc) {
  GPMF_stream find_stream;
  uint32_t total_samples = 0;

  // Iterate through all payloads to find if this fourcc exists
  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) continue;
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) continue;

    while (GPMF_OK == GPMF_FindNext(ms_, STR2FOURCC("STRM"), kRecurseTolerant)) {
      if (GPMF_OK != GPMF_FindNext(ms_, fourcc, kRecurseTolerant)) continue;

      GPMF_CopyState(ms_, &find_stream);
      if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TOTAL_SAMPLES, kCurrentTolerant))
        total_samples = BYTESWAP32(*static_cast<uint32_t*>(GPMF_RawData(&find_stream)));

      // Found it, return the total samples from first payload
      GPMF_ResetState(ms_);
      return total_samples;
    }
    GPMF_ResetState(ms_);
  }

  return total_samples;  // Return 0 if not found
}

void GoProImuExtractor::getPayloadStamps(uint32_t fourcc,
                                         std::vector<uint64_t>& start_stamps,
                                         std::vector<uint32_t>& samples) {
  start_stamps.clear();
  samples.clear();

  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) cleanup();
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) cleanup();

    uint64_t stamp = getStamp(fourcc);
    uint32_t total_samples = getNumOfSamples(fourcc);

    start_stamps.push_back(stamp);
    samples.push_back(total_samples);
  }
}

void GoProImuExtractor::skipPayloads(uint32_t num_payloads) { payloads_skipped_ = num_payloads; }

uint64_t GoProImuExtractor::getPayloadStartStamp(uint32_t fourcc, uint32_t index) {
  GPMF_ERR ret;
  uint32_t payload_size;

  payload_size = GetPayloadSize(mp4_, index);
  payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
  payload_ = GetPayload(mp4_, payload_res_, index);

  if (payload_ == nullptr) cleanup();
  ret = GPMF_Init(ms_, payload_, payload_size);
  if (ret != GPMF_OK) cleanup();

  uint64_t stamp = getStamp(fourcc);
  return stamp;
}

void GoProImuExtractor::readImuData(std::deque<AcclMeasurement>& accl_queue,
                                    std::deque<GyroMeasurement>& gyro_queue,
                                    uint64_t accl_end_time,
                                    uint64_t gyro_end_time) {
  std::vector<std::vector<double>> accl_data;
  std::vector<std::vector<double>> gyro_data;

  uint64_t current_accl_stamp, prev_accl_stamp;
  uint64_t current_gyro_stamp, prev_gyro_stamp;

  uint64_t total_samples = 0;

  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) cleanup();
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) cleanup();

    current_accl_stamp = getStamp(STR2FOURCC("ACCL"));
    current_gyro_stamp = getStamp(STR2FOURCC("GYRO"));

    if (current_gyro_stamp != current_accl_stamp) {
      int32_t diff = current_gyro_stamp - current_accl_stamp;
      if (std::abs(diff) > 100) {
        PRINT_WARNING("ACCL and GYRO timestamp slightly un-synchronized ....");
        PRINT_WARNING("Index: " << index << " accl stamp: " << current_accl_stamp
                                << " gyro stamp: " << current_gyro_stamp << " diff: " << diff);
      }
    }

    current_accl_stamp = current_accl_stamp * 1000;  // us to ns
    current_gyro_stamp = current_gyro_stamp * 1000;  // us to ns

    if (index > 0) {
      uint64_t accl_time_span = current_accl_stamp - prev_accl_stamp;
      uint64_t gyro_time_span = current_gyro_stamp - prev_gyro_stamp;

      if (accl_time_span < 0 || gyro_time_span < 0) {
        PRINT_ERROR("previous timestamp should be smaller than current stamp");
        exit(1);
      }

      double accl_step_size = static_cast<double>(accl_time_span) / accl_data.size();
      double gyro_step_size = static_cast<double>(gyro_time_span) / gyro_data.size();

      if (accl_data.size() != gyro_data.size()) {
        PRINT_ERROR("ACCL and GYRO data are not of same size");
      }

      for (uint32_t i = 0; i < accl_data.size(); i++) {
        uint64_t accl_time = prev_accl_stamp + static_cast<uint64_t>(i * accl_step_size);
        uint64_t accl_stamp = movie_creation_time_ + accl_time;
        std::vector<double> accl_sample = accl_data.at(i);

        // Data comes in ZXY order
        ImuAccl accl;
        accl << accl_sample.at(1), accl_sample.at(2), accl_sample.at(0);
        accl_queue.push_back(AcclMeasurement(accl_stamp, accl));
      }

      for (uint32_t i = 0; i < gyro_data.size(); ++i) {
        uint64_t gyro_time = prev_gyro_stamp + static_cast<uint64_t>(i * gyro_step_size);
        uint64_t gyro_stamp = movie_creation_time_ + gyro_time;
        std::vector<double> gyro_sample = gyro_data.at(i);

        // Data comes in ZXY order
        ImuGyro gyro;
        gyro << gyro_sample.at(1), gyro_sample.at(2), gyro_sample.at(0);
        gyro_queue.push_back(GyroMeasurement(gyro_stamp, gyro));

        total_samples += 1;
      }
    }

    gyro_data.clear();
    accl_data.clear();
    getScaledData(STR2FOURCC("ACCL"), accl_data);
    getScaledData(STR2FOURCC("GYRO"), gyro_data);

    prev_accl_stamp = current_accl_stamp;
    prev_gyro_stamp = current_gyro_stamp;

    GPMF_Free(ms_);
  }

  // If this is not the last video file, extract the last payload.
  // For the last video file, the last payload is not used since we do not have the end time of
  // the last payload.
  if (accl_end_time > 0 && gyro_end_time > 0) {
    uint64_t accl_time_span = accl_end_time * 1000 - current_accl_stamp;
    double accl_step_size = static_cast<double>(accl_time_span) / accl_data.size();

    uint64_t gyro_time_span = gyro_end_time * 1000 - current_gyro_stamp;
    double gyro_step_size = static_cast<double>(gyro_time_span) / gyro_data.size();

    if (accl_data.size() != gyro_data.size()) {
      PRINT_WARNING("ACCL and GYRO data must be of same size; ACCL data size: "
                    << accl_data.size() << " GYRO data size: " << gyro_data.size());
    }

    for (uint32_t i = 0; i < accl_data.size(); i++) {
      uint64_t accl_time = prev_accl_stamp + static_cast<uint64_t>(i * accl_step_size);
      uint64_t accl_stamp = movie_creation_time_ + accl_time;
      std::vector<double> accl_sample = accl_data.at(i);

      // Data comes in ZXY order
      ImuAccl accl;
      accl << accl_sample.at(1), accl_sample.at(2), accl_sample.at(0);
      accl_queue.push_back(AcclMeasurement(accl_stamp, accl));
    }

    for (uint32_t i = 0; i < gyro_data.size(); ++i) {
      uint64_t gyro_time = prev_gyro_stamp + static_cast<uint64_t>(i * gyro_step_size);
      uint64_t gyro_stamp = movie_creation_time_ + gyro_time;
      std::vector<double> gyro_sample = gyro_data.at(i);

      // Data comes in ZXY order
      ImuGyro gyro;
      gyro << gyro_sample.at(1), gyro_sample.at(2), gyro_sample.at(0);
      gyro_queue.push_back(GyroMeasurement(gyro_stamp, gyro));

      total_samples += 1;
    }
  }
}

void GoProImuExtractor::readMagnetometerData(std::deque<MagMeasurement>& mag_queue,
                                             uint64_t mag_end_time) {
  std::vector<std::vector<double>> mag_data;
  uint64_t current_mag_stamp, prev_mag_stamp;

  // This extracts measurements from payload 0 to payloads_ - 1
  for (uint32_t index = 0; index < payloads_; index++) {
    GPMF_ERR ret;
    uint32_t payload_size;

    payload_size = GetPayloadSize(mp4_, index);
    payload_res_ = GetPayloadResource(mp4_, payload_res_, payload_size);
    payload_ = GetPayload(mp4_, payload_res_, index);

    if (payload_ == nullptr) cleanup();
    ret = GPMF_Init(ms_, payload_, payload_size);
    if (ret != GPMF_OK) cleanup();

    current_mag_stamp = getStamp(STR2FOURCC("MAGN"));
    current_mag_stamp = current_mag_stamp * 1000;  // us to ns

    if (index > 0) {
      uint64_t mag_time_span = current_mag_stamp - prev_mag_stamp;

      if (mag_time_span < 0) {
        PRINT_ERROR("previous magnetometer timestamp should be smaller than current stamp");
        exit(1);
      }

      double mag_step_size = static_cast<double>(mag_time_span) / mag_data.size();

      for (uint32_t i = 0; i < mag_data.size(); i++) {
        Timestamp mag_time = prev_mag_stamp + static_cast<uint64_t>(i * mag_step_size);
        Timestamp mag_stamp = movie_creation_time_ + mag_time;
        std::vector<double> mag_sample = mag_data.at(i);

        // The data comes in ZXY order
        MagneticField mag;
        mag << mag_sample.at(1), mag_sample.at(2), mag_sample.at(0);
        mag_queue.push_back(MagMeasurement(mag_stamp, mag));
      }
    }

    mag_data.clear();
    getScaledData(STR2FOURCC("MAGN"), mag_data);

    prev_mag_stamp = current_mag_stamp;
    GPMF_Free(ms_);
  }

  // If this is not the last video file, extract the last payload
  if (mag_end_time > 0) {
    uint64_t mag_time_span = mag_end_time * 1000 - current_mag_stamp;
    double mag_step_size = static_cast<double>(mag_time_span) / mag_data.size();

    for (uint32_t i = 0; i < mag_data.size(); i++) {
      Timestamp mag_time = prev_mag_stamp + static_cast<uint64_t>(i * mag_step_size);
      Timestamp mag_stamp = movie_creation_time_ + mag_time;
      std::vector<double> mag_sample = mag_data.at(i);

      // Data comes in XYZ order, aligned with camera
      MagneticField magnetic_field;
      magnetic_field << mag_sample.at(1), mag_sample.at(2), mag_sample.at(0);
      mag_queue.push_back(MagMeasurement(mag_stamp, magnetic_field));
    }
  }
}

}  // namespace gopro_ros
