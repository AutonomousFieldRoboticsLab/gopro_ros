//
// Created by bjoshi on 10/29/20.
//

#include <ros/ros.h>

#include <experimental/filesystem>
#include <iostream>

#include "ImuExtractor.h"
#include "VideoExtractor.h"
#include "color_codes.h"

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "gopro_to_asl");
  ros::NodeHandle nh_private("~");

  string gopro_video;
  string asl_dir;
  string gopro_folder;

  bool is_gopro_video = nh_private.getParam("gopro_video", gopro_video);
  bool is_gopro_folder = nh_private.getParam("gopro_folder", gopro_folder);

  if (!is_gopro_video && !is_gopro_folder) {
    ROS_FATAL("Please specify the gopro video or folder");
    ros::shutdown();
  }

  ROS_FATAL_STREAM_COND(!nh_private.getParam("asl_dir", asl_dir), "No asl directory specified");

  double scaling = 1.0;
  if (nh_private.hasParam("scale")) nh_private.getParam("scale", scaling);

  bool grayscale = false;
  if (nh_private.hasParam("grayscale")) nh_private.getParam("grayscale", grayscale);

  bool display_images = false;
  if (nh_private.hasParam("display_images")) nh_private.getParam("display_images", display_images);

  bool multiple_files = false;
  if (nh_private.hasParam("multiple_files")) nh_private.getParam("multiple_files", multiple_files);

  vector<fs::path> video_files;

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

  string image_folder = asl_dir + "/mav0/cam0";
  // print image folder
  cout << "Image folder: " << image_folder << endl;

  if (!experimental::filesystem::is_directory(image_folder)) {
    experimental::filesystem::create_directories(image_folder);
  }

  std::string image_data_folder = image_folder + "/data";
  if (!std::experimental::filesystem::is_directory(image_data_folder)) {
    std::experimental::filesystem::create_directories(image_data_folder);
  }

  std::string image_file = image_folder + "/data.csv";
  std::ofstream image_stream;
  image_stream.open(image_file);
  image_stream << std::fixed << std::setprecision(19);
  image_stream << "#timestamp [ns],filename" << std::endl;
  image_stream.close();

  string imu_folder = asl_dir + "/mav0/imu0";
  if (!experimental::filesystem::is_directory(imu_folder)) {
    experimental::filesystem::create_directories(imu_folder);
  }

  string imu_file = imu_folder + "/data.csv";

  vector<uint64_t> start_stamps;
  vector<uint32_t> samples;
  vector<uint64_t> image_stamps;

  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;

  for (uint32_t i = 0; i < video_files.size(); i++) {
    image_stamps.clear();

    ROS_WARN_STREAM("Opening Video File: " << video_files[i].filename().string());

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling);

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    ROS_INFO_STREAM("[ACCL] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    ROS_INFO_STREAM("[GYRO] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    ROS_INFO_STREAM("[CORI] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));

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
      ROS_FATAL_STREAM("Video and metadata frame count do not match");
      ros::shutdown();
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      ROS_FATAL_STREAM("Video creation time does not match");
      ros::shutdown();
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      ROS_FATAL_STREAM("ffmpeg and gpmf frame count does not match.");
      ROS_FATAL_STREAM(image_stamps.size() << " vs " << ffmpeg_frame_count);
      ros::shutdown();
    }

    video_extractor.extractFrames(image_folder, image_stamps, grayscale, display_images);
  }

  ROS_INFO_STREAM("[ACCL] Payloads: " << accl_queue.size());
  ROS_INFO_STREAM("[GYRO] Payloads: " << gyro_queue.size());

  ofstream imu_stream;
  imu_stream.open(imu_file);
  imu_stream << std::fixed << std::setprecision(19);
  imu_stream << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
             << endl;

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    // ROS_INFO_STREAM("***************Here**********");
    Timestamp stamp;
    int64_t diff = accl.timestamp_ - gyro.timestamp_;
    if (abs(diff) > 100000) {
      // I will need to handle this case more carefully
      ROS_WARN_STREAM(diff << " ns difference between gyro and accl");
      stamp = (Timestamp)(((double)accl.timestamp_ + (double)gyro.timestamp_) / 2.0);
    } else {
      stamp = accl.timestamp_;
    }
    imu_stream << uint64_to_string(stamp);

    imu_stream << "," << gyro.data_.x();
    imu_stream << "," << gyro.data_.y();
    imu_stream << "," << gyro.data_.z();

    imu_stream << "," << accl.data_.x();
    imu_stream << "," << accl.data_.y();
    imu_stream << "," << accl.data_.z() << endl;

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  imu_stream.close();

  return 0;
}