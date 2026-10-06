//
// Created by bjoshi on 10/29/20.
//

#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace gopro_ros2 {

class GoProVideoExtractor {
public:
  /// Receives every decoded frame (BGR8, or MONO8 if grayscale) and its timestamp in ns.
  using FrameCallback = std::function<void(const cv::Mat& image, uint64_t stamp_ns)>;

  GoProVideoExtractor(const std::string& file, double scaling_factor = 1.0, bool dump_info = false);
  ~GoProVideoExtractor();

  void saveToPng(AVFrame* frame,
                 AVCodecContext* codec_context,
                 int width,
                 int height,
                 AVRational time_base,
                 const std::string& filename);

  void saveRaw(AVFrame* frame, int width, int height, const std::string& filename);

  int extractFrames(const std::string& image_folder, uint64_t last_image_stamp_ns);
  int extractFrames(const std::string& image_folder,
                    const std::vector<uint64_t>& image_stamps,
                    bool grayscale = false,
                    bool display_images = false);
  int getFrameStamps(std::vector<uint64_t>& stamps);

  void displayImages();

  /**
   * @brief Decode the video, stamping the i-th frame with image_stamps[i], and pass every frame to
   * the callback.
   *
   * @return 0 on success, -1 if the video could not be opened
   */
  int processFrames(const std::vector<uint64_t>& image_stamps,
                    bool grayscale,
                    bool display_images,
                    const FrameCallback& callback);

  inline uint32_t getFrameCount() { return num_frames_; }
  inline uint64_t getVideoCreationTime() { return video_creation_time_; }

private:
  std::string video_file_;

  // Video properties
  AVFormatContext* format_context_ = nullptr;
  uint32_t video_stream_index_;
  AVCodecContext* codec_context_ = nullptr;
  const AVCodec* codec_ = nullptr;
  AVFrame* frame_ = nullptr;
  AVFrame* frame_rgb_ = nullptr;
  AVPacket packet_;

  AVDictionary* options_dict_ = nullptr;
  AVDictionaryEntry* tag_dict_ = nullptr;
  struct SwsContext* sws_ctx_ = nullptr;
  AVStream* video_stream_ = nullptr;
  AVCodecParameters* codec_parameters_;

  uint64_t video_creation_time_;
  uint32_t image_width_;
  uint32_t image_height_;
  uint32_t num_frames_;
};

}  // namespace gopro_ros2
