//
// Created by bjoshi on 10/29/20.
//

#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace gopro_ros {

/// How processFrames() encodes every frame (in parallel worker threads).
enum class ImageEncoding { kNone, kJpeg, kPng };

class GoProVideoExtractor {
public:
  /// A converted video frame, delivered in video order.
  struct Frame {
    cv::Mat image;                 ///< BGR8, or MONO8 if grayscale
    std::vector<uint8_t> encoded;  ///< JPEG/PNG bytes, empty for ImageEncoding::kNone
    uint64_t stamp_ns = 0;
  };
  using FrameCallback = std::function<void(const Frame& frame)>;
  /// Polled once per frame; returning false stops the conversion early (e.g. on Ctrl+C).
  using KeepRunning = std::function<bool()>;

  /**
   * @param hardware_decoding try NVIDIA (CUDA/NVDEC) and then VAAPI hardware decoding, falling back
   * to CPU decoding if neither is available
   */
  GoProVideoExtractor(const std::string& file,
                      double scaling_factor = 1.0,
                      bool dump_info = false,
                      bool hardware_decoding = true);
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
                    bool display_images = false,
                    const KeepRunning& keep_running = {});
  int getFrameStamps(std::vector<uint64_t>& stamps);

  void displayImages();

  /**
   * @brief Decode the video, stamping the i-th frame with image_stamps[i], and pass every frame to
   * the callback in video order.
   *
   * Scaling, color conversion and encoding run in parallel worker threads; the callback is always
   * called from the calling thread. If keep_running returns false, decoding stops and the frames
   * already being processed are still delivered.
   *
   * @return 0 on success, -1 if the video could not be opened
   */
  int processFrames(const std::vector<uint64_t>& image_stamps,
                    bool grayscale,
                    bool display_images,
                    ImageEncoding encoding,
                    const FrameCallback& callback,
                    const KeepRunning& keep_running = {});

  inline uint32_t getFrameCount() { return num_frames_; }
  inline uint64_t getVideoCreationTime() { return video_creation_time_; }

private:
  /// Receives every decoded frame (in CPU memory); return false to stop decoding.
  using DecodedFrameCallback = std::function<bool(const AVFrame* frame)>;

  /// Decodes the whole video, including the frames buffered in the decoder at the end.
  int decodeVideo(const DecodedFrameCallback& on_frame);

  /// Scales a decoded frame to the output size and converts it to BGR8 or MONO8. Thread-safe.
  cv::Mat convertFrame(const AVFrame* frame, bool grayscale) const;

  bool setupHardwareDecoding();
  static AVPixelFormat getHardwareFormat(AVCodecContext* context, const AVPixelFormat* formats);

  /// Seconds since the start of the video at which the frame is shown.
  double frameTime(const AVFrame* frame) const;

  std::string video_file_;

  // Video properties
  AVFormatContext* format_context_ = nullptr;
  int video_stream_index_ = -1;
  AVCodecContext* codec_context_ = nullptr;
  const AVCodec* codec_ = nullptr;
  AVFrame* frame_ = nullptr;
  AVFrame* sw_frame_ = nullptr;
  AVPacket* packet_ = nullptr;

  // Hardware decoding
  AVBufferRef* hw_device_ctx_ = nullptr;
  AVPixelFormat hw_pix_fmt_ = AV_PIX_FMT_NONE;

  AVDictionary* options_dict_ = nullptr;
  AVDictionaryEntry* tag_dict_ = nullptr;
  AVStream* video_stream_ = nullptr;
  AVCodecParameters* codec_parameters_ = nullptr;

  uint64_t video_creation_time_ = 0;
  uint32_t image_width_ = 0;
  uint32_t image_height_ = 0;
  uint32_t num_frames_ = 0;
};

}  // namespace gopro_ros
