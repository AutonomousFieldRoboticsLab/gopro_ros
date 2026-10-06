//
// Created by bjoshi on 10/29/20.
//
// Decoding is based on the libavformat/libavcodec tutorial by Stephen Dranger
// (dranger@gmail.com), itself based on a tutorial by Martin Bohme.
//

#include "core/video_extractor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <thread>
#include <tuple>

#include "utils/print.hpp"
#include "utils/progress_bar.hpp"
#include "utils/thread_pool.hpp"
#include "utils/time_utils.hpp"

namespace gopro_ros {

namespace fs = std::filesystem;

namespace {

/// A per-thread scaling context, reused as long as the input and output formats do not change.
struct SwsCache {
  SwsContext* context = nullptr;
  std::tuple<int, int, int, int, bool> key{-1, -1, -1, -1, false};

  ~SwsCache() { sws_freeContext(context); }
};

/// Maps the deprecated full-range "J" pixel formats to their regular counterparts.
AVPixelFormat normalizePixelFormat(AVPixelFormat format, bool& full_range) {
  switch (format) {
    case AV_PIX_FMT_YUVJ420P:
      full_range = true;
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
      full_range = true;
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
      full_range = true;
      return AV_PIX_FMT_YUV444P;
    default:
      return format;
  }
}

std::size_t alignUp(std::size_t value, std::size_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

}  // namespace

GoProVideoExtractor::GoProVideoExtractor(const std::string& file,
                                         double scaling_factor,
                                         bool dump_info,
                                         bool hardware_decoding) {
  video_file_ = file;

  // Open video file
  PRINT_INFO("Opening Video File: " << video_file_);
  format_context_ = avformat_alloc_context();
  if (!format_context_) {
    PRINT_ERROR("Could not allocate memory for Format Context");
    return;
  }

  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
    return;
  }

  // Retrieve stream information
  if (avformat_find_stream_info(format_context_, nullptr) < 0) {
    PRINT_ERROR("Couldn't find stream information");
    return;
  }

  // Dump information about file onto standard error
  if (dump_info) av_dump_format(format_context_, 0, video_file_.c_str(), 0);

  // Find the first video stream
  std::string creation_time;

  for (uint32_t i = 0; i < format_context_->nb_streams; i++) {
    codec_parameters_ = format_context_->streams[i]->codecpar;
    if (codec_parameters_->codec_type == AVMEDIA_TYPE_VIDEO) {
      tag_dict_ = av_dict_get(format_context_->metadata, "", tag_dict_, AV_DICT_IGNORE_SUFFIX);
      while (tag_dict_) {
        if (strcmp(tag_dict_->key, "creation_time") == 0) {
          std::stringstream ss;
          ss << tag_dict_->value;
          ss >> creation_time;
        }
        tag_dict_ = av_dict_get(format_context_->metadata, "", tag_dict_, AV_DICT_IGNORE_SUFFIX);
      }

      video_stream_index_ = i;
      break;
    }
  }

  video_creation_time_ = parseIsoDate(creation_time);
  if (video_stream_index_ == -1) {
    PRINT_ERROR("Didn't find a video stream");
    return;
  }

  video_stream_ = format_context_->streams[video_stream_index_];
  num_frames_ = video_stream_->nb_frames;

  // Find the decoder for the video stream
  codec_ = avcodec_find_decoder(codec_parameters_->codec_id);
  if (codec_ == nullptr) {
    PRINT_ERROR("Unsupported codec!");
    return;
  }

  codec_context_ = avcodec_alloc_context3(codec_);
  if (!codec_context_) {
    PRINT_ERROR("Failed to allocated memory for AVCodecContext");
    return;
  }

  codec_context_->thread_count = 0;
  codec_context_->thread_type = FF_THREAD_FRAME;

  if (avcodec_parameters_to_context(codec_context_, codec_parameters_) < 0) {
    PRINT_ERROR("Failed to copy codec params to codec context");
  }

  if (hardware_decoding && !setupHardwareDecoding()) {
    PRINT_INFO("No hardware video decoder available, decoding on the CPU");
  }

  // Open codec
  if (avcodec_open2(codec_context_, codec_, &options_dict_) < 0)
    PRINT_ERROR("Could not open codec");

  frame_ = av_frame_alloc();
  sw_frame_ = av_frame_alloc();
  packet_ = av_packet_alloc();

  image_height_ = codec_context_->height;
  image_width_ = codec_context_->width;

  if (scaling_factor != 1.0) {
    image_height_ = static_cast<int>(image_height_ * scaling_factor);
    image_width_ = static_cast<int>(image_width_ * scaling_factor);
  }

  // Close the video format context
  avformat_close_input(&format_context_);
}

GoProVideoExtractor::~GoProVideoExtractor() {
  av_frame_free(&frame_);
  av_frame_free(&sw_frame_);
  av_packet_free(&packet_);
  avcodec_free_context(&codec_context_);
  av_buffer_unref(&hw_device_ctx_);
  avformat_close_input(&format_context_);
}

bool GoProVideoExtractor::setupHardwareDecoding() {
  // Preferred hardware decoders, in order
  const AVHWDeviceType device_types[] = {AV_HWDEVICE_TYPE_CUDA, AV_HWDEVICE_TYPE_VAAPI};

  for (AVHWDeviceType device_type : device_types) {
    for (int i = 0;; i++) {
      const AVCodecHWConfig* config = avcodec_get_hw_config(codec_, i);
      if (config == nullptr) break;
      if (!(config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) ||
          config->device_type != device_type) {
        continue;
      }

      if (av_hwdevice_ctx_create(&hw_device_ctx_, device_type, nullptr, nullptr, 0) < 0) break;

      hw_pix_fmt_ = config->pix_fmt;
      codec_context_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
      codec_context_->opaque = this;
      codec_context_->get_format = &GoProVideoExtractor::getHardwareFormat;
      PRINT_INFO("Using hardware video decoding (" << av_hwdevice_get_type_name(device_type)
                                                   << ")");
      return true;
    }
  }
  return false;
}

AVPixelFormat GoProVideoExtractor::getHardwareFormat(AVCodecContext* context,
                                                     const AVPixelFormat* formats) {
  const auto* self = static_cast<const GoProVideoExtractor*>(context->opaque);
  for (const AVPixelFormat* format = formats; *format != AV_PIX_FMT_NONE; format++) {
    if (*format == self->hw_pix_fmt_) return *format;
  }
  PRINT_WARNING("The hardware decoder does not support this video, decoding on the CPU");
  return avcodec_default_get_format(context, formats);
}

int GoProVideoExtractor::decodeVideo(const DecodedFrameCallback& on_frame) {
  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
    return -1;
  }
  video_stream_ = format_context_->streams[video_stream_index_];

  bool keep_going = true;

  // Hands every frame the decoder has ready to on_frame
  auto receive_frames = [&]() {
    while (keep_going) {
      int ret = avcodec_receive_frame(codec_context_, frame_);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return;
      if (ret < 0) {
        PRINT_ERROR("Error during decoding: " << ret);
        return;
      }

      const AVFrame* decoded = frame_;
      if (frame_->format == hw_pix_fmt_) {
        // Copy the frame from GPU to CPU memory
        if (av_hwframe_transfer_data(sw_frame_, frame_, 0) < 0) {
          PRINT_ERROR("Error copying a decoded frame from the GPU");
          av_frame_unref(frame_);
          continue;
        }
        av_frame_copy_props(sw_frame_, frame_);
        decoded = sw_frame_;
      }

      keep_going = on_frame(decoded);
      av_frame_unref(sw_frame_);
      av_frame_unref(frame_);
    }
  };

  while (keep_going && av_read_frame(format_context_, packet_) >= 0) {
    // Is this a packet from the video stream?
    if (packet_->stream_index == video_stream_index_) {
      int ret = avcodec_send_packet(codec_context_, packet_);
      if (ret < 0) {
        PRINT_ERROR("Error sending packet for decoding: " << ret);
      } else {
        receive_frames();
      }
    }

    // Free the packet that was allocated by av_read_frame
    av_packet_unref(packet_);
  }

  // At the end of the file, flush the frames still buffered in the decoder
  if (keep_going) {
    avcodec_send_packet(codec_context_, nullptr);
    receive_frames();
  }
  avcodec_flush_buffers(codec_context_);

  // Close the video file
  avformat_close_input(&format_context_);
  return 0;
}

cv::Mat GoProVideoExtractor::convertFrame(const AVFrame* frame, bool grayscale) const {
  thread_local SwsCache cache;

  bool full_range = frame->color_range == AVCOL_RANGE_JPEG;
  const AVPixelFormat src_format =
      normalizePixelFormat(static_cast<AVPixelFormat>(frame->format), full_range);
  const AVPixelFormat dst_format = grayscale ? AV_PIX_FMT_GRAY8 : AV_PIX_FMT_BGR24;

  const auto key = std::make_tuple(frame->width,
                                   frame->height,
                                   static_cast<int>(src_format),
                                   static_cast<int>(dst_format),
                                   full_range);
  if (cache.context == nullptr || cache.key != key) {
    sws_freeContext(cache.context);
    cache.context = sws_getContext(frame->width,
                                   frame->height,
                                   src_format,
                                   image_width_,
                                   image_height_,
                                   dst_format,
                                   SWS_BILINEAR,
                                   nullptr,
                                   nullptr,
                                   nullptr);
    if (cache.context == nullptr) {
      PRINT_ERROR("Could not create the scaling context");
      return cv::Mat();
    }
    // Input range as stored in the video (GoPro uses the full range); output is always full range
    const int* coefficients = sws_getCoefficients(SWS_CS_DEFAULT);
    sws_setColorspaceDetails(
        cache.context, coefficients, full_range, coefficients, 1, 0, 1 << 16, 1 << 16);
    cache.key = key;
  }

  // Rows are padded to 64 bytes for swscale's SIMD code
  const int channels = grayscale ? 1 : 3;
  const std::size_t step = alignUp(static_cast<std::size_t>(image_width_) * channels, 64);
  cv::Mat buffer(image_height_, static_cast<int>(step), CV_8UC1);

  uint8_t* dst_data[4] = {buffer.data, nullptr, nullptr, nullptr};
  int dst_linesize[4] = {static_cast<int>(step), 0, 0, 0};
  sws_scale(cache.context, frame->data, frame->linesize, 0, frame->height, dst_data, dst_linesize);

  return buffer.colRange(0, image_width_ * channels).reshape(channels);
}

double GoProVideoExtractor::frameTime(const AVFrame* frame) const {
  const double frame_delay = av_q2d(video_stream_->time_base);
  double time = frame->best_effort_timestamp == AV_NOPTS_VALUE
                    ? 0.0
                    : static_cast<double>(frame->best_effort_timestamp) * frame_delay;

  // Account for repeated pictures
  if (frame->repeat_pict > 0) time += frame->repeat_pict * (frame_delay * 0.5);
  return time;
}

void GoProVideoExtractor::saveToPng(AVFrame* frame,
                                    AVCodecContext* codec_context,
                                    int width,
                                    int height,
                                    AVRational time_base,
                                    const std::string& filename) {
  const AVCodec* out_codec = avcodec_find_encoder(AV_CODEC_ID_PNG);
  AVCodecContext* out_codec_ctx = avcodec_alloc_context3(out_codec);

  out_codec_ctx->width = width;
  out_codec_ctx->height = height;
  out_codec_ctx->pix_fmt = AV_PIX_FMT_RGB24;
  out_codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
  out_codec_ctx->codec_id = AV_CODEC_ID_PNG;
  out_codec_ctx->time_base.num = codec_context->time_base.num;
  out_codec_ctx->time_base.den = codec_context->time_base.den;

  frame->height = height;
  frame->width = width;
  frame->format = AV_PIX_FMT_RGB24;

  if (!out_codec || avcodec_open2(out_codec_ctx, out_codec, nullptr) < 0) {
    return;
  }

  AVPacket out_packet;
  av_init_packet(&out_packet);
  out_packet.size = 0;
  out_packet.data = nullptr;

  avcodec_send_frame(out_codec_ctx, frame);
  int ret = -1;
  while (ret < 0) {
    ret = avcodec_receive_packet(out_codec_ctx, &out_packet);
  }

  const std::string png_filename = filename + ".png";
  FILE* out_png = fopen(png_filename.c_str(), "wb");
  fwrite(out_packet.data, out_packet.size, 1, out_png);
  fclose(out_png);
}

void GoProVideoExtractor::saveRaw(AVFrame* frame,
                                  int width,
                                  int height,
                                  const std::string& filename) {
  // Open file
  const std::string ppm_filename = filename + ".ppm";
  FILE* file = fopen(ppm_filename.c_str(), "wb");
  if (file == nullptr) return;

  // Write header
  fprintf(file, "P6\n%d %d\n255\n", width, height);

  // Write pixel data
  for (int y = 0; y < height; y++)
    fwrite(frame->data[0] + y * frame->linesize[0], 1, width * 3, file);

  // Close file
  fclose(file);
}

int GoProVideoExtractor::extractFrames(const std::string& image_folder,
                                       uint64_t last_image_stamp_ns) {
  std::string image_data_folder = image_folder + "/data";
  if (!fs::is_directory(image_data_folder)) {
    fs::create_directories(image_data_folder);
  }
  std::string image_file = image_folder + "/data.csv";
  std::ofstream image_stream;
  image_stream.open(image_file);
  image_stream << std::fixed << std::setprecision(19);
  image_stream << "#timestamp [ns],filename" << std::endl;

  int ret = decodeVideo([&](const AVFrame* frame) {
    uint64_t nanosecs = static_cast<uint64_t>(frameTime(frame) * 1e9);
    if (nanosecs > last_image_stamp_ns) return false;

    std::string string_stamp = uint64ToString(video_creation_time_ + nanosecs);
    image_stream << string_stamp << "," << string_stamp + ".png" << std::endl;
    cv::imwrite(image_data_folder + "/" + string_stamp + ".png", convertFrame(frame, false));
    return true;
  });

  image_stream.close();
  return ret;
}

int GoProVideoExtractor::getFrameStamps(std::vector<uint64_t>& stamps) {
  ProgressBar progress(std::clog, 70u, "Progress", '#');
  stamps.clear();

  return decodeVideo([&](const AVFrame* frame) {
    stamps.push_back(static_cast<uint64_t>(frameTime(frame) * 1000000));
    progress.write(static_cast<double>(stamps.size()) / num_frames_);
    return true;
  });
}

void GoProVideoExtractor::displayImages() {
  decodeVideo([&](const AVFrame* frame) {
    cv::imshow("GoPro Video", convertFrame(frame, false));
    cv::waitKey(1);
    return true;
  });
  cv::destroyAllWindows();
}

int GoProVideoExtractor::processFrames(const std::vector<uint64_t>& image_stamps,
                                       bool grayscale,
                                       bool display_images,
                                       ImageEncoding encoding,
                                       const FrameCallback& callback,
                                       const KeepRunning& keep_running) {
  ProgressBar progress(std::clog, 80u, "Progress");

  // Scaling, color conversion and encoding of each frame run in parallel; the frames are handed to
  // the callback in video order
  const std::size_t num_workers =
      std::clamp<std::size_t>(std::thread::hardware_concurrency(), 2, 8);
  ThreadPool pool(num_workers);
  std::deque<std::future<Frame>> in_flight;

  auto deliver_oldest = [&]() {
    Frame frame = in_flight.front().get();
    in_flight.pop_front();
    if (display_images) {
      cv::imshow("GoPro Video", frame.image);
      cv::waitKey(1);
    }
    callback(frame);
  };

  uint32_t frame_count = 0;
  int ret = decodeVideo([&](const AVFrame* decoded) {
    if (keep_running && !keep_running()) {
      PRINT_WARNING("Stopping early after " << frame_count << "/" << num_frames_ << " images");
      return false;
    }

    if (frame_count == image_stamps.size()) {
      PRINT_WARNING(
          "Number of images does not match number of timestamps. "
          "This should only happen in last/single GoPro Video !!");
      PRINT_WARNING("Skipping " << (num_frames_ - image_stamps.size()) << "/" << num_frames_
                                << " images");
      return false;
    }

    AVFrame* frame = av_frame_clone(decoded);
    const uint64_t stamp_ns = image_stamps[frame_count++];
    in_flight.push_back(pool.submit([this, frame, stamp_ns, grayscale, encoding]() mutable {
      Frame result;
      result.stamp_ns = stamp_ns;
      result.image = convertFrame(frame, grayscale);
      av_frame_free(&frame);

      if (encoding == ImageEncoding::kJpeg) {
        cv::imencode(".jpg", result.image, result.encoded);
      } else if (encoding == ImageEncoding::kPng) {
        cv::imencode(".png", result.image, result.encoded);
      }
      return result;
    }));

    // Limit the number of frames held in memory
    if (in_flight.size() >= 2 * num_workers) deliver_oldest();

    progress.write(static_cast<double>(frame_count) / num_frames_);
    return true;
  });

  // Frames that are already being processed are still delivered, also when stopping early
  while (!in_flight.empty()) deliver_oldest();

  if (display_images) cv::destroyAllWindows();
  return ret;
}

int GoProVideoExtractor::extractFrames(const std::string& image_folder,
                                       const std::vector<uint64_t>& image_stamps,
                                       bool grayscale,
                                       bool display_images,
                                       const KeepRunning& keep_running) {
  std::string image_data_folder = image_folder + "/data";
  std::string image_file = image_folder + "/data.csv";
  std::ofstream image_stream;
  image_stream.open(image_file, std::ofstream::app);
  image_stream << std::fixed << std::setprecision(19);

  int ret = processFrames(
      image_stamps,
      grayscale,
      display_images,
      ImageEncoding::kPng,
      [&](const Frame& frame) {
        std::string string_stamp = uint64ToString(frame.stamp_ns);
        image_stream << string_stamp << "," << string_stamp + ".png" << std::endl;
        std::ofstream png(image_data_folder + "/" + string_stamp + ".png", std::ios::binary);
        png.write(reinterpret_cast<const char*>(frame.encoded.data()), frame.encoded.size());
      },
      keep_running);

  image_stream.close();
  return ret;
}

}  // namespace gopro_ros
