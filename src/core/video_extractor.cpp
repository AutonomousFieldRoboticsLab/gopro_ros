//
// Created by bjoshi on 10/29/20.
//
// Decoding is based on the libavformat/libavcodec tutorial by Stephen Dranger
// (dranger@gmail.com), itself based on a tutorial by Martin Bohme.
//

#include "core/video_extractor.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "utils/color_codes.hpp"
#include "utils/print.hpp"
#include "utils/progress_bar.hpp"
#include "utils/time_utils.hpp"

namespace gopro_ros2 {

namespace fs = std::filesystem;

GoProVideoExtractor::GoProVideoExtractor(const std::string& file,
                                         double scaling_factor,
                                         bool dump_info) {
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
  video_stream_index_ = -1;
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
  if (video_stream_index_ == -1) PRINT_ERROR("Didn't find a video stream");

  video_stream_ = format_context_->streams[video_stream_index_];
  num_frames_ = video_stream_->nb_frames;

  // Find the decoder for the video stream
  codec_ = avcodec_find_decoder(format_context_->streams[video_stream_index_]->codecpar->codec_id);
  if (codec_ == nullptr) {
    PRINT_ERROR("Unsupported codec!");
  }

  // Get a pointer to the codec context for the video stream
  codec_context_ = avcodec_alloc_context3(codec_);
  if (!codec_context_) {
    PRINT_ERROR("Failed to allocated memory for AVCodecContext");
  }

  codec_context_->thread_count = 0;
  codec_context_->thread_type = FF_THREAD_FRAME;

  if (avcodec_parameters_to_context(codec_context_, codec_parameters_) < 0) {
    PRINT_ERROR("Failed to copy codec params to codec context");
  }

  // Open codec
  if (avcodec_open2(codec_context_, codec_, &options_dict_) < 0)
    PRINT_ERROR("Could not open codec");

  // Allocate video frame
  frame_ = av_frame_alloc();

  // Allocate an AVFrame structure
  frame_rgb_ = av_frame_alloc();
  if (frame_rgb_ == nullptr) PRINT_ERROR("Cannot allocate RGB Frame");

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
  // Free the RGB image
  av_free(frame_rgb_);

  // Free the YUV frame
  av_free(frame_);

  // Close the codec
  avcodec_close(codec_context_);

  // Close the format context
  avformat_close_input(&format_context_);
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

  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
    return -1;
  }
  video_stream_ = format_context_->streams[video_stream_index_];

  // Determine required buffer size and allocate buffer
  int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width_, image_height_, 1);
  uint8_t* buffer = static_cast<uint8_t*>(av_malloc(num_bytes * sizeof(uint8_t)));

  sws_ctx_ = sws_getContext(codec_context_->width,
                            codec_context_->height,
                            codec_context_->pix_fmt,
                            image_width_,
                            image_height_,
                            AV_PIX_FMT_RGB24,
                            SWS_BILINEAR,
                            nullptr,
                            nullptr,
                            nullptr);

  // Assign appropriate parts of buffer to image planes in frame_rgb_
  av_image_fill_arrays(frame_rgb_->data,
                       frame_rgb_->linesize,
                       buffer,
                       AV_PIX_FMT_RGB24,
                       image_width_,
                       image_height_,
                       1);

  double global_clock;
  uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;

  while (av_read_frame(format_context_, &packet_) >= 0) {
    // Is this a packet from the video stream?
    if (packet_.stream_index == video_stream_index_) {
      // Decode video frame
      int ret = avcodec_send_packet(codec_context_, &packet_);
      if (ret < 0) {
        PRINT_ERROR("Error sending packet for decoding: " << ret);
        av_packet_unref(&packet_);
        continue;
      }

      while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          break;  // need more packets or end of stream
        } else if (ret < 0) {
          PRINT_ERROR("Error during decoding: " << ret);
          break;
        }

        if (packet_.dts != AV_NOPTS_VALUE) {
          global_clock = frame_->best_effort_timestamp;
          global_video_pkt_pts = packet_.pts;
        } else if (global_video_pkt_pts && global_video_pkt_pts != AV_NOPTS_VALUE) {
          global_clock = global_video_pkt_pts;
        } else {
          global_clock = 0;
        }

        double frame_delay = av_q2d(video_stream_->time_base);
        global_clock *= frame_delay;

        // Account for repeated pictures
        if (frame_->repeat_pict > 0) {
          double extra_delay = frame_->repeat_pict * (frame_delay * 0.5);
          global_clock += extra_delay;
        }

        // Convert the image from its native format to RGB
        sws_scale(sws_ctx_,
                  frame_->data,
                  frame_->linesize,
                  0,
                  codec_context_->height,
                  frame_rgb_->data,
                  frame_rgb_->linesize);

        // Save the frame to disk
        uint64_t nanosecs = static_cast<uint64_t>(global_clock * 1e9);
        if (nanosecs > last_image_stamp_ns) {
          break;
        }
        uint64_t current_stamp = video_creation_time_ + nanosecs;
        std::string string_stamp = uint64ToString(current_stamp);
        std::string stamped_image_filename = image_data_folder + "/" + string_stamp;
        image_stream << string_stamp << "," << string_stamp + ".png" << std::endl;
        saveToPng(frame_rgb_,
                  codec_context_,
                  image_width_,
                  image_height_,
                  video_stream_->time_base,
                  stamped_image_filename);
      }
    }

    // Free the packet that was allocated by av_read_frame
    av_packet_unref(&packet_);
  }

  image_stream.close();

  // Free the RGB image
  av_free(buffer);

  // Close the video file
  avformat_close_input(&format_context_);

  return 0;
}

int GoProVideoExtractor::getFrameStamps(std::vector<uint64_t>& stamps) {
  ProgressBar progress(std::clog, 70u, "Progress", '#');
  stamps.clear();

  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
  }
  video_stream_ = format_context_->streams[video_stream_index_];

  double global_clock;
  uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;
  uint32_t frame_count = 0;

  while (av_read_frame(format_context_, &packet_) >= 0) {
    // Is this a packet from the video stream?
    if (packet_.stream_index == video_stream_index_) {
      // Decode video frame
      int ret = avcodec_send_packet(codec_context_, &packet_);
      if (ret < 0) {
        PRINT_ERROR("Error sending packet for decoding: " << ret);
        av_packet_unref(&packet_);
        continue;
      }

      while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          break;  // need more packets or end of stream
        } else if (ret < 0) {
          PRINT_ERROR("Error during decoding: " << ret);
          break;
        }

        if (packet_.dts != AV_NOPTS_VALUE) {
          global_clock = frame_->best_effort_timestamp;
          global_video_pkt_pts = packet_.pts;
        } else if (global_video_pkt_pts && global_video_pkt_pts != AV_NOPTS_VALUE) {
          global_clock = global_video_pkt_pts;
        } else {
          global_clock = 0;
        }

        double frame_delay = av_q2d(video_stream_->time_base);
        global_clock *= frame_delay;

        // Account for repeated pictures
        if (frame_->repeat_pict > 0) {
          double extra_delay = frame_->repeat_pict * (frame_delay * 0.5);
          global_clock += extra_delay;
        }

        uint64_t usecs = static_cast<uint64_t>(global_clock * 1000000);
        stamps.push_back(usecs);

        frame_count++;
        double percent = static_cast<double>(frame_count) / num_frames_;
        progress.write(percent);
      }
    }
    av_packet_unref(&packet_);
  }

  // Close the video file
  avformat_close_input(&format_context_);

  return 0;
}

void GoProVideoExtractor::displayImages() {
  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
    exit(1);
  }

  video_stream_ = format_context_->streams[video_stream_index_];

  // Determine required buffer size and allocate buffer
  int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width_, image_height_, 1);
  uint8_t* buffer = static_cast<uint8_t*>(av_malloc(num_bytes * sizeof(uint8_t)));

  sws_ctx_ = sws_getContext(codec_context_->width,
                            codec_context_->height,
                            codec_context_->pix_fmt,
                            image_width_,
                            image_height_,
                            AV_PIX_FMT_RGB24,
                            SWS_BILINEAR,
                            nullptr,
                            nullptr,
                            nullptr);

  // Assign appropriate parts of buffer to image planes in frame_rgb_
  av_image_fill_arrays(frame_rgb_->data,
                       frame_rgb_->linesize,
                       buffer,
                       AV_PIX_FMT_RGB24,
                       image_width_,
                       image_height_,
                       1);

  while (av_read_frame(format_context_, &packet_) >= 0) {
    // Is this a packet from the video stream?
    if (packet_.stream_index == video_stream_index_) {
      // Decode video frame
      int ret = avcodec_send_packet(codec_context_, &packet_);
      if (ret < 0) {
        PRINT_ERROR("Error sending packet for decoding: " << ret);
        av_packet_unref(&packet_);
        continue;
      }

      while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          break;  // need more packets or end of stream
        } else if (ret < 0) {
          PRINT_ERROR("Error during decoding: " << ret);
          break;
        }

        // Convert the image from its native format to RGB
        sws_scale(sws_ctx_,
                  frame_->data,
                  frame_->linesize,
                  0,
                  codec_context_->height,
                  frame_rgb_->data,
                  frame_rgb_->linesize);

        cv::Mat img(
            image_height_, image_width_, CV_8UC3, frame_rgb_->data[0], frame_rgb_->linesize[0]);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        cv::imshow("GoPro Video", img);
        cv::waitKey(1);
      }
    }

    // Free the packet that was allocated by av_read_frame
    av_packet_unref(&packet_);
  }

  cv::destroyAllWindows();

  // Free the RGB image
  av_free(buffer);

  // Close the video file
  avformat_close_input(&format_context_);
}

int GoProVideoExtractor::processFrames(const std::vector<uint64_t>& image_stamps,
                                       bool grayscale,
                                       bool display_images,
                                       const FrameCallback& callback) {
  ProgressBar progress(std::clog, 80u, "Progress");

  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, video_file_.c_str(), nullptr, nullptr) != 0) {
    PRINT_ERROR("Could not open file" << video_file_.c_str());
    return -1;
  }
  video_stream_ = format_context_->streams[video_stream_index_];

  // Determine required buffer size and allocate buffer
  int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width_, image_height_, 1);
  uint8_t* buffer = static_cast<uint8_t*>(av_malloc(num_bytes * sizeof(uint8_t)));

  sws_ctx_ = sws_getContext(codec_context_->width,
                            codec_context_->height,
                            codec_context_->pix_fmt,
                            image_width_,
                            image_height_,
                            AV_PIX_FMT_RGB24,
                            SWS_BILINEAR,
                            nullptr,
                            nullptr,
                            nullptr);

  // Assign appropriate parts of buffer to image planes in frame_rgb_
  av_image_fill_arrays(frame_rgb_->data,
                       frame_rgb_->linesize,
                       buffer,
                       AV_PIX_FMT_RGB24,
                       image_width_,
                       image_height_,
                       1);

  uint32_t frame_count = 0;
  while (av_read_frame(format_context_, &packet_) >= 0) {
    // Is this a packet from the video stream?
    if (packet_.stream_index == video_stream_index_) {
      // Decode video frame
      int ret = avcodec_send_packet(codec_context_, &packet_);
      if (ret < 0) {
        PRINT_ERROR("Error sending packet for decoding: " << ret);
        av_packet_unref(&packet_);
        continue;
      }

      while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          break;  // need more packets or end of stream
        } else if (ret < 0) {
          PRINT_ERROR("Error during decoding: " << ret);
          break;
        }

        if (frame_count == image_stamps.size()) {
          PRINT_WARNING(
              "Number of images does not match number of timestamps. "
              "This should only happen in last/single GoPro Video !!");
          PRINT_WARNING("Skipping " << (num_frames_ - image_stamps.size()) << "/" << num_frames_
                                    << " images");
          break;
        }

        // Convert the image from its native format to RGB
        sws_scale(sws_ctx_,
                  frame_->data,
                  frame_->linesize,
                  0,
                  codec_context_->height,
                  frame_rgb_->data,
                  frame_rgb_->linesize);

        cv::Mat img(
            image_height_, image_width_, CV_8UC3, frame_rgb_->data[0], frame_rgb_->linesize[0]);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        if (grayscale) {
          cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        }

        if (display_images) {
          cv::imshow("GoPro Video", img);
          cv::waitKey(1);
        }

        callback(img, image_stamps[frame_count]);
        frame_count++;
      }

      double percent = static_cast<double>(frame_count) / num_frames_;
      progress.write(percent);
    }

    // Free the packet that was allocated by av_read_frame
    av_packet_unref(&packet_);
  }

  if (display_images) cv::destroyAllWindows();

  // Free the RGB image
  av_free(buffer);

  // Close the video file
  avformat_close_input(&format_context_);

  return 0;
}

int GoProVideoExtractor::extractFrames(const std::string& image_folder,
                                       const std::vector<uint64_t>& image_stamps,
                                       bool grayscale,
                                       bool display_images) {
  std::string image_data_folder = image_folder + "/data";
  std::string image_file = image_folder + "/data.csv";
  std::ofstream image_stream;
  image_stream.open(image_file, std::ofstream::app);
  image_stream << std::fixed << std::setprecision(19);

  int ret = processFrames(
      image_stamps, grayscale, display_images, [&](const cv::Mat& image, uint64_t stamp_ns) {
        std::string string_stamp = uint64ToString(stamp_ns);
        image_stream << string_stamp << "," << string_stamp + ".png" << std::endl;
        cv::imwrite(image_data_folder + "/" + string_stamp + ".png", image);
      });

  image_stream.close();
  return ret;
}

}  // namespace gopro_ros2
