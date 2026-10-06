//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <cstddef>
#include <ostream>
#include <string>

namespace gopro_ros {

class ProgressBar {
public:
  ProgressBar(std::ostream& os,
              std::size_t line_width,
              std::string message,
              const char symbol = '.');

  // Not copyable
  ProgressBar(const ProgressBar&) = delete;
  ProgressBar& operator=(const ProgressBar&) = delete;

  ~ProgressBar();

  void write(double fraction);

private:
  static constexpr auto kOverhead = sizeof " [100%]";

  std::ostream& os_;
  // A terminal shows the bar updating in place; otherwise (e.g. under ros2 launch, which only
  // forwards complete lines) a line is printed every 5%
  const bool interactive_;
  const std::size_t bar_width_;
  std::string message_;
  const std::string full_bar_;
  int last_printed_percent_ = -1;
};

}  // namespace gopro_ros
