//
// Created by bjoshi on 8/26/20.
//

#include "utils/progress_bar.hpp"

#include <iomanip>
#include <utility>

namespace gopro_ros2 {

ProgressBar::ProgressBar(std::ostream& os,
                         std::size_t line_width,
                         std::string message,
                         const char symbol)
    : os_{os},
      bar_width_{line_width - kOverhead},
      message_{std::move(message)},
      full_bar_{std::string(bar_width_, symbol) + std::string(bar_width_, ' ')} {
  if (message_.size() + 1 >= bar_width_ || message_.find('\n') != message_.npos) {
    os_ << message_ << '\n';
    message_.clear();
  } else {
    message_ += ' ';
  }
  write(0.0);
}

ProgressBar::~ProgressBar() {
  write(1.0);
  os_ << '\n';
}

void ProgressBar::write(double fraction) {
  // Clamp fraction to valid range [0, 1]
  if (fraction < 0)
    fraction = 0;
  else if (fraction > 1)
    fraction = 1;

  auto width = bar_width_ - message_.size();
  auto offset = bar_width_ - static_cast<unsigned>(width * fraction);

  os_ << '\r' << message_;
  os_.write(full_bar_.data() + offset, width);
  os_ << " [" << std::setw(3) << static_cast<int>(100 * fraction) << "%] " << std::flush;
}

}  // namespace gopro_ros2
