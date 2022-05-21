//
// Created by bjoshi on 8/26/20.
//
#include "Utils.h"

#include <chrono>
#include <iostream>
#include <sstream>

#include "date.h"

uint64_t parseISO(const std::string& iso_date) {
  using namespace date;
  using namespace std::chrono;

  date::sys_time<std::chrono::nanoseconds> tp;
  std::istringstream in(iso_date);
  in >> date::parse("%FT%TZ", tp);
  if (in.fail()) {
    in.clear();
    in.exceptions(std::ios::failbit);
    in.str(iso_date);
    in >> date::parse("%FT%T%Ez", tp);
  }

  uint64_t time = tp.time_since_epoch().count();

  return time;
}

std::string uint64_to_string(uint64_t value) {
  std::ostringstream os;
  os << value;
  return os.str();
}

uint64_t get_offset_1904() {
  using namespace date;
  using namespace std::chrono;
  constexpr auto offset = sys_days{January / 1 / 1970} - sys_days{January / 1 / 1904};
  uint64_t offset_secs = duration_cast<seconds>(offset).count();
  return offset_secs;
}

ProgressBar::ProgressBar(std::ostream& os,
                         std::size_t line_width,
                         std::string message_,
                         const char symbol)
    : os{os},
      bar_width{line_width - overhead},
      message{std::move(message_)},
      full_bar{std::string(bar_width, symbol) + std::string(bar_width, ' ')} {
  if (message.size() + 1 >= bar_width || message.find('\n') != message.npos) {
    os << message << '\n';
    message.clear();
  } else {
    message += ' ';
  }
  write(0.0);
}

ProgressBar::~ProgressBar() {
  write(1.0);
  os << '\n';
}

void ProgressBar::write(double fraction) {
  // clamp fraction to valid range [0,1]
  if (fraction < 0)
    fraction = 0;
  else if (fraction > 1)
    fraction = 1;

  auto width = bar_width - message.size();
  auto offset = bar_width - static_cast<unsigned>(width * fraction);

  os << '\r' << message;
  os.write(full_bar.data() + offset, width);
  os << " [" << std::setw(3) << static_cast<int>(100 * fraction) << "%] " << std::flush;
}