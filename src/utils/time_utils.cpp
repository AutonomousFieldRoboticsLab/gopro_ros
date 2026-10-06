//
// Created by bjoshi on 8/26/20.
//

#include "utils/time_utils.hpp"

#include <chrono>
#include <sstream>

#include "date/date.h"

namespace gopro_ros {

uint64_t parseIsoDate(const std::string& iso_date) {
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

std::string uint64ToString(uint64_t value) {
  std::ostringstream os;
  os << value;
  return os.str();
}

uint64_t getOffset1904() {
  using namespace date;
  constexpr auto offset = sys_days{January / 1 / 1970} - sys_days{January / 1 / 1904};
  uint64_t offset_secs = std::chrono::duration_cast<std::chrono::seconds>(offset).count();
  return offset_secs;
}

}  // namespace gopro_ros
