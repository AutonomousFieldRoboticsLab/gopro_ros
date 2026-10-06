//
// Created by bjoshi on 8/26/20.
//

#pragma once

#include <cstdint>
#include <string>

namespace gopro_ros2 {

/**
 * @brief Parse an ISO 8601 date string (e.g. "2020-10-29T12:00:00Z") into nanoseconds since epoch.
 */
uint64_t parseIsoDate(const std::string& iso_date);

std::string uint64ToString(uint64_t value);

/**
 * @brief Seconds between the MP4 epoch (1904-01-01) and the Unix epoch (1970-01-01).
 */
uint64_t getOffset1904();

}  // namespace gopro_ros2
