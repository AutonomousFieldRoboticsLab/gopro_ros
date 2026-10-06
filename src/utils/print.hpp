#pragma once

#include <iostream>

#include "utils/color_codes.hpp"

// ROS-agnostic logging helpers so that the core library does not depend on ROS 1 or ROS 2.
// They take stream-style arguments, e.g. PRINT_INFO("Read " << n << " samples").

#define PRINT_INFO(msg)            \
  do {                             \
    std::cout << msg << std::endl; \
  } while (0)

#define PRINT_WARNING(msg)                            \
  do {                                                \
    std::cout << YELLOW << msg << RESET << std::endl; \
  } while (0)

#define PRINT_ERROR(msg)                           \
  do {                                             \
    std::cerr << RED << msg << RESET << std::endl; \
  } while (0)
