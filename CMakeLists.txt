cmake_minimum_required(VERSION 3.10)
project(gopro_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# By default we build the ROS executables, but this can be disabled to build only the library
option(ENABLE_ROS "Build the ROS executables (ROS 1 or ROS 2, whichever is found)" ON)
option(BUILD_GOPRO_TO_ASL "Build gopro_to_asl executable" ON)

# ===========================
# Dependencies shared by all builds
# ===========================
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ffmpeg REQUIRED)

# ===========================
# ROS-agnostic core library
# ===========================
set(CORE_LIBRARY_SOURCES
  src/core/imu_extractor.cpp
  src/core/video_extractor.cpp
  src/utils/progress_bar.cpp
  src/utils/rosbag_utils.cpp
  src/utils/time_utils.cpp
  # Vendored GPMF parser (https://github.com/gopro/gpmf-parser)
  src/gpmf/GPMF_mp4reader.c
  src/gpmf/GPMF_parser.c
  src/gpmf/GPMF_print.cpp
  src/gpmf/GPMF_utils.c
)

# Called from ROS1.cmake / ROS2.cmake once the ROS package has been declared.
# The library is internal to the executables, so it is always static (catkin otherwise defaults to
# shared libraries, which would then also have to be installed).
macro(gopro_add_core_library)
  add_library(${PROJECT_NAME}_lib STATIC ${CORE_LIBRARY_SOURCES})
  target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
      src
      ${FFMPEG_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_lib
    ${FFMPEG_LIBRARIES}
    ${OpenCV_LIBS}
  )
endmacro()

# ===========================
# Find our ROS version
# ===========================
# NOTE: Default to ROS 1 if both are in the environment
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if(catkin_FOUND AND ENABLE_ROS)
  message(STATUS "ROS *1* found, building ROS1.cmake")
  include(${PROJECT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif(ament_cmake_FOUND AND ENABLE_ROS)
  message(STATUS "ROS *2* found, building ROS2.cmake")
  include(${PROJECT_SOURCE_DIR}/cmake/ROS2.cmake)
else()
  message(WARNING "No ROS found (or ENABLE_ROS=OFF): building only the core library")
  gopro_add_core_library()
endif()
