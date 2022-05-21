cmake_minimum_required(VERSION 3.10)
project(gopro_ros)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs rosbag
    roscpp sensor_msgs std_msgs )

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS cv_bridge geometry_msgs rosbag roscpp sensor_msgs std_msgs
		LIBRARIES ${PROJECT_NAME}
)

find_package(ffmpeg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3 REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG/LibAV libraries")
else()
    message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif()

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${FMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )


file(GLOB c_src "src/*.c")
file(GLOB cpp_src "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${c_src} ${cpp_src})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} ${FFMPEG_LIBRARIES} ${OpenCV_LIBS}  -lstdc++fs)

add_executable(gopro_to_asl gopro_to_asl.cpp)
target_link_libraries(gopro_to_asl ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(gopro_to_rosbag gopro_to_rosbag.cpp)
target_link_libraries(gopro_to_rosbag ${PROJECT_NAME} ${catkin_LIBRARIES})

