# ===========================
# ROS 2 (ament) build
# ===========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(ament_libraries
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  rosbag2_cpp
)

gopro_add_core_library()

# ===========================
# Executables
# ===========================
add_executable(gopro_to_rosbag
  src/gopro_to_rosbag.cpp
  src/ros/ros2_bag_writer.cpp
)
ament_target_dependencies(gopro_to_rosbag ${ament_libraries})
target_compile_definitions(gopro_to_rosbag PRIVATE ROS_AVAILABLE=2)
target_link_libraries(gopro_to_rosbag ${PROJECT_NAME}_lib)
install(TARGETS gopro_to_rosbag
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_GOPRO_TO_ASL)
  add_executable(gopro_to_asl src/gopro_to_asl.cpp)
  ament_target_dependencies(gopro_to_asl rclcpp)
  target_compile_definitions(gopro_to_asl PRIVATE ROS_AVAILABLE=2)
  target_link_libraries(gopro_to_asl ${PROJECT_NAME}_lib)
  install(TARGETS gopro_to_asl
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ===========================
# Install
# ===========================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
