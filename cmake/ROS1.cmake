# ===========================
# ROS 1 (catkin) build
# ===========================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS roscpp rosbag std_msgs sensor_msgs geometry_msgs cv_bridge
)

gopro_add_core_library()

# ===========================
# Executables
# ===========================
add_executable(gopro_to_rosbag
  src/gopro_to_rosbag.cpp
  src/ros/ros1_bag_writer.cpp
)
target_include_directories(gopro_to_rosbag SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_definitions(gopro_to_rosbag PRIVATE ROS_AVAILABLE=1)
target_link_libraries(gopro_to_rosbag ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
install(TARGETS gopro_to_rosbag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(BUILD_GOPRO_TO_ASL)
  add_executable(gopro_to_asl src/gopro_to_asl.cpp)
  target_include_directories(gopro_to_asl SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_compile_definitions(gopro_to_asl PRIVATE ROS_AVAILABLE=1)
  target_link_libraries(gopro_to_asl ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  install(TARGETS gopro_to_asl
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# ===========================
# Install
# ===========================
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
