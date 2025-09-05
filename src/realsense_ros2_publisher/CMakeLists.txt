cmake_minimum_required(VERSION 3.8)
project(realsense_ros2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(realsense_publisher src/realsense_publisher.cpp)

target_include_directories(realsense_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/librealsense2
)

target_link_libraries(realsense_publisher
  ${OpenCV_LIBRARIES}
  realsense2
  realsense2-net
)

ament_target_dependencies(realsense_publisher
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

install(TARGETS realsense_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
