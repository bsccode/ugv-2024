cmake_minimum_required(VERSION 3.5)
project(ugv_dev)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(subscriber_dev src/subscriber_dev.cpp)
ament_target_dependencies(subscriber_dev rclcpp std_msgs sensor_msgs)

install(TARGETS
  subscriber_dev
  DESTINATION lib/${PROJECT_NAME})

ament_package()