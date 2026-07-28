cmake_minimum_required(VERSION 3.8)
project(tmc_realtime_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_gtest(realtime_publisher_test test/realtime_publisher_test.cpp)
  ament_target_dependencies(realtime_publisher_test rclcpp std_msgs)
  target_include_directories(realtime_publisher_test PRIVATE include)
  target_compile_features(realtime_publisher_test PUBLIC cxx_std_20)
endif()

ament_export_dependencies(
  rclcpp
)
ament_export_include_directories(
  include
)
ament_package()
