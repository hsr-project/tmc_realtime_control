cmake_minimum_required(VERSION 3.8)
project(tmc_joint_command_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tmc_realtime_tools REQUIRED)
find_package(tmc_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/sources/joint_holding.cpp
  src/sources/joint_trajectory_action.cpp
  src/sources/joint_trajectory_topic.cpp
  src/sources/position_jog_topic.cpp
  src/sources/velocity_jog_topic.cpp
  src/basic_joint_command_controller.cpp
  src/common.cpp
  src/joint_command_controller.cpp
  src/joint_command_source_base.cpp
  src/joints_info.cpp
  src/realtime_trajectory.cpp
  src/state_publisher.cpp
  src/tolerances.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  controller_interface
  hardware_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  tmc_realtime_tools
  tmc_utils
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(controller_interface controllers_plugins.xml)
pluginlib_export_plugin_description_file(tmc_joint_command_controller command_source_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(basic_joint_command_controller_test test/basic_joint_command_controller_test.cpp)
  target_link_libraries(basic_joint_command_controller_test ${PROJECT_NAME})

  ament_add_gmock(joint_command_controller_test test/joint_command_controller_test.cpp)
  target_link_libraries(joint_command_controller_test ${PROJECT_NAME})

  ament_add_gtest(joint_holding_test test/joint_holding_test.cpp)
  target_link_libraries(joint_holding_test ${PROJECT_NAME})

  # async_send_goal実行時にCIでERRORがでて失敗する
  # TODO(Hidaka) actionがテストできるようにする
  # ament_add_gtest(joint_trajectory_action_test test/joint_trajectory_action_test.cpp)
  # target_link_libraries(joint_trajectory_action_test ${PROJECT_NAME})

  ament_add_gtest(joint_trajectory_topic_test test/joint_trajectory_topic_test.cpp)
  target_link_libraries(joint_trajectory_topic_test ${PROJECT_NAME})

  ament_add_gtest(joints_info_test test/joints_info_test.cpp)
  target_link_libraries(joints_info_test ${PROJECT_NAME})

  ament_add_gtest(position_jog_topic_test test/position_jog_topic_test.cpp)
  target_link_libraries(position_jog_topic_test ${PROJECT_NAME})

  ament_add_gtest(state_publisher_test test/state_publisher_test.cpp)
  target_link_libraries(state_publisher_test ${PROJECT_NAME})

  ament_add_gtest(velocity_jog_topic_test test/velocity_jog_topic_test.cpp)
  target_link_libraries(velocity_jog_topic_test ${PROJECT_NAME})
endif()

ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  joint_trajectory_controller
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  tmc_utils
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
