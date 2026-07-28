cmake_minimum_required(VERSION 3.8)
project(tmc_realtime_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(tmc_exxx_servo_motor_protocol REQUIRED)
find_package(tmc_realtime_tools REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(urdf REQUIRED)

find_package(fmt REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/digital_output_controller.cpp
  src/${PROJECT_NAME}/digital_input_controller.cpp
  src/${PROJECT_NAME}/empty_command_controller.cpp
  src/${PROJECT_NAME}/exxx_drive_mode_controller.cpp
  src/${PROJECT_NAME}/motion_command_limiter_controller.cpp
  src/${PROJECT_NAME}/servo_state_broadcaster.cpp
  src/${PROJECT_NAME}/infrequent_servo_access_controller.cpp
  src/${PROJECT_NAME}/utils.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:test>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  moveit_msgs
  pluginlib
  realtime_tools
  std_msgs
  std_srvs
  tmc_control_msgs
  tmc_exxx_servo_motor_protocol
  tmc_realtime_tools
  tmc_utils
  urdf
)
target_link_libraries(${PROJECT_NAME} fmt::fmt)

pluginlib_export_plugin_description_file(controller_interface controllers_plugins.xml)

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
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(digital_output_controller_test test/digital_output_controller_test.cpp)
  target_link_libraries(digital_output_controller_test ${PROJECT_NAME})
  ament_add_gtest(digital_input_controller_test test/digital_input_controller_test.cpp)
  target_link_libraries(digital_input_controller_test ${PROJECT_NAME})

  ament_add_gtest(empty_command_controller_test test/empty_command_controller_test.cpp)
  target_link_libraries(empty_command_controller_test ${PROJECT_NAME})

  ament_add_gtest(exxx_drive_mode_controller_test test/exxx_drive_mode_controller_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}
    controller_interface
    lifecycle_msgs
    pluginlib
    realtime_tools
    std_msgs
    tmc_control_msgs
    tmc_utils
  )

  target_link_libraries(exxx_drive_mode_controller_test ${PROJECT_NAME})

  ament_add_gtest(servo_state_broadcaster_test test/servo_state_broadcaster_test.cpp)
  ament_target_dependencies(servo_state_broadcaster_test
    controller_interface
    lifecycle_msgs
    pluginlib
    realtime_tools
    std_msgs
    tmc_control_msgs
    tmc_utils
  )

  target_link_libraries(servo_state_broadcaster_test ${PROJECT_NAME})
  ament_add_gtest(infrequent_servo_access_controller_test test/infrequent_servo_access_controller_test.cpp
                  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
  ament_target_dependencies(infrequent_servo_access_controller_test
    controller_interface
    lifecycle_msgs
    pluginlib
    realtime_tools
    std_msgs
    tmc_control_msgs
    tmc_exxx_servo_motor_protocol
    tmc_utils
  )

  target_link_libraries(infrequent_servo_access_controller_test ${PROJECT_NAME})

  ament_add_gtest(motion_command_limiter_controller_test test/motion_command_limiter_controller_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(motion_command_limiter_controller_test ${PROJECT_NAME})
endif()


ament_export_dependencies(
  controller_interface
  moveit_msgs
  realtime_tools
  std_msgs
  tmc_control_msgs
  tmc_realtime_tools
  tmc_utils
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_include_directories(
  include
)
ament_package()
