cmake_minimum_required(VERSION 3.8)
project(tmc_realtime_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/digital_output_controller.cpp
  src/${PROJECT_NAME}/digital_input_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  realtime_tools
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface controllers_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(digital_output_controller_test test/digital_output_controller_test.cpp)
  target_link_libraries(digital_output_controller_test ${PROJECT_NAME})
  ament_add_gtest(digital_input_controller_test test/digital_input_controller_test.cpp)
  target_link_libraries(digital_input_controller_test ${PROJECT_NAME})
endif()


ament_export_dependencies(
  controller_interface
  realtime_tools
  std_msgs
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
