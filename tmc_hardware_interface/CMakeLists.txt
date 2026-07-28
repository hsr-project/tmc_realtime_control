# This package has no tests
cmake_minimum_required(VERSION 3.5)
project(tmc_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/generic_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface mock_components_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME})
ament_package()
