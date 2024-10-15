cmake_minimum_required(VERSION 3.5)
project(tmc_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GripperApplyEffort.action"
)

ament_package()