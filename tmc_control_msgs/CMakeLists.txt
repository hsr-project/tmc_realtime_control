cmake_minimum_required(VERSION 3.5)
project(tmc_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GripperApplyEffort.action"
  "action/GripperSetDistance.action"
  "msg/ExxxDriveMode.msg"
  "msg/JointExxxDriveMode.msg"
  "msg/OverloadJointState.msg"
  "msg/ServoParam.msg"
  "msg/ServoState.msg"
  "srv/ChangeExxxDriveMode.srv"
  "srv/InitActuatorEncorder.srv"
  "srv/ReadParameters.srv"
  "srv/WriteParameters.srv"
  "srv/SetBumperMode.srv"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
