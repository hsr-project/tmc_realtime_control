// Copyright (c) 2026 Toyota Motor Corporation
#include <tmc_joint_command_controller/joints_info.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_utils/parameters.hpp>

#include "common.hpp"

namespace {

bool ContainsEmptyString(const std::vector<std::string>& vec) {
  for (const auto& str : vec) {
    if (str.empty()) {
      return true;
    }
  }
  return false;
}

bool ContainsDuplicate(const std::vector<std::string>& vec) {
  std::set<std::string> unique_set(vec.begin(), vec.end());
  return unique_set.size() != vec.size();
}

}  // namespace

namespace tmc_joint_command_controller {

JointsInfo::JointsInfo(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  names_ = tmc_utils::GetParameter<std::vector<std::string>>(node.get(), "joints", {});
  command_joints_ = tmc_utils::GetParameter<std::vector<std::string>>(node.get(), "command_joints", names_);
}

bool JointsInfo::IsValid(const rclcpp::Logger& logger) const {
  if (names_.empty()) {
    RCLCPP_ERROR(logger, "The number of joints must be greater than zero.");
    return false;
  }
  if (command_joints_.size() != names_.size()) {
    RCLCPP_ERROR(logger, "The number of command_joints (%zu) must be the same as the number of joints (%zu).",
                 command_joints_.size(), names_.size());
    return false;
  }
  if (ContainsEmptyString(names_)) {
    RCLCPP_ERROR(logger, "Joint names must not contain empty strings.");
    return false;
  }
  if (ContainsEmptyString(command_joints_)) {
    RCLCPP_ERROR(logger, "Command joint names must not contain empty strings.");
    return false;
  }
  if (ContainsDuplicate(names_)) {
    RCLCPP_ERROR(logger, "Joint names must not contain duplicates.");
    return false;
  }
  if (ContainsDuplicate(command_joints_)) {
    RCLCPP_ERROR(logger, "Command joint names must not contain duplicates.");
    return false;
  }
  return true;
}

void JointsInfo::AddCommandPositionInterfaces(std::vector<std::string>& command_interfaces) const {
  AddInterfaces(command_joints_, hardware_interface::HW_IF_POSITION, command_interfaces);
}

void JointsInfo::AddCommandVelocityInterfaces(std::vector<std::string>& command_interfaces) const {
  AddInterfaces(command_joints_, hardware_interface::HW_IF_VELOCITY, command_interfaces);
}

void JointsInfo::AddStatePositionInterfaces(std::vector<std::string>& state_interfaces) const {
  AddInterfaces(names_, hardware_interface::HW_IF_POSITION, state_interfaces);
}

void JointsInfo::AddStateVelocityInterfaces(std::vector<std::string>& state_interfaces) const {
  AddInterfaces(names_, hardware_interface::HW_IF_VELOCITY, state_interfaces);
}

void JointsInfo::AddStateEffortInterfaces(std::vector<std::string>& state_interfaces) const {
  AddInterfaces(names_, hardware_interface::HW_IF_EFFORT, state_interfaces);
}

}  // namespace tmc_joint_command_controller
