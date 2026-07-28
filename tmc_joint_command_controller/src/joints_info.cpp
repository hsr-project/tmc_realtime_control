/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
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
