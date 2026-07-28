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
#include "generic_system.hpp"

namespace {
const char* const kCommandDriveMode = "command_drive_mode";
}  /* namespace */

namespace tmc_hardware_interface {

std::vector<hardware_interface::InterfaceDescription> GenericSystem::export_unlisted_command_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    hardware_interface::InterfaceInfo info;
    info.name = kCommandDriveMode;
    command_interfaces.emplace_back(hardware_interface::InterfaceDescription(joint_name, info));
  }

  return command_interfaces;
}

hardware_interface::return_type GenericSystem::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) {
  // The original only checks for duplicates in command_interfaces_, so it's fine to just return OK without doing anything
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type GenericSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (const auto & key : start_interfaces) {
    auto joint_it_found = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info_.joints.end()) {
      const size_t joint_index = static_cast<size_t>(std::distance(info_.joints.begin(), joint_it_found));
      auto inteface_name = info_.joints[joint_index].name + "/" + kCommandDriveMode;
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION) {
        set_command(inteface_name, static_cast<double>(mock_components::POSITION_INTERFACE_INDEX));
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        set_command(inteface_name, static_cast<double>(mock_components::VELOCITY_INTERFACE_INDEX));
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION) {
        set_command(inteface_name, static_cast<double>(mock_components::ACCELERATION_INTERFACE_INDEX));
      }
    }
  }
  return mock_components::GenericSystem::perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type GenericSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Call perform_command_mode_switch to update the private joint_control_mode_
  // However, it is only valid when calculate_dynamics_ is true
  std::vector<std::string> start_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto inteface_name = info_.joints[i].name + "/" + kCommandDriveMode;
    const auto command_drive_mode_int = static_cast<size_t>(get_command(inteface_name));
    if (command_drive_mode_int == mock_components::POSITION_INTERFACE_INDEX) {
      start_interfaces.emplace_back(info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION);
    } else if (command_drive_mode_int == mock_components::VELOCITY_INTERFACE_INDEX) {
      start_interfaces.emplace_back(info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY);
    } else if (command_drive_mode_int == mock_components::ACCELERATION_INTERFACE_INDEX) {
      start_interfaces.emplace_back(info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION);
    }
  }

  const auto result = perform_command_mode_switch(start_interfaces, {});
  if (result != hardware_interface::return_type::OK) {
    return result;
  }
  return mock_components::GenericSystem::read(time, period);
}

}  // namespace tmc_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tmc_hardware_interface::GenericSystem, hardware_interface::SystemInterface)
