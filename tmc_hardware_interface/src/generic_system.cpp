// Copyright (c) 2026 Toyota Motor Corporation
#include "generic_system.hpp"

namespace tmc_hardware_interface {

std::vector<hardware_interface::CommandInterface> GenericSystem::export_command_interfaces() {
  auto command_interfaces = mock_components::GenericSystem::export_command_interfaces();

  command_drive_modes_.resize(info_.joints.size(), 0.0);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name, "command_drive_mode", &command_drive_modes_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type GenericSystem::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) {
  // 本家はcommand_interfaces_の重複チェックをしているだけなので，何もせずにOKを返す
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
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION) {
        command_drive_modes_[joint_index] = static_cast<double>(mock_components::POSITION_INTERFACE_INDEX);
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        command_drive_modes_[joint_index] = static_cast<double>(mock_components::VELOCITY_INTERFACE_INDEX);
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION) {
        command_drive_modes_[joint_index] = static_cast<double>(mock_components::ACCELERATION_INTERFACE_INDEX);
      }
    }
  }
  return mock_components::GenericSystem::perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type GenericSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // privateなjoint_control_mode_を更新するために、perform_command_mode_switchを呼び出す
  // ただしcalculate_dynamics_がtrueのときのみ有効
  std::vector<std::string> start_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto command_drive_mode_int = static_cast<size_t>(command_drive_modes_[i]);
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
