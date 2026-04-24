// Copyright (c) 2026 Toyota Motor Corporation
#include <tmc_joint_command_controller/sources/joint_holding.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_utils/parameters.hpp>

#include "../common.hpp"

namespace tmc_joint_command_controller {

std::vector<std::string> PositionHolding::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandPositionInterfaces(command_interfaces);
  return command_interfaces;
}

std::vector<std::string> PositionHolding::GetStateInterfaces() const {
  std::vector<std::string> state_interfaces;
  joints_info_->AddStatePositionInterfaces(state_interfaces);
  return state_interfaces;
}

void PositionHolding::ReadAndUpdate(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  if (open_loop_control_ && previous_desired_state.positions.size() == joints_info_->names().size()) {
    desired_state_.positions = previous_desired_state.positions;
  } else {
    desired_state_.positions.clear();
    for (const auto& index : position_state_indices_) {
      desired_state_.positions.push_back(accessor_->GetState(index));
    }
  }
}

void PositionHolding::WriteCommand() {
  for (auto i = 0u; i < joints_info_->names().size(); ++i) {
    accessor_->SetCommand(position_command_indices_[i], desired_state_.positions[i]);
  }
}

trajectory_msgs::msg::JointTrajectoryPoint PositionHolding::GetDesiredState() const {
  return desired_state_;
}

bool PositionHolding::ConfigureImpl() {
  UpdatePriority(std::numeric_limits<int32_t>::min() + 1);  // 最低優先度よりは高くしておく
  open_loop_control_ = tmc_utils::GetParameter<bool>(node_, source_name_ + ".open_loop_control", true);
  return true;
}

bool PositionHolding::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  position_command_indices_ = GetPositionInterfaceIndices(command_interfaces, joints_info_->command_joints());
  position_state_indices_ = GetPositionInterfaceIndices(state_interfaces, joints_info_->names());
  return true;
}

std::vector<std::string> VelocityHolding::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandVelocityInterfaces(command_interfaces);
  return command_interfaces;
}

void VelocityHolding::WriteCommand() {
  for (const auto& index : velocity_command_indices_) {
    accessor_->SetCommand(index, 0.0);
  }
}

trajectory_msgs::msg::JointTrajectoryPoint VelocityHolding::GetDesiredState() const {
  trajectory_msgs::msg::JointTrajectoryPoint desired;
  desired.velocities.resize(joints_info_->names().size());
  std::fill(desired.velocities.begin(), desired.velocities.end(), 0.0);
  return desired;
}

bool VelocityHolding::ConfigureImpl() {
  UpdatePriority(std::numeric_limits<int32_t>::min() + 1);  // 最低優先度よりは高くしておく
  return true;
}

bool VelocityHolding::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    [[maybe_unused]] const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  velocity_command_indices_ = GetVelocityInterfaceIndices(command_interfaces, joints_info_->command_joints());
  return true;
}


std::vector<std::string> PositionVelocityHolding::GetCommandInterfaces() const {
  auto command_interfaces = PositionHolding::GetCommandInterfaces();
  joints_info_->AddCommandVelocityInterfaces(command_interfaces);
  return command_interfaces;
}

void PositionVelocityHolding::ReadAndUpdate(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  PositionHolding::ReadAndUpdate(time, period, previous_desired_state);
  desired_state_.velocities.resize(joints_info_->names().size(), 0.0);
}


void PositionVelocityHolding::WriteCommand() {
  PositionHolding::WriteCommand();
  for (const auto& index : velocity_command_indices_) {
    accessor_->SetCommand(index, 0.0);
  }
}

bool PositionVelocityHolding::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  if (!PositionHolding::ActivateImpl(command_interfaces, state_interfaces)) {
    return false;
  }
  velocity_command_indices_ = GetVelocityInterfaceIndices(command_interfaces, joints_info_->command_joints());
  return true;
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::PositionHolding,
    tmc_joint_command_controller::IJointCommandSource)

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::VelocityHolding,
    tmc_joint_command_controller::IJointCommandSource)

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::PositionVelocityHolding,
    tmc_joint_command_controller::IJointCommandSource)
