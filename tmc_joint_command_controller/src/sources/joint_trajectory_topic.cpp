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
#include <tmc_joint_command_controller/sources/joint_trajectory_topic.hpp>

#include <tmc_utils/parameters.hpp>

#include "../common.hpp"

namespace tmc_joint_command_controller {

std::vector<std::string> JointTrajectoryTopic::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandPositionInterfaces(command_interfaces);
  return command_interfaces;
}

std::vector<std::string> JointTrajectoryTopic::GetStateInterfaces() const {
  return trajectory_->GetStateInterfaces();
}

void JointTrajectoryTopic::ReadAndUpdate(
    const rclcpp::Time& time,
    const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  UpdatePriority(priority_policy_->GetDefaultPriority());

  desired_state_ = std::nullopt;

  const auto [error_code, sampled_state] = trajectory_->SampleFromRT(time, period, previous_desired_state);
  if (error_code != RealtimeTrajectory::ErrorCode::kSampled) {
    return;
  }

  desired_state_ = sampled_state.desired;
  const auto command_received_time_buffer = *command_received_time_buffer_.readFromRT();
  UpdateLastCommandTime(command_received_time_buffer);
  UpdatePriority(priority_policy_->GetPriority(time, command_received_time_buffer));
}

void JointTrajectoryTopic::WriteCommand() {
  for (size_t i = 0; i < desired_state_.value().positions.size(); ++i) {
    accessor_->SetCommand(position_command_indices_[i], desired_state_.value().positions[i]);
  }
}

void JointTrajectoryTopic::Preempt() {
  trajectory_->PreemptFromNonRT();
  desired_state_ = std::nullopt;
}

bool JointTrajectoryTopic::InitImpl() {
  trajectory_ = std::make_shared<RealtimeTrajectory>(node_, source_name_, accessor_);
  return true;
}

bool JointTrajectoryTopic::ConfigureImpl() {
  if (!trajectory_->Configure(joints_info_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure trajectory.");
    return false;
  }
  priority_policy_ = std::make_shared<TrajectoryPriorityPolicy>(node_, source_name_, GetPriority());

  // This 0 is aligned with ros2_control/hardware_interface/include/mock_components/generic_system.hpp
  UpdateTargetControlMode(GetTargetControlModeFromParameters(node_, joints_info_->names(), source_name_, 0));

  const auto topic_name = tmc_utils::GetParameter<std::string>(
      node_, source_name_ + ".topic_name", "~/joint_trajectory");
  sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      topic_name, rclcpp::QoS(1), std::bind(&JointTrajectoryTopic::Callback, this, std::placeholders::_1));

  return true;
}

bool JointTrajectoryTopic::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  position_command_indices_ = GetPositionInterfaceIndices(command_interfaces, joints_info_->command_joints());

  if (!trajectory_->Activate(state_interfaces)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to activate trajectory.");
    return false;
  }
  return true;
}

void JointTrajectoryTopic::Callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  if (!trajectory_->ValidateTrajectory(*msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Received invalid trajectory message.");
    return;
  }

  trajectory_->WriteFromNonRT(*msg);

  auto command_received_time = rclcpp::Time(msg->header.stamp);
  if (command_received_time.nanoseconds() == 0) {
    command_received_time = node_->now();
  }
  command_received_time_buffer_.writeFromNonRT(command_received_time);
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::JointTrajectoryTopic,
    tmc_joint_command_controller::IJointCommandSource)
