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
#include <tmc_joint_command_controller/sources/position_jog_topic.hpp>

#include <tmc_utils/parameters.hpp>

#include "../common.hpp"

namespace tmc_joint_command_controller {

PositionJogSubscription::PositionJogSubscription(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& joint_names)
    : RealtimeSubscriptionBase<control_msgs::msg::JointJog>(node, "~/joint_position"),
      joint_names_(joint_names) {}

void PositionJogSubscription::Callback(const control_msgs::msg::JointJog::SharedPtr msg) {
  if (!ValidateJogCommand(node(), msg->joint_names, msg->displacements, joint_names_)) {
    return;
  }

  auto modified_msg = *msg;
  UpdateStamp(node(), modified_msg.header.stamp);

  Write(modified_msg);
}


std::vector<std::string> PositionJogTopic::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandPositionInterfaces(command_interfaces);
  return command_interfaces;
}

std::vector<std::string> PositionJogTopic::GetStateInterfaces() const {
  std::vector<std::string> state_interfaces;
  joints_info_->AddStatePositionInterfaces(state_interfaces);
  return state_interfaces;
}

void PositionJogTopic::ReadAndUpdate(
    const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    [[maybe_unused]] const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  const auto command = *realtime_sub_->Read();
  position_command_.clear();

  if ((time - rclcpp::Time(command.header.stamp, time.get_clock_type())) > command_timeout_) {
    return;
  }
  if (command.joint_names.empty()) {
    return;
  }

  for (auto i = 0u; i < joints_info_->names().size(); ++i) {
    const auto it = std::find(command.joint_names.begin(), command.joint_names.end(), joints_info_->names()[i]);
    if (it != command.joint_names.end()) {
      position_command_.push_back(command.displacements[std::distance(command.joint_names.begin(), it)]);
    } else {
      position_command_.push_back(accessor_->GetState(position_state_indices_[i]));
    }
  }

  UpdateLastCommandTime(command.header.stamp);
}

bool PositionJogTopic::HasCommand() const {
  return !position_command_.empty();
}

void PositionJogTopic::WriteCommand() {
  for (auto i = 0u; i < position_command_.size(); ++i) {
    accessor_->SetCommand(position_command_indices_[i], position_command_[i]);
  }
}

trajectory_msgs::msg::JointTrajectoryPoint PositionJogTopic::GetDesiredState() const {
  trajectory_msgs::msg::JointTrajectoryPoint desired;
  desired.positions = position_command_;
  return desired;
}

void PositionJogTopic::Preempt() {
  position_command_.clear();
  realtime_sub_->ResetBuffer();
}

bool PositionJogTopic::ConfigureImpl() {
  // Ideally, the condition would be "when the instructed position is reached" instead of dividing by time, but since it's difficult, we divide by time
  // Since we want to wait for arrival, the default value is set to be relatively large
  const auto command_timeout = tmc_utils::GetParameter<double>(node_, source_name_ + ".command_timeout", 0.5);
  if (command_timeout < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR(node_->get_logger(), "Command timeout must be positive. Given: %f", command_timeout);
    return false;
  }
  command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

  // This 0 is aligned with ros2_control/hardware_interface/include/mock_components/generic_system.hpp
  UpdateTargetControlMode(GetTargetControlModeFromParameters(node_, joints_info_->names(), source_name_, 0));

  realtime_sub_ = std::make_shared<PositionJogSubscription>(node_, joints_info_->names());

  return true;
}

bool PositionJogTopic::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  position_command_indices_ = GetPositionInterfaceIndices(command_interfaces, joints_info_->command_joints());
  position_state_indices_ = GetPositionInterfaceIndices(state_interfaces, joints_info_->names());

  realtime_sub_->ResetBuffer();
  return true;
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::PositionJogTopic,
    tmc_joint_command_controller::IJointCommandSource)
