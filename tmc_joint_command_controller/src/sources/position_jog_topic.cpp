// Copyright (c) 2026 Toyota Motor Corporation
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
  // 時間で区切るのでなく，"指令した位置に到達したら"という条件が理想だが，難しいので時間で区切る
  // 到達を待ちたいので，デフォルト値は大きめに設定
  const auto command_timeout = tmc_utils::GetParameter<double>(node_, source_name_ + ".command_timeout", 0.5);
  if (command_timeout < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR(node_->get_logger(), "Command timeout must be positive. Given: %f", command_timeout);
    return false;
  }
  command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

  // この0は ros2_control/hardware_interface/include/mock_components/generic_system.hpp に合わせている
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
