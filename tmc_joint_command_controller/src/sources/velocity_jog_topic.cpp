// Copyright (c) 2026 Toyota Motor Corporation
#include <tmc_joint_command_controller/sources/velocity_jog_topic.hpp>

#include <tmc_utils/parameters.hpp>

#include "../common.hpp"

namespace tmc_joint_command_controller {

VelocityJogSubscription::VelocityJogSubscription(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& joint_names)
    : RealtimeSubscriptionBase<control_msgs::msg::JointJog>(node, "~/joint_velocity"),
      joint_names_(joint_names) {}

void VelocityJogSubscription::Callback(const control_msgs::msg::JointJog::SharedPtr msg) {
  if (!ValidateJogCommand(node(), msg->joint_names, msg->velocities, joint_names_)) {
    return;
  }

  auto modified_msg = *msg;
  UpdateStamp(node(), modified_msg.header.stamp);

  Write(modified_msg);
}


std::vector<std::string> VelocityJogTopic::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandVelocityInterfaces(command_interfaces);
  return command_interfaces;
}

std::vector<std::string> VelocityJogTopic::GetStateInterfaces() const {
  return {};
}

void VelocityJogTopic::ReadAndUpdate(
    const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    [[maybe_unused]] const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  const auto command = *realtime_sub_->Read();
  velocity_command_.clear();

  // 一定時刻経過の場合，指令値が途絶えたものとする
  if ((time - rclcpp::Time(command.header.stamp, time.get_clock_type())) > command_timeout_) {
    return;
  }
  if (command.joint_names.empty()) {
    return;
  }

  for (auto i = 0u; i < joints_info_->names().size(); ++i) {
    const auto it = std::find(command.joint_names.begin(), command.joint_names.end(), joints_info_->names()[i]);
    if (it != command.joint_names.end()) {
      velocity_command_.push_back(command.velocities[std::distance(command.joint_names.begin(), it)]);
    } else {
      velocity_command_.push_back(0.0);
    }
  }

  UpdateLastCommandTime(command.header.stamp);
}

bool VelocityJogTopic::HasCommand() const {
  return !velocity_command_.empty();
}

void VelocityJogTopic::WriteCommand() {
  for (auto i = 0u; i < velocity_command_.size(); ++i) {
    accessor_->SetCommand(velocity_command_indices_[i], velocity_command_[i]);
  }
}

trajectory_msgs::msg::JointTrajectoryPoint VelocityJogTopic::GetDesiredState() const {
  trajectory_msgs::msg::JointTrajectoryPoint desired;
  desired.velocities = velocity_command_;
  return desired;
}

void VelocityJogTopic::Preempt() {
  velocity_command_.clear();
  realtime_sub_->ResetBuffer();
}

bool VelocityJogTopic::ConfigureImpl() {
  const auto command_timeout = tmc_utils::GetParameter<double>(node_, source_name_ + ".command_timeout", 0.1);
  if (command_timeout < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR(node_->get_logger(), "Command timeout must be positive. Given: %f", command_timeout);
    return false;
  }
  command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

  // この 1 は ros2_control/hardware_interface/include/mock_components/generic_system.hpp に合わせている
  UpdateTargetControlMode(GetTargetControlModeFromParameters(node_, joints_info_->names(), source_name_, 1));

  realtime_sub_ = std::make_shared<VelocityJogSubscription>(node_, joints_info_->names());

  return true;
}

bool VelocityJogTopic::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    [[maybe_unused]] const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  velocity_command_indices_ = GetVelocityInterfaceIndices(command_interfaces, joints_info_->command_joints());

  realtime_sub_->ResetBuffer();
  return true;
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::VelocityJogTopic,
    tmc_joint_command_controller::IJointCommandSource)
