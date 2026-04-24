// Copyright (c) 2026 Toyota Motor Corporation
#include <tmc_joint_command_controller/sources/joint_trajectory_action.hpp>

#include <rclcpp_action/create_server.hpp>

#include <tmc_utils/parameters.hpp>

#include "../common.hpp"

namespace tmc_joint_command_controller {

std::vector<std::string> JointTrajectoryAction::GetCommandInterfaces() const {
  std::vector<std::string> command_interfaces;
  joints_info_->AddCommandPositionInterfaces(command_interfaces);
  return command_interfaces;
}

std::vector<std::string> JointTrajectoryAction::GetStateInterfaces() const {
  return trajectory_->GetStateInterfaces();
}

void JointTrajectoryAction::Preempt() {
  ActionServerBase::Preempt();
  trajectory_->PreemptFromNonRT(true);
}

void JointTrajectoryAction::PreemptActiveGoal() {
  ActionServerBase::PreemptActiveGoal();
  trajectory_->PreemptFromNonRT(false);
}

void JointTrajectoryAction::CancelActiveGoal(RealtimeGoalHandlePtr active_goal) {
  ActionServerBase::CancelActiveGoal(active_goal);
  trajectory_->PreemptFromNonRT(true);
}

void JointTrajectoryAction::OnNoGoal(const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period) {
  trajectory_->UpdateLastSampledTimeFromRT(time);
  UpdatePriority(priority_policy_->GetDefaultPriority());
}

bool JointTrajectoryAction::OnActiveGoal(
    const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  UpdatePriority(priority_policy_->GetDefaultPriority());

  const auto [error_code, sampled_state] = trajectory_->SampleFromRT(time, period, previous_desired_state);
  if (error_code == RealtimeTrajectory::ErrorCode::kSampled) {
    desired_state_ = sampled_state.desired;

    auto feedback = std::make_shared<ActionType::Feedback>();
    feedback->desired = sampled_state.desired;
    feedback->actual = sampled_state.actual;
    feedback->error = sampled_state.error;
    SetFeedback(feedback);

    UpdatePriority(priority_policy_->GetPriority(time, GetCommandReceivedTime()));
    return true;
  } else if (error_code == RealtimeTrajectory::ErrorCode::kEmptyTrajectory) {
    return false;
  } else if (error_code == RealtimeTrajectory::ErrorCode::kSuccessful) {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
    SetSucceeded(result);
    return false;
  } else {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(static_cast<int32_t>(error_code));
    SetAborted(result);
    return false;
  }
}

bool JointTrajectoryAction::ValidateGoal(const std::shared_ptr<const ActionType::Goal> goal) {
  return trajectory_->ValidateTrajectory(goal->trajectory);
}

void JointTrajectoryAction::ReceiveGoal(const std::shared_ptr<const ActionType::Goal> goal) {
  auto command_received_time = rclcpp::Time(goal->trajectory.header.stamp);
  if (command_received_time.nanoseconds() == 0) {
    command_received_time = node_->now();
  }
  WriteCommandReceivedTime(command_received_time);

  trajectory_->WriteFromNonRT(*goal);
}

void JointTrajectoryAction::WriteCommand() {
  for (size_t i = 0; i < desired_state_.positions.size(); ++i) {
    accessor_->SetCommand(position_command_indices_[i], desired_state_.positions[i]);
  }
}

bool JointTrajectoryAction::InitImpl() {
  trajectory_ = std::make_shared<RealtimeTrajectory>(node_, source_name_, accessor_);
  return true;
}

bool JointTrajectoryAction::ConfigureImpl() {
  if (!trajectory_->Configure(joints_info_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure trajectory.");
    return false;
  }

  // この0は ros2_control/hardware_interface/include/mock_components/generic_system.hpp に合わせている
  UpdateTargetControlMode(GetTargetControlModeFromParameters(node_, joints_info_->names(), source_name_, 0));

  return true;
}

bool JointTrajectoryAction::ActivateImpl(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  position_command_indices_ = GetPositionInterfaceIndices(command_interfaces, joints_info_->command_joints());

  if (!trajectory_->Activate(state_interfaces)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to activate trajectory.");
    return false;
  }
  priority_policy_ = std::make_shared<TrajectoryPriorityPolicy>(node_, source_name_, GetPriority());

  return true;
}

void JointTrajectoryAction::PreemptImpl() {
  const auto do_reset = HasActiveGoal();
  ActionServerBase::Preempt();
  if (do_reset) {
    trajectory_->PreemptFromNonRT(false);
  }
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::JointTrajectoryAction,
    tmc_joint_command_controller::IJointCommandSource)
