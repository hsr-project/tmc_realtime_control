// Copyright (c) 2026 Toyota Motor Corporation
#include <tmc_joint_command_controller/realtime_trajectory.hpp>

#include <tmc_utils/parameters.hpp>

#include "common.hpp"

namespace {

// joint_trajectoryからjoint_namesで指定されている関節だけを取り出したjoint_trajectoryを取得．
// joint_trajectoryに存在しない関節はinitial_pointより取得．
trajectory_msgs::msg::JointTrajectory FillAndSortTrajectory(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    const std::vector<std::string>& joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint& initial_point) {
  uint32_t num_points = joint_trajectory.points.size();
  uint32_t num_joints = joint_names.size();
  // joint_trajectoryの対応番号を作る -1は対応なし
  std::vector<int32_t> index_map(num_joints);
  for (unsigned int joint_index = 0; joint_index < num_joints; ++joint_index) {
    index_map[joint_index] = -1;
    for (unsigned int input_index = 0; input_index < joint_trajectory.joint_names.size(); ++input_index) {
      if (joint_trajectory.joint_names[input_index] == joint_names[joint_index]) {
        index_map[joint_index] = input_index;
      }
    }
  }

  trajectory_msgs::msg::JointTrajectory partial_joint_trajectory_out;
  partial_joint_trajectory_out.header = joint_trajectory.header;
  partial_joint_trajectory_out.joint_names = joint_names;

  partial_joint_trajectory_out.points.resize(num_points);
  for (unsigned int point_index = 0; point_index < num_points; ++point_index) {
    auto& point_out = partial_joint_trajectory_out.points[point_index];
    point_out.positions.resize(num_joints);

    auto& point_in = joint_trajectory.points[point_index];
    point_out.time_from_start = point_in.time_from_start;

    bool has_velocities = !(point_in.velocities.empty());
    if (has_velocities) point_out.velocities.resize(num_joints);

    bool has_accelerations = !(point_in.accelerations.empty());
    if (has_accelerations) point_out.accelerations.resize(num_joints);

    for (unsigned int joint_index = 0; joint_index < num_joints; ++joint_index) {
      if (index_map[joint_index] != -1) {
        point_out.positions[joint_index] = point_in.positions[index_map[joint_index]];
        if (has_velocities) {
          point_out.velocities[joint_index] = point_in.velocities[index_map[joint_index]];
        }
        if (has_accelerations) {
          point_out.accelerations[joint_index] = point_in.accelerations[index_map[joint_index]];
        }
      } else {
        point_out.positions[joint_index] = initial_point.positions[joint_index];
        if (has_velocities) {
          point_out.velocities[joint_index] = initial_point.velocities[joint_index];
        }
        if (has_accelerations) {
          point_out.accelerations[joint_index] = initial_point.accelerations[joint_index];
        }
      }
    }
  }
  return partial_joint_trajectory_out;
}

trajectory_msgs::msg::JointTrajectoryPoint ComputeError(
    const trajectory_msgs::msg::JointTrajectoryPoint& desired,
    const trajectory_msgs::msg::JointTrajectoryPoint& actual) {
  const size_t num_joints = desired.positions.size();

  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions.resize(num_joints);
  error.velocities.resize(num_joints, 0.0);
  error.accelerations.resize(num_joints, 0.0);
  for (size_t i = 0; i < num_joints; ++i) {
    error.positions[i] = desired.positions[i] - actual.positions[i];
    if (!desired.velocities.empty() && !actual.velocities.empty()) {
      error.velocities[i] = desired.velocities[i] - actual.velocities[i];
    }
    if (!desired.accelerations.empty() && !actual.accelerations.empty()) {
      error.accelerations[i] = desired.accelerations[i] - actual.accelerations[i];
    }
  }
  return error;
}

}  // namespace

namespace tmc_joint_command_controller {

std::vector<std::string> RealtimeTrajectory::GetStateInterfaces() const {
  std::vector<std::string> state_interfaces;
  joints_info_->AddStatePositionInterfaces(state_interfaces);
  if (use_velocity_state_) {
    joints_info_->AddStateVelocityInterfaces(state_interfaces);
  }
  return state_interfaces;
}

bool RealtimeTrajectory::Configure(const JointsInfo::Ptr& joints_info) {
  joints_info_ = joints_info;

  open_loop_control_ = tmc_utils::GetParameter<bool>(node_, parameter_ns_ + ".open_loop_control", true);
  use_velocity_state_ = tmc_utils::GetParameter<bool>(node_, parameter_ns_ + ".use_velocity_state", true);

  default_tolerances_ = GetSegmentTolerances(node_, joints_info_->names());

  return true;
}

bool RealtimeTrajectory::Activate(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  ActivateInterfaces(state_interfaces);
  ActivateTrajectory();

  has_last_command_state_ = false;
  return true;
}

void RealtimeTrajectory::ActivateInterfaces(
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  position_state_indices_ = GetPositionInterfaceIndices(state_interfaces, joints_info_->names());
  if (use_velocity_state_) {
    velocity_state_indices_ = GetVelocityInterfaceIndices(state_interfaces, joints_info_->names());
  }
}

void RealtimeTrajectory::ActivateTrajectory() {
  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;

  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  active_tolerances_.writeFromNonRT(default_tolerances_);
}

void RealtimeTrajectory::WriteFromNonRT(const trajectory_msgs::msg::JointTrajectory& msg) {
  auto trajectory_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(msg);

  auto command_received_time = rclcpp::Time(trajectory_ptr->header.stamp);
  if (command_received_time.nanoseconds() == 0) {
    command_received_time = node_->now();
  }

  trajectory_msg_buffer_.writeFromNonRT(trajectory_ptr);
}

void RealtimeTrajectory::WriteFromNonRT(const control_msgs::action::FollowJointTrajectory::Goal& msg) {
  auto logger = node_->get_logger();
  active_tolerances_.writeFromNonRT(GetSegmentTolerances(logger, default_tolerances_, msg, joints_info_->names()));
  WriteFromNonRT(msg.trajectory);
}

void RealtimeTrajectory::PreemptFromNonRT(bool clear_last_command) {
  if (clear_last_command) {
    has_last_command_state_ = false;
  }
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>());
}

trajectory_msgs::msg::JointTrajectoryPoint RealtimeTrajectory::GetInitialStateForSampling(
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) const {
  trajectory_msgs::msg::JointTrajectoryPoint initial_point;
  if (open_loop_control_ && has_last_command_state_) {
    initial_point = sampled_state_.desired;
  } else if (open_loop_control_ && previous_desired_state.positions.size() == joints_info_->names().size()) {
    initial_point.positions = previous_desired_state.positions;
    if (use_velocity_state_ && previous_desired_state.velocities.size() == joints_info_->names().size()) {
      initial_point.velocities = previous_desired_state.velocities;
    } else if (use_velocity_state_) {
      initial_point.velocities = GetCurrentStateFromRT().velocities;
    }
  } else {
    initial_point = GetCurrentStateFromRT();
  }
  return initial_point;
}

RealtimeTrajectory::SampleResult RealtimeTrajectory::SampleFromRT(
    const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  // 軌道があるかのチェック
  auto current_msg = trajectory_ptr_->get_trajectory_msg();
  auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    auto filled_msg_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
        FillAndSortTrajectory(**new_msg, joints_info_->names(), GetInitialStateForSampling(previous_desired_state)));
    trajectory_msg_buffer_.writeFromNonRT(filled_msg_ptr);
    trajectory_ptr_->update(filled_msg_ptr);
  }
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg() ||
      (*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    last_sampled_time_ = time;
    return {ErrorCode::kEmptyTrajectory, sampled_state_};
  }

  // 目標位置のサンプル
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    (*trajectory_active_ptr_)->set_point_before_trajectory_msg(
        time, GetInitialStateForSampling(previous_desired_state));
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  const bool valid_point = (*trajectory_active_ptr_)->sample(
      time, joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
      sampled_state_.desired, start_segment_it, end_segment_it);
  if (!valid_point) {
    // sampleの実装的には，有効なtrajectoryが入っていればtrueを返すはずなので，ここに来るのはおかしい
    RCLCPP_ERROR(node_->get_logger(), "Sampling trajectory failed unexpectedly.");
    return {ErrorCode::kEmptyTrajectory, sampled_state_};
  }

  has_last_command_state_ = true;
  last_sampled_time_ = time;

  // 成否判定
  // TODO(Takeshita) 長いので分割するか？
  sampled_state_.actual = GetCurrentStateFromRT();
  sampled_state_.error = ComputeError(sampled_state_.desired, sampled_state_.actual);
  const auto active_tolerances = *active_tolerances_.readFromRT();

  const auto before_last_point = end_segment_it != (*trajectory_active_ptr_)->end();
  if (before_last_point) {
    // 軌道追従中なので，経路がズレていないかチェックするだけ
    for (uint32_t i = 0; i < active_tolerances.state_tolerance.size(); ++i) {
      if (!CheckStateTolerancePerJoint(sampled_state_.error, i, active_tolerances.state_tolerance[i])) {
        RCLCPP_ERROR(node_->get_logger(), "Path tolerance violated.");
        return {ErrorCode::kPathToleranceViolated, sampled_state_};
      }
    }
  } else {
    // ゴールしたかのチェック，時間内なら何もせずに待つ
    bool abort = false;
    for (uint32_t i = 0; i < active_tolerances.goal_state_tolerance.size(); ++i) {
      if (!CheckStateTolerancePerJoint(sampled_state_.error, i, active_tolerances.goal_state_tolerance[i])) {
        abort = true;
        break;
      }
    }
    if (!abort) {
      return {ErrorCode::kSuccessful, sampled_state_};
    } else if (active_tolerances.goal_time_tolerance != 0.0) {
      // 0.0との!=は危険だが，デフォルト値が0.0なので，これでいく
      const rclcpp::Time start_stamp = (*trajectory_active_ptr_)->time_from_start();
      const rclcpp::Time end_stamp = start_stamp + start_segment_it->time_from_start;
      const double time_from_point = time.seconds() - end_stamp.seconds();
      if (time_from_point > active_tolerances.goal_time_tolerance) {
        RCLCPP_ERROR(node_->get_logger(), "Goal tolerance violated.");
        return {ErrorCode::kGoalToleranceViolated, sampled_state_};
      }
    }
  }
  return {ErrorCode::kSampled, sampled_state_};
}

trajectory_msgs::msg::JointTrajectoryPoint RealtimeTrajectory::GetCurrentStateFromRT() const {
  trajectory_msgs::msg::JointTrajectoryPoint current_state;
  for (auto i = 0u; i < joints_info_->names().size(); ++i) {
    current_state.positions.push_back(accessor_->GetState(position_state_indices_[i]));
    if (use_velocity_state_) {
      current_state.velocities.push_back(accessor_->GetState(velocity_state_indices_[i]));
    }
  }
  return current_state;
}

bool RealtimeTrajectory::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& msg) const {
  // エラーメッセージはjoint_trajectory_controller.cppにあわせた

  // 以下で実装している，必要ならパラメータにする
  // allow_partial_joints_goal: true
  // allow_nonzero_velocity_at_trajectory_end: true
  // allow_integration_in_goal_trajectories: false

  if (msg.joint_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  if (msg.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Empty trajectory received.");
    return false;
  }

  const auto trajectory_start_time = rclcpp::Time(msg.header.stamp);
  if (trajectory_start_time.nanoseconds() != 0) {
    const auto trajectory_end_time = trajectory_start_time + msg.points.back().time_from_start;
    if (trajectory_end_time < node_->now()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Received trajectory with non-zero start time (%f) that ends in the past (%f)",
                   trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  const auto& controller_joints = joints_info_->names();
  for (const auto& incoming_joint_name : msg.joint_names) {
    auto it = std::find(controller_joints.begin(), controller_joints.end(), incoming_joint_name);
    if (it == controller_joints.end()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Incoming joint %s doesn't match the controller's joints.",
                   incoming_joint_name.c_str());
      return false;
    }
  }

  const auto incoming_dof = msg.joint_names.size();
  auto validate_point = [&] (const std::vector<double>& values,
                             const std::string& name,
                             size_t index,
                             bool allow_empty) {
    if (allow_empty && values.empty()) {
      return true;
    }
    if (values.size() != incoming_dof) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Mismatch between joint_names size (%zu) and %s (%zu) at point #%zu.",
                   incoming_dof, name.c_str(), values.size(), index);
      return false;
    }
    return true;
  };

  auto previous_time = rclcpp::Duration(0, 0);
  for (auto i = 0u; i < msg.points.size(); ++i) {
    if ((i > 0) && (rclcpp::Duration(msg.points[i].time_from_start) <= previous_time)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Time between points %u and %u is not strictly increasing, it is %f and %f respectively",
                   i - 1, i, previous_time.seconds(),
                   rclcpp::Duration(msg.points[i].time_from_start).seconds());
      return false;
    }
    previous_time = msg.points[i].time_from_start;

    if (!validate_point(msg.points[i].positions, "positions", i, false) ||
        !validate_point(msg.points[i].velocities, "velocities", i, true) ||
        !validate_point(msg.points[i].accelerations, "accelerations", i, true)) {
      return false;
    }
    if (!msg.points[i].effort.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectories with effort fields are currently not supported.");
      return false;
    }
  }
  return true;
}

}  // namespace tmc_joint_command_controller
