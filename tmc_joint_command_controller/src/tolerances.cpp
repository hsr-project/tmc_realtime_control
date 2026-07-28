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
#include <tmc_joint_command_controller/tolerances.hpp>

#include <limits>

#include <tmc_utils/parameters.hpp>

namespace {

std::optional<double> ResolveToleranceSource(const double default_value, const double goal_value) {
  // from https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/JointTolerance.msg
  // There are two special values for tolerances:
  // * 0 - The tolerance is unspecified and will remain at whatever the default is
  // * -1 - The tolerance is "erased".
  //        If there was a default, the joint will be allowed to move without restriction.
  constexpr double ERASE_VALUE = -1.0;

  if (goal_value > 0.0) {
    return goal_value;
  } else if (std::abs(goal_value - ERASE_VALUE) < std::numeric_limits<float>::epsilon()) {
    return 0.0;
  } else if (goal_value < 0.0) {
    return std::nullopt;
  }
  return default_value;
}

}  // namespace

namespace tmc_joint_command_controller {

SegmentTolerances GetSegmentTolerances(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& joint_names) {
  // The default value aligns with the implementation of the joint_trajectory_controller
  SegmentTolerances tolerances;
  tolerances.goal_time_tolerance = tmc_utils::GetParameter<double>(node, "constraints.goal_time", 0.0);

  const double stopped_velocity_tolerance = tmc_utils::GetParameter<double>(
      node, "constraints.stopped_velocity_tolerance", 0.01);
  for (const auto& joint_name : joint_names) {
    const std::string prefix = "constraints." + joint_name;

    StateTolerances state_tolerance;
    state_tolerance.position = tmc_utils::GetParameter<double>(node, prefix + ".trajectory", 0.0);
    tolerances.state_tolerance.push_back(state_tolerance);

    StateTolerances goal_state_tolerance;
    goal_state_tolerance.position = tmc_utils::GetParameter<double>(node, prefix + ".goal", 0.0);
    goal_state_tolerance.velocity = stopped_velocity_tolerance;
    tolerances.goal_state_tolerance.push_back(goal_state_tolerance);
  }
  return tolerances;
}

SegmentTolerances GetSegmentTolerances(
    rclcpp::Logger& logger,
    const SegmentTolerances& default_tolerances,
    const control_msgs::action::FollowJointTrajectory::Goal& goal,
    const std::vector<std::string>& joint_names) {
  SegmentTolerances active_tolerances(default_tolerances);

  const auto goal_time_opt = ResolveToleranceSource(
      default_tolerances.goal_time_tolerance, rclcpp::Duration(goal.goal_time_tolerance).seconds());
  if (goal_time_opt.has_value()) {
    active_tolerances.goal_time_tolerance = goal_time_opt.value();
  } else {
    RCLCPP_ERROR(
        logger, "Specified illegal goal_time_tolerance: %f. Using default tolerances",
        rclcpp::Duration(goal.goal_time_tolerance).seconds());
    return default_tolerances;
  }

  // State and goal state tolerances
  for (const auto& joint_tol : goal.path_tolerance) {
    const auto& joint = joint_tol.name;
    const auto it = std::find(joint_names.begin(), joint_names.end(), joint);
    if (it == joint_names.end()) {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.path_tolerance does not exist. Using default tolerances.",
                   joint.c_str());
      return default_tolerances;
    }
    auto i = static_cast<size_t>(std::distance(joint_names.cbegin(), it));

    auto& state_tolerance_in = default_tolerances.state_tolerance[i];
    auto& state_tolerance_out = active_tolerances.state_tolerance[i];

    const auto position_opt = ResolveToleranceSource(state_tolerance_in.position, joint_tol.position);
    if (position_opt.has_value()) {
      state_tolerance_out.position = position_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.path_tolerance has a invalid position tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }

    const auto velocity_opt = ResolveToleranceSource(state_tolerance_in.velocity, joint_tol.velocity);
    if (velocity_opt.has_value()) {
      state_tolerance_out.velocity = velocity_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.path_tolerance has a invalid velocity tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }

    const auto acceleration_opt = ResolveToleranceSource(state_tolerance_in.acceleration, joint_tol.acceleration);
    if (acceleration_opt.has_value()) {
      state_tolerance_out.acceleration = acceleration_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.path_tolerance has a invalid acceleration tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }
  }

  for (auto goal_tol : goal.goal_tolerance) {
    const auto& joint = goal_tol.name;
    const auto it = std::find(joint_names.begin(), joint_names.end(), joint);
    if (it == joint_names.end()) {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.goal_tolerance does not exist. Using default tolerances.",
                   joint.c_str());
      return default_tolerances;
    }
    auto i = static_cast<size_t>(std::distance(joint_names.cbegin(), it));

    auto& goal_state_tolerance_in = default_tolerances.goal_state_tolerance[i];
    auto& goal_state_tolerance_out = active_tolerances.goal_state_tolerance[i];

    const auto position_opt = ResolveToleranceSource(goal_state_tolerance_in.position, goal_tol.position);
    if (position_opt.has_value()) {
      goal_state_tolerance_out.position = position_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.goal_tolerance has a invalid position tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }

    const auto velocity_opt = ResolveToleranceSource(goal_state_tolerance_in.velocity, goal_tol.velocity);
    if (velocity_opt.has_value()) {
      goal_state_tolerance_out.velocity = velocity_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.goal_tolerance has a invalid velocity tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }

    const auto acceleration_opt = ResolveToleranceSource(goal_state_tolerance_in.acceleration, goal_tol.acceleration);
    if (acceleration_opt.has_value()) {
      goal_state_tolerance_out.acceleration = acceleration_opt.value();
    } else {
      RCLCPP_ERROR(logger, "joint '%s' specified in goal.goal_tolerance has a invalid acceleration tolerance. "
                           "Using default tolerances.", joint.c_str());
      return default_tolerances;
    }
  }

  return active_tolerances;
}


bool CheckStateTolerancePerJoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& state_error,
    size_t joint_idx,
    const StateTolerances& state_tolerance) {
  const double error_position = state_error.positions[joint_idx];
  const double error_velocity = state_error.velocities[joint_idx];
  const double error_acceleration = state_error.accelerations[joint_idx];

  return !(state_tolerance.position > 0.0 && std::abs(error_position) > state_tolerance.position) &&
         !(state_tolerance.velocity > 0.0 && std::abs(error_velocity) > state_tolerance.velocity) &&
         !(state_tolerance.acceleration > 0.0 && std::abs(error_acceleration) > state_tolerance.acceleration);
}

}  // namespace tmc_joint_command_controller
