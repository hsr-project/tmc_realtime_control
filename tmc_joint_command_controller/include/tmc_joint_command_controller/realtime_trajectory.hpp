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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_REALTIME_TRAJECTORY_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_REALTIME_TRAJECTORY_HPP_

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "joint_command_source.hpp"
#include "tolerances.hpp"

namespace tmc_joint_command_controller {

class RealtimeTrajectory {
 public:
  using Ptr = std::shared_ptr<RealtimeTrajectory>;

  // tmc_control_msgs/action/FollowJointTrajectory.action compliant
  enum class ErrorCode : int32_t {
    kSuccessful = 0,
    kInvalidGoal = -1,
    kInvalidJoints = -2,
    kOldHeaderTimestamp = -3,
    kPathToleranceViolated = -4,
    kGoalToleranceViolated = -5,

    kSampled = 1,
    kEmptyTrajectory = -100,
  };
  struct State {
    trajectory_msgs::msg::JointTrajectoryPoint desired;
    trajectory_msgs::msg::JointTrajectoryPoint actual;
    trajectory_msgs::msg::JointTrajectoryPoint error;
  };
  using SampleResult = std::tuple<ErrorCode, State&>;

  RealtimeTrajectory(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                     const std::string& parameter_ns,
                     Accessor* accessor)
      : node_(node), accessor_(accessor), parameter_ns_(parameter_ns) {}

  virtual std::vector<std::string> GetStateInterfaces() const;

  virtual bool Configure(const JointsInfo::Ptr& joints_info);
  virtual bool Activate(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  void WriteFromNonRT(const trajectory_msgs::msg::JointTrajectory& msg);
  void WriteFromNonRT(const control_msgs::action::FollowJointTrajectory::Goal& msg);
  void PreemptFromNonRT(bool clear_last_command = true);

  void UpdateLastSampledTimeFromRT(const rclcpp::Time& time) { last_sampled_time_ = time; }
  SampleResult SampleFromRT(const rclcpp::Time& time,
                            const rclcpp::Duration& period,
                            const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state);

  // Since only joint_info_ is used, it might not need to be a class function
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& msg) const;

 protected:
  virtual trajectory_msgs::msg::JointTrajectoryPoint GetCurrentStateFromRT() const;
  virtual trajectory_msgs::msg::JointTrajectoryPoint GetInitialStateForSampling(
      const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  Accessor* accessor_;

  virtual void ActivateInterfaces(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);
  virtual void ActivateTrajectory();

 private:
  std::string parameter_ns_;

  JointsInfo::Ptr joints_info_;

  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  rclcpp::Time last_sampled_time_;
  bool has_last_command_state_;
  State sampled_state_;

  bool open_loop_control_;
  bool use_velocity_state_;

  std::vector<size_t> position_state_indices_;
  std::vector<size_t> velocity_state_indices_;

  SegmentTolerances default_tolerances_;
  realtime_tools::RealtimeBuffer<SegmentTolerances> active_tolerances_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_REALTIME_TRAJECTORY_HPP_
