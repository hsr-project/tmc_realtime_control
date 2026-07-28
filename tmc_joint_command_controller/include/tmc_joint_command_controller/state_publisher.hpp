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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tmc_realtime_tools/realtime_publisher.hpp>

#include <tmc_joint_command_controller/joint_command_source.hpp>

namespace tmc_joint_command_controller {

class StatePublisher {
 public:
  using Ptr = std::shared_ptr<StatePublisher>;

  StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const JointsInfo::Ptr& joints_info,
                 Accessor* accessor);
  virtual ~StatePublisher() = default;

  void AddStateInterfaces(std::vector<std::string>& state_interfaces) const;

  void Activate(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  void Publish(const rclcpp::Time& time, const trajectory_msgs::msg::JointTrajectoryPoint& desired);

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  JointsInfo::Ptr joints_info_;
  Accessor* accessor_;

  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr publisher_impl_;
  using RealtimePublisher = tmc_realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>;
  std::unique_ptr<RealtimePublisher> publisher_;

  bool publish_position_;
  std::vector<size_t> position_state_interface_indices_;

  bool publish_velocity_;
  std::vector<size_t> velocity_state_interface_indices_;

  bool publish_effort_;
  std::vector<size_t> effort_state_interface_indices_;

  // Some unnecessary class functions are included, but they are kept as class functions for consistency
  void UpdateReference(const std::vector<double>& desired, std::vector<double>& reference_out) const;
  void UpdateFeedback(const std::vector<size_t>& state_interface_indices, std::vector<double>& feedback_out) const;
  void UpdateError(const std::vector<double>& desired, const std::vector<double>& feedback,
                   std::vector<double>& error_out) const;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_
