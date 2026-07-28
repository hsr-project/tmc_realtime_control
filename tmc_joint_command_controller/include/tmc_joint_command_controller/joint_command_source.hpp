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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tmc_joint_command_controller/joints_info.hpp>


namespace tmc_joint_command_controller {

class Accessor {
 public:
  // TODO(Takeshita) boolとoptional<double>で成否を返したほうがいいか？
  virtual void SetCommand(size_t index, double command_value) = 0;
  virtual double GetState(size_t index) const = 0;
};

class IJointCommandSource {
 public:
  using Ptr = std::shared_ptr<IJointCommandSource>;

  virtual ~IJointCommandSource() = default;

  virtual std::vector<std::string> GetCommandInterfaces() const = 0;
  virtual std::vector<std::string> GetStateInterfaces() const = 0;

  virtual bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                    const std::string& _namespace,
                    Accessor* accessor) = 0;
  virtual bool Configure(const JointsInfo::Ptr& joints_info) = 0;
  virtual bool Activate(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) = 0;

  virtual void ReadAndUpdate(const rclcpp::Time& time,
                             const rclcpp::Duration& period,
                             const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) = 0;
  virtual bool HasCommand() const = 0;
  virtual void WriteCommand() = 0;
  virtual trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const = 0;

  virtual void Preempt() = 0;

  virtual std::vector<double> GetTargetControlMode() const = 0;

  virtual int32_t GetPriority() const = 0;
  virtual rclcpp::Time GetLastCommandTime() const = 0;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_HPP_
