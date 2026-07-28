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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_

#include <map>
#include <string>
#include <vector>

#include <tmc_utils/parameters.hpp>

#include <tmc_joint_command_controller/joint_command_source.hpp>

namespace tmc_joint_command_controller {

class JointCommandSourceBase : public IJointCommandSource {
 public:
  virtual ~JointCommandSourceBase() = default;

  bool Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
            const std::string& source_name,
            Accessor* accessor) final;
  bool Configure(const JointsInfo::Ptr& joints_info) override;
  bool Activate(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

  void Preempt() override {}

  std::vector<double> GetTargetControlMode() const override { return target_control_mode_; }

  int32_t GetPriority() const final { return priority_; }
  rclcpp::Time GetLastCommandTime() const final { return last_command_time_; }

 protected:
  // The items provided in Init or Configure are intended to be usable as-is even in subclasses.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string source_name_;
  Accessor* accessor_;

  JointsInfo::Ptr joints_info_;

  void UpdateTargetControlMode(std::vector<double> mode);

  void UpdatePriority(int32_t priority) {priority_ = priority; }

  void UpdateLastCommandTime() { last_command_time_ = node_->get_clock()->now(); }
  void UpdateLastCommandTime(const rclcpp::Time& time) { last_command_time_ = time; }

  virtual bool InitImpl() { return true; }
  virtual bool ConfigureImpl() = 0;
  virtual bool ActivateImpl(
      [[maybe_unused]] const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      [[maybe_unused]] const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) { return true; }

 private:
  int32_t priority_;
  rclcpp::Time last_command_time_;
  std::vector<double> target_control_mode_;

  std::vector<tmc_utils::AtomicDynamicParameter<int32_t>::Ptr> dynamic_parameters_;
};

}  // namespace tmc_joint_command_controller
#endif  // #define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_
