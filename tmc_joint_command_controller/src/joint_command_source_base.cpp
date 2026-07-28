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
#include <tmc_joint_command_controller/joint_command_source_base.hpp>

#include <tmc_utils/parameters.hpp>

namespace tmc_joint_command_controller {

bool JointCommandSourceBase::Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                  const std::string& source_name,
                                  Accessor* accessor) {
  node_ = node;
  source_name_ = source_name;
  accessor_ = accessor;
  target_control_mode_.clear();

  return InitImpl();
}

bool JointCommandSourceBase::Configure(const JointsInfo::Ptr& joints_info) {
  joints_info_ = joints_info;
  priority_ = tmc_utils::GetParameter<int32_t>(node_.get(), source_name_ + ".priority", 0);
  last_command_time_ = node_->get_clock()->now();
  return ConfigureImpl();
}

bool JointCommandSourceBase::Activate(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  return ActivateImpl(command_interfaces, state_interfaces);
}

void JointCommandSourceBase::UpdateTargetControlMode(std::vector<double> mode) {
  target_control_mode_ = mode;

  if (dynamic_parameters_.empty()) {
    const auto& joint_names = joints_info_->names();
    for (auto i = 0u; i < joint_names.size(); ++i) {
      dynamic_parameters_.push_back(std::make_shared<tmc_utils::AtomicDynamicParameter<int32_t>>(
          node_, source_name_ + "." + joint_names[i] + ".target_control_mode", static_cast<int32_t>(mode[i]),
          [this, i](int32_t value) {
            target_control_mode_[i] = static_cast<double>(value);
          }));
    }
  } else {
    const auto& joint_names = joints_info_->names();
    for (auto i = 0u; i < joint_names.size(); ++i) {
      node_->set_parameters({
          rclcpp::Parameter(
              source_name_ + "." + joint_names[i] + ".target_control_mode", static_cast<int32_t>(mode[i]))
      });
    }
  }
}

}  // namespace tmc_joint_command_controller
