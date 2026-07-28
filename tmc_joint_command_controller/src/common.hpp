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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace tmc_joint_command_controller {

void AddInterfaces(const std::vector<std::string>& joint_names,
                   const std::string& interface_type,
                   std::vector<std::string>& interfaces);


bool ValidateJogCommand(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                        const std::vector<std::string>& command_joint_names,
                        const std::vector<double>& command_values,
                        const std::vector<std::string>& controlled_joint_names);

void UpdateStamp(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 builtin_interfaces::msg::Time& stamp);


std::vector<double> GetTargetControlModeFromParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& joint_names,
    const std::string& source_name,
    int32_t default_mode);


class TrajectoryPriorityPolicy {
 public:
  TrajectoryPriorityPolicy(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                           const std::string& parameter_ns,
                           int32_t default_priority);

  int32_t GetDefaultPriority() const { return default_priority_; }
  int32_t GetPriority(const rclcpp::Time& now, const rclcpp::Time& command_received_time) const;

 private:
  int32_t default_priority_;
  int32_t boosted_priority_;
  double boost_duration_;
};


template <typename T>
size_t GetIndex(const std::vector<T>& interfaces, const std::string& name, const std::string& interface) {
  const auto full_name = name + "/" + interface;
  for (auto i = 0u; i < interfaces.size(); ++i) {
    if (interfaces[i].get_name() == full_name) {
      return i;
    }
  }
  return interfaces.size();
}

template<typename T>
std::vector<size_t> GetIndices(
    const std::vector<T>& interfaces,
    const std::vector<std::string>& joint_names,
    const std::string& interface) {
  std::vector<size_t> indices;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto& joint_name = joint_names[i];
    const auto index = GetIndex(interfaces, joint_name, interface);
    if (index == interfaces.size()) {
      throw std::runtime_error("Interface not found: " + joint_name + "/" + interface);
    }
    indices.push_back(index);
  }
  return indices;
}

template <typename T>
std::vector<size_t> GetPositionInterfaceIndices(
    const std::vector<T>& interfaces,
    const std::vector<std::string>& joint_names) {
  return GetIndices(interfaces, joint_names, hardware_interface::HW_IF_POSITION);
}

template <typename T>
std::vector<size_t> GetVelocityInterfaceIndices(
    const std::vector<T>& interfaces,
    const std::vector<std::string>& joint_names) {
  return GetIndices(interfaces, joint_names, hardware_interface::HW_IF_VELOCITY);
}

template <typename T>
std::vector<size_t> GetEffortInterfaceIndices(
    const std::vector<T>& interfaces,
    const std::vector<std::string>& joint_names) {
  return GetIndices(interfaces, joint_names, hardware_interface::HW_IF_EFFORT);
}

// TODO(Hidaka) tmc_utilsに移行する
template<typename TYPE>
void SetCommandInterfaceValue(const rclcpp::Logger& logger,
                              TYPE& interface,
                              const double value) {
  if (!interface.set_value(value)) {
    RCLCPP_WARN(logger,
                "Unable to set the command interface value %s: value = %f",
                interface.get_name().c_str(), value);
  }
}

template<typename TYPE>
double GetStateInterfaceValue(const TYPE& interface) {
  std::optional<double> opt_value = interface.get_optional();
  if (opt_value.has_value()) {
    return opt_value.value();
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_
