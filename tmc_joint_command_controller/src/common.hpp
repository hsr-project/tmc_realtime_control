// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
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

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SRC_COMMON_HPP_
