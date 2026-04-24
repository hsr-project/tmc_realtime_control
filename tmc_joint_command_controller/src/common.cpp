// Copyright (c) 2026 Toyota Motor Corporation

#include "common.hpp"

#include <tmc_utils/parameters.hpp>

namespace tmc_joint_command_controller {

void AddInterfaces(const std::vector<std::string>& joint_names,
                   const std::string& interface_type,
                   std::vector<std::string>& interfaces) {
  for (const auto& name : joint_names) {
    interfaces.push_back(name + "/" + interface_type);
  }
}


bool ValidateJogCommand(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                        const std::vector<std::string>& command_joint_names,
                        const std::vector<double>& command_values,
                        const std::vector<std::string>& controlled_joint_names) {
  if (command_joint_names.empty()) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                          "Received message with empty joint_names.");
    return false;
  }

  for (const auto& incoming_joint_name : command_joint_names) {
    auto it = std::find(controlled_joint_names.begin(), controlled_joint_names.end(), incoming_joint_name);
    if (it == controlled_joint_names.end()) {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                            "Incoming joint %s doesn't match the controller's joints.",
                            incoming_joint_name.c_str());
      return false;
    }
  }

  if (command_values.size() != command_joint_names.size()) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                          "Received message with invalid sizes.");
    return false;
  }
  return true;
}

void UpdateStamp(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    stamp = node->now();
  }
}


std::vector<double> GetTargetControlModeFromParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& joint_names,
    const std::string& source_name,
    int32_t default_mode) {
  auto target_mode_common = tmc_utils::GetParameter<int32_t>(node, source_name + ".target_control_mode", default_mode);

  std::vector<double> target_control_mode;
  for (const auto& joint_name : joint_names) {
    auto target_mode = tmc_utils::GetParameter<int32_t>(
        node, source_name + "." + joint_name + ".target_control_mode", target_mode_common);
    target_control_mode.push_back(static_cast<double>(target_mode));
  }

  return target_control_mode;
}


TrajectoryPriorityPolicy::TrajectoryPriorityPolicy(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                   const std::string& parameter_ns,
                                                   int32_t default_priority)
    : default_priority_(default_priority) {
  // 特に100の根拠はない
  boosted_priority_ = tmc_utils::GetParameter<int32_t>(node, parameter_ns + ".boosted_priority", 100);
  boost_duration_ = tmc_utils::GetParameter<double>(node, parameter_ns + ".boost_duration", 0.05);
}

int32_t TrajectoryPriorityPolicy::GetPriority(const rclcpp::Time& now,
                                              const rclcpp::Time& command_received_time) const {
  if ((now - command_received_time).seconds() < boost_duration_) {
    return boosted_priority_;
  }
  return default_priority_;
}

}  // namespace tmc_joint_command_controller
