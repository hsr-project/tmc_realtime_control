// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_JOINTS_INFO_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_JOINTS_INFO_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace tmc_joint_command_controller {

class JointsInfo {
 public:
  using Ptr = std::shared_ptr<JointsInfo>;

  explicit JointsInfo(const std::vector<std::string>& joint_names)
      : names_(joint_names), command_joints_(joint_names) {}
  explicit JointsInfo(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  bool IsValid(const rclcpp::Logger& logger) const;

  void AddCommandPositionInterfaces(std::vector<std::string>& command_interfaces_out) const;
  void AddCommandVelocityInterfaces(std::vector<std::string>& command_interfaces_out) const;
  void AddStatePositionInterfaces(std::vector<std::string>& state_interfaces_out) const;
  void AddStateVelocityInterfaces(std::vector<std::string>& state_interfaces_out) const;
  void AddStateEffortInterfaces(std::vector<std::string>& state_interfaces_out) const;

  const std::vector<std::string>& names() const { return names_; }
  const std::vector<std::string>& command_joints() const { return command_joints_; }

 private:
  std::vector<std::string> names_;
  std::vector<std::string> command_joints_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_JOINTS_INFO_HPP_
