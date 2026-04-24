// Copyright (c) 2026 Toyota Motor Corporation
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

}  // namespace tmc_joint_command_controller
