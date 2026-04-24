// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_

#include <map>
#include <string>
#include <vector>

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
  // InitやConfigureで与えられたものは，サブクラスでもそのまま使えるように，と考えた
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string source_name_;
  Accessor* accessor_;

  JointsInfo::Ptr joints_info_;

  void UpdateTargetControlMode(std::vector<double> mode) { target_control_mode_ = mode; }

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
};

}  // namespace tmc_joint_command_controller
#endif  // #define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_SOURCE_BASE_HPP_
