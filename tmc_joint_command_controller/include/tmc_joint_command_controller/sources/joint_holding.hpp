// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_HOLDING_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_HOLDING_HPP_

#include <map>
#include <string>
#include <vector>

#include <tmc_joint_command_controller/joint_command_source_base.hpp>

namespace tmc_joint_command_controller {

class PositionHolding : public JointCommandSourceBase {
 public:
  ~PositionHolding() = default;

  std::vector<std::string> GetCommandInterfaces() const override;
  std::vector<std::string> GetStateInterfaces() const override;

  void ReadAndUpdate(const rclcpp::Time& time,
                     const rclcpp::Duration& period,
                     const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override;
  bool HasCommand() const override { return true; }
  void WriteCommand() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const override;

 protected:
  bool ConfigureImpl() override;
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

  trajectory_msgs::msg::JointTrajectoryPoint desired_state_;

 private:
  bool open_loop_control_;

  std::vector<size_t> position_command_indices_;
  std::vector<size_t> position_state_indices_;
};


class VelocityHolding : public JointCommandSourceBase {
 public:
  ~VelocityHolding() = default;

  std::vector<std::string> GetCommandInterfaces() const override;
  std::vector<std::string> GetStateInterfaces() const override { return {}; };

  void ReadAndUpdate(
      [[maybe_unused]] const rclcpp::Time& time,
      [[maybe_unused]] const rclcpp::Duration& period,
      [[maybe_unused]] const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override {}
  bool HasCommand() const override { return true; }
  void WriteCommand() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const override;

 protected:
  bool ConfigureImpl() override;
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

 private:
  std::vector<size_t> velocity_command_indices_;
};


class PositionVelocityHolding : public PositionHolding {
 public:
  ~PositionVelocityHolding() = default;

  std::vector<std::string> GetCommandInterfaces() const override;

  void ReadAndUpdate(const rclcpp::Time& time,
                     const rclcpp::Duration& period,
                     const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override;
  void WriteCommand() override;

 protected:
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

 private:
  std::vector<size_t> velocity_command_indices_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_HOLDING_HPP_
