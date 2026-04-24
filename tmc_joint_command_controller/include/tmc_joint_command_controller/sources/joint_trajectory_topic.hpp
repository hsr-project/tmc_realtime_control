// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_TOPIC_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_TOPIC_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <tmc_joint_command_controller/joint_command_source_base.hpp>
#include <tmc_joint_command_controller/realtime_trajectory.hpp>

namespace tmc_joint_command_controller {

class TrajectoryPriorityPolicy;

class JointTrajectoryTopic : public JointCommandSourceBase {
 public:
  virtual ~JointTrajectoryTopic() = default;

  std::vector<std::string> GetCommandInterfaces() const override;
  std::vector<std::string> GetStateInterfaces() const override;

  void ReadAndUpdate(const rclcpp::Time& time,
                     const rclcpp::Duration& period,
                     const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override;
  bool HasCommand() const override {
    return desired_state_.has_value();
  }
  void WriteCommand() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const override {
    return desired_state_.value_or(trajectory_msgs::msg::JointTrajectoryPoint());
  }

  void Preempt() override;

 protected:
  bool InitImpl() override;
  bool ConfigureImpl() override;
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

 private:
  std::optional<trajectory_msgs::msg::JointTrajectoryPoint> desired_state_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  void Callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  realtime_tools::RealtimeBuffer<rclcpp::Time> command_received_time_buffer_;

  RealtimeTrajectory::Ptr trajectory_;

  std::vector<size_t> position_command_indices_;

  std::shared_ptr<TrajectoryPriorityPolicy> priority_policy_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_TOPIC_HPP_
