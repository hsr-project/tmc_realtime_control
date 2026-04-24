// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SOURCES_POSITION_JOG_TOPIC_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SOURCES_POSITION_JOG_TOPIC_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/msg/joint_jog.hpp>

#include <tmc_joint_command_controller/joint_command_source_base.hpp>
#include <tmc_joint_command_controller/realtime_subscription_base.hpp>

namespace tmc_joint_command_controller {

class PositionJogSubscription : public RealtimeSubscriptionBase<control_msgs::msg::JointJog> {
 public:
  explicit PositionJogSubscription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                   const std::vector<std::string>& joint_names);

 protected:
  void Callback(const control_msgs::msg::JointJog::SharedPtr msg) override;

 private:
  std::vector<std::string> joint_names_;
};

class PositionJogTopic : public JointCommandSourceBase {
 public:
  PositionJogTopic() : command_timeout_(0, 0) {}
  ~PositionJogTopic() = default;

  std::vector<std::string> GetCommandInterfaces() const override;
  std::vector<std::string> GetStateInterfaces() const override;

  void ReadAndUpdate(const rclcpp::Time& time,
                     const rclcpp::Duration& period,
                     const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override;
  bool HasCommand() const override;
  void WriteCommand() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const override;

  void Preempt() override;

 protected:
  bool ConfigureImpl() override;
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

 private:
  rclcpp::Duration command_timeout_;

  std::vector<double> position_command_;

  std::vector<size_t> position_command_indices_;
  std::vector<size_t> position_state_indices_;

  std::shared_ptr<PositionJogSubscription> realtime_sub_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SOURCES_POSITION_JOG_TOPIC_HPP_
