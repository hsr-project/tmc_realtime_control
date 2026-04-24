// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tmc_realtime_tools/realtime_publisher.hpp>

#include <tmc_joint_command_controller/joint_command_source.hpp>

namespace tmc_joint_command_controller {

class StatePublisher {
 public:
  using Ptr = std::shared_ptr<StatePublisher>;

  StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const JointsInfo::Ptr& joints_info,
                 Accessor* accessor);
  virtual ~StatePublisher() = default;

  void AddStateInterfaces(std::vector<std::string>& state_interfaces) const;

  void Activate(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  void Publish(const rclcpp::Time& time, const trajectory_msgs::msg::JointTrajectoryPoint& desired);

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  JointsInfo::Ptr joints_info_;
  Accessor* accessor_;

  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr publisher_impl_;
  using RealtimePublisher = tmc_realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>;
  std::unique_ptr<RealtimePublisher> publisher_;

  bool publish_position_;
  std::vector<size_t> position_state_interface_indices_;

  bool publish_velocity_;
  std::vector<size_t> velocity_state_interface_indices_;

  bool publish_effort_;
  std::vector<size_t> effort_state_interface_indices_;

  // クラス関数の必要のない関数も混じっているが統一性のためにクラス関数としておく
  void UpdateReference(const std::vector<double>& desired, std::vector<double>& reference_out) const;
  void UpdateFeedback(const std::vector<size_t>& state_interface_indices, std::vector<double>& feedback_out) const;
  void UpdateError(const std::vector<double>& desired, const std::vector<double>& feedback,
                   std::vector<double>& error_out) const;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_STATE_PUBLISHER_HPP_
