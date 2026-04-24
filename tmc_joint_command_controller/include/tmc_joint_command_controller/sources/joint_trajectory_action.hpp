// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_ACTION_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_ACTION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <tmc_joint_command_controller/action_server_base.hpp>
#include <tmc_joint_command_controller/realtime_trajectory.hpp>

namespace tmc_joint_command_controller {

class TrajectoryPriorityPolicy;

class JointTrajectoryAction : public ActionServerBase<control_msgs::action::FollowJointTrajectory> {
 public:
  JointTrajectoryAction() : ActionServerBase("~/follow_joint_trajectory") {}
  explicit JointTrajectoryAction(const std::string& action_name) : ActionServerBase(action_name) {}
  virtual ~JointTrajectoryAction() = default;

  std::vector<std::string> GetCommandInterfaces() const override;
  std::vector<std::string> GetStateInterfaces() const override;

  void WriteCommand() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetDesiredState() const { return desired_state_; }

  void Preempt() override;

 protected:
  using ActionType = control_msgs::action::FollowJointTrajectory;

  void PreemptActiveGoal() override;
  void CancelActiveGoal(RealtimeGoalHandlePtr active_goal) override;

  void OnNoGoal(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  bool OnActiveGoal(const rclcpp::Time& time,
                    const rclcpp::Duration& period,
                    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) override;

  bool ValidateGoal(const std::shared_ptr<const ActionType::Goal> goal) override;
  void ReceiveGoal(const std::shared_ptr<const ActionType::Goal> goal) override;

  bool InitImpl() override;
  bool ConfigureImpl() override;
  bool ActivateImpl(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;

 private:
  RealtimeTrajectory::Ptr trajectory_;
  trajectory_msgs::msg::JointTrajectoryPoint desired_state_;
  std::vector<size_t> position_command_indices_;

  void PreemptImpl();

  std::shared_ptr<TrajectoryPriorityPolicy> priority_policy_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_SOURCES_JOINT_TRAJECTORY_ACTION_HPP_
