// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_ACTION_SERVER_BASE_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_ACTION_SERVER_BASE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

#include <tmc_utils/parameters.hpp>

#include <tmc_joint_command_controller/joint_command_source_base.hpp>

namespace tmc_joint_command_controller {

template <typename ActionType>
class ActionServerBase : public JointCommandSourceBase {
 public:
  explicit ActionServerBase(const std::string& action_name) : JointCommandSourceBase(), action_name_(action_name) {}
  virtual ~ActionServerBase() = default;

  bool Configure(const JointsInfo::Ptr& joints_info) final;
  bool Activate(
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) final;

  void ReadAndUpdate(const rclcpp::Time& time,
                     const rclcpp::Duration& period,
                     const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) final;
  bool HasCommand() const final;

  // 他の種類の指令値が来たとき中断
  void Preempt() override;

 protected:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ActionType>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  // 新しいゴールが来たときの中断
  virtual void PreemptActiveGoal();
  // アクションがキャンセルされたときの処理
  virtual void CancelActiveGoal(RealtimeGoalHandlePtr active_goal);

  void SetSucceeded(std::shared_ptr<typename ActionType::Result> result);
  void SetAborted(std::shared_ptr<typename ActionType::Result> result);

  void SetFeedback(std::shared_ptr<typename ActionType::Feedback> feedback);

  bool HasActiveGoal() const;
  void WriteCommandReceivedTime(const rclcpp::Time& time);

  virtual void OnNoGoal([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period) {}
  virtual bool OnActiveGoal(const rclcpp::Time& time,
                            const rclcpp::Duration& period,
                            const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) = 0;

  virtual bool ValidateGoal([[maybe_unused]] const std::shared_ptr<const typename ActionType::Goal> goal) {
    return true;
  }
  virtual void ReceiveGoal(const std::shared_ptr<const typename ActionType::Goal> goal) = 0;

  rclcpp::Time GetCommandReceivedTime() const { return *command_received_time_buffer_.readFromNonRT(); }

 private:
  std::string action_name_;

  double action_monitor_period_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;

  typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> goal_handle_buffer_;
  realtime_tools::RealtimeBuffer<rclcpp::Time> command_received_time_buffer_;

  bool has_command_;

  rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const typename ActionType::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const std::shared_ptr<GoalHandle> goal_handle);
  void FeedbackSetupCallback(std::shared_ptr<GoalHandle> goal_handle);
};


template <typename ActionType>
bool ActionServerBase<ActionType>::Configure(const JointsInfo::Ptr& joints_info) {
  if (!JointCommandSourceBase::Configure(joints_info)) {
    return false;
  }

  const auto action_monitor_rate = tmc_utils::GetParameter<double>(node_, source_name_ + ".action_monitor_rate", 20.0);
  if (action_monitor_rate <= std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR(node_->get_logger(), "Action monitor rate must be positive.");
    return false;
  }
  action_monitor_period_ = 1.0 / action_monitor_rate;

  // 継承でアクション名を与えたいケースと，パラメータで与えたいケースがあると思うので両方に対応させる
  const auto action_name = tmc_utils::GetParameter<std::string>(node_, source_name_ + ".action_name", action_name_);
  action_server_ = rclcpp_action::create_server<ActionType>(
      node_, action_name,
      std::bind(&ActionServerBase<ActionType>::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServerBase<ActionType>::CancelCallback, this, std::placeholders::_1),
      std::bind(&ActionServerBase<ActionType>::FeedbackSetupCallback, this, std::placeholders::_1));

  return true;
}

template <typename ActionType>
bool ActionServerBase<ActionType>::Activate(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  command_received_time_buffer_.writeFromNonRT(rclcpp::Time(0));

  has_command_ = false;

  return ActivateImpl(command_interfaces, state_interfaces);
}

template <typename ActionType>
void ActionServerBase<ActionType>::ReadAndUpdate(
    const rclcpp::Time& time,
    const rclcpp::Duration& period,
    const trajectory_msgs::msg::JointTrajectoryPoint& previous_desired_state) {
  // goalがあるかのチェック
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal) {
    has_command_ = false;
    OnNoGoal(time, period);
    return;
  }

  if (!OnActiveGoal(time, period, previous_desired_state)) {
    has_command_ = false;
    return;
  }

  has_command_ = true;
  UpdateLastCommandTime(*command_received_time_buffer_.readFromNonRT());
}

template <typename ActionType>
bool ActionServerBase<ActionType>::HasCommand() const {
  return has_command_;
}

template <typename ActionType>
void ActionServerBase<ActionType>::WriteCommandReceivedTime(const rclcpp::Time& time) {
  command_received_time_buffer_.writeFromNonRT(time);
}

template <typename ActionType>
void ActionServerBase<ActionType>::Preempt() {
  has_command_ = false;

  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal) {
     return;
  }

  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  goal_handle_timer_.reset();

  command_received_time_buffer_.writeFromNonRT(rclcpp::Time(0));
}

template <typename ActionType>
void ActionServerBase<ActionType>::PreemptActiveGoal() {
  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal) {
     return;
  }

  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
}

template <typename ActionType>
void ActionServerBase<ActionType>::CancelActiveGoal(RealtimeGoalHandlePtr active_goal) {
  has_command_ = false;

  active_goal->setCanceled(std::make_shared<typename ActionType::Result>());
  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());

  command_received_time_buffer_.writeFromNonRT(rclcpp::Time(0));
}

template <typename ActionType>
void ActionServerBase<ActionType>::SetSucceeded(std::shared_ptr<typename ActionType::Result> result) {
  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal) {
    active_goal->setSucceeded(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

template <typename ActionType>
void ActionServerBase<ActionType>::SetAborted(std::shared_ptr<typename ActionType::Result> result) {
  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal) {
    active_goal->setAborted(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

template <typename ActionType>
void ActionServerBase<ActionType>::SetFeedback(std::shared_ptr<typename ActionType::Feedback> feedback) {
  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal) {
    active_goal->setFeedback(feedback);
  }
}

template <typename ActionType>
bool ActionServerBase<ActionType>::HasActiveGoal() const {
  auto active_goal = *goal_handle_buffer_.readFromNonRT();
  return static_cast<bool>(active_goal);
}

template <typename ActionType>
rclcpp_action::GoalResponse ActionServerBase<ActionType>::GoalCallback(
    [[maybe_unused]] const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const typename ActionType::Goal> goal) {
  if (!ValidateGoal(goal)) {
    RCLCPP_WARN(node_->get_logger(), "Rejected goal request");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionType>
rclcpp_action::CancelResponse ActionServerBase<ActionType>::CancelCallback(
    const std::shared_ptr<GoalHandle> goal_handle) {
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    CancelActiveGoal(active_goal);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionType>
void ActionServerBase<ActionType>::FeedbackSetupCallback(std::shared_ptr<GoalHandle> goal_handle) {
  PreemptActiveGoal();
  command_received_time_buffer_.writeFromNonRT(node_->now());

  ReceiveGoal(goal_handle->get_goal());

  auto realtime_goal_handle = std::make_shared<RealtimeGoalHandle>(goal_handle);
  realtime_goal_handle->execute();
  goal_handle_buffer_.writeFromNonRT(realtime_goal_handle);

  goal_handle_timer_ = node_->create_wall_timer(std::chrono::duration<double>(action_monitor_period_),
                                                std::bind(&RealtimeGoalHandle::runNonRealtime, realtime_goal_handle));
}

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_ACTION_SERVER_BASE_HPP_
