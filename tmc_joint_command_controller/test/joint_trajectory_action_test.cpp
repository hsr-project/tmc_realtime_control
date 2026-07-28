/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <gtest/gtest.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <tmc_joint_command_controller/sources/joint_trajectory_action.hpp>

#include "joint_trajectory_test_common.hpp"

namespace tmc_joint_command_controller {

class JointTrajectoryActionTest : public JointTrajectoryCommandSourceTest {
 protected:
  using ActionType = control_msgs::action::FollowJointTrajectory;
  using ClientType = rclcpp_action::Client<ActionType>;

  void InitializeCommandSource() override;
  bool InitializeCommandClient() override;
  void SendTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;
  bool WaitFor(const rclcpp::Time& time,
               const std::optional<trajectory_msgs::msg::JointTrajectoryPoint>& expected_previous_desired_state = std::nullopt) override;  // NOLINT

  bool WaitForStatus(int8_t expected_status);
  bool ValidateResult(rclcpp_action::ResultCode expected_status,
                      int32_t expected_error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
                      ClientType::GoalHandle::SharedPtr goal_handle = nullptr);

  ClientType::SharedPtr client_;
  std::shared_future<ClientType::GoalHandle::SharedPtr> goal_handle_future_;
  ClientType::GoalHandle::SharedPtr goal_handle_;
  bool goal_handle_received_ = false;
};

void JointTrajectoryActionTest::InitializeCommandSource() {
  command_source_ = std::make_shared<JointTrajectoryAction>();
}

bool JointTrajectoryActionTest::InitializeCommandClient() {
  client_ = rclcpp_action::create_client<ActionType>(client_node_, kControllerName + "/follow_joint_trajectory");
  return client_->wait_for_action_server(std::chrono::seconds(1));
}

void JointTrajectoryActionTest::SendTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  ActionType::Goal goal;
  goal.trajectory = trajectory;

  goal_handle_future_ = client_->async_send_goal(goal);
}

bool JointTrajectoryActionTest::WaitFor(
    const rclcpp::Time& time,
    const std::optional<trajectory_msgs::msg::JointTrajectoryPoint>& expected_previous_desired_state) {
  if (!JointTrajectoryCommandSourceTest::WaitFor(time, expected_previous_desired_state)) {
    return false;
  }
  return WaitForStatus(action_msgs::msg::GoalStatus::STATUS_ACCEPTED);
}

bool JointTrajectoryActionTest::WaitForStatus(int8_t expected_status) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.3);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      return false;
    }
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      goal_handle_ = goal_handle_future_.get();
      goal_handle_received_ = true;
      if (goal_handle_ && goal_handle_->get_status() == expected_status) {
        return true;
      } else {
        return false;
      }
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
  }
  return false;
}

bool JointTrajectoryActionTest::ValidateResult(rclcpp_action::ResultCode expected_status,
                                               int32_t expected_error_code,
                                               ClientType::GoalHandle::SharedPtr goal_handle) {
  ClientType::GoalHandle::SharedPtr target_goal_handle = goal_handle ? goal_handle : goal_handle_;
  auto result_future = client_->async_get_result(target_goal_handle);
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      return false;
    }
    if (result_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      auto result = result_future.get();
      return result.code == expected_status && result.result->error_code == expected_error_code;
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
  }
  return false;
}


TEST_F(JointTrajectoryActionTest, WithOpenLoopControl) {
  TestWithOpenLoopControl();
}

TEST_F(JointTrajectoryActionTest, GlobalOpenLoopControl) {
  TestGlobalOpenLoopControl();
}

TEST_F(JointTrajectoryActionTest, TimeZeroTrajectory) {
  TestTimeZeroTrajectory();
}

TEST_F(JointTrajectoryActionTest, WithoutOpenLoopControl) {
  TestWithoutOpenLoopControl();
}

TEST_F(JointTrajectoryActionTest, UseVelocityState) {
  TestUseVelocityState();
}

TEST_F(JointTrajectoryActionTest, OneJointTrajectory) {
  TestOneJointTrajectory();
}

TEST_F(JointTrajectoryActionTest, UnorderedJointTrajectory) {
  TestUnorderedJointTrajectory();
}

TEST_F(JointTrajectoryActionTest, PositionVelocityTrajectory) {
  TestPositionVelocityTrajectory();
}

TEST_F(JointTrajectoryActionTest, PositionVelocityAccerationTrajectory) {
  TestPositionVelocityAccerationTrajectory();
}

TEST_F(JointTrajectoryActionTest, PathToleranceViolated) {
  TestPathToleranceViolated();
  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::ABORTED,
      control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED));
}

TEST_F(JointTrajectoryActionTest, GoalToleranceViolatedWithoutVelocityState) {
  TestGoalToleranceViolatedWithoutVelocityState();
  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::ABORTED,
      control_msgs::action::FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED));
}

TEST_F(JointTrajectoryActionTest, GoalToleranceWithVelocityState) {
  TestGoalToleranceWithVelocityState();
  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::SUCCEEDED,
      control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL));
}

TEST_F(JointTrajectoryActionTest, MismatchedPositionsSize) {
  TestMismatchedPositionsSize();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, MismatchedVelocitiesSize) {
  TestMismatchedVelocitiesSize();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, MismatchedAccelerationsSize) {
  TestMismatchedAccelerationsSize();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, NonIncreasingTimeFromStart) {
  TestNonIncreasingTimeFromStart();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, WithInvalidJointName) {
  TestWithInvalidJointName();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, EmptyJointNames) {
  TestEmptyJointNames();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, EmptyTrajectoryPoints) {
  TestEmptyTrajectoryPoints();
  EXPECT_FALSE(WaitForStatus(action_msgs::msg::GoalStatus::STATUS_UNKNOWN));
  EXPECT_TRUE(goal_handle_received_);
  EXPECT_EQ(goal_handle_, nullptr);
}

TEST_F(JointTrajectoryActionTest, WithCommandJoints) {
  TestWithCommandJoints();
}

TEST_F(JointTrajectoryActionTest, PreemptFromOutside) {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0, 2.0});
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->Preempt();
  EXPECT_FALSE(command_source_->HasCommand());

  EXPECT_TRUE(ValidateResult(rclcpp_action::ResultCode::CANCELED));
}

TEST_F(JointTrajectoryActionTest, PreemptFromNewGoal) {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0, 2.0});
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  auto first_goal_handle = goal_handle_;

  auto new_command_time = GetUpdateTime(command_time, 0.5);
  SendTrajectory(GetPositionTrajectory(new_command_time, {0.0, 0.0}));
  ASSERT_TRUE(WaitFor(new_command_time));

  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::CANCELED,
      control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL,
      first_goal_handle));
}

TEST_F(JointTrajectoryActionTest, CancelGoal) {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0, 2.0});
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  auto cancel_future = client_->async_cancel_goal(goal_handle_);
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      FAIL() << "Cancel goal response not received within timeout";
    }
    if (cancel_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      auto cancel_response = cancel_future.get();
      EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
      break;
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
  }

  EXPECT_TRUE(ValidateResult(rclcpp_action::ResultCode::CANCELED));
  EXPECT_FALSE(command_source_->HasCommand());
}

TEST_F(JointTrajectoryActionTest, ActionFeedback) {
  SetUp();

  accessor_->SetStatePositions({0.0, 0.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0, 2.0});

  bool feedback_received = false;
  auto feedback_func = [&feedback_received](ClientType::GoalHandle::SharedPtr,
                                            const std::shared_ptr<const ActionType::Feedback> feedback) {
    feedback_received = true;

    EXPECT_EQ(feedback->actual.positions.size(), 2);
    EXPECT_EQ(feedback->actual.positions[0], 0.0);
    EXPECT_EQ(feedback->actual.positions[1], 0.0);
    EXPECT_TRUE(feedback->actual.velocities.empty());
    EXPECT_TRUE(feedback->actual.accelerations.empty());

    EXPECT_EQ(feedback->desired.positions.size(), 2);
    EXPECT_NEAR(feedback->desired.positions[0], 0.5, kValidationTolerance);
    EXPECT_NEAR(feedback->desired.positions[1], 1.0, kValidationTolerance);
    EXPECT_EQ(feedback->desired.velocities.size(), 2);
    EXPECT_NEAR(feedback->desired.velocities[0], 1.0, kValidationTolerance);
    EXPECT_NEAR(feedback->desired.velocities[1], 2.0, kValidationTolerance);
    EXPECT_EQ(feedback->desired.accelerations.size(), 2);
    EXPECT_EQ(feedback->desired.accelerations[0], 0.0);
    EXPECT_EQ(feedback->desired.accelerations[1], 0.0);

    EXPECT_NEAR(feedback->error.positions[0], 0.5, kValidationTolerance);
    EXPECT_NEAR(feedback->error.positions[1], 1.0, kValidationTolerance);
    EXPECT_EQ(feedback->error.velocities.size(), 2);
    EXPECT_NEAR(feedback->error.velocities[0], 0.0, kValidationTolerance);
    EXPECT_NEAR(feedback->error.velocities[1], 0.0, kValidationTolerance);
    EXPECT_EQ(feedback->error.accelerations.size(), 2);
    EXPECT_EQ(feedback->error.accelerations[0], 0.0);
    EXPECT_EQ(feedback->error.accelerations[1], 0.0);
  };
  auto send_goal_options = ClientType::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_func;
  ActionType::Goal goal;
  goal.trajectory = msg;
  goal_handle_future_ = client_->async_send_goal(goal, send_goal_options);

  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);

  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      FAIL() << "Feedback not received within timeout";
    }
    if (feedback_received) {
      break;
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
  }
  EXPECT_TRUE(feedback_received);
}

TEST_F(JointTrajectoryActionTest, OverridePathTolerance) {
  SetUp(true, false, kPositionTolerance, kTimeTolerance);

  accessor_->SetStatePositions({0.0, 0.0});

  const auto command_time = client_node_->now();
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = command_time;
  msg.joint_names = {kJointNames[0]};
  msg.points.resize(2);
  msg.points[0].positions = {0.5};
  msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
  msg.points[1].positions = {1.0};
  msg.points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);

  ActionType::Goal goal;
  goal.trajectory = msg;
  goal.path_tolerance.resize(1);
  goal.path_tolerance[0].name = kJointNames[0];
  goal.path_tolerance[0].position = kPositionTolerance * 2.0;
  goal_handle_future_ = client_->async_send_goal(goal);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, kPositionTolerance), kUpdatePeriod, empty_state_);
  ValidateCommand({kPositionTolerance, 0.0}, {1.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 2.0 * kPositionTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({2.0 * kPositionTolerance, 0.0}, {1.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 2.0 * kPositionTolerance + kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());

  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::ABORTED,
      control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED));
}

TEST_F(JointTrajectoryActionTest, OverrideGoalTimeTolerance) {
  SetUp(true, false, kPositionTolerance, kTimeTolerance);

  accessor_->SetStatePositions({0.0, 0.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {kJointNames[0]};

  ActionType::Goal goal;
  goal.trajectory = msg;
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(kTimeTolerance * 2.0);
  goal_handle_future_ = client_->async_send_goal(goal);
  ASSERT_TRUE(WaitFor(command_time));

  accessor_->SetStatePositions({1.0 + kPositionTolerance + kEpsilon, 0.0});

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 + kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + 2.0 * kTimeTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + 2.0 * kTimeTolerance + kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());

  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::ABORTED,
      control_msgs::action::FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED));
}

TEST_F(JointTrajectoryActionTest, OverrideGoalTolerance) {
  SetUp(true, true, kPositionTolerance, kTimeTolerance);

  accessor_->SetStatePositions({0.0, 0.0});
  accessor_->SetStateVelocities({0.0, 0.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {kJointNames[0]};

  constexpr double kVelocityTolerance = 0.01;  // Default value

  ActionType::Goal goal;
  goal.trajectory = msg;
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(kTimeTolerance * 2.0);
  goal.goal_tolerance.resize(1);
  goal.goal_tolerance[0].name = kJointNames[0];
  goal.goal_tolerance[0].velocity = kVelocityTolerance * 2.0;
  goal_handle_future_ = client_->async_send_goal(goal);
  ASSERT_TRUE(WaitFor(command_time));

  accessor_->SetStatePositions({1.0, 0.0});
  accessor_->SetStateVelocities({2.0 * kVelocityTolerance + kEpsilon, 0.0});

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 + kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  accessor_->SetStateVelocities({2.0 * kVelocityTolerance - kEpsilon, 0.0});
  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + kTimeTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());

  EXPECT_TRUE(ValidateResult(
      rclcpp_action::ResultCode::SUCCEEDED,
      control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL));
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
