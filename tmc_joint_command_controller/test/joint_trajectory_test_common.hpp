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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_TEST_JOINT_TRAJECTORY_TEST_COMMON_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_TEST_JOINT_TRAJECTORY_TEST_COMMON_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_joint_command_controller/joint_command_source.hpp>

#include "utils.hpp"

namespace tmc_joint_command_controller {

constexpr double kEpsilon = 1.0e-6;
constexpr double kValidationTolerance = 1.0e-2;

constexpr double kPositionTolerance = 0.1;
constexpr double kTimeTolerance = 0.1;

const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const std::vector<std::string> kCommandJoints = {"command/joint1", "command/joint2"};

const std::string kControllerName = "joint_command_controller";  // NOLINT
const std::string kSourceName = "joint_trajectory";  // NOLINT

const rclcpp::Duration kUpdatePeriod = rclcpp::Duration::from_seconds(0.002);
constexpr int32_t kTrajectorySec = 1;

rclcpp::Time GetUpdateTime(const rclcpp::Time& command_time, double rate) {
  return command_time + rclcpp::Duration::from_seconds(static_cast<double>(kTrajectorySec) * rate);
}

trajectory_msgs::msg::JointTrajectory GetPositionTrajectory(const rclcpp::Time& time,
                                                            const std::vector<double>& positions) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = time;
  msg.joint_names = kJointNames;
  msg.points.resize(1);
  msg.points[0].positions = positions;
  msg.points[0].time_from_start.sec = kTrajectorySec;
  return msg;
}

trajectory_msgs::msg::JointTrajectory GetAllProvidedTrajectory(const rclcpp::Time& time) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = time;
  msg.joint_names = kJointNames;
  msg.points.resize(1);
  msg.points[0].positions = {1.0, 2.0};
  msg.points[0].velocities = {0.5, 1.0};
  msg.points[0].accelerations = {0.25, 0.5};
  msg.points[0].time_from_start.sec = kTrajectorySec;
  return msg;
}

class AccessorMock : public Accessor {
 public:
  explicit AccessorMock(const std::vector<std::string>& command_joints,
                        const std::vector<std::string>& state_joints) {
    command_positions_.resize(command_joints.size());
    for (size_t i = 0; i < command_joints.size(); ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          command_joints[i], hardware_interface::HW_IF_POSITION, &command_positions_[i]));
    }

    state_positions_.resize(state_joints.size());
    state_velocities_.resize(state_joints.size());
    for (size_t i = 0; i < state_joints.size(); ++i) {
      state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
          state_joints[i], hardware_interface::HW_IF_POSITION, &state_positions_[i]));
      state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
          state_joints[i], hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]));
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> GetLoanedCommandInterfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    for (auto& command_interface : command_interfaces_) {
      loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface, nullptr));
    }
    return loaned_command_interfaces;
  }

  std::vector<hardware_interface::LoanedStateInterface> GetLoanedStateInterfaces() {
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    for (auto& state_interface : state_interfaces_) {
      loaned_state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(state_interface, nullptr));
    }
    return loaned_state_interfaces;
  }

  void SetStatePositions(const std::vector<double>& positions) {
    state_positions_ = positions;
  }

  void SetStateVelocities(const std::vector<double>& velocities) {
    state_velocities_ = velocities;
  }

  std::vector<double> GetCommandPositions() const {
    return command_positions_;
  }

  void SetCommand(size_t index, double command_value) override {
    SetCommandInterfaceValue(rclcpp::get_logger("rclcpp"), command_interfaces_[index], command_value);
  }

  double GetState(size_t index) const override {
    return GetStateInterfaceValue(state_interfaces_[index]);
  }

 private:
  std::vector<double> command_positions_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
};

class JointTrajectoryCommandSourceTest : public ::testing::Test {
 protected:
  void SetUp(bool open_loop_control = true,
             bool use_velocity_state = false,
             double position_tolerance = 0.0,
             double time_tolerance = 0.0);

  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  JointsInfo::Ptr joints_info_;
  std::shared_ptr<AccessorMock> accessor_;

  IJointCommandSource::Ptr command_source_;
  virtual void InitializeCommandSource() = 0;

  rclcpp::Node::SharedPtr client_node_;
  virtual bool InitializeCommandClient() { return false; }

  trajectory_msgs::msg::JointTrajectoryPoint empty_state_;

  virtual void SendTrajectory([[maybe_unused]] const trajectory_msgs::msg::JointTrajectory& trajectory) {}

  virtual bool WaitFor(
      const rclcpp::Time& time,
      const std::optional<trajectory_msgs::msg::JointTrajectoryPoint>& expected_previous_desired_state = std::nullopt);
  void ValidateCommand(const std::vector<double>& positions, const std::vector<double>& velocities);

  void TestWithOpenLoopControl();
  void TestGlobalOpenLoopControl();
  void TestTimeZeroTrajectory();
  void TestWithoutOpenLoopControl();
  void TestUseVelocityState();
  void TestOneJointTrajectory();
  void TestUnorderedJointTrajectory();
  void TestPositionVelocityTrajectory();
  void TestPositionVelocityAccerationTrajectory();
  void TestPathToleranceViolated();
  void TestGoalToleranceViolatedWithoutVelocityState();
  void TestGoalToleranceWithVelocityState();
  void TestMismatchedPositionsSize();
  void TestMismatchedVelocitiesSize();
  void TestMismatchedAccelerationsSize();
  void TestNonIncreasingTimeFromStart();
  void TestWithInvalidJointName();
  void TestEmptyJointNames();
  void TestEmptyTrajectoryPoints();
  void TestWithCommandJoints();
};

void JointTrajectoryCommandSourceTest::SetUp(bool open_loop_control,
                                             bool use_velocity_state,
                                             double position_tolerance,
                                             double time_tolerance) {
  const std::string constraints_prefix = "constraints.";
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("joints", kJointNames),
      rclcpp::Parameter(kSourceName + ".priority", 42),
      rclcpp::Parameter(kSourceName + ".boosted_priority", 108),
      rclcpp::Parameter(kSourceName + ".boost_duration", 0.1),
      rclcpp::Parameter(kSourceName + ".target_control_mode", -1),
      rclcpp::Parameter(kSourceName + "." + kJointNames[1] + ".target_control_mode", -2),
      rclcpp::Parameter(kSourceName + ".open_loop_control", open_loop_control),
      rclcpp::Parameter(kSourceName + ".use_velocity_state", use_velocity_state),
      rclcpp::Parameter(constraints_prefix + kJointNames[0] + ".trajectory", position_tolerance),
      rclcpp::Parameter(constraints_prefix + kJointNames[0] + ".goal", position_tolerance),
      rclcpp::Parameter(constraints_prefix + "goal_time", time_tolerance)};
  controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(kControllerName, options);
  joints_info_ = std::make_shared<JointsInfo>(controller_node_);
  accessor_ = std::make_shared<AccessorMock>(kJointNames, kJointNames);

  InitializeCommandSource();
  ASSERT_TRUE(command_source_->Init(controller_node_, kSourceName, accessor_.get()));
  ASSERT_TRUE(command_source_->Configure(joints_info_));

  const auto command_interfaces = command_source_->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : kJointNames) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, command_interfaces);
  }

  const auto state_interfaces = command_source_->GetStateInterfaces();
  if (use_velocity_state) {
    EXPECT_EQ(state_interfaces.size(), 4);
    for (const auto& joint_name : kJointNames) {
      AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, state_interfaces);
      AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, state_interfaces);
    }
  } else {
    EXPECT_EQ(state_interfaces.size(), 2);
    for (const auto& joint_name : kJointNames) {
      AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, state_interfaces);
    }
  }

  ASSERT_TRUE(command_source_->Activate(accessor_->GetLoanedCommandInterfaces(),
                                        accessor_->GetLoanedStateInterfaces()));

  client_node_ = std::make_shared<rclcpp::Node>("client_node");
  ASSERT_TRUE(InitializeCommandClient());

  // Initially, there is no command value
  EXPECT_FALSE(command_source_->HasCommand());
}

bool JointTrajectoryCommandSourceTest::WaitFor(
    const rclcpp::Time& time,
    const std::optional<trajectory_msgs::msg::JointTrajectoryPoint>& expected_previous_desired_state) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.3);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      return false;
    }
    if (command_source_->GetLastCommandTime() >= time) {
      return true;
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
    if (expected_previous_desired_state) {
      command_source_->ReadAndUpdate(time, kUpdatePeriod, expected_previous_desired_state.value());
    } else {
      command_source_->ReadAndUpdate(time, kUpdatePeriod, empty_state_);
    }
  }
  return false;
}

void JointTrajectoryCommandSourceTest::ValidateCommand(const std::vector<double>& positions,
                                                       const std::vector<double>& velocities) {
  ASSERT_TRUE(command_source_->HasCommand());
  command_source_->WriteCommand();

  const auto command_positions = accessor_->GetCommandPositions();
  ASSERT_EQ(command_positions.size(), positions.size());
  for (auto i = 0u; i < positions.size(); ++i) {
    EXPECT_NEAR(command_positions[i], positions[i], kValidationTolerance);
  }

  const auto desired_state = command_source_->GetDesiredState();
  ASSERT_EQ(desired_state.positions.size(), positions.size());
  for (auto i = 0u; i < positions.size(); ++i) {
    EXPECT_NEAR(desired_state.positions[i], positions[i], kValidationTolerance);
  }
  for (auto i = 0u; i < velocities.size(); ++i) {
    EXPECT_NEAR(desired_state.velocities[i], velocities[i], kValidationTolerance);
  }
}

void JointTrajectoryCommandSourceTest::TestWithOpenLoopControl() {
  SetUp();

  accessor_->SetStatePositions({-1.0, -2.0});

  const auto command_time = client_node_->now();
  SendTrajectory(GetPositionTrajectory(command_time, {1.0, 2.0}));
  ASSERT_TRUE(WaitFor(command_time));

  // The update time is the time in the topic's header
  EXPECT_EQ(command_source_->GetLastCommandTime(), command_time);

  // The control mode is specified by the parameter
  const auto target_control_mode = command_source_->GetTargetControlMode();
  EXPECT_EQ(target_control_mode.size(), 2);
  EXPECT_EQ(target_control_mode[0], -1);
  EXPECT_EQ(target_control_mode[1], -2);

  // The priority immediately after receiving the topic is specified by the parameter
  EXPECT_EQ(command_source_->GetPriority(), 108);

  // During startup tracking
  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({0.0, 0.0}, {2.0, 4.0});

  // After boost_duration has elapsed since receiving the topic, the priority returns to normal
  EXPECT_EQ(command_source_->GetPriority(), 42);

  // Before startup tracking is complete
  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 2.0}, {2.0, 4.0});

  // After startup tracking is complete
  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 + kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());

  // Moves from the point when the previous startup tracking was completed
  const auto new_command_time = GetUpdateTime(command_time, 2.0);
  command_source_->ReadAndUpdate(new_command_time - kUpdatePeriod, kUpdatePeriod, empty_state_);

  SendTrajectory(GetPositionTrajectory(new_command_time, {2.0, 4.0}));
  ASSERT_TRUE(WaitFor(new_command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(new_command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({1.5, 3.0}, {1.0, 2.0});

  // If in startup tracking, moves from the value at that point
  const auto interrupt_time = GetUpdateTime(new_command_time, 0.5);
  SendTrajectory(GetPositionTrajectory(interrupt_time, {0.0, 0.0}));
  ASSERT_TRUE(WaitFor(interrupt_time));

  command_source_->ReadAndUpdate(GetUpdateTime(interrupt_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({0.75, 1.5}, {-1.5, -3.0});

  // If interrupted, moves from the current state
  command_source_->Preempt();
  EXPECT_FALSE(command_source_->HasCommand());

  const auto post_interrupt_time = GetUpdateTime(interrupt_time, 0.5);
  SendTrajectory(GetPositionTrajectory(post_interrupt_time, {0.0, 0.0}));
  ASSERT_TRUE(WaitFor(post_interrupt_time));

  command_source_->ReadAndUpdate(GetUpdateTime(post_interrupt_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({-0.5, -1.0}, {1.0, 2.0});
}

void JointTrajectoryCommandSourceTest::TestGlobalOpenLoopControl() {
  SetUp();

  // At the start of startup tracking, moves from previous_desired_state
  accessor_->SetStatePositions({-1.0, -2.0});
  trajectory_msgs::msg::JointTrajectoryPoint previous_desired_state;
  previous_desired_state.positions = {2.0, 4.0};

  const auto command_time = client_node_->now();
  SendTrajectory(GetPositionTrajectory(command_time, {1.0, 2.0}));
  ASSERT_TRUE(WaitFor(command_time, previous_desired_state));

  // During startup tracking
  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({1.5, 3.0}, {-1.0, -2.0});
}

void JointTrajectoryCommandSourceTest::TestTimeZeroTrajectory() {
  SetUp();

  const auto before_time = controller_node_->now();

  SendTrajectory(GetPositionTrajectory(rclcpp::Time(0, 0), {1.0, 2.0}));
  ASSERT_TRUE(WaitFor(before_time));

  const auto after_time = controller_node_->now();

  EXPECT_GE(command_source_->GetLastCommandTime(), before_time);
  EXPECT_LE(command_source_->GetLastCommandTime(), after_time);
}

void JointTrajectoryCommandSourceTest::TestWithoutOpenLoopControl() {
  SetUp(false);

  accessor_->SetStatePositions({-1.0, -2.0});

  const auto command_time = client_node_->now();
  SendTrajectory(GetPositionTrajectory(command_time, {0.0, 0.0}));
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({-0.5, -1.0}, {1.0, 2.0});

  // Always moves from the current state
  const auto interrupt_time = GetUpdateTime(command_time, 0.5);

  SendTrajectory(GetPositionTrajectory(interrupt_time, {1.0, 2.0}));
  ASSERT_TRUE(WaitFor(interrupt_time));

  command_source_->ReadAndUpdate(GetUpdateTime(interrupt_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({0.0, 0.0}, {2.0, 4.0});
}

void JointTrajectoryCommandSourceTest::TestUseVelocityState() {
  SetUp(true, true);

  accessor_->SetStatePositions({0.0, 0.0});
  accessor_->SetStateVelocities({1.0, 2.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0, 2.0});
  msg.points[0].velocities = {0.0, 0.0};
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ASSERT_TRUE(command_source_->HasCommand());
  command_source_->WriteCommand();

  // Since interpolation considering velocity is performed, it progresses beyond the intermediate point and is faster than linear interpolation
  const auto command_positions = accessor_->GetCommandPositions();
  ASSERT_EQ(command_positions.size(), 2);
  constexpr double kPositionDelta = 0.1;
  EXPECT_GT(command_positions[0], 0.5 + kPositionDelta);
  EXPECT_GT(command_positions[1], 1.0 + kPositionDelta);

  const auto desired_state = command_source_->GetDesiredState();
  ASSERT_EQ(desired_state.positions.size(), 2);
  EXPECT_GT(desired_state.positions[0], 0.5 + kPositionDelta);
  EXPECT_GT(desired_state.positions[1], 1.0 + kPositionDelta);

  constexpr double kVelocityDelta = 0.2;
  ASSERT_EQ(desired_state.velocities.size(), 2);
  EXPECT_GT(desired_state.velocities[0], 1.0 + kVelocityDelta);
  EXPECT_GT(desired_state.velocities[1], 2.0 + kVelocityDelta);
}

void JointTrajectoryCommandSourceTest::TestOneJointTrajectory() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {kJointNames[0]};
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({0.5, 0.0}, {1.0, 0.0});
}

void JointTrajectoryCommandSourceTest::TestUnorderedJointTrajectory() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {2.0, 1.0});
  msg.joint_names = {kJointNames[1], kJointNames[0]};
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 0.5), kUpdatePeriod, empty_state_);
  ValidateCommand({0.5, 1.0}, {1.0, 2.0});
}

void JointTrajectoryCommandSourceTest::TestPositionVelocityTrajectory() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetAllProvidedTrajectory(command_time);
  msg.points[0].accelerations.clear();
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  // The check in between is difficult due to the calculation of expected values, so it is only checked just before the trajectory tracking is completed
  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 2.0}, {0.5, 1.0});
}

void JointTrajectoryCommandSourceTest::TestPositionVelocityAccerationTrajectory() {
  SetUp();

  const auto command_time = client_node_->now();
  SendTrajectory(GetAllProvidedTrajectory(command_time));
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 - kEpsilon), kUpdatePeriod, empty_state_);

  ASSERT_TRUE(command_source_->HasCommand());
  command_source_->WriteCommand();
  ValidateCommand({1.0, 2.0}, {0.5, 1.0});

  const auto desired_state = command_source_->GetDesiredState();
  EXPECT_NEAR(desired_state.accelerations[0], 0.25, kValidationTolerance);
  EXPECT_NEAR(desired_state.accelerations[1], 0.5, kValidationTolerance);
}

void JointTrajectoryCommandSourceTest::TestPathToleranceViolated() {
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
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, kPositionTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({kPositionTolerance, 0.0}, {1.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, kPositionTolerance + kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

void JointTrajectoryCommandSourceTest::TestGoalToleranceViolatedWithoutVelocityState() {
  SetUp(true, false, kPositionTolerance, kTimeTolerance);

  accessor_->SetStatePositions({0.0, 0.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {kJointNames[0]};
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  accessor_->SetStatePositions({1.0 + kPositionTolerance + kEpsilon, 0.0});

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 + kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + kTimeTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + kTimeTolerance + kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

void JointTrajectoryCommandSourceTest::TestGoalToleranceWithVelocityState() {
  SetUp(true, true, kPositionTolerance, kTimeTolerance);

  accessor_->SetStatePositions({0.0, 0.0});
  accessor_->SetStateVelocities({0.0, 0.0});

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {kJointNames[0]};
  SendTrajectory(msg);
  ASSERT_TRUE(WaitFor(command_time));

  accessor_->SetStatePositions({1.0, 0.0});
  constexpr double kVelocityTolerance = 0.01;  // Default value
  accessor_->SetStateVelocities({kVelocityTolerance + kEpsilon, 0.0});

  command_source_->ReadAndUpdate(GetUpdateTime(command_time, 1.0 + kEpsilon), kUpdatePeriod, empty_state_);
  ValidateCommand({1.0, 0.0}, {0.0, 0.0});

  accessor_->SetStateVelocities({kVelocityTolerance - kEpsilon, 0.0});
  command_source_->ReadAndUpdate(
      GetUpdateTime(command_time, 1.0 + kTimeTolerance - kEpsilon), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

void JointTrajectoryCommandSourceTest::TestMismatchedPositionsSize() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetAllProvidedTrajectory(command_time);
  msg.points[0].positions.resize(1);
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestMismatchedVelocitiesSize() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetAllProvidedTrajectory(command_time);
  msg.points[0].velocities.resize(1);
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestMismatchedAccelerationsSize() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetAllProvidedTrajectory(command_time);
  msg.points[0].accelerations.resize(1);
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestNonIncreasingTimeFromStart() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetAllProvidedTrajectory(command_time);
  msg.points.push_back(msg.points[0]);
  msg.points[0].time_from_start.sec = 2 * kTrajectorySec;
  msg.points[1].time_from_start.sec = kTrajectorySec;
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestWithInvalidJointName() {
  SetUp();

  const auto command_time = client_node_->now();
  auto msg = GetPositionTrajectory(command_time, {1.0});
  msg.joint_names = {"invalid_joint"};
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestEmptyJointNames() {
  SetUp();

  const auto command_time = client_node_->now();
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = command_time;
  msg.joint_names = {};
  msg.points.resize(1);
  msg.points[0].time_from_start.sec = kTrajectorySec;
  SendTrajectory(msg);
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestEmptyTrajectoryPoints() {
  SetUp();

  const auto command_time = client_node_->now();
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = command_time;
  msg.joint_names = kJointNames;
  SendTrajectory(msg);
  // The specifications of JointTrajectoryController differ between ROS1 and ROS2
  // In ROS1, it was stopped, but in ROS2, it is not accepted
  EXPECT_FALSE(WaitFor(command_time));
}

void JointTrajectoryCommandSourceTest::TestWithCommandJoints() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("joints", kJointNames),
      rclcpp::Parameter("command_joints", kCommandJoints)};
  controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(kControllerName, options);
  joints_info_ = std::make_shared<JointsInfo>(controller_node_);
  accessor_ = std::make_shared<AccessorMock>(kCommandJoints, kJointNames);

  InitializeCommandSource();
  ASSERT_TRUE(command_source_->Init(controller_node_, kSourceName, accessor_.get()));
  ASSERT_TRUE(command_source_->Configure(joints_info_));

  const auto command_interfaces = command_source_->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : kCommandJoints) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, command_interfaces);
  }

  const auto state_interfaces = command_source_->GetStateInterfaces();
  EXPECT_EQ(state_interfaces.size(), 4);
  for (const auto& joint_name : kJointNames) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, state_interfaces);
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, state_interfaces);
  }

  ASSERT_TRUE(command_source_->Activate(accessor_->GetLoanedCommandInterfaces(),
                                        accessor_->GetLoanedStateInterfaces()));
}

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_TEST_JOINT_TRAJECTORY_TEST_COMMON_HPP_
