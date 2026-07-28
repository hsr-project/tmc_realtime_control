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

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_joint_command_controller/sources/joint_holding.hpp>

#include "utils.hpp"

namespace tmc_joint_command_controller {
const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const std::vector<std::string> kCommandJoints = {"command/joint1", "command/joint2"};

const std::string kSourceName = "joint_holding";  // NOLINT

const rclcpp::Duration kUpdatePeriod = rclcpp::Duration::from_seconds(0.01);
}

namespace tmc_joint_command_controller {

class AccessorMock : public Accessor {
 public:
  explicit AccessorMock(const std::vector<std::string>& command_joints) {
    command_positions_.resize(command_joints.size());
    command_velocities_.resize(command_joints.size());
    for (size_t i = 0; i < command_joints.size(); ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          command_joints[i], hardware_interface::HW_IF_POSITION, &command_positions_[i]));
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          command_joints[i], hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]));
    }

    state_positions_.resize(kJointNames.size());
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
          kJointNames[i], hardware_interface::HW_IF_POSITION, &state_positions_[i]));
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

  std::vector<double> GetCommandPositions() const {
    return command_positions_;
  }

  std::vector<double> GetCommandVelocities() const {
    return command_velocities_;
  }

  void SetCommand(size_t index, double command_value) override {
    SetCommandInterfaceValue(rclcpp::get_logger("rclcpp"), command_interfaces_[index], command_value);
  }

  double GetState(size_t index) const override {
    return GetStateInterfaceValue(state_interfaces_[index]);
  }

 private:
  std::vector<double> command_positions_;
  std::vector<double> command_velocities_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  std::vector<double> state_positions_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
};

struct TestInput {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  JointsInfo::Ptr joints_info;
  std::shared_ptr<AccessorMock> accessor;
  std::vector<std::string> command_joints;

  explicit TestInput(bool use_command_joints, bool open_loop_control = false) {
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {
        rclcpp::Parameter("joints", kJointNames),
        rclcpp::Parameter(kSourceName + ".priority", 42),
        rclcpp::Parameter(kSourceName + ".open_loop_control", open_loop_control)};
    if (use_command_joints) {
      options.parameter_overrides().push_back(rclcpp::Parameter("command_joints", kCommandJoints));
    }
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node", options);
    joints_info = std::make_shared<JointsInfo>(node);
    command_joints = use_command_joints ? kCommandJoints : kJointNames;
    accessor = std::make_shared<AccessorMock>(command_joints);
  }
};

void ValidateCommonProperties(const IJointCommandSource::Ptr& joint_command_source,
                              const rclcpp::Time& update_time) {
  // Always has a command value
  EXPECT_TRUE(joint_command_source->HasCommand());

  // No changes to control mode
  EXPECT_TRUE(joint_command_source->GetTargetControlMode().empty());

  // Priority is very low but not the lowest for int32
  EXPECT_NE(joint_command_source->GetPriority(), std::numeric_limits<int32_t>::min());
  EXPECT_LT(joint_command_source->GetPriority(), -1e8);

  // Update time is indefinite but older than ReadAndUpdate
  EXPECT_LE(joint_command_source->GetLastCommandTime(), update_time);

  // Preempt has no effect or side effects, not testable but called just in case
  joint_command_source->Preempt();
}


// Not worth parameterizing the value for testing
void PositionHoldingTestImpl(bool use_command_joints, bool open_loop_control) {
  TestInput input(use_command_joints, open_loop_control);

  const auto position_holding = std::make_shared<PositionHolding>();
  ASSERT_TRUE(position_holding->Init(input.node, kSourceName, input.accessor.get()));
  ASSERT_TRUE(position_holding->Configure(input.joints_info));

  const auto command_interfaces = position_holding->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : input.command_joints) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, command_interfaces);
  }

  const auto state_interfaces = position_holding->GetStateInterfaces();
  EXPECT_EQ(state_interfaces.size(), 2);
  for (const auto& joint_name : kJointNames) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, state_interfaces);
  }

  ASSERT_TRUE(position_holding->Activate(input.accessor->GetLoanedCommandInterfaces(),
                                         input.accessor->GetLoanedStateInterfaces()));

  // Since previous_desired_state_ is empty, the current value is the command value
  const std::vector<double> current_positions = {1.0, 2.0};
  input.accessor->SetStatePositions(current_positions);

  const auto time = input.node->now();

  trajectory_msgs::msg::JointTrajectoryPoint previous_desired_state_;
  position_holding->ReadAndUpdate(time, kUpdatePeriod, previous_desired_state_);
  position_holding->WriteCommand();
  EXPECT_EQ(input.accessor->GetCommandPositions(), current_positions);

  const auto desired_state = position_holding->GetDesiredState();
  EXPECT_EQ(desired_state.positions, current_positions);
  EXPECT_TRUE(desired_state.velocities.empty());

  ValidateCommonProperties(position_holding, time);

  // If open_loop_control is enabled and previous_desired_state_ is not empty, previous_desired_state_ is the command value
  previous_desired_state_.positions = {3.0, 4.0};
  position_holding->ReadAndUpdate(time, kUpdatePeriod, previous_desired_state_);
  position_holding->WriteCommand();
  if (open_loop_control) {
    EXPECT_EQ(input.accessor->GetCommandPositions(), previous_desired_state_.positions);
    EXPECT_EQ(position_holding->GetDesiredState().positions, previous_desired_state_.positions);
  } else {
    EXPECT_EQ(input.accessor->GetCommandPositions(), current_positions);
    EXPECT_EQ(position_holding->GetDesiredState().positions, current_positions);
  }
}

TEST(PositionHoldingTest, WithoutCommandJoints) {
  PositionHoldingTestImpl(false, false);
}

TEST(PositionHoldingTest, WithCommandJoints) {
  PositionHoldingTestImpl(true, false);
}

TEST(PositionHoldingTest, OpenLoopControl) {
  PositionHoldingTestImpl(false, true);
}


void VelocityHoldingTestImpl(bool use_command_joints) {
  TestInput input(use_command_joints);

  const auto velocity_holding = std::make_shared<VelocityHolding>();
  ASSERT_TRUE(velocity_holding->Init(input.node, kSourceName, input.accessor.get()));
  ASSERT_TRUE(velocity_holding->Configure(input.joints_info));

  const auto command_interfaces = velocity_holding->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : input.command_joints) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, command_interfaces);
  }

  const auto state_interfaces = velocity_holding->GetStateInterfaces();
  EXPECT_TRUE(state_interfaces.empty());

  ASSERT_TRUE(velocity_holding->Activate(input.accessor->GetLoanedCommandInterfaces(),
                                         input.accessor->GetLoanedStateInterfaces()));

  const auto time = input.node->now();

  // Always zero speed command
  trajectory_msgs::msg::JointTrajectoryPoint previous_desired_state_;
  velocity_holding->ReadAndUpdate(time, kUpdatePeriod, previous_desired_state_);
  velocity_holding->WriteCommand();
  const std::vector<double> expected_velocities(input.command_joints.size(), 0.0);
  EXPECT_EQ(input.accessor->GetCommandVelocities(), expected_velocities);

  const auto desired_state = velocity_holding->GetDesiredState();
  EXPECT_TRUE(desired_state.positions.empty());
  EXPECT_EQ(desired_state.velocities, expected_velocities);

  ValidateCommonProperties(velocity_holding, time);
}


TEST(VelocityHoldingTest, WithoutCommandJoints) {
  VelocityHoldingTestImpl(false);
}

TEST(VelocityHoldingTest, WithCommandJoints) {
  VelocityHoldingTestImpl(true);
}

void PositionVelocityHoldingTestImpl(bool use_command_joints, bool open_loop_control) {
  TestInput input(use_command_joints, open_loop_control);

  const auto position_velocity_holding = std::make_shared<PositionVelocityHolding>();
  ASSERT_TRUE(position_velocity_holding->Init(input.node, kSourceName, input.accessor.get()));
  ASSERT_TRUE(position_velocity_holding->Configure(input.joints_info));

  const auto command_interfaces = position_velocity_holding->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 4);
  for (const auto& joint_name : input.command_joints) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, command_interfaces);
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, command_interfaces);
  }

  const auto state_interfaces = position_velocity_holding->GetStateInterfaces();
  EXPECT_EQ(state_interfaces.size(), 2);
  for (const auto& joint_name : kJointNames) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_POSITION, state_interfaces);
  }

  ASSERT_TRUE(position_velocity_holding->Activate(input.accessor->GetLoanedCommandInterfaces(),
                                                  input.accessor->GetLoanedStateInterfaces()));

  // Since previous_desired_state_ is empty, the position command is the current value
  // Speed command is always zero
  const std::vector<double> current_positions = {1.0, 2.0};
  input.accessor->SetStatePositions(current_positions);

  const auto time = input.node->now();

  trajectory_msgs::msg::JointTrajectoryPoint previous_desired_state_;
  position_velocity_holding->ReadAndUpdate(time, kUpdatePeriod, previous_desired_state_);
  position_velocity_holding->WriteCommand();
  EXPECT_EQ(input.accessor->GetCommandPositions(), current_positions);
  const std::vector<double> expected_velocities(input.command_joints.size(), 0.0);
  EXPECT_EQ(input.accessor->GetCommandVelocities(), expected_velocities);

  const auto desired_state = position_velocity_holding->GetDesiredState();
  EXPECT_EQ(desired_state.positions, current_positions);
  EXPECT_EQ(desired_state.velocities, expected_velocities);

  ValidateCommonProperties(position_velocity_holding, time);

  // If open_loop_control is enabled and previous_desired_state_ is not empty, obtain position command value from previous_desired_state_
  // Speed command is always zero
  previous_desired_state_.positions = {3.0, 4.0};
  previous_desired_state_.velocities = {0.5, 0.5};
  position_velocity_holding->ReadAndUpdate(time, kUpdatePeriod, previous_desired_state_);
  position_velocity_holding->WriteCommand();
  if (open_loop_control) {
    EXPECT_EQ(input.accessor->GetCommandPositions(), previous_desired_state_.positions);
    EXPECT_EQ(position_velocity_holding->GetDesiredState().positions, previous_desired_state_.positions);
  } else {
    EXPECT_EQ(input.accessor->GetCommandPositions(), current_positions);
    EXPECT_EQ(position_velocity_holding->GetDesiredState().positions, current_positions);
  }
  EXPECT_EQ(input.accessor->GetCommandVelocities(), expected_velocities);
  EXPECT_EQ(position_velocity_holding->GetDesiredState().velocities, expected_velocities);
}

TEST(PositionVelocityHoldingTest, WithoutCommandJoints) {
  PositionVelocityHoldingTestImpl(false, false);
}

TEST(PositionVelocityHoldingTest, WithCommandJoints) {
  PositionVelocityHoldingTestImpl(true, false);
}

TEST(PositionVelocityHoldingTest, OpenLoopControl) {
  PositionVelocityHoldingTestImpl(false, true);
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
