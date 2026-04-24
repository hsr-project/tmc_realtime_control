/// Copyright (C) 2026 Toyota Motor Corporation
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
      command_interfaces_.emplace_back(hardware_interface::CommandInterface(
          command_joints[i], hardware_interface::HW_IF_POSITION, &command_positions_[i]));
      command_interfaces_.emplace_back(hardware_interface::CommandInterface(
          command_joints[i], hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]));
    }

    state_positions_.resize(kJointNames.size());
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      state_interfaces_.emplace_back(hardware_interface::StateInterface(
          kJointNames[i], hardware_interface::HW_IF_POSITION, &state_positions_[i]));
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> GetLoanedCommandInterfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    for (auto& command_interface : command_interfaces_) {
      loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface));
    }
    return loaned_command_interfaces;
  }

  std::vector<hardware_interface::LoanedStateInterface> GetLoanedStateInterfaces() {
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    for (auto& state_interface : state_interfaces_) {
      loaned_state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(state_interface));
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
    command_interfaces_[index].set_value(command_value);
  }

  double GetState(size_t index) const override {
    return state_interfaces_[index].get_value();
  }

 private:
  std::vector<double> command_positions_;
  std::vector<double> command_velocities_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<double> state_positions_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
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
  // 常に指令値あり
  EXPECT_TRUE(joint_command_source->HasCommand());

  // 制御モードの変更はしない
  EXPECT_TRUE(joint_command_source->GetTargetControlMode().empty());

  // 優先度はとても低いがint32の最低ではない
  EXPECT_NE(joint_command_source->GetPriority(), std::numeric_limits<int32_t>::min());
  EXPECT_LT(joint_command_source->GetPriority(), -1e8);

  // 更新時刻は不定だが，ReadAndUpdateよりは古い
  EXPECT_LE(joint_command_source->GetLastCommandTime(), update_time);

  // Preemptは作用も副作用もない，テスト不可能だが一応呼んでおく
  joint_command_source->Preempt();
}


// 値をパラメータ化したテストをするほどではないと思う
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

  // previous_desired_state_が空なので現在値が指令値
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

  // open_loop_controlが有効かつprevious_desired_state_が空でない場合は、previous_desired_state_が指令値
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

  // 常にゼロ速度指令
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

  // previous_desired_state_が空なので位置指令は現在値
  // 速度指令は常にゼロ
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

  // open_loop_controlが有効かつprevious_desired_state_が空でない場合は、previous_desired_state_から位置指令値を取得
  // 速度指令は常にゼロ
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
