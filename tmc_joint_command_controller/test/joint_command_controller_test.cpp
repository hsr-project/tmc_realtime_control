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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_joint_command_controller/joint_command_controller.hpp>

#include "utils.hpp"

namespace {
const char* const kTestControllerName = "joint_command_controller";
const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const std::vector<std::string> kCommandJoints = {"command/joint1", "command/joint2"};
const std::vector<std::string> kSourceNames = {"source1", "source2"};
const std::vector<std::string> kSourceTypes = {"source1_type", "source2_type"};
const char* const kControlMode = "control_mode";
}

namespace tmc_joint_command_controller {

class CommandSourceMock : public IJointCommandSource {
 public:
  using Ptr = std::shared_ptr<CommandSourceMock>;

  MOCK_CONST_METHOD0(GetCommandInterfaces, std::vector<std::string>());
  MOCK_CONST_METHOD0(GetStateInterfaces, std::vector<std::string>());

  MOCK_METHOD3(Init, bool(const rclcpp_lifecycle::LifecycleNode::SharedPtr&, const std::string&, Accessor*));
  MOCK_METHOD1(Configure, bool(const JointsInfo::Ptr&));
  MOCK_METHOD2(Activate, bool(const std::vector<hardware_interface::LoanedCommandInterface>&,
                              const std::vector<hardware_interface::LoanedStateInterface>&));

  MOCK_METHOD3(ReadAndUpdate, void(const rclcpp::Time&,
                                   const rclcpp::Duration&,
                                   const trajectory_msgs::msg::JointTrajectoryPoint&));
  MOCK_CONST_METHOD0(HasCommand, bool());
  MOCK_METHOD0(WriteCommand, void());
  MOCK_CONST_METHOD0(GetDesiredState, trajectory_msgs::msg::JointTrajectoryPoint());

  MOCK_METHOD0(Preempt, void());

  MOCK_CONST_METHOD0(GetTargetControlMode, std::vector<double>());
  MOCK_CONST_METHOD0(GetPriority, int32_t());
  MOCK_CONST_METHOD0(GetLastCommandTime, rclcpp::Time());
};

void CheckDesiredState(const trajectory_msgs::msg::JointTrajectoryPoint& actual,
                       const trajectory_msgs::msg::JointTrajectoryPoint& expected) {
  ASSERT_EQ(actual.positions.size(), expected.positions.size());
  for (size_t i = 0; i < actual.positions.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.positions[i], expected.positions[i]);
  }
  ASSERT_EQ(actual.velocities.size(), expected.velocities.size());
  for (size_t i = 0; i < actual.velocities.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.velocities[i], expected.velocities[i]);
  }
}

void SetUsedSourceMock(const CommandSourceMock::Ptr& source_mock,
                       const rclcpp::Time& time,
                       const rclcpp::Duration& period,
                       const trajectory_msgs::msg::JointTrajectoryPoint& expected_previous_desired_state) {
  auto check_desired_state_func = [&expected_previous_desired_state](
      [[maybe_unused]] const rclcpp::Time& time,
      [[maybe_unused]] const rclcpp::Duration& period,
      const trajectory_msgs::msg::JointTrajectoryPoint& actual) {
    CheckDesiredState(actual, expected_previous_desired_state);
  };
  EXPECT_CALL(*source_mock, ReadAndUpdate(time, period, ::testing::_))
      .Times(1)
      .WillOnce(::testing::Invoke(check_desired_state_func));
  EXPECT_CALL(*source_mock, HasCommand()).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock, WriteCommand()).Times(1);
  EXPECT_CALL(*source_mock, Preempt()).Times(0);
}

void SetUnusedSourceMock(const CommandSourceMock::Ptr& source_mock,
                         const rclcpp::Time& time,
                         const rclcpp::Duration& period,
                         const trajectory_msgs::msg::JointTrajectoryPoint& expected_previous_desired_state,
                         bool has_command = true) {
  auto check_desired_state_func = [&expected_previous_desired_state](
      [[maybe_unused]] const rclcpp::Time& time,
      [[maybe_unused]] const rclcpp::Duration& period,
      const trajectory_msgs::msg::JointTrajectoryPoint& actual) {
    CheckDesiredState(actual, expected_previous_desired_state);
  };
  EXPECT_CALL(*source_mock, ReadAndUpdate(time, period, ::testing::_))
      .Times(1)
      .WillOnce(::testing::Invoke(check_desired_state_func));
  EXPECT_CALL(*source_mock, HasCommand()).WillRepeatedly(::testing::Return(has_command));
  EXPECT_CALL(*source_mock, WriteCommand()).Times(0);
  EXPECT_CALL(*source_mock, GetTargetControlMode()).Times(0);
  EXPECT_CALL(*source_mock, GetDesiredState()).Times(0);
  EXPECT_CALL(*source_mock, Preempt()).Times(1);
}

class CommandSourceLoaderMock : public CommandSourceLoader {
 public:
  IJointCommandSource::Ptr Create(const std::string& type) override {
    if (source_mocks_.find(type) == source_mocks_.end()) {
      return nullptr;
    }
    return source_mocks_[type];
  }

  void SetSourceMock(const std::string& type, const CommandSourceMock::Ptr& mock_source) {
    source_mocks_[type] = mock_source;
  }

 private:
  std::map<std::string, CommandSourceMock::Ptr> source_mocks_;
};

class JointCommandControllerTest : public ::testing::Test {
 protected:
  void SetUp();
  void InitController();
  void ConfigureController(const std::vector<std::string>& command_joints = kCommandJoints);
  void AssignInterfaces(const std::vector<std::string>& command_joints = kCommandJoints,
                        bool use_control_mode_setting = true);
  void ActivateController(const std::vector<std::string>& command_joints = kCommandJoints,
                          bool use_control_mode_setting = true);

  void WaitForControllerState();

  std::shared_ptr<CommandSourceLoaderMock> source_loader_mock_;
  CommandSourceMock::Ptr source_mock_1_;
  CommandSourceMock::Ptr source_mock_2_;

  std::shared_ptr<JointCommandController> controller_;

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>> subscription_;

  double command_value_unused_;
  std::vector<double> control_modes_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  std::vector<double> state_positions_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
};

void JointCommandControllerTest::SetUp() {
  source_loader_mock_ = std::make_shared<CommandSourceLoaderMock>();

  source_mock_1_ = std::make_shared<CommandSourceMock>();
  source_loader_mock_->SetSourceMock(kSourceTypes[0], source_mock_1_);

  source_mock_2_ = std::make_shared<CommandSourceMock>();
  source_loader_mock_->SetSourceMock(kSourceTypes[1], source_mock_2_);

  controller_ = std::make_shared<JointCommandController>(source_loader_mock_);

  client_node_ = rclcpp::Node::make_shared("test_client");
  subscription_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, std::string(kTestControllerName) + "/controller_state");
}

void JointCommandControllerTest::InitController() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("command_joints", kCommandJoints),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
    rclcpp::Parameter("command_source_names", kSourceNames),
    rclcpp::Parameter(kSourceNames[0] + ".type", kSourceTypes[0]),
    rclcpp::Parameter(kSourceNames[1] + ".type", kSourceTypes[1]),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);
}

void JointCommandControllerTest::ConfigureController(const std::vector<std::string>& command_joints) {
  JointsInfo::Ptr joints_info_1;
  EXPECT_CALL(*source_mock_1_, Init(controller_->get_node(), kSourceNames[0], controller_.get()))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_))
      .Times(1).WillOnce(::testing::DoAll(::testing::SaveArg<0>(&joints_info_1), ::testing::Return(true)));

  JointsInfo::Ptr joints_info_2;
  EXPECT_CALL(*source_mock_2_, Init(controller_->get_node(), kSourceNames[1], controller_.get()))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_))
      .Times(1).WillOnce(::testing::DoAll(::testing::SaveArg<0>(&joints_info_2), ::testing::Return(true)));

  ASSERT_EQ(controller_->configure().label(), "inactive");

  EXPECT_EQ(joints_info_1->names(), kJointNames);
  EXPECT_EQ(joints_info_1->command_joints(), command_joints);

  EXPECT_EQ(joints_info_2->names(), kJointNames);
  EXPECT_EQ(joints_info_2->command_joints(), command_joints);
}

void JointCommandControllerTest::AssignInterfaces(const std::vector<std::string>& command_joints,
                                                  bool use_control_mode_setting) {
  EXPECT_CALL(*source_mock_1_, GetCommandInterfaces())
      .Times(1).WillOnce(::testing::Return(std::vector<std::string>(
          {command_joints[0] + "/" + hardware_interface::HW_IF_POSITION,
           command_joints[1] + "/" + hardware_interface::HW_IF_POSITION})));
  EXPECT_CALL(*source_mock_1_, GetStateInterfaces())
      .Times(1).WillOnce(::testing::Return(std::vector<std::string>(
          {kJointNames[0] + "/" + hardware_interface::HW_IF_POSITION,
           kJointNames[1] + "/" + hardware_interface::HW_IF_POSITION})));

  EXPECT_CALL(*source_mock_2_, GetCommandInterfaces())
      .Times(1).WillOnce(::testing::Return(std::vector<std::string>(
          {command_joints[0] + "/" + hardware_interface::HW_IF_VELOCITY,
           command_joints[1] + "/" + hardware_interface::HW_IF_VELOCITY})));
  EXPECT_CALL(*source_mock_2_, GetStateInterfaces())
      .Times(1).WillOnce(::testing::Return(std::vector<std::string>(
          {kJointNames[0] + "/" + hardware_interface::HW_IF_POSITION,
           kJointNames[1] + "/" + hardware_interface::HW_IF_POSITION})));

  const auto command_interface_configuration = controller_->command_interface_configuration();
  EXPECT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  if (use_control_mode_setting) {
    EXPECT_EQ(command_interface_configuration.names.size(), 6);
  } else {
    EXPECT_EQ(command_interface_configuration.names.size(), 4);
  }
  for (const auto& name : command_joints) {
    AssertIn(name + "/" + hardware_interface::HW_IF_POSITION, command_interface_configuration.names);
    AssertIn(name + "/" + hardware_interface::HW_IF_VELOCITY, command_interface_configuration.names);
    if (use_control_mode_setting) {
      AssertIn(name + "/" + kControlMode, command_interface_configuration.names);
    }
  }

  const auto state_interface_configuration = controller_->state_interface_configuration();
  EXPECT_EQ(state_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state_interface_configuration.names.size(), 2);
  AssertIn(kJointNames[0] + "/" + hardware_interface::HW_IF_POSITION, state_interface_configuration.names);
  AssertIn(kJointNames[1] + "/" + hardware_interface::HW_IF_POSITION, state_interface_configuration.names);

  if (use_control_mode_setting) {
    control_modes_.resize(command_joints.size(), 0.0);
  }
  command_interfaces_.clear();
  for (size_t i = 0; i < command_joints.size(); ++i) {
    // The position and velocity command values are dummy variables. Writing is done via IJointCommandSource::WriteCommand, but since this is a mock, nothing will be written.
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        command_joints[i], hardware_interface::HW_IF_POSITION, &command_value_unused_));
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        command_joints[i], hardware_interface::HW_IF_VELOCITY, &command_value_unused_));
    if (use_control_mode_setting) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          command_joints[i], kControlMode, &control_modes_[i]));
    }
  }
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  for (auto& command_interface : command_interfaces_) {
    loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface, nullptr));
  }

  state_positions_.resize(kJointNames.size(), 0.0);
  state_interfaces_.clear();
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        kJointNames[i], hardware_interface::HW_IF_POSITION, &state_positions_[i]));
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
  for (auto& state_interface : state_interfaces_) {
    loaned_state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(state_interface));
  }

  controller_->assign_interfaces(std::move(loaned_command_interfaces), std::move(loaned_state_interfaces));
}

void JointCommandControllerTest::ActivateController(const std::vector<std::string>& command_joints,
                                                    bool use_control_mode_setting) {
  auto check_interfaces_func = [command_joints, use_control_mode_setting](
      const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
      const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    if (use_control_mode_setting) {
      EXPECT_EQ(command_interfaces.size(), 6);
    } else {
      EXPECT_EQ(command_interfaces.size(), 4);
    }
    std::vector<std::string> command_interface_names;
    for (const auto& command_interface : command_interfaces) {
      command_interface_names.push_back(command_interface.get_name());
    }
    for (const auto& name : command_joints) {
      AssertIn(name + "/" + hardware_interface::HW_IF_POSITION, command_interface_names);
      AssertIn(name + "/" + hardware_interface::HW_IF_VELOCITY, command_interface_names);
      if (use_control_mode_setting) {
        AssertIn(name + "/" + kControlMode, command_interface_names);
      }
    }

    EXPECT_EQ(state_interfaces.size(), 2);
    std::vector<std::string> state_interface_names;
    for (const auto& state_interface : state_interfaces) {
      state_interface_names.push_back(state_interface.get_name());
    }
    AssertIn(kJointNames[0] + "/" + hardware_interface::HW_IF_POSITION, state_interface_names);
    AssertIn(kJointNames[1] + "/" + hardware_interface::HW_IF_POSITION, state_interface_names);
    return true;
  };
  EXPECT_CALL(*source_mock_1_, Activate(::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Invoke(check_interfaces_func));
  EXPECT_CALL(*source_mock_2_, Activate(::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Invoke(check_interfaces_func));

  ASSERT_EQ(controller_->get_node()->activate().label(), "active");
}

void JointCommandControllerTest::WaitForControllerState() {
  subscription_->reset();

  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (subscription_->count() < 1 && client_node_->now() < timeout) {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(subscription_->count(), 1);
}

TEST_F(JointCommandControllerTest, UseHighPriorityCommandSource) {
  InitController();
  ConfigureController();
  AssignInterfaces();
  ActivateController();

  // If both are valid, the one with higher priority will be used.
  const auto time = controller_->get_node()->now();
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto empty_state = trajectory_msgs::msg::JointTrajectoryPoint();

  const std::vector<double> control_mode = {1.0, 2.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  desired_state.positions = {0.1, 0.2};

  state_positions_ = {-0.1, -0.2};

  SetUsedSourceMock(source_mock_1_, time, period, empty_state);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));
  EXPECT_CALL(*source_mock_1_, GetTargetControlMode()).Times(1).WillRepeatedly(::testing::Return(control_mode));
  EXPECT_CALL(*source_mock_1_, GetDesiredState()).Times(1).WillRepeatedly(::testing::Return(desired_state));

  SetUnusedSourceMock(source_mock_2_, time, period, empty_state);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(0));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));

  EXPECT_EQ(controller_->update(time, period), controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(control_modes_[0], control_mode[0]);
  EXPECT_DOUBLE_EQ(control_modes_[1], control_mode[1]);

  WaitForControllerState();

  const std::vector<double> zero_vector(kJointNames.size(), 0.0);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  EXPECT_EQ(msg.reference.positions, desired_state.positions);
  EXPECT_EQ(msg.reference.velocities, zero_vector);
  EXPECT_EQ(msg.reference.effort, zero_vector);
  EXPECT_EQ(msg.feedback.positions, state_positions_);
  EXPECT_TRUE(msg.feedback.velocities.empty());
  EXPECT_TRUE(msg.feedback.effort.empty());
  EXPECT_EQ(msg.error.positions, std::vector<double>({0.2, 0.4}));
  EXPECT_TRUE(msg.error.velocities.empty());
  EXPECT_TRUE(msg.error.effort.empty());

  // On the second time, the previous desired_state is passed as previous_desired_state.
  SetUsedSourceMock(source_mock_1_, time + period, period, desired_state);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time + period));
  EXPECT_CALL(*source_mock_1_, GetTargetControlMode()).Times(1).WillRepeatedly(::testing::Return(control_mode));
  EXPECT_CALL(*source_mock_1_, GetDesiredState()).Times(1).WillRepeatedly(::testing::Return(desired_state));

  SetUnusedSourceMock(source_mock_2_, time + period, period, desired_state);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(0));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time + period));

  EXPECT_EQ(controller_->update(time + period, period), controller_interface::return_type::OK);
}

TEST_F(JointCommandControllerTest, UnuseControlModeSetting) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("command_joints", kCommandJoints),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", false),
    rclcpp::Parameter("command_source_names", kSourceNames),
    rclcpp::Parameter(kSourceNames[0] + ".type", kSourceTypes[0]),
    rclcpp::Parameter(kSourceNames[1] + ".type", kSourceTypes[1]),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);

  ConfigureController();
  AssignInterfaces(kCommandJoints, false);
  ActivateController(kCommandJoints, false);

  // The controller will operate even if control_mode_setting is not used.
  const auto time = controller_->get_node()->now();
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto empty_state = trajectory_msgs::msg::JointTrajectoryPoint();

  const std::vector<double> control_mode = {1.0, 2.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  desired_state.positions = {0.1, 0.2};

  state_positions_ = {-0.1, -0.2};

  SetUsedSourceMock(source_mock_1_, time, period, empty_state);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));
  EXPECT_CALL(*source_mock_1_, GetTargetControlMode()).Times(1).WillRepeatedly(::testing::Return(control_mode));
  EXPECT_CALL(*source_mock_1_, GetDesiredState()).Times(1).WillRepeatedly(::testing::Return(desired_state));

  SetUnusedSourceMock(source_mock_2_, time, period, empty_state);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(0));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));

  EXPECT_EQ(controller_->update(time, period), controller_interface::return_type::OK);

  WaitForControllerState();

  const std::vector<double> zero_vector(kJointNames.size(), 0.0);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  EXPECT_EQ(msg.reference.positions, desired_state.positions);
  EXPECT_EQ(msg.reference.velocities, zero_vector);
  EXPECT_EQ(msg.reference.effort, zero_vector);
  EXPECT_EQ(msg.feedback.positions, state_positions_);
  EXPECT_TRUE(msg.feedback.velocities.empty());
  EXPECT_TRUE(msg.feedback.effort.empty());
  EXPECT_EQ(msg.error.positions, std::vector<double>({0.2, 0.4}));
  EXPECT_TRUE(msg.error.velocities.empty());
  EXPECT_TRUE(msg.error.effort.empty());
}

TEST_F(JointCommandControllerTest, UseNewerCommandSourceWhenSamePriority) {
  InitController();
  ConfigureController();
  AssignInterfaces();
  ActivateController();

  // If both are valid and have the same priority, the most recent one will be used.
  const auto time = controller_->get_node()->now();
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto empty_state = trajectory_msgs::msg::JointTrajectoryPoint();

  const std::vector<double> control_mode = {1.0, 2.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  desired_state.positions = {0.1, 0.2};

  SetUsedSourceMock(source_mock_1_, time, period, empty_state);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));
  EXPECT_CALL(*source_mock_1_, GetTargetControlMode()).Times(1).WillRepeatedly(::testing::Return(control_mode));
  EXPECT_CALL(*source_mock_1_, GetDesiredState()).Times(1).WillRepeatedly(::testing::Return(desired_state));

  SetUnusedSourceMock(source_mock_2_, time, period, empty_state);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time - period));

  EXPECT_EQ(controller_->update(time, period), controller_interface::return_type::OK);
}

TEST_F(JointCommandControllerTest, UseSingleValidSource) {
  InitController();
  ConfigureController();
  AssignInterfaces();
  ActivateController();

  // If only one is valid, that one will be used.
  const auto time = controller_->get_node()->now();
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto empty_state = trajectory_msgs::msg::JointTrajectoryPoint();

  const std::vector<double> control_mode = {1.0, 2.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  desired_state.positions = {0.1, 0.2};

  SetUsedSourceMock(source_mock_1_, time, period, empty_state);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));
  EXPECT_CALL(*source_mock_1_, GetTargetControlMode()).Times(1).WillRepeatedly(::testing::Return(control_mode));
  EXPECT_CALL(*source_mock_1_, GetDesiredState()).Times(1).WillRepeatedly(::testing::Return(desired_state));

  SetUnusedSourceMock(source_mock_2_, time, period, empty_state, false);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(2));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));

  EXPECT_EQ(controller_->update(time, period), controller_interface::return_type::OK);
}

TEST_F(JointCommandControllerTest, NoValidSource) {
  InitController();
  ConfigureController();
  AssignInterfaces();
  ActivateController();

  const auto time = controller_->get_node()->now();
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const auto empty_state = trajectory_msgs::msg::JointTrajectoryPoint();

  SetUnusedSourceMock(source_mock_1_, time, period, empty_state, false);
  EXPECT_CALL(*source_mock_1_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_1_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));

  SetUnusedSourceMock(source_mock_2_, time, period, empty_state, false);
  EXPECT_CALL(*source_mock_2_, GetPriority()).WillRepeatedly(::testing::Return(1));
  EXPECT_CALL(*source_mock_2_, GetLastCommandTime()).WillRepeatedly(::testing::Return(time));

  EXPECT_EQ(controller_->update(time, period), controller_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(control_modes_[0], 0.0);
  EXPECT_DOUBLE_EQ(control_modes_[1], 0.0);

  WaitForControllerState();

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  EXPECT_EQ(msg.reference.positions, std::vector<double>({0.0, 0.0}));
}

TEST_F(JointCommandControllerTest, NoCommandJoints) {
  // If command_joints is not specified, the same as joints will be used, so it will succeed even after configure.
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
    rclcpp::Parameter("command_source_names", kSourceNames),
    rclcpp::Parameter(kSourceNames[0] + ".type", kSourceTypes[0]),
    rclcpp::Parameter(kSourceNames[1] + ".type", kSourceTypes[1]),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);

  ConfigureController(kJointNames);
  AssignInterfaces(kJointNames);
  ActivateController(kJointNames);
}

TEST_F(JointCommandControllerTest, CommandJointsSizeMismatch) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("command_joints", std::vector<std::string>({kCommandJoints[0]})),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
    rclcpp::Parameter("command_source_names", kSourceNames),
    rclcpp::Parameter(kSourceNames[0] + ".type", kSourceTypes[0]),
    rclcpp::Parameter(kSourceNames[1] + ".type", kSourceTypes[1]),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);

  // If the size of command_joints differs from the size of joints, an error will occur during configure.
  EXPECT_CALL(*source_mock_1_, Init(::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_2_, Init(::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_)).Times(0);
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, EmptyJoints) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", std::vector<std::string>()),
    rclcpp::Parameter("command_joints", std::vector<std::string>()),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
    rclcpp::Parameter("command_source_names", kSourceNames),
    rclcpp::Parameter(kSourceNames[0] + ".type", kSourceTypes[0]),
    rclcpp::Parameter(kSourceNames[1] + ".type", kSourceTypes[1]),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);

  // If joints is empty, an error will occur during configure.
  EXPECT_CALL(*source_mock_1_, Init(::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_2_, Init(::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_)).Times(0);
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, NoCommandSource) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("command_joints", kCommandJoints),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, CommandSourceCreateFailure) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("command_joints", kCommandJoints),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_control_mode_setting", true),
    rclcpp::Parameter("control_mode_interface_name", kControlMode),
    rclcpp::Parameter("command_source_names", std::vector<std::string>({"invalid_source"})),
    rclcpp::Parameter("invalid_source.type", "invalid_type"),
  };
  ASSERT_EQ(controller_->init(kTestControllerName, "", 0, "", options), controller_interface::return_type::OK);

  // If the creation of CommandSource fails, an error will occur during configure.
  EXPECT_CALL(*source_mock_1_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_)).WillRepeatedly(::testing::Return(true));
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, CommandSourceInitFailure) {
  InitController();

  // If the initialization of CommandSource fails, an error will occur during configure.
  EXPECT_CALL(*source_mock_1_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_)).WillRepeatedly(::testing::Return(true));
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, CommandSourceConfigureFailure) {
  InitController();

  // If the configuration of CommandSource fails, an error will occur during configure.
  EXPECT_CALL(*source_mock_1_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_1_, Configure(::testing::_)).WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(*source_mock_2_, Init(::testing::_, ::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*source_mock_2_, Configure(::testing::_)).WillRepeatedly(::testing::Return(true));
  ASSERT_EQ(controller_->configure().label(), "unconfigured");
}

TEST_F(JointCommandControllerTest, CommandSourceActivateFailure) {
  InitController();
  ConfigureController();

  // If the activation of CommandSource fails, an error will occur during activate.
  EXPECT_CALL(*source_mock_1_, Activate(::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(*source_mock_2_, Activate(::testing::_, ::testing::_)).WillRepeatedly(::testing::Return(true));
  ASSERT_EQ(controller_->get_node()->activate().label(), "unconfigured");
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
