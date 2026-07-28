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

#include <tmc_joint_command_controller/sources/velocity_jog_topic.hpp>

#include "utils.hpp"

namespace tmc_joint_command_controller {
const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const std::vector<std::string> kCommandJoints = {"command/joint1", "command/joint2"};

const std::string kControllerName = "joint_command_controller";  // NOLINT
const std::string kSourceName = "velocity_jog_topic";  // NOLINT

const rclcpp::Duration kUpdatePeriod = rclcpp::Duration::from_seconds(0.01);
}

namespace tmc_joint_command_controller {

class AccessorMock : public Accessor {
 public:
  explicit AccessorMock(const std::vector<std::string>& command_joints) {
    command_velocities_.resize(command_joints.size());
    for (size_t i = 0; i < command_joints.size(); ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          command_joints[i], hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]));
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> GetLoanedCommandInterfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    for (auto& command_interface : command_interfaces_) {
      loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface, nullptr));
    }
    return loaned_command_interfaces;
  }

  std::vector<double> GetCommandVelocities() const {
    return command_velocities_;
  }

  void SetCommand(size_t index, double command_value) override {
    SetCommandInterfaceValue(rclcpp::get_logger("rclcpp"), command_interfaces_[index], command_value);
  }

  double GetState([[maybe_unused]] size_t index) const override {
    return 0.0;
  }

 private:
  std::vector<double> command_velocities_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
};

class VelocityJogTopicTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  JointsInfo::Ptr joints_info_;
  std::shared_ptr<AccessorMock> accessor_;

  IJointCommandSource::Ptr command_source_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr command_pub_;

  trajectory_msgs::msg::JointTrajectoryPoint empty_state_;

  void PublishCommand(const rclcpp::Time& time,
                      const std::vector<std::string>& names,
                      const std::vector<double>& values);

  bool WaitForCommand();
};

void VelocityJogTopicTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("joints", kJointNames),
      rclcpp::Parameter(kSourceName + ".priority", 42),
      rclcpp::Parameter(kSourceName + ".target_control_mode", -1),
      rclcpp::Parameter(kSourceName + "." + kJointNames[1] + ".target_control_mode", -2)};
  controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(kControllerName, options);
  joints_info_ = std::make_shared<JointsInfo>(controller_node_);
  accessor_ = std::make_shared<AccessorMock>(kJointNames);

  command_source_ = std::make_shared<VelocityJogTopic>();
  ASSERT_TRUE(command_source_->Init(controller_node_, kSourceName, accessor_.get()));
  ASSERT_TRUE(command_source_->Configure(joints_info_));

  const auto command_interfaces = command_source_->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : kJointNames) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, command_interfaces);
  }

  const auto state_interfaces = command_source_->GetStateInterfaces();
  EXPECT_TRUE(state_interfaces.empty());

  ASSERT_TRUE(command_source_->Activate(accessor_->GetLoanedCommandInterfaces(), {}));

  client_node_ = std::make_shared<rclcpp::Node>("client_node");
  command_pub_ = client_node_->create_publisher<control_msgs::msg::JointJog>(
      kControllerName + "/joint_velocity", rclcpp::SystemDefaultsQoS());
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (command_pub_->get_subscription_count() == 0) {
    if (client_node_->now() > timeout) {
      FAIL() << "Timeout while waiting for subscription to be ready";
    }
  }

  // No command value before the topic jumps
  EXPECT_FALSE(command_source_->HasCommand());
}

void VelocityJogTopicTest::PublishCommand(const rclcpp::Time& time,
                                          const std::vector<std::string>& names,
                                          const std::vector<double>& values) {
  control_msgs::msg::JointJog msg;
  msg.header.stamp = time;
  msg.joint_names = names;
  msg.velocities = values;
  command_pub_->publish(msg);
}

bool VelocityJogTopicTest::WaitForCommand() {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.3);
  while (!command_source_->HasCommand()) {
    if (client_node_->now() > timeout) {
      break;
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_->get_node_base_interface());
    command_source_->ReadAndUpdate(controller_node_->now(), kUpdatePeriod, empty_state_);
  }
  return command_source_->HasCommand();
}

TEST_F(VelocityJogTopicTest, AllJointsCommand) {
  // The incoming topic becomes the command value
  const auto command_time = client_node_->now();
  PublishCommand(command_time, kJointNames, {1.0, 2.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->WriteCommand();
  EXPECT_EQ(accessor_->GetCommandVelocities(), std::vector<double>({1.0, 2.0}));

  const auto desired_state = command_source_->GetDesiredState();
  EXPECT_TRUE(desired_state.positions.empty());
  EXPECT_EQ(desired_state.velocities, std::vector<double>({1.0, 2.0}));

  // Update time is the time in the topic's header
  EXPECT_EQ(command_source_->GetLastCommandTime(), command_time);

  // Control mode is specified by the parameter
  const auto target_control_mode = command_source_->GetTargetControlMode();
  EXPECT_EQ(target_control_mode.size(), 2);
  EXPECT_EQ(target_control_mode[0], -1);
  EXPECT_EQ(target_control_mode[1], -2);

  // Priority is specified by the parameter
  EXPECT_EQ(command_source_->GetPriority(), 42);

  // Reset by interrupt
  command_source_->Preempt();
  EXPECT_FALSE(command_source_->HasCommand());

  command_source_->ReadAndUpdate(client_node_->now(), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

TEST_F(VelocityJogTopicTest, CommandTimeout) {
  // Command value is reset after a certain time from the topic's timestamp
  const auto command_time = client_node_->now();
  PublishCommand(command_time, kJointNames, {1.0, 2.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->ReadAndUpdate(command_time + rclcpp::Duration::from_seconds(0.05), kUpdatePeriod, empty_state_);
  EXPECT_TRUE(command_source_->HasCommand());

  command_source_->ReadAndUpdate(command_time + rclcpp::Duration::from_seconds(0.15), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

TEST_F(VelocityJogTopicTest, OneJointCommand) {
  // Accepts topics for only some joints
  const auto command_time = client_node_->now();
  PublishCommand(command_time, {kJointNames[0]}, {1.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->WriteCommand();
  EXPECT_EQ(accessor_->GetCommandVelocities(), std::vector<double>({1.0, 0.0}));

  const auto desired_state = command_source_->GetDesiredState();
  EXPECT_TRUE(desired_state.positions.empty());
  EXPECT_EQ(desired_state.velocities, std::vector<double>({1.0, 0.0}));
}

TEST_F(VelocityJogTopicTest, UnorderedJointCommand) {
  // Accepts topics with different joint orders
  const auto command_time = client_node_->now();
  PublishCommand(command_time, {kJointNames[1], kJointNames[0]}, {2.0, 1.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->WriteCommand();
  EXPECT_EQ(accessor_->GetCommandVelocities(), std::vector<double>({1.0, 2.0}));

  const auto desired_state = command_source_->GetDesiredState();
  EXPECT_TRUE(desired_state.positions.empty());
  EXPECT_EQ(desired_state.velocities, std::vector<double>({1.0, 2.0}));
}

TEST_F(VelocityJogTopicTest, TimeZeroCommand) {
  // Topics with a timestamp of 0 will have their update time set to the topic reception time
  const auto before_time = controller_node_->now();

  PublishCommand(rclcpp::Time(0, 0), kJointNames, {1.0, 2.0});
  ASSERT_TRUE(WaitForCommand());

  const auto after_time = controller_node_->now();

  EXPECT_GE(command_source_->GetLastCommandTime(), before_time);
  EXPECT_LE(command_source_->GetLastCommandTime(), after_time);
}

TEST_F(VelocityJogTopicTest, InvalidJointName) {
  // Topics for non-existent joints are ignored
  PublishCommand(client_node_->now(), {"invalid_joint"}, {1.0});
  EXPECT_FALSE(WaitForCommand());
}

TEST_F(VelocityJogTopicTest, EmptyCommand) {
  // Empty topics are ignored
  PublishCommand(client_node_->now(), {}, {});
  EXPECT_FALSE(WaitForCommand());
}

TEST_F(VelocityJogTopicTest, MismatchedSizes) {
  // Topics with mismatched sizes of joint_names and velocities are ignored
  PublishCommand(client_node_->now(), {kJointNames[0]}, {1.0, 2.0});
  EXPECT_FALSE(WaitForCommand());
}

TEST_F(VelocityJogTopicTest, WithCommandJoints) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("joints", kJointNames),
      rclcpp::Parameter("command_joints", kCommandJoints),
      rclcpp::Parameter(kSourceName + ".priority", 42),
      rclcpp::Parameter(kSourceName + ".target_control_mode", -1),
      rclcpp::Parameter(kSourceName + "." + kJointNames[1] + ".target_control_mode", -2)};
  controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(kControllerName, options);
  joints_info_ = std::make_shared<JointsInfo>(controller_node_);
  accessor_ = std::make_shared<AccessorMock>(kCommandJoints);

  command_source_ = std::make_shared<VelocityJogTopic>();
  ASSERT_TRUE(command_source_->Init(controller_node_, kSourceName, accessor_.get()));
  ASSERT_TRUE(command_source_->Configure(joints_info_));

  const auto command_interfaces = command_source_->GetCommandInterfaces();
  EXPECT_EQ(command_interfaces.size(), 2);
  for (const auto& joint_name : kCommandJoints) {
    AssertIn(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, command_interfaces);
  }

  const auto state_interfaces = command_source_->GetStateInterfaces();
  EXPECT_TRUE(state_interfaces.empty());

  ASSERT_TRUE(command_source_->Activate(accessor_->GetLoanedCommandInterfaces(), {}));
}

TEST_F(VelocityJogTopicTest, NotPositiveCommandTimeout) {
  auto test_func = [](double command_timeout) {
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {
        rclcpp::Parameter("joints", kJointNames),
        rclcpp::Parameter(kSourceName + ".command_timeout", command_timeout)};
    const auto controller_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(kControllerName, options);

    const auto joints_info = std::make_shared<JointsInfo>(controller_node);
    const auto accessor = std::make_shared<AccessorMock>(kJointNames);

    const auto command_source = std::make_shared<VelocityJogTopic>();
    ASSERT_TRUE(command_source->Init(controller_node, kSourceName, accessor.get()));
    EXPECT_FALSE(command_source->Configure(joints_info));
  };

  {
    SCOPED_TRACE("command_timeout is negative");
    test_func(-1.0);
  }
  {
    SCOPED_TRACE("command_timeout is zero");
    test_func(0.0);
  }
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
