/// Copyright (C) 2026 Toyota Motor Corporation
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
      command_interfaces_.emplace_back(hardware_interface::CommandInterface(
          command_joints[i], hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]));
    }
  }

  std::vector<hardware_interface::LoanedCommandInterface> GetLoanedCommandInterfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    for (auto& command_interface : command_interfaces_) {
      loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface));
    }
    return loaned_command_interfaces;
  }

  std::vector<double> GetCommandVelocities() const {
    return command_velocities_;
  }

  void SetCommand(size_t index, double command_value) override {
    command_interfaces_[index].set_value(command_value);
  }

  double GetState([[maybe_unused]] size_t index) const override {
    return 0.0;
  }

 private:
  std::vector<double> command_velocities_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
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

  // トピックが飛ぶ前は指令値なし
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
  // 飛んできたトピックが指令値になる
  const auto command_time = client_node_->now();
  PublishCommand(command_time, kJointNames, {1.0, 2.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->WriteCommand();
  EXPECT_EQ(accessor_->GetCommandVelocities(), std::vector<double>({1.0, 2.0}));

  const auto desired_state = command_source_->GetDesiredState();
  EXPECT_TRUE(desired_state.positions.empty());
  EXPECT_EQ(desired_state.velocities, std::vector<double>({1.0, 2.0}));

  // 更新時刻はトピックのheaderの時刻
  EXPECT_EQ(command_source_->GetLastCommandTime(), command_time);

  // 制御モードはパラメータで指定したもの
  const auto target_control_mode = command_source_->GetTargetControlMode();
  EXPECT_EQ(target_control_mode.size(), 2);
  EXPECT_EQ(target_control_mode[0], -1);
  EXPECT_EQ(target_control_mode[1], -2);

  // 優先度はパラメータで指定したもの
  EXPECT_EQ(command_source_->GetPriority(), 42);

  // 割り込みでリセットされる
  command_source_->Preempt();
  EXPECT_FALSE(command_source_->HasCommand());

  command_source_->ReadAndUpdate(client_node_->now(), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

TEST_F(VelocityJogTopicTest, CommandTimeout) {
  // 指令値はトピックの時刻から一定時間経つとリセットされる
  const auto command_time = client_node_->now();
  PublishCommand(command_time, kJointNames, {1.0, 2.0});

  ASSERT_TRUE(WaitForCommand());

  command_source_->ReadAndUpdate(command_time + rclcpp::Duration::from_seconds(0.05), kUpdatePeriod, empty_state_);
  EXPECT_TRUE(command_source_->HasCommand());

  command_source_->ReadAndUpdate(command_time + rclcpp::Duration::from_seconds(0.15), kUpdatePeriod, empty_state_);
  EXPECT_FALSE(command_source_->HasCommand());
}

TEST_F(VelocityJogTopicTest, OneJointCommand) {
  // 一部の関節だけのトピックも受け付ける
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
  // 関節の順番が違うトピックも受け付ける
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
  // 時刻が0のトピックは，トピック受信時刻が更新時刻になる
  const auto before_time = controller_node_->now();

  PublishCommand(rclcpp::Time(0, 0), kJointNames, {1.0, 2.0});
  ASSERT_TRUE(WaitForCommand());

  const auto after_time = controller_node_->now();

  EXPECT_GE(command_source_->GetLastCommandTime(), before_time);
  EXPECT_LE(command_source_->GetLastCommandTime(), after_time);
}

TEST_F(VelocityJogTopicTest, InvalidJointName) {
  // 存在しない関節のトピックは無視される
  PublishCommand(client_node_->now(), {"invalid_joint"}, {1.0});
  EXPECT_FALSE(WaitForCommand());
}

TEST_F(VelocityJogTopicTest, EmptyCommand) {
  // 空のトピックは無視される
  PublishCommand(client_node_->now(), {}, {});
  EXPECT_FALSE(WaitForCommand());
}

TEST_F(VelocityJogTopicTest, MismatchedSizes) {
  // joint_namesとvelocitiesのサイズが違うトピックは無視される
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
