// Copyright (c) 2026 Toyota Motor Corporation
#include <gtest/gtest.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_joint_command_controller/basic_joint_command_controller.hpp>

#include "utils.hpp"

namespace {
const char* const kTestControllerName = "joint_command_controller";
const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const rclcpp::Duration kUpdatePeriod = rclcpp::Duration::from_seconds(0.01);
}

namespace tmc_joint_command_controller {

class BasicJointCommandControllerTest : public ::testing::Test {
 protected:
  using ActionType = control_msgs::action::FollowJointTrajectory;
  using ClientType = rclcpp_action::Client<ActionType>;

  void SetUp() override;

  bool WaitForStatus(std::shared_future<ClientType::GoalHandle::SharedPtr> goal_handle_future);
  void WaitForControllerState(const rclcpp::Time& target_stamp,
                              std::function<bool()> validation_func = []() { return true; });

  void ConfigureController(const rclcpp::NodeOptions& options);
  void ConfigureController();
  void ActivateController(bool use_velocity_state = true);

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>> subscription_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr command_position_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr command_velocity_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_trajectory_pub_;
  ClientType::SharedPtr trajectory_action_client_;

  std::shared_ptr<JointCommandController> controller_;

  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::vector<double> command_drive_mode_;

  std::vector<double> state_position_;
  std::vector<double> state_velocity_;

  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
};

void BasicJointCommandControllerTest::SetUp() {
  client_node_ = rclcpp::Node::make_shared("test_client");
  subscription_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, std::string(kTestControllerName) + "/controller_state");
  command_position_pub_ = client_node_->create_publisher<control_msgs::msg::JointJog>(
      std::string(kTestControllerName) + "/joint_position", rclcpp::SystemDefaultsQoS());
  command_velocity_pub_ = client_node_->create_publisher<control_msgs::msg::JointJog>(
      std::string(kTestControllerName) + "/joint_velocity", rclcpp::SystemDefaultsQoS());
  command_trajectory_pub_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kTestControllerName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  trajectory_action_client_ = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kTestControllerName) + "/follow_joint_trajectory");

  controller_ = std::make_shared<BasicJointCommandController>();
}

void BasicJointCommandControllerTest::WaitForControllerState(const rclcpp::Time& target_stamp,
                                                             std::function<bool()> validation_func) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (client_node_->now() < timeout) {
    if (rclcpp::Time(subscription_->last_msg().header.stamp) == target_stamp && validation_func()) {
      return;
    }
    EXPECT_EQ(controller_->update(target_stamp, kUpdatePeriod), controller_interface::return_type::OK);
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "Timeout while waiting for controller state with target_stamp: " << target_stamp.seconds();
}

bool BasicJointCommandControllerTest::WaitForStatus(
    std::shared_future<ClientType::GoalHandle::SharedPtr> goal_handle_future) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.3);
  while (rclcpp::ok()) {
    if (client_node_->now() > timeout) {
      return false;
    }
    if (goal_handle_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      const auto goal_handle = goal_handle_future.get();
      if (goal_handle && goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
        return true;
      } else {
        return false;
      }
    }
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }
  return false;
}

void BasicJointCommandControllerTest::ConfigureController(const rclcpp::NodeOptions& options) {
  ASSERT_EQ(controller_->init(kTestControllerName, "", options), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().label(), "inactive");
}

void BasicJointCommandControllerTest::ConfigureController() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
  };
  ConfigureController(options);
}

void BasicJointCommandControllerTest::ActivateController(bool use_velocity_state) {
  const auto command_interface_configuration = controller_->command_interface_configuration();
  EXPECT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(command_interface_configuration.names.size(), 6);
  for (const auto& name : kJointNames) {
    AssertIn(name + "/" + hardware_interface::HW_IF_POSITION, command_interface_configuration.names);
    AssertIn(name + "/" + hardware_interface::HW_IF_VELOCITY, command_interface_configuration.names);
    AssertIn(name + "/command_drive_mode", command_interface_configuration.names);
  }

  const auto state_interface_configuration = controller_->state_interface_configuration();
  EXPECT_EQ(state_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  if (use_velocity_state) {
    EXPECT_EQ(state_interface_configuration.names.size(), 4);
  } else {
    EXPECT_EQ(state_interface_configuration.names.size(), 2);
  }
  for (const auto& name : kJointNames) {
    AssertIn(name + "/" + hardware_interface::HW_IF_POSITION, state_interface_configuration.names);
    if (use_velocity_state) {
      AssertIn(name + "/" + hardware_interface::HW_IF_VELOCITY, state_interface_configuration.names);
    }
  }

  command_position_.resize(kJointNames.size(), 0.0);
  command_velocity_.resize(kJointNames.size(), 0.0);
  command_drive_mode_.resize(kJointNames.size(), 0.0);

  command_interfaces_.clear();
  for (auto i = 0u; i < kJointNames.size(); ++i) {
    command_interfaces_.emplace_back(kJointNames[i], hardware_interface::HW_IF_POSITION, &command_position_[i]);
    command_interfaces_.emplace_back(kJointNames[i], hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]);
    command_interfaces_.emplace_back(kJointNames[i], "command_drive_mode", &command_drive_mode_[i]);
  }
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  for (auto& command_interface : command_interfaces_) {
    loaned_command_interfaces.emplace_back(command_interface);
  }

  state_position_ = {1.0, 2.0};
  state_velocity_ = {0.1, 0.2};
  state_interfaces_.clear();
  for (auto i = 0u; i < kJointNames.size(); ++i) {
    state_interfaces_.emplace_back(kJointNames[i], hardware_interface::HW_IF_POSITION, &state_position_[i]);
    if (use_velocity_state) {
      state_interfaces_.emplace_back(kJointNames[i], hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    }
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
  for (auto& state_interface : state_interfaces_) {
    loaned_state_interfaces.emplace_back(state_interface);
  }

  controller_->assign_interfaces(std::move(loaned_command_interfaces), std::move(loaned_state_interfaces));
  ASSERT_EQ(controller_->get_node()->activate().label(), "active");

  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (command_position_pub_->get_subscription_count() == 0 ||
         command_velocity_pub_->get_subscription_count() == 0 ||
         command_trajectory_pub_->get_subscription_count() == 0 ||
         !trajectory_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
    if (client_node_->now() > timeout) {
      FAIL() << "Timeout while waiting for server to be ready";
    }
  }
}

TEST_F(BasicJointCommandControllerTest, DefaultParameters) {
  ConfigureController();
  ActivateController();

  // 何も指令値がないので，現在位置を保つ指令値になる
  WaitForControllerState(controller_->get_node()->now());

  const std::vector<double> zero_vector(kJointNames.size(), 0.0);
  {
    const auto& msg = subscription_->last_msg();
    EXPECT_EQ(msg.joint_names, kJointNames);

    EXPECT_EQ(msg.reference.positions, state_position_);
    EXPECT_EQ(msg.reference.velocities, zero_vector);
    EXPECT_EQ(msg.reference.effort, zero_vector);

    EXPECT_EQ(msg.feedback.positions, state_position_);
    EXPECT_EQ(msg.feedback.velocities, state_velocity_);
    EXPECT_TRUE(msg.feedback.effort.empty());

    EXPECT_EQ(msg.error.positions, zero_vector);
    AssertEq(msg.error.velocities, std::vector<double>({-state_velocity_[0], -state_velocity_[1]}));
    EXPECT_TRUE(msg.error.effort.empty());

    EXPECT_DOUBLE_EQ(command_drive_mode_[0], 0.0);
    EXPECT_DOUBLE_EQ(command_drive_mode_[1], 0.0);
  }

  // 位置指令値
  control_msgs::msg::JointJog command_position_msg;
  command_position_msg.joint_names = kJointNames;
  command_position_msg.displacements = {0.5, 1.0};
  command_position_pub_->publish(command_position_msg);

  const auto position_command_stamp = controller_->get_node()->now();
  WaitForControllerState(position_command_stamp, [&]() {
    const auto& msg = subscription_->last_msg();
    return msg.reference.positions[0] == command_position_msg.displacements[0] &&
           msg.reference.positions[1] == command_position_msg.displacements[1];
  });
  {
    const auto& msg = subscription_->last_msg();
    EXPECT_EQ(msg.joint_names, kJointNames);

    EXPECT_EQ(msg.reference.positions, command_position_msg.displacements);
    EXPECT_EQ(msg.reference.velocities, zero_vector);
    EXPECT_EQ(msg.reference.effort, zero_vector);

    EXPECT_EQ(msg.feedback.positions, state_position_);
    EXPECT_EQ(msg.feedback.velocities, state_velocity_);
    EXPECT_TRUE(msg.feedback.effort.empty());

    AssertEq(msg.error.positions, std::vector<double>({-0.5, -1.0}));
    EXPECT_EQ(msg.error.velocities, zero_vector);
    EXPECT_TRUE(msg.error.effort.empty());

    EXPECT_DOUBLE_EQ(command_drive_mode_[0], 0.0);
    EXPECT_DOUBLE_EQ(command_drive_mode_[1], 0.0);
  }

  // 現在位置を保つ指令に戻る
  WaitForControllerState(position_command_stamp + rclcpp::Duration::from_seconds(1.0), [&]() {
    const auto& msg = subscription_->last_msg();
    return msg.reference.positions[0] == command_position_msg.displacements[0] &&
           msg.reference.positions[1] == command_position_msg.displacements[1] &&
           msg.reference.velocities.size() == 2;
  });
  {
    const auto& msg = subscription_->last_msg();
    EXPECT_EQ(msg.joint_names, kJointNames);

    // 位置指令値が維持されることを確認
    EXPECT_EQ(msg.reference.positions, command_position_msg.displacements);
    EXPECT_EQ(msg.reference.velocities, zero_vector);
    EXPECT_EQ(msg.reference.effort, zero_vector);

    EXPECT_EQ(msg.feedback.positions, state_position_);
    EXPECT_EQ(msg.feedback.velocities, state_velocity_);
    EXPECT_TRUE(msg.feedback.effort.empty());

    AssertEq(msg.error.positions, std::vector<double>({-0.5, -1.0}));
    AssertEq(msg.error.velocities, std::vector<double>({-state_velocity_[0], -state_velocity_[1]}));
    EXPECT_TRUE(msg.error.effort.empty());

    EXPECT_DOUBLE_EQ(command_drive_mode_[0], 0.0);
    EXPECT_DOUBLE_EQ(command_drive_mode_[1], 0.0);
  }

  // 速度指令値
  control_msgs::msg::JointJog command_velocity_msg;
  command_velocity_msg.joint_names = kJointNames;
  command_velocity_msg.velocities = {0.3, 0.4};
  command_velocity_pub_->publish(command_velocity_msg);

  WaitForControllerState(controller_->get_node()->now(), [&]() {
    const auto& msg = subscription_->last_msg();
    return msg.reference.velocities[0] == command_velocity_msg.velocities[0] &&
           msg.reference.velocities[1] == command_velocity_msg.velocities[1];
  });
  {
    const auto& msg = subscription_->last_msg();
    EXPECT_EQ(msg.joint_names, kJointNames);

    EXPECT_EQ(msg.reference.positions, zero_vector);
    EXPECT_EQ(msg.reference.velocities, command_velocity_msg.velocities);
    EXPECT_EQ(msg.reference.effort, zero_vector);

    EXPECT_EQ(msg.feedback.positions, state_position_);
    EXPECT_EQ(msg.feedback.velocities, state_velocity_);
    EXPECT_TRUE(msg.feedback.effort.empty());

    EXPECT_EQ(msg.error.positions, zero_vector);
    AssertEq(msg.error.velocities, std::vector<double>({0.2, 0.2}));
    EXPECT_TRUE(msg.error.effort.empty());

    EXPECT_DOUBLE_EQ(command_drive_mode_[0], 1.0);
    EXPECT_DOUBLE_EQ(command_drive_mode_[1], 1.0);
  }

  // 起動追従トピック
  state_position_ = {1.0, 2.0};
  state_velocity_ = {-1.0, -1.0};
  const auto trajectory_topic_command_time = controller_->get_node()->now();

  trajectory_msgs::msg::JointTrajectory command_trajectory_msg;
  command_trajectory_msg.header.stamp = trajectory_topic_command_time;
  command_trajectory_msg.joint_names = kJointNames;
  command_trajectory_msg.points.resize(1);
  command_trajectory_msg.points[0].positions = {2.0, 3.0};
  command_trajectory_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);
  command_trajectory_pub_->publish(command_trajectory_msg);

  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(controller_->update(trajectory_topic_command_time + kUpdatePeriod * i, kUpdatePeriod),
              controller_interface::return_type::OK);
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }

  constexpr double kPositionTolerance = 0.01;
  constexpr double kVelocityTolerance = 0.02;
  WaitForControllerState(trajectory_topic_command_time + rclcpp::Duration::from_seconds(0.5), [&]() {
    const auto& msg = subscription_->last_msg();
    if (std::abs(msg.reference.positions[0] - 1.5) < kPositionTolerance &&
        std::abs(msg.reference.positions[1] - 2.5) < kPositionTolerance) {
      return true;
    }
    return false;
  });
  {
    const auto& msg = subscription_->last_msg();
    EXPECT_EQ(msg.joint_names, kJointNames);

    AssertEq(msg.reference.positions, std::vector<double>({1.5, 2.5}), kPositionTolerance);
    AssertEq(msg.reference.velocities, std::vector<double>({1.0, 1.0}), kVelocityTolerance);
    EXPECT_EQ(msg.reference.effort, zero_vector);

    EXPECT_EQ(msg.feedback.positions, state_position_);
    EXPECT_EQ(msg.feedback.velocities, state_velocity_);
    EXPECT_TRUE(msg.feedback.effort.empty());

    AssertEq(msg.error.positions, std::vector<double>({0.5, 0.5}), kPositionTolerance);
    AssertEq(msg.error.velocities, std::vector<double>({2.0, 2.0}), kVelocityTolerance);
    EXPECT_TRUE(msg.error.effort.empty());

    EXPECT_DOUBLE_EQ(command_drive_mode_[0], 0.0);
    EXPECT_DOUBLE_EQ(command_drive_mode_[1], 0.0);
  }

  // 起動追従アクション，起動追従自体の確認がしたいわけでないので，アクションがAcceptedになることだけ確認する
  const auto trajectory_action_command_time = controller_->get_node()->now();
  ActionType::Goal goal;
  goal.trajectory = command_trajectory_msg;
  goal.trajectory.header.stamp = trajectory_action_command_time;
  const auto goal_handle_future = trajectory_action_client_->async_send_goal(goal);
  WaitForStatus(goal_handle_future);
}

TEST_F(BasicJointCommandControllerTest, ControlModeSetting) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("position_control_mode", -1),
    rclcpp::Parameter(kJointNames[0] + ".position_control_mode", -10),
    rclcpp::Parameter("velocity_control_mode", -2),
    rclcpp::Parameter(kJointNames[1] + ".velocity_control_mode", -100),
  };
  ConfigureController(options);
  ActivateController(true);

  // 位置指令値
  control_msgs::msg::JointJog command_position_msg;
  command_position_msg.joint_names = kJointNames;
  command_position_msg.displacements = {0.5, 1.0};
  command_position_pub_->publish(command_position_msg);

  WaitForControllerState(controller_->get_node()->now(), [&]() {
    const auto& msg = subscription_->last_msg();
    return msg.reference.positions[0] == command_position_msg.displacements[0] &&
           msg.reference.positions[1] == command_position_msg.displacements[1];
  });
  EXPECT_DOUBLE_EQ(command_drive_mode_[0], -10);
  EXPECT_DOUBLE_EQ(command_drive_mode_[1], -1);

  // 速度指令
  control_msgs::msg::JointJog command_velocity_msg;
  command_velocity_msg.joint_names = kJointNames;
  command_velocity_msg.velocities = {0.3, 0.4};
  command_velocity_pub_->publish(command_velocity_msg);

  WaitForControllerState(controller_->get_node()->now(), [&]() {
    const auto& msg = subscription_->last_msg();
    return msg.reference.velocities[0] == command_velocity_msg.velocities[0] &&
           msg.reference.velocities[1] == command_velocity_msg.velocities[1];
  });
  EXPECT_DOUBLE_EQ(command_drive_mode_[0], -2);
  EXPECT_DOUBLE_EQ(command_drive_mode_[1], -100);
}

TEST_F(BasicJointCommandControllerTest, WithtoutVelocityState) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("joints", kJointNames),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>({hardware_interface::HW_IF_POSITION})),
    rclcpp::Parameter("use_velocity_state_for_trajectory", false),
  };
  ConfigureController(options);
  ActivateController(false);

  // 起動追従トピック
  state_position_ = {1.0, 2.0};
  state_velocity_ = {0.0, 0.0};
  const auto trajectory_topic_command_time = controller_->get_node()->now();

  trajectory_msgs::msg::JointTrajectory command_trajectory_msg;
  command_trajectory_msg.header.stamp = trajectory_topic_command_time;
  command_trajectory_msg.joint_names = kJointNames;
  command_trajectory_msg.points.resize(1);
  command_trajectory_msg.points[0].positions = {2.0, 3.0};
  command_trajectory_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);
  command_trajectory_pub_->publish(command_trajectory_msg);

  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(controller_->update(trajectory_topic_command_time + kUpdatePeriod * i, kUpdatePeriod),
              controller_interface::return_type::OK);
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }

  constexpr double kPositionTolerance = 0.01;
  constexpr double kVelocityTolerance = 0.02;
  WaitForControllerState(trajectory_topic_command_time + rclcpp::Duration::from_seconds(0.5), [&]() {
    const auto& msg = subscription_->last_msg();
    if (std::abs(msg.reference.positions[0] - 1.5) < kPositionTolerance &&
        std::abs(msg.reference.positions[1] - 2.5) < kPositionTolerance) {
      return true;
    }
    return false;
  });

  const std::vector<double> zero_vector(kJointNames.size(), 0.0);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);

  AssertEq(msg.reference.positions, std::vector<double>({1.5, 2.5}), kPositionTolerance);
  AssertEq(msg.reference.velocities, std::vector<double>({1.0, 1.0}), kVelocityTolerance);
  EXPECT_EQ(msg.reference.effort, zero_vector);

  EXPECT_EQ(msg.feedback.positions, state_position_);
  EXPECT_TRUE(msg.feedback.velocities.empty());
  EXPECT_TRUE(msg.feedback.effort.empty());

  AssertEq(msg.error.positions, std::vector<double>({0.5, 0.5}), kPositionTolerance);
  EXPECT_TRUE(msg.error.velocities.empty());
  EXPECT_TRUE(msg.error.effort.empty());
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
