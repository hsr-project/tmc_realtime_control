/// Copyright (C) 2026 Toyota Motor Corporation
#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_joint_command_controller/state_publisher.hpp>

#include "utils.hpp"

namespace {
const std::vector<std::string> kJointNames = {"joint1", "joint2"};
const std::vector<std::string> kStateInterfaces = {hardware_interface::HW_IF_POSITION,
                                                   hardware_interface::HW_IF_VELOCITY,
                                                   hardware_interface::HW_IF_EFFORT};
}  // namespace

namespace tmc_joint_command_controller {

class AccessorMock : public Accessor {
 public:
  AccessorMock() {
    for (const auto& joint_name : kJointNames) {
      for (const auto& interface : kStateInterfaces) {
        states_[joint_name][interface] = 0.0;
        state_interfaces_.emplace_back(hardware_interface::StateInterface(
            joint_name, interface, &states_[joint_name][interface]));
      }
    }
  }

  std::vector<hardware_interface::LoanedStateInterface> GetLoanedStateInterfaces() {
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    for (auto& state_interface : state_interfaces_) {
      loaned_state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(state_interface));
    }
    return loaned_state_interfaces;
  }

  void SetState(const std::string& joint_name, const std::string& interface, double value) {
    states_[joint_name][interface] = value;
  }

  void SetCommand([[maybe_unused]] size_t index, [[maybe_unused]] double command_value) override {
    FAIL() << "SetCommand should not be called in StatePublisherTest";
  }

  double GetState(size_t index) const override {
    return state_interfaces_[index].get_value();
  }

 private:
  std::map<std::string, std::map<std::string, double>> states_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
};

class StatePublisherTest : public ::testing::Test {
 protected:
  void SetUp(const std::vector<std::string>& state_interfaces);

  void WaitForMessage(const trajectory_msgs::msg::JointTrajectoryPoint& desired,
                      uint32_t target_count,
                      double timeout_sec = 1.0);

  rclcpp_lifecycle::LifecycleNode::SharedPtr server_node_;
  std::shared_ptr<StatePublisher> state_publisher_;

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>> subscription_;

  JointsInfo::Ptr joints_info_;
  std::shared_ptr<AccessorMock> accessor_mock_;
};

void StatePublisherTest::SetUp(const std::vector<std::string>& state_interfaces) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("state_interfaces", state_interfaces)};

  server_node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_server", options);
  joints_info_ = std::make_shared<JointsInfo>(kJointNames);
  accessor_mock_ = std::make_shared<AccessorMock>();

  state_publisher_ = std::make_shared<StatePublisher>(server_node_, joints_info_, accessor_mock_.get());

  // "Add"であることをテストするために,0個目に適当な値を入れておく
  std::vector<std::string> state_interfaces_added = {"dummy"};
  state_publisher_->AddStateInterfaces(state_interfaces_added);

  EXPECT_EQ(state_interfaces_added.size(), 1 + kJointNames.size() * state_interfaces.size());
  AssertIn("dummy", state_interfaces_added);
  for (const auto& joint_name : kJointNames) {
    for (const auto& interface : state_interfaces) {
      AssertIn(joint_name + "/" + interface, state_interfaces_added);
    }
  }

  state_publisher_->Activate(accessor_mock_->GetLoanedStateInterfaces());

  client_node_ = rclcpp::Node::make_shared("test_client");
  subscription_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, "test_server/controller_state");
}

void StatePublisherTest::WaitForMessage(const trajectory_msgs::msg::JointTrajectoryPoint& desired,
                                        uint32_t target_count,
                                        double timeout_sec) {
  const auto start = server_node_->now();
  const auto timeout = start + rclcpp::Duration::from_seconds(timeout_sec);
  while (subscription_->count() < target_count && client_node_->now() < timeout) {
    state_publisher_->Publish(server_node_->now(), desired);
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto end = server_node_->now();
  EXPECT_GE(subscription_->count(), target_count);

  const auto& msg = subscription_->last_msg();
  EXPECT_LT(start, msg.header.stamp);
  EXPECT_GT(end, msg.header.stamp);
}

TEST_F(StatePublisherTest, UseAllStateInterfaces) {
  SetUp(kStateInterfaces);

  trajectory_msgs::msg::JointTrajectoryPoint desired;
  desired.positions = {1.0, 2.0};
  desired.velocities = {3.0, 4.0};
  desired.effort = {5.0, 6.0};

  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_POSITION, -1.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_POSITION, -2.0);
  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_VELOCITY, -3.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_VELOCITY, -4.0);
  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_EFFORT, -5.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_EFFORT, -6.0);

  WaitForMessage(desired, 1);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  EXPECT_EQ(msg.reference.positions, desired.positions);
  EXPECT_EQ(msg.reference.velocities, desired.velocities);
  EXPECT_EQ(msg.reference.effort, desired.effort);
  EXPECT_EQ(msg.feedback.positions, std::vector<double>({-1.0, -2.0}));
  EXPECT_EQ(msg.feedback.velocities, std::vector<double>({-3.0, -4.0}));
  EXPECT_EQ(msg.feedback.effort, std::vector<double>({-5.0, -6.0}));
  EXPECT_EQ(msg.error.positions, std::vector<double>({2.0, 4.0}));
  EXPECT_EQ(msg.error.velocities, std::vector<double>({6.0, 8.0}));
  EXPECT_EQ(msg.error.effort, std::vector<double>({10.0, 12.0}));
}

TEST_F(StatePublisherTest, NoStateInterfaces) {
  SetUp({});

  trajectory_msgs::msg::JointTrajectoryPoint desired;
  desired.positions = {1.0, 2.0};
  desired.velocities = {3.0, 4.0};
  desired.effort = {5.0, 6.0};

  WaitForMessage(desired, 1);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  EXPECT_EQ(msg.reference.positions, desired.positions);
  EXPECT_EQ(msg.reference.velocities, desired.velocities);
  EXPECT_EQ(msg.reference.effort, desired.effort);
  EXPECT_TRUE(msg.feedback.positions.empty());
  EXPECT_TRUE(msg.feedback.velocities.empty());
  EXPECT_TRUE(msg.feedback.effort.empty());
  EXPECT_TRUE(msg.error.positions.empty());
  EXPECT_TRUE(msg.error.velocities.empty());
  EXPECT_TRUE(msg.error.effort.empty());
}

TEST_F(StatePublisherTest, NoDesiredState) {
  SetUp(kStateInterfaces);

  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_POSITION, -1.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_POSITION, -2.0);
  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_VELOCITY, -3.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_VELOCITY, -4.0);
  accessor_mock_->SetState("joint1", hardware_interface::HW_IF_EFFORT, -5.0);
  accessor_mock_->SetState("joint2", hardware_interface::HW_IF_EFFORT, -6.0);

  WaitForMessage(trajectory_msgs::msg::JointTrajectoryPoint(), 1);

  const auto& msg = subscription_->last_msg();
  EXPECT_EQ(msg.joint_names, kJointNames);
  const std::vector<double> zero_vector(kJointNames.size(), 0.0);
  EXPECT_EQ(msg.reference.positions, zero_vector);
  EXPECT_EQ(msg.reference.velocities, zero_vector);
  EXPECT_EQ(msg.reference.effort, zero_vector);
  EXPECT_EQ(msg.feedback.positions, std::vector<double>({-1.0, -2.0}));
  EXPECT_EQ(msg.feedback.velocities, std::vector<double>({-3.0, -4.0}));
  EXPECT_EQ(msg.feedback.effort, std::vector<double>({-5.0, -6.0}));
  EXPECT_EQ(msg.error.positions, zero_vector);
  EXPECT_EQ(msg.error.velocities, zero_vector);
  EXPECT_EQ(msg.error.effort, zero_vector);
}

TEST_F(StatePublisherTest, PublishEveryUpdate) {
  SetUp(kStateInterfaces);

  const auto start = client_node_->now();
  WaitForMessage(trajectory_msgs::msg::JointTrajectoryPoint(), 100);
  const auto end = client_node_->now();

  // WaitForMessageのsleepが1ms
  EXPECT_NEAR((end - start).seconds(), 0.1, 0.05);
}
}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
