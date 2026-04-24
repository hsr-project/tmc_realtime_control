/// Copyright (C) 2026 Toyota Motor Corporation
#include <gtest/gtest.h>

#include <tmc_joint_command_controller/sources/joint_trajectory_topic.hpp>

#include "joint_trajectory_test_common.hpp"
#include "utils.hpp"

namespace tmc_joint_command_controller {

class JointTrajectoryTopicTest : public JointTrajectoryCommandSourceTest {
 protected:
  void InitializeCommandSource() override;
  bool InitializeCommandClient() override;
  void SendTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
};

void JointTrajectoryTopicTest::InitializeCommandSource() {
  command_source_ = std::make_shared<JointTrajectoryTopic>();
}

bool JointTrajectoryTopicTest::InitializeCommandClient() {
  command_pub_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      kControllerName + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (command_pub_->get_subscription_count() == 0) {
    if (client_node_->now() > timeout) {
      return false;
    }
  }
  return true;
}

void JointTrajectoryTopicTest::SendTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  command_pub_->publish(trajectory);
}


TEST_F(JointTrajectoryTopicTest, WithOpenLoopControl) {
  TestWithOpenLoopControl();
}

TEST_F(JointTrajectoryTopicTest, GlobalOpenLoopControl) {
  TestGlobalOpenLoopControl();
}

TEST_F(JointTrajectoryTopicTest, TimeZeroTrajectory) {
  TestTimeZeroTrajectory();
}

TEST_F(JointTrajectoryTopicTest, WithoutOpenLoopControl) {
  TestWithoutOpenLoopControl();
}

TEST_F(JointTrajectoryTopicTest, UseVelocityState) {
  TestUseVelocityState();
}

TEST_F(JointTrajectoryTopicTest, OneJointTrajectory) {
  TestOneJointTrajectory();
}

TEST_F(JointTrajectoryTopicTest, UnorderedJointTrajectory) {
  TestUnorderedJointTrajectory();
}

TEST_F(JointTrajectoryTopicTest, PositionVelocityTrajectory) {
  TestPositionVelocityTrajectory();
}

TEST_F(JointTrajectoryTopicTest, PositionVelocityAccerationTrajectory) {
  TestPositionVelocityAccerationTrajectory();
}

TEST_F(JointTrajectoryTopicTest, PathToleranceViolated) {
  TestPathToleranceViolated();
}

TEST_F(JointTrajectoryTopicTest, GoalToleranceViolatedWithoutVelocityState) {
  TestGoalToleranceViolatedWithoutVelocityState();
}

TEST_F(JointTrajectoryTopicTest, GoalToleranceWithVelocityState) {
  TestGoalToleranceWithVelocityState();
}

TEST_F(JointTrajectoryTopicTest, MismatchedPositionsSize) {
  TestMismatchedPositionsSize();
}

TEST_F(JointTrajectoryTopicTest, MismatchedVelocitiesSize) {
  TestMismatchedVelocitiesSize();
}

TEST_F(JointTrajectoryTopicTest, MismatchedAccelerationsSize) {
  TestMismatchedAccelerationsSize();
}

TEST_F(JointTrajectoryTopicTest, NonIncreasingTimeFromStart) {
  TestNonIncreasingTimeFromStart();
}

TEST_F(JointTrajectoryTopicTest, WithInvalidJointName) {
  TestWithInvalidJointName();
}

TEST_F(JointTrajectoryTopicTest, EmptyJointNames) {
  TestEmptyJointNames();
}

TEST_F(JointTrajectoryTopicTest, EmptyTrajectoryPoints) {
  TestEmptyTrajectoryPoints();
}

TEST_F(JointTrajectoryTopicTest, WithCommandJoints) {
  TestWithCommandJoints();
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
