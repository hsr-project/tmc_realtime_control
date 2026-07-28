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
