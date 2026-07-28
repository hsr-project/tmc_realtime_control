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

#include <tmc_joint_command_controller/joints_info.hpp>

#include "utils.hpp"

namespace tmc_joint_command_controller {

void TestAddCommandPositionInterfaces(const JointsInfo& joints_info,
                                      const std::vector<std::string>& expected_joints) {
  // To test that it is "Add", insert an appropriate value at the 0th position
  std::vector<std::string> interfaces = {"dummy"};
  joints_info.AddCommandPositionInterfaces(interfaces);

  EXPECT_EQ(interfaces.size(), expected_joints.size() + 1);
  AssertIn("dummy", interfaces);
  for (const auto& joint : expected_joints) {
    AssertIn(joint + "/" + hardware_interface::HW_IF_POSITION, interfaces);
  }
}

void TestAddCommandVelocityInterfaces(const JointsInfo& joints_info,
                                      const std::vector<std::string>& expected_joints) {
  std::vector<std::string> interfaces = {"dummy"};
  joints_info.AddCommandVelocityInterfaces(interfaces);

  EXPECT_EQ(interfaces.size(), expected_joints.size() + 1);
  AssertIn("dummy", interfaces);
  for (const auto& joint : expected_joints) {
    AssertIn(joint + "/" + hardware_interface::HW_IF_VELOCITY, interfaces);
  }
}

void TestAddStatePositionInterfaces(const JointsInfo& joints_info,
                                    const std::vector<std::string>& expected_joints) {
  std::vector<std::string> interfaces = {"dummy"};
  joints_info.AddStatePositionInterfaces(interfaces);
  EXPECT_EQ(interfaces.size(), expected_joints.size() + 1);
  AssertIn("dummy", interfaces);
  for (const auto& joint : expected_joints) {
    AssertIn(joint + "/" + hardware_interface::HW_IF_POSITION, interfaces);
  }
}

void TestAddStateVelocityInterfaces(const JointsInfo& joints_info,
                                    const std::vector<std::string>& expected_joints) {
  std::vector<std::string> interfaces = {"dummy"};
  joints_info.AddStateVelocityInterfaces(interfaces);

  EXPECT_EQ(interfaces.size(), expected_joints.size() + 1);
  AssertIn("dummy", interfaces);
  for (const auto& joint : expected_joints) {
    AssertIn(joint + "/" + hardware_interface::HW_IF_VELOCITY, interfaces);
  }
}

void TestAddStateEffortInterfaces(const JointsInfo& joints_info,
                                  const std::vector<std::string>& expected_joints) {
  std::vector<std::string> interfaces = {"dummy"};
  joints_info.AddStateEffortInterfaces(interfaces);

  EXPECT_EQ(interfaces.size(), expected_joints.size() + 1);
  AssertIn("dummy", interfaces);
  for (const auto& joint : expected_joints) {
    AssertIn(joint + "/" + hardware_interface::HW_IF_EFFORT, interfaces);
  }
}

TEST(JointsInfoTest, ConstructFromJointNames) {
  const std::vector<std::string> joint_names = {"joint1", "joint2"};
  JointsInfo joints_info(joint_names);

  EXPECT_TRUE(joints_info.IsValid(rclcpp::get_logger("test")));
  EXPECT_EQ(joints_info.names(), joint_names);
  EXPECT_EQ(joints_info.command_joints(), joint_names);

  TestAddCommandPositionInterfaces(joints_info, joint_names);
  TestAddCommandVelocityInterfaces(joints_info, joint_names);
  TestAddStatePositionInterfaces(joints_info, joint_names);
  TestAddStateVelocityInterfaces(joints_info, joint_names);
  TestAddStateEffortInterfaces(joints_info, joint_names);
}

TEST(JointsInfoTest, ConstructFromNode) {
  const std::vector<std::string> joint_names = {"joint1", "joint2"};

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("joints", joint_names)};
  const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

  JointsInfo joints_info(node);

  EXPECT_TRUE(joints_info.IsValid(node->get_logger()));
  EXPECT_EQ(joints_info.names(), joint_names);
  EXPECT_EQ(joints_info.command_joints(), joint_names);

  TestAddCommandPositionInterfaces(joints_info, joint_names);
  TestAddCommandVelocityInterfaces(joints_info, joint_names);
  TestAddStatePositionInterfaces(joints_info, joint_names);
  TestAddStateVelocityInterfaces(joints_info, joint_names);
  TestAddStateEffortInterfaces(joints_info, joint_names);
}

TEST(JointsInfoTest, ConstructFromNodeWithCommandJoints) {
  const std::vector<std::string> joint_names = {"joint1", "joint2"};
  const std::vector<std::string> command_joints = {"command/joint1", "command/joint2"};

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("joints", joint_names),
      rclcpp::Parameter("command_joints", command_joints)};
  const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

  JointsInfo joints_info(node);

  EXPECT_TRUE(joints_info.IsValid(node->get_logger()));
  EXPECT_EQ(joints_info.names(), joint_names);
  EXPECT_EQ(joints_info.command_joints(), command_joints);

  TestAddCommandPositionInterfaces(joints_info, command_joints);
  TestAddCommandVelocityInterfaces(joints_info, command_joints);
  TestAddStatePositionInterfaces(joints_info, joint_names);
  TestAddStateVelocityInterfaces(joints_info, joint_names);
  TestAddStateEffortInterfaces(joints_info, joint_names);
}

TEST(JointsInfoTest, InvalidJointsInfo) {
  {
    // The number of joints given as an argument is zero
    JointsInfo joints_info(std::vector<std::string>({}));
    EXPECT_FALSE(joints_info.IsValid(rclcpp::get_logger("test")));
  }

  {
    // The number of joints retrieved from the node is zero
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {rclcpp::Parameter("joints", std::vector<std::string>({}))};

    const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

    JointsInfo joints_info(node);
    EXPECT_FALSE(joints_info.IsValid(node->get_logger()));
  }

  {
    // Mismatch in the number of joints in command_joints
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {
        rclcpp::Parameter("joints", std::vector<std::string>({"joint1", "joint2"})),
        rclcpp::Parameter("command_joints", std::vector<std::string>({"command/joint1"}))};

    const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

    JointsInfo joints_info(node);
    EXPECT_FALSE(joints_info.IsValid(node->get_logger()));
  }

  {
    // An empty string is included
    JointsInfo joints_info(std::vector<std::string>({"joint1", ""}));
    EXPECT_FALSE(joints_info.IsValid(rclcpp::get_logger("test")));
  }

  {
    // Duplicate joint names
    JointsInfo joints_info(std::vector<std::string>({"joint1", "joint1"}));
    EXPECT_FALSE(joints_info.IsValid(rclcpp::get_logger("test")));
  }
}

}  // namespace tmc_joint_command_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
