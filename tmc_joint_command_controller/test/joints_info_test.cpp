/// Copyright (C) 2026 Toyota Motor Corporation
#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_joint_command_controller/joints_info.hpp>

#include "utils.hpp"

namespace tmc_joint_command_controller {

void TestAddCommandPositionInterfaces(const JointsInfo& joints_info,
                                      const std::vector<std::string>& expected_joints) {
  // "Add"であることをテストするために,0個目に適当な値を入れておく
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
    // 引数で与えた関節数がゼロ
    JointsInfo joints_info(std::vector<std::string>({}));
    EXPECT_FALSE(joints_info.IsValid(rclcpp::get_logger("test")));
  }

  {
    // ノードから取得した関節数がゼロ
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {rclcpp::Parameter("joints", std::vector<std::string>({}))};

    const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

    JointsInfo joints_info(node);
    EXPECT_FALSE(joints_info.IsValid(node->get_logger()));
  }

  {
    // command_jointsの関節数が不一致
    rclcpp::NodeOptions options;
    options.parameter_overrides() = {
        rclcpp::Parameter("joints", std::vector<std::string>({"joint1", "joint2"})),
        rclcpp::Parameter("command_joints", std::vector<std::string>({"command/joint1"}))};

    const auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test", options);

    JointsInfo joints_info(node);
    EXPECT_FALSE(joints_info.IsValid(node->get_logger()));
  }

  {
    // 空文字列が含まれている
    JointsInfo joints_info(std::vector<std::string>({"joint1", ""}));
    EXPECT_FALSE(joints_info.IsValid(rclcpp::get_logger("test")));
  }

  {
    // 関節名の重複
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
