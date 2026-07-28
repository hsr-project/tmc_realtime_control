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
/// @file exxx_drive_mode_controller-test.cpp
/// @brief Test of the controller that changes the drive mode

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tmc_realtime_controllers/exxx_drive_mode_controller.hpp>

#include "exxx_drive_mode_hardware_stub.hpp"

namespace tmc_realtime_controllers {

class ExxxDriveModeControllerTest : public ::testing::Test {
 public:
  void SetUp() override {
    controller_ = std::make_shared<TestableExxxDriveModeController>();
    controller_node_ = controller_->get_node();

    if (controller_node_->has_parameter("joints")) {
      std::vector<rclcpp::Parameter> params;
      params.push_back(rclcpp::Parameter("joints", std::vector<std::string>{ "arm_lift_joint", "arm_flex_joint" }));
      controller_node_->set_parameters(params);
    } else {
      controller_node_->declare_parameter<std::vector<std::string> >("joints", { "arm_lift_joint", "arm_flex_joint" });
    }

    if (controller_node_->has_parameter("publish_rate")) {
      std::vector<rclcpp::Parameter> params;
      params.push_back(rclcpp::Parameter("publish_rate", 10.0));
      controller_node_->set_parameters(params);
    } else {
      controller_node_->declare_parameter<double>("publish_rate", 10.0);
    }

    controller_->InitImpl();

    std::vector<std::string> joint_names = { "arm_lift_joint", "arm_flex_joint" };
    hardware_ = std::make_shared<HardwareStub>(joint_names);

    controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));

    EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = rclcpp::Node::make_shared("test_node");

    change_mode_srv_client_ =
        client_node_->create_client<tmc_control_msgs::srv::ChangeExxxDriveMode>("change_drive_mode");

    EXPECT_TRUE(change_mode_srv_client_->wait_for_service());
  }

  void TearDown() override {
    controller_->release_interfaces();
  }

  void spin_some_thread(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) {
    while (!do_interrupt_) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  tmc_control_msgs::msg::JointExxxDriveMode WaitForDriveModeMessage() {
    tmc_control_msgs::msg::JointExxxDriveMode msg;
    std::function<void(const tmc_control_msgs::msg::JointExxxDriveMode::SharedPtr)> callback =
        [&msg](const tmc_control_msgs::msg::JointExxxDriveMode::SharedPtr message) { msg = *message; };
    auto subscription = client_node_->create_subscription<tmc_control_msgs::msg::JointExxxDriveMode>(
        "drive_mode", 1, callback);
    for (auto i = 0; i < 100; ++i) {
      EXPECT_EQ(controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.1)),
                controller_interface::return_type::OK);
      rclcpp::spin_some(client_node_);
      if (msg.drive_modes.size() > 0) {
        return msg;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return msg;
  }

 protected:
  TestableExxxDriveModeController::Ptr controller_;
  std::thread spinner_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> controller_node_;
  HardwareStub::Ptr hardware_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  bool do_interrupt_;
  rclcpp::Client<tmc_control_msgs::srv::ChangeExxxDriveMode>::SharedPtr change_mode_srv_client_;
};
TEST_F(ExxxDriveModeControllerTest, CheckPublication) {
  // Waiting for subscription
  const auto msg = WaitForDriveModeMessage();
  ASSERT_EQ(2, msg.drive_modes.size());
  EXPECT_EQ("arm_lift_joint", msg.drive_modes[0].joint);
  EXPECT_EQ("arm_flex_joint", msg.drive_modes[1].joint);
  EXPECT_EQ(0, msg.drive_modes[0].value);
  EXPECT_EQ(0, msg.drive_modes[1].value);
}
TEST_F(ExxxDriveModeControllerTest, CheckServiceSuccess) {
  auto request = std::make_shared<tmc_control_msgs::srv::ChangeExxxDriveMode::Request>();
  request->drive_mode_request.drive_modes.resize(1);
  request->drive_mode_request.drive_modes[0].joint = "arm_lift_joint";
  request->drive_mode_request.drive_modes[0].value = 2;
  auto response = change_mode_srv_client_->async_send_request(request);
  hardware_->drive_mode[0]->set_current(2.0);
  // Perform service wait.
  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&ExxxDriveModeControllerTest::spin_some_thread, this, controller_node_->get_node_base_interface()));
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(controller_->update(controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Perform topic wait.
  do_interrupt_ = true;
  spinner_.join();

  ASSERT_TRUE(rclcpp::spin_until_future_complete(client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);
  std::shared_ptr<tmc_control_msgs::srv::ChangeExxxDriveMode::Response> response_value = response.get();
  ASSERT_TRUE(response_value->success);

  const auto msg = WaitForDriveModeMessage();
  ASSERT_EQ(2, msg.drive_modes.size());
  EXPECT_EQ("arm_lift_joint", msg.drive_modes[0].joint);
  EXPECT_EQ("arm_flex_joint", msg.drive_modes[1].joint);
  EXPECT_EQ(2, msg.drive_modes[0].value);
  EXPECT_EQ(0, msg.drive_modes[1].value);
}
TEST_F(ExxxDriveModeControllerTest, InvalidJoint) {
  auto request = std::make_shared<tmc_control_msgs::srv::ChangeExxxDriveMode::Request>();
  request->drive_mode_request.drive_modes.resize(1);
  request->drive_mode_request.drive_modes[0].joint = "invalid_drive";
  request->drive_mode_request.drive_modes[0].value = 1;

  auto result = change_mode_srv_client_->async_send_request(request);

  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&ExxxDriveModeControllerTest::spin_some_thread, this, controller_node_->get_node_base_interface()));

  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(controller_->update(controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();

  ASSERT_TRUE(rclcpp::spin_until_future_complete(client_node_, result) == rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_FALSE(result.get()->success);
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
