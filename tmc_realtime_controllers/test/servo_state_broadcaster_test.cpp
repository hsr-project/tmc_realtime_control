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
/// @brief Test for the controller that changes the drive mode

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>

#include "servo_state_broadcaster_hardware_stub.hpp"

namespace tmc_realtime_controllers {

class ServoStateBroadcasterTest : public ::testing::Test {
 public:
  void SetUp() override {
    controller_ = std::make_shared<TestableServoStateBroadcaster>();
    controller_node_ = controller_->get_node();

    if (controller_node_->has_parameter("joints")) {
      std::vector<rclcpp::Parameter> params;
      params.push_back(rclcpp::Parameter("joints", std::vector<std::string>{ "arm_lift_joint", "arm_flex_joint" }));
      controller_node_->set_parameters(params);
    } else {
      controller_node_->declare_parameter<std::vector<std::string> >("joints", { "arm_lift_joint", "arm_flex_joint" });
    }

    // To eliminate the effect of multiple calls to update in WaitFor, set igain to zero
    if (controller_node_->has_parameter("publish_rate")) {
      std::vector<rclcpp::Parameter> params;
      params.push_back(rclcpp::Parameter("publish_rate", 30.0));
      controller_node_->set_parameters(params);
    } else {
      controller_node_->declare_parameter<double>("publish_rate", 30.0);
    }

    controller_->InitImpl();

    joint_names_ = { "arm_lift_joint", "arm_flex_joint" };
    hardware_ = std::make_shared<HardwareStub>(joint_names_);
    controller_->assign_interfaces({}, std::move(hardware_->state_interfaces));

    EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = rclcpp::Node::make_shared("test_node");

    servostate_subscription_ = client_node_->create_subscription<tmc_control_msgs::msg::ServoState>(
        "servo_states", rclcpp::SystemDefaultsQoS(),
        std::bind(&ServoStateBroadcasterTest::current_servo_state_callback, this, std::placeholders::_1));
  }

  void TearDown() override {
    controller_->release_interfaces();
  }

 protected:
  TestableServoStateBroadcaster::Ptr controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> controller_node_;
  HardwareStub::Ptr hardware_;
  rclcpp::Node::SharedPtr client_node_;
  std::thread spinner_;
  std::vector<std::string> joint_names_;
  rclcpp::Subscription<tmc_control_msgs::msg::ServoState>::SharedPtr servostate_subscription_;

  tmc_control_msgs::msg::ServoState servostate_msg_;
  bool is_recive_servo_sate_;
  double start_time_;

  void current_servo_state_callback(const tmc_control_msgs::msg::ServoState& msg) {
    servostate_msg_ = msg;
    is_recive_servo_sate_ = true;
  }
};

TEST_F(ServoStateBroadcasterTest, CheckPublication) {
  hardware_->current_drive_mode[0]->set_current(1.0);
  hardware_->current_position[0]->set_current(30.0);
  hardware_->current_velocity[0]->set_current(40.0);
  hardware_->current_effort[0]->set_current(50.0);
  hardware_->temperature[0]->set_current(60.0);
  hardware_->current[0]->set_current(70.0);
  hardware_->mrpos[0]->set_current(80.0);
  hardware_->avagopos[0]->set_current(90.0);
  hardware_->error_status[0]->set_current(100.0);

  start_time_ = client_node_->get_clock()->now().seconds();

  is_recive_servo_sate_ = false;
  rclcpp::WallRate rate(0.1);

  while ((client_node_->get_clock()->now().seconds() - start_time_) < 10.0) {
    if (is_recive_servo_sate_ == true) {
      break;
    }
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.1));
    rclcpp::spin_some(client_node_);
  }
  EXPECT_TRUE(is_recive_servo_sate_);
  EXPECT_EQ(1.0, servostate_msg_.current_drive_mode[0]);
  EXPECT_EQ(30.0, servostate_msg_.present_position[0]);
  EXPECT_EQ(40.0, servostate_msg_.present_velocity[0]);
  EXPECT_EQ(50.0, servostate_msg_.present_effort[0]);
  EXPECT_EQ(60.0, servostate_msg_.present_temperature[0]);
  EXPECT_EQ(70.0, servostate_msg_.present_current[0]);
  EXPECT_EQ(80.0, servostate_msg_.present_motor_shaft_position[0]);
  EXPECT_EQ(90.0, servostate_msg_.present_driven_shaft_position[0]);
  EXPECT_EQ(100.0, servostate_msg_.error_status[0]);
}
}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
