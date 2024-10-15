/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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

#include <rclcpp/rclcpp.hpp>

#include "../src/tmc_realtime_controllers/digital_output_controller.hpp"

namespace tmc_realtime_controllers {

class DigitalOutputControllerTest : public ::testing::Test {
 protected:
  std::shared_ptr<DigitalOutputController> controller_;
  double command_value_;
  std::shared_ptr<hardware_interface::CommandInterface> command_interface_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

  void SetUp() override;
  void ConfigureController(const std::vector<rclcpp::Parameter>& parameters);
  void ConfigureController();

  void WaitForSubscriber();
  void WaitForCommand(bool expected);
};

void DigitalOutputControllerTest::SetUp() {
  controller_ = std::make_shared<DigitalOutputController>();

  client_node_ = rclcpp::Node::make_shared("client");
  publisher_ = client_node_->create_publisher<std_msgs::msg::Bool>("test_command", rclcpp::SystemDefaultsQoS());
}

void DigitalOutputControllerTest::ConfigureController(const std::vector<rclcpp::Parameter>& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = parameters;
  ASSERT_EQ(controller_->init("digital_output_controller", "", node_options), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interface_ = std::make_shared<hardware_interface::CommandInterface>(
      "system", "test_interface", &command_value_);
  command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(*command_interface_));
  controller_->assign_interfaces(std::move(command_interfaces), {});

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
}

void DigitalOutputControllerTest::ConfigureController() {
  ConfigureController({rclcpp::Parameter("command_interface_name", "system/test_interface"),
                       rclcpp::Parameter("topic_name", "test_command")});
}

void DigitalOutputControllerTest::WaitForSubscriber() {
  rclcpp::WallRate rate(100.0);
  auto timeout = client_node_->now() + rclcpp::Duration(1, 0);
  while (rclcpp::ok() && publisher_->get_subscription_count() == 0) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    rclcpp::spin_some(client_node_);
    if (client_node_->now() > timeout) {
      FAIL();
    }
  }
}

void DigitalOutputControllerTest::WaitForCommand(bool expected) {
  rclcpp::WallRate rate(100.0);
  auto timeout = client_node_->now() + rclcpp::Duration(1, 0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    ASSERT_EQ(controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)),
              controller_interface::return_type::OK);
    if (client_node_->now() > timeout) {
      FAIL();
    }
    if (expected && command_value_ > 0.5) {
      return;
    } else if (!expected && command_value_ < 0.5) {
      return;
    }
  }
}

TEST_F(DigitalOutputControllerTest, SubscribeCommand) {
  ConfigureController();
  WaitForSubscriber();

  std_msgs::msg::Bool msg;
  msg.data = true;
  publisher_->publish(msg);

  WaitForCommand(msg.data);

  msg.data = false;
  publisher_->publish(msg);

  WaitForCommand(msg.data);
}

TEST_F(DigitalOutputControllerTest, DefaultValueNotSet) {
  ConfigureController();

  ASSERT_EQ(controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_value_, 0.0);
}

TEST_F(DigitalOutputControllerTest, DefaultValueTrue) {
  ConfigureController({rclcpp::Parameter("command_interface_name", "system/test_interface"),
                       rclcpp::Parameter("topic_name", "test_command"),
                       rclcpp::Parameter("default_output", true)});

  ASSERT_EQ(controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(command_value_, 1.0);
}

TEST_F(DigitalOutputControllerTest, CommandInterfaceNameNotSet) {
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = {rclcpp::Parameter("topic_name", "servo_enable")};
  EXPECT_EQ(controller_->init("digital_output_controller", "", node_options), controller_interface::return_type::ERROR);
}

TEST_F(DigitalOutputControllerTest, CommandInterfaceName) {
  ConfigureController();

  const auto interface_config = controller_->command_interface_configuration();
  EXPECT_EQ(interface_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(interface_config.names.size(), 1);
  EXPECT_EQ(interface_config.names[0], "system/test_interface");
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
