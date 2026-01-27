/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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

#include "../src/tmc_realtime_controllers/empty_command_controller.hpp"

namespace tmc_realtime_controllers {

class EmptyCommandControllerTest : public ::testing::Test {
 protected:
  void SetUp() override;
  void TearDown() override;

  std::shared_ptr<EmptyCommandController> controller_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;

  double command_value_;
  std::shared_ptr<hardware_interface::CommandInterface> command_interface_;

  bool do_interrupt_;
  std::thread spinner_;
  void SpinSomeThread();
};

void EmptyCommandControllerTest::SetUp() {
  controller_ = std::make_shared<EmptyCommandController>();

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = {
      rclcpp::Parameter("command_interface_name", "test_device/test_interface"),
      rclcpp::Parameter("service_name", "~/test_trigger"),
      rclcpp::Parameter("command_value", 42.0)
  };
  ASSERT_EQ(controller_->init("empty_command_controller", "", node_options), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interface_ = std::make_shared<hardware_interface::CommandInterface>(
      "test_device", "test_interface", &command_value_);
  command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(*command_interface_));
  controller_->assign_interfaces(std::move(command_interfaces), {});

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);

  client_node_ = rclcpp::Node::make_shared("client");
  client_ = client_node_->create_client<std_srvs::srv::Empty>("empty_command_controller/test_trigger");

  do_interrupt_ = false;
  spinner_ = std::thread(std::bind(&EmptyCommandControllerTest::SpinSomeThread, this));
}

void EmptyCommandControllerTest::TearDown() {
  do_interrupt_ = true;
  spinner_.join();
}

void EmptyCommandControllerTest::SpinSomeThread() {
  while (!do_interrupt_) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(EmptyCommandControllerTest, CallEmptyService) {
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future_result = client_->async_send_request(request);

  while (rclcpp::ok() && future_result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
    ASSERT_EQ(controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)),
              controller_interface::return_type::OK);
    rclcpp::spin_some(client_node_);
  }

  const auto response = future_result.get();
  EXPECT_DOUBLE_EQ(command_value_, 42.0);

  // Ensure that only one command is sent per service invocation
  command_value_ = 0.0;
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_DOUBLE_EQ(command_value_, 0.0);
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
