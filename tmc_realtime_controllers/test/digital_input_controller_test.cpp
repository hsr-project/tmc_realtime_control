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

# include <functional>

# include <gtest/gtest.h>

# include <rclcpp/rclcpp.hpp>

# include "../src/tmc_realtime_controllers/digital_input_controller.hpp"


namespace {

// State topic name
constexpr const char * const kStateTopicName = "test_state_topic";
// state_interface prefix
constexpr const char * const kStateInterfacePrefix = "test_system";
// state_interface base name
constexpr const char * const kStateInterfaceBaseName = "test_interface";

}  // anonymous namespace


namespace tmc_realtime_controllers {
using std::placeholders::_1;

class DigitalInputControllerTest :public ::testing::Test {
 protected:
  // SetUp for fixture
  void SetUp() override {
    rclcpp::init(0, nullptr);
    // test subject controller
    controller_ = std::make_shared<DigitalInputController>();
    // test probe node
    client_node_ = rclcpp::Node::make_shared("client");
    subscription_ = client_node_->create_subscription<std_msgs::msg::Bool>(
      kStateTopicName, rclcpp::SystemDefaultsQoS(),
      std::bind(&DigitalInputControllerTest::TopicCallback, this, _1));
  }

  // TearDown for test cleanup
  void TearDown() override {
    rclcpp::shutdown();
  }

  // Launch default Controller
  void LaunchDefaultController(const bool init_state = false) {
    LaunchController({
      rclcpp::Parameter(
        "state_interface_name", std::string(kStateInterfacePrefix) + "/" + std::string(kStateInterfaceBaseName)),
      rclcpp::Parameter(
        "topic_name", kStateTopicName),
      rclcpp::Parameter(
        "inverse_mode", false)},
      init_state);
  }

  // Launch controller with parameters
  void LaunchController(const std::vector<rclcpp::Parameter>& parameters, const bool init_state = false) {
    // Node Parameters
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides() = parameters;
    // Controller State Change to UnConfigured
    ASSERT_EQ(
      controller_interface::return_type::OK, controller_->init("digital_input_controller", "", node_options));
    // Controller State Change to Configured
    ASSERT_EQ(
      controller_interface::CallbackReturn::SUCCESS, controller_->on_configure(rclcpp_lifecycle::State()));

    SetHardwareStateInterface(init_state);

    // Controller State Change to Activation
    ASSERT_EQ(
      controller_interface::CallbackReturn::SUCCESS,
      controller_->on_activate(rclcpp_lifecycle::State()));
  }

  void SetHardwareStateInterface(const bool init_state) {
    state_interface_value_ = static_cast<double>(init_state);
    // Exposed loaned state interface
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interface_ = std::make_shared<hardware_interface::StateInterface>(
        kStateInterfacePrefix, kStateInterfaceBaseName, &state_interface_value_);
    state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(*state_interface_));
    controller_->assign_interfaces({}, std::move(state_interfaces));
  }

  // Set state value of state_interface
  void SetStateValue(const double update_value) {
    state_interface_value_ = update_value;
  }

  void WaitForPublisher() {
    WaitForCondition(
      [this]() { return subscription_->get_publisher_count() > 0; },
      "Publisher not available");
  }

  void WaitForState(bool expected_state) {
    // Prevent invalid pass from initial value
    received_state_msg_.data = !expected_state;
    WaitForCondition(
      [this, expected_state]() { return received_state_msg_.data == expected_state; },
      "Expected state not received");
  }

  void WaitForCondition(std::function<bool()> condition, const std::string& error_message) {
    rclcpp::WallRate rate(100.0);
    auto timeout = client_node_->now() + rclcpp::Duration(1, 0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
      rclcpp::spin_some(client_node_);
      ASSERT_EQ(
        controller_interface::return_type::OK,
        controller_->update(rclcpp::Time(0, 0), rclcpp::Duration(0, 0)));
      if (client_node_->now() > timeout) {
        FAIL() << error_message;
      }
      if (condition()) { break; }
    }
  }

  // Topic Callback
  void TopicCallback(const std_msgs::msg::Bool msg) {
    received_state_msg_ = msg;
    RCLCPP_INFO(client_node_->get_logger(), "callback_message is %d", received_state_msg_.data);
  }

  // test controller
  std::shared_ptr<DigitalInputController> controller_;
  // state value
  double state_interface_value_;
  // received state
  std_msgs::msg::Bool received_state_msg_;
  // test state_interface
  std::shared_ptr<hardware_interface::StateInterface> state_interface_;
  // test probe node
  rclcpp::Node::SharedPtr client_node_;
  // subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};


TEST_F(DigitalInputControllerTest, ConfigureProperInterface) {
  LaunchDefaultController();

  controller_interface::InterfaceConfiguration interface_config =
    controller_->state_interface_configuration();
  EXPECT_EQ(
    controller_interface::interface_configuration_type::INDIVIDUAL, interface_config.type);
  ASSERT_EQ(
    1, interface_config.names.size());
  EXPECT_EQ(
    std::string(kStateInterfacePrefix) + "/" + std::string(kStateInterfaceBaseName), interface_config.names[0]);
}


TEST_F(DigitalInputControllerTest, ConfigureNoNameInterface) {
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = {rclcpp::Parameter("topic_name", kStateTopicName)};

  EXPECT_EQ(
    controller_interface::return_type::ERROR, controller_->init("digital_input_controller", "", node_options));
}


TEST_F(DigitalInputControllerTest, InitialStateTrue) {
  LaunchDefaultController(true);
  WaitForPublisher();

  WaitForState(true);
  EXPECT_TRUE(received_state_msg_.data);
}


TEST_F(DigitalInputControllerTest, InitialStateFalse) {
  LaunchDefaultController(false);
  WaitForPublisher();

  WaitForState(false);
  EXPECT_FALSE(received_state_msg_.data);
}


TEST_F(DigitalInputControllerTest, PublishForwardState) {
  LaunchDefaultController(false);
  WaitForPublisher();

  SetStateValue(true);
  WaitForState(true);
  EXPECT_TRUE(received_state_msg_.data);

  SetStateValue(false);
  WaitForState(false);
  EXPECT_FALSE(received_state_msg_.data);
}


TEST_F(DigitalInputControllerTest, PublishInverseState) {
  LaunchController({
    rclcpp::Parameter(
      "state_interface_name", std::string(kStateInterfacePrefix) + "/" + std::string(kStateInterfaceBaseName)),
    rclcpp::Parameter(
      "topic_name", kStateTopicName),
    rclcpp::Parameter(
      "inverse_mode", true)},
    false);
  WaitForPublisher();

  SetStateValue(true);
  WaitForState(false);
  EXPECT_FALSE(received_state_msg_.data);

  SetStateValue(false);
  WaitForState(true);
  EXPECT_TRUE(received_state_msg_.data);
}

}  // namespace tmc_realtime_controllers
