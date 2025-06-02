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
/// @brief Test of the controller that provides a service to read and write parameters

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tmc_control_msgs/msg/servo_param.hpp>
#include <tmc_control_msgs/srv/read_parameters.hpp>
#include <tmc_control_msgs/srv/write_parameters.hpp>

#include "infrequent_servo_hardware_stub.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"  // Header for NodeBaseInterface

namespace {
const char* const kReadingService = "/reader/access";
const char* const kWritingService = "/writer/access";
const char* const kReadControllerNodeName = "reader_controller_manager";
const char* const kWriteControllerNodeName = "writer_controller_manager";
const std::vector<std::string> kDeniedKeys = { "denied_key" };
}  // namespace

namespace tmc_realtime_controllers {

class InfrequentControllerTest : public ::testing::Test {
 public:
  void SetUp() override {
    hsrb_hw_node_ = rclcpp::Node::make_shared("hsrb_hw");

    hsrb_hw_node_->declare_parameter<std::string>("control_table_path", "./test/control_table.csv");
    do_interrupt_ = false;
    spinner_ = std::thread(
        std::bind(&InfrequentControllerTest::spin_some_thread, this, hsrb_hw_node_->get_node_base_interface()));

    reader_controller_ = std::make_shared<TestableInfrequentReadingController>();
    reader_controller_node_ = reader_controller_->get_node();

    writer_controller_ = std::make_shared<TestableInfrequentWritingController>();
    writer_controller_node_ = writer_controller_->get_node();
    // Parameter definition
    std::vector<std::string> joint_names = { "JointA" };

    reader_controller_node_->declare_parameter<std::vector<std::string> >("joints", joint_names);
    reader_controller_node_->declare_parameter<std::string>("attribute", "reader");

    writer_controller_node_->declare_parameter<std::vector<std::string> >("joints", joint_names);
    writer_controller_node_->declare_parameter<std::string>("attribute", "writer");
    writer_controller_node_->set_parameter(rclcpp::Parameter("denied_keys", kDeniedKeys));

    EXPECT_EQ(reader_controller_->init(kReadControllerNodeName), controller_interface::return_type::OK);

    EXPECT_EQ(writer_controller_->init(kWriteControllerNodeName), controller_interface::return_type::OK);


    reader_hardware_ = std::make_shared<HardwareStub>(joint_names);
    writer_hardware_ = std::make_shared<HardwareStub>(joint_names);

    reader_controller_->assign_interfaces(std::move(reader_hardware_->command_interfaces),
                                          std::move(reader_hardware_->state_interfaces));
    writer_controller_->assign_interfaces(std::move(writer_hardware_->command_interfaces),
                                          std::move(writer_hardware_->state_interfaces));

    do_interrupt_ = true;
    spinner_.join();
    EXPECT_EQ(reader_controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(writer_controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    EXPECT_EQ(reader_controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(writer_controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    reader_client_node_ = rclcpp::Node::make_shared("reader_test_node");
    writer_client_node_ = rclcpp::Node::make_shared("writer_test_node");

    reader_param_srv_client_ =
        reader_client_node_->create_client<tmc_control_msgs::srv::ReadParameters>(kReadingService);

    writer_param_srv_client_ =
        writer_client_node_->create_client<tmc_control_msgs::srv::WriteParameters>(kWritingService);

    EXPECT_TRUE(reader_param_srv_client_->wait_for_service());
    EXPECT_TRUE(writer_param_srv_client_->wait_for_service());
  }

  void spin_some_thread(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) {
    while (!do_interrupt_) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

 protected:
  TestableInfrequentReadingController::Ptr reader_controller_;
  TestableInfrequentWritingController::Ptr writer_controller_;
  std::thread spinner_;
  bool do_interrupt_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> reader_controller_node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> writer_controller_node_;
  HardwareStub::Ptr reader_hardware_;
  HardwareStub::Ptr writer_hardware_;

  rclcpp::Node::SharedPtr reader_client_node_;
  rclcpp::Node::SharedPtr writer_client_node_;
  rclcpp::Node::SharedPtr hsrb_hw_node_;

  rclcpp::Client<tmc_control_msgs::srv::ReadParameters>::SharedPtr reader_param_srv_client_;
  rclcpp::Client<tmc_control_msgs::srv::WriteParameters>::SharedPtr writer_param_srv_client_;
};

// Parameter write, joint name does not exist
TEST_F(InfrequentControllerTest, InvalidJointName) {
  tmc_control_msgs::msg::ServoParam key_value;
  key_value.key = "hoge";
  key_value.value = 42.0;

  auto request = std::make_shared<tmc_control_msgs::srv::WriteParameters::Request>();
  request->name = "JointC";
  request->values.push_back(key_value);

  writer_hardware_->writer_value[0]->set_current(0.0);
  auto response = writer_param_srv_client_->async_send_request(request);

  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(writer_controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(rclcpp::spin_until_future_complete(writer_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_FALSE(response.get()->success);

  // Nothing has been written
  double tmp_value = writer_hardware_->writer_value[0]->command();

  EXPECT_DOUBLE_EQ(0.0, tmp_value);
}
// Parameter write, specify key that refuses writing
TEST_F(InfrequentControllerTest, DeniedKey) {
  rclcpp::spin_some(writer_controller_node_->get_node_base_interface());

  tmc_control_msgs::msg::ServoParam key_value;
  key_value.key = "denied_key";
  key_value.value = 42.0;

  auto request = std::make_shared<tmc_control_msgs::srv::WriteParameters::Request>();
  request->name = "JointA";
  request->values.push_back(key_value);

  writer_hardware_->writer_value[0]->set_current(0.0);
  auto response = writer_param_srv_client_->async_send_request(request);

  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(writer_controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(rclcpp::spin_until_future_complete(writer_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_FALSE(response.get()->success);

  // Nothing has been written
  double tmp_value = writer_hardware_->writer_value[0]->command();

  EXPECT_DOUBLE_EQ(0.0, tmp_value);
}

// Parameter write, parameter does not exist
TEST_F(InfrequentControllerTest, InvalidParameterName) {
  auto request = std::make_shared<tmc_control_msgs::srv::WriteParameters::Request>();
  request->name = "JointA";

  tmc_control_msgs::msg::ServoParam hoge;
  hoge.key = "hoge";
  hoge.value = 42.0;

  tmc_control_msgs::msg::ServoParam piyo;
  piyo.key = "present_motor_voltage";
  piyo.value = 33.4;

  request->name = "JointA";
  request->values.push_back(hoge);
  request->values.push_back(piyo);

  auto response = writer_param_srv_client_->async_send_request(request);

  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, writer_controller_node_->get_node_base_interface()));

  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(writer_controller_->update(writer_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();

  ASSERT_TRUE(rclcpp::spin_until_future_complete(writer_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(response.get()->success);
}

// Parameter write, there are parameters that cannot be read even after retry
TEST_F(InfrequentControllerTest, WritingParameterFailure) {
  auto request = std::make_shared<tmc_control_msgs::srv::WriteParameters::Request>();
  request->name = "JointA";

  tmc_control_msgs::msg::ServoParam hoge;
  hoge.key = "present_12v_voltage";
  hoge.value = 42.0;

  tmc_control_msgs::msg::ServoParam piyo;
  piyo.key = "present_motor_voltage";
  piyo.value = 33.4;

  request->name = "JointA";
  request->values.push_back(hoge);
  request->values.push_back(piyo);

  auto response = writer_param_srv_client_->async_send_request(request);

  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, writer_controller_node_->get_node_base_interface()));

  bool command_execute = false;
  for (int i = 0; i < 10; i++) {
    int has_command = writer_hardware_->writer_has_command[0]->command();
    if (has_command == 1) {
      writer_hardware_->writer_has_command[0]->set_current(1.0);
      command_execute = true;
    }
    if (command_execute == true) {
      int command_index = writer_hardware_->writer_command_index[0]->command();
      command_execute = false;
      if (command_index == 5) {
        double tmp_value = writer_hardware_->writer_value[0]->command();
        EXPECT_DOUBLE_EQ(42.0, tmp_value);
        writer_hardware_->writer_value[0]->set_current(42.0);
        writer_hardware_->writer_has_command[0]->set_current(0.0);
        writer_hardware_->writer_is_success[0]->set_current(1.0);
      }
      if (command_index == 6) {
        double tmp_value = writer_hardware_->writer_value[0]->command();
        EXPECT_DOUBLE_EQ(33.4, tmp_value);
        writer_hardware_->writer_value[0]->set_current(33.4);
        writer_hardware_->writer_has_command[0]->set_current(1.0);
        writer_hardware_->writer_is_success[0]->set_current(0.0);
        writer_hardware_->writer_trial_num[0]->set_current(3.0);
      }
    }
    EXPECT_EQ(writer_controller_->update(writer_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();

  ASSERT_TRUE(rclcpp::spin_until_future_complete(writer_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(response.get()->success);
}

// Parameter write, normal case
TEST_F(InfrequentControllerTest, Normal) {
  auto request = std::make_shared<tmc_control_msgs::srv::WriteParameters::Request>();
  request->name = "JointA";

  tmc_control_msgs::msg::ServoParam hoge;
  hoge.key = "present_12v_voltage";
  hoge.value = 42.0;

  tmc_control_msgs::msg::ServoParam piyo;
  piyo.key = "present_motor_voltage";
  piyo.value = 33.4;

  request->name = "JointA";
  request->values.push_back(hoge);
  request->values.push_back(piyo);

  auto response = writer_param_srv_client_->async_send_request(request);

  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, writer_controller_node_->get_node_base_interface()));

  bool command_execute = false;
  for (int i = 0; i < 10; i++) {
    int has_command = writer_hardware_->writer_has_command[0]->command();
    if (has_command == 1) {
      writer_hardware_->writer_has_command[0]->set_current(1.0);
      command_execute = true;
    }
    if (command_execute == true) {
      int command_index = writer_hardware_->writer_command_index[0]->command();
      command_execute = false;
      if (command_index == 5) {
        double tmp_value = writer_hardware_->writer_value[0]->command();
        EXPECT_DOUBLE_EQ(42.0, tmp_value);
        writer_hardware_->writer_value[0]->set_current(42.0);
        writer_hardware_->writer_has_command[0]->set_current(0.0);
        writer_hardware_->writer_is_success[0]->set_current(1.0);
      }
      if (command_index == 6) {
        double tmp_value = writer_hardware_->writer_value[0]->command();
        EXPECT_DOUBLE_EQ(33.4, tmp_value);
        writer_hardware_->writer_value[0]->set_current(33.4);
        writer_hardware_->writer_has_command[0]->set_current(0.0);
        writer_hardware_->writer_is_success[0]->set_current(1.0);
      }
    }
    EXPECT_EQ(writer_controller_->update(writer_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;

  spinner_.join();
  ASSERT_TRUE(rclcpp::spin_until_future_complete(writer_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(response.get()->success);
}

// Parameter read, joint name does not exist
TEST_F(InfrequentControllerTest, ReadInvalidJointName) {
  tmc_control_msgs::msg::ServoParam key_value;
  auto request = std::make_shared<tmc_control_msgs::srv::ReadParameters::Request>();
  request->name = "JointC";

  reader_hardware_->writer_value[0]->set_current(0.0);
  auto response = reader_param_srv_client_->async_send_request(request);

  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(reader_controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(rclcpp::spin_until_future_complete(reader_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_FALSE(response.get()->success);
}

// Parameter read, parameter does not exist
TEST_F(InfrequentControllerTest, ReadInvalidParameterName) {
  auto request = std::make_shared<tmc_control_msgs::srv::ReadParameters::Request>();
  request->name = "JointA";
  request->keys.push_back("present_12v_voltage");
  request->keys.push_back("hoge");

  auto response = reader_param_srv_client_->async_send_request(request);
  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, reader_controller_node_->get_node_base_interface()));

  bool command_execute = false;
  for (int i = 0; i < 10; i++) {
    int has_command = reader_hardware_->reader_has_command[0]->command();
    if (has_command == 1) {
      reader_hardware_->reader_has_command[0]->set_current(1.0);
      command_execute = true;
    }
    if (command_execute == true) {
      int command_index = reader_hardware_->reader_command_index[0]->command();
      command_execute = false;
      if (command_index == 5) {
        reader_hardware_->reader_value[0]->set_current(42.0);
        reader_hardware_->reader_has_command[0]->set_current(0.0);
        reader_hardware_->reader_is_success[0]->set_current(1.0);
      }
    }
    EXPECT_EQ(reader_controller_->update(reader_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();
  ASSERT_TRUE(rclcpp::spin_until_future_complete(reader_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  std::shared_ptr<tmc_control_msgs::srv::ReadParameters::Response> response_value = response.get();

  EXPECT_FALSE(response_value->success);
  ASSERT_EQ(1, response_value->values.size());
  EXPECT_EQ("present_12v_voltage", response_value->values[0].key);
  EXPECT_DOUBLE_EQ(42.0, response_value->values[0].value);
}

// Parameter read, there are parameters that cannot be read even after retry
TEST_F(InfrequentControllerTest, ReadingParameterFailure) {
  auto request = std::make_shared<tmc_control_msgs::srv::ReadParameters::Request>();
  request->name = "JointA";
  request->keys.push_back("present_12v_voltage");
  request->keys.push_back("present_motor_voltage");

  auto response = reader_param_srv_client_->async_send_request(request);
  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, reader_controller_node_->get_node_base_interface()));

  bool command_execute = false;
  for (int i = 0; i < 10; i++) {
    int has_command = reader_hardware_->reader_has_command[0]->command();
    if (has_command == 1) {
      reader_hardware_->reader_has_command[0]->set_current(1.0);
      command_execute = true;
    }
    if (command_execute == true) {
      int command_index = reader_hardware_->reader_command_index[0]->command();
      command_execute = false;
      if (command_index == 5) {
        reader_hardware_->reader_value[0]->set_current(42.0);
        reader_hardware_->reader_has_command[0]->set_current(0.0);
        reader_hardware_->reader_is_success[0]->set_current(1.0);
      }
      if (command_index == 6) {
        reader_hardware_->reader_has_command[0]->set_current(1.0);
        reader_hardware_->reader_is_success[0]->set_current(0.0);
        reader_hardware_->reader_trial_num[0]->set_current(3.0);
      }
    }
    EXPECT_EQ(reader_controller_->update(reader_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();
  ASSERT_TRUE(rclcpp::spin_until_future_complete(reader_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  std::shared_ptr<tmc_control_msgs::srv::ReadParameters::Response> response_value = response.get();

  EXPECT_FALSE(response_value->success);
  ASSERT_EQ(1, response_value->values.size());
  EXPECT_EQ("present_12v_voltage", response_value->values[0].key);
  EXPECT_DOUBLE_EQ(42.0, response_value->values[0].value);
}

// Parameter read, normal case
TEST_F(InfrequentControllerTest, ReadNormal) {
  auto request = std::make_shared<tmc_control_msgs::srv::ReadParameters::Request>();
  request->name = "JointA";
  request->keys.push_back("present_12v_voltage");
  request->keys.push_back("present_motor_voltage");

  auto response = reader_param_srv_client_->async_send_request(request);
  do_interrupt_ = false;
  spinner_ = std::thread(
      std::bind(&InfrequentControllerTest::spin_some_thread, this, reader_controller_node_->get_node_base_interface()));

  bool command_execute = false;
  for (int i = 0; i < 10; i++) {
    int has_command = reader_hardware_->reader_has_command[0]->command();
    if (has_command == 1) {
      reader_hardware_->reader_has_command[0]->set_current(1.0);
      command_execute = true;
    }
    if (command_execute == true) {
      int command_index = reader_hardware_->reader_command_index[0]->command();
      command_execute = false;
      if (command_index == 5) {
        reader_hardware_->reader_value[0]->set_current(42.0);
        reader_hardware_->reader_has_command[0]->set_current(0.0);
        reader_hardware_->reader_is_success[0]->set_current(1.0);
      }
      if (command_index == 6) {
        double tmp_value = reader_hardware_->reader_value[0]->command();
        reader_hardware_->reader_value[0]->set_current(33.4);
        reader_hardware_->reader_has_command[0]->set_current(0.0);
        reader_hardware_->reader_is_success[0]->set_current(1.0);
      }
    }
    EXPECT_EQ(reader_controller_->update(reader_controller_node_->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_interrupt_ = true;
  spinner_.join();
  ASSERT_TRUE(rclcpp::spin_until_future_complete(reader_client_node_, response) == rclcpp::FutureReturnCode::SUCCESS);

  std::shared_ptr<tmc_control_msgs::srv::ReadParameters::Response> response_value = response.get();

  EXPECT_TRUE(response_value->success);
  ASSERT_EQ(2, response_value->values.size());
  EXPECT_EQ("present_12v_voltage", response_value->values[0].key);
  EXPECT_DOUBLE_EQ(42.0, response_value->values[0].value);
  EXPECT_EQ("present_motor_voltage", response_value->values[1].key);
  EXPECT_DOUBLE_EQ(33.4, response_value->values[1].value);
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
