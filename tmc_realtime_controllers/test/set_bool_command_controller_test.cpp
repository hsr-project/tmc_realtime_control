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
#include <algorithm>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gtest/gtest.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <tmc_hardware_interface/set_bool_command_interface.hpp>
#include "../src/tmc_realtime_controllers/set_bool_command_controller.hpp"

#include "test_methods.hpp"

// set_bool_command_controller
// SetBoolCommandController

namespace {
const size_t kHandleCount = 2;  //!/ Number of handles to register
}  // namespace

class SetBoolCommandControllerTest : public ::testing::Test {
 public:
  SetBoolCommandControllerTest() : controller_nh_("test_ok/tmc_set_bool_command_controller") {
    // Registration of handles and creation of corresponding publishers
    for (size_t i = 0; i < kHandleCount; i++) {
      // Topic name is "set_bool*"
      names_[i] = "/set_bool";
      names_[i] += boost::lexical_cast<std::string>(i + 1);
      tmc_hardware_interface::SetBoolCommandHandle set_bool_handle(names_[i], handle_datas_[i]);
      set_bool_iface_.registerHandle(set_bool_handle);
      service_clients_[i] = root_nh_.serviceClient<std_srvs::SetBool>(names_[i]);
    }
  }

 protected:
  ros::NodeHandle root_nh_;                                         //!/ root_nh
  ros::NodeHandle controller_nh_;                                   //!/ controller_nh
  tmc_hardware_interface::SetBoolCommandInterface set_bool_iface_;  //!/ Interface
  bool is_call_service_;                //!/ Flag set when CallServices completes
  boost::mutex is_call_service_mutex_;  //!/ Flag for is_call_service_

  boost::array<std::string, kHandleCount> names_;                                                //!/ Node names
  boost::array<tmc_hardware_interface::SetBoolCommandHandle::Data, kHandleCount> handle_datas_;  //!/ Data for handles
  boost::array<std_srvs::SetBool, kHandleCount> service_messages_;  //!/ Messages for services
  boost::array<ros::ServiceClient, kHandleCount> service_clients_;  //!/ Opposing ServiceClient

  /**
   * @brief Comparison of handle states
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareHandle(const tmc_hardware_interface::SetBoolCommandHandle::Data& x,
                     const tmc_hardware_interface::SetBoolCommandHandle::Data& y) const {
    EXPECT_EQ(x.request_, y.request_);
    EXPECT_EQ(x.has_request_, y.has_request_);
    EXPECT_EQ(x.response_, y.response_);
  }

  /**
   * @brief Comparison of responses
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareResponse(const std_srvs::SetBool::Response& x, const std_srvs::SetBool::Response& y) const {
    EXPECT_EQ(x.success, y.success);
    EXPECT_STREQ(x.message.c_str(), y.message.c_str());
  }

  /**
   * @brief Helper for Init test
   * Handle values do not change when there is no service call
   * Test to confirm that the handle flag is down when there is no service call
   * @param initial_value
   * @param controller
   */
  void InitTestHelper(const tmc_hardware_interface::SetBoolCommandHandle::Data& initial_value,
                      tmc_realtime_controllers::SetBoolCommandController& controller) {
    ASSERT_TRUE(controller.init(&set_bool_iface_, root_nh_, controller_nh_)) << "Init succeeds unconditionally";
    controller.starting(ros::Time::now());

    for (size_t i = 0; i < kHandleCount; i++) {
      handle_datas_[i] = initial_value;
    }

    controller.update(ros::Time::now(), ros::Duration());
    ros::spinOnce();

    for (size_t i = 0; i < kHandleCount; i++) {
      CompareHandle(handle_datas_[i], initial_value);
    }
  }

  /**
   * @brief Helper for service calls in a separate thread
   * Test to confirm that calling a service in a separate thread completes processing normally.
   * If this method is called without preparing an asynchronous spinner, the test will deadlock.
   * It is recommended to use via the CallServices method, which also manages asynchronous spinners.
   *
   * @param[in] index Index of the service to call
   */
  void CallService(const uint32_t index) {
    EXPECT_TRUE(service_clients_[index].exists());
    EXPECT_TRUE(service_clients_[index].isValid());
    EXPECT_TRUE(service_clients_[index].call(service_messages_[index]));
  }

  /**
   * @brief Helper for managing AsyncSpinner and service calls in a separate thread
   * After starting the AsyncSpinner, service calls are made to multiple services specified by bits.
   * This function waits until all service calls are completed,
   * so the caller should consider calling this function itself in a separate thread.
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   */
  void CallServices(const uint32_t bits) {
    ros::AsyncSpinner async_spinner((kHandleCount * 2) + 1);  // Spin the spinner for callback resolution. Maximum number of services + 1 reserve
    async_spinner.start();

    // Service call
    boost::thread_group threads;
    for (size_t i = 0; i < kHandleCount; ++i) {
      if ((bits & (1 << i)) != 0) {
        threads.create_thread(boost::bind(&SetBoolCommandControllerTest::CallService, this, i));
      }
    }

    threads.join_all();  // Wait until all service calls are completed
  }

  /**
   * @brief Helper template for service tests
   * Test to confirm that the flag of the handle is set correctly after calling the service
   * Robot_hw processing is customized for each test
   * Specify the serviceClient to call using bits.
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   * @param[in] func Processing performed by Robot_hw during execution
   */
  void ServiceTestHelperBase(const uint32_t bits, tmc_realtime_controllers::SetBoolCommandController& controller,
                             boost::function<void(const size_t)> func) {
    ROS_INFO("thread start");

    // Service call
    boost::thread call_thread(&SetBoolCommandControllerTest::CallServices, this, bits);
    // Wait until the service call is completed
    ros::Duration(0.1).sleep();

    // Transition to Processing in this Update
    controller.update(ros::Time::now(), ros::Duration());
    // Processing
    for (size_t i = 0; i < kHandleCount; ++i) {
      bool expect = false;
      if ((bits & (1 << i)) != 0) {
        expect = true;
      }
      EXPECT_EQ(handle_datas_[i].has_request_, expect);
      if (expect) {
        func(i);  // Update processing in the Hw layer
      }
    }

    // Transition to kResponsed if handle_.has_request_ is down
    controller.update(ros::Time::now(), ros::Duration());

    // Service completed, is_call_service_ is down
    call_thread.join();

    // Transition to Standby
    controller.update(ros::Time::now(), ros::Duration());
    ros::spinOnce();
  }

  /**
   * @brief Helper for normal service call tests
   * Robot_hw immediately drops the handle_.has_request_ flag
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceNomalTestHelper(const uint32_t bits, tmc_realtime_controllers::SetBoolCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&SetBoolCommandControllerTest::NomalProcess, this, _1));
  }
  void NomalProcess(const size_t index) {
    // Normal case drops the flag as processing is completed
    handle_datas_[index].has_request_ = false;
  }

  /**
   * @brief Helper for service call tests during timeout
   * Robot_hw waits for the timeout duration defined in ROS Param
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceTimeoutTestHelper(const uint32_t bits, tmc_realtime_controllers::SetBoolCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&SetBoolCommandControllerTest::TimeoutProcess, this, _1));
  }
  void TimeoutProcess(const size_t index) {
    (void)(index);  // unused

    // Wait for timeout without dropping the flag
    int32_t timeout_ms;
    controller_nh_.getParam("service_time_out", timeout_ms);
    ros::Duration duration(static_cast<double>(timeout_ms) / 1000.0);
    duration.sleep();
  }

  /**
   * @brief Helper for service multiple call tests
   * Test to check if calling the same service during Robot_hw update returns an immediate error
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceAlreadyUseTestHelper(const uint32_t bits,
                                   tmc_realtime_controllers::SetBoolCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&SetBoolCommandControllerTest::AlreadyUseProcess, this, _1));
  }
  void AlreadyUseProcess(const size_t index) {
    // Create message
    std_srvs::SetBool sb;
    sb.request.data = true;
    sb.response.success = true;
    sb.response.message = "";

    // Service call itself completes successfully
    ros::ServiceClient client = root_nh_.serviceClient<std_srvs::SetBool>(names_[index]);
    EXPECT_TRUE(client.exists());
    EXPECT_TRUE(client.isValid());
    EXPECT_TRUE(client.call(sb));

    // Service call returns a failure
    EXPECT_EQ(sb.response.success, false);
    EXPECT_STREQ(sb.response.message.c_str(), "This service is already in use.");

    // Drop the flag as processing is completed
    handle_datas_[index].has_request_ = false;
  }
};

// Check node initialization and controller initial values
TEST_F(SetBoolCommandControllerTest, InitNomalTest) {
  boost::array<tmc_hardware_interface::SetBoolCommandHandle::Data, 2> init_values;
  init_values[0].response_ = false;
  init_values[0].request_ = false;
  init_values[1].response_ = true;
  init_values[1].request_ = true;

  BOOST_FOREACH (const tmc_hardware_interface::SetBoolCommandHandle::Data b, init_values) {
    std::string name = "init " + boost::lexical_cast<std::string>(b.request_);
    SCOPED_TRACE(name);
    tmc_realtime_controllers::SetBoolCommandController set_bool_c;
    InitTestHelper(b, set_bool_c);
    set_bool_c.stopping(ros::Time::now());
  }
}

// Confirm that init fails when required Params are missing
TEST_F(SetBoolCommandControllerTest, InitFailureTest_BadNamespace) {
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(set_bool_c.init(&set_bool_iface_, root_nh_, bad_controller_nh));
}

// Confirm that init fails when Params are invalid (less than or equal to 0)
TEST_F(SetBoolCommandControllerTest, InitFailureTest_BadParam) {
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ros::NodeHandle bad_controller_nh("test_ko/tmc_set_bool_command_controller");
  EXPECT_FALSE(set_bool_c.init(&set_bool_iface_, root_nh_, bad_controller_nh));
}

// When there are two nodes, confirm that the flag of the handle called by the Service is set correctly with proper allocation
TEST_F(SetBoolCommandControllerTest, ServiceNomalTest_Allocation) {
  tmc_hardware_interface::SetBoolCommandHandle::Data init_value;
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, set_bool_c));

  boost::array<uint32_t, 5> params = {
      0b00, 0b01, 0b11, 0b10, 0b00,
  };
  BOOST_FOREACH (uint32_t bits, params) {
    std::string name = "bits = " + boost::lexical_cast<std::string>(bits);
    SCOPED_TRACE(name);
    ServiceNomalTestHelper(bits, set_bool_c);
  }
}

// Confirm that the same value as the argument is sent to the handle during the service call
TEST_F(SetBoolCommandControllerTest, ServiceNomalTest_RequestData) {
  tmc_hardware_interface::SetBoolCommandHandle::Data init_value;
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, set_bool_c));

  boost::array<bool, 2> expects = {true, false};
  BOOST_FOREACH (bool expect, expects) {
    std::stringstream name;
    name << "expect = " << expect;
    SCOPED_TRACE(name.str());

    service_messages_[0].request.data = expect;
    ServiceNomalTestHelper(1, set_bool_c);
    EXPECT_EQ(handle_datas_[0].request_, expect);
  }
}

// Confirm that the execution result is returned to the caller during the service call
TEST_F(SetBoolCommandControllerTest, ServiceNomalTest_ResponseData) {
  tmc_hardware_interface::SetBoolCommandHandle::Data init_value;
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, set_bool_c));

  // Test for response
  boost::array<bool, 2> expect_bools = {true, false};
  BOOST_FOREACH (bool expect, expect_bools) {
    std::stringstream name;
    name << "expect = " << expect;
    SCOPED_TRACE(name.str());

    handle_datas_[0].response_ = expect;
    ServiceNomalTestHelper(1, set_bool_c);
    std_srvs::SetBool sb;
    sb.response.success = expect;
    sb.response.message = "OK";
    CompareResponse(service_messages_[0].response, sb.response);
  }

  // No test is conducted as the message is a fixed value
}

// Confirm that the flag automatically drops during timeout
TEST_F(SetBoolCommandControllerTest, ServiceFailureTest_Timeout) {
  tmc_hardware_interface::SetBoolCommandHandle::Data init_value;
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, set_bool_c));

  ServiceTimeoutTestHelper(1, set_bool_c);
  std_srvs::SetBool sb;
  sb.response.success = false;
  sb.response.message = "Hardware did not respond. Timeout";
  CompareResponse(service_messages_[0].response, sb.response);

  // Confirm that it has returned to the normal state
  sb.response.success = true;
  handle_datas_[0].response_ = true;
  sb.response.message = "OK";
  ServiceNomalTestHelper(1, set_bool_c);
  CompareResponse(service_messages_[0].response, sb.response);
}

// When another service call is made during service execution, confirm that an immediate error is returned to the second service
TEST_F(SetBoolCommandControllerTest, ServiceFailureTest_AlreadyInUse) {
  tmc_hardware_interface::SetBoolCommandHandle::Data init_value;
  tmc_realtime_controllers::SetBoolCommandController set_bool_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, set_bool_c));

  std_srvs::SetBool sb;
  sb.response.success = true;
  handle_datas_[0].response_ = true;
  sb.response.message = "OK";

  // TODO(kitsunai): サービスの多重呼び出しのテスト方法を相談
  // ServiceAlreadyUseTestHelper(1, set_bool_c);
  // CompareResponse(service_messages_[0].response, sb.response);

  // Confirm that it has returned to the normal state
  ServiceNomalTestHelper(1, set_bool_c);
  CompareResponse(service_messages_[0].response, sb.response);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  // Start roscore
  ros::init(argc, argv, "set_bool_command_controller_test");
  int32_t ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
