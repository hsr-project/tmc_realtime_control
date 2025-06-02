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
#include <std_srvs/Trigger.h>

#include <tmc_hardware_interface/trigger_command_interface.hpp>
#include "../src/tmc_realtime_controllers/trigger_command_controller.hpp"

#include "test_methods.hpp"

// trigger_command_controller
// TriggerCommandController

namespace {
const size_t kHandleCount = 2;  //!/ Number of handles to register
}  // namespace

class TriggerCommandControllerTest : public ::testing::Test {
 public:
  TriggerCommandControllerTest() : controller_nh_("test_ok/tmc_trigger_command_controller") {
    // Registration of handles and creation of corresponding publishers
    for (size_t i = 0; i < kHandleCount; i++) {
      // Topic name is "trigger*"
      names_[i] = "/trigger";
      names_[i] += boost::lexical_cast<std::string>(i + 1);
      tmc_hardware_interface::TriggerCommandHandle trigger_handle(names_[i], handle_datas_[i]);
      trigger_iface_.registerHandle(trigger_handle);
      service_clients_[i] = root_nh_.serviceClient<std_srvs::Trigger>(names_[i]);
    }
  }

 protected:
  ros::NodeHandle root_nh_;                                        //!/ root_nh
  ros::NodeHandle controller_nh_;                                  //!/ controller_nh
  tmc_hardware_interface::TriggerCommandInterface trigger_iface_;  //!/ Interface
  bool is_call_service_;                //!/ Flag set when CallServices completes
  boost::mutex is_call_service_mutex_;  //!/ Flag for is_call_service_

  boost::array<std::string, kHandleCount> names_;                                                //!/ Node names
  boost::array<tmc_hardware_interface::TriggerCommandHandle::Data, kHandleCount> handle_datas_;  //!/ Data for handles
  boost::array<std_srvs::Trigger, kHandleCount> service_messages_;  //!/ Messages for services
  boost::array<ros::ServiceClient, kHandleCount> service_clients_;  //!/ Opposing ServiceClient

  /**
   * @brief State comparison of handles
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareHandle(const tmc_hardware_interface::TriggerCommandHandle::Data& x,
                     const tmc_hardware_interface::TriggerCommandHandle::Data& y) const {
    EXPECT_EQ(x.has_request_, y.has_request_);
    EXPECT_EQ(x.response_, y.response_);
  }

  /**
   * @brief Response comparison
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareResponse(const std_srvs::Trigger::Response& x, const std_srvs::Trigger::Response& y) const {
    EXPECT_EQ(x.success, y.success);
    EXPECT_STREQ(x.message.c_str(), y.message.c_str());
  }

  /**
   * @brief Helper for Init test
   * When no service call is made, confirm that the handle value does not change
   * Test to confirm that the handle flag is dropped when no service call is made
   * @param initial_value
   * @param controller
   */
  void InitTestHelper(const tmc_hardware_interface::TriggerCommandHandle::Data& initial_value,
                      tmc_realtime_controllers::TriggerCommandController& controller) {
    ASSERT_TRUE(controller.init(&trigger_iface_, root_nh_, controller_nh_)) << "Init should succeed unconditionally";
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
   * @brief Helper for calling services in another thread
   * Test to call services in another thread and confirm that processing completes successfully.
   * Calling this method without preparing an asynchronous spinner will cause the test to deadlock.
   * Recommended to use through the CallServices method that also manages the asynchronous spinner
   *
   * @param[in] index Index of the service to call
   */
  void CallService(const uint32_t index) {
    EXPECT_TRUE(service_clients_[index].exists());
    EXPECT_TRUE(service_clients_[index].isValid());
    EXPECT_TRUE(service_clients_[index].call(service_messages_[index]));
  }

  /**
   * @brief Helper for managing AsyncSpinner and calling services in another thread
   * After starting AsyncSpinner, call services to multiple targets specified by bits.
   * This function waits until all service calls complete, so
   * Caller should make effort such as calling this function itself in another thread.
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   */
  void CallServices(const uint32_t bits) {
    ros::AsyncSpinner async_spinner((kHandleCount * 2) + 1);  // Spin spinner to resolve callbacks. Maximum number of services + 1 reserve
    async_spinner.start();

    // Service call
    boost::thread_group threads;
    for (size_t i = 0; i < kHandleCount; ++i) {
      if ((bits & (1 << i)) != 0) {
        threads.create_thread(boost::bind(&TriggerCommandControllerTest::CallService, this, i));
      }
    }

    threads.join_all();  // Wait for all service calls to complete
  }

  /**
   * @brief Helper template for service test
   * Perform tests to call service and confirm the handle flag is properly set
   * Robot_hw processing is customized for each test
   * Specify serviceClient to call with bits.
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   * @param[in] func Processing Robot_hw performs during test
   */
  void ServiceTestHelperBase(const uint32_t bits, tmc_realtime_controllers::TriggerCommandController& controller,
                             boost::function<void(const size_t)> func) {
    ROS_INFO("thread start");

    // Service call
    boost::thread call_thread(&TriggerCommandControllerTest::CallServices, this, bits);
    // Wait for service call to complete
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
        func(i);  // Update processing in Hw layer
      }
    }

    // If handle_.has_request_ is dropped, transition to kResponsed
    controller.update(ros::Time::now(), ros::Duration());

    // Service complete, is_call_service_ is dropped
    call_thread.join();

    // Transition to Standby
    controller.update(ros::Time::now(), ros::Duration());
    ros::spinOnce();
  }

  /**
   * @brief Helper for normal service call test
   * Robot_hw immediately drops handle_.has_request_ flag
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceNomalTestHelper(const uint32_t bits, tmc_realtime_controllers::TriggerCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&TriggerCommandControllerTest::NomalProcess, this, _1));
  }
  void NomalProcess(const size_t index) {
    // Normal process, flag is dropped as processing completes
    handle_datas_[index].has_request_ = false;
  }

  /**
   * @brief Helper for timeout service call test
   * Robot_hw waits for the timeout time defined in ROS Param
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceTimeoutTestHelper(const uint32_t bits, tmc_realtime_controllers::TriggerCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&TriggerCommandControllerTest::TimeoutProcess, this, _1));
  }
  void TimeoutProcess(const size_t index) {
    (void)(index);  // unused

    // Wait for timeout without dropping flag
    int32_t timeout_ms;
    controller_nh_.getParam("service_time_out", timeout_ms);
    ros::Duration duration(static_cast<double>(timeout_ms) / 1000.0);
    duration.sleep();
  }

  /**
   * @brief Helper for multiple service call test
   * Call the same service during Robot_hw update and test for immediate error return
   *
   * @param[in] bits Bit value of the serviceClient to call (least significant bit is 0)
   * @param[in] controller controller
   */
  void ServiceAlreadyUseTestHelper(const uint32_t bits,
                                   tmc_realtime_controllers::TriggerCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&TriggerCommandControllerTest::AlreadyUseProcess, this, _1));
  }
  void AlreadyUseProcess(const size_t index) {
    // Create message
    std_srvs::Trigger sb;
    sb.response.success = true;
    sb.response.message = "OK";

    // Service call itself completes successfully
    ros::ServiceClient client = root_nh_.serviceClient<std_srvs::Trigger>(names_[index]);
    EXPECT_TRUE(client.exists());
    EXPECT_TRUE(client.isValid());
    EXPECT_TRUE(client.call(sb));

    // Service call returns failure
    EXPECT_EQ(sb.response.success, false);
    EXPECT_STREQ(sb.response.message.c_str(), "This service is already in use.");

    // Drop flag as processing completes
    handle_datas_[index].has_request_ = false;
  }
};

// Confirm node initialization and initial controller values
TEST_F(TriggerCommandControllerTest, InitNomalTest) {
  tmc_hardware_interface::TriggerCommandHandle::Data init_value;
  init_value.response_ = false;

  std::string name = "init test";
  SCOPED_TRACE(name);
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  InitTestHelper(init_value, trigger_c);
  trigger_c.stopping(ros::Time::now());
}

// Init fails when necessary param is missing
TEST_F(TriggerCommandControllerTest, InitFailureTest_BadNamespace) {
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(trigger_c.init(&trigger_iface_, root_nh_, bad_controller_nh));
}

// Init fails with invalid param (0 or less)
TEST_F(TriggerCommandControllerTest, InitFailureTest_BadParam) {
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ros::NodeHandle bad_controller_nh("test_ko/tmc_trigger_command_controller");
  EXPECT_FALSE(trigger_c.init(&trigger_iface_, root_nh_, bad_controller_nh));
}

// When there are two nodes, confirm handle flag is set with proper service call allocation
TEST_F(TriggerCommandControllerTest, ServiceNomalTest_Allocation) {
  tmc_hardware_interface::TriggerCommandHandle::Data init_value;
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, trigger_c));

  boost::array<uint32_t, 5> params = {
      0b00, 0b01, 0b11, 0b10, 0b00,
  };
  BOOST_FOREACH (uint32_t bits, params) {
    std::string name = "bits = " + boost::lexical_cast<std::string>(bits);
    SCOPED_TRACE(name);
    ServiceNomalTestHelper(bits, trigger_c);
  }
}

// Confirm execution result is returned to caller during service call
TEST_F(TriggerCommandControllerTest, ServiceNomalTest_ResponseData) {
  tmc_hardware_interface::TriggerCommandHandle::Data init_value;
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, trigger_c));

  // Test for response
  boost::array<bool, 2> expect_bools = {true, false};
  BOOST_FOREACH (bool expect, expect_bools) {
    std::stringstream name;
    name << "expect = " << expect;
    SCOPED_TRACE(name.str());

    handle_datas_[0].response_ = expect;
    ServiceNomalTestHelper(1, trigger_c);
    std_srvs::Trigger sb;
    sb.response.success = expect;
    sb.response.message = "OK";
    CompareResponse(service_messages_[0].response, sb.response);
  }

  // Fixed value for message, so perform no test
}

// Flag automatically drops upon timeout occurrence
TEST_F(TriggerCommandControllerTest, ServiceFailureTest_Timeout) {
  tmc_hardware_interface::TriggerCommandHandle::Data init_value;
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, trigger_c));

  ServiceTimeoutTestHelper(1, trigger_c);
  std_srvs::Trigger sb;
  sb.response.success = false;
  sb.response.message = "Hardware did not respond. Timeout";
  CompareResponse(service_messages_[0].response, sb.response);

  // Confirm return to normal process
  sb.response.success = true;
  handle_datas_[0].response_ = true;
  sb.response.message = "OK";
  ServiceNomalTestHelper(1, trigger_c);
  CompareResponse(service_messages_[0].response, sb.response);
}

// During service execution, if another service call is made, the second service immediately returns error
TEST_F(TriggerCommandControllerTest, ServiceFailureTest_AlreadyInUse) {
  tmc_hardware_interface::TriggerCommandHandle::Data init_value;
  tmc_realtime_controllers::TriggerCommandController trigger_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, trigger_c));

  std_srvs::Trigger sb;
  sb.response.success = true;
  handle_datas_[0].response_ = true;
  sb.response.message = "OK";

  // ServiceAlreadyUseTestHelper(1, trigger_c);
  // CompareResponse(service_messages_[0].response, sb.response);

  // Confirm return to normal process
  ServiceNomalTestHelper(1, trigger_c);
  CompareResponse(service_messages_[0].response, sb.response);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  // Start roscore
  ros::init(argc, argv, "trigger_command_controller_test");
  int32_t ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
