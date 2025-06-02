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
#include <std_srvs/Empty.h>

#include <tmc_hardware_interface/empty_command_interface.hpp>
#include "../src/tmc_realtime_controllers/empty_command_controller.hpp"

#include "test_methods.hpp"

// empty_command_controller
// EmptyCommandController

namespace {
const size_t kHandleCount = 2;  //!/ Number of handles to register
}  // namespace

class EmptyCommandControllerTest : public ::testing::Test {
 public:
  EmptyCommandControllerTest() : controller_nh_("test_ok/tmc_empty_command_controller") {
    // Register handle and create corresponding publisher
    for (size_t i = 0; i < kHandleCount; i++) {
      // Topic name is "empty*"
      names_[i] = "/empty";
      names_[i] += boost::lexical_cast<std::string>(i + 1);
      tmc_hardware_interface::EmptyCommandHandle empty_handle(names_[i], handle_datas_[i]);
      empty_iface_.registerHandle(empty_handle);
      service_clients_[i] = root_nh_.serviceClient<std_srvs::Empty>(names_[i]);
    }
  }

 protected:
  ros::NodeHandle root_nh_;                                    //!/ root_nh
  ros::NodeHandle controller_nh_;                              //!/ controller_nh
  tmc_hardware_interface::EmptyCommandInterface empty_iface_;  //!/ Interface
  bool is_call_service_;                //!/ Flag set when CallServices is complete
  boost::mutex is_call_service_mutex_;  //!/ Mutex for is_call_service_ flag

  boost::array<std::string, kHandleCount> names_;                                              //!/ Node names
  boost::array<tmc_hardware_interface::EmptyCommandHandle::Data, kHandleCount> handle_datas_;  //!/ Data for Handle
  boost::array<std_srvs::Empty, kHandleCount> service_messages_;    //!/ Messages for services
  boost::array<ros::ServiceClient, kHandleCount> service_clients_;  //!/ Corresponding ServiceClient

  /**
   * @brief Handle state comparison
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareHandle(const tmc_hardware_interface::EmptyCommandHandle::Data& x,
                     const tmc_hardware_interface::EmptyCommandHandle::Data& y) const {
    EXPECT_EQ(x.has_request_, y.has_request_);
  }

  /**
   * @brief Comparison of responses
   * @param[in] x Comparison target
   * @param[in] y Comparison target
   */
  void CompareResponse(const std_srvs::Empty::Response& x, const std_srvs::Empty::Response& y) const {
    (void)(x);  // unused
    (void)(y);  // unused
  }

  /**
   * @brief Helper for init test
   * Handle value does not change when there is no service call
   * Test to confirm that the handle flag is down when there are no service calls
   * @param initial_value
   * @param controller
   */
  void InitTestHelper(const tmc_hardware_interface::EmptyCommandHandle::Data& initial_value,
                      tmc_realtime_controllers::EmptyCommandController& controller) {
    ASSERT_TRUE(controller.init(&empty_iface_, root_nh_, controller_nh_)) << "Init succeeds unconditionally";
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
   * @brief Helper for calling service in a separate thread
   * Test to confirm that calling the service in a separate thread processes correctly.
   * Calling this method without preparing an asynchronous spinner will lead to a deadlock in the test.
   * It is recommended to use via the CallServices method, which also manages the asynchronous spinner
   *
   * @param[in] index Index of the service to call
   */
  void CallService(const uint32_t index) {
    EXPECT_TRUE(service_clients_[index].exists());
    EXPECT_TRUE(service_clients_[index].isValid());
    EXPECT_TRUE(service_clients_[index].call(service_messages_[index]));
  }

  /**
   * @brief Helper for managing AsyncSpinner and calling service in a separate thread
   * After starting AsyncSpinner, call the services specified by bits.
   * This function waits until all service calls are completed, so
   * Callers should consider calling this function itself in a separate thread.
   * @param[in] bits Bit value of the serviceClient to call (lowest bit is 0)
   */
  void CallServices(const uint32_t bits) {
    ros::AsyncSpinner async_spinner((kHandleCount * 2) + 1);  // Spinning for callback resolution, max number of services + 1 extra
    async_spinner.start();

    // Service call
    boost::thread_group threads;
    for (size_t i = 0; i < kHandleCount; ++i) {
      if ((bits & (1 << i)) != 0) {
        threads.create_thread(boost::bind(&EmptyCommandControllerTest::CallService, this, i));
      }
    }

    threads.join_all();  // Wait until all service calls are complete
  }

  /**
   * @brief Helper template for service test
   * Test to confirm that the handle flag is set correctly after calling the service
   * Customize Robot_hw processing for each test
   * Specify the serviceClient to call by bit.
   * @param[in] bits Bit value of the serviceClient to call (lowest bit is 0)
   * @param[in] controller controller
   * @param[in] func Processing performed by Robot_hw during execution
   */
  void ServiceTestHelperBase(const uint32_t bits, tmc_realtime_controllers::EmptyCommandController& controller,
                             boost::function<void(const size_t)> func) {
    ROS_INFO("thread start");

    // Service call
    boost::thread call_thread(&EmptyCommandControllerTest::CallServices, this, bits);
    // Wait until the service call is complete
    ros::Duration(0.1).sleep();

    // Move to Processing in this Update
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

    // If handle_.has_request_ is false, move to kResponsed
    controller.update(ros::Time::now(), ros::Duration());

    // Service complete, is_call_service_ becomes false
    call_thread.join();

    // Move to Standby
    controller.update(ros::Time::now(), ros::Duration());
    ros::spinOnce();
  }

  /**
   * @brief Helper for normal service call test
   * Robot_hw drops the handle_.has_request_ flag immediately
   *
   * @param[in] bits Bit value of the serviceClient to call (lowest bit is 0)
   * @param[in] controller controller
   */
  void ServiceNomalTestHelper(const uint32_t bits, tmc_realtime_controllers::EmptyCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&EmptyCommandControllerTest::NomalProcess, this, _1));
  }
  void NomalProcess(const size_t index) {
    // For normal cases, drop the flag as process complete
    handle_datas_[index].has_request_ = false;
  }

  /**
   * @brief Helper for service call test on timeout
   * Robot_hw waits for the timeout duration defined by ROS Param
   *
   * @param[in] bits Bit value of the serviceClient to call (lowest bit is 0)
   * @param[in] controller controller
   */
  void ServiceTimeoutTestHelper(const uint32_t bits, tmc_realtime_controllers::EmptyCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&EmptyCommandControllerTest::TimeoutProcess, this, _1));
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
   * @brief Helper for service multi-call test
   * Test whether instant error is returned by calling the same service during Robot_hw update
   *
   * @param[in] bits Bit value of the serviceClient to call (lowest bit is 0)
   * @param[in] controller controller
   */
  void ServiceAlreadyUseTestHelper(const uint32_t bits, tmc_realtime_controllers::EmptyCommandController& controller) {
    ServiceTestHelperBase(bits, controller, boost::bind(&EmptyCommandControllerTest::AlreadyUseProcess, this, _1));
  }
  void AlreadyUseProcess(const size_t index) {
    // Message creation
    std_srvs::Empty sb;

    // Service call itself completes successfully
    ros::ServiceClient client = root_nh_.serviceClient<std_srvs::Empty>(names_[index]);
    EXPECT_TRUE(client.exists());
    EXPECT_TRUE(client.isValid());
    EXPECT_TRUE(client.call(sb));

    // Service call returns failure

    // Drop the flag as process complete
    handle_datas_[index].has_request_ = false;
  }
};

// Check node initialization and initial values of the controller
TEST_F(EmptyCommandControllerTest, InitNomalTest) {
  tmc_hardware_interface::EmptyCommandHandle::Data init_value;

  std::string name = "init test";
  SCOPED_TRACE(name);
  tmc_realtime_controllers::EmptyCommandController empty_c;
  InitTestHelper(init_value, empty_c);
  empty_c.stopping(ros::Time::now());
}

// Fails to init when required Params are missing
TEST_F(EmptyCommandControllerTest, InitFailureTest_BadNamespace) {
  tmc_realtime_controllers::EmptyCommandController empty_c;
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(empty_c.init(&empty_iface_, root_nh_, bad_controller_nh));
}

// Fails to init when Params are invalid (0 or below)
TEST_F(EmptyCommandControllerTest, InitFailureTest_BadParam) {
  tmc_realtime_controllers::EmptyCommandController empty_c;
  ros::NodeHandle bad_controller_nh("test_ko/tmc_empty_command_controller");
  EXPECT_FALSE(empty_c.init(&empty_iface_, root_nh_, bad_controller_nh));
}

// When there are two nodes, the handle flag is set correctly by a normal service call assignment
TEST_F(EmptyCommandControllerTest, ServiceNomalTest_Allocation) {
  tmc_hardware_interface::EmptyCommandHandle::Data init_value;
  tmc_realtime_controllers::EmptyCommandController empty_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, empty_c));

  boost::array<uint32_t, 5> params = {
      0b00, 0b01, 0b11, 0b10, 0b00,
  };
  BOOST_FOREACH (uint32_t bits, params) {
    std::string name = "bits = " + boost::lexical_cast<std::string>(bits);
    SCOPED_TRACE(name);
    ServiceNomalTestHelper(bits, empty_c);
  }
}

// Automatically drops the flag when timeout occurs
TEST_F(EmptyCommandControllerTest, ServiceFailureTest_Timeout) {
  tmc_hardware_interface::EmptyCommandHandle::Data init_value;
  tmc_realtime_controllers::EmptyCommandController empty_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, empty_c));

  ServiceTimeoutTestHelper(1, empty_c);

  // Confirm if it is restored to the normal state
  ServiceNomalTestHelper(1, empty_c);
}

// When calling another service during one service execution, the second service returns an error immediately
TEST_F(EmptyCommandControllerTest, ServiceFailureTest_AlreadyInUse) {
  tmc_hardware_interface::EmptyCommandHandle::Data init_value;
  tmc_realtime_controllers::EmptyCommandController empty_c;
  ASSERT_NO_FATAL_FAILURE(InitTestHelper(init_value, empty_c));

  // ServiceAlreadyUseTestHelper(1, empty_c);
  // CompareResponse(service_messages_[0].response, sb.response);

  // Confirm if it is restored to the normal state
  ServiceNomalTestHelper(1, empty_c);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  // Launch roscore
  ros::init(argc, argv, "empty_command_controller_test");
  int32_t ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
