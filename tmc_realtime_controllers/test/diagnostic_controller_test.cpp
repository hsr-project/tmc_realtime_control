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
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <tmc_hardware_interface/diagnostic_interface.hpp>
#include "../src/tmc_realtime_controllers/diagnostic_controller.hpp"

#include "test_methods.hpp"

namespace {
const size_t kHandleCount = 4;  //!/ Number of handles to register

/**
 * @brief Allocation of handle and Value types
 */
enum TypeAllocate {
  kIntIndex = 0,
  kUintIndex = 1,
  kDoubleIndex = 2,
  kStringIndex = 3,
};

const size_t kBufferSize = 100;
const size_t kMessageBufferSize = 5;
bool FindDiagByName(const diagnostic_msgs::DiagnosticArray& array, const std::string& name) {
  BOOST_FOREACH (const diagnostic_msgs::DiagnosticStatus& s, array.status) {
    if (s.name == name) {
      return true;
    }
  }
  return false;
}
std::string GetFiestValueByName(const diagnostic_msgs::DiagnosticArray& array, const std::string& name) {
  BOOST_FOREACH (const diagnostic_msgs::DiagnosticStatus& s, array.status) {
    if (s.name == name) {
      return s.values.begin()->value;
    }
  }
  return "";
}
std::string GetMessageByName(const diagnostic_msgs::DiagnosticArray& array, const std::string& name) {
  BOOST_FOREACH (const diagnostic_msgs::DiagnosticStatus& s, array.status) {
    if (s.name == name) {
      return s.message;
    }
  }
  return "";
}
}  // namespace

class DiagnosticControllerTest : public ::testing::Test {
 private:
  typedef tmc_hardware_interface::DiagnosticHandle::FixedSizeKeyValueFactory<kBufferSize> KeyValueFactory;

 public:
  DiagnosticControllerTest()
      : controller_nh_("test_ok/tmc_diagnostic_controller"),
        int_value_(0),
        uint_value_(0),
        double_value_(0.0),
        string_value_() {
    // Register handle and create corresponding subscriber
    for (size_t i = 0; i < kHandleCount; ++i) {
      levels_[i] = 0;
      messages_[i] = "";
      messages_[i].reserve(kMessageBufferSize);

      subscribe_datas_[i] =
          boost::make_shared<StateSubscriber<diagnostic_msgs::DiagnosticArray> >("/diagnostics", &root_nh_);
    }
    EXPECT_GE(kHandleCount, 4);
    // int
    names_[kIntIndex] = "int_diag";
    tmc_hardware_interface::DiagnosticHandle diag_handle_int(names_[kIntIndex], names_[kIntIndex],
                                                             &(levels_[kIntIndex]), &(messages_[kIntIndex]));
    diag_handle_int.AddKeyValue(KeyValueFactory::Create("int_value", int_value_));
    diag_iface_.registerHandle(diag_handle_int);

    // uint
    names_[kUintIndex] = "uint_diag";
    tmc_hardware_interface::DiagnosticHandle diag_handle_uint(names_[kUintIndex], names_[kUintIndex],
                                                              &(levels_[kUintIndex]), &(messages_[kUintIndex]));
    diag_handle_uint.AddKeyValue(KeyValueFactory::Create("uint_value", uint_value_));
    diag_iface_.registerHandle(diag_handle_uint);

    // double
    names_[kDoubleIndex] = "double_diag";
    tmc_hardware_interface::DiagnosticHandle diag_handle_double(names_[kDoubleIndex], names_[kDoubleIndex],
                                                                &(levels_[kDoubleIndex]), &(messages_[kDoubleIndex]));
    diag_handle_double.AddKeyValue(KeyValueFactory::Create("double_value", double_value_));
    diag_iface_.registerHandle(diag_handle_double);

    // string
    names_[kStringIndex] = "string_diag";
    tmc_hardware_interface::DiagnosticHandle diag_handle_string(names_[kStringIndex], names_[kStringIndex],
                                                                &(levels_[kStringIndex]), &(messages_[kStringIndex]));
    diag_handle_string.AddKeyValue(KeyValueFactory::Create("string_value", string_value_));
    diag_iface_.registerHandle(diag_handle_string);
  }

  /**
   * @brief Basic test for controller initialization
   * Initialize the controller using default params and calculate duration
   * @param[out] controller controller
   * @param[out] duration Duration
   */
  void InitialTestHelper(tmc_realtime_controllers::DiagnosticController& controller, ros::Duration& duration) {
    ASSERT_TRUE(controller.init(&diag_iface_, root_nh_, controller_nh_));
    controller.starting(ros::Time::now());

    double rate;
    controller_nh_.getParam("publish_rate", rate);
    duration = ros::Duration((1.0 / rate) * 2.0);  // Sleep twice the publish_rate to ensure it is published
  }

  /**
   * @brief Test for one cycle of ros_controll1
   * Wait for subscribe, update controller1 cycle, and verify update results
   *
   * @param[in] expect_int Expected value of int_value
   * @param[in] expect_uint Expected value of uint_value
   * @param[in] expect_double Expected value of double_value
   * @param[in] expect_string Expected value of string_value
   * @param[in] duration Waiting time
   * @param[in] controller controller
   */
  void SingleCycleTest(const std::string& expect_int, const std::string& expect_uint, const std::string& expect_double,
                       const std::string& expect_string, const ros::Duration& duration,
                       tmc_realtime_controllers::DiagnosticController& controller) {
    for (size_t i = 0; i < kHandleCount; ++i) {
      subscribe_datas_[i]->ResetIsSubscribe();
    }

    duration.sleep();
    controller.update(ros::Time::now(), ros::Duration());
    duration.sleep();
    ros::spinOnce();

    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_TRUE(subscribe_datas_[i]->IsSubscribe());
      for (size_t j = 0; j < kHandleCount; ++j) {
        EXPECT_TRUE(FindDiagByName(subscribe_datas_[i]->GetLastMessage(), names_[j]));
      }
    }
    EXPECT_STREQ(GetFiestValueByName(subscribe_datas_[kIntIndex]->GetLastMessage(), names_[kIntIndex]).c_str(),
                 expect_int.c_str());
    EXPECT_STREQ(GetFiestValueByName(subscribe_datas_[kUintIndex]->GetLastMessage(), names_[kUintIndex]).c_str(),
                 expect_uint.c_str());
    EXPECT_STREQ(GetFiestValueByName(subscribe_datas_[kDoubleIndex]->GetLastMessage(), names_[kDoubleIndex]).c_str(),
                 expect_double.c_str());
    EXPECT_STREQ(GetFiestValueByName(subscribe_datas_[kStringIndex]->GetLastMessage(), names_[kStringIndex]).c_str(),
                 expect_string.c_str());
  }

 protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle controller_nh_;
  tmc_hardware_interface::DiagnosticInterface diag_iface_;

  boost::array<std::string, kHandleCount> names_;     //!/ hw_id
  boost::array<uint8_t, kHandleCount> levels_;        //!/ level
  boost::array<std::string, kHandleCount> messages_;  //!/ message
  boost::array<boost::shared_ptr<StateSubscriber<diagnostic_msgs::DiagnosticArray> >, kHandleCount>
      subscribe_datas_;  //!/ Corresponding subscriber

  int32_t int_value_;         //!/ int_value
  uint32_t uint_value_;       //!/ uint_value
  double double_value_;       //!/ double_value
  std::string string_value_;  //!/ string_value
};

// Check node initialization and controller initial values
TEST_F(DiagnosticControllerTest, InitNomalTest) {
  tmc_realtime_controllers::DiagnosticController diag_c;

  ASSERT_TRUE(diag_c.init(&diag_iface_, root_nh_, controller_nh_));
  // Probably due to the influence of the realtime_publisher thread
  // A phenomenon occurs in Jenkins where the test itself fails if the destructor is called immediately
  // Add sleep to stabilize the test
  ros::Duration(0.2).sleep();
}

// Initialization fails if publish_rate is not defined in ros param
TEST_F(DiagnosticControllerTest, InitFailureTest_BadNamespace) {
  tmc_realtime_controllers::DiagnosticController *diag_c;
  diag_c = new tmc_realtime_controllers::DiagnosticController();
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(diag_c->init(&diag_iface_, root_nh_, bad_controller_nh));
}

// Initialization fails if publish_rate is 0 or less
TEST_F(DiagnosticControllerTest, InitFailureTest_BadParam) {
  tmc_realtime_controllers::DiagnosticController *diag_c;
  diag_c = new tmc_realtime_controllers::DiagnosticController();
  ros::NodeHandle bad_controller_nh("test_ko/tmc_diagnostic_controller");
  EXPECT_FALSE(diag_c->init(&diag_iface_, root_nh_, bad_controller_nh));
}

// Normal test case
TEST_F(DiagnosticControllerTest, SubscribeNomalTest) {
  tmc_realtime_controllers::DiagnosticController diag_c;
  ros::Duration duration;

  ASSERT_NO_FATAL_FAILURE(InitialTestHelper(diag_c, duration));

  {
    // Initial conditions
    SCOPED_TRACE("init");
    SingleCycleTest("0", "0", "0", "", duration, diag_c);
  }

  {
    // Allocation of int_value_ is correct and normal type conversion is performed
    SCOPED_TRACE("int_value_");
    int_value_ = 1;
    SingleCycleTest("1", "0", "0", "", duration, diag_c);
    int_value_ = 100;
    SingleCycleTest("100", "0", "0", "", duration, diag_c);
  }

  {
    // Allocation of uint_value_ is correct and normal type conversion is performed
    SCOPED_TRACE("uint_value_");
    uint_value_ = 1;
    SingleCycleTest("100", "1", "0", "", duration, diag_c);
    uint_value_ = 100;
    SingleCycleTest("100", "100", "0", "", duration, diag_c);
  }

  {
    // Allocation of double_value_ is correct and normal type conversion is performed
    SCOPED_TRACE("double_value_");
    double_value_ = 1.5;
    SingleCycleTest("100", "100", "1.5", "", duration, diag_c);
    double_value_ = 1.0 / 3.0;
    std::string expect_string = boost::lexical_cast<std::string>(1.0 / 3.0);
    EXPECT_LT(expect_string.length(), kBufferSize);
    SingleCycleTest("100", "100", expect_string, "", duration, diag_c);
    double_value_ = 0.0;
  }

  {
    // Allocation of string_value_ is correct and normal type conversion is performed
    SCOPED_TRACE("string_value_");
    string_value_ = "test";
    SingleCycleTest("100", "100", "0", string_value_, duration, diag_c);
    string_value_ = "test test test";
    SingleCycleTest("100", "100", "0", string_value_, duration, diag_c);
  }
}

TEST_F(DiagnosticControllerTest, SubscribeAssertTest) {
  // Initial conditions
  tmc_realtime_controllers::DiagnosticController diag_c;
  ros::Duration duration;
  ASSERT_NO_FATAL_FAILURE(InitialTestHelper(diag_c, duration));
  {
    SCOPED_TRACE("init");
    SingleCycleTest("0", "0", "0", "", duration, diag_c);
  }

  {
    // When the message exceeds the value at buffer initialization,
    // Only the buffer size is copied
    size_t reserved = diag_iface_.getHandle(names_[kIntIndex]).getMessageMaxLength();
    messages_[kIntIndex] = "12345678901234567890";
    EXPECT_GT(messages_[kIntIndex].length(), reserved);

    std::string test_string = messages_[kIntIndex].substr(0, reserved - 1);
    SingleCycleTest("0", "0", "0", "", duration, diag_c);
    EXPECT_STREQ(GetMessageByName(subscribe_datas_[kIntIndex]->GetLastMessage(), names_[kIntIndex]).c_str(),
                 test_string.c_str());
  }
}

int main(int argc, char* argv[]) {
  try {
    testing::InitGoogleTest(&argc, argv);
    // Start roscore
    ros::init(argc, argv, "diagnostic_controller_test");
    int32_t ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
  } catch (...) {
    std::cerr << "Not expected exception" << std::endl;
  }
}
