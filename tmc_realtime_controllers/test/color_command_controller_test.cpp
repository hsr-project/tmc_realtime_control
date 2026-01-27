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
#include <cmath>

#include <algorithm>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <tmc_hardware_interface/color_command_interface.hpp>
#include "../src/tmc_realtime_controllers/color_command_controller.hpp"

#include "test_methods.hpp"

namespace {
const size_t kHandleCount = 2;  //!/ Number of handles to register
}  // namespace

class ColorCommandControllerTest : public ::testing::Test {
 public:
  ColorCommandControllerTest() : controller_nh_("test_ok/tmc_color_command_controller") {
    std::fill(values_r_.begin(), values_r_.end(), 0.0);
    std::fill(values_g_.begin(), values_g_.end(), 0.0);
    std::fill(values_b_.begin(), values_b_.end(), 0.0);
    // Register handle and create corresponding publisher
    for (size_t i = 0; i < kHandleCount; i++) {
      // Topic name is "color*"
      names_[i] = "color";
      names_[i] += boost::lexical_cast<std::string>(i + 1);
      tmc_hardware_interface::ColorCommandHandle colorhandle(names_[i], &values_r_[i], &values_g_[i], &values_b_[i]);
      coloriface_.registerHandle(colorhandle);
      publishers_[i] = root_nh_.advertise<std_msgs::ColorRGBA>(names_[i], 10);
    }
  }

 protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle controller_nh_;
  tmc_hardware_interface::ColorCommandInterface coloriface_;

  boost::array<std::string, kHandleCount> names_;
  boost::array<double, kHandleCount> values_r_;
  boost::array<double, kHandleCount> values_g_;
  boost::array<double, kHandleCount> values_b_;
  boost::array<ros::Publisher, kHandleCount> publishers_;
};

// Check node initialization and controller initial values
TEST_F(ColorCommandControllerTest, InitNomalTest) {
  tmc_realtime_controllers::ColorCommandController color_c;
  EXPECT_TRUE(color_c.init(&coloriface_, root_nh_, controller_nh_)) << "initialize";
  // Controller initialization
  color_c.starting(ros::Time::now());
  // Since the initial value is 0.0,
  // Set the receiving variable to a non-zero value to detect
  for (size_t i = 0; i < kHandleCount; i++) {
    values_r_[i] = 1.0;
    values_g_[i] = 1.0;
    values_b_[i] = 1.0;
  }

  color_c.update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  for (size_t i = 0; i < kHandleCount; i++) {
    EXPECT_DOUBLE_EQ(values_r_[i], 0.0) << "Initial value is 0.0";
    EXPECT_DOUBLE_EQ(values_g_[i], 0.0) << "Initial value is 0.0";
    EXPECT_DOUBLE_EQ(values_b_[i], 0.0) << "Initial value is 0.0";
  }

  color_c.stopping(ros::Time::now());
}

// Topic reception test
TEST_F(ColorCommandControllerTest, PublishNomalTest_R) {
  // Controller initialization
  tmc_realtime_controllers::ColorCommandController color_c;
  EXPECT_TRUE(color_c.init(&coloriface_, root_nh_, controller_nh_)) << "Initialization";
  color_c.starting(ros::Time::now());
  color_c.update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  boost::array<std_msgs::ColorRGBA, kHandleCount> input;
  boost::array<double, kHandleCount> output;
  std::fill(output.begin(), output.end(), 0.0);

  input[0].g = 0.0;
  input[1].g = 0.0;
  input[0].b = 0.0;
  input[1].b = 0.0;

  {
    SCOPED_TRACE("01");
    input[0].r = 0.0;
    input[1].r = 1.0;
    output[0] = 0.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_r_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("11");
    input[0].r = 1.0;
    input[1].r = 1.0;
    output[0] = 1.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_r_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("10");
    input[0].r = 1.0;
    input[1].r = 0.0;
    output[0] = 1.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_r_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("00");
    input[0].r = 0.0;
    input[1].r = 0.0;
    output[0] = 0.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_r_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
    }
  }
}

TEST_F(ColorCommandControllerTest, PublishNomalTest_G) {
  // Controller initialization
  tmc_realtime_controllers::ColorCommandController color_c;
  EXPECT_TRUE(color_c.init(&coloriface_, root_nh_, controller_nh_)) << "Initialization";
  color_c.starting(ros::Time::now());
  color_c.update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  boost::array<std_msgs::ColorRGBA, kHandleCount> input;
  boost::array<double, kHandleCount> output;
  std::fill(output.begin(), output.end(), 0.0);

  input[0].b = 0.0;
  input[1].b = 0.0;
  input[0].g = 0.0;
  input[1].g = 0.0;

  {
    SCOPED_TRACE("01");
    input[0].g = 0.0;
    input[1].g = 1.0;
    output[0] = 0.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_g_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("11");
    input[0].g = 1.0;
    input[1].g = 1.0;
    output[0] = 1.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_g_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("10");
    input[0].g = 1.0;
    input[1].g = 0.0;
    output[0] = 1.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_g_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("00");
    input[0].g = 0.0;
    input[1].g = 0.0;
    output[0] = 0.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_g_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_b_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
    }
  }
}

TEST_F(ColorCommandControllerTest, PublishNomalTest_B) {
  // Controller initialization
  tmc_realtime_controllers::ColorCommandController color_c;
  EXPECT_TRUE(color_c.init(&coloriface_, root_nh_, controller_nh_)) << "Initialization";
  color_c.starting(ros::Time::now());
  color_c.update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  boost::array<std_msgs::ColorRGBA, kHandleCount> input;
  boost::array<double, kHandleCount> output;
  std::fill(output.begin(), output.end(), 0.0);

  input[0].r = 0.0;
  input[1].r = 0.0;
  input[0].g = 0.0;
  input[1].g = 0.0;

  {
    SCOPED_TRACE("01");
    input[0].b = 0.0;
    input[1].b = 1.0;
    output[0] = 0.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_b_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("11");
    input[0].b = 1.0;
    input[1].b = 1.0;
    output[0] = 1.0;
    output[1] = 1.0;
    RunOutputInterfaceTest(input, output, publishers_, values_b_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("10");
    input[0].b = 1.0;
    input[1].b = 0.0;
    output[0] = 1.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_b_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
    }
  }

  {
    SCOPED_TRACE("00");
    input[0].b = 0.0;
    input[1].b = 0.0;
    output[0] = 0.0;
    output[1] = 0.0;
    RunOutputInterfaceTest(input, output, publishers_, values_b_, &color_c);
    for (size_t i = 0; i < kHandleCount; i++) {
      EXPECT_DOUBLE_EQ(values_r_[i], 0.0);
      EXPECT_DOUBLE_EQ(values_g_[i], 0.0);
    }
  }
}

// Test if the specified color is set when a timeout occurs
TEST_F(ColorCommandControllerTest, TestoutTest) {
  // Controller initialization
  tmc_realtime_controllers::ColorCommandController color_c;
  EXPECT_TRUE(color_c.init(&coloriface_, root_nh_, controller_nh_)) << "Initialization";
  color_c.starting(ros::Time::now());
  color_c.update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  boost::array<std_msgs::ColorRGBA, kHandleCount> input;
  boost::array<double, kHandleCount> output;
  std::fill(output.begin(), output.end(), 0.0);

  for (size_t i = 0; i < kHandleCount; ++i) {
    // Set input to (1, 1, 1)
    input[i].r = 1.0;
    input[i].g = 1.0;
    input[i].b = 1.0;
    // Set output to (0, 0, 0)
    values_r_[i] = 0.0;
    values_g_[i] = 0.0;
    values_b_[i] = 0.0;
    // Publish input
    publishers_[i].publish(input[i]);
  }

  // Update process
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  color_c.update(ros::Time::now(), ros::Duration());

  // (1, 1, 1) is output
  for (size_t i = 0; i < kHandleCount; ++i) {
    EXPECT_DOUBLE_EQ(values_r_[i], 1.0);
    EXPECT_DOUBLE_EQ(values_g_[i], 1.0);
    EXPECT_DOUBLE_EQ(values_b_[i], 1.0);
  }

  ros::Duration(1.0).sleep();
  ros::spinOnce();
  color_c.update(ros::Time::now(), ros::Duration());

  // Check timeout
  // The specified timeout color (0.5, 0.5, 0.5) is output
  for (size_t i = 0; i < kHandleCount; ++i) {
    EXPECT_DOUBLE_EQ(0.5, values_r_[i]);
    EXPECT_DOUBLE_EQ(0.5, values_g_[i]);
    EXPECT_DOUBLE_EQ(0.5, values_b_[i]);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  // Start roscore
  ros::init(argc, argv, "color_command_controller_test");
  int32_t ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
