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
#ifndef TMC_JOINT_COMMAND_CONTROLLER_TEST_UTILS_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_TEST_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace tmc_joint_command_controller {

template<typename ValueType, typename ContainerType>
void AssertIn(const ValueType& value, const ContainerType& container) {
  EXPECT_NE(std::find(container.begin(), container.end(), value), container.end());
}

void AssertEq(const std::vector<double>& expected, const std::vector<double>& actual, double abs_error = 1.0e-6) {
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(expected[i], actual[i], abs_error);
  }
}

template<typename TYPE>
class SubscriptionCounter {
 public:
  using Ptr = std::shared_ptr<SubscriptionCounter>;

  SubscriptionCounter(const rclcpp::Node::SharedPtr& node,
                      const std::string& topic_name) : count_(0) {
    subscriber_ = node->template create_subscription<TYPE>(
        topic_name, 1, std::bind(&SubscriptionCounter<TYPE>::Callback, this, std::placeholders::_1));
  }

  uint32_t count() const { return count_; }
  TYPE last_msg() const { return last_msg_; }
  void reset() { count_ = 0; }

 private:
  void Callback(const typename TYPE::SharedPtr msg) {
    ++count_;
    last_msg_ = *msg;
  }
  typename rclcpp::Subscription<TYPE>::SharedPtr subscriber_;
  uint32_t count_;
  TYPE last_msg_;
};

template<typename TYPE>
void SetCommandInterfaceValue(const rclcpp::Logger& logger,
                              TYPE& interface,
                              const double value) {
  EXPECT_TRUE(interface->set_value(value));
}

template<typename TYPE>
double GetStateInterfaceValue(const TYPE& interface) {
  std::optional<double> opt_value = interface->get_optional();
  if (opt_value.has_value()) {
    return opt_value.value();
  } else {
    ADD_FAILURE() << "Failure get StateInterface value.";
  }
}


}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_TEST_UTILS_HPP_
