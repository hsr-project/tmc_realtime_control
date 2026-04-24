// Copyright (c) 2026 Toyota Motor Corporation
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

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_TEST_UTILS_HPP_
