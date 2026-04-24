// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_REALTIME_SUBSCRIPTION_BASE_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_REALTIME_SUBSCRIPTION_BASE_HPP_

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace tmc_joint_command_controller {

template <typename T>
class RealtimeSubscriptionBase {
 public:
  RealtimeSubscriptionBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                           const std::string& topic,
                           const rclcpp::QoS& qos = rclcpp::QoS(1))
      : node_(node) {
    ResetBuffer();

    sub_ = node->create_subscription<T>(
        topic, qos,
        std::bind(&RealtimeSubscriptionBase::Callback, this, std::placeholders::_1));
  }
  virtual ~RealtimeSubscriptionBase() = default;

  void ResetBuffer() {
    buffer_.writeFromNonRT(T());
  }

  T* Read() {
    return buffer_.readFromRT();
  }

 protected:
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node() const { return node_; }

  void Write(const T& msg) { buffer_.writeFromNonRT(msg); }

  virtual void Callback(const typename T::SharedPtr msg) = 0;

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<T> buffer_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_REALTIME_SUBSCRIPTION_BASE_HPP_
