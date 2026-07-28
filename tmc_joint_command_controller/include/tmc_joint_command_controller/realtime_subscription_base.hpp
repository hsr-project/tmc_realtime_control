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
