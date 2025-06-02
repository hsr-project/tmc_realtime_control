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
#ifndef TMC_REALTIME_CONTROLLERS_SERVO_STATE_BROADCASTER_HPP_
#define TMC_REALTIME_CONTROLLERS_SERVO_STATE_BROADCASTER_HPP_
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <boost/range/adaptor/indexed.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <controller_interface/controller_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tmc_control_msgs/msg/servo_state.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>

namespace tmc_realtime_controllers {

template <typename TYPE>
std::optional<uint32_t> GetIndex(const std::vector<TYPE>& interfaces,
                                 const std::string& joint_name, const std::string& name) {
  for (const auto& interface : interfaces | boost::adaptors::indexed()) {
    if (interface.value().get_prefix_name() == joint_name && interface.value().get_interface_name() == name) {
      return interface.index();
    }
  }
  return std::nullopt;
}

template <typename ParameterType>
auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
                  const ParameterType& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterType>(name, default_value);
  } else {
    return node->get_parameter(name).get_value<ParameterType>();
  }
}

class ServoStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  ServoStateBroadcaster();
  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Initialization of parts other than ControllerInterface::init, separation for testing
  bool InitImpl();

 private:
  using RealtimePublisher = realtime_tools::RealtimePublisher<tmc_control_msgs::msg::ServoState>;

  rclcpp::Publisher<tmc_control_msgs::msg::ServoState>::SharedPtr publisher_impl_;

  std::unique_ptr<RealtimePublisher> publisher_;

  double last_published_time_;

  double publish_rate_;
  double expected_publish_time_;

  std::vector<std::string> joint_names_;
  std::vector<std::optional<uint32_t>> state_current_drive_mode_index_;
  std::vector<std::optional<uint32_t>> state_position_index_;
  std::vector<std::optional<uint32_t>> state_velocity_index_;
  std::vector<std::optional<uint32_t>> state_effort_index_;
  std::vector<std::optional<uint32_t>> state_temperature_index_;
  std::vector<std::optional<uint32_t>> state_current_index_;
  std::vector<std::optional<uint32_t>> state_mrpos_index_;
  std::vector<std::optional<uint32_t>> state_avagopos_index_;
  std::vector<std::optional<uint32_t>> state_error_status_index_;
};

}  // namespace tmc_realtime_controllers

#endif /*TMC_REALTIME_CONTROLLERS_SERVO_STATE_CONTROLLER_HPP_*/
