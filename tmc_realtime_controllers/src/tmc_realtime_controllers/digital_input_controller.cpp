/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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

#include "digital_input_controller.hpp"


namespace tmc_realtime_controllers {

controller_interface::CallbackReturn DigitalInputController::on_init() {
  auto_declare<std::string>("state_interface_name", "");
  state_interface_name_ = get_node()->get_parameter("state_interface_name").as_string();
  if (state_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "state_interface_name is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto_declare<std::string>("topic_name", "~/input");
  auto_declare<bool>("inverse_mode", false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DigitalInputController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration DigitalInputController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.push_back(state_interface_name_);
  return state_interfaces_config;
}

controller_interface::CallbackReturn
DigitalInputController::on_configure(const rclcpp_lifecycle::State& /* previous_state */) {
  publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      get_node()->get_parameter("topic_name").as_string(), 1);
  is_inverse_mode_ = get_node()->get_parameter("inverse_mode").as_bool();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DigitalInputController::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Expected size state_interface is 1, but actual size is " << state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DigitalInputController::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
DigitalInputController::update(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // Publish State
  std_msgs::msg::Bool gpio_msg;
  gpio_msg.data = static_cast<bool>(state_interfaces_.at(0).get_value());

  // Inverse published state
  if (is_inverse_mode_) {
    gpio_msg.data = !gpio_msg.data;
  }

  publisher_->publish(gpio_msg);

  return controller_interface::return_type::OK;
}

}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tmc_realtime_controllers::DigitalInputController,
  controller_interface::ControllerInterface);
