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
#include "digital_output_controller.hpp"

#include <string>

namespace tmc_realtime_controllers {

controller_interface::CallbackReturn DigitalOutputController::on_init() {
  auto_declare<std::string>("command_interface_name", "");
  command_interface_name_ = get_node()->get_parameter("command_interface_name").as_string();
  if (command_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_interface_name is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto_declare<std::string>("topic_name", "~/output");
  auto_declare<bool>("default_output", false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DigitalOutputController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(command_interface_name_);
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DigitalOutputController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
DigitalOutputController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      get_node()->get_parameter("topic_name").as_string(), 1,
      std::bind(&DigitalOutputController::Callback, this, std::placeholders::_1));

  command_.writeFromNonRT(get_node()->get_parameter("default_output").as_bool());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DigitalOutputController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  if (command_interfaces_.size() != 1) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Expected size of command_interface is 1, but actual size is " << command_interfaces_.size());
    controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DigitalOutputController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
DigitalOutputController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto output = *command_.readFromRT();
  command_interfaces_[0].set_value(output);
  return controller_interface::return_type::OK;
}

void DigitalOutputController::Callback(const std_msgs::msg::Bool::SharedPtr msg) {
  command_.writeFromNonRT(msg->data);
}

}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::DigitalOutputController,
                       controller_interface::ControllerInterface)
