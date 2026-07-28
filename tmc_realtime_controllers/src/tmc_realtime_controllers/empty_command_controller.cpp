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
#include "empty_command_controller.hpp"
#include "utils.hpp"

namespace tmc_realtime_controllers {

controller_interface::CallbackReturn EmptyCommandController::on_init() {
  command_interface_name_ = auto_declare<std::string>("command_interface_name", "");
  if (command_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_interface_name is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EmptyCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(command_interface_name_);
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration EmptyCommandController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
EmptyCommandController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  command_value_ = auto_declare<double>("command_value", 1.0);
  const auto use_no_request_command_value = auto_declare<bool>("use_no_request_command_value", false);
  if (use_no_request_command_value) {
    no_request_command_value_ = auto_declare<double>("no_request_command_value", 0.0);
  } else {
    no_request_command_value_ = std::nullopt;
  }

  const auto srv_name = auto_declare<std::string>("service_name", "~/trigger");
  srv_ = get_node()->create_service<std_srvs::srv::Empty>(
      srv_name, std::bind(&EmptyCommandController::Callback, this, std::placeholders::_1, std::placeholders::_2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EmptyCommandController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  has_command_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EmptyCommandController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
EmptyCommandController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // It's okay if some parts are skipped, so make it a simple implementation
  bool has_command;
  if (command_mutex_.try_lock()) {
    has_command = has_command_;
    has_command_ = false;
    command_mutex_.unlock();
  } else {
    has_command = false;
  }
  if (has_command) {
      SetCommandInterfaceValue(get_node()->get_logger(), command_interfaces_[0], command_value_);
  } else {
    if (no_request_command_value_) {
      SetCommandInterfaceValue(get_node()->get_logger(), command_interfaces_[0], no_request_command_value_.value());
    }
  }
  return controller_interface::return_type::OK;
}

void EmptyCommandController::Callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                      const std_srvs::srv::Empty::Response::SharedPtr response) {
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    has_command_ = true;
  }
  while (true) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!has_command_) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::EmptyCommandController,
                       controller_interface::ControllerInterface)
