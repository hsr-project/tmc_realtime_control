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
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>

namespace tmc_realtime_controllers {

ServoStateBroadcaster::ServoStateBroadcaster() {}

controller_interface::return_type ServoStateBroadcaster::init(const std::string& controller_name,
                                                              const std::string& namespace_,
                                                              const rclcpp::NodeOptions& node_options) {
  // NOTE: no member
  // node_options.enable_logger_service(true);
  const auto ret = ControllerInterface::init(controller_name, namespace_, node_options);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

bool ServoStateBroadcaster::InitImpl() {
  joint_names_ = GetParameter(get_node(), "joints", std::vector<std::string>({ "" }));
  // If joints cannot be obtained, respond with an error.
  if (joint_names_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "joints parameter is empty");
    return false;
  }
  publish_rate_ = GetParameter(get_node(), "publish_rate", 30.0);
  expected_publish_time_ = 1.0 / publish_rate_;

  return true;
}

controller_interface::InterfaceConfiguration ServoStateBroadcaster::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration ServoStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/current_drive_mode");
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    conf.names.push_back(joint_name + "/temperature");
    conf.names.push_back(joint_name + "/current");
    conf.names.push_back(joint_name + "/mr_pos");
    conf.names.push_back(joint_name + "/avago_pos");
    conf.names.push_back(joint_name + "/error_status");
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ServoStateBroadcaster::on_init() {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ServoStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Initialization of publisher
  publisher_impl_ =
      get_node()->create_publisher<tmc_control_msgs::msg::ServoState>("servo_states", rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ServoStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  // Initialize last_published_time_
  last_published_time_ = get_node()->get_clock()->now().seconds();

  // Initialize the index of each item
  for (auto joint_name : joint_names_) {
    state_current_drive_mode_index_.push_back(GetIndex(state_interfaces_, joint_name, "current_drive_mode"));
    state_position_index_.push_back(GetIndex(state_interfaces_, joint_name, hardware_interface::HW_IF_POSITION));
    state_velocity_index_.push_back(GetIndex(state_interfaces_, joint_name, hardware_interface::HW_IF_VELOCITY));
    state_effort_index_.push_back(GetIndex(state_interfaces_, joint_name, hardware_interface::HW_IF_EFFORT));
    state_temperature_index_.push_back(GetIndex(state_interfaces_, joint_name, "temperature"));
    state_current_index_.push_back(GetIndex(state_interfaces_, joint_name, "current"));
    state_mrpos_index_.push_back(GetIndex(state_interfaces_, joint_name, "mr_pos"));
    state_avagopos_index_.push_back(GetIndex(state_interfaces_, joint_name, "avago_pos"));
    state_error_status_index_.push_back(GetIndex(state_interfaces_, joint_name, "error_status"));
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ServoStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ServoStateBroadcaster::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period) {
  if (fabs(time.seconds() - last_published_time_) >= expected_publish_time_) {
    if (publisher_->trylock()) {
      tmc_control_msgs::msg::ServoState& msg = publisher_->msg_;
      msg.header.stamp = time;
      msg.name.clear();
      msg.current_drive_mode.clear();
      msg.present_velocity.clear();
      msg.present_position.clear();
      msg.present_effort.clear();
      msg.present_temperature.clear();
      msg.present_current.clear();
      msg.present_motor_shaft_position.clear();
      msg.present_driven_shaft_position.clear();
      msg.error_status.clear();
      msg.message.clear();

      for (int i = 0; i < joint_names_.size(); i++) {
        if ((!state_current_drive_mode_index_[i].has_value()) ||
            (!state_position_index_[i].has_value()) ||
            (!state_velocity_index_[i].has_value()) ||
            (!state_effort_index_[i].has_value()) ||
            (!state_temperature_index_[i].has_value()) ||
            (!state_current_index_[i].has_value()) ||
            (!state_mrpos_index_[i].has_value()) ||
            (!state_avagopos_index_[i].has_value()) ||
            (!state_error_status_index_[i].has_value())) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint name was not found. Joint : " << joint_names_[i]);
          continue;
        }
        msg.name.push_back(joint_names_[i]);
        msg.current_drive_mode.push_back(state_interfaces_[state_current_drive_mode_index_[i].value()].get_value());
        msg.present_position.push_back(state_interfaces_[state_position_index_[i].value()].get_value());
        msg.present_velocity.push_back(state_interfaces_[state_velocity_index_[i].value()].get_value());
        msg.present_effort.push_back(state_interfaces_[state_effort_index_[i].value()].get_value());
        msg.present_temperature.push_back(state_interfaces_[state_temperature_index_[i].value()].get_value());
        msg.present_current.push_back(state_interfaces_[state_current_index_[i].value()].get_value());
        msg.present_motor_shaft_position.push_back(state_interfaces_[state_mrpos_index_[i].value()].get_value());
        msg.present_driven_shaft_position.push_back(state_interfaces_[state_avagopos_index_[i].value()].get_value());
        msg.error_status.push_back(state_interfaces_[state_error_status_index_[i].value()].get_value());
      }
      publisher_->unlockAndPublish();
      last_published_time_ += expected_publish_time_;
    }
  }
  return controller_interface::return_type::OK;
}
}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::ServoStateBroadcaster, controller_interface::ControllerInterface);
