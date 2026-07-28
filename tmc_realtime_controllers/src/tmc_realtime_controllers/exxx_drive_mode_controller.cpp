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
/// @file exxx_drive_mode_controller.cpp
/// @brief Controller to change the drive mode
#include <limits>
#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tmc_control_msgs/msg/exxx_drive_mode.hpp>
#include <tmc_control_msgs/srv/change_exxx_drive_mode.hpp>
#include <tmc_realtime_controllers/exxx_drive_mode_controller.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>

#include "utils.hpp"

using tmc_control_msgs::msg::ExxxDriveMode;

namespace {

/// Waiting time for mode switching [s]
const int kDriveModeTick = 10;
/// Timeout for mode switching
const double kRequestTimeout = 10.0;
/// Default publishing rate of the mode [Hz]
const double kDefalutPublishRate = 10.0;

}  // unnamed namespace


namespace tmc_realtime_controllers {

bool ExxxDriveModeController::InitImpl() {
  joint_names_ = GetParameter(get_node(), "joints", std::vector<std::string>({ "" }));
  // If joints cannot be retrieved, respond with an error.
  if (joint_names_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "joints parameter is empty");
    return false;
  }

  if (get_node()->has_parameter("publish_rate")) {
    publish_rate_ = get_node()->get_parameter("publish_rate").get_value<double>();
  } else {
    publish_rate_ = get_node()->declare_parameter<double>("publish_rate", kDefalutPublishRate);
  }

  expected_publish_time_ = 1.0 / publish_rate_;

  joint_drivemode_publisher_ = get_node()->create_publisher<tmc_control_msgs::msg::JointExxxDriveMode>(
      "drive_mode", rclcpp::SystemDefaultsQoS());

  request_state_ = kNoRequest;
  std::vector<tmc_control_msgs::msg::ExxxDriveMode> drive_modes;
  drive_modes.reserve(joint_names_.size());
  drive_modes_buffer_.initRT(drive_modes);
  request_buffer_.initRT(drive_modes);

  return true;
}


controller_interface::return_type ExxxDriveModeController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period) {
  std::vector<tmc_control_msgs::msg::ExxxDriveMode> drive_modes;

  for (std::string joint_name : joint_names_) {
    std::optional<uint32_t> current_drive_mode_index = 0;
    current_drive_mode_index = GetIndex(state_interfaces_, joint_name, "current_drive_mode");
    if (!current_drive_mode_index.has_value()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint name was not found. Joint : " << joint_name);
      continue;
    }
    tmc_control_msgs::msg::ExxxDriveMode drive_mode;
    drive_mode.joint = joint_name;
    drive_mode.value = GetStateInterfaceValue(state_interfaces_[current_drive_mode_index.value()]);
    drive_modes.push_back(drive_mode);
  }

  drive_modes_buffer_.writeFromNonRT(drive_modes);

  // Publish drive_mode at the specified interval
  if ((time.seconds() - last_published_time_.seconds()) >= expected_publish_time_) {
    if (publisher_->trylock()) {
      tmc_control_msgs::msg::JointExxxDriveMode& msg = publisher_->msg_;
      msg.drive_modes = drive_modes;
      publisher_->unlockAndPublish();
      last_published_time_ = time;
    }
  }

  // Progress the request state in the order of Send->Receive->Done
  // The service side confirms the state transition after it becomes Done
  {
    boost::mutex::scoped_lock lock(request_lock_, boost::try_to_lock);
    if (lock) {
      if (request_state_ == kRequestSend) {
        request_state_ = kRequestReceive;
      } else if (request_state_ == kRequestReceive) {
        request_state_ = kRequestDone;
      }
    }
  }

  // Send the request if it is not empty
  if (!request_buffer_.readFromRT()->empty()) {
    for (ExxxDriveMode drive_mode : *request_buffer_.readFromRT()) {
      std::optional<uint32_t> command_drive_mode_index = 0;
      command_drive_mode_index = GetIndex(command_interfaces_, drive_mode.joint, "command_drive_mode");
      if (!command_drive_mode_index.has_value()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint name was not found. Joint : " << drive_mode.joint);
        continue;
      }
      SetCommandInterfaceValue(rclcpp::get_logger("rclcpp"),
                               command_interfaces_[command_drive_mode_index.value()],
                               static_cast<double>(drive_mode.value));
    }
    {
      boost::mutex::scoped_lock lock(request_lock_, boost::try_to_lock);
      if (lock) {
        request_state_ = kRequestSend;
        // Reset the request to empty
        std::vector<tmc_control_msgs::msg::ExxxDriveMode> empty;
        request_buffer_.writeFromNonRT(empty);
      }
    }
  }

  return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn ExxxDriveModeController::on_init() {
  if (InitImpl()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration ExxxDriveModeController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::string joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/command_drive_mode");
  }
  return conf;
}

controller_interface::InterfaceConfiguration ExxxDriveModeController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (std::string joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/current_drive_mode");
  }
  return conf;
}

controller_interface::CallbackReturn ExxxDriveModeController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  change_drive_mode_server_ = get_node()->create_service<tmc_control_msgs::srv::ChangeExxxDriveMode>(
      "change_drive_mode",
      std::bind(&ExxxDriveModeController::ChangeDriveModeCallBack, this, std::placeholders::_1, std::placeholders::_2));
  publisher_ = std::make_unique<RealtimePublisher>(joint_drivemode_publisher_);


  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ExxxDriveModeController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ExxxDriveModeController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ExxxDriveModeController::ChangeDriveModeCallBack(
    const std::shared_ptr<tmc_control_msgs::srv::ChangeExxxDriveMode::Request> request,
    const std::shared_ptr<tmc_control_msgs::srv::ChangeExxxDriveMode::Response> response) {
  // Immediately fail if the request is not in the registered handle
  std::vector<std::string> joints = joint_names_;
  for (ExxxDriveMode command_mode : request->drive_mode_request.drive_modes) {
    if (std::find(joints.begin(), joints.end(), command_mode.joint) == joints.end()) {
      response->success = false;
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Joint (" << command_mode.joint << ") resource is not found.");
      return;
    }
  }

  // Communicate the request to the real-time side
  request_buffer_.writeFromNonRT(request->drive_mode_request.drive_modes);

  // Wait until the new drive_mode is expected to arrive
  bool is_not_request_done = true;
  {
    boost::mutex::scoped_lock lock(request_lock_);
    bool is_not_request_done = (request_state_ != kRequestDone);
  }
  rclcpp::Time start = get_node()->get_clock()->now();
  // Wait until request_state_ becomes Done
  // End with failure if it times out
  while (is_not_request_done) {
    rclcpp::Time now = get_node()->get_clock()->now();
    if ((now.seconds() - start.seconds()) > kRequestTimeout) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Timeout to change drive_mode");
      response->success = false;
      {
        boost::mutex::scoped_lock lock(request_lock_);
        request_state_ = kNoRequest;
      }
      return;
    }
    {
      boost::mutex::scoped_lock lock(request_lock_);
      is_not_request_done = (request_state_ != kRequestDone);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kDriveModeTick));
  }
  {
    boost::mutex::scoped_lock lock(request_lock_);
    request_state_ = kNoRequest;
  }

  // Verify if the drive_mode was changed as requested
  std::vector<ExxxDriveMode> drive_modes =
    *drive_modes_buffer_.readFromNonRT();
  bool success = true;
  bool found = false;
  // Check if the request matches the current state
  for (ExxxDriveMode command_mode : request->drive_mode_request.drive_modes) {
    found = false;
    for (ExxxDriveMode current_mode : drive_modes) {
      if ((current_mode.joint == command_mode.joint) &&
          (current_mode.value == command_mode.value)) {
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot change drive_mode");
      success = false;
      break;
    }
  }

  response->success = success;
  return;
}

}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::ExxxDriveModeController, controller_interface::ControllerInterface)
