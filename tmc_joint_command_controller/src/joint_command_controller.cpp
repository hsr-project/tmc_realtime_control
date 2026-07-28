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

#include <tmc_joint_command_controller/joint_command_controller.hpp>

#include "common.hpp"

namespace {
void MakeUnique(std::vector<std::string>& vec) {
  std::sort(vec.begin(), vec.end());
  vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}
}  // namespace

namespace tmc_joint_command_controller {

controller_interface::CallbackReturn JointCommandController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (use_control_mode_setting_) {
    AddInterfaces(joints_info_->command_joints(), control_mode_interface_name_, command_interfaces_config.names);
  }

  for (const auto& source : command_sources_) {
    const auto& names = source->GetCommandInterfaces();
    command_interfaces_config.names.insert(command_interfaces_config.names.end(), names.begin(), names.end());
  }
  MakeUnique(command_interfaces_config.names);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointCommandController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_publisher_->AddStateInterfaces(state_interfaces_config.names);

  for (const auto& source : command_sources_) {
    const auto& names = source->GetStateInterfaces();
    state_interfaces_config.names.insert(state_interfaces_config.names.end(), names.begin(), names.end());
  }
  MakeUnique(state_interfaces_config.names);

  return state_interfaces_config;
}

controller_interface::return_type
JointCommandController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Find the highest priority active command; if priorities are the same, use the most recent command
  IJointCommandSource::Ptr highest_priority_active_source = nullptr;
  auto highest_priority = std::numeric_limits<int32_t>::min();
  int64_t last_command_time_ns = 0;

  for (const auto& source : command_sources_) {
    source->ReadAndUpdate(time, period, desired_state_);
    if (!source->HasCommand()) {
      continue;
    }

    if (source->GetPriority() > highest_priority) {
      highest_priority_active_source = source;
      highest_priority = source->GetPriority();
      last_command_time_ns = source->GetLastCommandTime().nanoseconds();
    } else if (source->GetPriority() == highest_priority) {
      if (source->GetLastCommandTime().nanoseconds() > last_command_time_ns) {
        highest_priority_active_source = source;
        last_command_time_ns = source->GetLastCommandTime().nanoseconds();
      }
    }
  }

  // Update command values
  if (highest_priority_active_source) {
    WriteControlMode(highest_priority_active_source->GetTargetControlMode());
    highest_priority_active_source->WriteCommand();

    desired_state_ = highest_priority_active_source->GetDesiredState();
  }

  // Stop inactive commands
  for (const auto& source : command_sources_) {
    if (source != highest_priority_active_source) {
      source->Preempt();
    }
  }


  // Publish state
  state_publisher_->Publish(time, desired_state_);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
JointCommandController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  joints_info_ = std::make_shared<JointsInfo>(get_node());
  if (!joints_info_->IsValid(get_node()->get_logger())) {
    return controller_interface::CallbackReturn::ERROR;
  }

  use_control_mode_setting_ = auto_declare<bool>("use_control_mode_setting", false);
  if (use_control_mode_setting_) {
    control_mode_interface_name_ = auto_declare<std::string>("control_mode_interface_name", "command_drive_mode");
  }

  command_sources_.clear();
  const auto command_source_names = auto_declare<std::vector<std::string>>(
      "command_source_names", std::vector<std::string>());
  if (command_source_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No command source specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto& name : command_source_names) {
    const auto type = auto_declare<std::string>(name + ".type", "");
    const auto source = command_source_loader_->Create(type);
    if (!source) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create command source instance for type: %s", type.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!source->Init(get_node(), name, this)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize command source: %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!source->Configure(joints_info_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure command source: %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    command_sources_.push_back(source);
  }

  state_publisher_ = std::make_shared<StatePublisher>(get_node(), joints_info_, this);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointCommandController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  for (const auto& source : command_sources_) {
    if (!source->Activate(command_interfaces_, state_interfaces_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate command source.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (use_control_mode_setting_) {
    control_mode_indices_ = GetIndices(command_interfaces_, joints_info_->command_joints(),
                                       control_mode_interface_name_);
    previous_control_mode_.resize(control_mode_indices_.size(), std::numeric_limits<double>::lowest());
  }

  state_publisher_->Activate(state_interfaces_);

  desired_state_ = trajectory_msgs::msg::JointTrajectoryPoint();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointCommandController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointCommandController::SetCommand(size_t index, double command_value) {
  SetCommandInterfaceValue(get_node()->get_logger(), command_interfaces_[index], command_value);
}

double JointCommandController::GetState(size_t index) const {
  return GetStateInterfaceValue(state_interfaces_[index]);
}


void JointCommandController::WriteControlMode(const std::vector<double>& target_control_mode) {
  // Since control_mode_index_map_ is empty, this if statement is not strictly necessary, but return early just in case
  if (!use_control_mode_setting_) {
    return;
  }
  if (target_control_mode.empty()) {
    return;
  }
  for (size_t i = 0; i < control_mode_indices_.size(); ++i) {
    const auto& target_mode = target_control_mode[i];
    if (std::abs(previous_control_mode_[i] - target_mode) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    previous_control_mode_[i] = target_mode;
    SetCommand(control_mode_indices_[i], target_mode);
  }
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(
    tmc_joint_command_controller::JointCommandController,
    controller_interface::ControllerInterface)
