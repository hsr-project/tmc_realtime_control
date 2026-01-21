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

#include <tmc_realtime_controllers/motion_command_limitter_controller.hpp>

#include <algorithm>
#include <limits>
#include <memory>

#include <fmt/core.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>

#include <tmc_utils/robot_description.hpp>


namespace {

template<typename T>
bool GetIndex(const rclcpp::Logger& logger,
              const std::vector<T>& interfaces,
              const std::string& name,
              const std::string& interface,
              size_t& index_out) {
  for (auto i = 0u; i < interfaces.size(); ++i) {
    if ((interfaces[i].get_prefix_name() == name) && (interfaces[i].get_interface_name() == interface)) {
      index_out = i;
      return true;
    }
  }
  RCLCPP_ERROR(logger, "Interface '%s/%s' not found", name.c_str(), interface.c_str());
  return false;
}

}  // namespace

namespace tmc_realtime_controllers {

std::vector<std::string> MotionCommandLimitter::command_interface_configuration() const {
  std::vector<std::string> names;
  if (use_command_position_) {
    names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  }
  if (use_command_velocity_) {
    names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return names;
}

std::vector<std::string> MotionCommandLimitter::state_interface_configuration() const {
  return {joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
          joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY};
}

bool MotionCommandLimitter::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                      const std::string& robot_description) {
  use_command_position_ = tmc_utils::GetParameter<bool>(node, parameter_namespace_ + "use_command_position", false);
  use_command_velocity_ = tmc_utils::GetParameter<bool>(node, parameter_namespace_ + "use_command_velocity", false);
  if (!use_command_position_ && !use_command_velocity_) {
    RCLCPP_ERROR(node->get_logger(),
                 "At least one of 'use_command_position' or 'use_command_velocity' must be true.");
    return false;
  }

  constexpr double kDefaultValue = 1.0e10;
  position_limit_lower_ = -kDefaultValue;
  position_limit_upper_ = kDefaultValue;
  double velocity_limit_local = kDefaultValue;
  const double acceleration_limit_local = kDefaultValue;

  // I don't like deep nesting, but considering the scope of the model, this is easier
  // Loading URDF shouldn't be a significant load, so it is done in each configure
  if (!robot_description.empty()) {
    urdf::Model urdf;
    if (urdf.initString(robot_description)) {
      const auto joint = urdf.getJoint(joint_name_);
      if (joint) {
        const auto limits = joint->limits;
        if (limits) {
          position_limit_lower_ = limits->lower;
          position_limit_upper_ = limits->upper;
          velocity_limit_local = limits->velocity;
        } else {
          RCLCPP_WARN(node->get_logger(), "Joint '%s' has no limits defined", joint_name_.c_str());
        }
        if (joint->type == urdf::Joint::CONTINUOUS) {
          position_limit_lower_ = -kDefaultValue;
          position_limit_upper_ = kDefaultValue;
        }
      } else {
        RCLCPP_WARN(node->get_logger(), "Joint '%s' not found in URDF", joint_name_.c_str());
      }
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to parse URDF contained in robot_description");
    }
  }

  // Overwrite if set by parameters
  position_limit_lower_ = tmc_utils::GetParameter<double>(
      node, parameter_namespace_ + "position_limit_lower", position_limit_lower_);
  position_limit_upper_ = tmc_utils::GetParameter<double>(
      node, parameter_namespace_ + "position_limit_upper", position_limit_upper_);
  velocity_limit_ = std::make_shared<tmc_utils::DynamicParameter<double>>(
      node, parameter_namespace_ + "velocity_limit", velocity_limit_local);
  acceleration_limit_ = std::make_shared<tmc_utils::DynamicParameter<double>>(
      node, parameter_namespace_ + "acceleration_limit", acceleration_limit_local);

  auto format_limit = [](double value) -> std::string {
    return fmt::format("{: .2f}", value);
  };
  RCLCPP_INFO(node->get_logger(), "Joint '%s' limits: position [%s, %s], velocity %s, acceleration %s",
              joint_name_.c_str(),
              format_limit(position_limit_lower_).c_str(),
              format_limit(position_limit_upper_).c_str(),
              format_limit(velocity_limit_->value()).c_str(),
              format_limit(acceleration_limit_->value()).c_str());
  return true;
}

bool MotionCommandLimitter::activate(const rclcpp::Logger& logger,
                                     const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                     const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  if (use_command_position_) {
    if (!GetIndex(logger, command_interfaces,
                  joint_name_, hardware_interface::HW_IF_POSITION, command_position_index_)) {
      return false;
    }
  }
  if (use_command_velocity_) {
    if (!GetIndex(logger, command_interfaces,
                  joint_name_, hardware_interface::HW_IF_VELOCITY, command_velocity_index_)) {
      return false;
    }
  }
  if (!GetIndex(logger, state_interfaces, joint_name_, hardware_interface::HW_IF_POSITION, state_position_index_) ||
      !GetIndex(logger, state_interfaces, joint_name_, hardware_interface::HW_IF_VELOCITY, state_velocity_index_)) {
    return false;
  }
  previous_position_ = state_interfaces[state_position_index_].get_value();
  previous_velocity_ = state_interfaces[state_velocity_index_].get_value();
  command_position_ = previous_position_;
  command_velocity_ = previous_velocity_;
  return true;
}

std::vector<hardware_interface::CommandInterface> MotionCommandLimitter::export_reference_interfaces(
    const std::string& node_name) {
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (use_command_position_) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        node_name, interface_namespace_ + hardware_interface::HW_IF_POSITION, &command_position_));
  }
  if (use_command_velocity_) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        node_name, interface_namespace_ + hardware_interface::HW_IF_VELOCITY, &command_velocity_));
  }
  return interfaces;
}

bool MotionCommandLimitter::update_and_write_commands(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const rclcpp::Duration& period,
    std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  const double velocity_limit_local = this->velocity_limit_->value();
  if (velocity_limit_local <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "velocity_limit must be positive");
    return false;
  }
  const double acceleration_limit_local = this->acceleration_limit_->value();
  if (acceleration_limit_local <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "acceleration_limit must be positive");
    return false;
  }

  const double dt = period.seconds();

  double vel_max_brake_safe;
  double vel_min_brake_safe;
  if (use_command_position_) {
    vel_max_brake_safe = std::sqrt(2.0 * acceleration_limit_local * (std::min(position_limit_upper_, command_position_) - previous_position_));  // NOLINT
    vel_min_brake_safe = -std::sqrt(2.0 * acceleration_limit_local * (previous_position_ - std::max(position_limit_lower_, command_position_)));  // NOLINT
  } else {
    vel_max_brake_safe = std::sqrt(2.0 * acceleration_limit_local * (position_limit_upper_ - previous_position_));
    vel_min_brake_safe = -std::sqrt(2.0 * acceleration_limit_local * (previous_position_ - position_limit_lower_));
  }

  const double vel_max_acc_limit = previous_velocity_ + acceleration_limit_local * dt;
  const double vel_min_acc_limit = previous_velocity_ - acceleration_limit_local * dt;

  const double vel_max = std::min({velocity_limit_local, vel_max_brake_safe, vel_max_acc_limit});
  const double vel_min = std::max({-velocity_limit_local, vel_min_brake_safe, vel_min_acc_limit});

  if (use_command_position_ && use_command_velocity_) {
    {
      const double vel_desired = (command_position_ - previous_position_) / dt;
      const double vel_saturated = std::clamp(vel_desired, vel_min, vel_max);
      const double pos_saturated = std::clamp(previous_position_ + vel_saturated * dt,
                                              position_limit_lower_, position_limit_upper_);
      command_interfaces[command_position_index_].set_value(pos_saturated);
      previous_position_ = pos_saturated;
    }
    {
      // TODO(Takeshita) 位置で計算したvel_saturatedを使うべきか否か？
      const double vel_saturated = std::clamp(command_velocity_, vel_min, vel_max);
      command_interfaces[command_velocity_index_].set_value(vel_saturated);
      previous_velocity_ = vel_saturated;
    }
  } else if (use_command_position_ && !use_command_velocity_) {
    const double vel_desired = (command_position_ - previous_position_) / dt;
    const double vel_saturated = std::clamp(vel_desired, vel_min, vel_max);
    const double pos_saturated = std::clamp(previous_position_ + vel_saturated * dt,
                                            position_limit_lower_, position_limit_upper_);

    command_interfaces[command_position_index_].set_value(pos_saturated);

    previous_position_ = pos_saturated;
    previous_velocity_ = vel_saturated;
  } else if (!use_command_position_ && use_command_velocity_) {
    const double vel_saturated = std::clamp(command_velocity_, vel_min, vel_max);

    command_interfaces[command_velocity_index_].set_value(vel_saturated);

    previous_position_ = state_interfaces[state_position_index_].get_value();
    previous_velocity_ = vel_saturated;
  }
  // TODO(Takeshita) previousは計算した値を使うべきか？ state_interfacesから取得した値を使うべきか？
  // previous_position_ = state_interfaces[state_position_index_].get_value();
  // previous_velocity_ = state_interfaces[state_velocity_index_].get_value();
  return true;
}


controller_interface::CallbackReturn MotionCommandLimitterController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotionCommandLimitterController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& limitter : limitters_) {
    const auto names = limitter->command_interface_configuration();
    conf.names.insert(conf.names.end(), names.begin(), names.end());
  }
  return conf;
}

controller_interface::InterfaceConfiguration MotionCommandLimitterController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& limitter : limitters_) {
    const auto names = limitter->state_interface_configuration();
    conf.names.insert(conf.names.end(), names.begin(), names.end());
  }
  return conf;
}


controller_interface::CallbackReturn MotionCommandLimitterController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto joint_names = auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  if (!joint_names.empty()) {
    for (const auto& joint_name : joint_names) {
      limitters_.emplace_back(std::make_shared<MotionCommandLimitter>(joint_name));
    }
  } else {
    const auto joint_name = auto_declare<std::string>("joint_name", "");
    if (joint_name.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Parameter 'joint_name' is not set. When 'joint_names' is specified, "
                   "'joint_name' must also be specified.");
      return controller_interface::CallbackReturn::ERROR;
    }
    limitters_.emplace_back(std::make_shared<MotionCommandLimitter>(joint_name, false));
  }

  const auto robot_description = tmc_utils::ResolveRobotDescription(get_node());
  for (const auto& limitter : limitters_) {
    if (!limitter->configure(get_node(), robot_description)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionCommandLimitterController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  for (auto& limitter : limitters_) {
    if (!limitter->activate(get_node()->get_logger(), command_interfaces_, state_interfaces_)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionCommandLimitterController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> MotionCommandLimitterController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto& limitter : limitters_) {
    // It is necessary to use get_node()->get_name() as the name
    auto ifs = limitter->export_reference_interfaces(get_node()->get_name());
    std::move(ifs.begin(), ifs.end(), std::back_inserter(interfaces));
  }
  // The size of reference_interfaces_ and interfaces must match
  reference_interfaces_.resize(interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  return interfaces;
}

controller_interface::return_type MotionCommandLimitterController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  for (auto& limitter : limitters_) {
    if (!limitter->update_and_write_commands(get_node(), period, command_interfaces_, state_interfaces_)) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace tmc_realtime_controllers

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::MotionCommandLimitterController,
                       controller_interface::ChainableControllerInterface)
