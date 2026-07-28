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

#include <tmc_realtime_controllers/motion_command_limiter_controller.hpp>

#include <algorithm>
#include <limits>
#include <memory>

#include <fmt/core.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>

#include <tmc_utils/qos.hpp>
#include <tmc_utils/robot_description.hpp>

#include "utils.hpp"

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

bool MotionCommandLimiter::Config::UpdateFromParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                       const std::string& parameter_namespace,
                                                       bool publish_log_error) {
  auto update_func = [this, node, parameter_namespace](auto& field, const std::string& name) {
    field = tmc_utils::GetParameter<std::decay_t<decltype(field)>>(node, parameter_namespace + name, field);
  };
  update_func(control_mode_switching, "control_mode_switching");
  if (control_mode_switching) {
    update_func(control_mode_interface_name, "control_mode_interface_name");
    update_func(position_control_mode, "position_control_mode");
    update_func(velocity_control_mode, "velocity_control_mode");
  } else {
    update_func(use_command_position, "use_command_position");
    update_func(use_command_velocity, "use_command_velocity");
    if (!use_command_position && !use_command_velocity) {
      if (publish_log_error) {
        RCLCPP_ERROR(node->get_logger(),
                     "At least one of 'use_command_position' or 'use_command_velocity' must be true.");
      }
      return false;
    }
  }
  return true;
}

std::vector<std::string> MotionCommandLimiter::command_interface_configuration() const {
  std::vector<std::string> names;
  if (config_.use_command_position || config_.control_mode_switching) {
    names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  }
  if (config_.use_command_velocity || config_.control_mode_switching) {
    names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  if (config_.control_mode_switching) {
    names.push_back(joint_name_ + "/" + config_.control_mode_interface_name);
  }
  return names;
}

std::vector<std::string> MotionCommandLimiter::state_interface_configuration() const {
  return {joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
          joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY};
}

bool MotionCommandLimiter::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     const std::string& robot_description,
                                     const double default_acceleration_time,
                                     const Config& common_config) {
  config_ = common_config;
  if (!config_.UpdateFromParameter(node, parameter_namespace_)) {
    return false;
  }

  control_mode_value_ = static_cast<double>(config_.position_control_mode);
  previous_control_mode_ = config_.position_control_mode;

  constexpr double kDefaultValue = 1.0e10;
  position_limit_lower_ = -kDefaultValue;
  position_limit_upper_ = kDefaultValue;
  double velocity_limit_local = kDefaultValue;

  // I don't like deep nesting, but considering the scope of the model, this approach is easier.
  // Loading URDF shouldn't be a significant load, so it is done in each configure.
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

  // Overwrite if it is set in the parameters.
  position_limit_lower_ = GetParameter<double>(node, "position_limit_lower", position_limit_lower_);
  position_limit_upper_ = GetParameter<double>(node, "position_limit_upper", position_limit_upper_);

  velocity_limit_buffer_.writeFromNonRT(velocity_limit_local);
  velocity_limit_ = std::make_shared<tmc_utils::AtomicDynamicParameter<double>>(
      node, parameter_namespace_ + "velocity_limit", velocity_limit_local,
      [this](const double& new_value) { velocity_limit_buffer_.writeFromNonRT(new_value); });

  double acceleration_limit_local = kDefaultValue;
  if (default_acceleration_time > 0.0) {
    acceleration_limit_local = velocity_limit_local / default_acceleration_time;
  }
  acceleration_limit_ = std::make_shared<tmc_utils::AtomicDynamicParameter<double>>(
      node, parameter_namespace_ + "acceleration_limit", acceleration_limit_local,
      [this](const double& new_value) { acceleration_limit_buffer_.writeFromNonRT(new_value); });

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

bool MotionCommandLimiter::activate(const rclcpp::Logger& logger,
                                    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  if (config_.use_command_position || config_.control_mode_switching) {
    if (!GetIndex(logger, command_interfaces,
                  joint_name_, hardware_interface::HW_IF_POSITION, command_position_index_)) {
      return false;
    }
  }
  if (config_.use_command_velocity || config_.control_mode_switching) {
    if (!GetIndex(logger, command_interfaces,
                  joint_name_, hardware_interface::HW_IF_VELOCITY, command_velocity_index_)) {
      return false;
    }
  }
  if (config_.control_mode_switching) {
    if (!GetIndex(logger, command_interfaces, joint_name_, config_.control_mode_interface_name, control_mode_index_)) {
      return false;
    }
  }
  if (!GetIndex(logger, state_interfaces, joint_name_, hardware_interface::HW_IF_POSITION, state_position_index_) ||
      !GetIndex(logger, state_interfaces, joint_name_, hardware_interface::HW_IF_VELOCITY, state_velocity_index_)) {
    return false;
  }
  previous_command_position_ = GetStateInterfaceValue(state_interfaces[state_position_index_]);
  previous_command_velocity_ = GetStateInterfaceValue(state_interfaces[state_velocity_index_]);
  command_position_ = previous_command_position_;
  command_velocity_ = previous_command_velocity_;
  return true;
}

std::vector<hardware_interface::CommandInterface> MotionCommandLimiter::export_reference_interfaces(
    const std::string& node_name) {
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (config_.use_command_position || config_.control_mode_switching) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        node_name, interface_namespace_ + hardware_interface::HW_IF_POSITION, &command_position_));
  }
  if (config_.use_command_velocity || config_.control_mode_switching) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        node_name, interface_namespace_ + hardware_interface::HW_IF_VELOCITY, &command_velocity_));
  }
  if (config_.control_mode_switching) {
    interfaces.emplace_back(hardware_interface::CommandInterface(
        node_name, interface_namespace_ + config_.control_mode_interface_name, &control_mode_value_));
  }

  return interfaces;
}

bool MotionCommandLimiter::update_and_write_commands(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const rclcpp::Duration& period,
    std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  const auto velocity_limit_ptr = velocity_limit_buffer_.readFromRT();
  if (!velocity_limit_ptr) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Failed to read velocity_limit");
    return false;
  }
  const auto velocity_limit_local = *velocity_limit_ptr;
  if (velocity_limit_local <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "velocity_limit must be positive");
    return false;
  }
  const auto acceleration_limit_ptr = acceleration_limit_buffer_.readFromRT();
  if (!acceleration_limit_ptr) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Failed to read acceleration_limit");
    return false;
  }
  const auto acceleration_limit_local = *acceleration_limit_ptr;
  if (acceleration_limit_local <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "acceleration_limit must be positive");
    return false;
  }
  if (config_.control_mode_switching) {
    SetCommandInterfaceValue(node->get_logger(), command_interfaces[control_mode_index_], control_mode_value_);

    const auto current_control_mode = static_cast<int32_t>(control_mode_value_);
    if (current_control_mode == config_.position_control_mode) {
      config_.use_command_position = true;
      config_.use_command_velocity = false;
      if (previous_control_mode_ != config_.position_control_mode) {
        previous_command_position_ = GetStateInterfaceValue(state_interfaces[state_position_index_]);
      }
    } else if (current_control_mode == config_.velocity_control_mode) {
      config_.use_command_position = false;
      config_.use_command_velocity = true;
    } else {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                            "Invalid control mode value: %d", current_control_mode);
      return false;
    }
    previous_control_mode_ = current_control_mode;
  }

  const double dt = period.seconds();

  double vel_max_brake_safe = 0.0;
  double vel_min_brake_safe = 0.0;
  double vel_max_pos_limit = 0.0;
  double vel_min_pos_limit = 0.0;
  if (config_.use_command_position) {
    vel_max_brake_safe = std::sqrt(2.0 * acceleration_limit_local * (std::min(position_limit_upper_, command_position_) - previous_command_position_));  // NOLINT
    vel_min_brake_safe = -std::sqrt(2.0 * acceleration_limit_local * (previous_command_position_ - std::max(position_limit_lower_, command_position_)));  // NOLINT

    // Since position clamping is applied, no need to consider it.
    vel_max_pos_limit = std::numeric_limits<double>::max();
    vel_min_pos_limit = -std::numeric_limits<double>::max();
  } else {
    const auto current_position = GetStateInterfaceValue(state_interfaces[state_position_index_]);
    if (position_limit_upper_ > current_position) {
      vel_max_brake_safe = std::sqrt(2.0 * acceleration_limit_local * (position_limit_upper_ - current_position));
      vel_max_pos_limit = (position_limit_upper_ - current_position) / dt;
    }
    if (position_limit_lower_ < current_position) {
      vel_min_brake_safe = -std::sqrt(2.0 * acceleration_limit_local * (current_position - position_limit_lower_));
      vel_min_pos_limit = (position_limit_lower_ - current_position) / dt;
    }
  }

  const double vel_max_acc_limit = previous_command_velocity_ + acceleration_limit_local * dt;
  const double vel_min_acc_limit = previous_command_velocity_ - acceleration_limit_local * dt;

  const double vel_max = std::min({velocity_limit_local, vel_max_brake_safe, vel_max_pos_limit, vel_max_acc_limit});
  const double vel_min = std::max({-velocity_limit_local, vel_min_brake_safe, vel_min_pos_limit, vel_min_acc_limit});

  if (config_.use_command_position && config_.use_command_velocity) {
    {
      const double vel_desired = (command_position_ - previous_command_position_) / dt;
      const double vel_saturated = std::clamp(vel_desired, vel_min, vel_max);
      const double pos_saturated = std::clamp(previous_command_position_ + vel_saturated * dt,
                                              position_limit_lower_, position_limit_upper_);
      SetCommandInterfaceValue(node->get_logger(), command_interfaces[command_position_index_], pos_saturated);
      previous_command_position_ = pos_saturated;
    }
    {
      // TODO(Takeshita) 位置で計算したvel_saturatedを使うべきか否か？
      const double vel_saturated = std::clamp(command_velocity_, vel_min, vel_max);
      SetCommandInterfaceValue(node->get_logger(), command_interfaces[command_velocity_index_], vel_saturated);
      previous_command_velocity_ = vel_saturated;
    }
  } else if (config_.use_command_position && !config_.use_command_velocity) {
    const double vel_desired = (command_position_ - previous_command_position_) / dt;
    const double vel_saturated = std::clamp(vel_desired, vel_min, vel_max);
    const double pos_saturated = std::clamp(previous_command_position_ + vel_saturated * dt,
                                            position_limit_lower_, position_limit_upper_);

    SetCommandInterfaceValue(node->get_logger(), command_interfaces[command_position_index_], pos_saturated);

    previous_command_position_ = pos_saturated;
    previous_command_velocity_ = vel_saturated;
  } else if (!config_.use_command_position && config_.use_command_velocity) {
    const double vel_saturated = std::clamp(command_velocity_, vel_min, vel_max);

    SetCommandInterfaceValue(node->get_logger(), command_interfaces[command_velocity_index_], vel_saturated);

    previous_command_velocity_ = vel_saturated;
  }
  // TODO(Takeshita) previousは計算した値を使うべきか？ state_interfacesから取得した値を使うべきか？
  // previous_position_ = state_interfaces[state_position_index_].get_value();
  // previous_velocity_ = state_interfaces[state_velocity_index_].get_value();
  return true;
}

void MotionCommandLimiter::UpdateLimitsCallback(const moveit_msgs::msg::JointLimits::SharedPtr msg) {
  if (msg->joint_name != joint_name_) {
    return;
  }
  // Position is not updated as there is no need for dynamic changes.
  // Jerk is not supported.
  // TODO(Takeshita) ROSのパラメータへの反映をしたほうがいいと思うが，処理時間等未計測なので，ひとまずは未実装
  if (msg->has_velocity_limits) {
    velocity_limit_buffer_.writeFromNonRT(msg->max_velocity);
  }
  if (msg->has_acceleration_limits) {
    acceleration_limit_buffer_.writeFromNonRT(msg->max_acceleration);
  }
}


controller_interface::CallbackReturn MotionCommandLimiterController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotionCommandLimiterController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& limiter : limiters_) {
    const auto names = limiter->command_interface_configuration();
    conf.names.insert(conf.names.end(), names.begin(), names.end());
  }
  return conf;
}

controller_interface::InterfaceConfiguration MotionCommandLimiterController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& limiter : limiters_) {
    const auto names = limiter->state_interface_configuration();
    conf.names.insert(conf.names.end(), names.begin(), names.end());
  }
  return conf;
}


controller_interface::CallbackReturn MotionCommandLimiterController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  const auto joint_names = auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  if (!joint_names.empty()) {
    for (const auto& joint_name : joint_names) {
      limiters_.emplace_back(std::make_shared<MotionCommandLimiter>(joint_name));
    }
  } else {
    const auto joint_name = auto_declare<std::string>("joint_name", "");
    if (joint_name.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Parameter 'joint_name' is not set. When 'joint_names' is specified, "
                   "'joint_name' must also be specified.");
      return controller_interface::CallbackReturn::ERROR;
    }
    limiters_.emplace_back(std::make_shared<MotionCommandLimiter>(joint_name, false));
  }

  // As long as individual settings are valid, no validation is performed for common settings.
  MotionCommandLimiter::Config config;
  (void)config.UpdateFromParameter(get_node(), "", false);

  const auto robot_description = tmc_utils::ResolveRobotDescription(get_node());
  const auto default_acceleration_time = auto_declare<double>("default_acceleration_time", 0.0);
  for (const auto& limiter : limiters_) {
    if (!limiter->configure(get_node(), robot_description, default_acceleration_time, config)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  limits_subscription_ = get_node()->create_subscription<moveit_msgs::msg::JointLimits>(
      "~/joint_limits", tmc_utils::BestEffortQoS(std::max<size_t>(1, joint_names.size())),
      std::bind(&MotionCommandLimiterController::LimitsCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionCommandLimiterController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  for (auto& limiter : limiters_) {
    if (!limiter->activate(get_node()->get_logger(), command_interfaces_, state_interfaces_)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionCommandLimiterController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> MotionCommandLimiterController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto& limiter : limiters_) {
    // The name needs to be set to get_node()->get_name().
    auto ifs = limiter->export_reference_interfaces(get_node()->get_name());
    std::move(ifs.begin(), ifs.end(), std::back_inserter(interfaces));
  }
  // The sizes of reference_interfaces_ and interfaces must match.
  reference_interfaces_.resize(interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  return interfaces;
}

controller_interface::return_type MotionCommandLimiterController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (period.nanoseconds() == 0) {
    // In the Gazebo simulation environment, the update cycle can become unstable and sometimes be zero.
    // To avoid division by zero, processing is skipped in such cases.
    return controller_interface::return_type::OK;
  }
  for (auto& limiter : limiters_) {
    if (!limiter->update_and_write_commands(get_node(), period, command_interfaces_, state_interfaces_)) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

void MotionCommandLimiterController::LimitsCallback(const moveit_msgs::msg::JointLimits::SharedPtr msg) {
  for (auto& limiter : limiters_) {
    limiter->UpdateLimitsCallback(msg);
  }
}

}  // namespace tmc_realtime_controllers

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::MotionCommandLimiterController,
                       controller_interface::ChainableControllerInterface)
