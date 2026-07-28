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
#ifndef TMC_REALTIME_CONTROLLERS_MOTION_COMMAND_LIMITER_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERS_MOTION_COMMAND_LIMITER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <moveit_msgs/msg/joint_limits.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <tmc_utils/parameters.hpp>

namespace tmc_realtime_controllers {

class MotionCommandLimiter {
 public:
  struct Config {
    bool control_mode_switching;

    std::string control_mode_interface_name;
    // The default value is aligned with ros2_control/hardware_interface/include/mock_components/generic_system.hpp
    int32_t position_control_mode;
    int32_t velocity_control_mode;

    bool use_command_position;
    bool use_command_velocity;

    Config() : control_mode_switching(false),
               control_mode_interface_name("command_drive_mode"),
               position_control_mode(0),
               velocity_control_mode(1),
               use_command_position(false),
               use_command_velocity(false) {}

    bool UpdateFromParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                             const std::string& parameter_namespace,
                             bool publish_log_error = true);
  };

  explicit MotionCommandLimiter(const std::string& joint_name)
      : MotionCommandLimiter(joint_name, true) {}
  MotionCommandLimiter(const std::string& joint_name, bool use_joint_namespace)
      : joint_name_(joint_name),
        parameter_namespace_(use_joint_namespace ? joint_name_ + "." : ""),
        interface_namespace_(use_joint_namespace ? joint_name_ + "/" : "") {}

  // The function names corresponding to ros2_control functions follow their conventions
  std::vector<std::string> command_interface_configuration() const;
  std::vector<std::string> state_interface_configuration() const;

  bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const std::string& robot_description,
                 const double default_acceleration_time,
                 const Config& common_config);
  bool activate(const rclcpp::Logger& logger,
                const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  std::vector<hardware_interface::CommandInterface> export_reference_interfaces(const std::string& node_name);

  bool update_and_write_commands(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                 const rclcpp::Duration& period,
                                 std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                 const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  // From here on, naming rules follow the Google style guide
  double get_position_limit_upper() const { return position_limit_upper_; }
  double get_position_limit_lower() const { return position_limit_lower_; }

  void UpdateUseCommandPosition(bool value) { config_.use_command_position = value; }
  void UpdateUseCommandVelocity(bool value) { config_.use_command_velocity = value; }

  void UpdatePositionLimits(double upper, double lower) {
    position_limit_upper_ = upper;
    position_limit_lower_ = lower;
  }

  void UpdateLimitsCallback(const moveit_msgs::msg::JointLimits::SharedPtr msg);

 private:
  std::string joint_name_;
  std::string parameter_namespace_;
  std::string interface_namespace_;

  size_t command_position_index_;
  size_t command_velocity_index_;

  size_t state_position_index_;
  size_t state_velocity_index_;

  double previous_command_position_;
  double previous_command_velocity_;

  double command_position_;
  double command_velocity_;

  Config config_;

  double control_mode_value_;
  int32_t previous_control_mode_;
  size_t control_mode_index_;

  // Using optional would be correct, but for simplicity, double is used as a substitute
  double position_limit_upper_;
  double position_limit_lower_;
  tmc_utils::AtomicDynamicParameter<double>::Ptr velocity_limit_;
  tmc_utils::AtomicDynamicParameter<double>::Ptr acceleration_limit_;

  realtime_tools::RealtimeBuffer<double> velocity_limit_buffer_;
  realtime_tools::RealtimeBuffer<double> acceleration_limit_buffer_;

  template <typename T>
  auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                    const std::string& name,
                    const T& default_value) {
    return tmc_utils::GetParameter<T>(node, parameter_namespace_ + name, default_value);
  }
};


// Limiter alone might suffice, but there is a convention to name it **Controller, so we follow that
class MotionCommandLimiterController : public controller_interface::ChainableControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  // Behavior based on subscribing to topics is not assumed
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override {
    return controller_interface::return_type::OK;
  }

  // The 0th is used for compatibility
  double get_position_limit_upper() const { return limiters_[0]->get_position_limit_upper(); }
  double get_position_limit_lower() const { return limiters_[0]->get_position_limit_lower(); }

  void UpdateUseCommandPosition(bool value) { limiters_[0]->UpdateUseCommandPosition(value); }
  void UpdateUseCommandVelocity(bool value) { limiters_[0]->UpdateUseCommandVelocity(value); }

  void UpdatePositionLimits(double upper, double lower) { limiters_[0]->UpdatePositionLimits(upper, lower); }

 private:
  std::vector<std::shared_ptr<MotionCommandLimiter>> limiters_;

  rclcpp::Subscription<moveit_msgs::msg::JointLimits>::SharedPtr limits_subscription_;
  void LimitsCallback(const moveit_msgs::msg::JointLimits::SharedPtr msg);
};

}  // namespace tmc_realtime_controllers
#endif  // TMC_REALTIME_CONTROLLERS_MOTION_COMMAND_LIMITER_CONTROLLER_HPP_
