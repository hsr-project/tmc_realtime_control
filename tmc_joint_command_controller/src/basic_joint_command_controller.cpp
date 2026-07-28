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

#include <tmc_joint_command_controller/basic_joint_command_controller.hpp>

namespace {

template <typename ParameterT>
void EnsureDeclared(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    const ParameterT& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<ParameterT>(name, default_value);
  }
}

}  // namespace

namespace tmc_joint_command_controller {

controller_interface::CallbackReturn
BasicJointCommandController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  // A bit wasteful, but necessary to generate individually
  auto joints_info = std::make_shared<JointsInfo>(get_node());
  if (!joints_info->IsValid(get_node()->get_logger())) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  EnsureDeclared(get_node(), "command_source_names", std::vector<std::string>(
      {"position_jog",
       "velocity_jog",
       "joint_trajectory_action",
       "joint_trajectory_topic",
       "position_velocity_holding"}));
  // Default values are aligned with ros2_control/hardware_interface/include/mock_components/generic_system.hpp
  EnsureDeclared(get_node(), "use_control_mode_setting", true);
  const auto position_control_mode = auto_declare<int32_t>("position_control_mode", 0);
  const auto velocity_control_mode = auto_declare<int32_t>("velocity_control_mode", 1);

  EnsureDeclared(get_node(), "position_jog.type", "tmc_joint_command_controller/PositionJogTopic");
  EnsureDeclared(get_node(), "position_jog.target_control_mode", position_control_mode);

  EnsureDeclared(get_node(), "velocity_jog.type", "tmc_joint_command_controller/VelocityJogTopic");
  EnsureDeclared(get_node(), "velocity_jog.target_control_mode", velocity_control_mode);

  EnsureDeclared(get_node(), "joint_trajectory_action.type", "tmc_joint_command_controller/JointTrajectoryAction");
  EnsureDeclared(get_node(), "joint_trajectory_action.target_control_mode", position_control_mode);

  EnsureDeclared(get_node(), "joint_trajectory_topic.type", "tmc_joint_command_controller/JointTrajectoryTopic");
  EnsureDeclared(get_node(), "joint_trajectory_topic.target_control_mode", position_control_mode);

  EnsureDeclared(get_node(), "position_velocity_holding.type", "tmc_joint_command_controller/PositionVelocityHolding");

  for (const auto& joint_name : joints_info->names()) {
    const auto specific_position_control_mode = auto_declare<int32_t>(
        joint_name + ".position_control_mode", position_control_mode);
    const auto specific_velocity_control_mode = auto_declare<int32_t>(
        joint_name + ".velocity_control_mode", velocity_control_mode);

    EnsureDeclared(get_node(), std::string("position_jog.") + joint_name + ".target_control_mode",
                   specific_position_control_mode);
    EnsureDeclared(get_node(), std::string("velocity_jog.") + joint_name + ".target_control_mode",
                   specific_velocity_control_mode);
    EnsureDeclared(get_node(), std::string("joint_trajectory_action.") + joint_name + ".target_control_mode",
                   specific_position_control_mode);
    EnsureDeclared(get_node(), std::string("joint_trajectory_topic.") + joint_name + ".target_control_mode",
                   specific_position_control_mode);
  }

  // Startup tracking settings
  const auto open_loop_control = auto_declare<bool>("open_loop_control", true);
  EnsureDeclared(get_node(), "joint_trajectory_action.open_loop_control", open_loop_control);
  EnsureDeclared(get_node(), "joint_trajectory_topic.open_loop_control", open_loop_control);
  EnsureDeclared(get_node(), "position_velocity_holding.open_loop_control", open_loop_control);

  const auto use_velocity_state = auto_declare<bool>("use_velocity_state_for_trajectory", true);
  EnsureDeclared(get_node(), "joint_trajectory_action.use_velocity_state", use_velocity_state);
  EnsureDeclared(get_node(), "joint_trajectory_topic.use_velocity_state", use_velocity_state);

  return JointCommandController::on_configure(previous_state);
}

}  // namespace tmc_joint_command_controller

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_joint_command_controller::BasicJointCommandController,
                       controller_interface::ControllerInterface)
