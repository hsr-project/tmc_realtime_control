//// Copyright (c) 2026 Toyota Motor Corporation

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
  // ちょっともったいないが必要なので個別に生成
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
  // デフォルト値は ros2_control/hardware_interface/include/mock_components/generic_system.hpp に合わせている
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

  // 起動追従の設定
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
