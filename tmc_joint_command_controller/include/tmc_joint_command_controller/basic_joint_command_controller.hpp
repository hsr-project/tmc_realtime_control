//// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_BASIC_JOINT_COMMAND_CONTROLLER_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_BASIC_JOINT_COMMAND_CONTROLLER_HPP_

#include <tmc_joint_command_controller/joint_command_controller.hpp>

namespace tmc_joint_command_controller {

// 位置指令/速度指令/起動追従指令のいずれかを受け取る基本的なJointCommandController
class BasicJointCommandController : public JointCommandController {
 public:
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_BASIC_JOINT_COMMAND_CONTROLLER_HPP_
