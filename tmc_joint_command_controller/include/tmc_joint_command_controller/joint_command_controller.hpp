// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_CONTROLLER_HPP_
#define TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_CONTROLLER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <tmc_joint_command_controller/joint_command_source.hpp>
#include <tmc_joint_command_controller/joints_info.hpp>
#include <tmc_joint_command_controller/state_publisher.hpp>

namespace tmc_joint_command_controller {

class CommandSourceLoader {
 public:
  using Ptr = std::shared_ptr<CommandSourceLoader>;

  CommandSourceLoader() : impl_("tmc_joint_command_controller", "tmc_joint_command_controller::IJointCommandSource") {}
  ~CommandSourceLoader() = default;

  virtual IJointCommandSource::Ptr Create(const std::string& type) {
    return impl_.createSharedInstance(type);
  }

 private:
  pluginlib::ClassLoader<IJointCommandSource> impl_;
};


class JointCommandController : public controller_interface::ControllerInterface, public Accessor {
 public:
  JointCommandController() : command_source_loader_(std::make_shared<CommandSourceLoader>()) {}
  explicit JointCommandController(const CommandSourceLoader::Ptr& loader) : command_source_loader_(loader) {}

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  void SetCommand(size_t index, double command_value) override;
  double GetState(size_t index) const override;

 private:
  JointsInfo::Ptr joints_info_;

  CommandSourceLoader::Ptr command_source_loader_;
  std::vector<IJointCommandSource::Ptr> command_sources_;

  bool use_control_mode_setting_;

  void WriteControlMode(const std::vector<double>& target_control_mode);

  std::vector<double> previous_control_mode_;
  std::vector<size_t> control_mode_indices_;
  std::string control_mode_interface_name_;

  StatePublisher::Ptr state_publisher_;

  trajectory_msgs::msg::JointTrajectoryPoint desired_state_;
};

}  // namespace tmc_joint_command_controller
#endif  // TMC_JOINT_COMMAND_CONTROLLER_JOINT_COMMAND_CONTROLLER_HPP_
