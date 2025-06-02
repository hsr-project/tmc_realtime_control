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
/// @file exxx_drive_mode_controller.hpp
/// @brief Controller for changing drive mode and issuing status

#ifndef TMC_REALTIME_CONTROLLERS_EXXX_DRIVE_MODE_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERS_EXXX_DRIVE_MODE_CONTROLLER_HPP_
#include <memory>
#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <controller_interface/controller_interface.hpp>
#include <tmc_control_msgs/msg/exxx_drive_mode.hpp>
#include <tmc_control_msgs/msg/joint_exxx_drive_mode.hpp>
#include <tmc_control_msgs/srv/change_exxx_drive_mode.hpp>
#include <tmc_realtime_controllers/exxx_drive_mode_controller.hpp>

namespace tmc_realtime_controllers {

class ExxxDriveModeController : public controller_interface::ControllerInterface {
 public:
  enum RequestState {
    kNoRequest = 0,
    kRequestSend = 1,
    kRequestReceive = 2,
    kRequestDone = 3,
  } request_state_;

  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;

  bool InitImpl();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::vector<std::string> joint_names_;
  using RealtimePublisher = realtime_tools::RealtimePublisher<tmc_control_msgs::msg::JointExxxDriveMode>;
  std::unique_ptr<RealtimePublisher> publisher_;
  rclcpp::Publisher<tmc_control_msgs::msg::JointExxxDriveMode>::SharedPtr joint_drivemode_publisher_;

  rclcpp::Service<tmc_control_msgs::srv::ChangeExxxDriveMode>::SharedPtr change_drive_mode_server_;

  rclcpp::Time last_published_time_;
  double publish_rate_;
  double expected_publish_time_;
  realtime_tools::RealtimeBuffer<std::vector<tmc_control_msgs::msg::ExxxDriveMode> > request_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<tmc_control_msgs::msg::ExxxDriveMode> > drive_modes_buffer_;
  boost::mutex request_lock_;

  void ChangeDriveModeCallBack(const std::shared_ptr<tmc_control_msgs::srv::ChangeExxxDriveMode::Request> request,
                               const std::shared_ptr<tmc_control_msgs::srv::ChangeExxxDriveMode::Response> response);
};

}  // namespace tmc_realtime_controllers

#endif  // TMC_REALTIME_CONTROLLERS_DRIVE_MODE_CONTROLLER_HPP_
