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

#include <cassert>
#include <string>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <tmc_realtime_controllers/diffdrive_command_controller.hpp>

namespace tmc_realtime_controllers {


bool DiffdriveCommandController::init(
    hardware_interface::VelocityJointInterface* hw,
    ros::NodeHandle& node) {
  std::string steer_joint_name;

  std::string left_wheel_joint_name;
  if (!node.getParam("left_wheel_joint", left_wheel_joint_name)) {
    ROS_ERROR("Could not find left_wheel_joint name");
    return false;
  }

  std::string right_wheel_joint_name;
  if (!node.getParam("right_wheel_joint", right_wheel_joint_name)) {
    ROS_ERROR("Could not find right_wheel_joint name");
    return false;
  }

  if (!node.getParam("tread", tread_)) {
    ROS_ERROR("Could not find tread value");
    return false;
  }


  if (!node.getParam("wheel_radius", wheel_radius_)) {
    ROS_ERROR("Could not find wheel_radius value");
    return false;
  }

  left_wheel_joint_ = hw->getHandle(left_wheel_joint_name);
  right_wheel_joint_ = hw->getHandle(right_wheel_joint_name);

  command_sub_ = node.subscribe<geometry_msgs::Twist>(
      "command_velocity",
      1,
      &DiffdriveCommandController::CommandCallback,
      this);

  node_ = node;

  return true;
}


void DiffdriveCommandController::update(const ros::Time& time,
                                        const ros::Duration& period) {
  if (command_mutex_.try_lock()) {
    last_command_ = command_;
    command_mutex_.unlock();
  }

  const double v = tread_ / wheel_radius_ * last_command_.linear.x;
  const double w = tread_ / wheel_radius_ * last_command_.angular.z;
  const double right_vel = v / wheel_radius_ + tread_ / wheel_radius_ * w;
  const double left_vel = v / wheel_radius_ - tread_ / wheel_radius_ * w;

  right_wheel_joint_.setCommand(right_vel);
  left_wheel_joint_.setCommand(left_vel);
}


void DiffdriveCommandController::starting(const ros::Time& time) {}


void DiffdriveCommandController::stopping(const ros::Time& time) {
  left_wheel_joint_.setCommand(0.0);
  right_wheel_joint_.setCommand(0.0);
}


void DiffdriveCommandController::CommandCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  if (command_mutex_.try_lock()) {
    command_ = *msg;
    command_mutex_.unlock();
  }
}

}  // namespace tmc_realtime_controllers


PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::DiffdriveCommandController,
                       controller_interface::ControllerBase);

