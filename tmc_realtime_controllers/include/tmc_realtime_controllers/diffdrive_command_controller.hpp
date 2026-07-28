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
#ifndef TMC_REALTIME_CONTROLLERS_DIFFDRIVE_COMMAND_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERS_DIFFDRIVE_COMMAND_CONTROLLER_HPP_

#include <cassert>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>

#include <geometry_msgs/Twist.h>


namespace tmc_realtime_controllers {


class DiffdriveCommandController
    : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
 public:
  virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& node);

  virtual void update(const ros::Time& time, const ros::Duration& period);

  virtual void starting(const ros::Time& time);

  virtual void stopping(const ros::Time& time);

  hardware_interface::JointHandle& left_wheel_joint() {
    return left_wheel_joint_;
  }
  hardware_interface::JointHandle& right_wheel_joint() {
    return right_wheel_joint_;
  }
 private:
  void CommandCallback(const geometry_msgs::Twist::ConstPtr& msg);

  hardware_interface::JointHandle left_wheel_joint_;
  hardware_interface::JointHandle right_wheel_joint_;

  double tread_;
  double wheel_radius_;

  ros::Subscriber command_sub_;

  boost::mutex command_mutex_;
  geometry_msgs::Twist command_;
  geometry_msgs::Twist last_command_;

  ros::NodeHandle node_;
};

}  // namespace tmc_realtime_controllers

#endif/*TMC_REALTIME_CONTROLLERS_DIFFDRIVE_COMMAND_CONTROLLER_HPP_*/
