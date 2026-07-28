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
#include "color_command_controller.hpp"

#include <cassert>
#include <string>
#include <vector>

#include <boost/algorithm/clamp.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_list_macros.h>

namespace {
const uint32_t kSubscriberQueueSize = 100;  //!/ Queue size
}  // anonymous namespace

namespace tmc_realtime_controllers {

/**
 * @brief Callback method for subscriber
 * @param[in] msg msg
 */
void ColorCommandController::StateSubscriber::CommandCallback(const std_msgs::ColorRGBA::ConstPtr& msg) {
  struct Command command;
  command.color = *msg;
  command.time = ros::Time::now();  // Record the subscribed time
  command_.writeFromNonRT(command);
}

/**
 * @brief Periodic update
 */
void ColorCommandController::StateSubscriber::Update(const ros::Time& time) {
  last_command_ = *command_.readFromRT();
  ROS_DEBUG_STREAM("led color"
                   << " last_command_.r = " << last_command_.color.r << " last_command_.g = " << last_command_.color.g
                   << " last_command_.b = " << last_command_.color.b);
  if (timeout_.toSec() > 0 &&                  // Timeout is set
      last_command_.time.toSec() > 0 &&        // Subscribed at least once before
      time - last_command_.time > timeout_) {  // Timeout duration has passed
    handle_.setColor(timeout_color_.r, timeout_color_.g, timeout_color_.b);
  } else {
    handle_.setColor(last_command_.color.r, last_command_.color.g, last_command_.color.b);
  }
}

/**
 * @brief Initialization
 * @param[in] hw hw
 * @param[in] root_nh root_nh
 * @param[in] controller_nh controller_nh
 * @return True on success
 */
bool ColorCommandController::init(tmc_hardware_interface::ColorCommandInterface* hw, ros::NodeHandle& root_nh,
                                  ros::NodeHandle& controller_nh) {
  std::vector<std::string> const names = hw->getNames();
  BOOST_FOREACH (std::string const name, names) {
    tmc_hardware_interface::ColorCommandHandle const handle = hw->getHandle(name);
    boost::shared_ptr<StateSubscriber> const state_subscriber =
        boost::make_shared<StateSubscriber>(handle, controller_nh);
    state_subscriber->command_sub_ = root_nh.subscribe(
        name, kSubscriberQueueSize, &ColorCommandController::StateSubscriber::CommandCallback, state_subscriber.get());
    state_subscribers_.push_back(state_subscriber);
  }

  return true;
}

/**
 * @brief Periodic update
 * @param[in] time time
 * @param[in] period period
 */
void ColorCommandController::update(const ros::Time& time, const ros::Duration& period) {
  (void)(time);    // unused
  (void)(period);  // unused

  // Update all registered handles
  BOOST_FOREACH (boost::shared_ptr<StateSubscriber> const state_subscriber, state_subscribers_) {
    state_subscriber->Update(time);
  }
}

/**
 * @brief Pre-start processing
 * @param[in] time time
 */
void ColorCommandController::starting(const ros::Time& time) {
  (void)(time);  // unused
}

/**
 * @brief Post-end processing
 * @param[in] time time
 */
void ColorCommandController::stopping(const ros::Time& time) {
  (void)(time);  // unused
}
}  // namespace tmc_realtime_controllers

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::ColorCommandController, controller_interface::ControllerBase);
