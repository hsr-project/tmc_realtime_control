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
#include "diagnostic_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/move/move.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace {
const uint32_t kPublisherQueueSize = 100;  //!/ Queue size

/**
 * @brief String copy without allocator
 * @param[in] source Source for copy
 * @param[out] dist Destination for copy
 */
inline void StringCopyRT(const std::string& source, std::string& dist) {
  std::string::const_iterator end_it;
  if (dist.capacity() >= source.size()) {
    end_it = source.end();
  } else {
    ROS_ERROR("string buffer is not enough buffer");
    end_it = source.begin() + dist.capacity() - 1;
  }
  // std::copy std::copy(source.begin(), source.end(), dist.begin()); results in
  // After copying, the content of dist is judged as "\0"
  dist.clear();
  std::copy(source.begin(), end_it, std::back_inserter(dist));
}

}  // anonymous namespace

namespace tmc_realtime_controllers {

/**
 * @brief Initialization
 * @param[in] hw hw
 * @param[in] root_nh root_nh
 * @param[in] controller_nh controller_nh
 * @return True on success
 */
bool DiagnosticController::init(tmc_hardware_interface::DiagnosticInterface* hw, ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh) {
  int32_t message_max_length = 0;
  int32_t value_max_length = 0;
  controller_nh.getParam("message_max_length", message_max_length);
  controller_nh.getParam("value_max_length", value_max_length);
  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }
  if (publish_rate_ <= 0.0) {
    return false;
  }
  expected_publish_time_ = 1.0 / publish_rate_;

  std::vector<std::string> const names = hw->getNames();
  diagnostic_msgs::DiagnosticArray& msg = publisher_.msg_;
  BOOST_FOREACH (std::string const name, names) {
    tmc_hardware_interface::DiagnosticHandle const handle = hw->getHandle(name);
    // Pre-allocate
    diagnostic_msgs::DiagnosticStatus status;
    status.name = handle.getName();
    status.hardware_id = handle.getHardwareId();
    for (size_t i = 0; i < handle.getSize(); ++i) {
      diagnostic_msgs::KeyValue key_value;
      key_value.key = handle.getKey(i);
      status.values.push_back(key_value);
    }
    msg.status.push_back(status);
    handles_.push_back(handle);
  }
  // Buffer allocation for std::string
  BOOST_FOREACH (diagnostic_msgs::DiagnosticStatus& status, msg.status) {
    tmc_hardware_interface::DiagnosticHandle const handle = hw->getHandle(status.name);
    size_t i = 0;
    status.message.reserve(std::max(handle.getMessageMaxLength(), static_cast<size_t>(message_max_length)));
    BOOST_FOREACH (diagnostic_msgs::KeyValue& key_value, status.values) {
      key_value.value.reserve(std::max(handle.getValueMaxLength(i), static_cast<size_t>(value_max_length)));
      ++i;
    }
  }

  publisher_.init(root_nh, "/diagnostics", kPublisherQueueSize);

  return true;
}

/**
 * @brief Periodic update
 * @param[in] time time
 * @param[in] period period
 */
void DiagnosticController::update(const ros::Time& time, const ros::Duration& period) {
  (void)(time);    // unused
  (void)(period);  // unused

  if (fabs((time - last_published_time_).toSec()) >= expected_publish_time_) {
    if (publisher_.trylock()) {
      diagnostic_msgs::DiagnosticArray& msg = publisher_.msg_;
      msg.header.stamp = time;
      for (size_t i = 0; i < handles_.size(); ++i) {
        tmc_hardware_interface::DiagnosticHandle& handle = handles_[i];
        diagnostic_msgs::DiagnosticStatus& status = msg.status[i];
        // status.name, status.hardware_id, status.values[n].key
        // Assumed to be unchanged
        status.level = handle.getLevel();
        StringCopyRT(handle.getMessage(), status.message);
        ROS_ASSERT_MSG((status.values.size() == handle.getSize()), " handle.size is changed.");
        for (size_t j = 0; j < handle.getSize(); ++j) {
          diagnostic_msgs::KeyValue& key_value = status.values[j];
          StringCopyRT(handle.getValue(j), key_value.value);
        }
      }
      publisher_.unlockAndPublish();
      last_published_time_ += ros::Duration(expected_publish_time_);
    }
  }
}

/**
 * @brief Pre-start processing
 * @param[in] time time
 */
void DiagnosticController::starting(const ros::Time& time) {
  (void)(time);  // unused
  last_published_time_ = time;
}

/**
 * @brief Processing at termination
 * @param[in] time time
 */
void DiagnosticController::stopping(const ros::Time& time) {
  (void)(time);  // unused
}
}  // namespace tmc_realtime_controllers

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::DiagnosticController, controller_interface::ControllerBase);
