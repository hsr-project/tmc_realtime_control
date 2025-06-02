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
#ifndef TMC_REALTIME_CONTROLLERSTMC_DIAGNOSTIC_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERSTMC_DIAGNOSTIC_CONTROLLER_HPP_
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <controller_interface/controller.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <tmc_hardware_interface/diagnostic_interface.hpp>

// TODO(kitsunai): テスト内容
// Basic test consisting of two handles with one key value each, and testing publishing
// Assign types to key value (int, uint, double, string)
// Test default conversion (when buffer is exceeded, etc.)

namespace tmc_realtime_controllers {
/**
 * @brief Controller for DiagnosticInterface
 * Controller class supporting multiple DiagnosticHandles
 *
 * Consolidates KeyValue from multiple handles, and publishes topics from the publisher named "/diagnostics".
 * Publisher sends topics at 1 [Hz].
 * To change the publication cycle, modify the "publish_rate" ros param.
 */
class DiagnosticController : public controller_interface::Controller<tmc_hardware_interface::DiagnosticInterface> {
 public:
  /**
   * @brief Initialization
   * @param[in] hw hw
   * @param[in] root_nh root_nh
   * @param[in] controller_nh controller_nh
   * @return True on success
   */
  virtual bool init(tmc_hardware_interface::DiagnosticInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);
  /**
   * @brief Periodic update
   * @param[in] time time
   * @param[in] period period
   */
  virtual void update(const ros::Time& time, const ros::Duration& period);
  /**
   * @brief Pre-start processing
   * @param[in] time time
   */
  virtual void starting(const ros::Time& time);
  /**
   * @brief Processing at termination
   * @param[in] time time
   */
  virtual void stopping(const ros::Time& time);

 private:
  std::vector<tmc_hardware_interface::DiagnosticHandle> handles_;                  //!/ List of handles
  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher_;  //!/ Publisher
  ros::Time last_published_time_;                                                  //!/ Time of last publish
  double publish_rate_;                                                            //!/ Publication cycle [Hz]
  double expected_publish_time_;                                                   //!/ Publication cycle [sec]
};
}  // namespace tmc_realtime_controllers
#endif  // TMC_REALTIME_CONTROLLERSTMC_DIAGNOSTIC_CONTROLLER_HPP_
