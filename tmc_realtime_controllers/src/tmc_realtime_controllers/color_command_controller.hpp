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
// Copyright (C) 2016 Toyeta Motor Corporation. All rights reserved.
#ifndef TMC_REALTIME_CONTROLLERSTMC_COLOR_COMMAND_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERSTMC_COLOR_COMMAND_CONTROLLER_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/ColorRGBA.h>

#include <tmc_hardware_interface/color_command_interface.hpp>

namespace tmc_realtime_controllers {
/**
 * @brief Controller for ColorCommand
 * Controller class supporting multiple ColorCommandHandles
 *
 * Manages subscribers with the same name as the handle name.
 * Each time the update method is called, the handle value is overwritten with the value of the last received topic.
 */
class ColorCommandController : public controller_interface::Controller<tmc_hardware_interface::ColorCommandInterface> {
 private:
  /**
   * @brief A complete set of items required for managing a single Handle
   */
  class StateSubscriber {
   private:
    StateSubscriber(StateSubscriber const&);             // = delete;
    StateSubscriber& operator=(StateSubscriber const&);  // = delete;

    /**
     * @brief Structure to store commands
     *
     * Made it a structure to retain the last received time
     */
    struct Command {
      std_msgs::ColorRGBA color;
      ros::Time time;
    };

   public:
    /**
     * @brief Constructor
     * @param handle handle
     * @param controller_nh NodeHandle for parameter loading
     */
    StateSubscriber(const tmc_hardware_interface::ColorCommandHandle& handle, const ros::NodeHandle& controller_nh)
        : handle_(handle) {
      ros::NodeHandle param_nh(controller_nh, handle.getName());
      double timeout = 0.0;
      std::vector<double> timeout_color;
      param_nh.getParam("timeout", timeout);
      param_nh.getParam("timeout_color", timeout_color);
      timeout_ = ros::Duration(timeout);
      if (timeout_color.size() > 0) {
        if (timeout_color.size() != 3) {
          ROS_WARN("The size of timeout_color array for %s is wrong: %lu",
                   handle.getName().c_str(), timeout_color.size());
        } else {
          timeout_color_.r = timeout_color[0];
          timeout_color_.g = timeout_color[1];
          timeout_color_.b = timeout_color[2];
        }
      }
    }

    /**
     * @brief Callback method for subscriber
     * @param[in] msg msg
     */
    void CommandCallback(const std_msgs::ColorRGBA::ConstPtr& msg);
    /**
     * @brief Periodic update
     * @param[in] time Call time
     */
    void Update(const ros::Time& time);

    tmc_hardware_interface::ColorCommandHandle handle_;            //!/ Target Handle
    realtime_tools::RealtimeBuffer<Command> command_;              //!/ Storage for received commands
    Command last_command_;                                         //!/ Storage for the last received command
    ros::Subscriber command_sub_;                                  //!/ Subscriber
    ros::Duration timeout_;                                        //!/ Timeout
    std_msgs::ColorRGBA timeout_color_;                            //!/ Color during timeout
  };

 public:
  /**
   * @brief Initialization
   * @param[in] hw hw
   * @param[in] root_nh root_nh
   * @param[in] controller_nh controller_nh
   * @return True on success
   */
  virtual bool init(tmc_hardware_interface::ColorCommandInterface* hw, ros::NodeHandle& root_nh,
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
  std::vector<boost::shared_ptr<ColorCommandController::StateSubscriber> > state_subscribers_;  //!/ Managed list
};
}  // namespace tmc_realtime_controllers

#endif  // TMC_REALTIME_CONTROLLERSTMC_COLOR_COMMAND_CONTROLLER_HPP_
