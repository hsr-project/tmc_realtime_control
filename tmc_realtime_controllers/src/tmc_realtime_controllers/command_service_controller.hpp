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
#ifndef TMC_REALTIME_CONTROLLERS_TMC_COMMAND_SERVICE_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERS_TMC_COMMAND_SERVICE_CONTROLLER_HPP_

#include <iostream>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>

#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <tmc_hardware_interface/command_service_handle.hpp>

namespace tmc_realtime_controllers {
/**
 * @brief Controller for CommandHandle
 * Superclass supporting multiple handles
 *
 * Creates a service with the name "[Handle Name]."
 * Updates the Handle value only once when a call is made to the service.
 *
 * Create a subclass according to the type of service to be used.
 */
template <class HardwareInterface, class ROSService>
class CommandServiceController : public controller_interface::Controller<HardwareInterface> {
 private:
  // Class constant definitions
#if (defined __GNUC__ && __GNUC__ >= 7)
  constexpr static const double kDefaultPollingRate = 100.0; /* Hz */
#else
  static const double kDefaultPollingRate = 100.0; /* Hz */
#endif
  // Message type definitions
  typedef typename HardwareInterface::ResourceHandleType ResourceHandle;
  typedef typename ResourceHandle::RequestType HandleRequest;
  typedef typename ResourceHandle::ResponseType HandleResponse;
  typedef typename ROSService::Response ROSResponse;
  typedef typename ROSService::Request ROSRequest;

  /**
   * @brief Command state transitions
   * The actors involved in state transitions are as follows:
   * - CommandService
   *   Gateway for ROSService calls, management of state transition flags
   * - Controller (calls CommandService::Update())
   *   Manages state transition flags and rewrites the handle
   * - robotHw
   *   Communication management with the actual device
   *
   * Variables used for communication between actors are shown below.
   * -------------------------------------------------
   *  CommandService <- -> Controller <- -> robotHw
   *                   A                A
   * State flag | state_            | handle_.has_request
   * Request    | request_          | handle_.request_
   * Response   | response_         | handle_.response_
   * -------------------------------------------------
   *
   * State transition rules
   * - State transitions are performed by CommandService and Controller. Mutex-based mutual exclusion is implemented.
   * - Only the Controller can operate handle_.
   * - Variables other than state_ are designed to not require mutual exclusion logically.
   *
   * The state transition diagram for commands is shown below.
   * -------------------------------------------------------------------------------
   * handle_.has_request             |____ooooooooo|oooooooooooo__|_______________|____
   * state_              kStandBy -> kRequested -> kProcessing -> kResponded|kTimeout -> kStandBy
   *                                                    |             A
   *                                                    |             |
   *                                                    +--(timeout)--+
   * -------------------------------------------------------------------------------
   *
   * Details of the behavior of each state are shown below.
   *
   * kStandBy : Service standby state
   *   State that accepts service calls.
   *   CommandService performs mutual exclusion to prevent multiple service calls simultaneously.
   * - CommandService
   *   Receives a service request and transitions to kRequested.
   * - Controller
   *   Keeps the handle_.has_request_ flag off.
   * - robotHw
   *   Standby
   *
   * kRequested : Service processing start
   *   State that has accepted a service call
   * - CommandService
   *   Polls and waits until it becomes kResponded|kTimeout
   * - Controller
   *   Turns on the handle_.has_request_ flag.
   *   Transitions the state to kProcessing.
   * - robotHw
   *   Standby
   *
   * kProcessing : robotHw processing
   *   State where RobotHW performs processing
   * - CommandService
   *   Polls and waits until it becomes kResponded|kTimeout
   * - Controller
   *   Polls and confirms until the handle.has_request_ flag drops
   *   If the flag has dropped, transitions to the kResponded state
   *   If the timeout period is exceeded, drops the handle_.has_request_ flag,
   *   and transitions to the kResponded state
   * - robotHw
   *   Performs processing.
   *   Drops the HasRequest flag upon completion
   *
   * kResponded : Sends results back to the service originator
   * - CommandService
   *   Replies with Response and Message to the service caller
   *   Transitions to kStandBy
   * - Controller
   *   Standby
   * - robotHw
   *   Standby
   *
   */
  enum CommandState {
    kStandBy,     //!/ Service standby state
    kRequested,   //!/ Service request accepted
    kProcessing,  //!/ robotHw processing
    kResponded,   //!/ robotHw processing completed, results sent to service caller
    kTimeout      //!/ robotHw timeout
  };

  /**
   * @brief Set of items required for managing a single Handle
   */
  class CommandService : boost::noncopyable {
   private:
    // Conversion between Handle and ROS message
    // Specialize this template
    /**
     * @brief Default RosMessage→Handle conversion function
     * By default, does nothing. Specialize this function to perform conversion processing.
     * @param[in] ros_req ROSRequest
     * @param[out] req HandleRequest
     */
    void SetRequest(const ROSRequest& ros_req, HandleRequest& req) {
      (void)ros_req;
      (void)req;
    }
    /**
     * @brief Default Handle→RosMessage conversion function
     * By default, does nothing. Specialize this function to perform conversion processing.
     * @param[in] res HandleResponse
     * @param[out] ros_res ROSResponse
     */
    void SetResponse(const HandleResponse& res, ROSResponse& ros_res) {
      (void)ros_res;
      (void)res;
    }
    /**
     * @brief Default RosMessage creation function for TimeOut
     * Assumes the existence of a member named message of type std::string
     * If a separate creation process is defined, specialize this function.
     * @param[out] ros_res ROSResponse
     */
    void SetTimeoutResponse(ROSResponse& ros_res) {
      (void)ros_res;
      ros_res.message = "Hardware did not respond. Timeout";
    }

   public:
    /**
     * @brief Constructor
     *
     * @param handle handle
     * @param polling_rate polling_rate
     * @param service_time_out service_time_out
     * @param name name
     * @param root_nh root_nh
     */
    CommandService(const ResourceHandle& handle, const ros::Rate& polling_rate, const ros::Duration& service_time_out,
                   const std::string& name, ros::NodeHandle& root_nh)
        : handle_(handle), polling_rate_(polling_rate), service_time_out_(service_time_out), state_(kStandBy) {
      set_bool_server_ = root_nh.advertiseService(
          name, &CommandServiceController<HardwareInterface, ROSService>::CommandService::CommandCallback, this);
    }

    /**
     * @brief Callback method when the service is called
     *        Performs mutual exclusion considering multi-spinner
     * @param[in] req req
     * @param[out] res res
     * @return True on success
     */
    bool CommandCallback(ROSRequest& ros_req, ROSResponse& ros_res) {
      // kStandBy
      // Changed to simple lock (waits until completion if there are simultaneous calls)
      // Low likelihood of simultaneous calls, and error handling for failures in simultaneous calls is not desired at a higher level
      boost::lock_guard<boost::mutex> service_lock(service_mutex_);

      start_time_ = ros::Time::now();
      SetRequest(ros_req, request_);

      //// Transition to kRequested
      {
        boost::lock_guard<boost::mutex> lock(state_mutex_);
        state_ = kRequested;
      }

      // kRequested, kProcessing
      //// Polling until kResponded state
      while (true) {
        {
          boost::lock_guard<boost::mutex> lock(state_mutex_);
          if ((state_ == kResponded) || (state_ == kTimeout)) {
            break;
          }
        }
        polling_rate_.sleep();
      }

      // kRequested
      //// Read the result
      if (state_ == kResponded) {
        SetResponse(response_, ros_res);
      } else {
        // Process message-related tasks in non-real-time
        SetTimeoutResponse(ros_res);
      }
      {
        boost::lock_guard<boost::mutex> lock(state_mutex_);
        state_ = kStandBy;
      }
      return true;
    }

    void Update() {
      // Attempt lock in real-time layer
      boost::unique_lock<boost::mutex> lock(state_mutex_, boost::try_to_lock);
      if (!lock) {
        return;
      }
      switch (state_) {
        case kStandBy:
          handle_.cancel();
          break;
        case kRequested:
          handle_.setRequest(request_);
          state_ = kProcessing;
        case kProcessing:
          if (!handle_.hasRequest()) {
            // Read the result after robotHw drops HasRequest (processing is complete)
            handle_.getResponse(response_);
            // Drop the request flag
            handle_.cancel();
            // Transition to kResponded state
            state_ = kResponded;
          } else if ((ros::Time::now() - start_time_) > service_time_out_) {
            // Timeout
            handle_.cancel();  // Send processing cancellation
            // Transition to kTimeout state
            state_ = kTimeout;
          }
          break;
        case kResponded:
        case kTimeout:
          break;
      }
    }

   private:
    ros::ServiceServer set_bool_server_;    //!/ Service for receiving reset commands
    ResourceHandle handle_;                 //!/ Handle
    ros::Rate polling_rate_;                //!/ Polling rate
    const ros::Duration service_time_out_;  //!/ Service timeout duration [ms]
    CommandState state_;                    //!/ Service state
    boost::mutex service_mutex_;            //!/ Mutex to prevent simultaneous activation of CommandCallback
    boost::mutex state_mutex_;              //!/ Mutex for state_
    ros::Time start_time_;                  //!/ Service processing start time
    HandleRequest request_;                 //!/ Request
    HandleResponse response_;               //!/ Response
  };

 public:
  CommandServiceController() {}
  virtual ~CommandServiceController() {}

  /**
   * @brief Initialization
   * @param[in] hw hw
   * @param[in] root_nh root_nh
   * @param[in] controller_nh controller_nh
   * @return True on success
   */
  virtual bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    double time_out_ms;
    if (!controller_nh.getParam("service_time_out", time_out_ms)) {
      ROS_ERROR("Parameter 'service_time_out' is not set.");
      return false;
    } else if (time_out_ms <= 0) {
      ROS_ERROR("Parameter 'service_time_out' is 0 or less.");
      return false;
    }
    double polling_rate;
    if (!controller_nh.getParam("service_polling_rate", polling_rate)) {
      ROS_WARN("Parameter 'service_polling_rate' is not set. Use default value: %f", kDefaultPollingRate);
      polling_rate = kDefaultPollingRate;
    } else if (polling_rate <= 0) {
      ROS_ERROR("Parameter 'service_polling_rate' is 0 or less.");
      return false;
    }
    std::vector<std::string> const names = hw->getNames();
    // Create CommandService for each managed Handle
    BOOST_FOREACH (const std::string& name, names) {
      ResourceHandle const handle = hw->getHandle(name);
      boost::shared_ptr<CommandService> const command_service = boost::shared_ptr<CommandService>(
          new CommandService(handle, ros::Rate(polling_rate), ros::Duration(time_out_ms / 1000.0), name, root_nh));
      command_services_.push_back(command_service);
    }
    return true;
  }
  /**
   * @brief Periodic update
   * @param[in] time time
   * @param[in] period period
   */
  virtual void update(const ros::Time& time, const ros::Duration& period) {
    (void)(time);    // unused
    (void)(period);  // unused
    BOOST_FOREACH (boost::shared_ptr<CommandService> p, command_services_) { p->Update(); }
  }
  /**
   * @brief Pre-start processing
   * @param[in] time time
   */
  virtual void starting(const ros::Time& time) {
    (void)(time);  // unused
  }
  /**
   * @brief Post-completion processing
   * @param[in] time time
   */
  virtual void stopping(const ros::Time& time) {
    (void)(time);  // unused
  }

 private:
  std::vector<boost::shared_ptr<CommandService> > command_services_;  //!/ List of managed items
};
}  // namespace tmc_realtime_controllers
#endif  // TMC_REALTIME_CONTROLLERS_TMC_COMMAND_SERVICE_CONTROLLER_HPP_
