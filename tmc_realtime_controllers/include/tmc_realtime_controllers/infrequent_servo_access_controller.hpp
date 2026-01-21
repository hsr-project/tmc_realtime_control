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
/// @brief Controller providing a service to read and write parameters
#ifndef TMC_REALTIME_CONTROLLERS_INFREQUENT_SERVO_ACCESS_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERS_INFREQUENT_SERVO_ACCESS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <boost/thread/mutex.hpp>
#include <controller_interface/controller_interface.hpp>
#include <tmc_control_msgs/msg/servo_param.hpp>
#include <tmc_control_msgs/srv/read_parameters.hpp>
#include <tmc_control_msgs/srv/write_parameters.hpp>
#include <tmc_exxx_servo_motor_protocol/control_table.hpp>

namespace tmc_realtime_controllers {

// Controller providing a service to read and write parameters
template <typename Type, typename SrvReq, typename SrvRes>
class InfrequentServoAccessController : public controller_interface::ControllerInterface {
 public:
  // Constructor
  InfrequentServoAccessController() {}
  // Destructor
  ~InfrequentServoAccessController() {}

  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Initialization of parts other than ControllerInterface::init, split for testing
  bool InitImpl();

 protected:
  // Receive results
  virtual void GetResult(uint32_t joint_index) {}

  // Get the number of read/write commands
  virtual uint32_t GetCommandSize(const SrvReq& request) const { return 0; }
  // Write commands to buffer
  virtual void WriteToBuffer(uint32_t key_index, const SrvReq& request) {}
  // Read results from buffer
  virtual void ReadFromBuffer(uint32_t key_index, SrvRes& response) {}
  // Send commands
  virtual bool SetRequest(uint32_t joint_index) { return true; }
  // Callback for the service to read and write parameters
  void ServiceCallBack(const std::shared_ptr<SrvReq> request, const std::shared_ptr<SrvRes> response);
  // Get the index of the common part.
  void CommonGetIndex(std::string joint_name);
  // Get the command interface of the common part.
  void CommonCommandInterfaceConfiguration(
    std::string joint_name, controller_interface::InterfaceConfiguration& conf) const;
  // Get the state interface of the common part.
  void CommonStateInterfaceConfiguration(
    std::string joint_name, controller_interface::InterfaceConfiguration& conf) const;

  // Service to read and write parameters
  typename rclcpp::Service<Type>::SharedPtr parameter_srv_;
  // For storing control table information
  tmc_exxx_servo_motor_protocol::ControlTable control_table_;
  // Node for service
  rclcpp::Node::SharedPtr srv_node_;

  // Controller attribute type
  std::string attribute_;
  // Joint names
  std::vector<std::string> joint_names_;
  // Keys to deny access
  std::vector<std::string> denied_keys_;
  // Controller name
  std::string controller_name_;

  // Index definition
  std::vector<std::optional<uint32_t>> command_index_;
  std::vector<std::optional<uint32_t>> command_has_command_index_;
  std::vector<std::optional<uint32_t>> command_trial_num_index_;
  std::vector<std::optional<uint32_t>> state_has_command_index_;
  std::vector<std::optional<uint32_t>> state_trial_num_index_;
  std::vector<std::optional<uint32_t>> state_avagopos_index_;
  std::vector<std::optional<uint32_t>> state_is_success_index_;


  // Content of the request
  realtime_tools::RealtimeBuffer<uint32_t> request_joint_index_;
  realtime_tools::RealtimeBuffer<std::string> request_key_;
  realtime_tools::RealtimeBuffer<double> request_value_;
  realtime_tools::RealtimeBuffer<bool> request_is_success_;

  // State of the request
  enum RequestState {
    kNoRequest,
    kRequestSend,
    kRequestDone,
  } request_state_;
  // For changing the state of the request
  boost::mutex request_lock_;

  // Check if the state of the request is the target state
  bool CheckRequestStateFromNonRT(RequestState target);
  // Check if the state of the request is the target state
  bool CheckRequestStateFromRT(RequestState target);
  // Rewrite the state of the request
  void UpdateRequestStateFromNonRT(RequestState target);
  // Rewrite the state of the request
  bool UpdateRequestStateFromRT(RequestState target);
};

// Controller to read parameters
class InfrequentReadingController
    : public InfrequentServoAccessController<tmc_control_msgs::srv::ReadParameters,
                                             tmc_control_msgs::srv::ReadParameters::Request,
                                             tmc_control_msgs::srv::ReadParameters::Response> {
 public:
  // Constructor
  InfrequentReadingController() {}
  // Destructor
  ~InfrequentReadingController() {}

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::vector<std::optional<uint32_t>> state_read_value_index_;

  // Receive results
  virtual void GetResult(uint32_t joint_index);
  // Get the number of read/write commands
  virtual uint32_t GetCommandSize(const tmc_control_msgs::srv::ReadParameters::Request& request) const;
  // Write commands to buffer
  virtual void WriteToBuffer(uint32_t key_index, const tmc_control_msgs::srv::ReadParameters::Request& request);
  // Read results from buffer
  virtual void ReadFromBuffer(uint32_t key_index, tmc_control_msgs::srv::ReadParameters::Response& response);
  // Send commands
  virtual bool SetRequest(uint32_t joint_index);
};

// Write parameters
class InfrequentWritingController
    : public InfrequentServoAccessController<tmc_control_msgs::srv::WriteParameters,
                                             tmc_control_msgs::srv::WriteParameters::Request,
                                             tmc_control_msgs::srv::WriteParameters::Response> {
 public:
  // Constructor
  InfrequentWritingController() {}
  // Destructor
  ~InfrequentWritingController() {}

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::vector<std::optional<uint32_t>> command_write_valude_index_;

  // Receive results
  virtual void GetResult(uint32_t joint_index) {}
  // Get the number of read/write commands
  virtual uint32_t GetCommandSize(const tmc_control_msgs::srv::WriteParameters::Request& request) const;
  // Write commands to buffer
  virtual void WriteToBuffer(uint32_t key_index, const tmc_control_msgs::srv::WriteParameters::Request& request);
  // Read results from buffer
  virtual void ReadFromBuffer(uint32_t key_index, tmc_control_msgs::srv::WriteParameters::Response& response) {}
  // Send commands
  virtual bool SetRequest(uint32_t joint_index);
};

}  // namespace tmc_realtime_controllers
#endif  // TMC_REALTIME_CONTROLLERS_INFREQUENT_SERVO_ACCESS_CONTROLLER_HPP_
