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
/// @brief Controller for reading and writing parameters

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <tmc_realtime_controllers/infrequent_servo_access_controller.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>

namespace {
// Upper limit of read/write retry count
const uint16_t kMaxRetry = 3;
// Read/write completion check wait time [sec]
const double kDriveModeTick = 0.01;
// Read/write timeout [sec]
const double kRequestTimeout = 3.0;
// Hardware controller wait time [msec]
const int kWaitForController = 3000;
// Get key
std::string GetKey(uint32_t key_index, const std::shared_ptr<tmc_control_msgs::srv::ReadParameters::Request> request) {
  return request->keys[key_index];
}
// Get key
std::string GetKey(uint32_t key_index, const std::shared_ptr<tmc_control_msgs::srv::WriteParameters::Request> request) {
  return request->values[key_index].key;
}

}  // namespace

namespace tmc_realtime_controllers {

template <typename Type, typename SrvReq, typename SrvRes>
controller_interface::return_type InfrequentServoAccessController<Type, SrvReq, SrvRes>::init(
    const std::string& controller_name, const std::string& namespace_, const rclcpp::NodeOptions& node_options) {
  const auto ret = ControllerInterface::init(controller_name, namespace_, node_options);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  controller_name_ = controller_name;
  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

template <typename Type, typename SrvReq, typename SrvRes>
bool InfrequentServoAccessController<Type, SrvReq, SrvRes>::InitImpl() {
  joint_names_ = GetParameter(get_node(), "joints", std::vector<std::string>());

  denied_keys_ = GetParameter(get_node(), "denied_keys", std::vector<std::string>());

  attribute_ = GetParameter(get_node(), "attribute", std::string(""));

  if ((attribute_ != "reader") && (attribute_ != "writer")) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "attribute must be set to read or write.");
    return false;
  }

  std::string controller_name = GetParameter<std::string>(get_node(), "hardware_controller_name", "hsrb_hw");

  const auto parameter = std::make_shared<rclcpp::SyncParametersClient>(get_node(), controller_name);
  if (!parameter->wait_for_service(std::chrono::milliseconds(kWaitForController))) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                       "Service for getting control table path is not available: " << controller_name);
    return false;
  }
  const auto parameters_get_results = parameter->get_parameters({ "control_table_path" }).at(0);
  std::string control_path = parameters_get_results.as_string();

  if (control_table_.Load(control_path) != tmc_exxx_servo_motor_protocol::ControlTable::kSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "control table read error.");
    return false;
  }
  request_joint_index_.initRT(joint_names_.size());

  // Initialize other variables
  request_state_ = kNoRequest;

  return true;
}

template <typename Type, typename SrvReq, typename SrvRes>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InfrequentServoAccessController<Type, SrvReq, SrvRes>::on_init() {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template <typename Type, typename SrvReq, typename SrvRes>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InfrequentServoAccessController<Type, SrvReq, SrvRes>::on_configure(const rclcpp_lifecycle::State& previous_state) {
  parameter_srv_ = get_node()->create_service<Type>(
      attribute_ + "/access",
      std::bind(&InfrequentServoAccessController::ServiceCallBack, this, std::placeholders::_1, std::placeholders::_2));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template <typename Type, typename SrvReq, typename SrvRes>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InfrequentServoAccessController<Type, SrvReq, SrvRes>::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Update controller
template <typename Type, typename SrvReq, typename SrvRes>
controller_interface::return_type InfrequentServoAccessController<Type, SrvReq, SrvRes>::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  uint32_t joint_index = *request_joint_index_.readFromRT();

  // Do nothing if no command is received.
  if (joint_index >= joint_names_.size()) {
    return controller_interface::return_type::OK;
  }

  if ((!state_is_success_index_[joint_index].has_value()) ||
      (!state_has_command_index_[joint_index].has_value()) ||
      (!state_trial_num_index_[joint_index].has_value())) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint index was not found. Joint : " << joint_index);
    return controller_interface::return_type::ERROR;
  }

  uint32_t is_success =
    static_cast<uint32_t>(state_interfaces_[state_is_success_index_[joint_index].value()].get_value());
  uint32_t has_command =
    static_cast<uint32_t>(state_interfaces_[state_has_command_index_[joint_index].value()].get_value());
  uint32_t trial_num =
    static_cast<uint32_t>(state_interfaces_[state_trial_num_index_[joint_index].value()].get_value());

  if (CheckRequestStateFromRT(kRequestSend) && ((has_command == 0) || (trial_num >= kMaxRetry))) {
    // Success if the command is consumed
    if ((has_command == 0) && (is_success == 1)) {
      GetResult(joint_index);
      request_is_success_.writeFromNonRT(true);
    } else {
      request_is_success_.writeFromNonRT(false);
    }
    // Since request_is_success_ is read after Done on the service side
    // Need to transition to Done state later
    UpdateRequestStateFromRT(kRequestDone);
  }

  // Send request if not empty
  if (CheckRequestStateFromRT(kNoRequest) && (joint_index < joint_names_.size())) {
    // Set command information.
    if (SetRequest(joint_index)) {
      UpdateRequestStateFromRT(kRequestSend);
    }
  }

  return controller_interface::return_type::OK;
}
// Callback for service to read and write parameters
template <typename Type, typename SrvReq, typename SrvRes>
void InfrequentServoAccessController<Type, SrvReq, SrvRes>::ServiceCallBack(const std::shared_ptr<SrvReq> request,
                                                                            const std::shared_ptr<SrvRes> response) {
  // Search for target joint
  response->success = false;
  std::vector<std::string>::iterator joint_it(std::find(joint_names_.begin(), joint_names_.end(), request->name));
  if (joint_it == joint_names_.end()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), request->name << " is not in this controller.");
    return;
  }
  uint32_t joint_index(std::distance(joint_names_.begin(), joint_it));

  // Check if key contains access denial
  uint32_t command_size = GetCommandSize(*request);
  for (uint32_t key_index = 0; key_index < command_size; ++key_index) {
    if (std::binary_search(denied_keys_.begin(), denied_keys_.end(), GetKey(key_index, request))) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), GetKey(key_index, request) << " is in denied keys.");
      return;
    }
  }

  // Read and write sequentially
  bool is_command_not_found = false;
  std::unique_ptr<rclcpp::Time> start = std::make_unique<rclcpp::Time>(get_node()->get_clock()->now());
  for (uint32_t key_index = 0; key_index < command_size; ++key_index) {
    int command_index = control_table_.GetCommandIndex(GetKey(key_index, request));
    if (command_index == -1) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), GetKey(key_index, request)
                                                            << " is not found in control table.");
      is_command_not_found = true;
      continue;
    }

    // Pack command into buffer
    WriteToBuffer(key_index, *request);
    request_joint_index_.writeFromNonRT(joint_index);
    // Wait for read/write to finish
    bool is_not_done = true;
    while (is_not_done) {
      std::unique_ptr<rclcpp::Time> now = std::make_unique<rclcpp::Time>(get_node()->get_clock()->now());
      if ((now->seconds() - start->seconds()) > kRequestTimeout) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "servo parameter service is timed out.");
        request_joint_index_.writeFromNonRT(joint_names_.size());
        UpdateRequestStateFromNonRT(kNoRequest);
        return;
      }
      is_not_done = !CheckRequestStateFromNonRT(kRequestDone);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Check if read/write completed successfully
    request_joint_index_.writeFromNonRT(joint_names_.size());
    if (*request_is_success_.readFromNonRT()) {
      ReadFromBuffer(key_index, *response);
    } else {
      UpdateRequestStateFromNonRT(kNoRequest);
      return;
    }
    UpdateRequestStateFromNonRT(kNoRequest);
  }

  // Success in all read/write
  if (is_command_not_found) {
    response->success = false;
  } else {
    response->success = true;
  }
  return;
}

template <typename Type, typename SrvReq, typename SrvRes>
void InfrequentServoAccessController<Type, SrvReq, SrvRes>::CommonCommandInterfaceConfiguration(
  std::string joint_name, controller_interface::InterfaceConfiguration& conf) const {
  conf.names.push_back(joint_name + "/" + attribute_ + "/command_index");
  conf.names.push_back(joint_name + "/" + attribute_ + "/has_command");
  conf.names.push_back(joint_name + "/" + attribute_ + "/trial_num");
}

template <typename Type, typename SrvReq, typename SrvRes>
void InfrequentServoAccessController<Type, SrvReq, SrvRes>::CommonStateInterfaceConfiguration(
  std::string joint_name, controller_interface::InterfaceConfiguration& conf) const {
  conf.names.push_back(joint_name + "/" + attribute_ + "/command_index");
  conf.names.push_back(joint_name + "/" + attribute_ + "/has_command");
  conf.names.push_back(joint_name + "/" + attribute_ + "/trial_num");
  conf.names.push_back(joint_name + "/" + attribute_ + "/is_success");
}

template <typename Type, typename SrvReq, typename SrvRes>
void InfrequentServoAccessController<Type, SrvReq, SrvRes>::CommonGetIndex(std::string joint_name) {
  command_index_.push_back(GetIndex(command_interfaces_, joint_name, attribute_ + "/command_index"));
  command_has_command_index_.push_back(GetIndex(command_interfaces_, joint_name, attribute_ + "/has_command"));
  command_trial_num_index_.push_back(GetIndex(command_interfaces_, joint_name, attribute_ + "/trial_num"));
  state_has_command_index_.push_back(GetIndex(state_interfaces_, joint_name, attribute_ + "/has_command"));
  state_trial_num_index_.push_back(GetIndex(state_interfaces_, joint_name, attribute_ + "/trial_num"));
  state_is_success_index_.push_back(GetIndex(state_interfaces_, joint_name, attribute_ + "/is_success"));
  return;
}

// Check if request state is target state
template <typename Type, typename SrvReq, typename SrvRes>
bool InfrequentServoAccessController<Type, SrvReq, SrvRes>::CheckRequestStateFromNonRT(RequestState target) {
  boost::mutex::scoped_lock lock(request_lock_);
  return (request_state_ == target);
}

// Check if request state is target state
template <typename Type, typename SrvReq, typename SrvRes>
bool InfrequentServoAccessController<Type, SrvReq, SrvRes>::CheckRequestStateFromRT(RequestState target) {
  boost::mutex::scoped_lock lock(request_lock_, boost::try_to_lock);
  return lock && (request_state_ == target);
}

// Rewrite request state
template <typename Type, typename SrvReq, typename SrvRes>
void InfrequentServoAccessController<Type, SrvReq, SrvRes>::UpdateRequestStateFromNonRT(RequestState target) {
  boost::mutex::scoped_lock lock(request_lock_);
  request_state_ = target;
}

// Rewrite request state
template <typename Type, typename SrvReq, typename SrvRes>
bool InfrequentServoAccessController<Type, SrvReq, SrvRes>::UpdateRequestStateFromRT(RequestState target) {
  boost::mutex::scoped_lock lock(request_lock_, boost::try_to_lock);
  if (lock) {
    request_state_ = target;
    return true;
  } else {
    return false;
  }
}

// Receive result
void InfrequentReadingController::GetResult(uint32_t joint_index) {
  if (!state_read_value_index_[joint_index].has_value()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint index was not found. Joint : " << joint_index);
    return;
  }
  double value = state_interfaces_[state_read_value_index_[joint_index].value()].get_value();
  request_value_.writeFromNonRT(value);
}
// Get number of read/write commands
uint32_t InfrequentReadingController::GetCommandSize(
    const tmc_control_msgs::srv::ReadParameters::Request& request) const {
  return request.keys.size();
}
// Write command to buffer
void InfrequentReadingController::WriteToBuffer(uint32_t key_index,
                                                const tmc_control_msgs::srv::ReadParameters::Request& request) {
  request_key_.writeFromNonRT(request.keys[key_index]);
}
// Read result from buffer
void InfrequentReadingController::ReadFromBuffer(uint32_t key_index,
                                                 tmc_control_msgs::srv::ReadParameters::Response& response) {
  tmc_control_msgs::msg::ServoParam value;
  value.key = *request_key_.readFromNonRT();
  value.value = *request_value_.readFromNonRT();

  response.values.push_back(value);
}

// Send command
bool InfrequentReadingController::SetRequest(uint32_t joint_index) {
  double command_index_value = static_cast<double>(control_table_.GetCommandIndex(*request_key_.readFromRT()));

  if ((!command_index_[joint_index].has_value()) ||
      (!command_has_command_index_[joint_index].has_value()) ||
      (!command_trial_num_index_[joint_index].has_value())) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint index was not found. Joint : " << joint_index);
    return false;
  }

  command_interfaces_[command_index_[joint_index].value()].set_value(command_index_value);
  command_interfaces_[command_has_command_index_[joint_index].value()].set_value(1.0);
  command_interfaces_[command_trial_num_index_[joint_index].value()].set_value(0.0);

  return true;
}

// Get command interface.
controller_interface::InterfaceConfiguration
InfrequentReadingController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::string joint_name : joint_names_) {
    CommonCommandInterfaceConfiguration(joint_name, conf);
  }

  return conf;
}

// Get state interface.
controller_interface::InterfaceConfiguration
InfrequentReadingController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto joint_name : joint_names_) {
    CommonStateInterfaceConfiguration(joint_name, conf);
    conf.names.push_back(joint_name + "/" + attribute_ + "/value");
  }

  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InfrequentReadingController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  for (auto joint_name : joint_names_) {
    CommonGetIndex(joint_name);
    state_read_value_index_.push_back(GetIndex(state_interfaces_, joint_name, attribute_ + "/value"));
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// Get number of read/write commands
uint32_t InfrequentWritingController::GetCommandSize(
    const tmc_control_msgs::srv::WriteParameters::Request& request) const {
  return request.values.size();
}
// Write command to buffer
void InfrequentWritingController::WriteToBuffer(uint32_t key_index,
                                                const tmc_control_msgs::srv::WriteParameters::Request& request) {
  request_key_.writeFromNonRT(request.values[key_index].key);
  request_value_.writeFromNonRT(request.values[key_index].value);
}

// Send command
bool InfrequentWritingController::SetRequest(uint32_t joint_index) {
  double command_index_value = static_cast<double>(control_table_.GetCommandIndex(*request_key_.readFromRT()));
  double command_value = *request_value_.readFromRT();

  if ((!command_index_[joint_index].has_value()) ||
      (!command_write_valude_index_[joint_index].has_value()) ||
      (!command_has_command_index_[joint_index].has_value()) ||
      (!command_trial_num_index_[joint_index].has_value())) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), " Joint index was not found. Joint : " << joint_index);
    return false;
  }

  command_interfaces_[command_index_[joint_index].value()].set_value(command_index_value);
  command_interfaces_[command_write_valude_index_[joint_index].value()].set_value(command_value);
  command_interfaces_[command_has_command_index_[joint_index].value()].set_value(1.0);
  command_interfaces_[command_trial_num_index_[joint_index].value()].set_value(0.0);

  return true;
}

// Get command interface.
controller_interface::InterfaceConfiguration
InfrequentWritingController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto joint_name : joint_names_) {
    CommonCommandInterfaceConfiguration(joint_name, conf);
    conf.names.push_back(joint_name + "/" + attribute_ + "/value");
  }

  return conf;
}

// Get state interface.
controller_interface::InterfaceConfiguration
InfrequentWritingController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto joint_name : joint_names_) {
    CommonStateInterfaceConfiguration(joint_name, conf);
  }

  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InfrequentWritingController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  for (auto joint_name : joint_names_) {
    CommonGetIndex(joint_name);
    command_write_valude_index_.push_back(GetIndex(command_interfaces_, joint_name, attribute_ + "/value"));
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace tmc_realtime_controllers

#include "pluginlib/class_list_macros.hpp"

// Declare controller as plugin
PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::InfrequentReadingController,
                       controller_interface::ControllerInterface);

PLUGINLIB_EXPORT_CLASS(tmc_realtime_controllers::InfrequentWritingController,
                       controller_interface::ControllerInterface);
