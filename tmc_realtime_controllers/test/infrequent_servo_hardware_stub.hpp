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
/// @brief Handle that simulates hardware for testing

#include <memory>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tmc_realtime_controllers/infrequent_servo_access_controller.hpp>


namespace tmc_realtime_controllers {
const char* const kReadControllerNodeName = "reader_controller_manager";
const char* const kWriteControllerNodeName = "writer_controller_manager";

class Handle {
 public:
  using Ptr = std::shared_ptr<Handle>;

  Handle(const std::string& joint_name, const std::string& state_name, const std::string& command_name)
      : current_(0.0), command_(0.0) {
    state_handle_ = std::make_shared<hardware_interface::StateInterface>(joint_name, state_name, &current_);
    command_handle_ = std::make_shared<hardware_interface::CommandInterface>(joint_name, command_name, &command_);
  }

  Handle(const std::string& joint_name, const std::string& interface_name)
      : Handle(joint_name, interface_name, interface_name) {}

  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface GetStateInterface() {
    return hardware_interface::LoanedStateInterface(state_handle_, nullptr);
  }
  hardware_interface::LoanedCommandInterface GetCommandInterface() {
    return hardware_interface::LoanedCommandInterface(command_handle_, nullptr);
  }

  double command() const { return command_; }
  void set_current(double x) { current_ = x; }
  double get_current() { return current_; }

 private:
  double current_;
  double command_;

  hardware_interface::StateInterface::SharedPtr state_handle_;
  hardware_interface::CommandInterface::SharedPtr command_handle_;
};

struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;

  std::vector<Handle::Ptr> reader_command_index;
  std::vector<Handle::Ptr> reader_value;
  std::vector<Handle::Ptr> reader_has_command;
  std::vector<Handle::Ptr> reader_trial_num;
  std::vector<Handle::Ptr> reader_is_success;

  std::vector<Handle::Ptr> writer_command_index;
  std::vector<Handle::Ptr> writer_value;
  std::vector<Handle::Ptr> writer_has_command;
  std::vector<Handle::Ptr> writer_trial_num;
  std::vector<Handle::Ptr> writer_is_success;

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;

  explicit HardwareStub(std::vector<std::string>& joint_names) {
    BOOST_FOREACH (std::string joint_name, joint_names) {
      reader_command_index.push_back(
          std::make_shared<Handle>(joint_name, "reader/command_index", "reader/command_index"));
      writer_command_index.push_back(
          std::make_shared<Handle>(joint_name, "writer/command_index", "writer/command_index"));

      reader_value.push_back(std::make_shared<Handle>(joint_name, "reader/value", "reader/value"));
      writer_value.push_back(std::make_shared<Handle>(joint_name, "writer/value", "writer/value"));

      reader_has_command.push_back(std::make_shared<Handle>(joint_name, "reader/has_command", "reader/has_command"));
      writer_has_command.push_back(std::make_shared<Handle>(joint_name, "writer/has_command", "writer/has_command"));

      reader_trial_num.push_back(std::make_shared<Handle>(joint_name, "reader/trial_num", "reader/trial_num"));
      writer_trial_num.push_back(std::make_shared<Handle>(joint_name, "writer/trial_num", "writer/trial_num"));

      reader_is_success.push_back(std::make_shared<Handle>(joint_name, "reader/is_success", "reader/is_success"));
      writer_is_success.push_back(std::make_shared<Handle>(joint_name, "writer/is_success", "writer/is_success"));
    }
    for (int i = 0; i < joint_names.size(); i++) {
      // Push command part
      command_interfaces.emplace_back(reader_command_index[i]->GetCommandInterface());
      command_interfaces.emplace_back(reader_value[i]->GetCommandInterface());
      command_interfaces.emplace_back(reader_has_command[i]->GetCommandInterface());
      command_interfaces.emplace_back(reader_trial_num[i]->GetCommandInterface());
      command_interfaces.emplace_back(reader_is_success[i]->GetCommandInterface());

      command_interfaces.emplace_back(writer_command_index[i]->GetCommandInterface());
      command_interfaces.emplace_back(writer_value[i]->GetCommandInterface());
      command_interfaces.emplace_back(writer_has_command[i]->GetCommandInterface());
      command_interfaces.emplace_back(writer_trial_num[i]->GetCommandInterface());
      command_interfaces.emplace_back(writer_is_success[i]->GetCommandInterface());

      // Push state part
      state_interfaces.emplace_back(reader_command_index[i]->GetStateInterface());
      state_interfaces.emplace_back(reader_value[i]->GetStateInterface());
      state_interfaces.emplace_back(reader_has_command[i]->GetStateInterface());
      state_interfaces.emplace_back(reader_trial_num[i]->GetStateInterface());
      state_interfaces.emplace_back(reader_is_success[i]->GetStateInterface());

      state_interfaces.emplace_back(writer_command_index[i]->GetStateInterface());
      state_interfaces.emplace_back(writer_value[i]->GetStateInterface());
      state_interfaces.emplace_back(writer_has_command[i]->GetStateInterface());
      state_interfaces.emplace_back(writer_trial_num[i]->GetStateInterface());
      state_interfaces.emplace_back(writer_is_success[i]->GetStateInterface());
    }
  }
};

// Test controller for writing
class TestableInfrequentWritingController : public InfrequentWritingController {
 public:
  using Ptr = std::shared_ptr<TestableInfrequentWritingController>;

  TestableInfrequentWritingController();
  ~TestableInfrequentWritingController() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  void SkipConfigure();
};

TestableInfrequentWritingController::TestableInfrequentWritingController() {
  rclcpp::NodeOptions node_options;
  EXPECT_EQ(ControllerInterface::init(kWriteControllerNodeName, "", 100, "", node_options),
            controller_interface::return_type::OK);
}

controller_interface::CallbackReturn TestableInfrequentWritingController::on_init() {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TestableInfrequentWritingController::SkipConfigure() {}

// Test controller for reading
class TestableInfrequentReadingController : public InfrequentReadingController {
 public:
  using Ptr = std::shared_ptr<InfrequentReadingController>;

  TestableInfrequentReadingController();
  ~TestableInfrequentReadingController() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  void SkipConfigure();
};

TestableInfrequentReadingController::TestableInfrequentReadingController() {
  rclcpp::NodeOptions node_options;
  EXPECT_EQ(ControllerInterface::init(kReadControllerNodeName, "", 100, "", node_options),
            controller_interface::return_type::OK);
}

controller_interface::CallbackReturn TestableInfrequentReadingController::on_init() {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TestableInfrequentReadingController::SkipConfigure() {}


}  // namespace tmc_realtime_controllers
