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
/// @brief テストのためのハードウェアを模擬するHandle
#include <memory>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace tmc_realtime_controllers {

const char* const kControllerNodeName = "controller_manager";

class Handle {
 public:
  using Ptr = std::shared_ptr<Handle>;

  Handle(const std::string& joint_name, const std::string& state_name, const std::string& command_name)
      : current_(0.0),
        command_(0.0),
        state_handle_(joint_name, state_name, &current_),
        command_handle_(joint_name, command_name, &command_) {}

  Handle(const std::string& joint_name, const std::string& interface_name)
      : Handle(joint_name, interface_name, interface_name) {}

  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface GetStateInterface() {
    return hardware_interface::LoanedStateInterface(state_handle_);
  }
  hardware_interface::LoanedCommandInterface GetCommandInterface() {
    return hardware_interface::LoanedCommandInterface(command_handle_);
  }

  double command() const { return command_; }
  void set_current(double x) { current_ = x; }

 private:
  double current_;
  double command_;

  hardware_interface::StateInterface state_handle_;
  hardware_interface::CommandInterface command_handle_;
};

class BoolHandle : public Handle {
 public:
  using Ptr = std::shared_ptr<BoolHandle>;

  BoolHandle(const std::string& joint_name, const std::string& state_name, const std::string& command_name)
      : Handle(joint_name, state_name, command_name) {}

  bool bool_command() const { return command() > 0.0; }
  void set_current(bool x) {
    if (x) {
      Handle::set_current(1.0);
    } else {
      Handle::set_current(-1.0);
    }
  }
};

struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  std::vector<Handle::Ptr> drive_mode;

  explicit HardwareStub(std::vector<std::string>& joint_names) {
    BOOST_FOREACH (std::string joint_name, joint_names) {
      drive_mode.push_back(std::make_shared<Handle>(joint_name, "current_drive_mode", "command_drive_mode"));
    }
    for (int i = 0; i < joint_names.size(); i++) {
      command_interfaces.emplace_back(drive_mode[i]->GetCommandInterface());
      state_interfaces.emplace_back(drive_mode[i]->GetStateInterface());
    }
  }
};


class TestableExxxDriveModeController : public ExxxDriveModeController {
 public:
  using Ptr = std::shared_ptr<ExxxDriveModeController>;

  TestableExxxDriveModeController();
  ~TestableExxxDriveModeController() = default;

  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;

  void SkipConfigure();
};

TestableExxxDriveModeController::TestableExxxDriveModeController() {
  EXPECT_EQ(ControllerInterface::init(kControllerNodeName), controller_interface::return_type::OK);
}

controller_interface::return_type TestableExxxDriveModeController::init(const std::string& controller_name,
                                                                        const std::string& namespace_,
                                                                        const rclcpp::NodeOptions& node_options) {
  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

void TestableExxxDriveModeController::SkipConfigure() {}


}  // namespace tmc_realtime_controllers
