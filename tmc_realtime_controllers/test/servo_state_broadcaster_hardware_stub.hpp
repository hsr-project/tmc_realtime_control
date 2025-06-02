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
/// @brief Handle that simulates hardware for testing
#include <memory>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tmc_realtime_controllers/servo_state_broadcaster.hpp>


namespace tmc_realtime_controllers {

const char* const kControllerNodeName = "controller_manager";

class Handle {
 public:
  using Ptr = std::shared_ptr<Handle>;

  Handle(const std::string& joint_name, const std::string& state_name)
      : current_(0.0), state_handle_(joint_name, state_name, &current_) {}

  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface GetStateInterface() {
    return hardware_interface::LoanedStateInterface(state_handle_);
  }

  void set_current(double x) { current_ = x; }

 private:
  double current_;

  hardware_interface::StateInterface state_handle_;
};

struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;
  std::vector<Handle::Ptr> current_drive_mode;
  std::vector<Handle::Ptr> current_position;
  std::vector<Handle::Ptr> current_velocity;
  std::vector<Handle::Ptr> current_effort;
  std::vector<Handle::Ptr> temperature;
  std::vector<Handle::Ptr> current;
  std::vector<Handle::Ptr> mrpos;
  std::vector<Handle::Ptr> avagopos;
  std::vector<Handle::Ptr> error_status;

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;

  explicit HardwareStub(std::vector<std::string>& joint_names) {
    BOOST_FOREACH (std::string joint_name, joint_names) {
      current_drive_mode.push_back(std::make_shared<Handle>(joint_name, "current_drive_mode"));
      current_position.push_back(std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_POSITION));
      current_velocity.push_back(std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_VELOCITY));
      current_effort.push_back(std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_EFFORT));
      temperature.push_back(std::make_shared<Handle>(joint_name, "temperature"));
      current.push_back(std::make_shared<Handle>(joint_name, "current"));
      mrpos.push_back(std::make_shared<Handle>(joint_name, "mr_pos"));
      avagopos.push_back(std::make_shared<Handle>(joint_name, "avago_pos"));
      error_status.push_back(std::make_shared<Handle>(joint_name, "error_status"));
    }
    for (int i = 0; i < joint_names.size(); i++) {
      state_interfaces.emplace_back(current_drive_mode[i]->GetStateInterface());
      state_interfaces.emplace_back(current_position[i]->GetStateInterface());
      state_interfaces.emplace_back(current_velocity[i]->GetStateInterface());
      state_interfaces.emplace_back(current_effort[i]->GetStateInterface());
      state_interfaces.emplace_back(temperature[i]->GetStateInterface());
      state_interfaces.emplace_back(current[i]->GetStateInterface());
      state_interfaces.emplace_back(mrpos[i]->GetStateInterface());
      state_interfaces.emplace_back(avagopos[i]->GetStateInterface());
      state_interfaces.emplace_back(error_status[i]->GetStateInterface());
    }
  }
};


class TestableServoStateBroadcaster : public ServoStateBroadcaster {
 public:
  using Ptr = std::shared_ptr<ServoStateBroadcaster>;

  TestableServoStateBroadcaster();
  ~TestableServoStateBroadcaster() = default;

  controller_interface::return_type init(const std::string& controller_name, const std::string& namespace_ = "",
                                         const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) override;

  void SkipConfigure();
};

TestableServoStateBroadcaster::TestableServoStateBroadcaster() {
  EXPECT_EQ(ControllerInterface::init(kControllerNodeName), controller_interface::return_type::OK);
}

controller_interface::return_type TestableServoStateBroadcaster::init(const std::string& controller_name,
                                                                      const std::string& namespace_,
                                                                      const rclcpp::NodeOptions& node_options) {
  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

void TestableServoStateBroadcaster::SkipConfigure() {}

}  // namespace tmc_realtime_controllers
