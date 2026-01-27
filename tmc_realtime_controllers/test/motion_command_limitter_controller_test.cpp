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
#include <fstream>

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_realtime_controllers/motion_command_limitter_controller.hpp>


namespace {
constexpr double kEpsilon = 1.0e-6;
const rclcpp::Duration kDefaultPeriod(0, 10'000'000);  // 10ms

const char* const kTestJointName = "test_joint";
const char* const kReferenceControllerName = "motion_command_limitter_controller";

std::string GetRobotDescription() {
  std::fstream xml_file("robot.xml", std::fstream::in);
  std::string robot_description;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    robot_description += (line + "\n");
  }
  xml_file.close();
  return robot_description;
}

uint32_t GetIndex(const std::vector<std::string>& names, const std::string& target_name) {
  const auto it = std::find(names.begin(), names.end(), target_name);
  return std::distance(names.begin(), it);
}

}  // namespace

namespace tmc_realtime_controllers {

class MotionCommandLimitterControllerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SetUpCommandPositionController(const std::optional<double>& acceleration_limit, double default_position);
  void SetUpCommandVelocityController(const std::optional<double>& acceleration_limit, double default_position);

  void ConfigureController(const bool use_command_position,
                           const bool use_command_velocity,
                           const std::optional<double>& acceleration_limit);
  void TestStateInterfaces();
  void AssignInterfaces();

  std::shared_ptr<MotionCommandLimitterController> controller_;

  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  double command_position_ = 0.0;
  double command_velocity_ = 0.0;

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  double state_position_ = 0.0;
  double state_velocity_ = 0.0;

  std::vector<hardware_interface::CommandInterface> reference_interfaces_;
  uint32_t reference_position_index_ = 0;
  uint32_t reference_velocity_index_ = 0;

  void SetReferencePosition(double value) {
    reference_interfaces_[reference_position_index_].set_value(value);
  }

  void SetReferenceVelocity(double value) {
    reference_interfaces_[reference_velocity_index_].set_value(value);
  }

  double GetCommandPosition() const {
    return command_position_;
  }

  double GetCommandVelocity() const {
    return command_velocity_;
  }

  void UpdateState(const bool use_command_position, const bool use_command_velocity,
                   const rclcpp::Duration& period = kDefaultPeriod);
};

void MotionCommandLimitterControllerTest::SetUp() {
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kTestJointName, hardware_interface::HW_IF_POSITION, &command_position_));
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kTestJointName, hardware_interface::HW_IF_VELOCITY, &command_velocity_));

  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kTestJointName, hardware_interface::HW_IF_POSITION, &state_position_));
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kTestJointName, hardware_interface::HW_IF_VELOCITY, &state_velocity_));
}

void MotionCommandLimitterControllerTest::SetUpCommandPositionController(
    const std::optional<double>& acceleration_limit,
    double default_position) {
  ConfigureController(true, false, acceleration_limit);

  const auto command_interface_configuration = controller_->command_interface_configuration();
  ASSERT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(command_interface_configuration.names.size(), 1);

  const auto command_position_index = GetIndex(command_interface_configuration.names,
                                               std::string(kTestJointName) + "/" + hardware_interface::HW_IF_POSITION);
  ASSERT_EQ(command_position_index, 0);

  TestStateInterfaces();

  reference_interfaces_ = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces_.size(), 1);
  ASSERT_EQ(reference_interfaces_[0].get_name(),
            std::string(kReferenceControllerName) + "/" + hardware_interface::HW_IF_POSITION);
  reference_position_index_ = 0;

  AssignInterfaces();

  state_position_ = default_position;
  state_velocity_ = 0.0;

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void MotionCommandLimitterControllerTest::SetUpCommandVelocityController(
    const std::optional<double>& acceleration_limit,
    double default_position) {
  ConfigureController(false, true, acceleration_limit);

  const auto command_interface_configuration = controller_->command_interface_configuration();
  ASSERT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(command_interface_configuration.names.size(), 1);

  const auto command_velocity_index = GetIndex(command_interface_configuration.names,
                                               std::string(kTestJointName) + "/" + hardware_interface::HW_IF_VELOCITY);
  ASSERT_EQ(command_velocity_index, 0);

  TestStateInterfaces();

  reference_interfaces_ = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces_.size(), 1);
  ASSERT_EQ(reference_interfaces_[0].get_name(),
            std::string(kReferenceControllerName) + "/" + hardware_interface::HW_IF_VELOCITY);
  reference_velocity_index_ = 0;

  AssignInterfaces();

  state_position_ = default_position;
  state_velocity_ = 0.0;

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void MotionCommandLimitterControllerTest::ConfigureController(
    const bool use_command_position,
    const bool use_command_velocity,
    const std::optional<double>& acceleration_limit) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("robot_description", GetRobotDescription()),
    rclcpp::Parameter("joint_name", kTestJointName),
    rclcpp::Parameter("use_command_position", use_command_position),
    rclcpp::Parameter("use_command_velocity", use_command_velocity),
  };
  if (acceleration_limit.has_value()) {
    options.parameter_overrides().emplace_back("acceleration_limit", acceleration_limit.value());
  }

  controller_ = std::make_shared<MotionCommandLimitterController>();
  ASSERT_EQ(controller_->init(kReferenceControllerName, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void MotionCommandLimitterControllerTest::TestStateInterfaces() {
  const auto state_interface_configuration = controller_->state_interface_configuration();
  ASSERT_EQ(state_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_interface_configuration.names.size(), 2);

  const auto state_position_index = GetIndex(state_interface_configuration.names,
                                             std::string(kTestJointName) + "/" + hardware_interface::HW_IF_POSITION);
  ASSERT_EQ(state_position_index, 0);

  const auto state_velocity_index = GetIndex(state_interface_configuration.names,
                                             std::string(kTestJointName) + "/" + hardware_interface::HW_IF_VELOCITY);
  ASSERT_EQ(state_velocity_index, 1);
}


void MotionCommandLimitterControllerTest::AssignInterfaces() {
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  for (auto& command_interface : command_interfaces_) {
    loaned_command_interfaces.emplace_back(hardware_interface::LoanedCommandInterface(command_interface));
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
  for (auto& state_interface : state_interfaces_) {
    loaned_state_interfaces.emplace_back(hardware_interface::LoanedStateInterface(state_interface));
  }
  controller_->assign_interfaces(std::move(loaned_command_interfaces), std::move(loaned_state_interfaces));
}

void MotionCommandLimitterControllerTest::UpdateState(
    const bool use_command_position, const bool use_command_velocity, const rclcpp::Duration& period) {
  ASSERT_EQ(controller_->update(rclcpp::Time(), period), controller_interface::return_type::OK);

  if (use_command_position) {
    state_position_ = command_position_;
  } else {
    state_position_ += command_velocity_ * period.seconds();
  }

  if (use_command_velocity) {
    state_velocity_ = command_velocity_;
  } else {
    state_velocity_ = (command_position_ - state_position_) / period.seconds();
  }
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandPositionWithAccelerationLimit) {
  SetUpCommandPositionController(10.0, 0.0);

  // Acceleration constraint violation
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.001, kEpsilon);

  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.003, kEpsilon);

  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.006, kEpsilon);

  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.010, kEpsilon);

  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.015, kEpsilon);

  // Speed constraint violation
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.020, kEpsilon);

  // Saturation due to position limit
  SetReferencePosition(5.0);
  for (auto i = 0; i < static_cast<int32_t>(5.0 / 0.5 / kDefaultPeriod.seconds()); ++i) {
    UpdateState(true, false);
  }
  EXPECT_NEAR(GetCommandPosition(), 5.0, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, DynamicLimits) {
  SetUpCommandPositionController(10.0, 0.0);

  // Acceleration constraint violation
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.001, kEpsilon);

  // Change acceleration constraint
  controller_->get_node()->set_parameter(rclcpp::Parameter("acceleration_limit", 1.0e10));

  // Reaching maximum speed (≒ speed constraint violation)
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.006, kEpsilon);

  // Change speed constraint
  controller_->get_node()->set_parameter(rclcpp::Parameter("velocity_limit", 1.0e10));

  // No speed/acceleration constraint violation
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 1.0, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandPositionWithoutAccelerationLimit) {
  SetUpCommandPositionController(std::nullopt, 0.0);

  // Saturation due to speed limit
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.005, kEpsilon);

  // Saturation due to position limit
  SetReferencePosition(5.0);
  for (auto i = 0; i < static_cast<int32_t>(5.0 / 0.5 / kDefaultPeriod.seconds()); ++i) {
    UpdateState(true, false);
  }
  EXPECT_NEAR(GetCommandPosition(), 5.0, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandVelocityWithAccelerationLimit) {
  SetUpCommandVelocityController(10.0, 0.0);

  // Acceleration constraint violation
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.1, kEpsilon);

  // No acceleration constraint violation
  SetReferenceVelocity(0.19);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.19, kEpsilon);

  // Saturation due to speed limit
  SetReferenceVelocity(1.0);
  for (auto i = 0; i < 4; ++i) {
    UpdateState(false, true);
  }
  EXPECT_NEAR(GetCommandVelocity(), 0.5, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandVelocityWithoutAccelerationLimit) {
  SetUpCommandVelocityController(std::nullopt, 0.0);

  // Saturation due to speed limit
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.5, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandVelocityNearPositionLimitUpper) {
  SetUpCommandVelocityController(2.0, 4.999999);

  // Constraint violation due to stopping at limit
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.002, kEpsilon);

  // Simply hitting the acceleration limit in the opposite direction
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), -0.018, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, UseCommandVelocityNearPositionLimitLower) {
  SetUpCommandVelocityController(2.0, -4.999999);

  // Constraint violation due to stopping at limit
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), -0.002, kEpsilon);

  // Simply hitting the acceleration limit in the opposite direction
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.018, kEpsilon);
}

TEST_F(MotionCommandLimitterControllerTest, MultiJointSupport) {
  // Conduct detailed behavior tests on a single axis, and for tests confirming multi-axis support, do not check behavior details
  // Here, confirm multi-axis support and correct parameter settings
  const std::string kJoint1 = kTestJointName;
  const std::string kJoint2 = "another_joint";
  const double kVelocityLimitJoint1 = 0.5;
  const double kVelocityLimitJoint2 = 0.3;

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("robot_description", GetRobotDescription()),
    rclcpp::Parameter("joint_names", std::vector<std::string>{kJoint1, kJoint2}),
    rclcpp::Parameter(kJoint1 + ".use_command_position", true),
    rclcpp::Parameter(kJoint1 + ".velocity_limit", kVelocityLimitJoint1),
    rclcpp::Parameter(kJoint2 + ".use_command_velocity", true),
    rclcpp::Parameter(kJoint2 + ".velocity_limit", kVelocityLimitJoint2)
  };

  controller_ = std::make_shared<MotionCommandLimitterController>();
  ASSERT_EQ(controller_->init(kReferenceControllerName, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  const auto command_interface_configuration = controller_->command_interface_configuration();
  ASSERT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(command_interface_configuration.names.size(), 2);
  ASSERT_EQ(GetIndex(command_interface_configuration.names, kJoint1 + "/" + hardware_interface::HW_IF_POSITION), 0);
  ASSERT_EQ(GetIndex(command_interface_configuration.names, kJoint2 + "/" + hardware_interface::HW_IF_VELOCITY), 1);

  const auto state_interface_configuration = controller_->state_interface_configuration();
  ASSERT_EQ(state_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_interface_configuration.names.size(), 4);
  ASSERT_EQ(GetIndex(state_interface_configuration.names, kJoint1 + "/" + hardware_interface::HW_IF_POSITION), 0);
  ASSERT_EQ(GetIndex(state_interface_configuration.names, kJoint1 + "/" + hardware_interface::HW_IF_VELOCITY), 1);
  ASSERT_EQ(GetIndex(state_interface_configuration.names, kJoint2 + "/" + hardware_interface::HW_IF_POSITION), 2);
  ASSERT_EQ(GetIndex(state_interface_configuration.names, kJoint2 + "/" + hardware_interface::HW_IF_VELOCITY), 3);

  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 2);
  ASSERT_EQ(reference_interfaces[0].get_name(),
            std::string(kReferenceControllerName) + "/" + kJoint1 + "/" + hardware_interface::HW_IF_POSITION);
  ASSERT_EQ(reference_interfaces[1].get_name(),
            std::string(kReferenceControllerName) + "/" + kJoint2 + "/" + hardware_interface::HW_IF_VELOCITY);

  double state_value = 0.0;
  state_interfaces_.clear();
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kJoint1, hardware_interface::HW_IF_POSITION, &state_value));
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kJoint1, hardware_interface::HW_IF_VELOCITY, &state_value));
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kJoint2, hardware_interface::HW_IF_POSITION, &state_value));
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kJoint2, hardware_interface::HW_IF_VELOCITY, &state_value));

  std::array<double, 2> command_values = {0.0, 0.0};
  command_interfaces_.clear();
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kJoint1, hardware_interface::HW_IF_POSITION, &command_values[0]));
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kJoint2, hardware_interface::HW_IF_VELOCITY, &command_values[1]));

  AssignInterfaces();

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  reference_interfaces[0].set_value(1.0);
  reference_interfaces[1].set_value(kVelocityLimitJoint2 * 2.0);
  ASSERT_EQ(controller_->update(rclcpp::Time(), kDefaultPeriod), controller_interface::return_type::OK);

  // Joint1 is position command, speed limit violation, so speed × time
  // Joint2 is speed command, speed limit violation, so use the speed limit value as is
  EXPECT_NEAR(command_values[0], kVelocityLimitJoint1 * kDefaultPeriod.seconds(), 0.0);
  EXPECT_NEAR(command_values[1], kVelocityLimitJoint2, kEpsilon);
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
