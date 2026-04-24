// Copyright (C) 2025 Toyota Motor Corporation. All rights reserved.
#include <fstream>

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_utils/qos.hpp>

#include <tmc_realtime_controllers/motion_command_limiter_controller.hpp>


namespace {
constexpr double kEpsilon = 1.0e-6;
const rclcpp::Duration kDefaultPeriod(0, 10'000'000);  // 10ms

const char* const kTestJointName = "test_joint";
const char* const kReferenceControllerName = "motion_command_limiter_controller";

const char* const kJointControlModeInterfaceName = "drive_mode";
constexpr int32_t kPositionControlModeValue = 100;
constexpr int32_t kVelocityControlModeValue = 200;

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

template<typename T>
void AssertIn(const T& value, const std::vector<T>& container) {
  ASSERT_TRUE(std::find(container.begin(), container.end(), value) != container.end());
}

}  // namespace

namespace tmc_realtime_controllers {

class MotionCommandLimiterControllerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SetUpCommandPositionController(const std::optional<double>& acceleration_limit,
                                      const double default_position);
  void SetUpCommandVelocityController(const std::optional<double>& acceleration_limit,
                                      const double default_position,
                                      const double default_acceleration_time = 0.0);

  void ConfigureController(const bool use_command_position,
                           const bool use_command_velocity,
                           const std::optional<double>& acceleration_limit,
                           const double default_acceleration_time = 0.0);
  void TestStateInterfaces();
  void AssignInterfaces();

  std::shared_ptr<MotionCommandLimiterController> controller_;

  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  double command_position_ = 0.0;
  double command_velocity_ = 0.0;
  double control_mode_ = 0.0;

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  double state_position_ = 0.0;
  double state_velocity_ = 0.0;

  std::vector<hardware_interface::CommandInterface> reference_interfaces_;
  uint32_t reference_position_index_ = 0;
  uint32_t reference_velocity_index_ = 0;
  uint32_t reference_control_mode_index_ = 0;

  void SetReferencePosition(double value) {
    reference_interfaces_[reference_position_index_].set_value(value);
  }

  void SetReferenceVelocity(double value) {
    reference_interfaces_[reference_velocity_index_].set_value(value);
  }

  void SetReferenceControlMode(int32_t value) {
    reference_interfaces_[reference_control_mode_index_].set_value(static_cast<double>(value));
  }

  double GetCommandPosition() const {
    return command_position_;
  }

  double GetCommandVelocity() const {
    return command_velocity_;
  }

  int32_t GetCommandControlMode() const {
    return static_cast<int32_t>(control_mode_);
  }

  void UpdateState(const bool use_command_position, const bool use_command_velocity,
                   const rclcpp::Duration& period = kDefaultPeriod);
};

void MotionCommandLimiterControllerTest::SetUp() {
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kTestJointName, hardware_interface::HW_IF_POSITION, &command_position_));
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kTestJointName, hardware_interface::HW_IF_VELOCITY, &command_velocity_));
  command_interfaces_.emplace_back(hardware_interface::CommandInterface(
      kTestJointName, kJointControlModeInterfaceName, &control_mode_));

  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kTestJointName, hardware_interface::HW_IF_POSITION, &state_position_));
  state_interfaces_.emplace_back(hardware_interface::StateInterface(
      kTestJointName, hardware_interface::HW_IF_VELOCITY, &state_velocity_));
}

void MotionCommandLimiterControllerTest::SetUpCommandPositionController(
    const std::optional<double>& acceleration_limit,
    const double default_position) {
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

void MotionCommandLimiterControllerTest::SetUpCommandVelocityController(
    const std::optional<double>& acceleration_limit,
    const double default_position,
    const double default_acceleration_time) {
  ConfigureController(false, true, acceleration_limit, default_acceleration_time);

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

void MotionCommandLimiterControllerTest::ConfigureController(
    const bool use_command_position,
    const bool use_command_velocity,
    const std::optional<double>& acceleration_limit,
    const double default_acceleration_time) {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("robot_description", GetRobotDescription()),
    rclcpp::Parameter("joint_name", kTestJointName),
    rclcpp::Parameter("use_command_position", use_command_position),
    rclcpp::Parameter("use_command_velocity", use_command_velocity),
    rclcpp::Parameter("default_acceleration_time", default_acceleration_time),
  };
  if (acceleration_limit.has_value()) {
    options.parameter_overrides().emplace_back("acceleration_limit", acceleration_limit.value());
  }

  controller_ = std::make_shared<MotionCommandLimiterController>();
  ASSERT_EQ(controller_->init(kReferenceControllerName, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void MotionCommandLimiterControllerTest::TestStateInterfaces() {
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


void MotionCommandLimiterControllerTest::AssignInterfaces() {
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

void MotionCommandLimiterControllerTest::UpdateState(
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

TEST_F(MotionCommandLimiterControllerTest, ControlModeSwitching) {
  // 特殊な初期化 + このテストケースのみなので，ここで書いてしまう
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("robot_description", GetRobotDescription()),
    rclcpp::Parameter("joint_name", kTestJointName),
    rclcpp::Parameter("control_mode_switching", true),
    rclcpp::Parameter("control_mode_interface_name", kJointControlModeInterfaceName),
    rclcpp::Parameter("position_control_mode", kPositionControlModeValue),
    rclcpp::Parameter("velocity_control_mode", kVelocityControlModeValue),
  };

  controller_ = std::make_shared<MotionCommandLimiterController>();
  ASSERT_EQ(controller_->init(kReferenceControllerName, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  const auto command_interface_configuration = controller_->command_interface_configuration();
  ASSERT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  const auto& command_interface_names = command_interface_configuration.names;
  ASSERT_EQ(command_interface_names.size(), 3);
  AssertIn(std::string(kTestJointName) + "/" + hardware_interface::HW_IF_POSITION, command_interface_names);
  AssertIn(std::string(kTestJointName) + "/" + hardware_interface::HW_IF_VELOCITY, command_interface_names);
  AssertIn(std::string(kTestJointName) + "/" + kJointControlModeInterfaceName, command_interface_names);

  TestStateInterfaces();

  reference_interfaces_ = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces_.size(), 3);
  std::vector<std::string> reference_interface_names;
  for (const auto& interface : reference_interfaces_) {
    reference_interface_names.push_back(interface.get_name());
  }
  const std::string prefix = std::string(kReferenceControllerName) + "/";
  reference_position_index_ = GetIndex(reference_interface_names, prefix + hardware_interface::HW_IF_POSITION);
  reference_velocity_index_ = GetIndex(reference_interface_names, prefix + hardware_interface::HW_IF_VELOCITY);
  reference_control_mode_index_ = GetIndex(reference_interface_names, prefix + kJointControlModeInterfaceName);
  AssertIn(reference_position_index_, std::vector<uint32_t>{0, 1, 2});
  AssertIn(reference_velocity_index_, std::vector<uint32_t>{0, 1, 2});
  AssertIn(reference_control_mode_index_, std::vector<uint32_t>{0, 1, 2});

  AssignInterfaces();

  state_position_ = 0.0;
  state_velocity_ = 0.0;

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  SetReferencePosition(1.0);
  SetReferenceVelocity(-1.0);
  SetReferenceControlMode(kPositionControlModeValue);

  double previous_command = GetCommandVelocity();

  // MaxVel: 0.5, MaxAcc: inf, Period: 0.01s

  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.005, kEpsilon);
  EXPECT_NEAR(GetCommandVelocity(), previous_command, kEpsilon);
  EXPECT_EQ(GetCommandControlMode(), kPositionControlModeValue);

  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.010, kEpsilon);
  EXPECT_NEAR(GetCommandVelocity(), previous_command, kEpsilon);
  EXPECT_EQ(GetCommandControlMode(), kPositionControlModeValue);

  SetReferenceControlMode(kVelocityControlModeValue);

  previous_command = GetCommandPosition();

  UpdateState(false, true);
  EXPECT_NEAR(GetCommandPosition(), previous_command, kEpsilon);
  EXPECT_NEAR(GetCommandVelocity(), -0.5, kEpsilon);
  EXPECT_EQ(GetCommandControlMode(), kVelocityControlModeValue);

  UpdateState(false, true);
  EXPECT_NEAR(GetCommandPosition(), previous_command, kEpsilon);
  EXPECT_NEAR(GetCommandVelocity(), -0.5, kEpsilon);
  EXPECT_EQ(GetCommandControlMode(), kVelocityControlModeValue);

  // ここで位置は0.0へ戻ってきているはず

  SetReferenceControlMode(kPositionControlModeValue);

  previous_command = GetCommandVelocity();

  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.005, kEpsilon);
  EXPECT_NEAR(GetCommandVelocity(), previous_command, kEpsilon);
  EXPECT_EQ(GetCommandControlMode(), kPositionControlModeValue);
}

TEST_F(MotionCommandLimiterControllerTest, CommonAndIndividualSettings) {
  const std::string kCommonJoint = "common_joint";
  const std::string kVelocityJoint = "velocity_joint";

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
    rclcpp::Parameter("robot_description", GetRobotDescription()),
    rclcpp::Parameter("joint_names", std::vector<std::string>{
        kCommonJoint, kVelocityJoint
    }),
    rclcpp::Parameter("control_mode_switching", true),
    rclcpp::Parameter("control_mode_interface_name", kJointControlModeInterfaceName),
    rclcpp::Parameter("position_control_mode", kPositionControlModeValue),
    rclcpp::Parameter("velocity_control_mode", kVelocityControlModeValue),
    rclcpp::Parameter(kVelocityJoint + ".control_mode_switching", false),
    rclcpp::Parameter(kVelocityJoint + ".use_command_velocity", true),
  };

  controller_ = std::make_shared<MotionCommandLimiterController>();
  ASSERT_EQ(controller_->init(kReferenceControllerName, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  const auto command_interface_configuration = controller_->command_interface_configuration();
  ASSERT_EQ(command_interface_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  const auto& interface_names = command_interface_configuration.names;
  // 個別設定優先なのでcommon_jointはposition, velocity, control_mode，velocity_jointはvelocityのみ
  ASSERT_EQ(interface_names.size(), 4);
  AssertIn(std::string(kCommonJoint) + "/" + hardware_interface::HW_IF_POSITION, interface_names);
  AssertIn(std::string(kCommonJoint) + "/" + hardware_interface::HW_IF_VELOCITY, interface_names);
  AssertIn(std::string(kCommonJoint) + "/" + kJointControlModeInterfaceName, interface_names);
  AssertIn(std::string(kVelocityJoint) + "/" + hardware_interface::HW_IF_VELOCITY, interface_names);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandPositionWithAccelerationLimit) {
  SetUpCommandPositionController(10.0, 0.0);

  // 加速度制約に引っかかる
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

  // 速度制約に引っかかる
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.020, kEpsilon);

  // 位置制限でサチる
  SetReferencePosition(5.0);
  for (auto i = 0; i < static_cast<int32_t>(5.0 / 0.5 / kDefaultPeriod.seconds()); ++i) {
    UpdateState(true, false);
  }
  EXPECT_NEAR(GetCommandPosition(), 5.0, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandPositionOverPositionLimitUpper) {
  SetUpCommandPositionController(std::nullopt, 5.000001);

  // 位置制限でサチる
  SetReferencePosition(5.000002);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 5.0, kEpsilon);

  // 反対方向では速度制限に引っかかる普通の動作
  SetReferencePosition(0.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 4.995, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandPositionOverPositionLimitLower) {
  SetUpCommandPositionController(std::nullopt, -5.000001);

  // 位置制限でサチる
  SetReferencePosition(-5.000002);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), -5.0, kEpsilon);

  // 反対方向では速度制限に引っかかる普通の動作
  SetReferencePosition(0.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), -4.995, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, ChangeLimitsViaROSParameter) {
  SetUpCommandPositionController(10.0, 0.0);

  // 加速度制約に引っかかる
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.001, kEpsilon);

  // 加速度制約を変更する
  controller_->get_node()->set_parameter(rclcpp::Parameter("acceleration_limit", 1.0e10));

  // 最大速度に達する(≒速度制約に引っかかる)
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.006, kEpsilon);

  // 速度制約を変更する
  controller_->get_node()->set_parameter(rclcpp::Parameter("velocity_limit", 1.0e10));

  // 速度/加速度制約に引っかからない
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 1.0, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, ChangeLimitsViaTopic) {
  SetUpCommandPositionController(10.0, 0.0);

  const auto client_node = rclcpp::Node::make_shared("test_client_node");
  const auto publisher = client_node->create_publisher<moveit_msgs::msg::JointLimits>(
      std::string(kReferenceControllerName) + "/joint_limits", tmc_utils::BestEffortQoS());
  const auto timeout = client_node->get_clock()->now() + rclcpp::Duration(1, 0);
  while (publisher->get_subscription_count() == 0) {
    if (client_node->get_clock()->now() > timeout) {
      FAIL() << "Timeout while waiting for subscription to joint_limits topic";
    }
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }

  // 加速度制約に引っかかる
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.001, kEpsilon);

  // 加速度制約を変更する，反映されたことの確認が不可能なので適当な回数送る
  moveit_msgs::msg::JointLimits max_acc_msg;
  max_acc_msg.joint_name = kTestJointName;
  max_acc_msg.has_acceleration_limits = true;
  max_acc_msg.max_acceleration = 1.0e10;
  for (int i = 0; i < 20; ++i) {
    publisher->publish(max_acc_msg);
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }

  // 最大速度に達する(≒速度制約に引っかかる)
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.006, kEpsilon);

  // 速度制約を変更する
  moveit_msgs::msg::JointLimits max_vel_msg;
  max_vel_msg.joint_name = kTestJointName;
  max_vel_msg.has_velocity_limits = true;
  max_vel_msg.max_velocity = 1.0e10;
  for (int i = 0; i < 20; ++i) {
    publisher->publish(max_vel_msg);
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
  }

  // 速度/加速度制約に引っかからない
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 1.0, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandPositionWithoutAccelerationLimit) {
  SetUpCommandPositionController(std::nullopt, 0.0);

  // 速度制限でサチる
  SetReferencePosition(1.0);
  UpdateState(true, false);
  EXPECT_NEAR(GetCommandPosition(), 0.005, kEpsilon);

  // 位置制限でサチる
  SetReferencePosition(5.0);
  for (auto i = 0; i < static_cast<int32_t>(5.0 / 0.5 / kDefaultPeriod.seconds()); ++i) {
    UpdateState(true, false);
  }
  EXPECT_NEAR(GetCommandPosition(), 5.0, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityWithAccelerationLimit) {
  SetUpCommandVelocityController(10.0, 0.0);

  // 加速度制約に引っかかる
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.1, kEpsilon);

  // 加速度制約に引っかからない
  SetReferenceVelocity(0.19);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.19, kEpsilon);

  // 速度制限でサチる
  SetReferenceVelocity(1.0);
  for (auto i = 0; i < 4; ++i) {
    UpdateState(false, true);
  }
  EXPECT_NEAR(GetCommandVelocity(), 0.5, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityWithoutAccelerationLimit) {
  SetUpCommandVelocityController(std::nullopt, 0.0);

  // 速度制限でサチる
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.5, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityNearPositionLimitUpper) {
  constexpr double kMaxAcc = 2.0;
  SetUpCommandVelocityController(kMaxAcc, 4.999999);

  // リミットで止まる制約に引っかかる
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.0001, kEpsilon);

  // 反対方向では単純に加速度リミットに引っかかる
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), -kMaxAcc * kDefaultPeriod.seconds() + 0.0001, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityOverPositionLimitUpper) {
  SetUpCommandVelocityController(2.0, 5.000001);

  // リミットを超えているのでその方向には動かない
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.0, kEpsilon);

  // 反対方向では単純に加速度リミットに引っかかる
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), -0.02, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityNearPositionLimitLower) {
  constexpr double kMaxAcc = 2.0;
  SetUpCommandVelocityController(kMaxAcc, -4.999999);

  // リミットで止まる制約に引っかかる
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), -0.0001, kEpsilon);

  // 反対方向では単純に加速度リミットに引っかかる
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), kMaxAcc * kDefaultPeriod.seconds() - 0.0001, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseCommandVelocityOverPositionLimitLower) {
  SetUpCommandVelocityController(2.0, -5.000001);

  // リミットを超えているのでその方向には動かない
  SetReferenceVelocity(-0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.0, kEpsilon);

  // 反対方向では単純に加速度リミットに引っかかる
  SetReferenceVelocity(0.5);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.02, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, DefaultAccelerationTime) {
  SetUpCommandVelocityController(std::nullopt, 0.0, 0.1);

  // 最大速度に達するまでに0.1秒かかるという加速度制約に引っかかる
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.05, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, UseExplicitAccelerationLimit) {
  SetUpCommandVelocityController(10.0, 0.0, 0.1);

  // 明示的な加速度リミットが優先される
  SetReferenceVelocity(1.0);
  UpdateState(false, true);
  EXPECT_NEAR(GetCommandVelocity(), 0.1, kEpsilon);
}

TEST_F(MotionCommandLimiterControllerTest, MultiJointSupport) {
  // 細かい挙動のテストは単軸で行い，複数軸のサポートを確認するテストでは挙動の詳細は確認しない
  // ここでは，複数軸のサポートと，パラメータの設定が正しく行われることを確認する
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

  controller_ = std::make_shared<MotionCommandLimiterController>();
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

  // joint1は位置指令，速度制限に引っかかるので速度×時間
  // joint2は速度指令，速度制限に引っかかるので速度制限の値そのまま
  EXPECT_NEAR(command_values[0], kVelocityLimitJoint1 * kDefaultPeriod.seconds(), 0.0);
  EXPECT_NEAR(command_values[1], kVelocityLimitJoint2, kEpsilon);
}

}  // namespace tmc_realtime_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
