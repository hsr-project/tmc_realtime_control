// Copyright (c) 2026 Toyota Motor Corporation

#include <tmc_joint_command_controller/state_publisher.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tmc_utils/parameters.hpp>

#include "common.hpp"

namespace tmc_joint_command_controller {

StatePublisher::StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                               const JointsInfo::Ptr& joints_info,
                               Accessor* accessor)
    : node_(node),
      joints_info_(joints_info),
      accessor_(accessor),
      publish_position_(false),
      publish_velocity_(false),
      publish_effort_(false) {
  const auto state_interfaces = tmc_utils::GetParameter<std::vector<std::string>>(
      node_, "state_interfaces", {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY});
  if (std::find(state_interfaces.begin(), state_interfaces.end(),
                hardware_interface::HW_IF_POSITION) != state_interfaces.end()) {
    publish_position_ = true;
  }
  if (std::find(state_interfaces.begin(), state_interfaces.end(),
                hardware_interface::HW_IF_VELOCITY) != state_interfaces.end()) {
    publish_velocity_ = true;
  }
  if (std::find(state_interfaces.begin(), state_interfaces.end(),
                hardware_interface::HW_IF_EFFORT) != state_interfaces.end()) {
    publish_effort_ = true;
  }

  publisher_impl_ = node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_,
                                                   [&](control_msgs::msg::JointTrajectoryControllerState& msg) {
    msg.joint_names = joints_info_->names();
    msg.reference.positions.resize(joints_info_->names().size(), 0.0);
    msg.reference.velocities.resize(joints_info_->names().size(), 0.0);
    msg.reference.effort.resize(joints_info_->names().size(), 0.0);
    if (publish_position_) {
      msg.feedback.positions.resize(joints_info_->names().size(), 0.0);
      msg.error.positions.resize(joints_info_->names().size(), 0.0);
    }
    if (publish_velocity_) {
      msg.feedback.velocities.resize(joints_info_->names().size(), 0.0);
      msg.error.velocities.resize(joints_info_->names().size(), 0.0);
    }
    if (publish_effort_) {
      msg.feedback.effort.resize(joints_info_->names().size(), 0.0);
      msg.error.effort.resize(joints_info_->names().size(), 0.0);
    }
  });
}

void StatePublisher::AddStateInterfaces(std::vector<std::string>& state_interfaces) const {
  if (publish_position_) {
    joints_info_->AddStatePositionInterfaces(state_interfaces);
  }
  if (publish_velocity_) {
    joints_info_->AddStateVelocityInterfaces(state_interfaces);
  }
  if (publish_effort_) {
    joints_info_->AddStateEffortInterfaces(state_interfaces);
  }
}

void StatePublisher::Activate(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  if (publish_position_) {
    position_state_interface_indices_ = GetPositionInterfaceIndices(state_interfaces, joints_info_->names());
  }
  if (publish_velocity_) {
    velocity_state_interface_indices_ = GetVelocityInterfaceIndices(state_interfaces, joints_info_->names());
  }
  if (publish_effort_) {
    effort_state_interface_indices_ = GetEffortInterfaceIndices(state_interfaces, joints_info_->names());
  }
}

void StatePublisher::Publish(const rclcpp::Time& time, const trajectory_msgs::msg::JointTrajectoryPoint& desired) {
  const auto msg = publisher_->trylock();
  if (msg) {
    // TODO(Takeshita) 無限回転軸対応
    UpdateReference(desired.positions, msg->reference.positions);
    UpdateReference(desired.velocities, msg->reference.velocities);
    UpdateReference(desired.effort, msg->reference.effort);

    if (publish_position_) {
      UpdateFeedback(position_state_interface_indices_, msg->feedback.positions);
      UpdateError(desired.positions, msg->feedback.positions, msg->error.positions);
    }
    if (publish_velocity_) {
      UpdateFeedback(velocity_state_interface_indices_, msg->feedback.velocities);
      UpdateError(desired.velocities, msg->feedback.velocities, msg->error.velocities);
    }
    if (publish_effort_) {
      UpdateFeedback(effort_state_interface_indices_, msg->feedback.effort);
      UpdateError(desired.effort, msg->feedback.effort, msg->error.effort);
    }

    msg->header.stamp = time;
    publisher_->unlockAndPublish();
  }
}

void StatePublisher::UpdateReference(const std::vector<double>& desired, std::vector<double>& reference_out) const {
  if (desired.size() == reference_out.size()) {
    reference_out = desired;
  } else {
    std::fill(reference_out.begin(), reference_out.end(), 0.0);
  }
}

void StatePublisher::UpdateFeedback(const std::vector<size_t>& state_interface_indices,
                                    std::vector<double>& feedback_out) const {
  for (size_t i = 0; i < state_interface_indices.size(); ++i) {
    feedback_out[i] = accessor_->GetState(state_interface_indices[i]);
  }
}

void StatePublisher::UpdateError(const std::vector<double>& desired,
                                 const std::vector<double>& feedback,
                                 std::vector<double>& error_out) const {
  if (desired.size() != feedback.size()) {
    std::fill(error_out.begin(), error_out.end(), 0.0);
    return;
  }
  for (size_t i = 0; i < desired.size(); ++i) {
    error_out[i] = desired[i] - feedback[i];
  }
}

}  // namespace tmc_joint_command_controller
