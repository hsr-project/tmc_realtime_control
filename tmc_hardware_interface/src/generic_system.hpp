// Copyright (c) 2026 Toyota Motor Corporation
#ifndef TMC_HARDWARE_INTERFACE_GENERIC_SYSTEM_HPP_
#define TMC_HARDWARE_INTERFACE_GENERIC_SYSTEM_HPP_

#include <string>
#include <vector>

#include <mock_components/generic_system.hpp>

namespace tmc_hardware_interface {

class GenericSystem : public mock_components::GenericSystem {
 public:
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::vector<double> command_drive_modes_;
};


}  // namespace tmc_hardware_interface
#endif  // TMC_HARDWARE_INTERFACE_GENERIC_SYSTEM_HPP_
