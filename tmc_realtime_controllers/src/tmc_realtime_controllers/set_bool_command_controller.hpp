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
#ifndef TMC_REALTIME_CONTROLLERSTMC_SET_BOOL_COMMAND_CONTROLLER_HPP_
#define TMC_REALTIME_CONTROLLERSTMC_SET_BOOL_COMMAND_CONTROLLER_HPP_

#include <std_srvs/SetBool.h>

#include <tmc_hardware_interface/set_bool_command_interface.hpp>
#include "command_service_controller.hpp"

namespace tmc_realtime_controllers {
/**
 * @brief Controller for SetBoolCommandHandle
 * Controller class supporting multiple handles
 *
 * Create a service with the name "[handle name]".
 */
// Ensure that the HardwareInterface passed to the controller is an actual class, not a template class
typedef CommandServiceController<tmc_hardware_interface::SetBoolCommandInterface, std_srvs::SetBool>
    SetBoolCommandController;

/**
 * @brief Conversion function from RosMessage to Handle for SetBool
 * @param[in] ros_req ROSRequest
 * @param[out] req HandleRequest
 */
template <>
void SetBoolCommandController::CommandService::SetRequest(
    const std_srvs::SetBool::Request& ros_req, tmc_hardware_interface::SetBoolCommandHandle::RequestType& req) {
  req = ros_req.data;
}

/**
 * @brief Conversion function from Handle to RosMessage for SetBool
 * @param[in] res HandleResponse
 * @param[out] ros_res ROSResponse
 */
template <>
void SetBoolCommandController::CommandService::SetResponse(
    const tmc_hardware_interface::SetBoolCommandHandle::ResponseType& res, std_srvs::SetBool::Response& ros_res) {
  ros_res.success = res;
  ros_res.message = "OK";
}
}  // namespace tmc_realtime_controllers
#endif  // TMC_REALTIME_CONTROLLERSTMC_SET_BOOL_COMMAND_CONTROLLER_HPP_
