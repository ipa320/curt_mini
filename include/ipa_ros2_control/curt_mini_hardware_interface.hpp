// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CURT_MINI_SYSTEM_HPP_
#define CURT_MINI_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mini_cheetah_motor_driver/MotorDriver.hpp"

namespace ipa_ros2_control {
class CurtMiniHardwareInterface : public hardware_interface::BaseInterface<
                                      hardware_interface::SystemInterface> {
public:
  hardware_interface::return_type
  configure(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  // inspired by Jackal github
  void writeCommandsToHardware();
  void updateJointsFromHardware();
  // {front_left, front_right, back_left, back_right}
  std::vector<int> motor_ids_ = { 0x01, 0x02, 0x03, 0x04 };
  const char* can_comm_ = "can0";
  motor_driver::MotorDriver* motor_controller_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_velocity_;

  // map for motors
  std::map<int, motor_driver::motorCommand> command_map_;
  // controller params
  float k_stiffness_ = 0.0;
  float k_damping_ = 1.0;
  // needed?
  float gear_ratio_ = 9.0;

  // map of joint names and there index
  std::map<std::string, uint8_t> wheel_joints_;
  
};

} // namespace ipa_ros2_control

#endif // CURT_MINI_SYSTEM_HPP