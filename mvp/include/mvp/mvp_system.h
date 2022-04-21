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

#ifndef MVP_SYSTEM_H_
#define MVP_SYSTEM_H_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mvp_system 
{
class MvpHardwareInterface : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  double left_wheel_velocity_command_;
  double left_wheel_position_state_;
  double left_wheel_velocity_state_;
  double right_wheel_velocity_command_;
  double right_wheel_position_state_;
  double right_wheel_velocity_state_;

  int32_t prev_left_wheel_raw_;
  int32_t prev_right_wheel_raw_;
  int odom_left_overflow_;
  int odom_right_overflow_;

  double update_rate_;

  static constexpr double ENCODER_RESOLUTION = 2048; // 512 * 4

  // Store time between update loops
  rclcpp::Clock clock_;
  rclcpp::Time last_timestamp_;
  rclcpp::Time current_timestamp; // Avoid initialization on each read

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;
};

} // namespace mvp_system

#endif // MVP_SYSTEM_H_