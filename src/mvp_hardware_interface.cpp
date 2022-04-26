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

#include "ipa_outdoor_drivers/mvp_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ipa_outdoor_drivers
{
hardware_interface::return_type MvpHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MvpHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MvpHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MvpHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MvpHardwareInterface"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MvpHardwareInterface"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  clock_ = rclcpp::Clock();

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MvpHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_motor", hardware_interface::HW_IF_POSITION, &left_wheel_position_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_motor", hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_motor", hardware_interface::HW_IF_POSITION, &right_wheel_position_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_motor", hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MvpHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "left_motor", hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "right_motor", hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_command_));
  return command_interfaces;
}

hardware_interface::return_type MvpHardwareInterface::start()
{
  // init variables
  left_wheel_velocity_command_ = 0;
  left_wheel_position_state_ = 0;
  right_wheel_velocity_command_ = 0;
  right_wheel_position_state_ = 0;

  ////////////////// ACTIVATE MOTORS ////////////////

  status_ = hardware_interface::status::STARTED;

  last_timestamp_ = clock_.now();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::stop()
{
  ////////////////// DEACTIVATE MOTORS ////////////////

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::read()
{
  // get encoder data convert to SI units and save them in position and velocity state member variables

  current_timestamp_ = clock_.now();
  update_rate_ = current_timestamp_.seconds() - last_timestamp_.seconds();

  left_wheel_velocity_state_ = read_left_wheel_raw_ * (2 * M_PI / 60) * (1 / GEAR_TRANSMISSION);
  right_wheel_velocity_state_ = read_right_wheel_raw_ * (2 * M_PI / 60) * (1 / GEAR_TRANSMISSION);

  left_wheel_position_state_ += update_rate_ * left_wheel_velocity_state_;
  right_wheel_position_state_ += update_rate_ * right_wheel_velocity_state_;
  // convert to -pi to pi???

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::write()
{
  write_left_wheel_raw_ = left_wheel_velocity_command_ * GEAR_TRANSMISSION * (60 / (2 * M_PI));
  write_right_wheel_raw_ = right_wheel_velocity_command_ * GEAR_TRANSMISSION * (60 / (2 * M_PI));
  // send the received commands (command member variables) to the motors (in RPM)

  return hardware_interface::return_type::OK;
}

}  // namespace mvp_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ipa_outdoor_drivers::MvpHardwareInterface, hardware_interface::SystemInterface)