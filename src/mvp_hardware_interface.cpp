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

#include "ipa_ros2_control/mvp_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ipa_ros2_control {
hardware_interface::return_type
MvpHardwareInterface::configure(const hardware_interface::HardwareInfo &info) {
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // get parameters from urdf setting
  GEAR_RATIO = std::stod(info_.hardware_parameters["gear_ratio"]);
  INTERFACE = info_.hardware_parameters["interface"];
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Gear Ratio: %f", GEAR_RATIO);
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Interface: %s", INTERFACE.c_str());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("MvpHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MvpHardwareInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("MvpHardwareInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MvpHardwareInterface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MvpHardwareInterface"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  // initialize ethercat
  auto ethercat_msg = ethercat_wrapper_.initializeEthercat(INTERFACE.c_str());
  if (!ethercat_msg.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger("MvpHardwareInterface"),
                 ethercat_msg.error());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                ethercat_msg.value());
  }

  clock_ = rclcpp::Clock();

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MvpHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_motor", hardware_interface::HW_IF_POSITION,
      &left_wheel_position_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_motor", hardware_interface::HW_IF_VELOCITY,
      &left_wheel_velocity_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_motor", hardware_interface::HW_IF_POSITION,
      &right_wheel_position_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_motor", hardware_interface::HW_IF_VELOCITY,
      &right_wheel_velocity_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MvpHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "left_motor", hardware_interface::HW_IF_VELOCITY,
      &left_wheel_velocity_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "right_motor", hardware_interface::HW_IF_VELOCITY,
      &right_wheel_velocity_command_));
  return command_interfaces;
}

hardware_interface::return_type MvpHardwareInterface::start() {
  // init variables
  left_wheel_velocity_command_ = 0;
  left_wheel_position_state_ = 0;
  right_wheel_velocity_command_ = 0;
  right_wheel_position_state_ = 0;

  ////////////////// ACTIVATE MOTORS ////////////////
  left_motor_.setSlave(&ec_slave[1], 1);
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "State Machine nach Initialisierung, state=%d\n",
              left_motor_.sdoReadInit(1, ec_SDOread));

  right_motor_.setSlave(&ec_slave[2], 2);
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "State Machine nach Initialisierung, state=%d\n",
              right_motor_.sdoReadInit(2, ec_SDOread));

  auto slave_msg = ethercat_wrapper_.setupSlaves();
  if (!slave_msg.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger("MvpHardwareInterface"), slave_msg.error());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), slave_msg.value());
  }

  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "State Machine nach setupSlave, state=%d",
              left_motor_.sdoReadInit(1, ec_SDOread));

  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "State Machine nach setupSlave, state=%d",
              right_motor_.sdoReadInit(2, ec_SDOread));

  // switch state of state machine 
  // 1.) SWITCH_ON_DISBALED -> Shutdown
  left_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);
  right_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);
  // 2.) READY_TO_SWITCH_ON -> SWITCH_ON
  left_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);
  right_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);
  // 3.) SWITCH_ON -> Operation_Enabled
  left_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);
  right_motor_.changeStatus(ipa_ros2_control::Operation_Enabled);

  status_ = hardware_interface::status::STARTED;

  last_timestamp_ = clock_.now();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::stop() {
  ////////////////// DEACTIVATE MOTORS ////////////////
  ethercat_wrapper_.close();

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::read() {
  ethercat_wrapper_.datacycle_callback();
  statecheck();
  // get encoder data convert to SI units and save them in position and velocity
  // state member variables
  read_left_wheel_raw_ = left_motor_.getRPM();
  read_right_wheel_raw_ = right_motor_.getRPM();

  current_timestamp_ = clock_.now();
  update_rate_ = current_timestamp_.seconds() - last_timestamp_.seconds();

  left_wheel_velocity_state_ =
      read_left_wheel_raw_ * (2 * M_PI / 60) * (1 / GEAR_RATIO);
  right_wheel_velocity_state_ =
      -1 * read_right_wheel_raw_ * (2 * M_PI / 60) * (1 / GEAR_RATIO);

  left_wheel_position_state_ += update_rate_ * left_wheel_velocity_state_;
  right_wheel_position_state_ += update_rate_ * right_wheel_velocity_state_;
  // convert to -pi to pi???

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MvpHardwareInterface::write() {
  write_left_wheel_raw_ =
      left_wheel_velocity_command_ * GEAR_RATIO * (60 / (2 * M_PI));
  write_right_wheel_raw_ =
      -1 * right_wheel_velocity_command_ * GEAR_RATIO * (60 / (2 * M_PI));

  // send the received commands (command member variables) to the motors (in
  // RPM)
  left_motor_.setRPM(write_left_wheel_raw_);
  right_motor_.setRPM(write_right_wheel_raw_);

  ethercat_wrapper_.datacycle_callback();
  statecheck();

  return hardware_interface::return_type::OK;
}

void MvpHardwareInterface::statecheck() {
  auto info_msg = ethercat_wrapper_.statecheck_callback();
  if (info_msg.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger("MvpHardwareInterface"), info_msg.value());
  }
}

} // namespace ipa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipa_ros2_control::MvpHardwareInterface,
                       hardware_interface::SystemInterface)