#include "ipa_ros2_control/curt_mini_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ipa_ros2_control {
hardware_interface::return_type CurtMiniHardwareInterface::configure(
    const hardware_interface::HardwareInfo &info) {
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"), "Name: %s",
              info_.name.c_str());

  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Number of Joints %u", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(),
                             std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(),
                             std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("CurtMiniHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("CurtMiniHardwareInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("CurtMiniHardwareInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("CurtMiniHardwareInterface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("CurtMiniHardwareInterface"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
  }

  // init motor controller
  motor_controller_ = new motor_driver::MotorDriver(
      motor_ids_, can_comm_, motor_driver::MotorType::AK80_9_V1p1);

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
CurtMiniHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CurtMiniHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("CurtMiniHardwareInterface"),
                "Wheel joint names and indices are set as follows:\n" << 
                info_.joints[wheel_joints_["front_left_motor"]].name << " at index: " << wheel_joints_["front_left_motor"] << "\n" <<
                info_.joints[wheel_joints_["front_right_motor"]].name << " at index: " << wheel_joints_["front_right_motor"] << "\n" <<
                info_.joints[wheel_joints_["back_left_motor"]].name << " at index: " << wheel_joints_["back_left_motor"] << "\n" <<
                info_.joints[wheel_joints_["back_right_motor"]].name << " at index: " << wheel_joints_["back_right_motor"]);

  }

  return command_interfaces;
}

hardware_interface::return_type CurtMiniHardwareInterface::start() {
  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (std::isnan(hw_states_position_[i])) {
      hw_states_position_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  // set zero position
  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Set zero position...");
  motor_controller_->setZeroPosition(motor_ids_);

  // enable motors
  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Enable motors...");
  motor_controller_->enableMotor(motor_ids_);

  // init command map
  motor_driver::motorCommand no_command = {0.0, 0.0, k_stiffness_, k_damping_,
                                           0.0};
  command_map_ = {{motor_ids_[0], no_command},
                  {motor_ids_[1], no_command},
                  {motor_ids_[2], no_command},
                  {motor_ids_[3], no_command}};

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::stop() {
  // disable motors
  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Disable motors...");
  motor_controller_->disableMotor(motor_ids_);

  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "System Successfully stopped!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::read() {
  updateJointsFromHardware();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::write() {
  writeCommandsToHardware();
  return hardware_interface::return_type::OK;
}

void CurtMiniHardwareInterface::writeCommandsToHardware() {
  // only front wheel commands are used
  // left side has to be multiplied with -1 due to the orientation of the motors
  float diff_speed_left = -1 * hw_commands_[wheel_joints_["front_left_motor"]];
  float diff_speed_right = hw_commands_[wheel_joints_["front_right_motor"]];
  RCLCPP_INFO_STREAM(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Send left side velocity:\t" << diff_speed_left << "\n" <<
              "Send right side velocity:\t" << diff_speed_right);

  motor_driver::motorCommand command_left = {0.0, diff_speed_left, k_stiffness_,
                                             k_damping_, 0.0};
  motor_driver::motorCommand command_right = {0.0, diff_speed_right,
                                              k_stiffness_, k_damping_, 0.0};
  command_map_ = {
      {motor_ids_[wheel_joints_["front_left_motor"]], command_left},
      {motor_ids_[wheel_joints_["front_right_motor"]], command_right},
      {motor_ids_[wheel_joints_["back_left_motor"]], command_left},
      {motor_ids_[wheel_joints_["back_right_motor"]], command_right}};
  auto motor_state = motor_controller_->sendRadCommand(command_map_);
}

void CurtMiniHardwareInterface::updateJointsFromHardware() {
  auto motor_state = motor_controller_->sendRadCommand(command_map_);
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    hw_states_position_[i] = motor_state[i].position;
    if (i % 2 == 0)
    {hw_states_velocity_[i] = motor_state[i].velocity;}
    else // correct velocities for left side
    {hw_states_velocity_[i] = -1 * motor_state[i].velocity;}
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("CurtMiniHardwareInterface"),
              "Read left side velocity:\t" << hw_states_velocity_[wheel_joints_["front_left_motor"]] << "\n" <<
              "Read right side velocity:\t" << hw_states_velocity_[wheel_joints_["front_right_motor"]]);
}

} // namespace ipa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipa_ros2_control::CurtMiniHardwareInterface,
                       hardware_interface::SystemInterface)