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
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "candle_ros2/msg/impedance_command.hpp"
#include "candle_ros2/msg/motion_command.hpp"
#include "candle_ros2/msg/position_pid_command.hpp"
#include "candle_ros2/msg/velocity_pid_command.hpp"
#include "candle_ros2/srv/add_md80s.hpp"
#include "candle_ros2/srv/generic_md80_msg.hpp"
#include "candle_ros2/srv/set_limits_md80.hpp"
#include "candle_ros2/srv/set_mode_md80s.hpp"

namespace ipa_ros2_control
{
class CurtMiniHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void jointsCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  rclcpp::Node::SharedPtr nh_;
  // inspired by Jackal github
  void writeCommandsToHardware();
  void updateJointsFromHardware();

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_velocity_;
  sensor_msgs::msg::JointState motor_joint_state_;
  // ROS services
  rclcpp::Client<candle_ros2::srv::AddMd80s>::SharedPtr add_controller_service_client_;
  rclcpp::Client<candle_ros2::srv::SetModeMd80s>::SharedPtr set_mode_service_client_;
  rclcpp::Client<candle_ros2::srv::GenericMd80Msg>::SharedPtr set_zero_service_client_;
  rclcpp::Client<candle_ros2::srv::GenericMd80Msg>::SharedPtr enable_motors_service_client_;
  rclcpp::Client<candle_ros2::srv::GenericMd80Msg>::SharedPtr disable_motors_service_client_;

  // ROS publishers
  rclcpp::Publisher<candle_ros2::msg::MotionCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<candle_ros2::msg::VelocityPidCommand>::SharedPtr config_pub_;

  // ROS subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  // controller params

  // map of joint names and there index
  std::map<std::string, uint8_t> wheel_joints_;
};

}  // namespace ipa_ros2_control

#endif  // CURT_MINI_SYSTEM_HPP