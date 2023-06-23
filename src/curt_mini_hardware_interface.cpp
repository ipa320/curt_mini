#include "ipa_ros2_control/curt_mini_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ipa_ros2_control
{
hardware_interface::return_type CurtMiniHardwareInterface::configure(const hardware_interface::HardwareInfo& info)
{
  nh_ = std::make_shared<rclcpp::Node>("CurtMiniHardwareInterface");
  RCLCPP_INFO(nh_->get_logger(), "Configure");
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(nh_->get_logger(), "Name: %s", info_.name.c_str());

  RCLCPP_INFO(nh_->get_logger(), "Number of Joints %u", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(nh_->get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(nh_->get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(nh_->get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(nh_->get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(nh_->get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
    status_ = hardware_interface::status::CONFIGURED;
    // return hardware_interface::return_type::OK;
  }
  RCLCPP_INFO(nh_->get_logger(), "Init ROS services etc");
  add_controller_service_client_ = nh_->create_client<candle_ros2::srv::AddMd80s>("candle_ros2_node/add_md80s");
  set_mode_service_client_ = nh_->create_client<candle_ros2::srv::SetModeMd80s>("candle_ros2_node/set_mode_md80s");
  set_zero_service_client_ = nh_->create_client<candle_ros2::srv::GenericMd80Msg>("candle_ros2_node/zero_md80s");
  enable_motors_service_client_ = nh_->create_client<candle_ros2::srv::GenericMd80Msg>("candle_ros2_node/enable_md80s");
  disable_motors_service_client_ =
      nh_->create_client<candle_ros2::srv::GenericMd80Msg>("candle_ros2_node/disable_md80s");

  joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      "/md80/joint_states", 10, std::bind(&CurtMiniHardwareInterface::jointsCallback, this, std::placeholders::_1));
  command_pub_ = nh_->create_publisher<candle_ros2::msg::MotionCommand>("/md80/motion_command", 10);
  config_pub_ = nh_->create_publisher<candle_ros2::msg::VelocityPidCommand>("/md80/velocity_pid_command", 10);

  // Init Motor:
  // Add Controllers
  // Set Mode of Controllers
  motor_joint_state_ = sensor_msgs::msg::JointState();
  motor_joint_state_.position = { 0.0, 0.0, 0.0, 0.0 };
  motor_joint_state_.velocity = { 0.0, 0.0, 0.0, 0.0 };
  motor_joint_state_.effort = { 0.0, 0.0, 0.0, 0.0 };

  // pid params
  pid_config_.kp = 8.0;
  pid_config_.ki = 1.0;
  pid_config_.kd = 0.0;
  pid_config_.i_windup = 6.0;
  pid_config_.max_output = 18.0;

  auto_declare<double>("pid_config.kp", pid_config_.kp);
  auto_declare<double>("pid_config.ki", pid_config_.ki);
  auto_declare<double>("pid_config.kd", pid_config_.kd);
  auto_declare<double>("pid_config.i_windup", pid_config_.i_windup);
  auto_declare<double>("pid_config.max_output", pid_config_.max_output);
  auto_declare<double>("standstill_thresh", standstill_thresh_);

  param_callback_handle_ =
      nh_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        for (const auto& param : parameters)
        {
          // path to goal distance
          if (param.get_name() == "pid_config.kp")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              pid_config_.kp = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for pid_config.kp";
              return result;
            }
          }
          if (param.get_name() == "pid_config.ki")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              pid_config_.ki = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for pid_config.ki";
              return result;
            }
          }
          if (param.get_name() == "pid_config.kd")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              pid_config_.kd = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for pid_config.kd";
              return result;
            }
          }
          if (param.get_name() == "pid_config.i_windup")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              pid_config_.i_windup = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for pid_config.i_windup";
              return result;
            }
          }
          if (param.get_name() == "pid_config.max_output")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              pid_config_.max_output = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for pid_config.max_output";
              return result;
            }
          }
          if (param.get_name() == "standstill_thresh")
          {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              standstill_thresh_ = param.as_double();
            }
            else
            {
              result.successful = false;
              result.reason = "wrong type for standstill_thresh";
              return result;
            }
          }
        }
        return result;
      });
  RCLCPP_INFO(nh_->get_logger(), "Init finished");

  return hardware_interface::return_type::OK;
}

void CurtMiniHardwareInterface::publishPIDParams(const candle_ros2::msg::Pid& pid_config)
{
  auto pid_msg = candle_ros2::msg::VelocityPidCommand();
  pid_msg.drive_ids = { 101, 100, 103, 102 };
  pid_msg.velocity_pid = { pid_config, pid_config, pid_config, pid_config };
  config_pub_->publish(pid_msg);
}

std::vector<hardware_interface::StateInterface> CurtMiniHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurtMiniHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;

    RCLCPP_INFO_STREAM(nh_->get_logger(), "Wheel joint names and indices are set as follows:\n"
                                              << info_.joints[wheel_joints_["front_left_motor"]].name
                                              << " at index: " << wheel_joints_["front_left_motor"] << "\n"
                                              << info_.joints[wheel_joints_["front_right_motor"]].name
                                              << " at index: " << wheel_joints_["front_right_motor"] << "\n"
                                              << info_.joints[wheel_joints_["back_left_motor"]].name
                                              << " at index: " << wheel_joints_["back_left_motor"] << "\n"
                                              << info_.joints[wheel_joints_["back_right_motor"]].name
                                              << " at index: " << wheel_joints_["back_right_motor"]);
  }

  return command_interfaces;
}

// bool CurtMiniHardwareInterface::sendGenericRequest(rclcpp::Client<candle_ros2::srv::GenericMd80Msg>::SharedPtr&
// client)
// {
//   auto request = std::make_shared<candle_ros2::srv::GenericMd80Msg::Request>();
//   request->drive_ids = { 102, 100, 103, 101 };
//   auto result = client->async_send_request(request);
//   if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
//   {
//     if(!std::all_of(result.get()->drives_success.begin(), result.get()->drives_success.end(), [](bool b){return b;}))
//     {
//       RCLCPP_ERROR_STREAM(nh_->get_logger(), "Service " << client->get_service_name() << " was not successfull for
//       all motors! Exiting"); return false;
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR_STREAM(nh_->get_logger(), "Calling " << client->get_service_name() << " failed! Exiting");
//     return false;
//   }
//   return true;
// }

hardware_interface::return_type CurtMiniHardwareInterface::start()
{
  RCLCPP_INFO(nh_->get_logger(), "Starting ...please wait...");

  // set some default values
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0);

  // wait for service available
  while (!add_controller_service_client_->wait_for_service(std::chrono::seconds(2)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for motor controller node");
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_INFO(nh_->get_logger(), "Waiting for motor controller node.");
  }
  // add controllers via service
  if (!sendCandleRequest<candle_ros2::srv::AddMd80s>(add_controller_service_client_))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Set Mode via service call
  auto set_mode_request = std::make_shared<candle_ros2::srv::SetModeMd80s::Request>();
  set_mode_request->mode = { "VELOCITY_PID", "VELOCITY_PID", "VELOCITY_PID", "VELOCITY_PID" };
  if (!sendCandleRequest<candle_ros2::srv::SetModeMd80s>(set_mode_service_client_, set_mode_request))
  {
    return hardware_interface::return_type::ERROR;
  }

  // set zero position via service call
  if (!sendCandleRequest<candle_ros2::srv::GenericMd80Msg>(set_zero_service_client_))
  {
    return hardware_interface::return_type::ERROR;
  }

  // enable motors via service call
  if (!sendCandleRequest<candle_ros2::srv::GenericMd80Msg>(enable_motors_service_client_))
  {
    return hardware_interface::return_type::ERROR;
  }

  // set pid and config values
  publishPIDParams(pid_config_);

  // publish zero velocity once
  auto zero_vel = candle_ros2::msg::MotionCommand();
  zero_vel.drive_ids = { 101, 100, 103, 102 };
  zero_vel.target_position = { 0.0, 0.0, 0.0, 0.0 };
  zero_vel.target_velocity = { 0.0, 0.0, 0.0, 0.0 };
  zero_vel.target_torque = { 0.0, 0.0, 0.0, 0.0 };
  command_pub_->publish(zero_vel);

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(nh_->get_logger(), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::stop()
{
  // disable motors
  // publish zero once before
  auto zero_vel = candle_ros2::msg::MotionCommand();
  zero_vel.drive_ids = { 101, 100, 103, 102 };
  zero_vel.target_position = { 0.0, 0.0, 0.0, 0.0 };
  zero_vel.target_velocity = { 0.0, 0.0, 0.0, 0.0 };
  zero_vel.target_torque = { 0.0, 0.0, 0.0, 0.0 };
  command_pub_->publish(zero_vel);

  // disable service call
  if (!sendCandleRequest<candle_ros2::srv::GenericMd80Msg>(disable_motors_service_client_))
  {
    return hardware_interface::return_type::ERROR;
  }

  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(nh_->get_logger(), "System Successfully stopped!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::read()
{
  updateJointsFromHardware();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtMiniHardwareInterface::write()
{
  writeCommandsToHardware();
  return hardware_interface::return_type::OK;
}

void CurtMiniHardwareInterface::writeCommandsToHardware()
{
  auto command_vel = candle_ros2::msg::MotionCommand();
  command_vel.drive_ids = { 101, 100, 103, 102 };
  command_vel.target_position = { 0.0, 0.0, 0.0, 0.0 };

  command_vel.target_torque = { 0.0, 0.0, 0.0, 0.0 };

  // turn off torque when standing still
  if (std::all_of(hw_commands_.begin(), hw_commands_.end(), [](double cmd) { return cmd == 0; }) &&
      std::none_of(hw_states_velocity_.begin(), hw_states_velocity_.end(),
                   [this](double vel) { return vel > standstill_thresh_; }))
  {
    if (!motors_paused_)
    {
      auto tmp_pid = pid_config_;
      tmp_pid.max_output = 0.0;
      publishPIDParams(tmp_pid);
      motors_paused_ = true;
    }
    command_vel.target_velocity = { 0.0, 0.0, 0.0, 0.0 };
    command_pub_->publish(command_vel);
  }
  else
  {
    if (motors_paused_)
    {
      publishPIDParams(pid_config_);
      motors_paused_ = false;
    }

    // only front wheel commands are used
    // right side has to be multiplied with -1 due to the orientation of the motors
    float diff_speed_left = hw_commands_[wheel_joints_["front_left_motor"]];
    float diff_speed_right = -1 * hw_commands_[wheel_joints_["front_right_motor"]];
    // RCLCPP_INFO_STREAM(nh_->get_logger(), "Send left side velocity:\t" <<
    // diff_speed_left); RCLCPP_INFO_STREAM(nh_->get_logger(), "Send right side velocity:\t"
    // << diff_speed_right);
    command_vel.target_velocity = { diff_speed_left, diff_speed_right, diff_speed_left, diff_speed_right };
  }

  // publish topic with values

  command_pub_->publish(command_vel);
}

void CurtMiniHardwareInterface::jointsCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  motor_joint_state_ = *msg;
}

void CurtMiniHardwareInterface::updateJointsFromHardware()
{
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    hw_states_position_[i] = motor_joint_state_.position[i];
    if (i % 2 == 0)
    {
      hw_states_velocity_[i] = motor_joint_state_.velocity[i];
    }
    else  // correct velocities for left side
    {
      hw_states_velocity_[i] = -1 * motor_joint_state_.velocity[i];
    }
  }

  // RCLCPP_INFO_STREAM(nh_->get_logger(), "Read left side velocity:\t" <<
  // hw_states_velocity_[wheel_joints_["front_left_motor"]]);
  // RCLCPP_INFO_STREAM(nh_->get_logger(), "Read right side velocity:\t" <<
  // hw_states_velocity_[wheel_joints_["front_right_motor"]]);
}

}  // namespace ipa_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ipa_ros2_control::CurtMiniHardwareInterface, hardware_interface::SystemInterface)
